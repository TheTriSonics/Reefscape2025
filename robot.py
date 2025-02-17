import math
import choreo

import magicbot
import wpilib
import ntcore
import wpilib.event
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Transform2d, Translation2d
from magicbot import tunable
from wpimath.geometry import Rotation3d, Translation3d

from components import (
    DrivetrainComponent,
    GyroComponent,
    VisionComponent,
    BatteryMonitorComponent,
    LEDComponent,
    WristComponent,
    ArmComponent,
    ElevatorComponent,
    IntakeComponent,
    PhotoEyeComponent,
)

from components.manipulator_sim import ManipulatorSim

from controllers.manipulator import Manipulator

from utilities.scalers import rescale_js
from utilities.game import is_red
from utilities.position import Positions
from utilities.waypoints import *  # JJB: Bad form, but we're going to refactor this later
from robotpy_ext.autonomous import AutonomousModeSelector

from hid.xbox_wired import ReefscapeDriver, ReefscapeOperator
from hid.logi_flight import ReefscapeDriver as ReefscapeDriverFlight
from hid.xbox_wireless import ReefscapeDriver as ReefscapeDriverWireless
from hid.thrustmaster import ReefscapeDriver as ReefscapeDriverThrustmaster


class MyRobot(magicbot.MagicRobot):
    # Controllers
    manipulator: Manipulator
    manipulator_sim: ManipulatorSim

    # Components
    gyro: GyroComponent
    photoeye: PhotoEyeComponent
    drivetrain: DrivetrainComponent
    vision: VisionComponent
    battery_monitor: BatteryMonitorComponent
    leds: LEDComponent

    # These 3 should not be used directly except in testing!
    # Only use the controller/state machine when doing real things!
    wrist: WristComponent
    arm: ArmComponent
    elevator: ElevatorComponent
    intake: IntakeComponent


    max_speed = magicbot.tunable(32)  # m/s
    lower_max_speed = magicbot.tunable(3)  # m/s
    max_spin_rate = magicbot.tunable(12)  # m/s
    lower_max_spin_rate = magicbot.tunable(4)  # m/s
    inclination_angle = tunable(0.0)
    controller_choice = tunable('')

    START_POS_TOLERANCE = 1

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        # side: (28*3)*2 + front: (30*3) - 2 (R.I.P)
        self.timer = wpilib.Timer()

        self.final_pose_pub = (ntcore.NetworkTableInstance.getDefault()
                                                .getStructTopic("LockOnPose", Pose2d)
                                                .publish()
        )

    def autonomousInit(self):
        return

    def autonomousPeriodic(self):
        pass

    def teleopInit(self) -> None:
        if self.isSimulation():
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)
        
        # Determine which Joystick to use for the driver.
        js_name = wpilib.DriverStation.getJoystickName(0)
        print('Joystick name:', js_name)
        # We'll default to a stock Xbox controller unless the name tells us
        # another one is better
        self.controller_choice = 'Stock Xbox controller'
        self.driver_controller = ReefscapeDriver(0)
        self.operator_controller = ReefscapeOperator(1)
        if js_name == 'Xbox Wireless Controller':
            self.controller_choice = 'Using Xbox wireless controller config'
            self.driver_controller = ReefscapeDriverWireless(0)
        elif js_name == 'Logitech Logitech Extreme 3D':
            self.controller_choice = 'Going with a flight stick, eh, Mav?'
            self.driver_controller = ReefscapeDriverFlight(0)
        elif js_name.startswith('Thrustmaster'):
            self.controller_choice = 'Talk to me, Goose!'
            self.driver_controller = ReefscapeDriverThrustmaster(0)
        # self.drivetrain.set_pose(Positions.auton_line_2(is_red()))
        tag = get_tag_id_from_letter('A', True)
        pose = get_tag_robot_away(tag, face_at=True)
        pose = shift_reef_right(pose)
        self.drivetrain.set_pose(pose)
        self.manipulator.engage()

    def handle_manipulator(self) -> None:
        from controllers.manipulator import Locations
        # Let's set some heights with the driver controller
        if self.driver_controller.getHeightPlacement1():
            self.manipulator.set_coral_level1()
        if self.driver_controller.getHeightPlacement2():
            self.manipulator.set_coral_level2()
        if self.driver_controller.getHeightPlacement3():
            self.manipulator.set_coral_level3()
        if self.driver_controller.getHeightPlacement4():
            self.manipulator.set_coral_level4()
        if self.driver_controller.getManipulatorAdvance():
            self.manipulator.request_advance()

        # Now let's do the operator controller, which is how the real robot
        # will likely work
        if self.operator_controller.goHome():
            self.manipulator.go_home()
        if self.operator_controller.getManipulatorAdvance():
            self.manipulator.request_advance()

        # Some buttons to force the manipulator to certain heights. Not to be
        # used in the actual driving of the robot, but handy for debugging
        if self.operator_controller.getRightBumper():
            dpad = self.operator_controller.getPOV()
            match dpad:
                case 0:  # Up arrow, top coral 
                    self.manipulator.request_location(Locations.CORAL_REEF_4)
                case 90:  # Right arrow, 2nd coral
                    self.manipulator.request_location(Locations.CORAL_REEF_2)
                case 270:  # Left arrow, 3rd coral
                    self.manipulator.request_location(Locations.CORAL_REEF_3)
                case 180:  # Down arrow, trough level 1
                    self.manipulator.request_location(Locations.CORAL_REEF_1)

    def handle_drivetrain(self) -> None:
        max_speed = self.max_speed
        max_spin_rate = self.max_spin_rate

        if self.driver_controller.getFieldReset():  # the window button
            self.gyro.reset_heading()
        drive_x = -rescale_js(self.driver_controller.getLeftY(), 0.05, 2.5) * max_speed
        drive_y = -rescale_js(self.driver_controller.getLeftX(), 0.05, 2.5) * max_speed
        drive_z = (
            -rescale_js(self.driver_controller.getRightX(), 0.1, exponential=2) * max_spin_rate
        )

        if self.driver_controller.getDriveLocal():
            max_speed = self.lower_max_speed
            max_spin_rate = self.lower_max_spin_rate
            self.drivetrain.drive_local(drive_x, drive_y, drive_z)
        else:
            if is_red():
                drive_x = -drive_x
                drive_y = -drive_y
            self.drivetrain.drive_field(drive_x, drive_y, drive_z)
        # Give rotational access to the driver
        if drive_z != 0:
            self.drivetrain.stop_snapping()

    def lock_reef(self, shift_left=False, shift_right=False):
        from utilities.waypoints import (
            closest_reef_tag_id, get_tag_robot_away,
            shift_reef_left, shift_reef_right
        )
        pose = self.drivetrain.get_pose()
        reef_tag_id, dist = closest_reef_tag_id(pose)
        final_pose = (
            get_tag_robot_away(reef_tag_id)
            .transformBy(Transform2d(Translation2d(0, 0), Rotation2d(math.pi)))
        )
        if shift_left:
            final_pose = shift_reef_left(final_pose)
        elif shift_right:
            final_pose = shift_reef_right(final_pose)
        self.final_pose_pub.set(final_pose)
        self.drivetrain.drive_to_pose(final_pose)

    def lock_ps(self):
        from utilities.waypoints import (
            closest_ps_tag_id, get_tag_robot_away,
        )
        pose = self.drivetrain.get_pose()
        tag_id, dist = closest_ps_tag_id(pose)
        final_pose = (
            get_tag_robot_away(tag_id)
        )
        self.final_pose_pub.set(final_pose)
        self.drivetrain.drive_to_pose(final_pose)

    def lock_processor(self):
        from utilities.waypoints import (
            closest_processor_tag_id, get_tag_robot_away,
        )
        pose = self.drivetrain.get_pose()
        tag_id, dist = closest_processor_tag_id(pose)
        final_pose = (
            get_tag_robot_away(tag_id)
        )
        self.final_pose_pub.set(final_pose)
        self.drivetrain.drive_to_pose(final_pose)

    def teleopPeriodic(self) -> None:
        self.handle_manipulator()

        if self.driver_controller.getReefAlgae():
            self.lock_reef()
        elif self.driver_controller.getReefLeft():
            self.lock_reef(shift_left=True)
        elif self.driver_controller.getReefRight():
            self.lock_reef(shift_right=True)
        elif self.driver_controller.getToWallTarget():
            if self.photoeye.algae_held:
                self.lock_processor()
            else:
                self.lock_ps()
        elif self.driver_controller.goHome():
            self.drivetrain.drive_to_pose(
                Positions.auton_line_2(is_red())
            )
        else:
            self.handle_drivetrain()

    def testInit(self) -> None:
        self.driver_controller = ReefscapeDriver(0)
        pass

    def testPeriodic(self) -> None:
        dpad = self.driver_controller.getPOV()
        wpilib.SmartDashboard.putNumber('DPAD', dpad)
        if dpad != -1:
            if is_red():
                self.drivetrain.snap_to_heading(-math.radians(dpad) + math.pi)
            else:
                self.drivetrain.snap_to_heading(-math.radians(dpad))
        else:
            self.drivetrain.stop_snapping()
            self.drivetrain.drive_local(0, 0, 0)

        self.drivetrain.execute()

        self.drivetrain.update_odometry()

    def disabledPeriodic(self) -> None:
        self.vision.execute()
        self.battery_monitor.execute()
        self.leds.execute()
        self.drivetrain.update_odometry()
        # mode = self._automodes.active_mode
        mode = self._automodes.chooser.getSelected()
        if mode and hasattr(mode, 'set_initial_pose'):
            mode.set_initial_pose()
