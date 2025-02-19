import math

import magicbot
import wpilib
import ntcore
import wpilib.event
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from magicbot import tunable

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from components.vision import VisionComponent
from components.battery_monitor import BatteryMonitorComponent
from components.leds import LEDComponent
from components.wrist import WristComponent
from components.arm import ArmComponent
from components.elevator import ElevatorComponent
from components.intake import IntakeComponent
from components.photoeye import PhotoEyeComponent

from components.leds_sim import LEDSim
from components.manipulator_sim import ManipulatorSim

from controllers.manipulator import Manipulator
from controllers.intimidator import Intimidator
from controllers.intake import IntakeControl

from utilities.scalers import rescale_js
from utilities.position import Positions
from utilities.game import GamePieces
from utilities import Waypoints, is_red

from hid.xbox_wired import ReefscapeDriver, ReefscapeOperator
from hid.logi_flight import ReefscapeDriver as ReefscapeDriverFlight
from hid.logi_gamepad import ReefscapeDriver as ReefscapeDriverLogiGamepad
from hid.xbox_wireless import ReefscapeDriver as ReefscapeDriverWireless
from hid.thrustmaster import ReefscapeDriver as ReefscapeDriverThrustmaster


class MyRobot(magicbot.MagicRobot):
    # Controllers
    manipulator: Manipulator
    intimidator: Intimidator
    manipulator_sim: ManipulatorSim
    leds_sim: LEDSim

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
    intake_control: IntakeControl

    max_speed = magicbot.tunable(32)  # m/s
    lower_max_speed = magicbot.tunable(3)  # m/s
    max_spin_rate = magicbot.tunable(12)  # m/s
    lower_max_spin_rate = magicbot.tunable(4)  # m/s
    inclination_angle = tunable(0.0)
    controller_choice = tunable('')
    controller_name = tunable('')

    # This is a debugging/test thing, not production code
    driver_reef_snap_distance = tunable(-1.0)
    driver_reef_radians_snap = tunable(0.0)

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

        self.reef_center_pose_pub = (ntcore.NetworkTableInstance.getDefault()
                                                .getStructTopic("ReefCenterPose", Pose2d)
                                                .publish()
        )
        self.strafe_positions = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructArrayTopic("/components/drivetrain/strafes", Pose2d)
            .publish()
        )

        self.strafe_next = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/drivetrain/strafe_next", Pose2d)
            .publish()
        )

    def autonomousInit(self):
        return

    def autonomousPeriodic(self):
        pass

    def teleopInit(self) -> None:
        if self.isSimulation():
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        self.leds.rainbow()
        
        # Determine which Joystick to use for the driver.
        js_name = wpilib.DriverStation.getJoystickName(0)
        print('Joystick name:', js_name)
        self.controller_name = js_name
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
        elif js_name.startswith('Logitech Gamepad'):
            self.controller_choice = 'Logitech Gamepad'
            self.driver_controller = ReefscapeDriverLogiGamepad(0)
        elif js_name.startswith('Thrustmaster') or js_name.startswith('T.Flight Hotas'):
            self.controller_choice = 'Talk to me, Goose!'
            self.driver_controller = ReefscapeDriverThrustmaster(0)
        # self.drivetrain.set_pose(Positions.auton_line_2(is_red()))
        self.photoeye.coral_held = False
        tag = Waypoints.get_tag_id_from_letter('C', True)
        pose = Waypoints.get_tag_robot_away(tag, face_at=True)
        pose = Waypoints.shift_reef_right(pose)
        self.drivetrain.set_pose(pose)
        self.manipulator.engage()
        self.intimidator.engage()

    def handle_manipulator(self) -> None:
        from controllers.manipulator import ManipLocations
        if self.driver_controller.getManipulatorAdvance():
            self.manipulator.request_advance()
        if self.driver_controller.getCoralMode():
            self.manipulator.coral_mode()
        if self.driver_controller.getAlgaeMode():
            self.manipulator.algae_mode()
        if self.driver_controller.goHome():
            self.manipulator.go_home()

        # Let's set some heights with the driver controller
        if self.manipulator.game_piece_mode == GamePieces.CORAL:
            if self.driver_controller.getHeightPlacement1():
                self.manipulator.set_coral_level1()
            if self.driver_controller.getHeightPlacement2():
                self.manipulator.set_coral_level2()
            if self.driver_controller.getHeightPlacement3():
                self.manipulator.set_coral_level3()
            if self.driver_controller.getHeightPlacement4():
                self.manipulator.set_coral_level4()
        elif self.manipulator.game_piece_mode == GamePieces.ALGAE:
            if self.driver_controller.getHeightPlacement1():
                self.manipulator.set_algae_level1()
            if self.driver_controller.getHeightPlacement2():
                self.manipulator.set_algae_level2()

        # Now let's do the operator controller, which is how the real robot
        # will likely work
        if self.operator_controller.goHome():
            self.manipulator.go_home()
        if self.operator_controller.getManipulatorAdvance():
            self.manipulator.request_advance()

        # Some buttons to force the manipulator to certain heights. Not to be
        # used in the actual driving of the robot, but handy for debugging
        dpad = self.operator_controller.getPOV()
        if self.operator_controller.getRightBumper():
            match dpad:
                case 0:  # Up arrow, top coral 
                    self.manipulator.request_location(ManipLocations.CORAL_REEF_4)
                case 90:  # Right arrow, 2nd coral
                    self.manipulator.request_location(ManipLocations.CORAL_REEF_2)
                case 270:  # Left arrow, 3rd coral
                    self.manipulator.request_location(ManipLocations.CORAL_REEF_3)
                case 180:  # Down arrow, trough level 1
                    self.manipulator.request_location(ManipLocations.CORAL_REEF_1)
        else:
            match dpad:
                case 0:  # Up arrow, top coral 
                    self.manipulator.set_coral_level4()
                case 90:  # Right arrow, 2nd coral
                    self.manipulator.set_coral_level2()
                case 270:  # Left arrow, 3rd coral
                    self.manipulator.set_coral_level3()
                case 180:  # Down arrow, trough level 1
                    self.manipulator.set_coral_level1()

    def handle_drivetrain(self) -> None:
        max_speed = self.max_speed
        max_spin_rate = self.max_spin_rate

        drive_x = -rescale_js(self.driver_controller.getLeftY(), 0.05, 2.5) * max_speed
        drive_y = -rescale_js(self.driver_controller.getLeftX(), 0.05, 2.5) * max_speed
        drive_z = (
            -rescale_js(self.driver_controller.getRightX(), 0.1, exponential=2) * max_spin_rate
        )
        self.intimidator.set_stick_values(drive_x, drive_y, drive_z)

        if self.driver_controller.getFieldReset():  # the window button
            self.gyro.reset_heading()

        if self.driver_controller.getReefAlgae():
            self.intimidator.go_lock_reef()
        elif self.driver_controller.getReefLeft():
            self.intimidator.go_lock_reef(shift_left=True)
        elif self.driver_controller.getReefRight():
            self.intimidator.go_lock_reef(shift_right=True)
        elif self.driver_controller.getToWallTarget():
            if self.photoeye.algae_held:
                self.intimidator.go_drive_processor()
                self.manipulator.set_algae_processor()
            else:
                dpad = self.driver_controller.getPOV()
                if dpad == -1:
                    self.intimidator.go_drive_nearest_ps()
                elif dpad == 90:
                    self.intimidator.go_drive_ps(2 if is_red() else 12)
                elif dpad == 270:
                    self.intimidator.go_drive_ps(1 if is_red() else 11)
                
        elif self.driver_controller.returnToHomeLocation():
            self.drivetrain.drive_to_pose(
                Positions.auton_line_2(is_red())
            )
        elif self.driver_controller.getDriveLocal():
            max_speed = self.lower_max_speed
            max_spin_rate = self.lower_max_spin_rate
            self.intimidator.go_drive_local()
        elif self.driver_controller.getStrafe():
            self.intimidator.go_drive_strafe()
        else:
            self.intimidator.go_drive_field()

    def teleopPeriodic(self) -> None:
        self.handle_manipulator()
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
