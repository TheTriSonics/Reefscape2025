import math
import choreo

import magicbot
import wpilib
import ntcore
import wpilib.event
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
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
)

from controllers.manipulator import Manipulator

from utilities.scalers import rescale_js
from utilities.game import is_red
from robotpy_ext.autonomous import AutonomousModeSelector


class MyRobot(magicbot.MagicRobot):
    # Controllers
    # manipulator: Manipulator

    # Components
    gyro: GyroComponent
    drivetrain: DrivetrainComponent
    vision: VisionComponent
    battery_monitor: BatteryMonitorComponent
    leds: LEDComponent

    # These 3 should not be used directly except in testing!
    # Only use the controller/state machine when doing real things!
    # wrist: WristComponent
    # arm: ArmComponent
    # elevator: ElevatorComponent
    # intake: IntakeComponent

    max_speed = magicbot.tunable(32)  # m/s
    lower_max_speed = magicbot.tunable(6)  # m/s
    max_spin_rate = magicbot.tunable(32)  # m/s
    lower_max_spin_rate = magicbot.tunable(8)  # m/s
    inclination_angle = tunable(0.0)

    START_POS_TOLERANCE = 1

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()

        self.driver_controller = wpilib.XboxController(0)

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

        self.field.getObject("Intended start pos").setPoses([])

    def handle_manipulator(self) -> None:
        return
        if self.driver_controller.getAButtonPressed():
            self.wrist.target_pos += 1
        pass

    def handle_drivetrain(self) -> None:
        max_speed = self.max_speed
        max_spin_rate = self.max_spin_rate
        if self.driver_controller.getRawButton(8):  # the hamburger menu button
            max_speed = self.lower_max_speed
            max_spin_rate = self.lower_max_spin_rate

        if self.driver_controller.getRawButton(7):  # the window button
            self.gyro.reset_heading()
        drive_x = -rescale_js(self.driver_controller.getLeftY(), 0.05, 2.5) * max_speed
        drive_y = -rescale_js(self.driver_controller.getLeftX(), 0.05, 2.5) * max_speed
        drive_z = (
            -rescale_js(self.driver_controller.getRightX(), 0.1, exponential=2) * max_spin_rate
        )
        local_driving = self.driver_controller.getRawButton(8)

        if local_driving:
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

    def teleopPeriodic(self) -> None:
        if self.driver_controller.getRightBumper() and self.driver_controller.getLeftBumper():
            self.lock_reef()
        elif self.driver_controller.getLeftBumper():
            self.lock_reef(shift_left=True)
        elif self.driver_controller.getRightBumper():
            self.lock_reef(shift_right=True)
        else:
            self.handle_drivetrain()
        self.handle_manipulator()

    def testInit(self) -> None:
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
