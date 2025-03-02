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
from components.intake import IntakeComponent, IntakeDirection
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
from hid.reefscape_driver_base import ReefscapeDriverBase


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

    max_speed = magicbot.tunable(25.0)  # m/s
    lower_max_speed = magicbot.tunable(6)  # m/s
    max_spin_rate = magicbot.tunable(2 * math.tau)
    lower_max_spin_rate = magicbot.tunable(math.pi)  # m/s
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
        Positions.update_alliance_positions()
        return

    def autonomousPeriodic(self):
        pose = self.drivetrain.get_pose()
        Positions.update_dynamic_positions(pose)
        pass

    def teleopInit(self) -> None:
        if self.isSimulation():
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        self.leds.rainbow()
        Positions.update_alliance_positions()
        
        # Determine which Joystick to use for the driver.
        js_name = wpilib.DriverStation.getJoystickName(0)
        print('Joystick name:', js_name)
        self.controller_name = js_name
        # We'll default to a stock Xbox controller unless the name tells us
        # another one is better
        self.controller_choice = 'Stock Xbox controller'
        self.driver_controller: ReefscapeDriverBase = ReefscapeDriver(0)
        self.operator_controller = ReefscapeOperator(1)
        self.debug_controller = wpilib.XboxController(2)
        if js_name == 'Xbox Wireless Controller':
            self.controller_choice = 'Using Xbox wireless controller config'
            self.driver_controller = ReefscapeDriverWireless(0)
        elif js_name == 'Logitech Logitech Extreme 3D':
            self.controller_choice = 'Going with a flight stick, eh, Mav?'
            self.driver_controller = ReefscapeDriverFlight(0)
        elif js_name.startswith('Logitech Gamepad') or js_name.startswith('Generic'):
            self.controller_choice = 'Logitech Gamepad'
            self.driver_controller = ReefscapeDriverLogiGamepad(0)
        elif js_name.startswith('Thrustmaster') or js_name.startswith('T.Flight Hotas'):
            self.controller_choice = 'Talk to me, Goose!'
            self.driver_controller = ReefscapeDriverThrustmaster(0)
        # self.drivetrain.set_pose(Positions.auton_line_2(is_red()))
        """
        Removed these due to using the hardware values from the CANdi
        self.photoeye.back_photoeye = False
        self.photoeye.front_photoeye = True
        """
        tag = Waypoints.get_tag_id_from_letter('C', True)
        pose = Waypoints.get_tag_robot_away(tag, face_at=True)
        pose = Waypoints.shift_reef_right(pose)
        self.drivetrain.set_pose(pose)
        self.manipulator.engage()
        self.intimidator.engage()

    def handle_manipulator(self) -> None:
        from controllers.manipulator import ManipLocations
        if self.operator_controller.getManipulatorAdvance():
            self.manipulator.request_advance()
        if self.operator_controller.getCoralMode():
            self.manipulator.coral_mode()
        if self.operator_controller.getAlgaeMode():
            self.manipulator.algae_mode()
        if self.operator_controller.goHome():
            self.manipulator.go_home()

        # Let's set some heights with the operator controller
        if self.manipulator.game_piece_mode == GamePieces.CORAL:
            if self.operator_controller.getHeightPlacement1():
                self.manipulator.set_coral_level1()
            if self.operator_controller.getHeightPlacement2():
                self.manipulator.set_coral_level2()
            if self.operator_controller.getHeightPlacement3():
                self.manipulator.set_coral_level3()
            if self.operator_controller.getHeightPlacement4():
                self.manipulator.set_coral_level4()
        elif self.manipulator.game_piece_mode == GamePieces.ALGAE:
            if self.operator_controller.getHeightPlacement2():
                self.manipulator.set_algae_level1()
            if self.operator_controller.getHeightPlacement3():
                self.manipulator.set_algae_level2()
            if self.operator_controller.getHeightPlacement1():
                self.manipulator.set_algae_processor
            if self.operator_controller.getHeightPlacement4():
                self.manipulator.set_algae_barge


        """ Moved the above to the operator controller, this part is no longer needed

        # Now let's do the operator controller, which is how the real robot
        # will likely work
        if self.operator_controller.goHome():
            self.manipulator.go_home()
        if self.operator_controller.getManipulatorAdvance():
            self.manipulator.request_advance()
        """
            
        # Hack in the right and left bumpers moving the elevator up and down
        rtrig = self.operator_controller.getRightTriggerAxis()
        if rtrig > 0.25:
            self.elevator.target_pos += rtrig
        ltrig = self.operator_controller.getLeftTriggerAxis()
        if ltrig > 0.25:
            self.elevator.target_pos -= ltrig
        
        # TODO: Implement deadbanding if not the whole resize_js() method that
        # we use on the driver's stick inputs.
        arm_movement = -rescale_js(
            self.operator_controller.getLeftY(), 0.05, 2.5
        )
        wrist_movement = -rescale_js(
            self.operator_controller.getRightY(), 0.05, 2.5
        )
        self.arm.target_pos += arm_movement
        self.wrist.target_pos += wrist_movement

        # Intake overrides
        if self.operator_controller.getRawButton(7):
            self.intake.force_coral_intake = True
        else:
            self.intake.force_coral_intake = False
        if self.operator_controller.getRawButton(8):
            self.intake.force_coral_score = True
        else:
            self.intake.force_coral_score = False


        """ This part commented out to start using real robot

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
        """

    def handle_drivetrain(self) -> None:
        max_speed = self.max_speed
        max_spin_rate = self.max_spin_rate

        self.drivetrain.max_wheel_speed = max_speed
        rtrig = self.driver_controller.getRightTriggerAxis()
        ltrig = self.driver_controller.getLeftTriggerAxis()
        pn = wpilib.SmartDashboard.putNumber
        pn('rtrig', rtrig)
        pn('ltrig', ltrig)

        dpad = self.driver_controller.getPOV()
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
            if dpad == 90:
                self.intimidator.go_drive_swoop(Positions.PS_RIGHT)
            elif dpad == 270:
                self.intimidator.go_drive_swoop(Positions.PS_LEFT)
            else:
                self.intimidator.go_drive_swoop(Positions.PS_CLOSEST)
        elif self.driver_controller.returnToHomeLocation():
            self.drivetrain.drive_to_pose(
                Positions.AUTON_LINE_CENTER
            )
        elif self.driver_controller.getDriveLocal():
            max_speed = self.lower_max_speed
            max_spin_rate = self.lower_max_spin_rate
            self.intimidator.go_drive_local()
        elif self.driver_controller.getStrafe():
            self.intimidator.go_drive_strafe()
        elif rtrig > 0.15:
            # Scale this between 0-1 instead of -1 to 1
            rscaled = rtrig
            self.intimidator.set_stick_values(0, 0, rscaled*10)
            self.intimidator.go_drive_strafe()
        elif ltrig > 0.15:
            lscaled = ltrig
            self.intimidator.set_stick_values(0, 0, -lscaled*10)
            self.intimidator.go_drive_strafe()
        else:
            self.intimidator.set_stick_values(drive_x, drive_y, drive_z)
            self.intimidator.go_drive_field()

    def teleopPeriodic(self) -> None:
        self.handle_manipulator()
        self.handle_drivetrain()
        pose = self.drivetrain.get_pose()
        Positions.update_dynamic_positions(pose)

    def robotPeriodic(self):
        self.manipulator.check_limits()
        return super().robotPeriodic()

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

    def disabledInit(self):
        mode = self._automodes.chooser.getSelected()
        if mode and hasattr(mode, 'pose_set'):
            mode.pose_set = False
        return super().disabledInit()

    def disabledPeriodic(self) -> None:
        Positions.update_alliance_positions()
        self.vision.execute()
        self.battery_monitor.execute()
        self.leds.execute()
        self.drivetrain.update_odometry()
        # mode = self._automodes.active_mode
        if Positions.PROCESSOR.X() == 0:
            return  # Skip trying to set pose, we don't have position data yet.
        mode = self._automodes.chooser.getSelected()
        if mode and hasattr(mode, 'set_initial_pose'):
            mode.set_initial_pose()
