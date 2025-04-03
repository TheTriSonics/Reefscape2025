import math
import magicbot
import wpilib
import ntcore
import wpilib.event
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from magicbot import tunable

from components.gyro import GyroComponent
from components.vision import VisionComponent
from components.battery_monitor import BatteryMonitorComponent
# from components.laeds import LEDComponent
from components.wrist import WristComponent
from components.arm import ArmComponent
from components.elevator import ElevatorComponent
from components.intake import IntakeComponent, IntakeDirection
from components.climber import ClimberComponent
from components.photoeye import PhotoEyeComponent
from components.drivetrain import DrivetrainComponent
from components.debug_panel import DebugPanel

from components.position_manager import PositionManagerComponent
# from components.leds_sim import LEDSim
from components.manipulator_sim import ManipulatorSim

from controllers.manipulator import Manipulator
from controllers.intimidator import Intimidator
from controllers.intake import IntakeControl
from controllers.spiderman import Spiderman

from utilities.scalers import rescale_js
from utilities.position import Positions
from utilities.game import GamePieces
from utilities import Waypoints, is_red, pn, is_sim

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
    # leds_sim: LEDSim

    # Components
    # position_manager: PositionManagerComponent
    gyro: GyroComponent
    photoeye: PhotoEyeComponent
    drivetrain: DrivetrainComponent
    debug_panel: DebugPanel
    vision: VisionComponent
    battery_monitor: BatteryMonitorComponent
    # leds: LEDComponent

    # These 3 should not be used directly except in testing!
    # Only use the controller/state machine when doing real things!
    wrist: WristComponent
    arm: ArmComponent
    elevator: ElevatorComponent
    intake: IntakeComponent
    intake_control: IntakeControl
    climber: ClimberComponent
    spiderman: Spiderman

    match_time = tunable(0.0)
    max_speed = magicbot.tunable(25.0)  # m/s
    lower_max_speed = magicbot.tunable(6)  # m/s
    max_spin_rate = magicbot.tunable(4 * math.tau)
    lower_max_spin_rate = magicbot.tunable(math.pi)  # m/s
    controller_choice = tunable('')
    controller_name = tunable('')

    # This is a debugging/test thing, not production code
    driver_reef_snap_distance = tunable(-1.0)
    driver_reef_radians_snap = tunable(0.0)

    START_POS_TOLERANCE = 1
    
    autonomous_has_run = False

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()
        wpilib.DriverStation.startDataLog(self.data_log, logJoysticks=True)

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
        self.manipulator.engage()
        self.intimidator.engage()
        self.autonomous_has_run = True
        return

    # This does not run at all.
    def autonomousPeriodic(self):
        pass

    def teleopInit(self) -> None:
        self.climber.lock_intake()
        if self.isSimulation():
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        # self.leds.rainbow()
        self.manipulator.go_hold()
        self.spiderman.engage()
        self.intimidator.engage()
        self.manipulator.engage()
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

    def handle_manipulator(self) -> None:
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
            elif self.operator_controller.getHeightPlacement2():
                self.manipulator.set_coral_level2()
            elif self.operator_controller.getHeightPlacement3():
                self.manipulator.set_coral_level3()
            elif self.operator_controller.getHeightPlacement4():
                self.manipulator.set_coral_level4()
        elif self.manipulator.game_piece_mode == GamePieces.ALGAE:
            if self.operator_controller.getHeightPlacement2():
                self.manipulator.set_algae_level1()
            elif self.operator_controller.getHeightPlacement3():
                self.manipulator.set_algae_level2()
            elif self.operator_controller.getHeightPlacement1():
                self.manipulator.set_algae_processor()
            elif self.operator_controller.getHeightPlacement4():
                self.manipulator.set_algae_barge()
            
        # Hack in the right and left bumpers moving the elevator up and down
        if not is_sim():
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
            ) * 5
            wrist_movement = -rescale_js(
                self.operator_controller.getRightY(), 0.05, 2.5
            ) * 5
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

        # Enable the climber controls only in the last 20 seconds
        self.match_time = wpilib.DriverStation.getMatchTime()
        #if self.match_time <= 0.0:
        if self.operator_controller.getBButton() and self.operator_controller.getXButton():
            # Engage the climber controller
            self.spiderman.go_break_intake()

        self.spiderman.tweak_up = False
        self.spiderman.tweak_down = False
        if self.operator_controller.getBButton():
            self.spiderman.tweak_up = True
        elif self.operator_controller.getXButton():
            self.spiderman.tweak_down = True

        # Climber overrides
        # if self.operator_controller.getRawButton(3):
        #     self.climber.force_climber_up = True
        # else:
        #     self.climber.force_climber_up = False
        # if self.operator_controller.getRawButton(2):
        #     self.climber.force_climber_down = True
        # else:
        #     self.climber.force_climber_down = False

    def handle_drivetrain(self) -> None:
        max_speed = self.max_speed
        max_spin_rate = self.max_spin_rate

        self.drivetrain.max_wheel_speed = max_speed
        rtrig = self.driver_controller.getRightTriggerAxis()
        ltrig = self.driver_controller.getLeftTriggerAxis()

        dpad = self.driver_controller.getPOV()
        drive_x = -rescale_js(self.driver_controller.getLeftY(), 0.05, 2.5) * max_speed
        drive_y = -rescale_js(self.driver_controller.getLeftX(), 0.05, 2.5) * max_speed
        drive_z = (
            -rescale_js(self.driver_controller.getRightX(), 0.1, exponential=2) * max_spin_rate
        )

        if self.driver_controller.getFieldReset():  # the window button
            self.gyro.reset_heading()

        if self.driver_controller.getReefAlgae():
            self.intimidator.go_lock_reef()
        elif self.driver_controller.getReefLeft():
            if self.manipulator.game_piece_mode == GamePieces.CORAL:
                self.intimidator.go_lock_reef(shift_left=True)
        elif self.driver_controller.getReefRight():
            if self.manipulator.game_piece_mode == GamePieces.CORAL:
                self.intimidator.go_lock_reef(shift_right=True)
        elif self.driver_controller.getToWallTarget():
            if self.manipulator.game_piece_mode == GamePieces.ALGAE:
                self.intimidator.go_drive_swoop(Positions.PROCESSOR)
            else:
                if dpad == 90:
                    self.intimidator.go_drive_swoop(Positions.PS_RIGHT)
                elif dpad == 270:
                    self.intimidator.go_drive_swoop(Positions.PS_LEFT)
                else:
                    self.intimidator.go_drive_swoop(Positions.PS_CLOSEST)
        elif self.driver_controller.returnToHomeLocation():
            self.drivetrain.drive_to_pose(
                Positions.AUTON_LINE_OUR_CAGE_CENTER
            )
        elif self.driver_controller.getDriveLocal():
            max_speed = self.lower_max_speed
            max_spin_rate = self.lower_max_spin_rate
            self.intimidator.set_stick_values(drive_x, drive_y, drive_z)
            self.intimidator.go_drive_local()
        else:
            self.intimidator.set_stick_values(drive_x, drive_y, drive_z)
            self.intimidator.go_drive_field()

    def teleopPeriodic(self) -> None:
        pose = self.drivetrain.get_pose()
        Positions.update_dynamic_positions(pose)
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

    def disabledInit(self):
        Positions.update_alliance_positions()
        mode = self._automodes.chooser.getSelected()
        if mode and hasattr(mode, 'pose_set'):
            mode.pose_set = False
        return super().disabledInit()

    def disabledPeriodic(self) -> None:
        self.vision.execute()
        self.battery_monitor.execute()
        # self.leds.execute()
        self.photoeye.execute()
        self.drivetrain.update_odometry()
        # mode = self._automodes.active_mode
        if Positions.PROCESSOR.X() == 0:
            return  # Skip trying to set pose, we don't have position data yet.
        Intimidator.load_trajectories()
        # We do NOT want to do this between auton and teleop, only before
        # auton.
        if not self.autonomous_has_run:
            mode = self._automodes.chooser.getSelected()
            if mode and hasattr(mode, 'set_initial_pose'):
                mode.set_initial_pose()
