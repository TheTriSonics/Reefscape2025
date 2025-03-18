import math
from logging import Logger

import magicbot
import ntcore
import wpilib
from magicbot import feedback
from phoenix6.configs import (
    CANcoderConfiguration,
    ClosedLoopGeneralConfigs,
    FeedbackConfigs,
    MagnetSensorConfigs,
    MotorOutputConfigs,
)
from phoenix6.controls import DutyCycleOut, VelocityVoltage, VoltageOut
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
    SimpleMotorFeedforwardMeters,
)
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpimath.trajectory import TrapezoidProfileRadians

from components.elevator import ElevatorComponent
from components.gyro import GyroComponent
from generated.tuner_constants import TunerConstants
from ids import CancoderId, TalonId
from utilities import is_red, is_sim, is_match

from choreo.trajectory import SwerveSample as ChoreoSwerveSample
from pathplannerlib.path import PathPlannerTrajectoryState


def angle_difference(angle1, angle2):
    """
    Calculate the smallest difference between two angles in radians.
    Returns the absolute difference in radians.
    """
    # Normalize the difference to be between -π and π
    diff = (angle1 - angle2) % (2 * math.pi)
    if diff > math.pi:
        diff = 2 * math.pi - diff
    return diff


class SwerveModule:
    # limit the acceleration of the commanded speeds of the robot to what is
    # actually achiveable without the wheels slipping. This is done to improve
    # odometry
    accel_limit = 15  # m/s^2

    def __init__(
        self,
        name: str,
        x: float,
        y: float,
        drive_id: int,
        steer_id: int,
        encoder_id: int,
        *,
        busname: str,
        mag_offset: float = 0.0,
        drive_reversed: bool = False,
        steer_reversed: bool = False
    ):
        """
        x, y: where the module is relative to the center of the robot
        *_id: can ids of steer and drive motors and absolute encoder
        """
        self.name = name
        self.busname = busname
        self.translation = Translation2d(x, y)
        self.state = SwerveModuleState(0, Rotation2d(0))
        self.mag_offset = mag_offset

        # Create Motor and encoder objects
        self.steer = TalonFX(steer_id, self.busname)
        self.drive = TalonFX(drive_id, self.busname)
        self.encoder = CANcoder(encoder_id, self.busname) 
        enc_config = CANcoderConfiguration()
        mag_config = MagnetSensorConfigs()
        mag_config.with_magnet_offset(mag_offset)
        enc_config.with_magnet_sensor(mag_config)
        self.encoder.configurator.apply(enc_config)  # type: ignore

        steer_motor_config = MotorOutputConfigs()
        steer_motor_config.neutral_mode = NeutralModeValue.BRAKE
        # The SDS Mk4i rotation has one pair of gears.
        steer_motor_config.inverted = InvertedValue.CLOCKWISE_POSITIVE
        steer_motor_config.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            if steer_reversed
            else InvertedValue.CLOCKWISE_POSITIVE
        )

        steer_gear_ratio_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            TunerConstants._steer_gear_ratio
        )

        # configuration for motor pid
        steer_pid = TunerConstants._steer_gains
        steer_closed_loop_config = ClosedLoopGeneralConfigs()
        steer_closed_loop_config.continuous_wrap = True

        self.steer.configurator.apply(steer_motor_config)
        self.steer.configurator.apply(steer_pid, 0.01)
        self.steer.configurator.apply(steer_gear_ratio_config)
        self.steer.configurator.apply(steer_closed_loop_config)

        # Configure drive motor
        drive_motor_config = MotorOutputConfigs()
        drive_motor_config.neutral_mode = NeutralModeValue.BRAKE
        drive_motor_config.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if drive_reversed
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

        drive_gear_ratio_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            TunerConstants._drive_gear_ratio
        )

        # configuration for motor pid and feedforward
        self.drive_pid = TunerConstants._drive_gains
        self.drive_ff = SimpleMotorFeedforwardMeters(kS=0.01, kV=0.09, kA=0.0)

        self.drive.configurator.apply(drive_motor_config)
        self.drive.configurator.apply(self.drive_pid, 0.01)
        self.drive.configurator.apply(drive_gear_ratio_config)

        self.central_angle = Rotation2d(x, y)

        self.steer_pid = PIDController(0.3, 0, 0)
        self.steer_pid.enableContinuousInput(-math.pi, math.pi)

        self.drive_request = VelocityVoltage(0)
        self.stop_request = VoltageOut(0)

    def get_angle_absolute(self) -> float:
        """Gets steer angle (rot) from absolute encoder"""
        return self.encoder.get_absolute_position().value * math.tau

    def get_rotation(self) -> Rotation2d:
        """Get the steer angle as a Rotation2d"""
        return Rotation2d(self.get_angle_absolute())

    def get_speed(self) -> float:
        # velocity is in rot/s, return in m/s
        return self.drive.get_velocity().value

    def get_distance_traveled(self) -> float:
        return self.drive.get_position().value * math.tau*TunerConstants._wheel_radius

    def set(self, desired_state: SwerveModuleState):
        self.state = desired_state
        current_angle = self.get_rotation()
        self.state.optimize(current_angle)

        if abs(self.state.speed) < 0.01:
            self.drive.set_control(self.stop_request)

        target_displacement = self.state.angle - current_angle
        target_angle = self.state.angle.radians()

        steer_output = self.steer_pid.calculate(current_angle.radians(), target_angle)
        self.steer.set_control(
            DutyCycleOut(steer_output)
        )

        diff = self.state.angle - current_angle
        if (abs(diff.degrees()) < 1):
            self.steer.set_control(DutyCycleOut(0))
        else:
            self.steer.set_control(DutyCycleOut(steer_output))

        # rescale the speed target based on how close we are to being correctly
        # aligned
        target_speed = self.state.speed * target_displacement.cos() ** 2

        # original position change/100ms, new m/s -> rot/s
        self.drive.set_control(self.drive_request.with_velocity(target_speed))

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.get_distance_traveled(), self.get_rotation())

    def get(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_speed(), self.get_rotation())


class DrivetrainComponent:
    # Here's where we inject the other components
    # Note that you can't use the components directly in the __init__ method
    # You have to use them in the setup() method
    gyro: GyroComponent
    elevator: ElevatorComponent

    HEADING_TOLERANCE = math.radians(1)

    # maxiumum speed for any wheel
    max_wheel_speed = TunerConstants.speed_at_12_volts

    chassis_speeds = magicbot.will_reset_to(ChassisSpeeds(0, 0, 0))

    field: wpilib.Field2d
    logger: Logger

    send_modules = magicbot.tunable(True)
    snapping_to_heading = magicbot.tunable(False)

    def __init__(self) -> None:

        self.publisher = (ntcore.NetworkTableInstance.getDefault()
                                                .getStructTopic("MyPose", Pose2d)
                                                .publish()
        )
        self.heading_controller = ProfiledPIDControllerRadians(
            0.5, 0, 0, TrapezoidProfileRadians.Constraints(3 * math.tau, 49 * 6)
        )
        self.heading_controller.enableContinuousInput(-math.pi, math.pi)
        self.heading_controller.setTolerance(self.HEADING_TOLERANCE)
        self.snap_heading: float | None = None

        # Leaving the old values here, using some more docile ones for driver practice temporarily
        self.default_xy_pid = (10, 1, 0)
        self.aggressive_xy_pid = (20, 0, 0)
        if is_sim():
            self.default_xy_pid = (14, 2, 0)
            self.aggressive_xy_pid = (30, 3, 0)

        self.choreo_x_controller = PIDController(*self.default_xy_pid)
        self.choreo_y_controller = PIDController(*self.default_xy_pid)
        self.choreo_heading_controller = PIDController(15, 0, 0)
        self.choreo_heading_controller.enableContinuousInput(-math.pi, math.pi)

        self.modules = (
            # Front Left
            SwerveModule(
                "Front Left",
                TunerConstants._front_left_x_pos,
                TunerConstants._front_left_y_pos,
                TalonId.DRIVE_FL.id,
                TalonId.TURN_FL.id,
                CancoderId.SWERVE_FL.id,
                busname=TalonId.DRIVE_FL.bus,
                mag_offset=TunerConstants._front_left_encoder_offset,
                steer_reversed=True,
                drive_reversed=False,
            ),
            # Front Right
            SwerveModule(
                "Front Right",
                TunerConstants._front_right_x_pos,
                TunerConstants._front_right_y_pos,
                TalonId.DRIVE_FR.id,
                TalonId.TURN_FR.id,
                CancoderId.SWERVE_FR.id,
                busname=TalonId.DRIVE_FR.bus,
                mag_offset=TunerConstants._front_right_encoder_offset,
                steer_reversed=True,
                drive_reversed=True,
            ),
            # Back Left
            SwerveModule(
                "Back Left",
                TunerConstants._back_left_x_pos,
                TunerConstants._back_left_y_pos,
                TalonId.DRIVE_BL.id,
                TalonId.TURN_BL.id,
                CancoderId.SWERVE_BL.id,
                busname=TalonId.DRIVE_BL.bus,
                mag_offset=TunerConstants._back_left_encoder_offset,
                steer_reversed=True,
                drive_reversed=False,
            ),
            # Back Right
            SwerveModule(
                "Back Right",
                TunerConstants._back_right_x_pos,
                TunerConstants._back_right_y_pos,
                TalonId.DRIVE_BR.id,
                TalonId.TURN_BR.id,
                CancoderId.SWERVE_BR.id,
                busname=TalonId.DRIVE_BR.bus,
                mag_offset=TunerConstants._back_right_encoder_offset,
                steer_reversed=True,
                drive_reversed=True,
            ),
        )

        self.kinematics = SwerveDrive4Kinematics(
            self.modules[0].translation,
            self.modules[1].translation,
            self.modules[2].translation,
            self.modules[3].translation,
        )

        nt = ntcore.NetworkTableInstance.getDefault().getTable("/components/drivetrain")
        module_states_table = nt.getSubTable("module_states")
        if not is_match():
            self.setpoints_publisher = module_states_table.getStructArrayTopic(
                "setpoints", SwerveModuleState
            ).publish()
            self.measurements_publisher = module_states_table.getStructArrayTopic(
                "measured", SwerveModuleState
            ).publish()

            wpilib.SmartDashboard.putData("Heading PID", self.heading_controller)

    def get_chassis_speeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.get_module_states())

    def get_module_states(
        self,
    ) -> tuple[
        SwerveModuleState,
        SwerveModuleState,
        SwerveModuleState,
        SwerveModuleState,
    ]:
        return (
            self.modules[0].get(),
            self.modules[1].get(),
            self.modules[2].get(),
            self.modules[3].get(),
        )

    def get_heading(self) -> Rotation2d:
        return self.gyro.get_Rotation2d()

    def setup(self) -> None:
        # TODO update with new game info
        initial_pose = Pose2d(Translation2d(0, 0), Rotation2d(0))

        self.estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_heading(),
            self.get_module_positions(),
            initial_pose,
            stateStdDevs=(0.01, 0.01, 0.01),  # How much to trust wheel odometry
            visionMeasurementStdDevs=(0.4, 0.4, 0.2),
        )
        self.field_obj = self.field.getObject("fused_pose")
        self.set_pose(initial_pose)
        heading = 180 if is_red() else 0
        self.gyro.reset_heading(heading)

    def drive_field(self, vx: float, vy: float, omega: float) -> None:
        """Field oriented drive commands"""
        pn = wpilib.SmartDashboard.putNumber
        pn('direct dx', vx)
        pn('direct dy', vy)
        pn('direct do', omega)
        current_heading = self.get_rotation()
        self.chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, omega, current_heading
        )

    def get_robot_speeds(self) -> tuple[float, float]:
        vx = self.chassis_speeds.vx
        vy = self.chassis_speeds.vy
        total_speed = math.sqrt(vx*vx + vy*vy)
        return total_speed, self.chassis_speeds.omega

    def halt(self):
        self.drive_local(0, 0, 0)
    
    def drive_to_pose(self, pose: Pose2d, aggressive=False) -> None:
        self.drive_to_position(
            pose.X(), pose.Y(), pose.rotation().radians(), aggressive=False
        )

    def drive_to_position(
        self, x: float, y: float, heading: float, aggressive=False
    ) -> None:
        if aggressive:
            self.choreo_x_controller.setPID(*self.aggressive_xy_pid)
            self.choreo_y_controller.setPID(*self.aggressive_xy_pid)
        else:
            self.choreo_x_controller.setPID(*self.default_xy_pid)
            self.choreo_y_controller.setPID(*self.default_xy_pid)
        robot_pose = self.get_pose()
        xvel = self.choreo_x_controller.calculate(robot_pose.X(), x)
        yvel = self.choreo_y_controller.calculate(robot_pose.Y(), y)
        hvel = self.choreo_heading_controller.calculate(robot_pose.rotation().radians(), heading)
        if is_sim():
            hvel = self.choreo_heading_controller.calculate(robot_pose.rotation().radians(), heading)
        self.drive_field(xvel, yvel, hvel)

    def drive_local(self, vx: float, vy: float, omega: float) -> None:
        """Robot oriented drive commands"""
        # if abs(omega) < 0.01 and self.snap_heading is None:
        #     self.snap_to_heading(self.get_heading().radians())
        self.chassis_speeds = ChassisSpeeds(vx, vy, omega)

    # Note that haeding should be in radians
    def snap_to_heading(self, heading: float) -> None:
        """set a heading target for the heading controller"""
        self.snapping_to_heading = True
        self.snap_heading = heading
        if self.snap_heading is not None:
            self.heading_controller.setGoal(self.snap_heading)

    def stop_snapping(self) -> None:
        """stops the heading_controller"""
        self.snapping_to_heading = False
        self.snap_heading = None

    def execute(self) -> None:
        if self.snapping_to_heading:
            self.chassis_speeds.omega = self.heading_controller.calculate(
                self.get_rotation().radians()
            )
        else:
            self.heading_controller.reset(
                self.get_rotation().radians(), self.get_rotational_velocity()
            )

        desired_speeds = self.chassis_speeds
        # ----------------------------------------
        # These limits should not change!
        # TODO Update based off real robot speeds.
        elevator_factor = 1.0
        if self.elevator.get_position() > 10:
            elevator_factor = 1.225 - (0.9 / 40) * self.elevator.get_position()
            elevator_factor = max(0.25, elevator_factor)
        # ----------------------------------------
        # ---------------------------------------- 

        desired_states = self.kinematics.toSwerveModuleStates(desired_speeds)
        desired_states = self.kinematics.desaturateWheelSpeeds(
            desired_states, attainableMaxSpeed=self.max_wheel_speed * elevator_factor
        )

        for state, module in zip(desired_states, self.modules):
            module.set(state)

        self.update_odometry()
    
    def follow_trajectory_pp(self, sample: PathPlannerTrajectoryState):
        # Get the current pose of the robot
        pose = self.get_pose()

        # Generate the next speeds for the robot
        dx = sample.fieldSpeeds.vx + self.choreo_x_controller.calculate(
            pose.X(), sample.pose.X()
        )
        dy = sample.fieldSpeeds.vy + self.choreo_y_controller.calculate(
            pose.Y(), sample.pose.Y()
        )
        do = sample.fieldSpeeds.omega + self.choreo_heading_controller.calculate(
            pose.rotation().radians(), sample.pose.rotation().radians()
        )

        # Apply the generated speeds
        self.drive_field(dx, dy, do)

    def follow_trajectory_choreo(self, sample: ChoreoSwerveSample):
        # Get the current pose of the robot
        pose = self.get_pose()

        # Generate the next speeds for the robot
        dx = sample.vx + self.choreo_x_controller.calculate(pose.X(), sample.x)
        dy = sample.vy + self.choreo_y_controller.calculate(pose.Y(), sample.y)
        # do = sample.omega + self.choreo_heading_controller.calculate(pose.rotation().radians(), sample.heading)
        do = sample.omega + self.heading_controller.calculate(pose.rotation().radians(), sample.heading)
        # Apply the generated speeds
        self.drive_field(dx, dy, do)

    def on_enable(self) -> None:
        """update the odometry so the pose estimator doesn't have an empty
        buffer

        While we should be building the pose buffer while disabled, this
        accounts for the edge case of crashing mid match and immediately
        enabling with an empty buffer"""
        self.update_odometry()

    def get_rotational_velocity(self) -> float:
        v = self.gyro.pigeon.get_angular_velocity_z_world().value
        return math.radians(v)

    def update_odometry(self) -> None:
        self.estimator.update(self.gyro.get_Rotation2d(), self.get_module_positions())

        self.field_obj.setPose(self.get_pose())
        self.publisher.set(self.get_pose())
        if self.send_modules:
            self.setpoints_publisher.set([module.state for module in self.modules])
            self.measurements_publisher.set([module.get() for module in self.modules])

    def set_pose(self, pose: Pose2d) -> None:
        self.estimator.resetPosition(
            self.gyro.get_Rotation2d(), self.get_module_positions(), pose
        )
        self.publisher.set(pose)
        self.field.setRobotPose(pose)
        self.field_obj.setPose(pose)

    def reset_yaw(self) -> None:
        """Sets pose to current pose but with a heading of forwards"""
        cur_pose = self.estimator.getEstimatedPosition()
        default_heading = math.pi if is_red() else 0
        self.set_pose(Pose2d(cur_pose.translation(), Rotation2d(default_heading)))

    def reset_odometry(self) -> None:
        """Reset odometry to current team's podium"""
        # TODO update with new game info
        pass

    def get_module_positions(
        self,
    ) -> tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        return (
            self.modules[0].get_position(),
            self.modules[1].get_position(),
            self.modules[2].get_position(),
            self.modules[3].get_position(),
        )

    def get_pose(self) -> Pose2d:
        """Get the current location of the robot relative to ???"""
        return self.estimator.getEstimatedPosition()

    def get_rotation(self) -> Rotation2d:
        """Get the current heading of the robot."""
        return self.get_pose().rotation()

    @feedback
    def at_desired_heading(self) -> bool:
        return self.heading_controller.atGoal()
