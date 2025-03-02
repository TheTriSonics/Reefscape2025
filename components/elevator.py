import wpilib
from magicbot import feedback, tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import (
    MotionMagicVoltage, Follower, MotionMagicDutyCycle, DutyCycleOut
)
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs
from utilities.game import ManipLocation
from utilities import is_sim
from phoenix6.signals import InvertedValue, NeutralModeValue
from components.arm import ArmComponent

from ids import TalonId

pn = wpilib.SmartDashboard.putNumber


class ElevatorComponent:
    bus = TalonId.MANIP_ELEVATOR_LEFT.bus
    motor_left = TalonFX(TalonId.MANIP_ELEVATOR_LEFT.id, bus)
    motor_right = TalonFX(TalonId.MANIP_ELEVATOR_RIGHT.id, bus)
    default_pos = 2.0
    target_pos = tunable(default_pos)
    motor_request = MotionMagicDutyCycle(0, override_brake_dur_neutral=True)
    arm: ArmComponent
    upper_limit = 40.0
    lower_limit = 0.5
    
    def __init__(self):
        config = TalonFXConfiguration()
        config.slot0.k_s = 0.0
        config.slot0.k_v = 0.0
        config.slot0.k_a = 0.0
        config.slot0.k_p = 0.5
        config.slot0.k_i = 0.0
        config.slot0.k_d = 0.0
        config.slot0.k_g = 0.0
        if is_sim():
            config.slot0.k_s = 0.0
            config.slot0.k_v = 0.0
            config.slot0.k_a = 0.0
            config.slot0.k_p = 0.05
            config.slot0.k_i = 0.0
            config.slot0.k_d = 0.0
            config.slot0.k_g = 0.0
        config.motion_magic.motion_magic_cruise_velocity = 40
        config.motion_magic.motion_magic_acceleration = 400
        config.motion_magic.motion_magic_jerk = 4000
        output_config = MotorOutputConfigs()
        output_config.inverted = InvertedValue.CLOCKWISE_POSITIVE
        output_config.neutral_mode = NeutralModeValue.BRAKE
        self.motor_left.configurator.apply(config)  # type: ignore
        self.motor_left.configurator.apply(output_config)
        self.motor_right.configurator.apply(config)  # type: ignore
        self.motor_right.configurator.apply(output_config)
    
    @feedback
    def get_position(self) -> float:
        return self.motor_left.get_position().value

    @feedback
    def at_goal(self):
        current_pos = self.get_position()
        target_loc = ManipLocation(self.target_pos, 0, 0)
        current_loc = ManipLocation(current_pos, 0, 0)
        return current_loc == target_loc

    def execute(self):
        # Elevator----------------------------------------
        # This limits should not change!
        if self.target_pos < self.lower_limit:
            # TODO Add the lower limit switch.
            #  Driving the elevator below 0 would be bad. Very bad. So don't let
            # anybody do that!
            self.target_pos = self.lower_limit
        if self.target_pos > self.upper_limit:
            # There's a max height to this elevator and we don't want to try and
            # exceed it. That would also be bad. Very bad.
            self.target_pos = self.upper_limit

        # if self.arm.get_position() < -65:
        #     self.target_pos = self.get_position()
        # This limits should not change!
        # Elevator----------------------------------------

        req = self.motor_request.with_position(self.target_pos)
        if self.at_goal():
            self.motor_left.set_control(DutyCycleOut(0))
            self.motor_right.set_control(DutyCycleOut(0))
        else:
            self.motor_left.set_control(req)
            # Should this only be done once in setup()? 
            self.motor_right.set_control(
                Follower(TalonId.MANIP_ELEVATOR_LEFT.id,
                            oppose_master_direction=True),
            )

