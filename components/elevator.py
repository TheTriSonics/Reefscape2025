import wpilib
from magicbot import feedback, tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import (
    MotionMagicVoltage, Follower, MotionMagicDutyCycle, DutyCycleOut
)
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs, CurrentLimitsConfigs, HardwareLimitSwitchConfigs
from utilities.game import ManipLocation
from utilities import is_sim
from phoenix6.signals import InvertedValue, NeutralModeValue

from ids import TalonId

pn = wpilib.SmartDashboard.putNumber


class ElevatorComponent:
    bus = TalonId.MANIP_ELEVATOR_LEFT.bus
    motor_left = TalonFX(TalonId.MANIP_ELEVATOR_LEFT.id, bus)
    motor_right = TalonFX(TalonId.MANIP_ELEVATOR_RIGHT.id, bus)
    default_pos = 2.0
    target_pos = tunable(default_pos)
    motor_request = MotionMagicDutyCycle(0, override_brake_dur_neutral=True)
    upper_limit = 80.0
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
        config.motion_magic.motion_magic_cruise_velocity = 80
        config.motion_magic.motion_magic_acceleration = 200
        config.motion_magic.motion_magic_jerk = 1000
        output_config = MotorOutputConfigs()
        output_config.inverted = InvertedValue.CLOCKWISE_POSITIVE
        output_config.neutral_mode = NeutralModeValue.BRAKE
        self.motor_left.configurator.apply(config)  # type: ignore
        self.motor_left.configurator.apply(output_config)
        self.motor_right.configurator.apply(config)  # type: ignore
        self.motor_right.configurator.apply(output_config)

        limit_configs = CurrentLimitsConfigs()
        # Limit the motor output so we don't harm anything (including the robot)
        # if it hits an obstacle.
        limit_configs.stator_current_limit = 50
        limit_configs.stator_current_limit_enable = True
        self.motor_left.configurator.apply(limit_configs)
        self.motor_right.configurator.apply(limit_configs)

        switch_configs = HardwareLimitSwitchConfigs()
        switch_configs.forward_limit_enable = True

    
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
        # if self.motor_left.get_forward_limit().value:
        #     self.motor_left.set_position(0.0)

        if self.target_pos < 0.5:
            self.target_pos = 0.5
        elif self.target_pos > self.upper_limit:
            self.target_pos = self.upper_limit

        req = self.motor_request.with_position(self.target_pos)
        self.motor_left.set_control(req)
        # Should this only be done once in setup()? 
        self.motor_right.set_control(
            Follower(TalonId.MANIP_ELEVATOR_LEFT.id,
                        oppose_master_direction=True),
        )

