import wpilib
from magicbot import feedback, tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import (
    DutyCycleOut,
    MotionMagicVoltage,
    DynamicMotionMagicVoltage,
)
from phoenix6.configs import TalonFXConfiguration
from enum import Enum
from ids import TalonId

pn = wpilib.SmartDashboard.putNumber


class WristComponent:
    bus = 'elevator_can'
    motor = TalonFX(TalonId.MANIP_WRIST, bus)
    target_pos = tunable(0.0)
    motor_request = MotionMagicVoltage(0, override_brake_dur_neutral=True)
    
    def __init__(self):
        arm_config = TalonFXConfiguration()
        arm_config.slot0.k_s = 0.25
        arm_config.slot0.k_v = 0.12
        arm_config.slot0.k_a = 0.01
        arm_config.slot0.k_p = 0.1
        arm_config.slot0.k_i = 0
        arm_config.slot0.k_d = 0.1
        arm_config.motion_magic.motion_magic_cruise_velocity = 10
        arm_config.motion_magic.motion_magic_acceleration = 40
        arm_config.motion_magic.motion_magic_jerk = 400
        self.motor.configurator.apply(arm_config)  # type: ignore
    
    @feedback
    def get_position(self) -> float:
        return self.motor.get_position().value

    @feedback
    def at_goal(self):
        current_pos = self.get_position()
        diff = abs(self.target_pos - current_pos)
        return diff < 0.01

    def execute(self):
        if not self.at_goal():
            req = self.motor_request.with_position(self.target_pos)
            self.motor.set_control(req)