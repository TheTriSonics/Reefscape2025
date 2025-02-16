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
    bus = 'canivore'
    # motor = TalonFX(TalonId.MANIP_WRIST, bus)
    default_pos = 135.0
    target_pos = tunable(default_pos)
    motor_request = MotionMagicVoltage(0, override_brake_dur_neutral=True)
    fake_pos = tunable(default_pos)
    
    def __init__(self):
        config = TalonFXConfiguration()
        config.slot0.k_s = 0.0
        config.slot0.k_v = 0.8
        config.slot0.k_a = 0.01
        config.slot0.k_p = 0.5
        config.slot0.k_i = 0
        config.slot0.k_d = 0.1
        config.motion_magic.motion_magic_cruise_velocity = 100
        config.motion_magic.motion_magic_acceleration = 1600
        config.motion_magic.motion_magic_jerk = 4000
        # self.motor.set_position(self.default_pos)
        # self.motor.configurator.apply(config)  # type: ignore
    
    @feedback
    def get_position(self) -> float:
        # return self.motor.get_position().value
        return self.fake_pos

    @feedback
    def at_goal(self):
        current_pos = self.get_position()
        diff = abs(self.target_pos - current_pos)
        return diff < 0.25

    def execute(self):
        if abs(self.fake_pos - self.target_pos) < 0.25:
            self.fake_pos = self.target_pos
        elif self.fake_pos < self.target_pos:
            self.fake_pos += min(3, self.target_pos - self.fake_pos)
        elif self.fake_pos > self.target_pos:
            self.fake_pos -= min(3, self.fake_pos - self.target_pos)

        if not self.at_goal():
            req = self.motor_request.with_position(self.target_pos)
            # self.motor.set_control(req)