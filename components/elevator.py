import wpilib
from magicbot import feedback, tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import (
    MotionMagicVoltage,
    Follower,
)
from phoenix6.configs import TalonFXConfiguration
from enum import Enum
from ids import TalonId
from utilities.game import ManipLocations, ManipLocation

pn = wpilib.SmartDashboard.putNumber


class ElevatorComponent:
    bus = 'canivore'
    # motor_left = TalonFX(TalonId.MANIP_ELEVATOR_LEFT, bus)
    # motor_right = TalonFX(TalonId.MANIP_ELEVATOR_RIGHT, bus)
    default_pos = 0.0
    target_pos = tunable(default_pos)
    motor_request = MotionMagicVoltage(0, override_brake_dur_neutral=True)
    fake_pos = tunable(default_pos)
    
    def __init__(self):
        config = TalonFXConfiguration()
        config.slot0.k_s = 0.0
        config.slot0.k_v = 0.12
        config.slot0.k_a = 0.01
        config.slot0.k_p = 0.5
        config.slot0.k_i = 0
        config.slot0.k_d = 0.1
        config.motion_magic.motion_magic_cruise_velocity = 10
        config.motion_magic.motion_magic_acceleration = 400
        config.motion_magic.motion_magic_jerk = 4000
        # self.motor_left.configurator.apply(config)  # type: ignore
    
    @feedback
    def get_position(self) -> float:
        # return self.motor_left.get_position().value
        return self.fake_pos

    @feedback
    def at_goal(self):
        current_pos = self.get_position()
        target_loc = ManipLocation(self.target_pos, 0, 0)
        current_loc = ManipLocation(current_pos, 0, 0)
        return current_loc == target_loc

    def execute(self):
        if abs(self.fake_pos - self.target_pos) < 0.1:
            self.fake_pos = self.target_pos
        elif self.fake_pos < self.target_pos:
            self.fake_pos += 1
        elif self.fake_pos > self.target_pos:
            self.fake_pos -= 1

        if not self.at_goal():
            req = self.motor_request.with_position(self.target_pos)
            # self.motor_left.set_control(req)
            # Should this only be done once in setup()? 
            # self.motor_right.set_control(
            #     Follower(TalonId.MANIP_ELEVATOR_LEFT,
            #              oppose_master_direction=True),
            # )

