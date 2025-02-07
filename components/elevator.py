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

pn = wpilib.SmartDashboard.putNumber


class ElevatorComponent:
    bus = 'canivore'
    motor_left = TalonFX(TalonId.MANIP_ELEVATOR_LEFT, bus)
    motor_right = TalonFX(TalonId.MANIP_ELEVATOR_RIGHT, bus)
    target_pos = tunable(0.0)
    motor_request = MotionMagicVoltage(0, override_brake_dur_neutral=True)
    
    @feedback
    def get_position(self) -> float:
        return self.motor_left.get_position().value

    @feedback
    def at_goal(self):
        current_pos = self.get_position()
        diff = abs(self.target_pos - current_pos)
        return diff < 0.01

    def execute(self):
        if not self.at_goal():
            req = self.motor_request.with_position(self.target_pos)
            self.motor_left.set_control(req)
            self.motor_right.set_control(
                Follower(TalonId.MANIP_ELEVATOR_LEFT,
                         oppose_master_direction=True),
            )

