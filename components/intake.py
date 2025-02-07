
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

class IntakeDirection(Enum):
    NONE = 0
    FORWARD = 1
    REVERSE = 2


class IntakeComponent:
    bus = 'canivore'
    motor = TalonFX(TalonId.MANIP_ARM, bus)
    target_pos = tunable(0.0)
    motor_request = DutyCycleOut(0, override_brake_dur_neutral=True)
    direction = IntakeDirection.NONE

    def intake_in(self):
        # TODO: This might not always mean forward, but for now
        # we'll keep it simple
        self.direction = IntakeDirection.FORWARD

    def intake_out(self):
        self.direction = IntakeDirection.REVERSE

    def intake_off(self):
        self.direction = IntakeDirection.NONE

    def execute(self):
        motor_power = 0
        if self.direction == IntakeDirection.FORWARD:
            motor_power = 0.2
        elif self.direction == IntakeDirection.REVERSE:
            motor_power = -0.2

        self.motor_request.output = motor_power
        self.motor.set_control(self.motor_request)

