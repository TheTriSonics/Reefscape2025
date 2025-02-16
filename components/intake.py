
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

from components.photoeye import PhotoEyeComponent
from utilities.game import is_sim

pn = wpilib.SmartDashboard.putNumber

class IntakeDirection(Enum):
    NONE = 0
    CORAL_IN = 1
    CORAL_SCORE = 2


class IntakeComponent:
    photoeye: PhotoEyeComponent
    bus = 'canivore'
    # motor = TalonFX(TalonId.MANIP_INTAKE, bus)

    force_coral_score = tunable(False)
    force_coral_intake = tunable(False)

    auto_coral_intake = tunable(True)

    motor_request = DutyCycleOut(0, override_brake_dur_neutral=True)
    direction = IntakeDirection.NONE
    
    def intake_coral_in(self):
        # TODO: This might not always mean forward, but for now
        # we'll keep it simple
        self.direction = IntakeDirection.CORAL_IN

    def intake_coral_score(self):
        self.direction = IntakeDirection.CORAL_SCORE

    def intake_off(self):
        self.direction = IntakeDirection.NONE

    def execute(self):
        motor_power = 0.0
        if self.force_coral_intake:
            motor_power = 0.2
        if self.force_coral_score:
            motor_power = -0.2

        if self.direction == IntakeDirection.CORAL_IN:
            motor_power = 0.2
        elif self.direction == IntakeDirection.CORAL_SCORE:
            motor_power = -0.2
        
        if is_sim():
            motor_power *= 10 

        self.motor_request.output = motor_power
        # self.motor.set_control(self.motor_request)
