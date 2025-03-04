from enum import Enum
from magicbot import tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import (
    DutyCycleOut,
)

from components.photoeye import PhotoEyeComponent
from components import (
    ElevatorComponent,
    WristComponent,
    ArmComponent,
    DrivetrainComponent,
)
from ids import TalonId


class IntakeDirection(Enum):
    NONE = 0
    CORAL_IN = 1
    CORAL_SCORE = 2
    ALGAE_SCORE = 3
    ALGAE_IN = 4


class IntakeComponent:
    elevator: ElevatorComponent
    wrist: WristComponent
    arm: ArmComponent
    photoeye: PhotoEyeComponent
    drivetrain: DrivetrainComponent
    motor = TalonFX(TalonId.MANIP_INTAKE.id, TalonId.MANIP_INTAKE.bus)

    force_coral_score = tunable(False)
    force_coral_intake = tunable(False)

    go_fast = tunable(False)

    coral_pose_msg = tunable('')

    has_coral = tunable(False)
    direction_int = tunable(0)

    force_algae_score = tunable(False)
    force_algae_intake = tunable(False)

    motor_request = DutyCycleOut(0, override_brake_dur_neutral=True)
    direction = IntakeDirection.NONE

    def __init__(self):
        self._coral_pose = None
        from phoenix6 import configs
        limit_configs = configs.CurrentLimitsConfigs()
        # enable stator current limit to keep algae from falling out when
        # the motor is trying to keep it in
        limit_configs.stator_current_limit = 12
        limit_configs.stator_current_limit_enable = True
        self.motor.configurator.apply(limit_configs)

    def coral_in(self):
        self.go_fast = False
        self.direction = IntakeDirection.CORAL_IN

    def algae_in(self):
        self.go_fast = True
        self.direction = IntakeDirection.ALGAE_IN

    def score_coral(self):
        self.go_fast = True
        self.direction = IntakeDirection.CORAL_SCORE

    def score_coral_reverse(self):
        self.go_fast = True
        self.direction = IntakeDirection.CORAL_IN

    def score_algae(self):
        self.go_fast = True
        self.direction = IntakeDirection.ALGAE_SCORE

    def intake_off(self):
        self.go_fast = False
        self.direction = IntakeDirection.NONE

    def execute(self):
        self.direction_int = self.direction.value
        speed_val = 0.3
        if self.go_fast:
            speed_val = 0.8

        motor_power = 0.0
        if self.force_coral_intake:
            motor_power = speed_val
        if self.force_coral_score:
            motor_power = -speed_val
        if self.force_algae_intake:
            motor_power = speed_val
        if self.force_algae_score:
            motor_power = -speed_val

        if self.direction in [IntakeDirection.CORAL_IN, IntakeDirection.ALGAE_SCORE]:
            motor_power = speed_val
        elif (
            self.direction in [IntakeDirection.CORAL_SCORE, IntakeDirection.ALGAE_IN]
        ):
            motor_power = -speed_val

        self.motor_request.output = motor_power
        self.motor.set_control(self.motor_request)
