import wpilib
from enum import Enum
from magicbot import tunable, feedback
from phoenix6.hardware import TalonFX
from phoenix6.controls import (
    DutyCycleOut,
)
from phoenix6.configs import (
    MotorOutputConfigs,
)
from ids import TalonId, PWM
from phoenix6.signals import InvertedValue, NeutralModeValue


class ClimbDirection(Enum):
    NONE = 0
    CLIMB_UP = 1
    CLIMB_DOWN = 2


class ClimberComponent:
    climber_motor = TalonFX(TalonId.CLIMB.id, TalonId.CLIMB.bus)
    intake_breaker = wpilib.Servo(PWM.INTAKE_BREAKER)

    position = tunable(0.0)

    speed = tunable(0.0)

    motor_request = DutyCycleOut(0, override_brake_dur_neutral=True)
    direction = ClimbDirection.NONE
    lower_limit = -10.0
    upper_limit = 140.0

    def __init__(self):
        from phoenix6 import configs
        climb_motor_configurator = self.climber_motor.configurator
        climb_motor_config = MotorOutputConfigs()
        climb_motor_config.neutral_mode = NeutralModeValue.BRAKE
        climb_motor_config.inverted = InvertedValue.CLOCKWISE_POSITIVE
        climb_motor_configurator.apply(climb_motor_config)

        limit_configs = configs.CurrentLimitsConfigs()
        limit_configs.stator_current_limit = 120
        limit_configs.stator_current_limit_enable = True
        self.climber_motor.configurator.apply(limit_configs)


    def setup(self):
        # Magic numbers are from documentation: https://cdn.andymark.com/media/W1siZiIsIjIwMTkvMDMvMjIvMTAvMjcvNTgvMDMxOTQ4ODUtYmM5Yi00M2UyLWE1NDAtZGNiMWVhNzEzMDEzL1VzaW5nIEwxNiBMaW5lYXIgU2Vydm8gMDMtMjAxOS5wZGYiXV0/Using%20L16%20Linear%20Servo%2003-2019.pdf?sha=ee4c9607408cc835
        # and they are wrong! you have to multiply them all by 1,000
        self.intake_breaker.setBounds(
            2.0 * 1000, 1.8 * 1000, 1.5 * 1000, 1.2 * 1000, 1.0 * 1000
        )
        pass

    def break_intake(self):
        self.intake_breaker.setSpeed(-1.0)

    def lock_intake(self):
        self.intake_breaker.setSpeed(-0.05)

    def climb_up(self):
        self.direction = ClimbDirection.CLIMB_UP

    def climb_down(self):
        self.direction = ClimbDirection.CLIMB_DOWN

    @feedback
    def get_position(self) -> float:
        return self.climber_motor.get_position().value

    def execute(self):
        self.direction_int = self.direction.value
        motor_power = 0.0

        if self.direction == ClimbDirection.CLIMB_UP:
            motor_power =  1.0
        elif self.direction == ClimbDirection.CLIMB_DOWN:
            motor_power = -0.8

        self.position = self.get_position()
        # ----------------------------------------
        # This limits should not change!
        if self.position < self.lower_limit:
            # Driving the climber below -60 would be bad. Very bad. So don't let
            # anybody do that!
            motor_power = max(motor_power, 0)
        if self.position > self.upper_limit:
            # Driving the climber above 60 would be bad. Very bad. So don't let
            # anybody do that!
            motor_power = min(motor_power, 0)
        # This limits should not change!

        wpilib.SmartDashboard.putNumber("Climber power", motor_power)

        self.motor_request.output = motor_power
        self.climber_motor.set_control(self.motor_request)