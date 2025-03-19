from enum import Enum
from magicbot import tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import (
    DutyCycleOut,
)
from phoenix6.configs import (
    MotorOutputConfigs,
)
from ids import TalonId
from phoenix6.signals import NeutralModeValue

class ClimbDirection(Enum):
    NONE = 0
    CLIMB_UP = 1
    CLIMB_DOWN = 2

class ClimberComponent:
    climber_motor = TalonFX(TalonId.CLIMB.id, TalonId.CLIMB.bus)

    force_climber_up = tunable(False)
    force_climber_down = tunable(False)
    position = tunable(0.0)

    go_fast = tunable(False)

    motor_request = DutyCycleOut(0, override_brake_dur_neutral=True)
    direction = ClimbDirection.NONE
    lower_limit = -90.0
    upper_limit = 60.0

    
    def __init__(self):
        from phoenix6 import configs
        climb_motor_configurator = self.climber_motor.configurator
        climb_motor_config = MotorOutputConfigs()
        climb_motor_config.neutral_mode = NeutralModeValue.BRAKE
        climb_motor_configurator.apply(climb_motor_config)

        limit_configs = configs.CurrentLimitsConfigs()
        limit_configs.stator_current_limit = 120
        limit_configs.stator_current_limit_enable = True
        self.climber_motor.configurator.apply(limit_configs)

    def climb_up(self):
        self.go_fast = False
        self.direction = ClimbDirection.CLIMB_UP

    def climb_down(self):
        self.go_fast = False
        self.direction = ClimbDirection.CLIMB_DOWN

    def execute(self):
        self.direction_int = self.direction.value
        motor_power = 0.0
        if self.go_fast:
            speed_val = 0.9
            motor_power = -speed_val

        # Put the force calls after the normal operation. Operator should win
        # if there is any disagreement. Obey the humans!!!
        if self.force_climber_up:
            motor_power = 0.15
        if self.force_climber_down:
            motor_power = -0.15
        self.position = self.climber_motor.get_position().value
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
        #--------------------------------------------


        self.motor_request.output = motor_power
        self.climber_motor.set_control(self.motor_request)