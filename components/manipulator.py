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


class ManipulatorComponent:
    # TODO: Change these numbers out for something in ids.py
    bus = 'elevator_can'
    elevator_motor_left = TalonFX(TalonId.MANIP_ELEVATOR_LEFT, bus)
    elevator_motor_right = TalonFX(TalonId.MANIP_ELEVATOR_RIGHT, bus)
    arm_motor = TalonFX(TalonId.MANIP_ARM, bus)
    wrist_motor = TalonFX(TalonId.MANIP_WRIST, bus)
    intake_motor = TalonFX(TalonId.MANIP_INTAKE, bus)

    elevator_target_pos = tunable(0.0)
    arm_target_pos = tunable(0.0)
    wrist_target_pos = tunable(0.0)

    wrist_request = MotionMagicVoltage(0)

    def __init__(self):
        # TODO: Set the right motor to follow the left and then set motion magic
        # controls up
        self.intake_direction = IntakeDirection.NONE

        wrist_config = TalonFXConfiguration()
        wrist_config.slot0.k_s = 0.25
        wrist_config.slot0.k_v = 0.12
        wrist_config.slot0.k_a = 0.01
        wrist_config.slot0.k_p = 0.1
        wrist_config.slot0.k_i = 0
        wrist_config.slot0.k_d = 0.1
        wrist_config.motion_magic.motion_magic_cruise_velocity = 10
        wrist_config.motion_magic.motion_magic_acceleration = 40
        wrist_config.motion_magic.motion_magic_jerk = 400
        self.wrist_motor.configurator.apply(wrist_config)  # type: ignore

    def elevator_go_level1(self):
        self.elevator_target_pos = 100

    def elevator_go_level2(self):
        self.elevator_target_pos = 200

    def elevator_go_level3(self):
        self.elevator_target_pos = 300

    def elevator_go_level4(self):
        self.elevator_target_pos = 400

    def arm_go_level1(self):
        self.arm_target_pos = 100

    def arm_go_level2(self):
        self.arm_target_pos = 200

    def arm_go_level3(self):
        self.arm_target_pos = 300

    def arm_go_level4(self):
        self.arm_target_pos = 400

    def wrist_go_level1(self):
        self.wrist_target_pos = 100

    def wrist_go_level2(self):
        self.wrist_target_pos = 200

    def wrist_go_level3(self):
        self.wrist_target_pos = 300

    def wrist_go_level4(self):
        self.wrist_target_pos = 400

    def intake_in(self):
        # On the actual robot this might not always be the same direction; the
        # system has the ability to intake and eject coral in both directions
        self.intake_direction = IntakeDirection.FORWARD

    def intake_eject(self):
        # On the actual robot this might not always be the same direction; the
        # system has the ability to intake and eject coral in both directions
        self.intake_direction = IntakeDirection.FORWARD

    def intake_off(self):
        self.intake_direction = IntakeDirection.NONE

    @feedback
    def elevator_at_goal(self):
        current_pos = self.elevator_motor_right.get_position().value
        diff = abs(self.elevator_target_pos - current_pos)
        return diff < 0.01

    @feedback
    def arm_at_goal(self):
        current_pos = self.arm_motor.get_position().value
        diff = abs(self.arm_target_pos - current_pos)
        return diff < 0.01

    @feedback
    def wrist_position(self) -> float:
        return self.wrist_motor.get_position().value

    @feedback
    def wrist_at_goal(self):
        current_pos = self.wrist_position()
        diff = abs(self.wrist_target_pos - current_pos)
        pn('wrist pos', current_pos)
        pn('wrist diff', diff)
        return diff < 0.50

    def execute(self):
        if not self.elevator_at_goal():
            arm_pos = self.arm_motor.get_position().value
            # TODO: Check to see if the arm is in a position where we can
            # move the elvator
            # TODO: Make this real
            pass
        if not self.arm_at_goal():
            # TODO: Make this real
            pass
        if not self.wrist_at_goal():
            req = self.wrist_request.with_position(self.wrist_target_pos).with_enable_foc(False)
            self.wrist_motor.set_control(req)

        intake_motor_speed = 0
        if self.intake_direction == IntakeDirection.FORWARD:
            intake_motor_speed = 0.5
        elif self.intake_direction == IntakeDirection.REVERSE:
            intake_motor_speed = -0.5
        self.intake_motor.set_control(DutyCycleOut(intake_motor_speed))

