from magicbot import feedback
from phoenix6.hardware import TalonFX
from phoenix6.controls import PositionDutyCycle, DutyCycleOut
from enum import Enum


class IntakeDirection(Enum):
    NONE = 0
    FORWARD = 1
    REVERSE = 2


class ManipulatorComponent:
    # TODO: Change these numbers out for something in ids.py
    elevator_motor_left = TalonFX(1, 'elevator_can')
    elevator_motor_right = TalonFX(2, 'elevator_can')
    arm_motor = TalonFX(3, 'elevator_can')
    wrist_motor = TalonFX(4, 'elevator_can')
    intake_motor = TalonFX(5, 'elevator_can')

    def __init__(self):
        # TODO: Set the right motor to follow the left and then set motion magic
        # controls up
        self.elevator_target_pos = 0
        self.arm_target_pos = 0
        self.wrist_target_pos = 0
        self.intake_direction = IntakeDirection.NONE
        pass

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
    def wrist_at_goal(self):
        current_pos = self.wrist_motor.get_position().value
        diff = abs(self.wrist_target_pos - current_pos)
        return diff < 0.01

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
            # TODO: Make this real
            pass

        intake_motor_speed = 0
        if self.intake_direction == IntakeDirection.FORWARD:
            intake_motor_speed = 0.5
        elif self.intake_direction == IntakeDirection.REVERSE:
            intake_motor_speed = -0.5
        self.intake_motor.set_control(DutyCycleOut(intake_motor_speed))

