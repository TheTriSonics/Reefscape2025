from magicbot import feedback
from phoenix6.hardware import TalonFX
from phoenix6.controls import PositionDutyCycle

class ManipulatorComponent:
    # TODO: Change these numbers out for something in ids.py
    left_motor = TalonFX(1, 'elevator_can')
    right_motor = TalonFX(2, 'elevator_can')

    def __init__(self):
        # TODO:
        # Set the right motor to follow the left
        # and then set motion magic controls up
        self.elevator_target_pos = 0
        pass

    def elevator_go_level1(self):
        self.elevator_target_pos = 100
    
    def elevator_go_level2(self):
        self.elevator_target_pos = 200
    
    def elevator_go_level3(self):
        self.elevator_target_pos = 300
    
    def elevator_go_level4(self):
        self.elevator_target_pos = 400

    @feedback
    def elevator_at_goal(self):
        current_pos = self.right_motor.get_position().value
        diff = abs(self.elevator_target_pos - current_pos)
        if diff < 0.01:
            return True
        return False


    def execute(self):
        if self.elevator_at_goal():
            return
        # TODO: Make this real; this is not right.
        self.right_motor.set_control(
            PositionDutyCycle(self.elevator_target_pos)
        )
        pass
