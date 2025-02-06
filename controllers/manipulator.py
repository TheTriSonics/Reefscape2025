from magicbot import StateMachine, state, timed_state, default_state, will_reset_to
from components import (
    ElevatorComponent,
    ArmComponent,
    WristComponent,
    IntakeComponent
)

from enum import Enum


class PieceLocations(Enum):
    HOME = 0
    CORAL_REEF_1 = 1
    CORAL_REEF_2 = 2 
    CORAL_REEF_3 = 3 
    CORAL_REEF_4 = 4 
    ALGAE_REEF_1 = 5
    ALGAE_REEF_2 = 6
    ALGAE_PROCESSOR = 7
    BARGE = 8
    HPS_LOADING = 9


class Manipulator(StateMachine):
    elevator: ElevatorComponent
    arm: ArmComponent
    wrist: WristComponent
    intake: IntakeComponent

    def place_coral_1(self):
        self.arm.target_pos = 1
        self.elevator.target_pos = 1
        self.wrist.target_pos = 1
    
    def place_coral_2(self):
        self.arm.target_pos = 2
        self.elevator.target_pos = 2
        self.wrist.target_pos = 2
    
    def place_coral_3(self):
        self.arm.target_pos = 3
        self.elevator.target_pos = 3
        self.wrist.target_pos = 3
    
    def place_coral_4(self):
        self.arm.target_pos = 4
        self.elevator.target_pos = 4
        self.wrist.target_pos = 4
    
    def place_algae_1(self):
        self.arm.target_pos = 5
        self.elevator.target_pos = 5
        self.wrist.target_pos = 5
    
    def place_algae_2(self):
        self.arm.target_pos = 6
        self.elevator.target_pos = 6
        self.wrist.target_pos = 6
    
    def place_algae_processor(self):
        self.arm.target_pos = 7
        self.elevator.target_pos = 7
        self.wrist.target_pos = 7

    def place_algae_barge(self):
        self.arm.target_pos = 8
        self.elevator.target_pos = 8
        self.wrist.target_pos = 8
    
    @state(first=True)
    def verify_clear(self, initial_call):
        arm_pos = self.arm.get_position()
        wrist_pos = self.wrist.get_position()
        elevator_pos = self.elevator.get_position()





