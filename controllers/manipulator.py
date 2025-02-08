from magicbot import StateMachine, state, timed_state, default_state, will_reset_to
import wpilib
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
    DRIVE_UP = 10


class Manipulator(StateMachine):
    elevator: ElevatorComponent
    arm: ArmComponent
    wrist: WristComponent
    intake: IntakeComponent
    
    target_score = 0
    go_score = False
    go_home_triggered = False
    algae_sequence = False
    coral_sequence = False
    algae_target_score = 0
    end_time = None

    def go_home(self):
        self.arm.target_pos = 0
        self.elevator.target_pos = 0
        self.wrist.target_pos = 0


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

    def go_hps_loading(self):
        self.arm.target_pos = 9
        self.elevator.target_pos = 9
        self.wrist.target_pos = 9

    def go_drive_up(self):
        self.arm.target_pos = 10
        self.elevator.target_pos = 10
        self.wrist.target_pos = 10

    def intake_algae(self):
        if self.intake.intake_loaded():
            self.intake.intake_off()
        else:
            self.intake.intake_rev()

    def eject_algae(self):
        self.end_time = wpilib.Timer.getFPGATimestamp() + 0.2
        if self.end_time is not None and wpilib.Timer.getFPGATimestamp() < self.end_time:
            self.intake.intake_fwd()
            return False
        else:
            self.end_time = None
            self.intake.intake_off()
            return True
        
    def intake_coral(self):
        if self.intake.intake_loaded():
            self.intake.intake_off()
        else:
            self.intake.intake_fwd()

    def eject_coral(self):
        self.end_time = wpilib.Timer.getFPGATimestamp() + 0.2
        if self.end_time is not None and wpilib.Timer.getFPGATimestamp() < self.end_time:
            self.intake.intake_fwd()
            return False
        else:
            self.end_time = None
            self.intake.intake_off()
            return True

    @state(first=True)
    def verify_clear(self, initial_call):
        arm_pos = self.arm.get_position()
        wrist_pos = self.wrist.get_position()
        elevator_pos = self.elevator.get_position()

    @state()
    def home(self, initial_call):
        if initial_call:
            self.algae_sequence = False
            self.coral_sequence = False
        self.go_home_triggered = False
        self.go_home() 
        if self.arm.at_goal() and self.elevator.at_goal() and self.wrist.at_goal():
            if self.intake.intake_loaded() and\
                (self.target_score == PieceLocations.CORAL_REEF_3 or
                 self.target_score == PieceLocations.CORAL_REEF_4):
                self.next_state(self.drive_up)
            elif (self.intake.intake_loaded() and
                  self.target_score == PieceLocations.CORAL_REEF_1 and
                    (self.go_score or self.coral_sequence)):
                    self.next_state(self.coral_reef_1)
            elif (self.intake.intake_loaded() and
                  self.target_score == PieceLocations.CORAL_REEF_2 and
                    (self.go_score or self.coral_sequence)):
                    self.next_state(self.coral_reef_2)
            elif (not self.intake.intake_loaded() and
                  self.coral_sequence):
                self.next_state(self.hps_loading)
            elif self.algae_sequence:
                self.next_state(self.algae_reef_1)
            
    @state()
    def drive_up(self, initial_call):
        if initial_call:
            self.algae_sequence = False
            self.coral_sequence = False
        self.go_drive_up()
        if self.arm.at_goal() and self.elevator.at_goal() and self.wrist.at_goal():
            if self.go_score or self.coral_sequence:
                if self.target_score == PieceLocations.CORAL_REEF_3:
                    self.next_state(self.coral_reef_3)
                if self.target_score == PieceLocations.CORAL_REEF_4:
                    self.next_state(self.coral_reef_4)
            if self.algae_sequence:
                if self.algae_target_score == PieceLocations.BARGE:
                    self.next_state(self.algae_barge)
                if self.algae_target_score == PieceLocations.ALGAE_PROCESSOR:
                    self.next_state(self.algae_processor)
        
    @state()
    def algae_barge(self, initial_call):
        if initial_call:
            self.algae_sequence = False
        self.coral_sequence = False
        self.place_algae_barge()
        if self.arm.at_goal() and self.elevator.at_goal() and self.wrist.at_goal():
            if self.algae_sequence:
                self.next_state(self.algae_eject)
    
    @state()
    def algae_processor(self, initial_call):
        if initial_call:
            self.algae_sequence = False
        self.coral_sequence = False
        self.place_algae_processor()
        if self.arm.at_goal() and self.elevator.at_goal() and self.wrist.at_goal():
            if self.algae_sequence:
                self.next_state(self.algae_eject) 

    @state()
    def algae_eject(self, initial_call):
        if initial_call:
            self.algae_sequence = False
        self.coral_sequence = False
        self.eject_algae()
        if self.eject_algae() and self.algae_sequence:
             self.next_state(self.home)


    @state()
    def hps_loading(self):
        self.go_hps_loading()
        if self.arm.at_goal() and self.elevator.at_goal() and self.wrist.at_goal():
            if self.intake.intake_loaded():
                self.next_state(self.home)

    @state()
    def coral_reef_1(self, initial_call):
        if initial_call:
            self.coral_sequence = False
        self.algae_sequence = False
        self.go_score = False
        self.place_coral_1()
        if self.arm.at_goal() and self.elevator.at_goal() and self.wrist.at_goal():
            if self.go_home_triggered:
                self.next_state(self.home)
            if self.coral_sequence:
                self.next_state(self.coral_eject)

    @state()
    def coral_reef_2(self, initial_call):
        if initial_call:
            self.coral_sequence = False
        self.algae_sequence = False
        self.go_score = False
        self.place_coral_2()
        if self.arm.at_goal() and self.elevator.at_goal() and self.wrist.at_goal():
            if self.go_home_triggered:
                self.next_state(self.home)
            if self.coral_sequence:
                self.next_state(self.coral_eject)

    @state()
    def coral_reef_3(self, initial_call):
        if initial_call:
            self.coral_sequence = False
        self.algae_sequence = False
        self.go_score = False
        self.place_coral_3()
        if self.arm.at_goal() and self.elevator.at_goal() and self.wrist.at_goal():
            if self.go_home_triggered:
                self.next_state(self.home)
            if self.coral_sequence:
                self.next_state(self.coral_eject)

    @state()
    def coral_reef_4(self, initial_call):
        if initial_call:
            self.coral_sequence = False
        self.algae_sequence = False
        self.go_score = False
        self.place_coral_4()
        if self.arm.at_goal() and self.elevator.at_goal() and self.wrist.at_goal():
            if self.go_home_triggered:
                self.next_state(self.home)
            if self.coral_sequence:
                self.next_state(self.coral_eject)

    @state()
    def coral_eject(self, initial_call):
        if initial_call:
            self.coral_sequence= False
        self.algae_sequence = False
        self.eject_coral()
        if self.eject_coral() and self.coral_sequence:
            self.next_state(self.home)

    @state()
    def algae_reef_1(self, initial_call):
        if initial_call:
            self.algae_sequence= False
        self.place_algae_1()
        if self.arm.at_goal() and self.elevator.at_goal() and self.wrist.at_goal():
            self.intake_algae()
            if self.algae_sequence:             
                if self.intake.intake_loaded():
                    self.next_state(self.drive_up)
                else:
                    self.next_state(self.algae_reef_2)

    @state()
    def algae_reef_2(self, initial_call):
        if initial_call:
            self.algae_sequence= False
        self.place_algae_2()
        if self.arm.at_goal() and self.elevator.at_goal() and self.wrist.at_goal():
            self.intake_algae()
            if self.algae_sequence:             
                if self.intake.intake_loaded():
                    self.next_state(self.drive_up)
                else:
                    self.next_state(self.algae_reef_1)
                    