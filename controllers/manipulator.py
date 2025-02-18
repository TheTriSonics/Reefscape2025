
""""
This is a state machine that will control the manipulator.
"""
import enum
from magicbot import StateMachine, state, tunable, feedback

from components.intake import IntakeComponent
from components.photoeye import PhotoEyeComponent
from components.arm import ArmComponent
from components.elevator import ElevatorComponent
from components.wrist import WristComponent


class GamePieces(enum.Enum):
    CORAL = enum.auto()
    ALGAE = enum.auto()


class Location:
    """
    This Location class provides a handy wrapper to the positions of the
    components we need to move to.
    """
    wrist_pos: float
    arm_pos: float
    elevator_pos: float

    def __init__(self, elevator, arm, wrist):
        self.elevator_pos = elevator
        self.arm_pos = arm
        self.wrist_pos = wrist

    def __repr__(self):
        """ 
        This is what's used to turn the object into a string; handy if you
        want to print it out for debugging, or put it on a smart dashboard
        """
        return f'Location(elevator={self.elevator_pos}, arm={self.arm_pos}, wrist={self.wrist_pos})'
    

class Locations:
    """
    And we can define what is basicaly another enumeration, but with a Location
    object for a datatype, not an integer.
    """
    # Order of params is elevator, arm, wrist, just as in the Location's
    # __init__ method
    HOME = Location(0, -80, 135) 
    CORAL_REEF_1 = Location(0, -60, 75)
    CORAL_REEF_2 = Location(0, -20, -12)
    CORAL_REEF_3 = Location(0, 35, -52)
    CORAL_REEF_4 = Location(12, 50, -130) 
    ALGAE_REEF_1 = Location(5, -10, 20)
    ALGAE_REEF_2 = Location(5, -10, 20)

    PROCESSOR = Location(10, -60, 100)
    BARGE = Location(50, 50, 90)


class Manipulator(StateMachine):
    wrist: WristComponent
    arm: ArmComponent
    elevator: ElevatorComponent
    intake: IntakeComponent
    photoeye: PhotoEyeComponent

    operator_advance = tunable(False)
    # JJB: Not sure if this intentional on MagicBot's part, but if we make
    # a tunable with this name the actual state value comes back with no
    # code required on our part! It's like magic!
    current_state = tunable('home')
    
    game_piece_mode: GamePieces = GamePieces.CORAL

    # Create some default targets for the robot. The operator can change these
    # over in robot.py with their controller.
    coral_scoring_target = Locations.CORAL_REEF_4
    algae_scoring_target = Locations.BARGE
    algae_intake_target = Locations.ALGAE_REEF_1
    
    # This is where the system will try and drive itself to at any given time
    # You do have to call drive_to_setpoints to actually make it move
    _target_location: Location = Locations.HOME

    # This gets called by the MagicBot framework when the system is enabled
    def on_enable(self):
        print('Manipulator enabled')
        # Wherever we're at when enabled we'll just assume we should stay at
        self._target_location = Location(
            self.wrist.get_position(),
            self.arm.get_position(),
            self.elevator.get_position()
        )

    # This begins a sections of methods meant to be exposed to the operator
    # to control the manipulator. Buttons and such can be mapped to these to
    # configure how it operates.

    def request_advance(self):
        print('advance requested by operator')
        self.operator_advance = True

    def coral_mode(self):
        self.game_piece_mode = GamePieces.CORAL
        self.go_home()

    def algae_mode(self):
        self.game_piece_mode = GamePieces.ALGAE
        self.go_home()

    def go_home(self):
        # Set the necessary targets for each component
        self.request_location(Locations.HOME)
        self.intake.intake_off()
        self.next_state_now(self.idling)
    
    def set_coral_level1(self):
        self.coral_scoring_target = Locations.CORAL_REEF_1
        self.intake.at_height = 1

    def set_coral_level2(self):
        self.coral_scoring_target = Locations.CORAL_REEF_2
        self.intake.at_height = 2

    def set_coral_level3(self):
        self.coral_scoring_target = Locations.CORAL_REEF_3
        self.intake.at_height = 3

    def set_coral_level4(self):
        self.coral_scoring_target = Locations.CORAL_REEF_4
        self.intake.at_height = 4
    
    def set_algae_level1(self):
        self.algae_intake_target = Locations.ALGAE_REEF_1

    def set_algae_level2(self):
        self.algae_intake_target = Locations.ALGAE_REEF_2
    
    def set_algae_processor(self):
        self.algae_scoring_target = Locations.PROCESSOR

    def set_algae_barge(self):
        self.algae_scoring_target = Locations.BARGE

    # That's the end of the operator interface portion

    # Now some methods that the states within the system will use as helpers
    
    def request_location(self, location: Location):
        self._target_location = location
        self.wrist.target_pos = self._target_location.wrist_pos
        self.arm.target_pos = self._target_location.arm_pos
        self.elevator.target_pos = self._target_location.elevator_pos

    @feedback
    def at_position_wrist(self) -> bool:
        return self.wrist.at_goal()
    @feedback
    def at_position_arm(self) -> bool:
        return self.arm.at_goal()
    @feedback
    def at_position_elevator(self) -> bool:
        return self.elevator.at_goal()

    # Check to see if the system is at the target position, or close enough
    # with some deadbanding. We'll leave the 'close enough' up to the
    # individual components to decide since they know their own units and
    # are responsible for getting to the target.
    @feedback
    def at_position(self) -> bool:
        return self.at_position_wrist() and self.at_position_arm() and self.at_position_elevator()
    
    # That's the end of the helper methods and from here down we have the
    # various states of the state machine itself.
   
    # We'll start off idle; do nothing  until the operator requests something
    @state(must_finish=True, first=True)
    def idling(self, initial_call):
        if initial_call:
            self.operator_advance = False
        # TODO: Put a photo eye condition here to jump to intake if
        # the eye is triggered
        if self.operator_advance:
            self.operator_advance = False
            if self.photoeye.coral_held:
                self.next_state(self.coral_in_system)
            elif self.photoeye.algae_held:
                # Do something with that
                pass
            else:
                if self.game_piece_mode == GamePieces.CORAL:
                    self.next_state(self.coral_intake)
                elif self.game_piece_mode == GamePieces.ALGAE:
                    self.next_state(self.algae_intake)
    
    @state(must_finish=True)
    def coral_intake(self, state_tm, initial_call):
        self.intake.coral_in()
        if self.photoeye.coral_held:
            self.intake.intake_off()
            self.next_state(self.coral_in_system)

    @state(must_finish=True)
    def coral_in_system(self, state_tm, initial_call):
        # Wait here until the operator wants to get into scoring position
        if initial_call:
            # We don't want the operator to spam the advance button and advance
            # to this step
            self.operator_advance = False
        if self.operator_advance:
            self.operator_advance = False
            self.next_state(self.coral_prepare_score)

    @feedback
    def opadvance(self) -> bool:
        return self.operator_advance
    
    @state(must_finish=True)
    def coral_prepare_score(self, initial_call, state_tm):
        if initial_call:
            # We don't want the operator to spam the advance button and advance
            # to this step
            self.operator_advance = False
            self.request_location(self.coral_scoring_target)

        # The operator could change the target value while we're in this state
        # so check for that!
        if self._target_location != self.coral_scoring_target:
            self.request_location(self.coral_scoring_target)

        # Here we can check if we're at the position or if we've been
        # waiting too long and we should just move on, like maybe we just can't
        # quite get to the right position, but we've got to try something
        if self.opadvance() and (self.at_position()):
            self.operator_advance = False
            self.next_state(self.coral_score)

    # JJB: I'm not thrilled with the names of these states, coral_score and
    # coral_scored are too similar, but they make sense.
    @state(must_finish=True)
    def coral_score(self, state_tm, initial_call):
        # Let's score a coral!
        if initial_call:
            self.intake.score_coral()
        # For now I'm just putting in a timer to simulate the time it takes
        # to score a coral. This should be replaced with a sensor but keep
        # the timer as a backup. I'm staring to like this pattern of using
        # the state_tm value instead of the @timed_state decorator's duration
        # parameter and the next_state.

        if state_tm > 1.0 and self.photoeye.coral_held is False:
            self.intake.intake_off()

        if self.operator_advance and self.photoeye.coral_held is False:
            self.operator_advance = False
            self.intake.intake_off()
            self.next_state(self.coral_scored)

    # NOTE: This step might not really be needed, we could return back
    # to the home position when the scoring state knows the coral has ejected
    @state(must_finish=True)
    def coral_scored(self, initial_call, state_tm):
        if initial_call:
            # What do we do after we score a coral?
            # Send ourself back into 'home' mode
            self.intake.intake_off()
            self.request_location(Locations.HOME)
        # Now ask the system to start moving. When it arrives at the HOME
        # location it'll again wait for the operator to advance

        # Wait for the lifts to get back to home before we move on
        # to the idling state where we wait on user input to begin intake
        if self.at_position() or state_tm > 2.0:
            self.next_state(self.idling)

    @state(must_finish=True)
    def algae_intake(self, state_tm, initial_call):
        self.intake.coral_in()
        if self.operator_advance and state_tm > 0.5:
            self.operator_advance = False
            # TODO: Finish out algae
            # self.next_state(self.algae_in_system)