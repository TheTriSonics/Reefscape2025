from magicbot import StateMachine, state, tunable
from components.climber import ClimberComponent, ClimbDirection  


# Who climbs? Spiderman climbs.
class Spiderman(StateMachine):
    climber: ClimberComponent

    tweak_up = tunable(False)
    tweak_down = tunable(False)

    def __init__(self):
        pass

    def setup(self):
        pass

    def go_break_intake(self):
        self.engage()
        if self.current_state != self.break_intake.name:
            self.next_state(self.break_intake)

    @state(first=True, must_finish=True)
    def idling(self):
        pass

    @state(must_finish=True)
    def break_intake(self, initial_call, state_tm):
        self.climber.break_intake()
        if state_tm > 1.2:
            self.next_state(self.climb)

    @state(must_finish=True)
    def ready_to_climb(self, initial_call, state_tm):
        # Wait for some reason to climb?
        pass

    @state(must_finish=True)
    def climb(self, initial_call, state_tm):
        # Run the climber motors until we are fully extended
        self.climber.direction = ClimbDirection.CLIMB_UP
        if self.climber.get_position() >= self.climber.upper_limit:
            self.next_state(self.tweak_climb)
    

    @state(must_finish=True)
    def tweak_climb(self):
        if self.tweak_up:
            self.climber.direction = ClimbDirection.CLIMB_UP
        elif self.tweak_down:
            self.climber.direction = ClimbDirection.CLIMB_DOWN
        else:
            self.climber.direction = ClimbDirection.NONE
    
