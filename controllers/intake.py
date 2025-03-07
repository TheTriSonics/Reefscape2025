from magicbot import StateMachine, state
from components.intake import IntakeComponent
from components.photoeye import PhotoEyeComponent


class IntakeControl(StateMachine):
    intake: IntakeComponent
    photoeye: PhotoEyeComponent

    def __init__(self):
        self.coral_score_reverse = False
        pass

    def go_idle(self):
        self.next_state(self.idling)
        self.engage()

    def go_coral_score(self, reverse=False):
        self.coral_score_reverse = reverse
        if self.current_state != self.coral_score.name:
            self.next_state(self.coral_score)
            self.engage()

    def go_algae_score(self):
        self.next_state(self.algae_score)
        self.engage()

    def go_coral_intake(self):
        self.next_state(self.coral_intake)
        self.engage()

    def go_algae_intake(self):
        print('intake controller engaging algae_intake')
        self.next_state(self.algae_intake)
        self.engage()

    def go_algae_hold(self):
        self.next_state(self.algae_hold)
        self.engage()


    @state(first=True, must_finish=True)
    def idling(self):
        self.intake.intake_off()

    @state(must_finish=True)
    def coral_score(self, initial_call, state_tm):
        if self.coral_score_reverse:
            self.intake.score_coral_reverse()
        else:
            self.intake.score_coral()
        if state_tm > 0.10 and self.photoeye.coral_held is False:
            self.next_state_now(self.idling)

    @state(must_finish=True)
    def algae_score(self, initial_call, state_tm):
        self.intake.score_algae()
        # Just run the intake to score for a bit, then go to idle
        if state_tm > 2.0:
            self.intake.intake_off()
            self.next_state(self.idling)

    @state(must_finish=True)
    def coral_intake(self, initial_call, state_tm):
        self.intake.coral_in()    
        if self.photoeye.front_photoeye is True:
            self.intake.intake_off()
            self.next_state(self.idling)
    
    @state(must_finish=True)
    def algae_intake(self, initial_call, state_tm):
        self.intake.algae_in()    
        if self.photoeye.front_photoeye is True:
            self.next_state(self.idling)

    # We just stay in this state until something forces us out
    # We were exiting back to idilng when the photoeye would clear but
    # since there's some bounce in the system we'll get rid of that. The
    # operator will have to manually take us to the 'home' position' if
    # the algae falls out.
    @state(must_finish=True)
    def algae_hold(self):
        self.intake.algae_in()    

