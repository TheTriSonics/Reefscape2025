from magicbot import StateMachine, state
from components.intake import IntakeComponent
from components.photoeye import PhotoEyeComponent


class IntakeControl(StateMachine):
    intake: IntakeComponent
    photoeye: PhotoEyeComponent

    def __init__(self):
        pass

    def go_idle(self):
        self.next_state(self.idling)
        self.engage()

    def go_coral_score(self):
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
        self.intake.score_coral()
        if self.photoeye.coral_held is False:
            self.next_state(self.idling)

    @state(must_finish=True)
    def algae_score(self, initial_call, state_tm):
        self.intake.score_algae()

        if self.photoeye.front_photoeye is True:
            self.score_algae_off_at = state_tm + 0.25

        if self.photoeye.front_photoeye is False and self.score_algae_off_at < state_tm:
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

    @state(must_finish=True)
    def algae_hold(self):
        self.intake.algae_in()    
        if self.photoeye.front_photoeye is False:
            self.next_state(self.idling)

