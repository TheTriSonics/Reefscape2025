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
        print('intake controller engaging coral_intake')
        self.next_state(self.coral_intake)
        self.engage()

    def go_algae_intake(self):
        print('intake controller engaging algae_intake')
        self.next_state(self.algae_intake)
        self.engage()


    @state(first=True, must_finish=True)
    def idling(self):
        self.intake.intake_off()

    @state(must_finish=True)
    def coral_score(self, initial_call, state_tm):
        self.intake.score_coral()

        if self.photoeye.coral_held is True:
            self.score_coral_off_at = state_tm + 1.0

        if self.photoeye.coral_held is False and self.score_coral_off_at < state_tm:
            self.next_state(self.idling)

    @state(must_finish=True)
    def algae_score(self, initial_call, state_tm):
        self.intake.score_algae()

        if self.photoeye.algae_held is True:
            self.score_algae_off_at = state_tm + 1.0

        if self.photoeye.algae_held is False and self.score_algae_off_at < state_tm:
            self.next_state(self.idling)

    @state(must_finish=True)
    def coral_intake(self, initial_call, state_tm):
        self.intake.coral_in()    

        if self.photoeye.coral_held is False:
            self.intake_coral_off_at = state_tm + 1.0

        if self.photoeye.coral_held is True and self.intake_coral_off_at < state_tm:
            self.next_state(self.idling)
    
    @state(must_finish=True)
    def algae_intake(self, initial_call, state_tm):
        self.intake.algae_in()    

        if self.photoeye.algae_held is False:
            self.intake_algae_off_at = state_tm + 1.0

        if self.photoeye.algae_held is True and self.intake_algae_off_at < state_tm:
            self.next_state(self.idling)

    @state(must_finish=True)
    def score_algae(self, initial_call, state_tm):
        if initial_call:
            self.intake.score_algae()
            pass

        if self.photoeye.algae_held is True:
            self.score_algae_off_at = state_tm + 1.0

        if self.photoeye.algae_held is False and self.score_algae_off_at < state_tm:
            self.next_state(self.idling)
