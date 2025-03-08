import wpilib
from phoenix6.hardware import CANdi
from magicbot import tunable
from ids import CanId
from utilities import is_sim


class PhotoEyeComponent:

    candi = CANdi(CanId.CANDI.id, CanId.CANDI.bus)

    algae_held = tunable(False)
    coral_held = tunable(False)

    back_photoeye = tunable(False)
    front_photoeye = tunable(False)

    coral_false_at = -1.0
    algae_false_at = -1.0

    coral_true_at = -1.0
    algae_true_at = -1.0
    def execute(self):
        # Int values that we get back from the state().value.value are:
        # 0 = EYE_OPEN
        # 1 = We don't know when this would happen
        # 2 = EYE_BLOCKED
        now = wpilib.Timer.getFPGATimestamp()
        EYE_BLOCKED = 2
        if not is_sim():
            self.back_photoeye = self.candi.get_s1_state().value.value == EYE_BLOCKED
            self.front_photoeye = self.candi.get_s2_state().value.value == EYE_BLOCKED

        if self.back_photoeye is False and self.front_photoeye is False:
            if self.coral_false_at < 0:
                self.coral_false_at = now + 0.250
            elif self.coral_false_at < now:
                self.coral_false_at = -1
                self.coral_true_at = -1
                self.coral_held = False

        if self.back_photoeye is True or self.front_photoeye is True:
            if self.coral_true_at < 0:
                self.coral_true_at = now + 0.0
            elif self.coral_true_at < now:
                self.coral_true_at = -1
                self.coral_false_at = -1
                self.coral_held = True
