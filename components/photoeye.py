import wpilib
from phoenix6.hardware import CANdi
from magicbot import tunable
from ids import CanId


class PhotoEyeComponent:

    candi = CANdi(CanId.CANDI.id, CanId.CANDI.bus)

    algae_held = tunable(False)
    coral_held = tunable(False)

    back_photoeye = tunable(False)
    front_photoeye = tunable(False)

    def execute(self):
        # Int values that we get back from the state().value.value are:
        # 0 = EYE_OPEN
        # 1 = We don't know when this would happen
        # 2 = EYE_BLOCKED
        EYE_BLOCKED = 2
        self.back_photoeye = self.candi.get_s1_state().value.value == EYE_BLOCKED
        self.front_photoeye = self.candi.get_s2_state().value.value == EYE_BLOCKED
        # Not the real logic, but we can get started with this.
        self.coral_held = self.front_photoeye
