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

    def execute(self):
        # Int values that we get back from the state().value.value are:
        # 0 = EYE_OPEN
        # 1 = We don't know when this would happen
        # 2 = EYE_BLOCKED
        EYE_BLOCKED = 2
        if not is_sim():
            self.back_photoeye = self.candi.get_s1_state().value.value == EYE_BLOCKED
            self.front_photoeye = self.candi.get_s2_state().value.value == EYE_BLOCKED
        # Not the real logic, but we can get started with this.
        self.coral_held = self.back_photoeye
        # I kind of want to debounce this or make sure we're actually trying
        # to pick up the algae but not sure where/how to do that yet.
        self.algae_held = self.front_photoeye
