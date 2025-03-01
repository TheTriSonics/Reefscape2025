import wpilib
from phoenix6.hardware import CANdi
from magicbot import tunable
from ids import CanId

class PhotoEyeComponent:

    candi = CANdi(CanId.CANDI.id, CanId.CANDI.bus)

    back_photoeye = False
    front_photoeye = False

    def execute(self):
        self.back_photoeye = self.candi.get_s1_state()
        self.front_photoeye = self.candi.get_s2_state()
        # Read sensors and set tunables
        # TODO

        # For now we'll have the intake system take care of changing the
        # tunables


        pass

