from utilities.game import ManipLocations
from magicbot import will_reset_to, tunable
from controllers.manipulator import Manipulator 


class DebugPanel:
    go_home = tunable(False)
    go_coral_4 = tunable(False)
    go_coral_3_arm = tunable(False)
    go_coral_3 = tunable(False)
    go_coral_2 = tunable(False)
    go_coral_1 = tunable(False)
    go_algae_1 = tunable(False)
    go_algae_2 = tunable(False)
    go_algae_processor = tunable(False)
    go_algae_barge = tunable(False)

    manipulator: Manipulator

    def execute(self):
        if self.go_home:
            self.manipulator.request_location(ManipLocations.HOME)
            self.go_home = False
        if self.go_coral_4:
            self.manipulator.request_location(ManipLocations.CORAL_REEF_4)
            self.go_coral_4 = False
        if self.go_coral_3:
            self.manipulator.request_location(ManipLocations.CORAL_REEF_3)
            self.go_coral_3 = False
        if self.go_coral_3_arm:
            self.manipulator.request_location(ManipLocations.CORAL_REEF_3_ARM)
            self.go_coral_3_arm = False
        if self.go_coral_2:
            self.manipulator.request_location(ManipLocations.CORAL_REEF_2)
            self.go_coral_2 = False
        if self.go_coral_1:
            self.manipulator.request_location(ManipLocations.CORAL_REEF_1)
            self.go_coral_1 = False
