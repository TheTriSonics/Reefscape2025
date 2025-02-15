from magicbot import tunable


class PhotoEyeComponent:

    coral_chute = tunable(False)
    algae_chute = tunable(False)
    coral_held = tunable(False)
    algae_held = tunable(False)

    def execute(self):
        # Read sensors and set tunables
        pass

