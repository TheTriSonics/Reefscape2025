from magicbot import AutonomousStateMachine, timed_state

from components.drivetrain import DrivetrainComponent


class MoveOffLine(AutonomousStateMachine):

    drivetrain: DrivetrainComponent

    MODE_NAME = 'Move Off Line'

    def __init__(self):
        return

    @timed_state(first=True, duration=1.0, must_finish=True)
    def drive(self):
        self.drivetrain.drive_local(2, 0, 0)
