from magicbot import AutonomousStateMachine, timed_state

from components.drivetrain import DrivetrainComponent

class MoveOffLine(AutonomousStateMachine):

    chassis: DrivetrainComponent

    MODE_NAME = 'Just do it'

    def __init__(self):
        return

    @timed_state(first=True, duration=1.0, must_finish=True)
    def drive(self):
        self.chassis.drive_local(2, 0, 0)
