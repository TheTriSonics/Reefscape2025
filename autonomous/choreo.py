from magicbot import AutonomousStateMachine, timed_state

from utilities.game import is_red

from components.drivetrain import DrivetrainComponent
from components.gyro import Gyro
from choreo import load_swerve_trajectory  # type: ignore

class MoveOffLine():

    chassis: DrivetrainComponent
    gyro: Gyro

    MODE_NAME = 'Circle Reef'

    def __init__(self):
        return

    def on_disable(self):
        return

    def on_enable(self):
        self.trajectory = load_swerve_trajectory('CircleReef')
        initial_pose = self.trajectory.get_initial_pose(is_red())
        if initial_pose is not None:
            self.chassis.set_pose(initial_pose)
            self.gyro.reset_heading(initial_pose.rotation().degrees())

    def on_iteration(self, tm):
        sample = self.trajectory.sample_at(tm, is_red())
        if sample:
            self.chassis.follow_trajectory(sample)
