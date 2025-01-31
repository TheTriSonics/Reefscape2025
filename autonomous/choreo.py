from magicbot import AutonomousStateMachine, timed_state

from utilities.game import is_red

from components.drivetrain import DrivetrainComponent
from components.gyro import Gyro
from choreo import load_swerve_trajectory  # type: ignore

class MoveOffLine():

    chassis: DrivetrainComponent
    gyro: Gyro

    MODE_NAME = 'Circle Reef'

    pose_set = False
    selected_alliance = None

    def __init__(self):
        self.trajectory = load_swerve_trajectory('CircleReef')
        return

    def on_disable(self):
        return

    def set_initial_pose(self) -> None:
        # No need to set the pose twice!
        alliance = 'red' if is_red() else 'blue'
        if alliance is not self.selected_alliance:
            self.pose_set = False

        if self.pose_set is True:
            return

        initial_pose = self.trajectory.get_initial_pose(is_red())
        if initial_pose is not None:
            print('setting pose in Circle Reef')
            self.chassis.set_pose(initial_pose)
            self.selected_alliance = alliance
            # self.gyro.reset_heading(initial_pose.rotation().degrees())
            self.pose_set = True

    def on_enable(self):
        pass

    def on_iteration(self, tm):
        sample = self.trajectory.sample_at(tm, is_red())
        if sample:
            self.chassis.follow_trajectory(sample)
