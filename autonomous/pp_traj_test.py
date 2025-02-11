import math
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d
from magicbot import AutonomousStateMachine, timed_state, state, feedback, tunable

from utilities.game import is_red

from controllers.manipulator import Manipulator

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from controllers.drive_to_pose import DriveToPose

pb = SmartDashboard.putBoolean
pn = SmartDashboard.putNumber
ps = SmartDashboard.putString


class AutonPPTrajTest(AutonomousStateMachine):
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    drive_to_pose: DriveToPose

    MODE_NAME = 'Test PathPlanner Trajectories'
    DEFAULT = False

    def __init__(self):
        return

    @state(first=True, must_finish=True)
    def run_traj(self, tm, state_tm, initial_call):
        if initial_call:
            current_pose = self.drivetrain.get_pose()
            target_pose = Pose2d(
                current_pose.X() + 2,
                current_pose.Y(),
                current_pose.rotation()
            )
            print('auton hitting drive to pose controller')
            self.drive_to_pose.drive_to_pose(target_pose)
            self.drive_to_pose.engage()  



