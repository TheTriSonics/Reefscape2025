import math
import wpilib
from magicbot import StateMachine, state

from components import DrivetrainComponent

from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState, PathPlannerTrajectory
from pathplannerlib.config import RobotConfig
from wpimath.geometry import Pose2d, Rotation2d
import math


class DriveToPose(StateMachine):
    drivetrain: DrivetrainComponent
    trajectory: PathPlannerTrajectory | None = None
    field: wpilib.Field2d

    def __init__(self):

        self.ppconfig = RobotConfig.fromGUISettings()

    def generate_trajectory(self, current_pose: Pose2d, target_pose: Pose2d):
        waypoints = PathPlannerPath.waypointsFromPoses(
            [current_pose, target_pose]
        )
        constraints = PathConstraints(3.0, 3.0, 2 * math.pi, 4 * math.pi)
        # You can also use unlimited constraints, only limited by motor torque
        # and nominal battery voltage
        # constraints = PathConstraints.unlimitedConstraints(12.0)

        # Create the path using the waypoints created above
        path = PathPlannerPath(
            waypoints,
            constraints,
            None, # ideal start state -- ignored
            GoalEndState(0.0, target_pose.rotation())
        )
        return path.generateTrajectory(
            self.drivetrain.get_chassis_speeds(),
            self.drivetrain.get_pose().rotation(),
            self.ppconfig
        )

    def drive_to_pose(self, target_pose: Pose2d):
        current_pose = self.drivetrain.get_pose()
        self.trajectory = self.generate_trajectory(current_pose, target_pose)
        self.next_state(self.follow_trajectory)

    @state(first=True, must_finish=True)
    def follow_trajectory(self, state_tm):
        if self.trajectory is None:
            self.done()
        else:
            sample = self.trajectory.sample(state_tm)
            if sample:
                self.drivetrain.follow_pp_trajectory(sample)
