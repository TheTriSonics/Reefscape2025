from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from magicbot import StateMachine, state
from components import DrivetrainComponent
from typing import Optional


class DriveToPosition(StateMachine):

    drivetrain: DrivetrainComponent


    def setup(self):
        self.config = TrajectoryConfig(1, 1)
        self.config.setKinematics(self.drivetrain.kinematics)
        self.trajectory: Optional[Trajectory] = None


    def move_to(self, goal_pose: Pose2d):
        current_pose = self.drivetrain.get_pose()
        waypoints = [current_pose, goal_pose]
        self.trajectory = TrajectoryGenerator.generateTrajectory(waypoints, self.config)
        self.engage()

    
    @state(first=True)
    def follow_trajectory(self, tm, state_tm):
        if self.trajectory:
            sample = self.trajectory.sample(state_tm)
            self.drivetrain.follow_trajectory(sample)

            if state_tm > self.trajectory.totalTime():
                self.next_state(self.finished)
                self.done()

    
    @state
    def finished(self):
        self.drivetrain.halt()
        self.trajectory = None
        self.done()


