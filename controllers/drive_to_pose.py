import math
import wpilib
from magicbot import StateMachine, state

from wpimath.geometry import Pose2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, Trajectory
from wpimath.trajectory.constraint import CentripetalAccelerationConstraint

from components import DrivetrainComponent


class DriveToPose(StateMachine):
    drivetrain: DrivetrainComponent
    trajectory: Trajectory | None = None
    field: wpilib.Field2d

    def generate_trajectory(self, current_pose: Pose2d, target_pose: Pose2d):
        print('Generating trajectory')
        print(f'Current pose: {current_pose}')
        print(f'Target pose: {target_pose}')
        config = TrajectoryConfig(maxVelocity=5, maxAcceleration=10)
        # Add centripetal acceleration constraint for smoother turns
        config.addConstraint(CentripetalAccelerationConstraint(8.0))
        # Configure for swerve drive (holonomic)
        config.setKinematics(self.drivetrain.kinematics)
        
        # Generate trajectory
        traj = TrajectoryGenerator.generateTrajectory(
            current_pose,  # Starting pose
            [],  # No intermediate waypoints, this is where we could put in a 1 meter out waypoint to line up
            target_pose,  # Ending pose
            config,
        )
        return traj

    def drive_to_pose(self, target_pose: Pose2d):
        current_pose = self.drivetrain.get_pose()
        self.trajectory = self.generate_trajectory(current_pose, target_pose)
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)
        self.field.getObject('WPI Trajectory').setTrajectory(self.trajectory)
        self.next_state(self.follow_trajectory)

    @state(first=True, must_finish=True)
    def follow_trajectory(self, state_tm):
        if self.trajectory is None:
            self.done()
        else:
            sample = self.trajectory.sample(state_tm)
            if sample:
                self.drivetrain.follow_wpi_trajectory(sample)
