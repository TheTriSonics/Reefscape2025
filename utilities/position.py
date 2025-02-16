import math
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from utilities.game import field_flip_pose2d, field_flip_rotation2d
from choreo.trajectory import SwerveTrajectory as ChoreoSwerveTrajectory
from choreo.trajectory import SwerveSample as ChoreoSwerveSample


def reverse_choreo(traj: ChoreoSwerveTrajectory) -> ChoreoSwerveTrajectory:
    new_samples: list[ChoreoSwerveSample] = []
    total_time = traj.get_total_time()
    for orig_sample in reversed(traj.samples):
        samp = ChoreoSwerveSample(
            total_time - orig_sample.timestamp,
            orig_sample.x,
            orig_sample.y,
            orig_sample.heading,
            -orig_sample.vx,
            -orig_sample.vy,
            -orig_sample.heading,
            -orig_sample.ax,
            -orig_sample.ay,
            -orig_sample.omega,
            orig_sample.fx,
            orig_sample.fy,
        )
        new_samples.append(samp)
    new_traj = ChoreoSwerveTrajectory('reversed', new_samples, [], [])
    return new_traj


class Positions:

    @classmethod
    def auton_line_2(cls, is_red: bool) -> Pose2d:
        pose = Pose2d(7.536, 6.190, math.pi)
        if is_red:
            pose = field_flip_pose2d(pose)
        return pose
