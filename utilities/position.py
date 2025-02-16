import math
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from utilities.game import field_flip_pose2d, field_flip_rotation2d


class Positions:

    @classmethod
    def auton_line_2(cls, is_red: bool) -> Pose2d:
        pose = Pose2d(7.536, 6.190, math.pi)
        if is_red:
            pose = field_flip_pose2d(pose)
        return pose
            