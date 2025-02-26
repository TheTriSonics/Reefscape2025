import math
import random
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from utilities.game import field_flip_pose2d, field_flip_rotation2d, is_red
from utilities.waypoints import Waypoints
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
    new_traj = ChoreoSwerveTrajectory(f'{traj.name} - reversed', new_samples, [], [])
    return new_traj


class Positions:
    REEF_CLOSEST = Pose2d()
    REEF_CLOSEST_LEFT = Pose2d()
    REEF_CLOSEST_RIGHT = Pose2d()

    REEF_A = Pose2d()
    REEF_A_LEFT = Pose2d()
    REEF_A_RIGHT = Pose2d()

    REEF_B = Pose2d()
    REEF_B_LEFT = Pose2d()
    REEF_B_RIGHT = Pose2d()

    REEF_C = Pose2d()
    REEF_C_LEFT = Pose2d()
    REEF_C_RIGHT = Pose2d()

    REEF_D = Pose2d()
    REEF_D_LEFT = Pose2d()
    REEF_D_RIGHT = Pose2d()

    REEF_E = Pose2d()
    REEF_E_LEFT = Pose2d()
    REEF_E_RIGHT = Pose2d()

    REEF_F = Pose2d()
    REEF_F_LEFT = Pose2d()
    REEF_F_RIGHT = Pose2d()

    AUTON_LINE_LEFT = Pose2d()
    AUTON_LINE_CENTER = Pose2d()
    AUTON_LINE_RIGHT = Pose2d()

    PS_CLOSEST = Pose2d()
    PS_LEFT = Pose2d()
    PS_RIGHT = Pose2d()

    PROCESSOR = Pose2d()

    BARGE_LEFT = Pose2d()
    BARGE_CENTER = Pose2d()
    BARGE_RIGHT = Pose2d()

    @classmethod
    def random_position(cls) -> Pose2d:
        positions = [
            value
            for name, value in vars(cls).items()
            if name.isupper()
            and isinstance(value, Pose2d)
            and not name.startswith("AUTON")
        ]
        return random.choice(positions)

    @classmethod
    def update_dynamic_positions(cls, robot_pose: Pose2d):
        reef_tag_id, _ = Waypoints.closest_reef_tag_id(robot_pose)
        cls.REEF_CLOSEST = Waypoints.get_tag_robot_away(reef_tag_id, face_at=True)
        cls.REEF_CLOSEST_LEFT = Waypoints.shift_reef_left(cls.REEF_CLOSEST)
        cls.REEF_CLOSEST_RIGHT = Waypoints.shift_reef_right(cls.REEF_CLOSEST)

        ps_tag_id, _ = Waypoints.closest_ps_tag_id(robot_pose)
        cls.PS_CLOSEST = Waypoints.get_tag_robot_away(ps_tag_id, face_at=False)


    @classmethod
    def update_alliance_positions(cls):
        for face in ['A', 'B', 'C', 'D', 'E', 'F']:
            tag_id = Waypoints.get_tag_id_from_letter(face, is_red())
            tag_pose = Waypoints.get_tag_robot_away(tag_id, face_at=True)
            setattr(cls, f'REEF_{face}', tag_pose)
            setattr(cls, f'REEF_{face}_LEFT', Waypoints.shift_reef_left(tag_pose))
            setattr(cls, f'REEF_{face}_RIGHT', Waypoints.shift_reef_right(tag_pose))
        
        processor_tag_id = 3 if is_red() else 16
        processor_pose = Waypoints.get_tag_robot_away(processor_tag_id, face_at=True)
        cls.PROCESSOR = processor_pose

        barge_tag_id = 5 if is_red() else 14
        barge_pose = Waypoints.get_tag_robot_away(barge_tag_id, face_at=True)
        cls.BARGE_CENTER = barge_pose
        cls.BARGE_LEFT = Waypoints.shift_barge_left(barge_pose)
        cls.BARGE_RIGHT = Waypoints.shift_barge_right(barge_pose)

        ps_left_tag_id = 1 if is_red() else 13
        ps_right_tag_id = 2 if is_red() else 12
        ps_left_pose = Waypoints.get_tag_robot_away(ps_left_tag_id, face_at=False)
        ps_right_pose = Waypoints.get_tag_robot_away(ps_right_tag_id, face_at=False)
        cls.PS_LEFT = ps_left_pose
        cls.PS_RIGHT = ps_right_pose

        cls.AUTON_LINE_CENTER = cls.auton_line_2(is_red())
        cls.AUTON_LINE_LEFT = Waypoints.shift_auton_left(cls.AUTON_LINE_CENTER)
        cls.AUTON_LINE_RIGHT = Waypoints.shift_auton_right(cls.AUTON_LINE_CENTER)

        # Default the dynamic ones! Just to be safe!
        cls.REEF_CLOSEST = cls.REEF_A
        cls.REEF_CLOSEST_LEFT = cls.REEF_A_LEFT
        cls.REEF_CLOSEST_RIGHT = cls.REEF_A_RIGHT

        cls.PS_CLOSEST = cls.PS_LEFT


    @classmethod
    def auton_line_2(cls, is_red: bool) -> Pose2d:
        pose = Pose2d(7.536, 6.190, math.pi)
        if is_red:
            pose = field_flip_pose2d(pose)
        return pose

