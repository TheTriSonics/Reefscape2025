import math
import random
from wpimath.geometry import Pose2d, Translation2d, Rotation2d, Transform2d
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

    AUTON_LINE_OUR_CAGE_CENTER = Pose2d()
    AUTON_LINE_MID = Pose2d()
    AUTON_LINE_THEIR_CAGE_CENTER = Pose2d()

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
    def get_facepos(cls, face: str, left=False, right=False) -> Pose2d:
        if left:
            return getattr(cls, f"REEF_{face}_LEFT")
        elif right:
            return getattr(cls, f"REEF_{face}_RIGHT")
        return getattr(cls, f"REEF_{face}")

    @classmethod
    def identity_face(cls, pose: Pose2d) -> tuple[str | None, str | None]:
        for face in ["A", "B", "C", "D", "E", "F"]:
            for suffix in ["_LEFT", "", "_RIGHT"]:
                val = getattr(cls, f"REEF_{face}{suffix}")
                if pose == val:
                    return face, f'REEF_{face}{suffix}'
        return None, None

    @classmethod
    def is_reef_pose(cls, pose: Pose2d) -> bool:
        return pose in [
            cls.REEF_A, cls.REEF_A_LEFT, cls.REEF_A_RIGHT,
            cls.REEF_B, cls.REEF_B_LEFT, cls.REEF_B_RIGHT,
            cls.REEF_C, cls.REEF_C_LEFT, cls.REEF_C_RIGHT,
            cls.REEF_D, cls.REEF_D_LEFT, cls.REEF_D_RIGHT,
            cls.REEF_E, cls.REEF_E_LEFT, cls.REEF_E_RIGHT,
            cls.REEF_F, cls.REEF_F_LEFT, cls.REEF_F_RIGHT,
        ]

    # @classmethod
    # def update_dynamic_positions(cls, robot_pose: Pose2d):
    #     reef_tag_id, _ = Waypoints.closest_reef_tag_id(robot_pose)
    #     reef_start = Waypoints.get_tag_robot_away(reef_tag_id, face_at=True)
    #     cls.REEF_CLOSEST_LEFT = Waypoints.shift_reef_left(reef_start)
    #     cls.REEF_CLOSEST_RIGHT = Waypoints.shift_reef_right(reef_start)
    #     cls.REEF_CLOSEST = reef_start

    #     ps_tag_id, _ = Waypoints.closest_ps_tag_id(robot_pose)
    #     cls.PS_CLOSEST = Waypoints.get_tag_robot_away(
    #         ps_tag_id, face_at=False
    #     ).transformBy(Transform2d(Translation2d(-0.1, 0.000), Rotation2d(0)))


    # @classmethod
    # def update_alliance_positions(cls):
    #     for face in ['A', 'B', 'C', 'D', 'E', 'F']:
    #         tag_id = Waypoints.get_tag_id_from_letter(face, is_red())
    #         tag_pose = Waypoints.get_tag_robot_away(tag_id, face_at=True)
    #         setattr(cls, f'REEF_{face}_LEFT', Waypoints.shift_reef_left(tag_pose))
    #         setattr(cls, f'REEF_{face}_RIGHT', Waypoints.shift_reef_right(tag_pose))
    #         tag_pose = tag_pose.transformBy(Transform2d(Translation2d(0.10, 0),Rotation2d(0)))
    #         setattr(cls, f'REEF_{face}', tag_pose)
        
    #     processor_tag_id = 3 if is_red() else 16
    #     processor_pose = Waypoints.get_tag_robot_away(processor_tag_id, face_at=True)
    #     cls.PROCESSOR = processor_pose.transformBy(
    #         Transform2d(Translation2d(-1.0, 0.0), Rotation2d(0))
    #     )

    #     barge_tag_id = 5 if is_red() else 14
    #     barge_pose = Waypoints.get_tag_robot_away(barge_tag_id, face_at=True)
    #     cls.BARGE_CENTER = barge_pose
    #     cls.BARGE_LEFT = Waypoints.shift_barge_left(barge_pose)
    #     cls.BARGE_RIGHT = Waypoints.shift_barge_right(barge_pose)

    #     ps_left_tag_id = 1 if is_red() else 13
    #     ps_right_tag_id = 2 if is_red() else 12
    #     ps_left_pose = Waypoints.get_tag_robot_away(ps_left_tag_id, face_at=False)
    #     ps_right_pose = Waypoints.get_tag_robot_away(ps_right_tag_id, face_at=False)
    #     cls.PS_LEFT = ps_left_pose.transformBy(Transform2d(Translation2d(-0.1, 0.0), Rotation2d(0)))
    #     cls.PS_RIGHT = ps_right_pose.transformBy(Transform2d(Translation2d(-0.1, 0.0), Rotation2d(0)))

    #     cls.AUTON_LINE_OUR_CAGE_CENTER = cls.auton_our_cage_center(is_red())
    #     cls.AUTON_LINE_MID = cls.auton_line_center(is_red())
    #     cls.AUTON_LINE_THEIR_CAGE_CENTER = cls.auton_their_cage_center(is_red())

    #     # Default the dynamic ones! Just to be safe!
    #     cls.REEF_CLOSEST = cls.REEF_A
    #     cls.REEF_CLOSEST_LEFT = cls.REEF_A_LEFT
    #     cls.REEF_CLOSEST_RIGHT = cls.REEF_A_RIGHT

    #     cls.PS_CLOSEST = cls.PS_LEFT


    @classmethod
    def auton_our_cage_center(cls, is_red: bool) -> Pose2d:
        pose = Pose2d(7.536, 6.190, math.pi)
        if is_red:
            pose = field_flip_pose2d(pose)
        return pose

    @classmethod
    def auton_line_center(cls, is_red: bool) -> Pose2d:
        pose = Pose2d(7.574, 4.046, math.pi)
        if is_red:
            pose = field_flip_pose2d(pose)
        return pose

    @classmethod
    def auton_their_cage_center(cls, is_red: bool) -> Pose2d:
        pose = Pose2d(7.536, 1.908, math.pi)
        if is_red:
            pose = field_flip_pose2d(pose)
        return pose


