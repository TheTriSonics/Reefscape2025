import math
from functools import cache
from wpimath.geometry import Pose2d, Transform2d, Translation2d, Rotation2d
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from generated.tuner_constants import TunerConstants

apriltags = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)

bumper_width = 0.30
robot_y_offset = TunerConstants._front_left_y_pos + bumper_width

reef_tags = [6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22]
ps_tags = [1, 2, 12, 13]
processor_tags = [3, 16]
barge_tags = [4, 5, 14, 15]


class Waypoints:

    def __init__(self):
        pass

    @classmethod
    @cache
    def get_tag_pose(cls, tag_id) -> Pose2d:
        pose = apriltags.getTagPose(tag_id)
        if pose is None:
            return Pose2d()
        return pose.toPose2d()

    @classmethod
    @cache
    def get_tag_letter(cls, tag_id) -> str:
        match tag_id:
            # Start with red first
            case 10:
                return 'A'
            case 9:
                return 'B'
            case 8:
                return 'C'
            case 7:
                return 'D'
            case 6:
                return 'E'
            case 11:
                return 'F'
            case 21:
                return 'A'
            case 22:
                return 'B'
            case 17:
                return 'C'
            case 18:
                return 'D'
            case 19:
                return 'E'
            case 20:
                return 'F'

        return 'Z'

    @classmethod
    @cache
    def get_tag_id_from_letter(cls, letter, is_red) -> int:
        if is_red:
            match letter:
                case 'A':
                    return 10
                case 'B':
                    return 9
                case 'C':
                    return 8
                case 'D':
                    return 7
                case 'E':
                    return 6
                case 'F':
                    return 11
        else:
            match letter:
                case 'A':
                    return 21
                case 'B':
                    return 22
                case 'C':
                    return 17
                case 'D':
                    return 18
                case 'E':
                    return 19
                case 'F':
                    return 20
        return -1

    @classmethod
    def get_reef_center(cls, is_red: bool) -> Pose2d:
        reef_A_id = cls.get_tag_id_from_letter('A', is_red)
        reef_D_id = cls.get_tag_id_from_letter('D', is_red)
        reef_A = cls.get_tag_pose(reef_A_id)
        reef_D = cls.get_tag_pose(reef_D_id)
        # Now find the midpoint between A andD 
        avg_x = (reef_A.X() + reef_D.X()) / 2
        avg_y = (reef_A.Y() + reef_D.Y()) / 2
        return Pose2d(avg_x, avg_y, Rotation2d())

    @classmethod
    def get_radians_to_reef_center(cls, pose: Pose2d, is_red: bool) -> float:
        # Get angle between the two poses
        center = cls.get_reef_center(is_red)
        # Now find the angle that points from the x,y of the robot pose to the x,y of the reef center
        robot_X = pose.X()
        robot_Y = pose.Y()
        center_X = center.X()
        center_Y = center.Y()
        return math.atan2(center_Y - robot_Y, center_X - robot_X)



    @classmethod
    def get_distance_to_reef_center(cls, pose: Pose2d, is_red: bool) -> float:
        # Get distance betwen the two poses
        center = cls.get_reef_center(is_red)
        return pose.translation().distance(center.translation())

    @classmethod
    def find_reef_path(cls, start, end) -> list[int]:
        """
        Find shortest path between two points in a tag_ids array.
        The array represents points arranged in a tag_ids clockwise.
        """
        # Filter out the red or blue tags depending on where we are
        if start <= 11:
            tag_ids = [tid for tid in reef_tags if tid <= 11]
        else:
            tag_ids = [tid for tid in reef_tags if tid >= 12]
        # Find positions in array
        start_idx = tag_ids.index(start)
        end_idx = tag_ids.index(end)
        
        # Calculate clockwise and counterclockwise distances
        length = len(tag_ids)
        clockwise_dist = (end_idx - start_idx) % length
        counter_dist = (start_idx - end_idx) % length
        
        # Get the shortest path(s)
        if clockwise_dist < counter_dist:
            # Clockwise is shorter
            path = []
            idx = start_idx
            while idx != end_idx:
                path.append(tag_ids[idx])
                idx = (idx + 1) % length
            path.append(tag_ids[end_idx])
            return path
        elif counter_dist <= clockwise_dist:
            # Counterclockwise is shorter
            path = []
            idx = start_idx
            while idx != end_idx:
                path.append(tag_ids[idx])
                idx = (idx - 1) % length
            path.append(tag_ids[end_idx])
            return path
        else:
            return []

    @classmethod
    @cache
    def get_tag_meter_away(cls, tag_id, meters=1.0) -> Pose2d:
        pose = cls.get_tag_pose(tag_id)
        meter_offset = Transform2d(Translation2d(meters, 0), Rotation2d(0))
        meter_pose = pose.transformBy(meter_offset)
        return meter_pose

    @classmethod
    def distance_from_tag(cls, pose: Pose2d, tag_id) -> float:
        tag_pose = cls.get_tag_pose(tag_id)
        return pose.translation().distance(tag_pose.translation())
        
    @classmethod
    def closest_reef_tag_id(cls, pose: Pose2d) -> tuple[int, float]:
        reef_distances = [
            pose.translation().distance(tpose.translation())
            for tpose in [cls.get_tag_pose(tag) for tag in reef_tags]
        ]
        dist = min(reef_distances)
        return reef_tags[reef_distances.index(dist)], dist

    @classmethod
    def closest_ps_tag_id(cls, pose: Pose2d) -> tuple[int, float]:
        ps_distances = [
            pose.translation().distance(tpose.translation())
            for tpose in [cls.get_tag_pose(tag) for tag in ps_tags]
        ]
        dist = min(ps_distances)
        return ps_tags[ps_distances.index(dist)], dist

    @classmethod
    def closest_processor_tag_id(cls, pose: Pose2d) -> tuple[int, float]:
        processor_distances = [
            pose.translation().distance(tpose.translation())
            for tpose in [cls.get_tag_pose(tag) for tag in processor_tags]
        ]
        dist = min(processor_distances)
        return processor_tags[processor_distances.index(dist)], dist

    @classmethod
    def closest_barge_tag_id(cls, pose: Pose2d) -> tuple[int, float]:
        barge_distances = [
            pose.translation().distance(tpose.translation())
            for tpose in [cls.get_tag_pose(tag) for tag in barge_tags]
        ]
        dist = min(barge_distances)
        return barge_tags[barge_distances.index(dist)], dist

    @classmethod
    @cache
    def get_tag_robot_away(cls, tag_id, face_at=False) -> Pose2d:
        pose = cls.get_tag_pose(tag_id)
        rot = 0 if face_at is False else math.pi
        robot_offset = Transform2d(Translation2d(robot_y_offset, 0), Rotation2d(rot))
        robot_pose = pose.transformBy(robot_offset)
        return robot_pose

    @classmethod
    def shift_reef_left(cls, pose: Pose2d) -> Pose2d:
        offset = Transform2d(Translation2d(0, 0.165), Rotation2d(0))
        return pose.transformBy(offset)

    @classmethod
    def shift_reef_right(cls, pose: Pose2d) -> Pose2d:
        offset = Transform2d(Translation2d(0, -0.165), Rotation2d(0))
        return pose.transformBy(offset)