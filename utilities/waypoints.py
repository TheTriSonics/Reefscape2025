import math
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


def get_tag_pose(tag_id) -> Pose2d:
    pose = apriltags.getTagPose(tag_id)
    if pose is None:
        return Pose2d()
    return pose.toPose2d()


def get_tag_letter(tag_id) -> str:
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


def get_tag_id_from_letter(letter, is_red) -> int:
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


def find_reef_path(start, end) -> list[int]:
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


def get_tag_meter_away(tag_id, meters=1.0) -> Pose2d:
    pose = get_tag_pose(tag_id)
    meter_offset = Transform2d(Translation2d(meters, 0), Rotation2d(0))
    meter_pose = pose.transformBy(meter_offset)
    return meter_pose


def distance_from_tag(pose: Pose2d, tag_id) -> float:
    tag_pose = get_tag_pose(tag_id)
    return pose.translation().distance(tag_pose.translation())
    

def closest_reef_tag_id(pose: Pose2d) -> tuple[int, float]:
    reef_distances = [
        pose.translation().distance(tpose.translation())
        for tpose in [get_tag_pose(tag) for tag in reef_tags]
    ]
    dist = min(reef_distances)
    return reef_tags[reef_distances.index(dist)], dist


def closest_ps_tag_id(pose: Pose2d) -> tuple[int, float]:
    ps_distances = [
        pose.translation().distance(tpose.translation())
        for tpose in [get_tag_pose(tag) for tag in ps_tags]
    ]
    dist = min(ps_distances)
    return ps_tags[ps_distances.index(dist)], dist


def closest_processor_tag_id(pose: Pose2d) -> tuple[int, float]:
    processor_distances = [
        pose.translation().distance(tpose.translation())
        for tpose in [get_tag_pose(tag) for tag in processor_tags]
    ]
    dist = min(processor_distances)
    return processor_tags[processor_distances.index(dist)], dist


def closest_barge_tag_id(pose: Pose2d) -> tuple[int, float]:
    barge_distances = [
        pose.translation().distance(tpose.translation())
        for tpose in [get_tag_pose(tag) for tag in barge_tags]
    ]
    dist = min(barge_distances)
    return barge_tags[barge_distances.index(dist)], dist


def get_tag_robot_away(tag_id, face_at=False) -> Pose2d:
    pose = get_tag_pose(tag_id)
    rot = 0 if face_at is False else math.pi
    robot_offset = Transform2d(Translation2d(robot_y_offset, 0), Rotation2d(rot))
    robot_pose = pose.transformBy(robot_offset)
    return robot_pose


def shift_reef_left(pose: Pose2d) -> Pose2d:
    offset = Transform2d(Translation2d(0, 0.2), Rotation2d(0))
    return pose.transformBy(offset)


def shift_reef_right(pose: Pose2d) -> Pose2d:
    offset = Transform2d(Translation2d(0, -0.25), Rotation2d(0))
    return pose.transformBy(offset)