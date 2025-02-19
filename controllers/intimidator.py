
""""
This is a state machine that will control the manipulator.
"""
import enum
import math
import ntcore
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d
from magicbot import StateMachine, state, tunable, feedback, will_reset_to

from components.drivetrain import DrivetrainComponent

from utilities.waypoints import Waypoints
from utilities.game import ManipLocations, ManipLocation, GamePieces, is_auton, is_red


def get_point_on_circle(center_x, center_y, radius, angle_degrees):
    # Convert angle from degrees to radians
    angle_radians = math.radians(angle_degrees)
    
    # Calculate x and y coordinates using parametric equations of a circle
    x = center_x + (radius * math.cos(angle_radians))
    y = center_y + (radius * math.sin(angle_radians))
    heading_radians = math.atan2(center_y - y, center_x - x)
    
    return (x, y, heading_radians)


class Intimidator(StateMachine):
    drivetrain: DrivetrainComponent

    stick_x, stick_y, stick_rotation = 0, 0, 0

    strafe_distance = tunable(-1.0)

    def __init__(self):
        self.target_pose: Pose2d
        self.target_pose_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/intimidator/target_pose", Pose2d)
            .publish()
        )

        self.reef_center_pose_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/intimidator/Reef_pose", Pose2d)
            .publish()
        )
        self.strafe_positions_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructArrayTopic("/components/intimidator/strafes", Pose2d)
            .publish()
        )

        self.strafe_next_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/intimidator/strafe_next", Pose2d)
            .publish()
        )
        pass

    def setup(self):
        pass

    def set_stick_values(self, x, y, rot):
        self.stick_x = x
        self.stick_y = y
        self.stick_rotation = rot

    def set_target_pose(self, pose):
        self.target_pose = pose

    def go_drive_field(self):
        self.next_state_now(self.drive_field)
        self.engage()

    @state(first=True, must_finish=True)
    def drive_field(self):
        # Put any heading snap-to logic somewhere in here
        x = -self.stick_x if is_red() else self.stick_x
        y = -self.stick_y if is_red() else self.stick_y
        self.drivetrain.drive_field(x, y, self.stick_rotation)
    
    def go_drive_local(self):
        self.next_state_now(self.drive_local)

    def go_drive_nearest_ps(self):
        pose = self.drivetrain.get_pose()
        tag_id, dist = Waypoints.closest_ps_tag_id(pose)
        final_pose = (
            Waypoints.get_tag_robot_away(tag_id)
        )
        self.target_pose_pub.set(final_pose)
        self.target_pose = final_pose
        self.next_state_now(self.drive_to_pose)
        self.engage()

    def go_drive_processor(self):
        pose = self.drivetrain.get_pose()
        tag_id, dist = Waypoints.closest_processor_tag_id(pose)
        final_pose = Waypoints.get_tag_robot_away(tag_id).transformBy(
            Transform2d(Translation2d(0, 0), Rotation2d(math.pi))
        )
        self.target_pose = final_pose
        self.target_pose_pub.set(final_pose)
        self.next_state_now(self.drive_to_pose)
        self.engage()

    def go_lock_reef(self, shift_left=False, shift_right=False):
        pose = self.drivetrain.get_pose()
        reef_tag_id, dist = Waypoints.closest_reef_tag_id(pose)
        final_pose = (
            Waypoints.get_tag_robot_away(reef_tag_id)
            .transformBy(Transform2d(Translation2d(0, 0), Rotation2d(math.pi)))
        )
        if shift_left:
            final_pose = Waypoints.shift_reef_left(final_pose)
        elif shift_right:
            final_pose = Waypoints.shift_reef_right(final_pose)
        self.target_pose = final_pose
        self.target_pose_pub.set(final_pose)
        self.next_state_now(self.drive_to_pose)
        self.engage()

    @state(must_finish=True)
    def drive_local(self):
        # Put any heading snap-to logic somewhere in here
        self.drivetrain.drive_local(self.stick_x, self.stick_y, self.stick_rotation)

    @state(must_finish=True)
    def drive_to_pose(self):
        self.drivetrain.drive_to_pose(self.target_pose)

    def go_drive_strafe(self):
        if self.current_state != self.drive_strafe.name:
            self.next_state_now(self.drive_strafe)
            self.engage()

    @state(must_finish=True)
    def drive_strafe(self, initial_call):
        if initial_call:
            robot_pose = self.drivetrain.get_pose()
            rc = Waypoints.get_reef_center(is_red())
            # Get the distance between robot pose and rc
            dist = robot_pose.translation().distance(rc.translation())
            self.strafe_distance = max([dist, 1.5])

        robot_pose = self.drivetrain.get_pose()
        rc = Waypoints.get_reef_center(is_red())
        self.reef_center_pose_pub.set(rc)
        strafe_poses = []
        for i in range(0, 360+45, 10):
            x, y, rad = get_point_on_circle(rc.translation().X(), rc.translation().Y(), self.strafe_distance, i)
            pose = Pose2d(Translation2d(x, y), Rotation2d(rad))
            strafe_poses.append(pose)
        self.strafe_positions_pub.set(strafe_poses)
        # Now get my current angle on the circle
        dx = robot_pose.X() - rc.X()
        dy = robot_pose.Y() - rc.Y()
        # Calculate angle using atan2
        angle_radians = math.atan2(dy, dx)
        angle_degrees = math.degrees(angle_radians)
        # Rotation will now control the angle on the circle
        deg_change = -self.stick_rotation * 4
        max_deg_change = 25
        if deg_change < -max_deg_change:
            deg_change = -max_deg_change
        elif deg_change > max_deg_change:
            deg_change = max_deg_change 

        angle_degrees -= deg_change
        # Now check if it is less than -2 or 2 and cap it within that range
        x, y, rad = get_point_on_circle(
            rc.X(), rc.Y(), self.strafe_distance, angle_degrees 
        )
        self.strafe_next_pub.set(Pose2d(Translation2d(x, y), Rotation2d(rad)))
        self.drivetrain.drive_to_position(x, y, rad)