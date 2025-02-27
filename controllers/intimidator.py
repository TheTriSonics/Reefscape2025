
""""
This is a state machine that will control the manipulator.
"""
import enum
import math
import ntcore
import wpilib
from pathlib import Path
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d
from magicbot import StateMachine, state, tunable, feedback, will_reset_to

from components.drivetrain import DrivetrainComponent
from choreo import load_swerve_trajectory
from choreo.trajectory import SwerveTrajectory as ChoreoSwerveTrajectory

from utilities.waypoints import Waypoints
from utilities.position import reverse_choreo
from utilities.game import ManipLocations, ManipLocation, GamePieces, is_auton, is_red
from utilities.position import Positions
from utilities import pn, ps, pb


def get_point_on_circle(center_x, center_y, radius, angle_degrees):
    # Convert angle from degrees to radians
    angle_radians = math.radians(angle_degrees)
    
    # Calculate x and y coordinates using parametric equations of a circle
    x = center_x + (radius * math.cos(angle_radians))
    y = center_y + (radius * math.sin(angle_radians))
    heading_radians = math.atan2(center_y - y, center_x - x)
    
    return (x, y, heading_radians)


def calculate_target(robot_x, robot_y, future_x, future_y):
    # Calculate the vector from robot to future pose
    vector_x = future_x - robot_x
    vector_y = future_y - robot_y
    
    # Calculate the length of this vector
    vector_length = (vector_x**2 + vector_y**2)**0.5
    if vector_length == 0:
        return future_x, future_y
    
    # Normalize the vector (make it unit length)
    unit_vector_x = vector_x / vector_length
    unit_vector_y = vector_y / vector_length
    
    # Calculate target point by extending 2 units from future pose
    target_x = future_x + (2 * unit_vector_x)
    target_y = future_y + (2 * unit_vector_y)
    
    return target_x, target_y


class Intimidator(StateMachine):
    drivetrain: DrivetrainComponent

    stick_x, stick_y, stick_rotation = 0, 0, 0

    strafe_distance = tunable(-1.0)
    strafe_to_face = tunable('A')

    max_start_dist_error = tunable(1.0)
    max_end_dist_error = tunable(1.0)
    dist_to_direct_drive = tunable(0.5)

    def __init__(self):
        self.trajectory: ChoreoSwerveTrajectory | None = None
        self.target_pose: Pose2d
        self.target_pose_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/intimidator/target_pose", Pose2d)
            .publish()
        )

        self.swoop_traj_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructArrayTopic("/components/intimidator/swoop_traj", Pose2d)
            .publish()
        )

        self.traj_desired_pose = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/intimidator/desired_pose", Pose2d)
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
        self.trajectories: list[ChoreoSwerveTrajectory] = []

    def load_trajectories(self):
        # iterate through every file in deploy/choreo that matches *.traj
        for traj_file in Path("deploy/choreo").rglob("*.traj"):
            print(f"Loading trajectory {traj_file.stem}")
            traj = load_swerve_trajectory(traj_file.stem)
            rtraj = reverse_choreo(traj)
            self.trajectories.append(traj)
            self.trajectories.append(rtraj)

    def _clear_traj_pub(self):
        self.swoop_traj_pub.set([])

    def _send_strafe_circle_poses(self, reef_pose: Pose2d, distance: float):
        strafe_poses = []
        for i in range(0, 370, 10):
            x, y, rad = get_point_on_circle(
                reef_pose.X(), reef_pose.Y(), self.strafe_distance, i
            )
            pose = Pose2d(x, y, Rotation2d(rad))
            strafe_poses.append(pose)
        self.strafe_positions_pub.set(strafe_poses)

    def setup(self):
        self.target_pose = Pose2d()
        self.load_trajectories()

    def at_position(self, tolerance=0.08) -> bool:
        pose = self.drivetrain.get_pose()
        return pose.relativeTo(self.target_pose).translation().norm() < tolerance

    def set_stick_values(self, x, y, rot):
        self.stick_x = x
        self.stick_y = y
        self.stick_rotation = rot

    def set_target_pose(self, pose):
        self.target_pose = pose

    def go_drive_field(self):
        if self.current_state != self.drive_field.name:
            self.next_state_now(self.drive_field)
            self.engage()

    @state(first=True, must_finish=True)
    def drive_field(self):
        self._clear_traj_pub()
        # Put any heading snap-to logic somewhere in here
        x = -self.stick_x if is_red() else self.stick_x
        y = -self.stick_y if is_red() else self.stick_y
        self.drivetrain.drive_field(x, y, self.stick_rotation)
    
    def go_drive_local(self):
        self.next_state_now(self.drive_local)

    def go_drive_nearest_ps(self):
        pose = self.drivetrain.get_pose()
        tag_id, dist = Waypoints.closest_ps_tag_id(pose)
        self.go_drive_ps(tag_id)

    def go_drive_ps(self, tag_id):
        final_pose = (
            Waypoints.get_tag_robot_away(tag_id)
        )
        # self.next_state_now(self.drive_to_pose)
        self.go_drive_swoop(final_pose)

    def go_drive_processor(self):
        self.go_drive_swoop(Positions.PROCESSOR)

    def go_lock_reef(self, shift_left=False, shift_right=False):
        final_pose = Positions.REEF_CLOSEST
        if shift_left:
            final_pose = Positions.REEF_CLOSEST_LEFT
        elif shift_right:
            final_pose = Positions.REEF_CLOSEST_RIGHT
        self.go_drive_swoop(final_pose)

    @state(must_finish=True)
    def drive_local(self):
        self._clear_traj_pub()
        # Put any heading snap-to logic somewhere in here
        self.drivetrain.drive_local(self.stick_x, self.stick_y, self.stick_rotation)

    @state(must_finish=True)
    def drive_to_pose(self):
        self._clear_traj_pub()
        self.drivetrain.drive_to_pose(self.target_pose)

    def go_drive_strafe(self):
        self._clear_traj_pub()
        if self.current_state != self.drive_strafe.name:
            self.next_state_now(self.drive_strafe)
            self.engage()

    def go_drive_strafe_fixed(self, distance):
        self.strafe_distance = distance
        self.next_state_now(self.drive_strafe_fixed)
        self.engage()

    def go_drive_strafe_reef_face(self, face_name: str):
        if face_name not in ['A', 'B', 'C', 'D', 'E', 'F']:
            raise ValueError('Invalid face name, must be A-F, capital.')
        if self.strafe_to_face != face_name:
            self.strafe_to_face = face_name 
            self.next_state_now(self.drive_strafe_reef_face)
        if self.current_state != self.drive_strafe_reef_face.name:
            self.strafe_to_face = face_name 
            self.next_state_now(self.drive_strafe_reef_face)
        self.engage()

    def go_drive_swoop(self, target_pose: Pose2d) -> None:
        if self.target_pose != target_pose:
            self.target_pose = target_pose
            self.next_state_now(self.drive_swoop)
        if self.current_state != self.drive_swoop.name:
            self.next_state_now(self.drive_swoop)
        self.engage()

    @state(must_finish=True)
    def drive_swoop(self, initial_call, state_tm):
        # A 'swoop' drive will try and find a Choreo trajectory that roughly
        # starts and ends around where the robot is starting and ending.
        # If a trajectory is found we'll follow along that one until we get
        # "close enough" to the final endpoint and then revert to using
        # 'drive to positon' in the drivetrain to lock in the final pose.
        curr_pose = self.drivetrain.get_pose()
        if initial_call:
            # Look up what path to take, a trajectory to load
            # Declare scores as a dict with str as the key type and float as the value
            # This will map each trajectory name (string) to a score (float)
            scores: dict[str, float] = {}
            scores = {}
            self.target_pose_pub.set(self.target_pose)
            for traj in self.trajectories:
                end_pose = traj.get_final_pose(is_red())
                start_pose = traj.get_initial_pose(is_red())
                assert end_pose, f'No end pose found for trajectory {traj.name}'
                assert start_pose, f'No start pose found for trajectory {traj.name}'
                # Calcuate the difference between the end pose of the trajectory
                # and our destination.
                ediff = end_pose.relativeTo(self.target_pose).translation().norm()
                # Likewise, check how far away we are from the start pose
                sdiff = start_pose.relativeTo(curr_pose).translation().norm() 
                # If we're too far from the start or end then we're going to
                # reject this trajectory for consideration.
                if sdiff > self.max_start_dist_error or ediff > self.max_end_dist_error:
                    continue  # Skip this; not worth considering
                # Now create a score entry for this valid trajectory; we just
                # add in the end distance different, start distance difference,
                # and the total time that the trajectory takes to run. The last
                # one picks the quicker in the caes of a complete tie
                scores[traj.name] = ediff + sdiff + traj.get_total_time()

            import json
            print('Scores:', json.dumps(scores, indent=4, sort_keys=True))
            # Find the entry in scores that has the lowest score 
            if len(scores) > 0:
                best_traj = min(scores, key=lambda k: float(scores[k]))
                print('Best trajectory:', best_traj)
                # Find trajectory in self.trajectories that has a name that matches best_traj
                traj = next(
                    traj for traj in self.trajectories if traj.name == best_traj
                )
                assert traj
                self.trajectory = traj
                # Now let's publish the poses of the trajectory
                traj_poses: list[Pose2d] = []
                for x in range(0, int(traj.get_total_time() * 1000), 100):
                    sample = traj.sample_at(x / 1000, is_red())
                    assert sample
                    traj_poses.append(Pose2d(sample.x, sample.y, sample.heading))
                self.swoop_traj_pub.set(traj_poses)
            else:
                self.trajectory = None
                self.swoop_traj_pub.set([])

        dist_to_end_pose = (
            curr_pose.relativeTo(self.target_pose).translation().norm()
        )
        if self.trajectory is None:
            # If there's no trajectory found we just drive right to the target
            # pose
            self.drivetrain.drive_to_pose(self.target_pose)
        elif (
            self.trajectory.get_total_time() < state_tm
            or (dist_to_end_pose < self.dist_to_direct_drive
                and state_tm > self.trajectory.get_total_time() * 0.90)
        ):
            # The trajectory has expired, so, drive to the final pose
            self.drivetrain.drive_to_pose(self.target_pose, aggressive=True)
        else:
            sample = self.trajectory.sample_at(state_tm, is_red())
            assert sample
            self.traj_desired_pose.set(sample.get_pose())
            self.drivetrain.follow_trajectory(sample)

    @state(must_finish=True)
    def drive_strafe(self, initial_call):
        robot_pose = self.drivetrain.get_pose()
        # This is a Pose2d that represents the center of the reef of our own
        # alliance
        rc = Waypoints.get_reef_center(is_red())
        if initial_call:
            # The first time we enter this state we'll get our distance away
            # from the center of the reef, cap it between a min and max value,
            # then 'lock in' that distance as how far we want to stay from the
            # center.
            # If the driver wanted to modify this an input could be detected
            # in robot.py and then 'punched' into this strafe_distance value
            # to change behavior, or call go_drive_strafe_fixed() with it
            dist = robot_pose.translation().distance(rc.translation())
            self.strafe_distance = min(max([dist, 1.5]), 3.3)

        # Now get the robot's current angle on the circle; atan2 is the version
        # of arctan that takes sign into account.
        angle_radians = math.atan2(robot_pose.Y() - rc.Y(),
                                   robot_pose.X() - rc.X())

        # 'stick rotation' will now control the angle on the circle
        # These units are all sorts of wrong, but seems to drive right - Justin
        # The idea is to figure out where we want to be on this strafing circle
        # at the end of the robot execution period (0.02 seconds)
        circum = 2 * math.pi * self.strafe_distance
        speed = self.stick_rotation * 5
        rad_per_sec = 2 * math.pi / (circum / speed) if speed != 0 else 0
        rad_per_sec = min(rad_per_sec, 18)
        pn('rad_per_sec', rad_per_sec)
        angle_radians += rad_per_sec * 0.02
        angle_degrees = math.degrees(angle_radians)
        x, y, rad = get_point_on_circle(
            rc.X(), rc.Y(), self.strafe_distance, angle_degrees 
        )

        # Publish poses out to help debug 
        self.reef_center_pose_pub.set(rc)
        self._send_strafe_circle_poses(rc, self.strafe_distance)
        self.strafe_next_pub.set(Pose2d(x, y, Rotation2d(rad)))

        self.drivetrain.drive_to_position(x, y, rad, aggressive=True)

    @state(must_finish=True)
    def drive_strafe_fixed(self, initial_call):
        robot_pose = self.drivetrain.get_pose()
        rc = Waypoints.get_reef_center(is_red())
        self.reef_center_pose_pub.set(rc)
        if initial_call:
            strafe_poses = []
            for i in range(0, 360, 10):
                x, y, rad = get_point_on_circle(rc.translation().X(), rc.translation().Y(), self.strafe_distance, i)
                pose = Pose2d(Translation2d(x, y), Rotation2d(rad))
                strafe_poses.append(pose)
            self.strafe_positions_pub.set(strafe_poses)
        # These units are all sorts of wrong, but seems to drive right - Justin
        circum = 2 * math.pi * self.strafe_distance
        speed = 40  # m/s
        time = circum / speed
        rad_per_sec = 2 * math.pi / time
        # Now get my current angle on the circle
        dx = robot_pose.X() - rc.X()
        dy = robot_pose.Y() - rc.Y()
        # Calculate angle using atan2 and add in where we are on the circle
        # in one more robot cycle at the above 'speed' in m/s
        angle_radians = math.atan2(dy, dx) + (rad_per_sec * 0.02)
        angle_degrees = math.degrees(angle_radians)
        x, y, rad = get_point_on_circle(
            rc.X(), rc.Y(), self.strafe_distance, angle_degrees 
        )
        self.strafe_next_pub.set(Pose2d(Translation2d(x, y), Rotation2d(rad)))
        self.drivetrain.drive_to_position(x, y, rad, aggressive=True)

    @state(must_finish=True)
    def drive_strafe_reef_face(self, initial_call):
        if initial_call:
            robot_pose = self.drivetrain.get_pose()
            rc = Waypoints.get_reef_center(is_red())
            # Get the distance between robot pose and rc
            dist = robot_pose.translation().distance(rc.translation())
            # Cap the distance between 1.5 and 3.3
            self.strafe_distance = min(max([dist, 1.5]), 3.3)

        robot_pose = self.drivetrain.get_pose()
        rc = Waypoints.get_reef_center(is_red())
        self.reef_center_pose_pub.set(rc)
        if initial_call:
            strafe_poses = []
            for i in range(0, 360, 10):
                x, y, rad = get_point_on_circle(rc.translation().X(), rc.translation().Y(), self.strafe_distance, i)
                pose = Pose2d(Translation2d(x, y), Rotation2d(rad))
                strafe_poses.append(pose)
            self.strafe_positions_pub.set(strafe_poses)
        # These units are all sorts of wrong, but seems to drive right - Justin
        circum = 2 * math.pi * self.strafe_distance
        speed = 40  # m/s
        time = circum / speed
        rad_per_sec = 2 * math.pi / time
        # Now get my current angle on the circle
        dx = robot_pose.X() - rc.X()
        dy = robot_pose.Y() - rc.Y()
        # Calculate angle using atan2 and add in where we are on the circle
        # in one more robot cycle at the above 'speed' in m/s
        angle_radians = math.atan2(dy, dx)
        pn = wpilib.SmartDashboard.putNumber
        target_radians = 0.0
        match self.strafe_to_face:
            case 'D':
                target_radians = 0
            case 'E':
                target_radians = math.pi / 3
            case 'F':
                target_radians = 2 * math.pi / 3
            case 'A':
                target_radians = math.pi
            case 'B':
                target_radians = 4 * math.pi / 3
            case 'C':
                target_radians = 5 * math.pi / 3
        pn('angle_radians', angle_radians)
        pn('target_radians', target_radians)
        # Determine if the shortest distance to the target is clockwise or counter-clockwise
        is_ccw = (target_radians - angle_radians) % (2 * math.pi) > math.pi
        if is_ccw:
            rad_per_sec = -rad_per_sec
        angle_radians += rad_per_sec * 0.02
        aggro = True
        if math.degrees(abs(angle_radians - target_radians)) < 10:
            angle_radians = target_radians
            agro = False
        angle_degrees = math.degrees(angle_radians)
        # Once we're within X degrees of the face call it good.
        if math.degrees(abs(target_radians - angle_radians)) < 0.5:
            self.next_state(self.completed)
        x, y, rad = get_point_on_circle(
            rc.X(), rc.Y(), self.strafe_distance, angle_degrees 
        )
        self.strafe_next_pub.set(Pose2d(Translation2d(x, y), Rotation2d(rad)))
        self.drivetrain.drive_to_position(x, y, rad, aggressive=aggro)

    @state(must_finish=True)
    def completed(self, initial_call):
        pass