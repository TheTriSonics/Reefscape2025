
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
from pathplannerlib.path import (
    PathPlannerPath,
    PathPlannerTrajectory,
    PathPlannerTrajectoryState,
    PathConstraints,
    GoalEndState,
    IdealStartingState,
    ConstraintsZone,
)
from pathplannerlib.config import RobotConfig
from components.drivetrain import DrivetrainComponent
from controllers.manipulator import Manipulator
from choreo import load_swerve_trajectory
from choreo.trajectory import SwerveTrajectory as ChoreoSwerveTrajectory

from utilities.waypoints import Waypoints
from utilities.position import reverse_choreo
from utilities.game import ManipLocations, ManipLocation, GamePieces, is_auton, is_red, field_flip_pose2d, field_flip_translation2d, field_flip_rotation2d, is_sim
from utilities.position import Positions
from utilities import pn, ps, pb


import time
from collections.abc import Iterator
from contextlib import contextmanager

@contextmanager
def time_it() -> Iterator[None]:
    tic: float = time.perf_counter()
    try:
        yield
    finally:
        toc: float = time.perf_counter()
        print(f"Computation time = {1000*(toc - tic):.3f}ms")


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
    manipulator: Manipulator

    stick_x, stick_y, stick_rotation = 0, 0, 0

    strafe_distance = tunable(-1.0)
    strafe_to_face = tunable('A')

    max_start_dist_error = tunable(1.0)
    max_end_dist_error = tunable(1.0)
    dist_to_direct_drive = tunable(0.5)

    trajectories: list[ChoreoSwerveTrajectory] = []
    waypoints: list[list[Pose2d]] = [[]]
    alliance_loaded = None

    def __init__(self):
        self.choreo_trajectory: ChoreoSwerveTrajectory | None = None
        self.pp_trajectory: PathPlannerTrajectory | None = None 
        self.pp_norm = PathConstraints(3.0, 5.0, 4 * math.tau, math.tau)
        self.pp_config = RobotConfig.fromGUISettings()
        self.target_pose: Pose2d
        self.drive_aggressive = False
        self.pp_traj_calc_ms_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getDoubleTopic("/components/intimidator/pp_traj_calc_ms")
            .publish()
        )

        self.target_pose_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/intimidator/target_pose", Pose2d)
            .publish()
        )

        self.choreo_traj_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructArrayTopic("/components/intimidator/choreo_swoop_traj", Pose2d)
            .publish()
        )

        self.pp_traj_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructArrayTopic("/components/intimidator/pp_swoop_traj", Pose2d)
            .publish()
        )

        self.pp_waypoints_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructArrayTopic("/components/intimidator/pp_swoop_waypoints", Pose2d)
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

        self.coral_level_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getIntegerTopic("/components/intimidator/coral_level")
            .publish()
        )

    @classmethod
    def load_trajectories(cls):
        if cls.alliance_loaded == is_red():
            return
        cls.alliance_loaded = is_red()
        # iterate through every file in deploy/choreo that matches *.traj
        for traj_file in Path("deploy/choreo").rglob("*.traj"):
            print(f"Loading trajectory {traj_file.stem}")
            traj = load_swerve_trajectory(traj_file.stem)
            rtraj = reverse_choreo(traj)
            cls.trajectories.append(traj)
            cls.trajectories.append(rtraj)
            # Ok let's pull the raw waypoints out now.
            import json
            waypoint_poses = []
            # Always get the blue side. We can flip to red later if needed
            rc = Waypoints.get_reef_center(False)
            with open(traj_file) as f:
                obj = json.load(f)
                waypoints = obj['snapshot']["waypoints"]
                # Iterate through every waypoint in the trajectory, but also grab
                # the next one at the same time
                for wp, next_wp in zip(waypoints, waypoints[1:]):
                    x = wp['x']
                    y = wp['y']
                    next_x = next_wp['x']
                    next_y = next_wp['y']
                    # The current waypoint should point at the next one
                    heading = math.atan2(next_y-y, next_x-x)
                    # This might be wrong on the blue side!!!
                    rot = Rotation2d(heading + math.pi)
                    pose = Pose2d(x, y, rot)
                    if is_red():
                        pose = field_flip_pose2d(pose)
                    waypoint_poses.append(pose)
                cls.waypoints.append(waypoint_poses)
                cls.waypoints.append(list(reversed(waypoint_poses)))

    def _clear_traj_pub(self):
        self.choreo_traj_pub.set([])
        self.pp_traj_pub.set([])

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

    @state(first=True, must_finish=True)
    def drive_field(self, initial_call):
        if initial_call:
            self._clear_traj_pub()
        # Put any heading snap-to logic somewhere in here
        x = -self.stick_x if is_red() else self.stick_x
        y = -self.stick_y if is_red() else self.stick_y
        self.drivetrain.drive_field(x, y, self.stick_rotation)
    
    def go_drive_local(self):
        if self.current_state != self.drive_local.name:
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
        lvl = self.manipulator.get_coral_scoring_level()
        self.coral_level_pub.set(lvl)
        if shift_left:
            final_pose = Positions.REEF_CLOSEST_LEFT_CLOSE if lvl in [3, 4] else Positions.REEF_CLOSEST_LEFT
        elif shift_right:
            final_pose = Positions.REEF_CLOSEST_RIGHT_CLOSE if lvl in [3, 4] else Positions.REEF_CLOSEST_RIGHT
        self.go_drive_swoop(final_pose)

    @state(must_finish=True)
    def drive_local(self, initial_call):
        if initial_call:
            self._clear_traj_pub()
        # Put any heading snap-to logic somewhere in here
        self.drivetrain.drive_local(self.stick_x, self.stick_y, self.stick_rotation)

    def go_drive_pose(self, pose: Pose2d, aggressive=False):
        self.target_pose = pose
        self.drive_aggressive = aggressive
        if self.current_state != self.drive_to_pose.name:
            self.next_state_now(self.drive_to_pose)

    @state(must_finish=True)
    def drive_to_pose(self, initial_call):
        if initial_call:
            self._clear_traj_pub()
        self.drivetrain.drive_to_pose(self.target_pose, aggressive=True)

    def go_drive_swoop(self, target_pose: Pose2d) -> None:
        if self.target_pose != target_pose:
            self.target_pose = target_pose
            self.next_state_now(self.drive_swoop)
        if self.current_state not in [self.drive_swoop.name, self.follow_pp.name]:
            self.next_state_now(self.drive_swoop)
        self.engage()

    @state(must_finish=True)
    def follow_choreo(self, initial_call, state_tm):
        traj = self.choreo_trajectory
        assert traj
        choreo_final = traj.get_final_pose(is_red())
        assert choreo_final
        if initial_call:
            traj_poses: list[Pose2d] = []
            for x in range(0, int(traj.get_total_time() * 1000), 100):
                sample = traj.sample_at(x / 1000, is_red())
                assert sample
                traj_poses.append(Pose2d(sample.x, sample.y, sample.heading))
            self.choreo_traj_pub.set(traj_poses)
            self.pp_traj_pub.set([])
        sample = traj.sample_at(state_tm, is_red())
        assert sample
        self.traj_desired_pose.set(sample.get_pose())
        self.drivetrain.follow_trajectory_choreo(sample)
        # We didn't make it there in time so kick the robot into motion again
        if state_tm > traj.get_total_time() * 1.1:
            self.go_drive_swoop(self.target_pose)
        tolerance = 0.15 if is_sim() else 0.040
        if self.at_position(tolerance=tolerance):
            self.next_state(self.completed)

    def find_choreo_trajectory(self, curr_pose: Pose2d, target_pose: Pose2d):
        scores: dict[str, float] = {}
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
            if (sdiff > 0.30 or ediff > 0.1):
                continue  # Skip this; not worth considering
            # Now create a score entry for this valid trajectory; we just
            # add in the end distance different, start distance difference,
            # and the total time that the trajectory takes to run. The last
            # one picks the quicker in the caes of a complete tie
            scores[traj.name] = ediff + sdiff + traj.get_total_time() * 10

        import json
        # Find the entry in scores that has the lowest score 
        if len(scores) > 0:
            best_traj = min(scores, key=lambda k: float(scores[k]))
            print('Winning trajectory:', best_traj)
            # Find trajectory in self.trajectories that has a name that matches best_traj
            traj = next(
                traj for traj in self.trajectories if traj.name == best_traj
            )
            assert traj
            return traj
        else:
            return None
    
    def find_choreo_waypoints(self, curr_pose: Pose2d, target_pose: Pose2d):
        self.target_pose_pub.set(self.target_pose)
        winner = None
        winner_score = math.inf
        for wp in self.waypoints:
            if len(wp) < 2:
                continue
            # If we're too far from the start or end then we're going to
            # reject this trajectory for consideration.
            start_pose = wp[0]
            assert start_pose
            sdiff = start_pose.relativeTo(curr_pose).translation().norm() 
            if sdiff > self.max_start_dist_error and self.target_pose != Positions.PROCESSOR:
                continue
            
            end_pose = wp[-1]
            assert end_pose
            ediff = end_pose.relativeTo(self.target_pose).translation().norm()
            if ediff > self.max_end_dist_error and self.target_pose != Positions.PROCESSOR:
                continue

            # Iterate through the waypoints so we can calculate distance between
            # each pair and sum itup
            distance = 0.0
            for p1, p2 in zip(wp, wp[1:]):
                distance += p1.relativeTo(p2).translation().norm()

            score = ediff + sdiff + distance
            if score < winner_score:
                winner = wp
                winner_score = score
        return winner

    @state(must_finish=True)
    def follow_pp(self, initial_call, state_tm):
        assert self.pp_trajectory
        if initial_call:
            traj_poses: list[Pose2d] = []
            for x in range(0, int(self.pp_trajectory.getTotalTimeSeconds() * 1000), 100):
                sample = self.pp_trajectory.sample(x / 1000)
                assert sample
                traj_poses.append(Pose2d(sample.pose.x, sample.pose.y, sample.heading))
            self.pp_traj_pub.set(traj_poses)
            self.choreo_traj_pub.set([])
        sample = self.pp_trajectory.sample(state_tm)
        self.drivetrain.follow_trajectory_pp(sample)
        if state_tm > self.pp_trajectory.getTotalTimeSeconds() * 1.1:
            self.next_state(self.drive_swoop)

    def prep_pp_trajectory(self, curr_pose: Pose2d, target_pose: Pose2d,
                           max_vel=3.0, max_accel=5.0,
                           max_omega=4 * math.tau, max_alpha=math.tau):
        constraints = PathConstraints(max_vel, max_accel, max_omega, max_alpha)
        self.target_pose = target_pose
        # First see if we can lift some waypoints into this from
        # our choreo paths
        tic: float = time.perf_counter()
        # choreo_waypoints = self.find_choreo_waypoints(curr_pose, self.target_pose)
        # Find the heading needed to get from curr_pose to self.target_pose
        heading = math.atan2(
            self.target_pose.Y() - curr_pose.Y(),
            self.target_pose.X() - curr_pose.X(),
        )
        initial_waypoint_pose = Pose2d(curr_pose.translation(), Rotation2d(heading))
        all_waypoints = [initial_waypoint_pose]
        # if choreo_waypoints is not None:
        #     all_waypoints = [initial_waypoint_pose, *choreo_waypoints]
        if Positions.is_reef_pose(self.target_pose):
            # Now get the one meter out pose
            tag_id, _ = Waypoints.closest_reef_tag_id(self.target_pose)
            vision_pose = Waypoints.get_tag_meter_away(tag_id)
            all_waypoints.append(
                Pose2d(vision_pose.translation(), vision_pose.rotation() + Rotation2d(math.pi))
            )
        second_to_last_pose = all_waypoints[-1]
        heading = math.atan2(
            self.target_pose.Y() - second_to_last_pose.Y(),
            self.target_pose.X() - second_to_last_pose.X(),
        )
        final_waypoint_pose = Pose2d(self.target_pose.translation(), Rotation2d(heading))
        all_waypoints.append(final_waypoint_pose)
        self.pp_waypoints_pub.set(all_waypoints)
        waypoints = PathPlannerPath.waypointsFromPoses(
            all_waypoints
        )
        path = PathPlannerPath(
            waypoints,
            constraints,
            None,
            GoalEndState(0.0, self.target_pose.rotation()),
        )
        path.preventFlipping = True
        self.pp_trajectory = path.generateTrajectory(
            self.drivetrain.chassis_speeds,
            curr_pose.rotation(),
            self.pp_config,
        )
        toc: float = time.perf_counter()
        self.pp_traj_calc_ms_pub.set(1000 * (toc - tic))

    @state(must_finish=True)
    def drive_swoop(self, initial_call, state_tm):
        # A 'swoop' drive will try and find a Choreo trajectory that roughly
        # starts and ends around where the robot is starting and ending.
        # If a trajectory is found we'll follow along that one until we get
        # "close enough" to the final endpoint and then revert to using
        # 'drive to positon' in the drivetrain to lock in the final pose.
        curr_pose = self.drivetrain.get_pose()
        dist_from_final = curr_pose.relativeTo(self.target_pose).translation().norm()
        if dist_from_final < 0.025:
            self.next_state(self.completed)
            return 

        if initial_call:
            if dist_from_final > 0.50:
                traj = self.find_choreo_trajectory(curr_pose, self.target_pose)
                if traj:
                    self.choreo_trajectory = traj
                    self.next_state_now(self.follow_choreo)
                else:
                    # Let's create a PathPlanner to go right to it
                    self.prep_pp_trajectory(curr_pose, self.target_pose)
                    self.next_state_now(self.follow_pp)
        else:
            self.drivetrain.drive_to_pose(self.target_pose, aggressive=True)

    @state(must_finish=True)
    def completed(self, initial_call):
        pass
