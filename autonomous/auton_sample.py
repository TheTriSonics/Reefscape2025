# File for all of the choreo related paths
import math
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d
from magicbot import AutonomousStateMachine, timed_state, state, feedback

from utilities.game import is_red
from utilities.position import Positions, reverse_choreo
from utilities.waypoints import (
    get_tag_id_from_letter, get_tag_robot_away, shift_reef_left, shift_reef_right
)

from controllers.manipulator import Manipulator

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from components.battery_monitor import BatteryMonitorComponent
from components.photoeye import PhotoEyeComponent
from choreo import load_swerve_trajectory  # type: ignore
from choreo.trajectory import SwerveTrajectory

pb = SmartDashboard.putBoolean
pn = SmartDashboard.putNumber
ps = SmartDashboard.putString


class AutonSample(AutonomousStateMachine):
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    photoeye: PhotoEyeComponent

    MODE_NAME = 'Sample - Place Infinite'
    DEFAULT = True

    pose_set = False
    selected_alliance = None

    def __init__(self):
        self.reef_f_ps_left = load_swerve_trajectory('ReefF_PSLeft')
        self.reef_e_ps_left = load_swerve_trajectory('ReefE_PSLeft')

    def set_initial_pose(self) -> None:
        # No need to set the pose twice!
        alliance = 'red' if is_red() else 'blue'
        if alliance is not self.selected_alliance:
            self.pose_set = False

        if self.pose_set is True:
            return

        initial_pose = Positions.auton_line_2(is_red())
        if initial_pose is not None:
            self.drivetrain.set_pose(initial_pose)
            self.selected_alliance = alliance
            # self.gyro.reset_heading(initial_pose.rotation().degrees())
            self.pose_set = True
            self.photoeye.coral_held = True

    def drive_trajectory(self, traj: SwerveTrajectory, tm):
        sample = traj.sample_at(tm, is_red())
        if sample:
            self.drivetrain.follow_trajectory(sample)

    def at_pose(self, pose: Pose2d) -> bool:
        robot_pose = self.drivetrain.get_pose()
        diff = robot_pose.relativeTo(pose)
        dist = math.sqrt(diff.X()**2 + diff.Y()**2)
        pn('distance of traj', dist)
        return dist < 0.03

    # We start here, drive from the line to the reef
    @state(first=True, must_finish=True)
    def drive_to_reef_start(self, tm, state_tm):
        tag_id = get_tag_id_from_letter('F', is_red())
        pose = shift_reef_left(get_tag_robot_away(tag_id, face_at=True))
        self.manipulator.set_coral_level4()
        self.drivetrain.drive_to_pose(pose)
        # Get distance from current pose to target pose
        current_pose = self.drivetrain.get_pose()
        diff = current_pose.relativeTo(pose).translation().norm()
        if diff < 0.3:
            # Start the manipulator moving
            self.manipulator.next_state(self.manipulator.coral_prepare_score)
        if self.at_pose(pose):
            self.next_state(self.place_first_coral)

    # Now score our first coral
    @state(must_finish=True)
    def place_first_coral(self, state_tm, initial_call):
        if (self.manipulator.at_position() and
            self.photoeye.coral_held and
            self.manipulator.current_state != self.manipulator.coral_score):
            self.manipulator.next_state(self.manipulator.coral_score)
        # Wait until the photo eye is clear?
        if self.photoeye.coral_held is False:
            self.next_state(self.drive_e_to_ps_choreo)

    # Drive from face C to the Player Station via Choreo path
    @state
    def drive_e_to_ps_choreo(self, tm, state_tm):
        traj = self.reef_e_ps_left
        self.drive_trajectory(traj, state_tm)
        end_pose = traj.get_final_pose(is_red())
        assert end_pose
        timeout = state_tm > traj.get_total_time()
        if self.at_pose(end_pose) or timeout:
            self.next_state(self.intake_second_coral)

    # Now shove ourselves into the correct pickup position and
    # wait for a coral to be intook
    @state(must_finish=True)
    def intake_second_coral(self, tm, state_tm):
        ps_target = 1 if is_red() else 13
        end_pose = get_tag_robot_away(ps_target)
        self.drivetrain.drive_to_pose(end_pose)
        if self.at_pose(end_pose):
            self.manipulator.intake.coral_in()
        if self.photoeye.coral_held:
            self.next_state(self.drive_ps_to_reef_e_choreo)
    
    # Drive back to Reef E via a Choreo path
    @state(must_finish=True)
    def drive_ps_to_reef_e_choreo(self, state_tm, initial_call):
        if initial_call:
            self.manipulator.intake.intake_off()
        traj = reverse_choreo(self.reef_e_ps_left)
        self.drive_trajectory(traj, state_tm)
        end_pose = traj.get_final_pose(is_red())
        assert end_pose
        # Check distance to end pose
        current_pose = self.drivetrain.get_pose()
        diff = current_pose.relativeTo(end_pose).translation().norm()
        self.manipulator.set_coral_level4()
        if (
            diff < 0.3
            and self.manipulator.current_state != self.manipulator.coral_prepare_score
        ):
            self.manipulator.next_state(self.manipulator.coral_prepare_score)
        timeout = state_tm > traj.get_total_time() * 2.0
        if self.at_pose(end_pose) or timeout:
            self.next_state(self.place_second_coral)

    # Now shove ourselves into the right scoring location and score the coral
    @state(must_finish=True)
    def place_second_coral(self, tm, state_tm):
        tag_id = get_tag_id_from_letter('E', is_red())
        end_pose = shift_reef_left(get_tag_robot_away(tag_id, face_at=True))
        assert end_pose
        self.drivetrain.drive_to_pose(end_pose)
        timeout = state_tm > 20.0
        if self.at_pose(end_pose) or timeout:  # And we have a coral ejected photoeye?
            # And this loops us back. We'll keep doing this leg until we disable
            self.next_state(self.drive_e_to_ps_choreo)