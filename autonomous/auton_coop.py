# File for all of the choreo related paths
import math
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d
from magicbot import AutonomousStateMachine, state, feedback

from utilities.game import is_red
from utilities.position import Positions, reverse_choreo
from utilities import Waypoints

from controllers.manipulator import Manipulator
from controllers.intake import IntakeControl

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
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    MODE_NAME = 'Sample - Coopertition'
    DEFAULT = False

    pose_set = False
    selected_alliance = None

    curr_level = 4
    curr_left = True

    def __init__(self):
        self.reef_a_proc = load_swerve_trajectory('ReefA_Proc')
        self.reef_b_proc = load_swerve_trajectory('ReefB_Proc')

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
            self.photoeye.coral_held = False
            self.photoeye.algae_held = False

    def drive_trajectory(self, traj: SwerveTrajectory, tm):
        sample = traj.sample_at(tm, is_red())
        if sample:
            self.drivetrain.follow_trajectory(sample)

    def at_pose(self, pose: Pose2d, tolerance=0.03) -> bool:
        robot_pose = self.drivetrain.get_pose()
        diff = robot_pose.relativeTo(pose)
        dist = math.sqrt(diff.X()**2 + diff.Y()**2)
        pn('distance of traj', dist)
        return dist < tolerance

    # We start here, drive from the line to the reef
    @state(first=True, must_finish=True)
    def drive_to_reef_start(self, tm, state_tm):
        tag_id = Waypoints.get_tag_id_from_letter('A', is_red())
        pose = Waypoints.get_tag_robot_away(tag_id, face_at=True)
        self.manipulator.algae_mode()
        self.manipulator.set_algae_level1()
        self.manipulator.next_state_now(self.manipulator.algae_prepare_intake)
        self.drivetrain.drive_to_pose(pose)
        if self.at_pose(pose, 0.10) and self.manipulator.at_position():
            self.next_state(self.intake_first_algae)

    # Now score our first coral
    @state(must_finish=True)
    def intake_first_algae(self, state_tm, initial_call):
        if initial_call:
            self.manipulator.intake_control.go_algae_intake()
        if self.photoeye.algae_held is True:
            self.next_state(self.drive_a_to_proc)

    # Drive from face C to the Player Station via Choreo path
    @state(must_finish=True)
    def drive_a_to_proc(self, tm, state_tm, initial_call):
        if initial_call:
            self.manipulator.go_home()
        traj = self.reef_a_proc
        self.drive_trajectory(traj, state_tm)
        end_pose = traj.get_final_pose(is_red())
        assert end_pose
        timeout = state_tm > traj.get_total_time()
        if self.at_pose(end_pose, tolerance=0.1) or timeout:
            self.manipulator.go_algae_score()
            self.next_state(self.score_first_algae)

    # Now shove ourselves into the correct pickup position and
    # wait for a coral to be intook
    @state(must_finish=True)
    def score_first_algae(self, tm, state_tm):
        proc_id = 3 if is_red() else 16
        end_pose = Waypoints.get_tag_robot_away(proc_id, face_at=True)
        self.drivetrain.drive_to_pose(end_pose)
        if self.at_pose(end_pose, tolerance=0.05):
            self.intake_control.go_algae_score()
        if self.photoeye.algae_held is False:
            self.next_state(self.idling)
            self.next_state(self.drive_proc_to_reef_b)

    @state(must_finish=True)
    def idling(self):
        pass
    
    # Drive back to Reef E via a Choreo path
    @state(must_finish=True)
    def drive_proc_to_reef_b(self, state_tm, initial_call):
        traj = reverse_choreo(self.reef_b_proc)
        self.drive_trajectory(traj, state_tm)
        end_pose = traj.get_final_pose(is_red())
        assert end_pose
        # Check distance to end pose
        current_pose = self.drivetrain.get_pose()
        self.manipulator.set_algae_level2()
        diff = current_pose.relativeTo(end_pose).translation().norm()
        if (
            diff < 1
            and self.manipulator.current_state != self.manipulator.algae_prepare_score
        ):
            self.manipulator.next_state_now(self.manipulator.algae_prepare_score)
            self.manipulator.engage()
        timeout = state_tm > traj.get_total_time()
        if ((self.at_pose(end_pose))
            or timeout):
            self.next_state(self.intake_second_algae)

    # Now shove ourselves into the right scoring location and score the coral
    @state(must_finish=True)
    def intake_second_algae(self, tm, state_tm, initial_call):
        pass