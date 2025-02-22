# File for all of the choreo related paths
import math
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d
from magicbot import AutonomousStateMachine, state

from utilities.game import is_red
from utilities.position import Positions, reverse_choreo
from utilities import Waypoints

from controllers.manipulator import Manipulator
from controllers.intake import IntakeControl

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from components.battery_monitor import BatteryMonitorComponent
from components.photoeye import PhotoEyeComponent
from choreo import load_swerve_trajectory
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

    MODE_NAME = 'Sample - Place Infinite'
    DEFAULT = True

    pose_set = False
    selected_alliance = None

    curr_level = 4
    curr_left = True

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

    def at_pose(self, pose: Pose2d, tolerance=0.03) -> bool:
        robot_pose = self.drivetrain.get_pose()
        diff = robot_pose.relativeTo(pose)
        dist = math.sqrt(diff.X()**2 + diff.Y()**2)
        pn('distance of traj', dist)
        return dist < tolerance

    # We start here, drive from the line to the reef
    @state(first=True, must_finish=True)
    def drive_to_reef_start(self, tm, state_tm):
        tag_id = Waypoints.get_tag_id_from_letter('F', is_red())
        pose = Waypoints.shift_reef_left(Waypoints.get_tag_robot_away(tag_id, face_at=True))
        self.manipulator.set_coral_level4()
        self.drivetrain.drive_to_pose(pose)
        # Get distance from current pose to target pose
        current_pose = self.drivetrain.get_pose()
        diff = current_pose.relativeTo(pose).translation().norm()
        if diff < 0.8:
            # Start the manipulator moving
            self.manipulator.next_state_now(self.manipulator.coral_prepare_score)
            self.manipulator.engage()
        if self.at_pose(pose):
            self.next_state(self.place_first_coral)

    # Now score our first coral
    @state(must_finish=True)
    def place_first_coral(self, state_tm, initial_call):
        if (self.manipulator.at_position() and
            self.photoeye.coral_held and
            self.manipulator.current_state != self.manipulator.coral_score.name):
            self.manipulator.go_coral_score()
        # Wait until the photo eye is clear?
        if self.photoeye.coral_held is False:
            self.next_state(self.drive_e_to_ps_choreo)

    # Drive from face C to the Player Station via Choreo path
    @state(must_finish=True)
    def drive_e_to_ps_choreo(self, tm, state_tm, initial_call):
        if initial_call:
            self.manipulator.go_home()
        traj = self.reef_e_ps_left
        self.drive_trajectory(traj, state_tm)
        end_pose = traj.get_final_pose(is_red())
        assert end_pose
        timeout = state_tm > traj.get_total_time()
        if self.at_pose(end_pose, tolerance=0.8) or timeout:
            self.next_state(self.intake_second_coral)

    # Now shove ourselves into the correct pickup position and
    # wait for a coral to be intook
    @state(must_finish=True)
    def intake_second_coral(self, tm, state_tm):
        ps_target = 1 if is_red() else 13
        end_pose = Waypoints.get_tag_robot_away(ps_target)
        self.drivetrain.drive_to_pose(end_pose)
        if self.at_pose(end_pose, tolerance=0.1):
            self.intake_control.go_coral_intake()
        if self.photoeye.coral_held:
            self.next_state(self.drive_ps_to_reef_e_choreo)
    
    # Drive back to Reef E via a Choreo path
    @state(must_finish=True)
    def drive_ps_to_reef_e_choreo(self, state_tm, initial_call):
        traj = reverse_choreo(self.reef_e_ps_left)
        self.drive_trajectory(traj, state_tm)
        end_pose = traj.get_final_pose(is_red())
        assert end_pose
        # Check distance to end pose
        current_pose = self.drivetrain.get_pose()
        diff = current_pose.relativeTo(end_pose).translation().norm()
        match self.curr_level:
            case 4:
                self.manipulator.set_coral_level4()
            case 3:
                self.manipulator.set_coral_level3()
            case 2:
                self.manipulator.set_coral_level2()
            case 1:
                self.manipulator.set_coral_level1()
        if (
            diff < 1
            and self.manipulator.current_state != self.manipulator.coral_prepare_score.name
        ):
            self.manipulator.next_state_now(self.manipulator.coral_prepare_score)
            self.manipulator.engage()
        timeout = state_tm > traj.get_total_time()
        if ((self.at_pose(end_pose))
            or timeout):
            self.next_state(self.place_second_coral)

    # Now shove ourselves into the right scoring location and score the coral
    @state(must_finish=True)
    def place_second_coral(self, tm, state_tm, initial_call):
        tag_id = Waypoints.get_tag_id_from_letter('E', is_red())
        shift_func = Waypoints.shift_reef_left if self.curr_left else Waypoints.shift_reef_right
        end_pose = shift_func(Waypoints.get_tag_robot_away(tag_id, face_at=True))
        assert end_pose
        self.drivetrain.drive_to_pose(end_pose)
        if (self.at_pose(end_pose, tolerance=0.05) and
            self.manipulator.at_position()):
            self.manipulator.go_coral_score()
            # And this loops us back. We'll keep doing this leg until we disable
            if self.photoeye.coral_held is False:
                if self.curr_left is False:
                    self.curr_level -= 1
                self.curr_left = not self.curr_left
                self.next_state(self.drive_e_to_ps_choreo)