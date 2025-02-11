import math
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d
from magicbot import AutonomousStateMachine, timed_state, state, feedback, tunable

from utilities.game import is_red

from controllers.manipulator import Manipulator

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from components.battery_monitor import BatteryMonitorComponent
from choreo import load_swerve_trajectory  # type: ignore
from choreo.trajectory import SwerveTrajectory

pb = SmartDashboard.putBoolean
pn = SmartDashboard.putNumber
ps = SmartDashboard.putString
class AutonPlaceHard(AutonomousStateMachine):
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    # manipulator: Manipulator

    MODE_NAME = 'Place Difficult Coral'
    DEFAULT = True

    pose_set = False
    selected_alliance = None

    corals_placed = tunable(0)
    corals_desired = tunable(3)
    def __init__(self):
        # Drive to position A
        self.to_reef_from_start = load_swerve_trajectory('Start_ReefA')
        # Drive to PS2
        self.reef_to_ps = load_swerve_trajectory('ReefA_PS2')
        # Drive back to A
        self.ps_to_reef = load_swerve_trajectory('PS2_ReefA')
        pb('auton/choreo/placing_coral', False)
        return

    def set_initial_pose(self) -> None:
        # No need to set the pose twice!
        alliance = 'red' if is_red() else 'blue'
        if alliance is not self.selected_alliance:
            self.pose_set = False

        if self.pose_set is True:
            return

        initial_pose = self.to_reef_from_start.get_initial_pose(is_red())
        if initial_pose is not None:
            self.drivetrain.set_pose(initial_pose)
            self.selected_alliance = alliance
            # self.gyro.reset_heading(initial_pose.rotation().degrees())
            self.pose_set = True

    def drive_trajectory(self, traj: SwerveTrajectory, tm):
        pb('auton/choreo/placing_coral', False)
        sample = traj.sample_at(tm, is_red())
        if sample:
            self.drivetrain.follow_choreo_trajectory(sample)

    def at_pose(self, pose: Pose2d) -> bool:
        robot_pose = self.drivetrain.get_pose()
        diff = robot_pose.relativeTo(pose)
        dist = math.sqrt(diff.X()**2 + diff.Y()**2)
        pn('distance of traj', dist)
        return dist < 0.05

    @timed_state(first=True, duration=2.0, must_finish=True,
                 next_state='place_first_coral')
    def drive_to_reef_first(self, tm, state_tm):
        self.corals_placed = 0
        self.drive_trajectory(self.to_reef_from_start, state_tm)
        last_pose = self.to_reef_from_start.get_final_pose(is_red())
        if last_pose is not None and self.at_pose(last_pose):
            self.next_state(self.place_first_coral)

    @timed_state(must_finish=True, duration=1.0, next_state="drive_to_ps")
    def place_first_coral(self, tm, state_tm, initial_call):
        pb('auton/choreo/placing_coral', True)
        if initial_call:
            self.corals_placed += 1
        return

    @timed_state(must_finish=True, duration=4.0,
                 next_state="wait_on_player_coral")
    def drive_to_ps(self, tm, state_tm):
        self.drive_trajectory(self.reef_to_ps, state_tm)
        last_pose = self.reef_to_ps.get_final_pose(is_red())
        if last_pose is not None and self.at_pose(last_pose):
            print('waiting on player now')
            self.next_state(self.wait_on_player_coral)
        pass

    @timed_state(duration=1.0, next_state="drive_to_reef")
    def wait_on_player_coral(self, tm, state_tm):
        """
        if photoeyes.has_coral():
            self.next_state(self.drive_to_reef)
        """
        # self.manipulator.intake.intake_in()
        print('waiting...')


    @timed_state(must_finish=True, duration=5.0, next_state='place_coral')
    def drive_to_reef(self, tm, state_tm):
        # self.manipulator.intake.intake_off()
        self.drive_trajectory(self.ps_to_reef, state_tm)
        final = self.ps_to_reef.get_final_pose(is_red())
        if final is not None and self.at_pose(final):
            self.next_state(self.place_coral)
    
    @timed_state(must_finish=True, duration=1.0, next_state="drive_to_ps")
    def place_coral(self, tm, state_tm, initial_call):
        pb('auton/choreo/placing_coral', True)
        if initial_call:
            self.corals_placed += 1
        if self.corals_placed >= self.corals_desired:
            self.done()
        return

