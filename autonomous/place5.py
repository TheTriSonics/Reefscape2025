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
class AutonPlace5(AutonomousStateMachine):
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    # manipulator: Manipulator

    MODE_NAME = 'Place 5 Coral'
    DEFAULT = False

    pose_set = False
    selected_alliance = None

    corals_placed = tunable(0)
    corals_desired = tunable(3)
    def __init__(self):
        # drive to reef position B
        self.to_reef_from_start = load_swerve_trajectory('AutonPlace2_01')
        # drive from B to PS1
        self.reef_to_ps = load_swerve_trajectory('AutonPlace2_02')
        # drive to reef position C from PS1
        self.to_reefC_from_ps1 = load_swerve_trajectory('AutonPS1_ReefC')
        # Drive from C to PS1
        self.to_ps1_from_reefC = load_swerve_trajectory('AutonReefC_PS1')
        # Return to start position; just for testing!
        self.reefC_to_start = load_swerve_trajectory('AutonReefC_Start')
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
            self.drivetrain.follow_trajectory(sample)

    def at_pose(self, pose: Pose2d) -> bool:
        robot_pose = self.drivetrain.get_pose()
        diff = robot_pose.relativeTo(pose)
        dist = math.sqrt(diff.X()**2 + diff.Y()**2)
        pn('distance of traj', dist)
        return dist < 0.05

    @timed_state(first=True, duration=2.0, must_finish=True,
                 next_state='place_coralB')
    def drive_to_reefB(self, tm, state_tm):
        self.corals_placed = 0
        self.drive_trajectory(self.to_reef_from_start, state_tm)
        last_pose = self.to_reef_from_start.get_final_pose(is_red())
        if last_pose is not None and self.at_pose(last_pose):
            print('close enough, drop the coral!')
            self.next_state(self.place_coralB)

    @timed_state(must_finish=True, duration=1.0, next_state="drive_to_ps")
    def place_coralB(self, tm, state_tm, initial_call):
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
    
    @timed_state(must_finish=True, duration=4.0,
                 next_state="wait_on_player_coral")
    def drive_to_ps1(self, tm, state_tm):
        # This one cycles back and forth
        self.drive_trajectory(self.to_ps1_from_reefC, state_tm)
        last_pose = self.to_ps1_from_reefC.get_final_pose(is_red())
        if last_pose is not None and self.at_pose(last_pose):
            print('waiting on player now')
            self.next_state(self.wait_on_player_coral)
        pass

    @timed_state(duration=1.0, next_state="drive_to_reefC")
    def wait_on_player_coral(self, tm, state_tm):
        """
        if photoeyes.has_coral():
            self.next_state(self.drive_to_reef)
        """
        # self.manipulator.intake.intake_in()
        print('waiting...')


    @timed_state(must_finish=True, duration=5.0, next_state='place_coralC')
    def drive_to_reefC(self, tm, state_tm):
        # self.manipulator.intake.intake_off()
        self.drive_trajectory(self.to_reefC_from_ps1, state_tm)
        final = self.to_reefC_from_ps1.get_final_pose(is_red())
        if final is not None and self.at_pose(final):
            self.next_state(self.place_coralC)
    
    @timed_state(must_finish=True, duration=1.0, next_state="drive_to_ps1")
    def place_coralC(self, tm, state_tm, initial_call):
        pb('auton/choreo/placing_coral', True)
        if initial_call:
            self.corals_placed += 1
        if self.corals_placed >= self.corals_desired:
            # self.done()
            self.next_state_now(self.return_to_start)
        return

    @timed_state(must_finish=True, duration=8.0)
    def return_to_start(self, tm, state_tm, initial_call):
        self.drive_trajectory(self.reefC_to_start, state_tm)
        final = self.reefC_to_start.get_final_pose(is_red())
        if final is not None and self.at_pose(final):
            self.done()

