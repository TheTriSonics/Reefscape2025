# File for all of the choreo related paths
import math
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d
from magicbot import AutonomousStateMachine, state, feedback

from utilities.game import is_red
from utilities.position import Positions, reverse_choreo
from utilities import Waypoints

from controllers.manipulator import Manipulator
from controllers.intimidator import Intimidator
from controllers.intake import IntakeControl

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from components.battery_monitor import BatteryMonitorComponent
from components.photoeye import PhotoEyeComponent
from choreo import load_swerve_trajectory  # type: ignore
from choreo.trajectory import SwerveTrajectory
from autonomous.base import AutonBase

pb = SmartDashboard.putBoolean
pn = SmartDashboard.putNumber
ps = SmartDashboard.putString


class AutonSample(AutonBase):
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    intimidator: Intimidator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    MODE_NAME = 'Sample - Place Infinite'
    DEFAULT = False

    pose_set = False
    selected_alliance = None

    curr_level = 4
    curr_left = True

    def __init__(self):
        pass

    def get_initial_pose(self) -> Pose2d:
        return Positions.AUTON_LINE_CENTER

    # We start here, drive from the line to the reef
    @state(first=True, must_finish=True)
    def drive_to_reef_start(self, tm, state_tm, initial_call):
        target_pose = Positions.REEF_F_RIGHT
        if initial_call:
            self.manipulator.go_home()
            self.intimidator.go_drive_swoop(target_pose)
            self.manipulator.set_coral_level4()
        # Get distance from current pose to target pose
        if self.at_pose(target_pose, tolerance=0.8):
            self.manipulator.go_coral_prepare_score()
        if self.at_pose(target_pose):
            self.next_state(self.place_first_coral)

    # Now score our first coral
    @state(must_finish=True)
    def place_first_coral(self, state_tm, initial_call):
        if (self.manipulator.at_position()):
            self.manipulator.go_coral_score()
        # Wait until the photo eye is clear?
        if self.photoeye.coral_held is False:
            self.next_state(self.intake_second_coral)

    # Drive from face C to the Player Station via Choreo path
    @state(must_finish=True)
    def intake_second_coral(self, tm, state_tm, initial_call):
        target_pose = Positions.PS_CLOSEST
        if initial_call:
            self.intimidator.go_drive_swoop(target_pose)
            self.manipulator.go_home()
        if self.at_pose(target_pose, tolerance=0.3):
            self.intake_control.go_coral_intake()
        if (
            self.at_pose(target_pose)
            and self.intake_control.current_state == self.intake_control.idling
        ):
            self.next_state(self.score_second_coral)
    
    @state(must_finish=True)
    def score_second_coral(self, state_tm, initial_call):
        target_pose = Positions.REEF_CLOSEST_LEFT
        if initial_call:
            self.intimidator.go_drive_swoop(target_pose)
            self.manipulator.set_coral_level4()
        if self.at_pose(target_pose, tolerance=0.8):
            self.manipulator.go_coral_prepare_score()
        if self.at_pose(target_pose):
            self.intake_control.go_coral_score()
        if self.at_pose(target_pose) and self.intake_control.current_state == self.intake_control.idling:
            self.next_state(self.wee)
