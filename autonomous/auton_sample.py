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
    DEFAULT = True

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
    def place_first_coral(self, tm, state_tm, initial_call):
        target_pose = Positions.REEF_F_RIGHT
        if initial_call:
            self.manipulator.go_home()
            self.intimidator.go_drive_swoop(target_pose)
            self.manipulator.set_coral_level4()
        if self.intimidator.at_position() and self.manipulator.at_position():
            # We're at the reef and the manipulator is in place.
            # Figure out if we need to still score coral or boogie over to
            # the next state
            if self.photoeye.coral_held is True:
                self.intake_control.go_coral_score()
            elif self.intake_control.is_idling():
                self.next_state(self.intake_second_coral)
        if self.at_pose(target_pose, tolerance=0.8):
            # Begin to move the manipulator if we're close enough
            self.manipulator.go_coral_prepare_score()

    # Driver to the player station and get a coral
    @state(must_finish=True)
    def intake_second_coral(self, tm, state_tm, initial_call):
        target_pose = Positions.PS_CLOSEST
        if initial_call:
            print('started intake of second coral')
            self.intimidator.go_drive_swoop(target_pose)
            self.manipulator.go_home()
        if self.intimidator.at_position(tolerance=0.08) and self.manipulator.at_position():
            if self.photoeye.coral_held is False:
                self.intake_control.go_coral_intake()
            elif self.intake_control.is_idling():
                self.next_state(self.score_second_coral)
    
    @state(must_finish=True)
    def score_second_coral(self, state_tm, initial_call):
        target_pose = (
            Positions.REEF_CLOSEST_LEFT
            if self.curr_left
            else Positions.REEF_CLOSEST_RIGHT
        )
        if initial_call:
            self.manipulator.go_home()
            self.intimidator.go_drive_swoop(target_pose)
            match self.curr_level:
                case 1:
                    self.manipulator.set_coral_level1()
                case 2:
                    self.manipulator.set_coral_level2()
                case 3:
                    self.manipulator.set_coral_level3()
                case 4:
                    self.manipulator.set_coral_level4()
                case _:
                    print('Ugly print statement')
        if self.intimidator.at_position() and self.manipulator.at_position():
            # We're at the reef and the manipulator is in place.
            # Figure out if we need to still score coral or boogie over to
            # the next state
            if self.photoeye.coral_held is True:
                self.intake_control.go_coral_score()
            elif self.intake_control.is_idling():
                self.curr_left = not self.curr_left
                if self.curr_left:
                    self.curr_level -= 1
                if self.curr_left == 0:
                    self.next_state(self.wee)
                self.next_state(self.intake_second_coral)
        if self.at_pose(target_pose, tolerance=0.8):
            # Begin to move the manipulator if we're close enough
            self.manipulator.go_coral_prepare_score()