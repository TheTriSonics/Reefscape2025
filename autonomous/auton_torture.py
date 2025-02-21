# File for all of the choreo related paths
import math
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d
from magicbot import AutonomousStateMachine, state, feedback

from utilities.game import is_red, ManipLocations
from utilities.position import Positions, reverse_choreo
from utilities import Waypoints

from controllers.manipulator import Manipulator
from controllers.intimidator import Intimidator
from controllers.intake import IntakeControl

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from components.battery_monitor import BatteryMonitorComponent
from components.photoeye import PhotoEyeComponent
from autonomous.base import AutonBase

pb = SmartDashboard.putBoolean
pn = SmartDashboard.putNumber
ps = SmartDashboard.putString


# Try and run every possible path in a match; see what happens.
class AutonTorture(AutonBase):
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    intimidator: Intimidator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    MODE_NAME = 'Testing - Torture'
    DEFAULT = False

    def __init__(self):
        pass

    # The base class uses this in set_initial_pose() to set where the robot
    # thinks it should be at the beginning of an auton.  If you dont' provide
    # this method in an auton that inherits from AutonBase, it will default to
    # the origin, positon 0, 0, which is behind a player station on the blue
    # side,
    def get_initial_pose(self):
        return Positions.AUTON_LINE_CENTER

    @state(must_finish=True, first=True)
    def test_random_path(self, state_tm, initial_call):
        if initial_call:
            self.target_pose = Positions.random_position()
            self.intimidator.go_drive_swoop(self.target_pose)
        if self.at_pose(self.target_pose, tolerance=0.1) is True:
            self.next_state(self.test_random_path)