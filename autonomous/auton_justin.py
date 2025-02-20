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


class AutonJustin(AutonomousStateMachine):
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    MODE_NAME = 'Sample - Justin'
    DEFAULT = True

    def at_pose(self, pose: Pose2d, tolerance=0.03) -> bool:
        robot_pose = self.drivetrain.get_pose()
        diff = robot_pose.relativeTo(pose)
        dist = math.sqrt(diff.X()**2 + diff.Y()**2)
        pn('distance of traj', dist)
        return dist < tolerance

    # We start here, drive from the line to the reef
    @state(first=True, must_finish=True)
    def drive_to_reef_start(self, tm, state_tm, initial_call):
        if initial_call:
            initial_pose = Positions.auton_line_2(is_red())
            self.drivetrain.set_pose(initial_pose)
            self.photoeye.coral_held = True
        tag_id = Waypoints.get_tag_id_from_letter('F', is_red())
        pose = Waypoints.shift_reef_left(Waypoints.get_tag_robot_away(tag_id, face_at=True))
        self.manipulator.set_coral_level4()
        self.drivetrain.drive_to_pose(pose)
        if self.at_pose(pose):
            self.next_state_now(self.score_first_coral)

    @state(must_finish=True)
    def score_first_coral(self, initial_call):
        if initial_call:
            self.manipulator.next_state_now(self.manipulator.coral_prepare_score)
            self.manipulator.engage()
        if self.intake_control.current_state == self.intake_control.idling.name:
            # We've scored the coral. Now what?
            self.manipulator.go_home()
            self.next_state_now(self.drive_to_ps)
        elif self.manipulator.at_position():
            self.intake_control.go_coral_score()

    @state(must_finish=True)
    def drive_to_ps(self, initial_call):
        pass