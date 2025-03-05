# File for all of the choreo related paths
import math
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d
from magicbot import AutonomousStateMachine, state, feedback, tunable

from utilities.game import is_red, ManipLocations
from utilities.position import Positions, reverse_choreo
from utilities import Waypoints

from controllers.manipulator import Manipulator
from controllers.intimidator import Intimidator
from controllers.intake import IntakeControl

from components.elevator import ElevatorComponent
from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from components.battery_monitor import BatteryMonitorComponent
from components.photoeye import PhotoEyeComponent
from components.arm import ArmComponent
from autonomous.base import AutonBase

pb = SmartDashboard.putBoolean
pn = SmartDashboard.putNumber
ps = SmartDashboard.putString


class AutonMountPleasant(AutonBase):
    elevator: ElevatorComponent
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    arm: ArmComponent
    intimidator: Intimidator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    MODE_NAME = 'Sample - Mt. Pleasant'
    DEFAULT = True
    
    def __init__(self):
        pass

    # The base class uses this in set_initial_pose() to set where the robot
    # thinks it should be at the beginning of an auton.  If you dont' provide
    # this method in an auton that inherits from AutonBase, it will default to
    # the origin, positon 0, 0, which is behind a player station on the blue
    # side,
    def get_initial_pose(self):
        self.photoeye.back_photoeye = True
        self.photoeye.coral_held = True
        return Positions.AUTON_LINE_CENTER

    @state(must_finish=True, first=True)
    def drive_to_reef(self, state_tm, initial_call):
        from utilities import is_sim
        target_pose = Positions.REEF_F_LEFT
        if initial_call:
            self.intimidator.go_drive_swoop(target_pose)
            self.manipulator.coral_mode()
            self.arm.target_pos = 90
            self.manipulator.set_coral_level4()
        if self.photoeye.coral_held is False:
            self.next_state(self.drive_to_a_safe)
            pass
        if self.at_pose(target_pose, 3.0) and self.arm.get_position() > 80:
            # Get the lift moving into the right position
            self.manipulator.go_coral_prepare_score()
            pass
        if (
            self.at_pose(target_pose, 0.04) and self.manipulator.at_position()
            and (self.photoeye.back_photoeye or self.photoeye.front_photoeye)
        ) or state_tm > 4.0:
            self.intake_control.go_coral_score()
            pass

    @state(must_finish=True)
    def drive_to_a_safe(self, state_tm, initial_call):
        from wpimath.geometry import Transform2d, Rotation2d
        robot_pose = Positions.REEF_F_LEFT.transformBy(Transform2d(-0.5, 0, Rotation2d.fromDegrees(90)))
        self.intimidator.go_drive_pose(robot_pose, aggressive=True)
        if self.at_pose(robot_pose, 0.04):
            self.manipulator.go_home()
            if self.elevator.get_position() < 10:
                self.next_state(self.drive_to_ps)


    @state(must_finish=True)    
    def drive_to_ps(self, state_tm, initial_call):
        target_pose = Positions.PS_CLOSEST
        if initial_call:
            self.intimidator.go_drive_swoop(target_pose)
        if self.at_pose(target_pose, 0.50):
            self.intake_control.go_coral_intake()
        if self.photoeye.front_photoeye or self.photoeye.back_photoeye:
            self.next_state(self.score_coral)

    @state(must_finish=True)
    def score_coral(self, state_tm, initial_call):
        target_pose = Positions.REEF_E_LEFT
        if initial_call:
            self.intimidator.go_drive_swoop(target_pose)
        if self.at_pose(target_pose, 0.04):
            self.intake_control.go_coral_score()
        