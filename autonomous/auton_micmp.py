import math
import ntcore
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d, Transform2d, Rotation2d
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


# Base class for 'big one' autons that try and fill a face on the reef
class JustDrive(AutonBase):
    elevator: ElevatorComponent
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    arm: ArmComponent
    intimidator: Intimidator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    first_face = tunable('F')
    fill_face = tunable('E')
    curr_level = tunable(4)
    curr_left = tunable(True)

    at_pose_counter = tunable(0)

    MODE_NAME = 'MICMP - Just Drive'
    DEFAULT = True
    
    def __init__(self):
        self.coral_scored = 0
        self.backup_pose_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/auton_wmi/backup_pose", Pose2d)
            .publish()
        )

    # The base class uses this in set_initial_pose() to set where the robot
    # thinks it should be at the beginning of an auton.  If you dont' provide
    # this method in an auton that inherits from AutonBase, it will default to
    # the origin, positon 0, 0, which is behind a player station on the blue
    # side.
    def get_initial_pose(self):
        self.photoeye.back_photoeye = True
        self.photoeye.coral_held = True
        return Positions.AUTON_LINE_OUR_CAGE_CENTER

    def prepare_first_trajectory(self):
        target_pose = Positions.get_facepos(self.first_face, left=True)
        curr_pose = self.drivetrain.get_pose()
        # self.intimidator.prep_pp_trajectory(curr_pose, target_pose)

    # Leave the initial starting position and head to the Reef to score
    @state(must_finish=True, first=True)
    def to_reef(self, state_tm, initial_call):
        target_pose = Positions.get_facepos(self.first_face, left=True, close=True)
        if initial_call:
            # self.intimidator.engage(self.intimidator.follow_pp)
            self.intimidator.go_drive_swoop(target_pose)
        if self.at_pose(target_pose):
            self.coral_scored += 1
            self.next_state_now(self.to_backup)

    @state(must_finish=True)
    def to_backup(self, state_tm, initial_call):
        ps_pose = Positions.PS_CLOSEST
        reef_pose = Positions.REEF_CLOSEST
        backup_target = reef_pose.transformBy(Transform2d(-0.3, 0, Rotation2d(0)))
        backup_pose = Pose2d(backup_target.translation(), ps_pose.rotation())
        self.backup_pose_pub.set(backup_pose)   
        self.intimidator.go_drive_pose(backup_target, aggressive=True)
        if self.at_pose(backup_pose):
            self.next_state_now(self.to_ps)

    @state(must_finish=True)
    def to_ps(self, state_tm, initial_call):
        if initial_call:
            # self.intimidator.engage(self.intimidator.follow_pp)
            self.intimidator.go_drive_swoop(Positions.PS_CLOSEST)
        if self.at_pose(Positions.PS_CLOSEST) and state_tm > 1.0:
            self.next_state_now(self.to_face)

    @state(must_finish=True)
    def to_face(self, state_tm, initial_call):
        close = self.curr_level in [3, 4]
        target_pose = Positions.get_facepos(
            self.fill_face, left=self.curr_left, right=not self.curr_left,
            close=close
        )
        if initial_call:
            self.intimidator.go_drive_swoop(target_pose)
            if self.coral_scored >= 1:
                self.curr_left = not self.curr_left
                if self.curr_left is True:
                    self.curr_level -= 1
                if self.curr_level == 0:
                    self.next_state_now(self.wee)
        elif self.intimidator.current_state == self.intimidator.completed.name:
            self.next_state_now(self.score_coral)

    @state(must_finish=True)
    def score_coral(self, state_tm):
        if state_tm > 1.0:
            self.coral_scored += 1
            self.next_state_now(self.to_ps)
