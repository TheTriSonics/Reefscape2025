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
class LeftCoral(AutonBase):
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

    MODE_NAME = 'MICMP - Left Coral'
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
        target_pose = Positions.get_facepos(self.first_face, left=False)
        curr_pose = self.drivetrain.get_pose()
        # self.intimidator.prep_pp_trajectory(curr_pose, target_pose)

    # Leave the initial starting position and head to the Reef to score
    @state(must_finish=True, first=True)
    def to_reef(self, state_tm, initial_call):
        target_pose = Positions.get_facepos(
            self.first_face, left=False, right=True, close=True
        )
        if initial_call:
            self.manipulator.set_coral_level(4)
            self.manipulator.coral_mode()
            self.arm.target_pos = 90
            self.intimidator.go_drive_swoop(target_pose)
        if self.arm.get_position() > 70:
            self.manipulator.go_coral_prepare_score()
        if self.at_pose(target_pose) and self.manipulator.at_position():
            self.intake_control.go_coral_score()
        if self.photoeye.coral_held is False:
            self.coral_scored += 1
            self.next_state_now(self.to_backup)

    @state(must_finish=True)
    def to_backup(self, state_tm, initial_call):
        if initial_call:
            ps_pose = Positions.PS_CLOSEST
            reef_pose = Positions.REEF_CLOSEST
            backup_target = reef_pose.transformBy(Transform2d(-0.5, 0, Rotation2d(0)))
            backup_pose = Pose2d(backup_target.translation(), ps_pose.rotation())
            curr_pose = self.drivetrain.get_pose()
            self.backup_pose_pub.set(backup_pose)   
            self.intimidator.prep_pp_trajectory(curr_pose, ps_pose, [backup_pose])
            self.intimidator.next_state_now(self.intimidator.follow_pp)

        if self.manipulator.reef_dist() > self.manipulator.reef_protection_dist:
            self.manipulator.go_coral_intake()
            if self.elevator.get_position() <= 20:
                self.next_state_now(self.to_ps)

    @state(must_finish=True)
    def to_ps(self, state_tm, initial_call):
        if initial_call:
            # If the elevator is up, we need to bring it down so just reset it
            # We don't just force it home because a previous state might have
            # it in intake already.
            if self.elevator.get_position() > 20:
                self.manipulator.go_home()
            self.intimidator.go_drive_swoop(Positions.PS_CLOSEST)
        if self.photoeye.coral_held:
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
        if self.manipulator.reef_dist() < 2.0:
            self.manipulator.go_coral_prepare_score()
        if self.at_pose(target_pose, 0.08) and self.manipulator.at_position():
            self.intake_control.go_coral_score()
        if self.photoeye.coral_held is False:
            self.next_state_now(self.to_face_backup)
            self.coral_scored += 1
            if self.coral_scored >= 1:
                self.curr_left = not self.curr_left
                if self.curr_left is True:
                    self.curr_level -= 1
                if self.curr_level == 0:
                    self.next_state_now(self.wee)

    @state(must_finish=True)
    def to_face_backup(self, state_tm, initial_call):
        if initial_call:
            self.intimidator.go_drive_pose(Positions.PS_CLOSEST, aggressive=False)
        if self.manipulator.reef_dist() > self.manipulator.reef_protection_dist:
            self.manipulator.go_coral_intake()
            if self.elevator.get_position() <= 20:
                self.next_state_now(self.to_ps)


class RightCoral(AutonBase):
    elevator: ElevatorComponent
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    arm: ArmComponent
    intimidator: Intimidator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    first_face = tunable('B')
    fill_face = tunable('C')
    curr_level = tunable(4)
    curr_left = tunable(True)

    at_pose_counter = tunable(0)

    MODE_NAME = 'MICMP - Right Coral'
    DEFAULT = False
    
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
            self.manipulator.set_coral_level(4)
            self.manipulator.coral_mode()
            self.arm.target_pos = 90
            self.intimidator.go_drive_swoop(target_pose)
        if self.arm.get_position() > 80:
            self.manipulator.go_coral_prepare_score()
        if self.at_pose(target_pose) and self.manipulator.at_position():
            self.intake_control.go_coral_score()
        if self.photoeye.coral_held is False:
            self.coral_scored += 1
            self.next_state_now(self.to_backup)

    @state(must_finish=True)
    def to_backup(self, state_tm, initial_call):
        if initial_call:
            ps_pose = Positions.PS_CLOSEST
            reef_pose = Positions.REEF_CLOSEST
            backup_target = reef_pose.transformBy(Transform2d(-0.5, 0, Rotation2d(0)))
            backup_pose = Pose2d(backup_target.translation(), ps_pose.rotation())
            curr_pose = self.drivetrain.get_pose()
            self.backup_pose_pub.set(backup_pose)   
            self.intimidator.prep_pp_trajectory(curr_pose, ps_pose, [backup_pose])
            self.intimidator.next_state_now(self.intimidator.follow_pp)

        if self.manipulator.reef_dist() > self.manipulator.reef_protection_dist:
            self.manipulator.go_coral_intake()
            if self.elevator.get_position() <= 20:
                self.next_state_now(self.to_ps)

    @state(must_finish=True)
    def to_ps(self, state_tm, initial_call):
        if initial_call:
            # If the elevator is up, we need to bring it down so just reset it
            # We don't just force it home because a previous state might have
            # it in intake already.
            if self.elevator.get_position() > 20:
                self.manipulator.go_home()
            self.intimidator.go_drive_swoop(Positions.PS_CLOSEST)
        if self.photoeye.coral_held:
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
        if self.manipulator.reef_dist() < 2.0:
            self.manipulator.go_coral_prepare_score()
        if self.at_pose(target_pose, 0.08) and self.manipulator.at_position():
            self.intake_control.go_coral_score()
        if self.photoeye.coral_held is False:
            self.next_state_now(self.to_face_backup)
            self.coral_scored += 1
            if self.coral_scored >= 1:
                self.curr_left = not self.curr_left
                if self.curr_left is True:
                    self.curr_level -= 1
                if self.curr_level == 0:
                    self.next_state_now(self.wee)

    @state(must_finish=True)
    def to_face_backup(self, state_tm, initial_call):
        if initial_call:
            self.intimidator.go_drive_pose(Positions.PS_CLOSEST, aggressive=False)
        if self.manipulator.reef_dist() > self.manipulator.reef_protection_dist:
            self.manipulator.go_coral_intake()
            if self.elevator.get_position() <= 20:
                self.next_state_now(self.to_ps)