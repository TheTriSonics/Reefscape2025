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
from components.wrist import WristComponent
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
        if (self.at_pose(target_pose) and self.manipulator.at_position()
            or state_tm > 5.0):
            self.intake_control.go_coral_score()
        if self.photoeye.coral_held is False:
            self.coral_scored += 1
            self.next_state_now(self.to_backup)

    @state(must_finish=True)
    def to_backup(self, state_tm, initial_call):
        if initial_call:
            ps_pose = Positions.PS_CLOSEST
            curr_pose = self.drivetrain.get_pose()
            backup_target = curr_pose.transformBy(Transform2d(-0.5, 0, Rotation2d(0)))
            backup_pose = Pose2d(backup_target.translation(), ps_pose.rotation())
            self.backup_pose_pub.set(backup_pose)   
            self.intimidator.prep_pp_trajectory(curr_pose, ps_pose, [backup_pose])
            self.intimidator.next_state_now(self.intimidator.follow_pp)

        if self.manipulator.reef_dist() > self.manipulator.reef_protection_dist:
            self.manipulator.go_coral_intake()
            if self.elevator.get_position() <= 20:
                self.next_state_now(self.to_ps)

    @state(must_finish=True)
    def to_ps(self, state_tm, initial_call):
        curr_pose = self.drivetrain.get_pose()
        if initial_call:
            # If the elevator is up, we need to bring it down so just reset it
            # We don't just force it home because a previous state might have
            # it in intake already.
            if self.elevator.get_position() > 20:
                self.manipulator.go_home()
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
            self.intimidator.prep_pp_trajectory(self.drivetrain.get_pose(), target_pose, max_vel=3.0, max_accel=1.5)
            self.intimidator.next_state_now(self.intimidator.follow_pp.name)
            self.arm.target_pos = 90
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
            self.intimidator.prep_pp_trajectory(self.drivetrain.get_pose(), Positions.PS_CLOSEST, max_vel=2.0, max_accel=1.5)
            self.intimidator.next_state_now(self.intimidator.follow_pp.name)
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
        self.curr_left = False
        target_pose = Positions.get_facepos(self.first_face, left=True, close=True)
        if initial_call:
            self.manipulator.set_coral_level(4)
            self.manipulator.coral_mode()
            self.arm.target_pos = 90
            self.intimidator.go_drive_swoop(target_pose)
        if self.arm.get_position() > 70:
            self.manipulator.go_coral_prepare_score()
        if (self.at_pose(target_pose) and self.manipulator.at_position()
            or state_tm > 5.0):
            self.intake_control.go_coral_score()
        if self.photoeye.coral_held is False:
            self.coral_scored += 1
            self.next_state_now(self.to_backup)

    @state(must_finish=True)
    def to_backup(self, state_tm, initial_call):
        if initial_call:
            ps_pose = Positions.PS_CLOSEST
            curr_pose = self.drivetrain.get_pose()
            backup_target = curr_pose.transformBy(Transform2d(-0.5, 0, Rotation2d(0)))
            backup_pose = Pose2d(backup_target.translation(), ps_pose.rotation())
            self.backup_pose_pub.set(backup_pose)   
            self.intimidator.prep_pp_trajectory(curr_pose, ps_pose, [backup_pose], max_vel=2.0, max_accel=1.5)  
            self.intimidator.next_state_now(self.intimidator.follow_pp)

        if self.manipulator.reef_dist() > self.manipulator.reef_protection_dist:
            self.manipulator.go_coral_intake()
            if self.elevator.get_position() <= 20:
                self.next_state_now(self.to_ps)

    @state(must_finish=True)
    def to_ps(self, state_tm, initial_call):
        if initial_call and self.elevator.get_position() > 20:
            # If the elevator is up, we need to bring it down so just reset it
            # We don't just force it home because a previous state might have
            # it in intake already.
            self.manipulator.go_home()
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
            self.intimidator.prep_pp_trajectory(self.drivetrain.get_pose(), target_pose, max_vel=3.0, max_accel=1.5)
            self.intimidator.next_state_now(self.intimidator.follow_pp.name)
            self.arm.target_pos = 90
        if self.manipulator.reef_dist() < 2.0:
            pass  # Do nothing for now; we move elevator earlier to clear Batman's vision
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
            self.intimidator.prep_pp_trajectory(self.drivetrain.get_pose(), Positions.PS_CLOSEST, max_vel=2.0, max_accel=1.5)
            self.intimidator.next_state_now(self.intimidator.follow_pp.name)
        if self.manipulator.reef_dist() > self.manipulator.reef_protection_dist:
            self.manipulator.go_coral_intake()
            if self.elevator.get_position() <= 20:
                self.next_state_now(self.to_ps)


class MiddleSafe(AutonBase):
    elevator: ElevatorComponent
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    wrist: WristComponent
    arm: ArmComponent
    intimidator: Intimidator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    first_face = tunable('F')
    fill_face = tunable('E')
    curr_level = tunable(4)
    curr_left = tunable(True)

    at_pose_counter = tunable(0)

    MODE_NAME = 'MICMP - Middle Safe'
    DEFAULT = True
    
    def __init__(self):
        self.coral_scored = 0
        self.backup_pose_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/auton_wmi/backup_pose", Pose2d)
            .publish()
        )

    def get_initial_pose(self):
        self.photoeye.back_photoeye = True
        self.photoeye.coral_held = True
        return Positions.AUTON_LINE_MID


    @state(must_finish=True, first=True)
    def to_reef(self, initial_call, state_tm):
        target_pose = Positions.REEF_A_LEFT_CLOSE
        if initial_call:
            self.manipulator.set_coral_level(4)
            self.manipulator.coral_mode()
            self.arm.target_pos = 90
            curr_pose = self.drivetrain.get_pose()
            # self.intimidator.go_drive_swoop(target_pose)
            self.intimidator.prep_pp_trajectory(
                curr_pose, target_pose, max_vel=2.0, max_accel=1.5
            )
            self.intimidator.next_state_now(self.intimidator.follow_pp)
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
            reef_pose = Positions.REEF_A_LEFT_CLOSE
            backup_pose = reef_pose.transformBy(Transform2d(-1.0, 0, Rotation2d(0)))
            curr_pose = self.drivetrain.get_pose()
            self.backup_pose_pub.set(backup_pose)   
            self.intimidator.prep_pp_trajectory(curr_pose, backup_pose, max_vel=2.0, max_accel=1.5)
            self.intimidator.next_state_now(self.intimidator.follow_pp)

        if self.manipulator.reef_dist() > 1.3:
            self.manipulator.set_algae_level1()
            self.manipulator.go_algae_prepare_intake()
            if self.manipulator.at_position():
                self.next_state_now(self.get_algae)

    @state(must_finish=True)
    def get_algae(self, initial_call, state_tm):
        if initial_call:
            self.intake_control.go_algae_intake()
            self.intimidator.prep_pp_trajectory(
                self.drivetrain.get_pose(),
                Positions.REEF_A.transformBy(Transform2d(0.0, 0, Rotation2d(0))),
                max_vel=1.5,
                max_accel=1.00,
            )
            self.intimidator.next_state_now(self.intimidator.follow_pp)
        if self.photoeye.algae_held is True or (
            self.at_pose(Positions.REEF_A) and state_tm > 1.0
        ):
            self.intake_control.go_algae_hold()
            self.next_state_now(self.prep_for_barge)
    
    @state(must_finish=True)
    def prep_for_barge(self, initial_call, state_tm):
        target_pose = Positions.BARGE_CENTER.transformBy(Transform2d(1.2, 0, Rotation2d(0)))
        if initial_call:
            self.manipulator.set_algae_barge()
            reef_pose = Positions.REEF_A
            backup_pose = reef_pose.transformBy(Transform2d(-1.0, 0, Rotation2d(0)))
            curr_pose = self.drivetrain.get_pose()
            self.backup_pose_pub.set(backup_pose)   
            self.intimidator.prep_pp_trajectory(curr_pose, target_pose, [backup_pose], max_vel=1.0, max_accel=1.0)
            self.intimidator.next_state_now(self.intimidator.follow_pp)
        # Once we're near the barge scoring area up the elevator
        if self.at_pose(target_pose, 1.5):
            self.manipulator.go_algae_prepare_score()


class MiddleDunk(AutonBase):
    elevator: ElevatorComponent
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    wrist: WristComponent
    arm: ArmComponent
    intimidator: Intimidator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    first_face = tunable('F')
    fill_face = tunable('E')
    curr_level = tunable(4)
    curr_left = tunable(True)

    at_pose_counter = tunable(0)

    MODE_NAME = 'MICMP - Middle Dunk'
    DEFAULT = False
    
    def __init__(self):
        self.coral_scored = 0
        self.backup_pose_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/auton_wmi/backup_pose", Pose2d)
            .publish()
        )

    def get_initial_pose(self):
        self.photoeye.back_photoeye = True
        self.photoeye.coral_held = True
        return Positions.AUTON_LINE_MID


    @state(must_finish=True, first=True)
    def to_reef(self, initial_call, state_tm):
        target_pose = Positions.REEF_A_LEFT_CLOSE
        if initial_call:
            self.manipulator.set_coral_level(4)
            self.manipulator.coral_mode()
            self.arm.target_pos = 90
            curr_pose = self.drivetrain.get_pose()
            # self.intimidator.go_drive_swoop(target_pose)
            self.intimidator.prep_pp_trajectory(
                curr_pose, target_pose, max_vel=2.0, max_accel=1.5
            )
            self.intimidator.next_state_now(self.intimidator.follow_pp)
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
            reef_pose = Positions.REEF_A_LEFT_CLOSE
            backup_pose = reef_pose.transformBy(Transform2d(-1.0, 0, Rotation2d(0)))
            curr_pose = self.drivetrain.get_pose()
            self.backup_pose_pub.set(backup_pose)   
            self.intimidator.prep_pp_trajectory(curr_pose, backup_pose, max_vel=2.0, max_accel=1.5)
            self.intimidator.next_state_now(self.intimidator.follow_pp)

        if self.manipulator.reef_dist() > 1.3:
            self.manipulator.set_algae_level1()
            self.manipulator.go_algae_prepare_intake()
            if self.manipulator.at_position():
                self.next_state_now(self.get_algae)

    @state(must_finish=True)
    def get_algae(self, initial_call, state_tm):
        if initial_call:
            self.intake_control.go_algae_intake()
            self.intimidator.prep_pp_trajectory(
                self.drivetrain.get_pose(),
                Positions.REEF_A.transformBy(Transform2d(0.0, 0, Rotation2d(0))),
                max_vel=1.5,
                max_accel=1.00,
            )
            self.intimidator.next_state_now(self.intimidator.follow_pp)
        if self.photoeye.algae_held is True or (
            self.at_pose(Positions.REEF_A) and state_tm > 1.0
        ):
            self.intake_control.go_algae_hold()
            self.next_state_now(self.score_in_barge)
    
    @state(must_finish=True)
    def score_in_barge(self, initial_call, state_tm):
        target_pose = Positions.BARGE_CENTER
        if initial_call:
            self.manipulator.set_algae_barge()
            reef_pose = Positions.REEF_A
            backup_pose = reef_pose.transformBy(Transform2d(-1.5, 0, Rotation2d(0)))
            curr_pose = self.drivetrain.get_pose()
            self.backup_pose_pub.set(backup_pose)   
            self.intimidator.prep_pp_trajectory(curr_pose, target_pose, [backup_pose], max_vel=1.0, max_accel=1.0)
            self.intimidator.next_state_now(self.intimidator.follow_pp)
        # Once we're near the barge scoring area up the elevator
        if self.at_pose(target_pose, 1.5):
            self.manipulator.go_algae_prepare_score()
        if self.at_pose(target_pose) and self.manipulator.at_position():
            self.next_state_now(self.score_algae)

    @state(must_finish=True)
    def score_algae(self, initial_call, state_tm):
        if initial_call:
            self.intake_control.go_algae_score()
        if self.photoeye.algae_held is False and state_tm > 0.75:
            self.next_state_now(self.to_away_from_startline)

    @state(must_finish=True)
    def to_away_from_startline(self, initial_call, state_tm):
        target_pose = Positions.REEF_F.transformBy(Transform2d(-2.0, 0, Rotation2d(0)))
        if initial_call:
            self.manipulator.go_home()
            self.wrist.target_pos = ManipLocations.INTAKE_CORAL.wrist_pos
            self.intimidator.go_drive_swoop(target_pose)
        if state_tm > 0.5:
            self.manipulator.go_home()

