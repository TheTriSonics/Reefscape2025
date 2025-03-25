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


class PlaceOneClose(AutonBase):
    elevator: ElevatorComponent
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    arm: ArmComponent
    intimidator: Intimidator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    MODE_NAME = 'WMI - Super Simple Place One (select level)'
    DEFAULT = False

    scoring_level = tunable(4)
    at_pose_counter = tunable(0)

    def __init__(self):
        pass

    # The base class uses this in set_initial_pose() to set where the robot
    # thinks it should be at the beginning of an auton.  If you dont' provide
    # this method in an auton that inherits from AutonBase, it will default to
    # the origin, positon 0, 0, which is behind a player station on the blue
    # side.
    def get_initial_pose(self):
        self.photoeye.back_photoeye = True
        self.photoeye.coral_held = True
        # This doesn't really matter for this routine
        return Positions.AUTON_LINE_THEIR_CAGE_CENTER

    # Leave the initial starting position and head to the Reef to score
    @state(must_finish=True, first=True)
    def drive_to_reef(self, state_tm, initial_call):
        target_pose = Positions.REEF_CLOSEST_LEFT
        # On our first run start putting things in motion
        if initial_call:
            self.manipulator.coral_mode()
            self.manipulator.set_coral_level(self.scoring_level)
            self.intimidator.go_drive_pose(target_pose)
        # If we don't have a coral we must have scored
        if self.photoeye.coral_held is False or state_tm > 7.0:
            self.next_state(self.drive_to_safe)
        # If we're close-ish to the reef and the arm has achieved an upright
        # position then start moving the whole manipulator into place
        # It would be nice on this one if the arm didn't go below X degrees
        # until the elevator has reached a certain height.
        if self.at_pose(target_pose, 2.0):
            # Get the lift moving into the right position
            self.manipulator.go_coral_prepare_score()
        if self.at_pose(target_pose):
            self.at_pose_counter += 1
        else:
            self.at_pose_counter = 0
        if (
            self.at_pose_counter >= 5 and self.manipulator.at_position()
            and (self.photoeye.back_photoeye or self.photoeye.front_photoeye)
        ) or state_tm > 6.0:
            if self.scoring_level in [2, 3]:
                self.intake_control.go_coral_score(reverse=True)
            else:
                self.intake_control.go_coral_score()

    # Move the robot into a safe position to lower the manipulator system.
    # Quick rotation 90 degrees is the current thought.
    @state(must_finish=True)
    def drive_to_safe(self, state_tm, initial_call):
        # After this we do nothing. We just get into a safe spot
        target_pose = Positions.REEF_CLOSEST_LEFT.transformBy(Transform2d(-0.5, 0, Rotation2d.fromDegrees(90)))
        if initial_call:
            self.intimidator.go_drive_pose(target_pose, aggressive=True)

        robot_pose = self.drivetrain.get_pose()
        # Get difference in rotation between the two
        rot_diff = target_pose.relativeTo(robot_pose)
        if rot_diff.rotation().degrees() < 25 or self.at_pose(
            target_pose, tolerance=0.08
        ):
            self.manipulator.go_home()


# Base class for 'big one' autons that try and fill a face on the reef
class BigOne(AutonBase):
    elevator: ElevatorComponent
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    arm: ArmComponent
    intimidator: Intimidator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    first_face = tunable('A')
    fill_face = tunable('A')
    curr_level = tunable(4)
    curr_left = tunable(True)

    at_pose_counter = tunable(0)
    
    def __init__(self):
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
        self.intimidator.prep_pp_trajectory(curr_pose, target_pose)

    # Leave the initial starting position and head to the Reef to score
    @state(must_finish=True, first=True)
    def drive_to_reef(self, state_tm, initial_call):
        target_pose = Positions.get_facepos(self.first_face, left=True)
        # On our first run start putting things in motion
        if initial_call:
            self.manipulator.coral_mode()
            self.arm.target_pos = 90
            self.manipulator.set_coral_level4()
            self.intimidator.engage(self.intimidator.follow_pp)
            self.at_pose_counter = 0
        # If we don't have a coral we must have scored
        if self.photoeye.coral_held is False:
            self.next_state(self.back_off_reef)
        # If we're close-ish to the reef and the arm has achieved an upright
        # position then start moving the whole manipulator into place
        # It would be nice on this one if the arm didn't go below X degrees
        # until the elevator has reached a certain height.
        if self.at_pose(target_pose, 1.8):
            # Get the lift moving into the right position
            self.manipulator.go_coral_prepare_score()
        if self.at_pose(target_pose):
            self.at_pose_counter += 1
        else:
            self.at_pose_counter = 0
        
        if (
            self.at_pose_counter >= 5 and self.manipulator.at_position()
            and (self.photoeye.back_photoeye or self.photoeye.front_photoeye)
        ) or state_tm > 6.0:
            self.next_state(self.score_first_coral)
    
    @state(must_finish=True)
    def score_first_coral(self, state_tm, initial_call):
        if self.photoeye.coral_held is True:
            self.manipulator.go_coral_score()
        else:
            self.next_state(self.back_off_reef)
        if state_tm > 1.0:
            self.next_state(self.back_off_reef)


    @state(must_finish=True)
    def back_off_reef(self, state_tm, initial_call):
        self.arm.target_pos = 90
        angle_check_ok = False
        if self.arm.get_position() > 85 or angle_check_ok:
            self.elevator.target_pos = 0
        ps_pose = Positions.PS_CLOSEST
        reef_pose = Positions.REEF_CLOSEST
        backup_target = reef_pose.transformBy(Transform2d(-0.3, 0, Rotation2d(0)))
        backup_pose = Pose2d(backup_target.translation(), ps_pose.rotation())
        self.backup_pose_pub.set(backup_pose)   
        self.intimidator.go_drive_pose(backup_target)
        if initial_call:
            self.intimidator.prep_pp_trajectory(backup_pose, Positions.PS_CLOSEST, max_vel=2.5)
        # self.drivetrain.drive_to_pose(backup_target, aggressive=True)
        if self.elevator.get_position() < 20 and self.elevator.target_pos < 10:
            self.next_state(self.drive_to_ps)

    @state(must_finish=True)    
    def drive_to_ps(self, state_tm, initial_call):
        target_pose = Positions.PS_CLOSEST
        if initial_call:
            # Set the drivetrain to send us to the player station
            self.intimidator.engage(self.intimidator.follow_pp)
        angle_check_ok = False
        if self.manipulator.reef_dist() > 1.5 or angle_check_ok:
            self.manipulator.go_home()
        if self.at_pose(target_pose, 0.50):
            # Turn the intake on when we're close to the station
            self.manipulator.request_location(ManipLocations.INTAKE_CORAL)
            self.intake_control.go_coral_intake()
        if self.at_pose(target_pose, 0.10) or self.photoeye.coral_held:
            # Once we've got a coral, or we've been here for a bit we can move
            # on to the reef
            self.next_state(self.drive_back_to_reef)

    # Leave the initial starting position and head to the Reef to score
    @state(must_finish=True)
    def drive_back_to_reef(self, state_tm, initial_call):
        if self.curr_left:
            target_pose = Positions.get_facepos(self.fill_face, left=True)
        else:
            target_pose = Positions.get_facepos(self.fill_face, right=True)
        assert target_pose
        # On our first run start putting things in motion
        if initial_call:
            # self.manipulator.coral_mode()
            self.manipulator.set_coral_level(self.curr_level)
            # This immediately asks the arm to go in the 'up' position
            if self.curr_level == 4 and self.photoeye.coral_held:
                self.arm.target_pos = 90
            self.intimidator.go_drive_swoop(target_pose)
        # If we're close-ish to the reef and the arm has achieved an upright
        # position then start moving the whole manipulator into place
        # It would be nice on this one if the arm didn't go below X degrees
        # until the elevator has reached a certain height.
        if self.at_pose(target_pose, 2.0):
            # Get the lift moving into the right position if we have coral
            if self.photoeye.coral_held:
                self.manipulator.go_coral_prepare_score()
            else:
                # Go back to the player station if we didn't get that coral
                self.next_state(self.drive_to_ps)
        if self.at_pose(target_pose):
            self.at_pose_counter += 1
        else:
            self.at_pose_counter = 0
        if (
            self.at_pose_counter >= 5 and self.manipulator.at_position()
            and (self.photoeye.coral_held)
        ):  # or state_tm > 4.0:
            if self.curr_level in [1, 4]:
                self.intake_control.go_coral_score()
            elif self.curr_level in [2, 3]:
                self.intake_control.go_coral_score(reverse=True)
            self.next_state(self.await_score)

    @state(must_finish=True)
    def await_score(self, state_tm, initial_call):
        # If we don't have a coral we must have scored
        if self.photoeye.coral_held is False or state_tm > 5.0:
            self.curr_left = not self.curr_left
            if self.curr_left:
                self.curr_level -= 1
            if self.curr_level == 0:
                self.next_state(self.wee)
                return
            self.next_state(self.drive_to_ps)


class BigOneLeft(BigOne):
    MODE_NAME = 'WMI - Big One Left'
    DEFAULT = False

    first_face = tunable('F')
    fill_face = tunable('E')

    def __init__(self) -> None:
        super().__init__()


class BigOneRight(BigOne):
    MODE_NAME = 'WMI - Big One Right'
    DEFAULT = False

    first_face = tunable('B')
    fill_face = tunable('C')

    def get_initial_pose(self):
        self.photoeye.back_photoeye = True
        self.photoeye.coral_held = True
        return Positions.AUTON_LINE_THEIR_CAGE_CENTER

    def __init__(self) -> None:
        super().__init__()


class BigOneLeft3(BigOne):
    MODE_NAME = 'WMI - Big One Left Level 3'
    DEFAULT = False

    first_face = tunable('F')
    fill_face = tunable('E')
    curr_level = tunable(3)

    def __init__(self) -> None:
        super().__init__()


class BigOneRight3(BigOne):
    MODE_NAME = 'WMI - Big One Right Level 3'
    DEFAULT = False

    first_face = tunable('B')
    fill_face = tunable('C')
    curr_level = tunable(3)

    def get_initial_pose(self):
        self.photoeye.back_photoeye = True
        self.photoeye.coral_held = True
        return Positions.AUTON_LINE_THEIR_CAGE_CENTER

    def __init__(self) -> None:
        super().__init__()
