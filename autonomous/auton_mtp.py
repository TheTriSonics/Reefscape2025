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

    MODE_NAME = 'Super Simple Place One (select level)'
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
        if self.photoeye.coral_held is False:
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
        ) or state_tm > 8.0:
            if self.scoring_level in [2, 3]:
                self.intake_control.go_coral_score(reverse=True)
            else:
                self.intake_control.go_coral_score()

    # Move the robot into a safe position to lower the manipulator system.
    # Quick rotation 90 degrees is the current thought.
    @state(must_finish=True)
    def drive_to_safe(self, state_tm, initial_call):
        # After this we do nothing. We just get into a safe spot
        from wpimath.geometry import Transform2d, Rotation2d
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


class AutonMountPleasantE(AutonBase):
    elevator: ElevatorComponent
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    arm: ArmComponent
    intimidator: Intimidator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    MODE_NAME = 'The Big One - Place at F, then fill E'
    DEFAULT = True

    curr_level = 2
    curr_left = True 

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
        return Positions.AUTON_LINE_OUR_CAGE_CENTER

    # Leave the initial starting position and head to the Reef to score
    @state(must_finish=True, first=True)
    def drive_to_reef(self, state_tm, initial_call):
        target_pose = Positions.REEF_F_LEFT
        # On our first run start putting things in motion
        if initial_call:
            self.manipulator.coral_mode()
            self.arm.target_pos = 90
            self.manipulator.set_coral_level4()
            # self.intimidator.go_drive_swoop(target_pose)
            self.intimidator.go_drive_pose(target_pose)
            self.at_pose_counter = 0
        # If we don't have a coral we must have scored
        if self.photoeye.coral_held is False:
            self.next_state(self.drive_to_a_safe)
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
            self.intake_control.go_coral_score()

    # Move the robot into a safe position to lower the manipulator system.
    # Quick rotation 90 degrees is the current thought.
    @state(must_finish=True)
    def drive_to_a_safe(self, state_tm, initial_call):
        from wpimath.geometry import Transform2d, Rotation2d
        target_pose = Positions.REEF_F_LEFT.transformBy(Transform2d(-0.5, 0, Rotation2d.fromDegrees(90)))
        if initial_call:
            self.intimidator.go_drive_pose(target_pose, aggressive=True)
        robot_pose = self.drivetrain.get_pose()
        # Get difference in rotation between the two
        rot_diff = target_pose.relativeTo(robot_pose)
        if rot_diff.rotation().degrees() < 25:
            self.manipulator.go_home()
            self.next_state(self.drive_to_ps)


    @state(must_finish=True)    
    def drive_to_ps(self, state_tm, initial_call):
        target_pose = Positions.PS_CLOSEST
        if initial_call:
            # Set the drivetrain to send us to the player station
            self.intimidator.go_drive_swoop(target_pose)
        if self.manipulator.reef_dist() > 1.7:
            self.manipulator.go_home()
        if self.at_pose(target_pose, 0.50):
            # Turn the intake on when we're close to the station
            self.manipulator.request_location(ManipLocations.INTAKE_CORAL)
            self.intake_control.go_coral_intake()
            if self.photoeye.coral_held:
                # Once we've got a coral we can move on
                self.next_state(self.drive_back_to_reef)

    # Leave the initial starting position and head to the Reef to score
    @state(must_finish=True)
    def drive_back_to_reef(self, state_tm, initial_call):
        target_pose = Positions.REEF_D_LEFT if self.curr_left else Positions.REEF_D_RIGHT
        # On our first run start putting things in motion
        if initial_call:
            self.manipulator.coral_mode()
            self.manipulator.set_coral_level(self.curr_level)
            # This immediately asks the arm to go in the 'up' position
            if self.curr_level == 4:
                self.arm.target_pos = 90
            self.intimidator.go_drive_pose(target_pose, aggressive=True)
        # If we don't have a coral we must have scored
        if self.photoeye.coral_held is False:
            self.curr_left = not self.curr_left
            if self.curr_left:
                self.curr_level -= 1
            if self.curr_level == 0:
                self.next_state(self.wee)
                return
            self.next_state(self.drive_to_ps)
        # If we're close-ish to the reef and the arm has achieved an upright
        # position then start moving the whole manipulator into place
        # It would be nice on this one if the arm didn't go below X degrees
        # until the elevator has reached a certain height.
        if self.at_pose(target_pose, 3.0):
            # Get the lift moving into the right position
            self.manipulator.go_coral_prepare_score()
        if self.at_pose(target_pose):
            self.at_pose_counter += 1
        else:
            self.at_pose_counter = 0
        if (
            self.at_pose_counter >= 5 and self.manipulator.at_position()
            and (self.photoeye.coral_held)
        ) or state_tm > 4.0:
            if self.curr_level in [1, 4]:
                self.intake_control.go_coral_score()
            elif self.curr_level in [2, 3]:
                self.intake_control.go_coral_score(reverse=True)
