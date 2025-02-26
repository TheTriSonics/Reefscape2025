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


# Auton routine that scores two algae in the processor, like setting up for
# the coopertition bonus.
class AutonCoop(AutonBase):
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    intimidator: Intimidator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    MODE_NAME = 'Sample - Coopertition'
    DEFAULT = True

    def __init__(self):
        pass

    # The base class uses this in set_initial_pose() to set where the robot
    # thinks it should be at the beginning of an auton.  If you dont' provide
    # this method in an auton that inherits from AutonBase, it will default to
    # the origin, positon 0, 0, which is behind a player station on the blue
    # side,
    def get_initial_pose(self):
        self.photoeye.coral_held = False
        return Positions.AUTON_LINE_CENTER

    # Clear name of the state objective
    @state(must_finish=True, first=True)
    def intake_first_algae(self, state_tm, initial_call):
        target_pose = Positions.REEF_A
        # Now the general pattern. On the first call we might have to
        # kick some other state machines into motion
        if initial_call:
            # Do we need to drive? Get that going.
            self.intimidator.go_drive_swoop(target_pose)
            # Set up the manipulator for algae intake
            self.manipulator.algae_mode()
            self.manipulator.set_algae_level1()
        # As we get closer to our destination we might want to kick off
        # other events.
        if self.at_pose(target_pose, 0.10):
            # Get the lift moving into the right position
            self.manipulator.go_algae_prepare_intake()
        if self.at_pose(target_pose, 0.08) and self.manipulator.at_position():
            # Once at position switch on the algae intake.
            self.intake_control.go_algae_intake()
        # We might want to swap this for a conditont that asks the 
        # intake controller if the intake was successful
        if self.photoeye.algae_held is True:
            # If we've got the algae move to scoring the thing!
            self.next_state(self.score_first_algae)

    @state(must_finish=True)
    def score_first_algae(self, tm, state_tm, initial_call):
        target_pose = Positions.PROCESSOR
        if initial_call:
            self.intimidator.go_drive_swoop(target_pose)
            self.manipulator.set_algae_processor()
            self.manipulator.go_algae_prepare_score()
        if self.at_pose(target_pose, tolerance=0.08) and self.manipulator.at_position():
            self.intake_control.go_algae_score()
        if self.photoeye.algae_held is False:
            self.manipulator.go_home()
            self.next_state(self.intake_second_algae)
    
    @state(must_finish=True)
    def intake_second_algae(self, tm, state_tm, initial_call):
        target_pose = Positions.REEF_B
        if initial_call:
            self.intimidator.go_drive_swoop(target_pose)
            self.manipulator.set_algae_level2()
        if self.at_pose(target_pose, tolerance=0.8):
            self.manipulator.go_algae_prepare_intake()
        if self.at_pose(target_pose, tolerance=0.1) and self.manipulator.at_position():
            self.intake_control.go_algae_intake()
        if self.photoeye.algae_held is True:
            self.next_state(self.score_second_algae)

    @state(must_finish=True)
    def score_second_algae(self, initial_call):
        target_pose = Positions.PROCESSOR
        if initial_call:
            self.manipulator.set_algae_processor()
            self.manipulator.go_algae_prepare_score()
            self.intimidator.go_drive_swoop(target_pose)
        if self.at_pose(target_pose, tolerance=0.08) and self.manipulator.at_position():
            self.intake_control.go_algae_score()
        if self.photoeye.algae_held is False:
            self.manipulator.go_home()
            self.next_state(self.intake_first_coral)

    @state(must_finish=True)
    def intake_first_coral(self, tm, state_tm, initial_call):
        target_pose = Positions.PS_LEFT
        if initial_call:
            self.manipulator.go_home()
            self.intimidator.go_drive_swoop(target_pose)
        if self.at_pose(target_pose, tolerance=0.1):
            self.intake_control.go_coral_intake()
        if self.photoeye.coral_held is True:
            self.next_state(self.score_first_coral)

    @state(must_finish=True)
    def score_first_coral(self, tm, state_tm, initial_call):
        target_pose = Positions.REEF_CLOSEST_LEFT
        if initial_call:
            self.manipulator.go_home()
            self.manipulator.set_coral_level1()
            self.intimidator.go_drive_swoop(target_pose)
        if self.at_pose(target_pose, tolerance=0.88):
            self.manipulator.go_coral_prepare_score()
        if self.at_pose(target_pose, tolerance=0.08) and self.manipulator.at_position():
            self.intake_control.go_coral_score()
        if self.photoeye.coral_held is False:
            self.next_state(self.intake_second_coral)

    @state(must_finish=True)
    def intake_second_coral(self, tm, state_tm, initial_call):
        target_pose = Positions.PS_RIGHT
        if initial_call:
            self.manipulator.go_home()
            self.intimidator.go_drive_swoop(target_pose)
        if self.at_pose(target_pose, tolerance=0.1):
            self.intake_control.go_coral_intake()
        if self.photoeye.coral_held is True:
            self.next_state(self.score_second_coral)

    @state(must_finish=True)
    def score_second_coral(self, tm, state_tm, initial_call):
        target_pose = Positions.REEF_CLOSEST_RIGHT
        if initial_call:
            self.manipulator.go_home()
            self.manipulator.set_coral_level2()
            self.intimidator.go_drive_swoop(target_pose)
        if self.at_pose(target_pose, tolerance=0.88):
            self.manipulator.go_coral_prepare_score()
        if self.at_pose(target_pose, tolerance=0.08) and self.manipulator.at_position():
            self.intake_control.go_coral_score()
        if self.photoeye.coral_held is False:
            self.next_state(self.wee)

    # @state(must_finish=True)
    # def wee(self):
    #     self.intimidator.go_drive_strafe_fixed(1.8)
    #     self.manipulator.set_algae_barge()
    #     self.manipulator.go_algae_prepare_score()


class AutonStrafe(AutonBase):
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    intimidator: Intimidator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    MODE_NAME = 'Testing - Strafe'
    DEFAULT = False

    def __init__(self):
        pass

    def get_initial_pose(self):
        return Positions.REEF_D

    @state(must_finish=True, first=True)
    def strafe(self, tm, state_tm, initial_call):
        if initial_call:
            self.intimidator.go_drive_strafe_fixed(2.0)

class AutonStrafeToFace(AutonBase):
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    intimidator: Intimidator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    MODE_NAME = 'Testing - Strafe to Face'
    DEFAULT = False

    def __init__(self):
        pass

    def get_initial_pose(self):
        return Positions.REEF_D

    @state(must_finish=True, first=True)
    def strafe(self, tm, state_tm, initial_call):
        if initial_call:
            self.intimidator.go_drive_strafe_reef_face('A')
        if state_tm > 3.0:
            self.intimidator.go_drive_strafe_reef_face('D')
        if state_tm > 6.0:
            self.intimidator.go_drive_strafe_reef_face('F')