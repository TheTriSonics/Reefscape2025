# File for all of the choreo related paths
import math
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d
from magicbot import AutonomousStateMachine, state, feedback, tunable

from utilities.game import is_red, is_sim
from utilities.position import Positions

from controllers.manipulator import Manipulator
from controllers.intimidator import Intimidator
from controllers.intake import IntakeControl

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from components.battery_monitor import BatteryMonitorComponent
from components.photoeye import PhotoEyeComponent

pb = SmartDashboard.putBoolean
pn = SmartDashboard.putNumber
ps = SmartDashboard.putString


# Auton routine that scores two algae in the processor, like setting up for
# the coopertition bonus.
class AutonBase(AutonomousStateMachine):
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    battery_monitor: BatteryMonitorComponent
    manipulator: Manipulator
    intimidator: Intimidator
    intake_control: IntakeControl
    photoeye: PhotoEyeComponent

    pose_set = False
    selected_alliance = None

    initial_pose = None
    failure_pose = None

    pose_check = tunable(False)
    pose_error = tunable(0.0)

    def __init__(self):
        pass

    def get_initial_pose(self):
        return Pose2d(0, 0, 0)

    def set_initial_pose(self) -> None:
        # No need to set the pose twice!
        alliance = 'red' if is_red() else 'blue'
        if alliance is not self.selected_alliance:
            self.pose_set = False

        if self.pose_set is True:
            return

        initial_pose = self.get_initial_pose()
        assert initial_pose
        self.drivetrain.set_pose(initial_pose)
        self.selected_alliance = alliance
        self.pose_set = True
    
    # In the event that this isn't overrided the robot will just sit there
    # if it is told to follow a trajectory they forgot to prepare
    def prepare_first_trajectory(self):
        curr_pose = self.drivetrain.get_pose()
        self.intimidator.prep_pp_trajectory_auto(curr_pose, curr_pose)

    def at_pose(self, pose: Pose2d, tolerance=None) -> bool:
        if tolerance is None:
            tolerance = 0.15 if is_sim() else 0.040
        robot_pose = self.drivetrain.get_pose()
        diff = robot_pose.relativeTo(pose)
        dist = math.sqrt(diff.X()**2 + diff.Y()**2)
        self.pose_error = dist
        self.pose_check = dist < tolerance
        return self.pose_check

    @state(must_finish=True)
    def wee(self, initial_call):
        pass  # Do nothing

    @state(must_finish=True)
    def robot_failure(self):
        from wpimath.geometry import Pose3d, Transform3d, Rotation3d, Translation3d
        if self.failure_pose is None:
            self.failure_pose = self.drivetrain.get_pose()
        # Now we are going to push back a robot pose that is upside down
        # so that it publishes a wonky pose on the telemetry stream.
        full_pose = Pose3d(self.failure_pose)
        full_pose = (
            full_pose
            .transformBy(Transform3d(
                Translation3d(0, 0, 1.5),
                Rotation3d(0, math.pi, 0))
            )
        )

    def execute(self):
        pose = self.drivetrain.get_pose()
        Positions.update_dynamic_positions(pose)
        super().execute()