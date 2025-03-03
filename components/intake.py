import math
import ntcore
import wpilib
from wpimath.geometry import Pose3d, Translation3d, Rotation3d, Pose2d, Translation2d, Transform3d, Transform2d, Rotation2d
from magicbot import feedback, tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import (
    DutyCycleOut,
)
from enum import Enum

from components.photoeye import PhotoEyeComponent
from components import (
    ElevatorComponent, WristComponent, ArmComponent, DrivetrainComponent
)
from utilities import Waypoints, is_sim
from utilities.game import ManipLocations, ManipLocation
from ids import TalonId

pn = wpilib.SmartDashboard.putNumber


class IntakeDirection(Enum):
    NONE = 0
    CORAL_IN = 1
    CORAL_SCORE = 2
    ALGAE_SCORE = 3
    ALGAE_IN = 4


class IntakeComponent:
    elevator: ElevatorComponent
    wrist: WristComponent
    arm: ArmComponent
    photoeye: PhotoEyeComponent
    drivetrain: DrivetrainComponent
    motor = TalonFX(TalonId.MANIP_INTAKE.id, TalonId.MANIP_INTAKE.bus)

    force_coral_score = tunable(False)
    force_coral_intake = tunable(False)

    coral_pose_msg = tunable('')

    has_coral = tunable(False)
    direction_int = tunable(0)

    force_algae_score = tunable(False)
    force_algae_intake = tunable(False)

    motor_request = DutyCycleOut(0, override_brake_dur_neutral=True)
    direction = IntakeDirection.NONE

    def __init__(self):
        self.coral_pub = (ntcore.NetworkTableInstance.getDefault()
                            .getStructArrayTopic('/components/intake/coral_locs', Pose3d)
                            .publish())
        self.algae_pub = (ntcore.NetworkTableInstance.getDefault()
                            .getStructArrayTopic('/components/intake/algae_locs', Pose3d)
                            .publish())
        self._coral_pose = None
        from phoenix6 import configs
        limit_configs = configs.CurrentLimitsConfigs()
        # enable stator current limit to keep algae from falling out when
        # the motor is trying to keep it in
        limit_configs.stator_current_limit = 12
        limit_configs.stator_current_limit_enable = True
        self.motor.configurator.apply(limit_configs)

    def setup(self):
        tfl = Waypoints.get_tag_id_from_letter
        self.coral_static: list[Pose3d] = [
        ]
        self.algae_static: list[Pose3d] = [
            self.calc_algae_pose(tfl('A', True), height=1),
            self.calc_algae_pose(tfl('B', True), height=2),
            self.calc_algae_pose(tfl('C', True), height=1),
            self.calc_algae_pose(tfl('D', True), height=2),
            self.calc_algae_pose(tfl('E', True), height=1),
            self.calc_algae_pose(tfl('F', True), height=2),

            self.calc_algae_pose(tfl('A', False), height=1),
            self.calc_algae_pose(tfl('B', False), height=2),
            self.calc_algae_pose(tfl('C', False), height=1),
            self.calc_algae_pose(tfl('D', False), height=2),
            self.calc_algae_pose(tfl('E', False), height=1),
            self.calc_algae_pose(tfl('F', False), height=2),
        ]
        self.algae_pub.set(self.algae_static)

    def coral_in(self):
        # TODO: This might not always mean forward, but for now
        # we'll keep it simple
        self.direction = IntakeDirection.CORAL_IN

    def algae_in(self):
        # TODO: This might not always mean forward, but for now
        # we'll keep it simple
        self.direction = IntakeDirection.ALGAE_IN

    def score_coral(self):
        self.direction = IntakeDirection.CORAL_SCORE

    def score_coral_reverse(self):
        self.direction = IntakeDirection.CORAL_IN

    def score_algae(self):
        self.direction = IntakeDirection.ALGAE_SCORE

    def intake_off(self):
        self.direction = IntakeDirection.NONE

    def pose2d_to_pose3d(self, pose2d: Pose2d, 
                        x_offset: float = 0.3,  # Forward offset in meters
                        y_offset: float = 0, 
                        z_offset: float = 1.0) -> Pose3d:
        """
        Convert a Pose2d to Pose3d with offsets relative to robot's orientation.
        
        Args:
            pose2d: The robot's current Pose2d
            x_offset: Forward offset in meters (positive is forward)
            y_offset: Left/right offset in meters (positive is left)
            z_offset: Up/down offset in meters (positive is up)
            
        Returns:
            Pose3d: The transformed 3D pose
        """
        # Get the robot's current rotation
        current_rotation = pose2d.rotation()
        
        # Create offset vector in robot's local coordinates
        local_offset = Translation2d(x_offset, y_offset)
        
        # Rotate the offset by the robot's rotation to get global coordinates
        rotated_offset = local_offset.rotateBy(current_rotation)
        
        # Add the rotated offset to the robot's position
        final_x = pose2d.X() + rotated_offset.X()
        final_y = pose2d.Y() + rotated_offset.Y()
        
        # Create the 3D pose with the calculated position and z offset
        # Use the 2D rotation for the Z axis rotation in 3D
        return Pose3d(
            Translation3d(final_x, final_y, z_offset),
            Rotation3d(0, 0, current_rotation.radians())
        )

    @feedback
    def get_current_coral_scoring_height(self) -> int:
        curr_loc = ManipLocation(
            self.elevator.get_position(),
            self.arm.get_position(),
            self.wrist.get_position(),
        )
        if curr_loc.within_tolerance(ManipLocations.CORAL_REEF_4, 2):
            return 4
        if curr_loc.within_tolerance(ManipLocations.CORAL_REEF_3, 2):
            return 3
        if curr_loc.within_tolerance(ManipLocations.CORAL_REEF_2, 2):
            return 2 
        if curr_loc.within_tolerance(ManipLocations.CORAL_REEF_1, 2):
            return 1 
        # print('Current location:', curr_loc)
        return -1
    
    def get_current_algae_scoring_height(self) -> int:
        curr_loc = ManipLocation(
            self.elevator.get_position(),
            self.arm.get_position(),
            self.wrist.get_position(),
        )
        if curr_loc == ManipLocations.PROCESSOR_5:
            return 5
        if curr_loc == ManipLocations.BARGE_6:
            return 6
        return -1
    
    def get_current_algae_intake_height(self) -> int:
        curr_loc = ManipLocation(
            self.elevator.get_position(),
            self.arm.get_position(),
            self.wrist.get_position(),
        )
        if curr_loc == ManipLocations.ALGAE_REEF_1:
            return 1
        if curr_loc == ManipLocations.ALGAE_REEF_2:
            return 2
        return -1

    def calc_algae_pose(self, reef_tag_id=None, height=1) -> Pose3d:
        zoffset = 0.7 if height == 1 else 1.1

        base_pose = Waypoints.get_tag_pose(reef_tag_id)
        tag_3d_pose = Pose3d(
            Translation3d(base_pose.X(), base_pose.Y(), 0.2),
            Rotation3d(0, 0, base_pose.rotation().radians()),
        )
        coral_pose = tag_3d_pose.transformBy(
            Transform3d(Translation3d(-0.1, 0, zoffset),
                        Rotation3d.fromDegrees(0, 0, 0))
        )
        return coral_pose

    def execute(self):
        self.direction_int = self.direction.value
        speed_val = 0.3

        motor_power = 0.0
        if self.force_coral_intake:
            motor_power = speed_val
        if self.force_coral_score:
            motor_power = -speed_val
        if self.force_algae_intake:
            motor_power = speed_val
        if self.force_algae_score:
            motor_power = -speed_val

        if self.direction in [IntakeDirection.CORAL_IN, IntakeDirection.ALGAE_SCORE]:
            motor_power = speed_val
        elif (
            self.direction in [IntakeDirection.CORAL_SCORE, IntakeDirection.ALGAE_IN]
        ):
            motor_power = -speed_val

        self.motor_request.output = motor_power
        self.motor.set_control(self.motor_request)
