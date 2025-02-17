import math
import ntcore
import wpilib
from wpimath.geometry import Pose3d, Translation3d, Rotation3d, Pose2d, Translation2d, Transform3d
from magicbot import feedback, tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import (
    DutyCycleOut,
    MotionMagicVoltage,
    DynamicMotionMagicVoltage,
)
from phoenix6.configs import TalonFXConfiguration
from enum import Enum
from ids import TalonId

from components import (
    ElevatorComponent, WristComponent, ArmComponent, PhotoEyeComponent, DrivetrainComponent
)
from utilities.game import is_sim
from utilities.waypoints import *

pn = wpilib.SmartDashboard.putNumber


class IntakeDirection(Enum):
    NONE = 0
    CORAL_IN = 1
    CORAL_SCORE = 2


class IntakeComponent:
    elevator: ElevatorComponent
    wrist: WristComponent
    arm: ArmComponent
    photoeye: PhotoEyeComponent
    drivetrain: DrivetrainComponent
    bus = 'canivore'
    # motor = TalonFX(TalonId.MANIP_INTAKE, bus)

    force_coral_score = tunable(False)
    force_coral_intake = tunable(False)

    auto_coral_intake = tunable(True)

    has_coral = tunable(False)

    x_off = tunable(0.0)
    y_off = tunable(0.0)
    z_off = tunable(1.6)

    roll = tunable(0.0)
    pitch = tunable(90.0)
    yaw = tunable(0.0)

    at_height = tunable(4)

    motor_request = DutyCycleOut(0, override_brake_dur_neutral=True)
    direction = IntakeDirection.NONE


    def __init__(self):
        self.coral_pub = (ntcore.NetworkTableInstance.getDefault()
                            .getStructArrayTopic('corals', Pose3d)
                            .publish())

        self._coral_pose = None

    def setup(self):
        self.coral_static: list[Pose3d] = [
        ]
        old_data = """
            self.calc_coral_pose(6, force_right=True, height=4),
            self.calc_coral_pose(6, force_right=True, height=3),
            self.calc_coral_pose(6, force_right=True, height=2),
            self.calc_coral_pose(6, force_right=True, height=1),
            self.calc_coral_pose(6, force_left=True, height=4),
            self.calc_coral_pose(6, force_left=True, height=3),
            self.calc_coral_pose(6, force_left=True, height=2),
            self.calc_coral_pose(6, force_left=True, height=1),
            """

    def coral_in(self):
        # TODO: This might not always mean forward, but for now
        # we'll keep it simple
        self.direction = IntakeDirection.CORAL_IN

    def score_coral(self):
        self.direction = IntakeDirection.CORAL_SCORE

    def intake_off(self):
        print('intake off called by somebody')
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

    def calc_coral_pose(self, reef_tag_id=None, force_left=False, force_right=False, height=None) -> Pose3d:
        xoff, yoff, zoff = 0.0, 0.0, 0.0
        roll, pitch, yaw = 0.0, 0.0, 0.0

        robot_pose = self.drivetrain.get_pose()
        height = height or self.at_height
        if reef_tag_id is None:
            reef_tag_id, dist = closest_reef_tag_id(robot_pose)
            if dist > 1.0:
                # Just drop it on the floor!
                robot_3d_pose = Pose3d(
                    Translation3d(robot_pose.X(), robot_pose.Y(), 0.2),
                    Rotation3d(0, 0, robot_pose.rotation().radians()),
                )
                coral_pose = robot_3d_pose.transformBy(
                    Transform3d(Translation3d(0.5, 0, 0.0),
                                Rotation3d.fromDegrees(0, 45, 0))
                )
                return coral_pose
        tag_pose = get_tag_pose(reef_tag_id)
        # Flip the pose so right/left make sense
        flip = Transform2d(Translation2d(0, 0), Rotation2d(math.pi))
        tag_pose = tag_pose.transformBy(flip)
        tag_pose_left = shift_reef_left(tag_pose)
        tag_pose_right = shift_reef_right(tag_pose)

        # If the distance from robot pose to left is less than to the right
        # then we'll use the left pose
        base_pose = tag_pose_right
        if robot_pose.translation().distance(tag_pose_left.translation()) < robot_pose.translation().distance(tag_pose_right.translation()):
            base_pose = tag_pose_left
        
        if force_left:
            base_pose = tag_pose_left
        elif force_right:
            base_pose = tag_pose_right

        tag_3d_pose = Pose3d(
            Translation3d(base_pose.X(), base_pose.Y(), 0.2),
            Rotation3d(0, 0, base_pose.rotation().radians()),
        )

        if height == 1:
            xoff = 0.20
            zoff = 0.35
            pitch = -30
        elif height == 2:
            xoff = 0.10
            zoff = 0.55
            pitch = 30
        elif height == 3:
            xoff = 0.10
            zoff = 0.95
            pitch = 30
        elif height == 4:
            xoff = 0.075
            zoff = 1.6
            pitch = 90

        coral_pose = tag_3d_pose.transformBy(
            Transform3d(Translation3d(xoff, yoff, zoff),
                        Rotation3d.fromDegrees(roll, pitch, yaw))
        )
        return coral_pose

    def do_3d_repr(self):
        # If we already think we have coral but the photo eye has gone false
        if self.has_coral is True and self.photoeye.coral_held is False:
            # Let's see if we can come up with a pose for the coral
            coral_pose = self.calc_coral_pose()
            self.coral_static.append(coral_pose)
            self.has_coral = False
            self._coral_pose = None
        coral_pose = None
        if self.photoeye.coral_held:
            self.has_coral = True
            robot_pose = self.drivetrain.get_pose()
            coral_pose = self.pose2d_to_pose3d(robot_pose)
            # Now move it around baseed on what the robot's doing
            z_offset = self.elevator.get_position() * 0.05
            z_offset += math.sin(math.radians(self.arm.get_position())) * 0.5
            coral_rotation = math.radians(self.wrist.get_position() + self.arm.get_position())
            coral_pose = coral_pose.transformBy(
                Transform3d(Translation3d(0, 0, z_offset),
                            Rotation3d(0, coral_rotation, math.pi))
            )
            self._coral_pose = coral_pose
        else:
            self.has_coral = False
            self._coral_pose = None
        
        if coral_pose:
            self.coral_pub.set(self.coral_static + [coral_pose])
        elif False:
            tag_id = get_tag_id_from_letter('A', True)
            tag_pose = shift_reef_left(get_tag_pose(tag_id))
            x = tag_pose.X() + self.x_off
            y = tag_pose.Y() + self.y_off
            z = self.z_off

            tpose = Pose3d(Translation3d(x, y, z), Rotation3d.fromDegrees(self.roll, self.pitch, self.yaw))
            self.coral_pub.set(self.coral_static + [tpose])
        else:
            self.coral_pub.set(self.coral_static)

    def execute(self):
        self.do_3d_repr()

        motor_power = 0.0
        if self.force_coral_intake:
            motor_power = 0.2
        if self.force_coral_score:
            motor_power = -0.2
        
        if self.photoeye.coral_chute:
            self.direction = IntakeDirection.CORAL_IN

        if self.direction == IntakeDirection.CORAL_IN:
            motor_power = 0.2
        elif self.direction == IntakeDirection.CORAL_SCORE:
            motor_power = -0.2
        
        if is_sim():
            motor_power *= 10 

        self.motor_request.output = motor_power
        # self.motor.set_control(self.motor_request)
