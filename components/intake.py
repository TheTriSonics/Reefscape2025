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

        self.coral_static: list[Pose3d] = [
            Pose3d(Translation3d(7, 5, 2), Rotation3d(0, 0, 0)),
        ]
        self._coral_pose = None

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

    def calc_coral_pose(self) -> Pose3d:
        xoff, yoff, zoff = 0.0, 0.0, 0.0
        roll, pitch, yaw = 0.0, 0.0, 0.0

        robot_pose = self.drivetrain.get_pose()
        reef_tag_id, dist = closest_reef_tag_id(robot_pose)
        tag_pose_left = shift_reef_left(get_tag_pose(reef_tag_id))
        tag_pose_right = shift_reef_right(get_tag_pose(reef_tag_id))

        # If the distance from robot pose to left is less than to the right
        # then we'll use the left pose
        base_pose = tag_pose_right
        if robot_pose.translation().distance(tag_pose_left.translation()) < robot_pose.translation().distance(tag_pose_right.translation()):
            base_pose = tag_pose_left

        tag_3d_pose = Pose3d(
            Translation3d(base_pose.X(), base_pose.Y(), 0.2),
            Rotation3d(0, 0, base_pose.rotation().radians()),
        )
        # For level 3:
        # x 0.15
        # y 0.045
        # z 1.1
        # pitch 30

        # for level 2 drop the z to: 0.75:w
        # for level 1 drop to: 0.5, pitch is -30
        # level 4 is 1.8, x is 0.05, 90 deg on the pitch
        # Let's mock up some that would be on the reef as if they're scored

        if self.at_height == 1:
            yoff = 0.045
            zoff = 0.5
            pitch = -30
        elif self.at_height == 2:
            yoff = 0.045
            zoff = 0.75
            pitch = 30
        elif self.at_height == 3:
            yoff = 0.045
            zoff = 1.1
            pitch = 30
        elif self.at_height == 4:
            xoff = 0.05
            yoff = 0.045
            zoff = 1.7
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
        else:
            tag_id = get_tag_id_from_letter('A', True)
            tag_pose = shift_reef_left(get_tag_pose(tag_id))
            x = tag_pose.X() + self.x_off
            y = tag_pose.Y() + self.y_off
            z = self.z_off

            tpose = Pose3d(Translation3d(x, y, z), Rotation3d.fromDegrees(self.roll, self.pitch, self.yaw))
            self.coral_pub.set(self.coral_static + [tpose])

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
