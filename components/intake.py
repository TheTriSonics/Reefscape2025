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

    coral_intake_at = tunable(-1.0)
    coral_score_at = tunable(-1.0)

    force_algae_score = tunable(False)
    force_algae_intake = tunable(False)

    has_algae = tunable(False)

    algae_intake_at = tunable(-1.0)
    algae_score_at = tunable(-1.0)

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

    def update_sim(self):
        # A method we can call from execute() to move along statuses until
        # we have the real hardware. We could do this in physics.py but
        # to make it run on the physical robot we can put it here so the logic
        # can be tested while actually driving it.

        now = wpilib.Timer.getFPGATimestamp()
        robot_pose = self.drivetrain.get_pose()
        ps_tag_id, _ = Waypoints.closest_ps_tag_id(robot_pose)
        ps_tag_pose = Waypoints.get_tag_pose(ps_tag_id)
        # Now get distance between them
        ps_dist = robot_pose.translation().distance(ps_tag_pose.translation())
        pn = wpilib.SmartDashboard.putNumber
        pn('ps_dist', ps_dist)

        if self.direction == IntakeDirection.NONE:
            self.coral_intake_at = -1
            self.coral_score_at = -1
        elif self.direction == IntakeDirection.CORAL_IN:
            self.coral_score_at = -1
        elif self.direction == IntakeDirection.CORAL_SCORE:
            self.coral_intake_at = -1
        
        if self.coral_intake_at < 0 and self.direction == IntakeDirection.CORAL_IN:
            self.coral_intake_at = now
        if self.coral_score_at < 0 and self.direction == IntakeDirection.CORAL_SCORE:
            self.coral_score_at = now

        if self.coral_score_at + 0.5 > now:
            # We've been moving for 0.5 seconds so clear out any held coral
            # or algae
            self.photoeye.front_photoeye = False
            self.photoeye.back_photoeye = False
            self.coral_score_at = -1
    
        pn('algae_score', self.algae_score_at)
        pn('algae_intake_at', self.algae_intake_at)
        pn('now', now)
        if self.algae_intake_at > 0 and self.algae_intake_at + 0.5 < now and ps_dist < 0.8:
            # We've been moving for 0.5 seconds so assume we have some algae
            self.photoeye.front_photoeye = True
            self.algae_intake_at = -1
        if self.direction == IntakeDirection.NONE:
            self.algae_intake_at = -1
            self.algae_score_at = -1
        elif self.direction == IntakeDirection.ALGAE_IN:
            self.algae_score_at = -1
        elif self.direction == IntakeDirection.ALGAE_SCORE:
            self.algae_intake_at = -1
        
        if self.algae_intake_at < 0 and self.direction == IntakeDirection.ALGAE_IN:
            self.algae_intake_at = now
        if self.algae_score_at < 0 and self.direction == IntakeDirection.ALGAE_SCORE:
            self.algae_score_at = now

        if self.algae_score_at + 0.5 > now:
            # We've been moving for 0.5 seconds so clear out any held algae
            # or algae
            self.photoeye.front_photoeye = False
            self.algae_score_at = -1
    
        pn('algae_score', self.algae_score_at)
        pn('algae_intake_at', self.algae_intake_at)
        pn('now', now)
        if self.algae_intake_at > 0 and self.algae_intake_at + 0.5 < now and ps_dist < 0.8:
            # We've been moving for 0.5 seconds so assume we have some algae
            self.photoeye.front_photoeye = True
            self.algae_intake_at = -1

        if self.coral_intake_at > 0 and self.coral_intake_at + 0.5 < now and ps_dist < 0.8:
            # We've been moving for 0.5 seconds so assume we have some coral
            self.photoeye.back_photoeye = True
            self.coral_intake_at = -1
    
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

    def calc_coral_pose(self, reef_tag_id=None, force_left=False, force_right=False, height=None) -> Pose3d:
        xoff, yoff, zoff = 0.0, 0.0, 0.0
        roll, pitch, yaw = 0.0, 0.0, 0.0

        robot_pose = self.drivetrain.get_pose()
        height = self.get_current_coral_scoring_height()
        curr_loc = ManipLocation(
            self.elevator.get_position(),
            self.arm.get_position(),
            self.wrist.get_position(),
        )
        self.coral_pose_msg = f'Scoring at height {height} for {curr_loc}'
        if reef_tag_id is None:
            reef_tag_id, dist = Waypoints.closest_reef_tag_id(robot_pose)
            if dist > 1.0 or height not in [1, 2, 3, 4]:
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
        tag_pose = Waypoints.get_tag_pose(reef_tag_id)
        # Flip the pose so right/left make sense
        flip = Transform2d(Translation2d(0, 0), Rotation2d(math.pi))
        tag_pose = tag_pose.transformBy(flip)
        tag_pose_left = Waypoints.shift_reef_left(tag_pose)
        tag_pose_right = Waypoints.shift_reef_right(tag_pose)

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
        coral_pose: None | Pose3d = None
        # If we already think we have coral but the photo eye has gone false
        if self.has_coral is True and self.photoeye.back_photoeye is False:
            # Let's see if we can come up with a pose for the coral
            # Okay, we can... but it seems like the manipulator has moved away
            # from the pose it was at when it scored by the time we get here.
            # and uhh... I think I just need to put a delay in my auton to fix
            # this.
            coral_pose = self.calc_coral_pose()
            self.coral_static.append(coral_pose)
            self.has_coral = False
            self._coral_pose = None
        coral_pose = None
        if self.photoeye.back_photoeye:
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
            self.coral_pub.set(self.coral_static)

    def execute(self):
        self.do_3d_repr()
        self.update_sim()

        speed_val = 1.0

        motor_power = 0.0
        if self.force_coral_intake:
            motor_power =  speed_val
        if self.force_coral_score:
            motor_power = -speed_val
        if self.force_algae_intake:
            motor_power =  speed_val
        if self.force_algae_score:
            motor_power = -speed_val

        if self.direction in [IntakeDirection.CORAL_IN, IntakeDirection.ALGAE_SCORE]:
            motor_power = speed_val
        elif self.direction in [IntakeDirection.CORAL_SCORE, IntakeDirection.CORAL_SCORE]:
            motor_power = -speed_val

        self.motor_request.output = motor_power
        self.motor.set_control(self.motor_request)
