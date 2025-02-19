"""Descriptions of the field and match state."""

import enum
import math

import robotpy_apriltag
import wpilib
from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Translation2d,
    Translation3d,
)

apriltag_layout = robotpy_apriltag.AprilTagFieldLayout.loadField(
    robotpy_apriltag.AprilTagField.k2025ReefscapeWelded
)

APRILTAGS = apriltag_layout.getTags()

FIELD_WIDTH = apriltag_layout.getFieldWidth()
FIELD_LENGTH = apriltag_layout.getFieldLength()


# TODO: write functions for rotational symmetry


def field_flip_pose2d(p: Pose2d):
    return Pose2d(
        field_flip_translation2d(p.translation()),
        field_flip_rotation2d(p.rotation()),
    )


def field_flip_translation3d(t: Translation3d):
    return Translation3d(FIELD_LENGTH - t.x, t.y, t.z)


def field_flip_rotation2d(r: Rotation2d):
    return Rotation2d(-r.cos(), r.sin())


def field_flip_angle(r: float):
    return math.atan2(math.sin(math.pi - r), math.cos(math.pi - r))


def field_flip_translation2d(t: Translation2d):
    return Translation2d(FIELD_LENGTH - t.x, FIELD_WIDTH - t.y)


# This will default to the blue alliance if a proper link to the driver station has not yet been established
def is_red() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed


def is_sim() -> bool:
    return wpilib.RobotBase.isSimulation()


def is_auton() -> bool:
    mode = wpilib.SmartDashboard.getString('/robot/mode', '')
    return mode in ['auto']


def is_disabled() -> bool:
    mode = wpilib.SmartDashboard.getString('/robot/mode', '')
    return mode in ['disabled', '']


class GamePieces(enum.Enum):
    CORAL = enum.auto()
    ALGAE = enum.auto()


class ManipLocation:
    """
    This Location class provides a handy wrapper to the positions of the
    components we need to move to.
    """
    wrist_pos: float
    arm_pos: float
    elevator_pos: float

    def __init__(self, elevator, arm, wrist):
        self.elevator_pos = elevator
        self.arm_pos = arm
        self.wrist_pos = wrist

    def within_tolerance(self, other, tol=0.5):
        ediff = abs(self.elevator_pos - other.elevator_pos)
        adiff = abs(self.arm_pos - other.arm_pos)
        wdiff = abs(self.wrist_pos - other.wrist_pos)
        max_diff = max(ediff, adiff, wdiff)
        # if tol != 0.5:
        #     print('max diff', max_diff)
        return max_diff < tol

    def __eq__(self, other):
        return self.within_tolerance(other, tol=0.5)

    def __sub__(self, other):
        return ManipLocation(
            self.elevator_pos - other.elevator_pos,
            self.arm_pos - other.arm_pos,
            self.wrist_pos - other.wrist_pos,
        )

    def __repr__(self):
        """ 
        This is what's used to turn the object into a string; handy if you
        want to print it out for debugging, or put it on a smart dashboard
        """
        return f'ManipLocation(elevator={self.elevator_pos}, arm={self.arm_pos}, wrist={self.wrist_pos})'
    

class ManipLocations:
    """
    And we can define what is basicaly another enumeration, but with a Location
    object for a datatype, not an integer.
    """
    # Order of params is elevator, arm, wrist, just as in the Location's
    # __init__ method
    HOME = ManipLocation(0, -80, 135) 
    CORAL_REEF_1 = ManipLocation(0, -60, 75)
    CORAL_REEF_2 = ManipLocation(0, -20, -12)
    CORAL_REEF_3 = ManipLocation(0, 35, -52)
    CORAL_REEF_4 = ManipLocation(12, 50, -130) 
    ALGAE_REEF_1 = ManipLocation(0, -30, -90)
    ALGAE_REEF_2 = ManipLocation(9, -30, -90)

    #changed from processor to processor_5 and barge to barge_6
    PROCESSOR_5 = ManipLocation(0, -60, 200)
    BARGE_6 = ManipLocation(20, 45, 160)

