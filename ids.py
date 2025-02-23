import enum
from dataclasses import dataclass


@dataclass
class CANDevice:
    id: int
    bus: str

drive = 'Drive'
manipulator = 'Arm'

class TalonId():
    """CAN ID for CTRE Talon motor controllers (e.g. Talon FX, Talon SRX)."""

    DRIVE_FR = CANDevice(12, drive)
    DRIVE_FL = CANDevice(11, drive)
    DRIVE_BL = CANDevice(14, drive)
    DRIVE_BR = CANDevice(13, drive)

    TURN_FR = CANDevice(22, drive)
    TURN_FL = CANDevice(21, drive)
    TURN_BL = CANDevice(24, drive)
    TURN_BR = CANDevice(23, drive)

    MANIP_ELEVATOR_LEFT = CANDevice(60, manipulator)
    MANIP_ELEVATOR_RIGHT = CANDevice(61, manipulator)
    MANIP_ARM = CANDevice(52, manipulator)
    MANIP_WRIST = CANDevice(50, manipulator)
    MANIP_INTAKE = CANDevice(51, manipulator)


class CancoderId:
    SWERVE_FR = CANDevice(32, drive)
    SWERVE_FL = CANDevice(34, drive)
    SWERVE_BL = CANDevice(33, drive)
    SWERVE_BR = CANDevice(31, drive)


class CanId:
    """CAN IDs for miscellaneous devices."""
    PIGEON = CANDevice(41, drive)
    MANIP_ARM_ENCODER = CANDevice(33, manipulator)
    MANIP_WRIST_ENCODER = CANDevice(34, manipulator)
    CANDI = CANDevice(36, manipulator)
