import enum
from dataclasses import dataclass


@dataclass
class CANDevice:
    id: int
    bus: str

drive = 'Drive'
manipulator = 'Arm'

class TalonId():
    # Drive Bus
    DRIVE_FL = CANDevice(11, drive)
    DRIVE_FR = CANDevice(12, drive)
    DRIVE_BR = CANDevice(13, drive)
    DRIVE_BL = CANDevice(14, drive)

    TURN_FL = CANDevice(21, drive)
    TURN_FR = CANDevice(22, drive)
    TURN_BR = CANDevice(23, drive)
    TURN_BL = CANDevice(24, drive)

    CLIMB_ONE = CANDevice(51, drive)
    CLIMB_TWO = CANDevice(52, drive)

    MANIP_ELEVATOR_LEFT = CANDevice(61, drive)
    MANIP_ELEVATOR_RIGHT = CANDevice(62, drive)

    # Manipulator / Arm bus
    MANIP_ARM = CANDevice(21, manipulator)
    MANIP_WRIST = CANDevice(22, manipulator)
    MANIP_INTAKE = CANDevice(23, manipulator)


class CancoderId:
    SWERVE_FL = CANDevice(31, drive)
    SWERVE_FR = CANDevice(32, drive)
    SWERVE_BR = CANDevice(33, drive)
    SWERVE_BL = CANDevice(34, drive)

    MANIP_ARM = CANDevice(31, manipulator)
    MANIP_WRIST = CANDevice(32, manipulator)


class CanId:
    """CAN IDs for miscellaneous devices."""
    PIGEON = CANDevice(41, drive)
    CANDI = CANDevice(41, manipulator)
