import enum


@enum.unique
class TalonId(enum.IntEnum):
    """CAN ID for CTRE Talon motor controllers (e.g. Talon FX, Talon SRX)."""

    DRIVE_FR = 12
    DRIVE_FL = 11
    DRIVE_BL = 14
    DRIVE_BR = 13

    TURN_FR = 22
    TURN_FL = 21
    TURN_BL = 24
    TURN_BR = 23

    MANIP_ELEVATOR_LEFT = 46
    MANIP_ELEVATOR_RIGHT = 47
    MANIP_ARM = 43
    MANIP_WRIST = 44
    MANIP_INTAKE = 45


@enum.unique
class CancoderId(enum.IntEnum):
    """CAN ID for CTRE CANcoder."""

    SWERVE_FR = 32
    SWERVE_FL = 34
    SWERVE_BL = 33
    SWERVE_BR = 31


@enum.unique
class CanId(enum.IntEnum):
    """CAN IDs for miscellaneous devices."""

    PIGEON = 41


@enum.unique
class SparkId(enum.IntEnum):
    """CAN ID for REV SPARK motor controllers (Spark Max, Spark Flex)."""

    INTAKE_MOTOR_FEED = 51
    AMP_FEED_MOTOR = 55

    CLIMBER_MOTOR_LEFT = 57
    CLIMBER_MOTOR_RIGHT = 58

    SHOOTER_MOTOR_FEED_RIGHT = 46
    SHOOTER_MOTOR_FEED_LEFT = 47

    SHOOTER_MOTOR_TILT_LEFT = 44
    SHOOTER_MOTOR_TILT_RIGHT = 45


@enum.unique
class DioChannel(enum.IntEnum):
    """roboRIO Digital I/O channel number."""


@enum.unique
class PwmChannel(enum.IntEnum):
    """roboRIO PWM output channel number."""
