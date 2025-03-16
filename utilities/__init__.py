from .waypoints import Waypoints
from .position import Positions
from .game import is_red, is_sim, is_match

import wpilib


pn = wpilib.SmartDashboard.putNumber
ps = wpilib.SmartDashboard.putString
pb = wpilib.SmartDashboard.putBoolean


def norm_deg(angle) -> float:
    return (angle + 180) % 360 - 180


def norm_rad(angle) -> float:
    from math import atan2, sin, cos
    return atan2(sin(angle), cos(angle))

