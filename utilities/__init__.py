from .waypoints import Waypoints
from .game import is_red, is_sim


def norm_deg(angle) -> float:
    return (angle + 180) % 360 - 180


def norm_rad(angle) -> float:
    from math import atan2, sin, cos
    return atan2(sin(angle), cos(angle))

