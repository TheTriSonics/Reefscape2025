from wpimath.geometry import Rotation2d
from phoenix6.hardware import Pigeon2
from ids import CanId
from magicbot import feedback


class GyroComponent:
    def __init__(self):
        self.pigeon = Pigeon2(CanId.PIGEON.id, CanId.PIGEON.bus)

    # TODO: Set this to heading=None
    # then use:
    # heading = 180 if is_red() else 0
    def reset_heading(self, heading=0.0) -> None:
        self.pigeon.set_yaw(heading)

    @feedback
    def get_heading(self) -> float:
        return self.pigeon.get_yaw().value - 3.0

    def get_Rotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.get_heading())

    def execute(self) -> None:
        pass
