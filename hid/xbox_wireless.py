from wpilib import XboxController
from hid.reefscape_controller_base import ReefscapeControllerBase


class ReefscapeDriver(ReefscapeControllerBase):

    def __init__(self, port: int):
        super().__init__(port)

    def has_extended_mode(self) -> bool:
        return False

    def goHome(self) -> bool:
        return False

    def getRightX(self) -> float:
        return self.getRawAxis(2)

    def getReefLeft(self) -> bool:
        return self.getRawButton(7)

    def getReefRight(self) -> bool:
        return self.getRawButton(8)

    def getReefAlgae(self) -> bool:
        return self.getRawButton(7) and self.getRawButton(8)

    def getToWallTarget(self) -> bool:
        return self.getAButton()

    def getFieldReset(self) -> bool:
        return self.getRawButton(11)

    def getDriveLocal(self) -> bool:
        return self.getRawButton(12)