from wpilib import XboxController
from hid.reefscape_driver_base import ReefscapeDriverBase


class ReefscapeDriver(ReefscapeDriverBase):

    def __init__(self, port: int):
        super().__init__(port)

    def has_extended_mode(self) -> bool:
        return False

    def returnToHomeLocation(self) -> bool:
        return False

    def getRightX(self) -> float:
        return self.getRawAxis(4)

    def getRightY(self) -> float:
        return self.getRawAxis(5)

    def getReefLeft(self) -> bool:
        return self.getRawButton(5)

    def getReefRight(self) -> bool:
        return self.getRawButton(6)

    def getReefAlgae(self) -> bool:
        return self.getRawButton(5) and self.getRawButton(6)

    def getToWallTarget(self) -> bool:
        return self.getAButton()

    def getFieldReset(self) -> bool:
        return self.getRawButton(7)

    def getDriveLocal(self) -> bool:
        return self.getRawButton(8)