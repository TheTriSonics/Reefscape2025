from wpilib import XboxController
from hid.reefscape_driver_base import ReefscapeDriverBase


class ReefscapeDriver(ReefscapeDriverBase):

    def __init__(self, port: int):
        super().__init__(port)

    def getLeftX(self) -> float:
        return self.getRawAxis(0)
    
    def getLeftY(self) -> float:
        return self.getRawAxis(1)

    def getRightX(self) -> float:
        return self.getRawAxis(5)

    def getReefLeft(self) -> bool:
        return self.getRawButton(1) and self.getPOV() == 270

    def getReefRight(self) -> bool:
        return self.getRawButton(1) and self.getPOV() == 90

    def getReefAlgae(self) -> bool:
        return self.getRawButton(1) and self.getPOV() == -1

    def getToWallTarget(self) -> bool:
        return self.getRawButton(4)

    def getFieldReset(self) -> bool:
        return self.getRawButton(11)

    def getDriveLocal(self) -> bool:
        return self.getRawButton(12)

    def getHeightPlacement4(self) -> bool:
        return self.getRawButton(5)

    def getHeightPlacement3(self) -> bool:
        return self.getRawButton(6)

    def getHeightPlacement2(self) -> bool:
        return self.getRawButton(7)

    def getHeightPlacement1(self) -> bool:
        return self.getRawButton(8)

    def has_extended_mode(self) -> bool:
        return True

    # Here are things I would not expect on a regular XBox controller
    def goHome(self) -> bool:
        return self.getRawButton(15)

    def getCoralMode(self) -> bool:
        return self.getRawAxis(2) < -0.25

    def getAlgaeMode(self) -> bool:
        return self.getRawAxis(2) < 0.25

    def getManipulatorAdvance(self) -> bool:
        return self.getRawButtonPressed(9)
