
from wpilib import XboxController


class ReefscapeDriverBase(XboxController):

    def getReefLeft(self) -> bool:
        return False

    def getReefRight(self) -> bool:
        return False

    def getReefAlgae(self) -> bool:
        return False

    def getToWallTarget(self) -> bool:
        return False

    def getFieldReset(self) -> bool:
        return False

    def getDriveLocal(self) -> bool:
        return False

    def getHeightPlacement4(self) -> bool:
        return False

    def getHeightPlacement3(self) -> bool:
        return False

    def getHeightPlacement2(self) -> bool:
        return False

    def getHeightPlacement1(self) -> bool:
        return False

    def has_extended_mode(self) -> bool:
        return False

    # Here are things I would not expect on a regular XBox controller
    def returnToHomeLocation(self) -> bool:
        return False

    def getCoralMode(self) -> bool:
        return False

    def getAlgaeMode(self) -> bool:
        return False

    def getManipulatorAdvance(self) -> bool:
        return False
    
    def getSnap(self) -> bool:
        return False

    def goHome(self) -> bool:
        return False


class ReefscapeOperatorBase(XboxController):

    def goHome(self) -> bool:
        return self.getAButton()

    def getCoralMode(self) -> bool:
        return False

    def getAlgaeMode(self) -> bool:
        return False

    def getManipulatorAdvance(self) -> bool:
        return self.getYButton()