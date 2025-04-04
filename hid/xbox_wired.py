from wpilib import XboxController

from hid.reefscape_driver_base import ReefscapeDriverBase, ReefscapeOperatorBase


class ReefscapeDriver(ReefscapeDriverBase):

    def __init__(self, port: int):
        super().__init__(port)

    def returnToHomeLocation(self) -> bool:
        return False

    def getReefLeft(self) -> bool:
        return self.getLeftBumper()

    def getReefRight(self) -> bool:
        return self.getRightBumper()

    def getReefAlgae(self) -> bool:
        return self.getRightBumper() and self.getLeftBumper()

    def getToWallTarget(self) -> bool:
        return self.getAButton()

    def getFieldReset(self) -> bool:
        return self.getRawButton(7)

    def getDriveLocal(self) -> bool:
        return self.getRawButton(8)
    
    # TODO: add strafe buttons
    # TODO: add hanger buttons
    # TODO: add dpad wall buttons


class ReefscapeOperator(ReefscapeOperatorBase):

    def goHome(self) -> bool:
        return self.getYButtonPressed()
    
    def getManipulatorAdvance(self) -> bool:
        return self.getAButtonPressed()

    def getCoralMode(self) -> bool:
        return self.getRightBumperButtonPressed()
    
    def getAlgaeMode(self) -> bool:
        return self.getLeftBumperButtonPressed()
    
    def getHeightPlacement1(self) -> bool:
        return self.getPOV() == 180
    
    def getHeightPlacement2(self) -> bool:
        return self.getPOV() == 90
    
    def getHeightPlacement3(self) -> bool:
        return self.getPOV() == 270
    
    def getHeightPlacement4(self) -> bool:
        return self.getPOV() == 0