from wpilib import XboxController


class ReefscapeDriver(XboxController):

    def __init__(self, port: int):
        super().__init__(port)

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



