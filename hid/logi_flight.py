from wpilib import XboxController


class ReefscapeDriver(XboxController):

    def __init__(self, port: int):
        super().__init__(port)

    def has_extended_moded(self) -> bool:
        return False

    def _master_throttle(self) -> float:
        return (-self.getRawAxis(3) + 1) / 2

    def getLeftX(self) -> float:
        return self.getRawAxis(0) * self._master_throttle()
    
    def getLeftY(self) -> float:
        return self.getRawAxis(1) * self._master_throttle()

    def getRightX(self) -> float:
        return self.getRawAxis(2) * self._master_throttle()

    def getReefLeft(self) -> bool:
        return self.getRawButton(3)

    def getReefRight(self) -> bool:
        return self.getRawButton(4)

    def getReefAlgae(self) -> bool:
        return self.getRawButton(1)

    def getToWallTarget(self) -> bool:
        return self.getRawButton(2)

    def getFieldReset(self) -> bool:
        return self.getRawButton(7)

    def getDriveLocal(self) -> bool:
        return self.getRawButton(8)

