from wpilib import RobotController
from magicbot import tunable, feedback
# A completely made up subsystem that pretends it can change out the
# robot's battery.


class BatteryMonitorComponent:
    warning_voltage = tunable(11.5)
    stop_voltage = tunable(11.0)

    warning_active = tunable(False)
    stop_active = tunable(False)

    def __init__(self):
        pass

    def set_warning_voltage(self, v: float) -> None:
        self.warning_voltage = v

    def set_stop_voltage(self, v: float) -> None:
        self.stop_voltage = v

    @feedback
    def get_current_voltage(self) -> float:
        voltage = RobotController.getBatteryVoltage()
        return voltage

    def is_warning_active(self) -> bool:
        return self.warning_active
    
    def is_stop_active(self) -> bool:
        return self.stop_active

    def execute(self):
        v = self.get_current_voltage()
        if v < self.warning_voltage:
            self.warning_active = True
        else:
            self.warning_active = False
        
        if v < self.stop_voltage:
            self.stop_active = True
        else:
            self.stop_active = False
