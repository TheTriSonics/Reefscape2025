import wpilib
import magicbot

from wpilib import AddressableLED, DriverStation
from wpilib.simulation import AddressableLEDSim
from components.battery_monitor import BatteryMonitorComponent
from components.intake import IntakeDirection, IntakeComponent
from components.photoeye import PhotoEyeComponent
from controllers.manipulator import Manipulator, Locations


def iterable(obj):
    try:
        iter(obj)
    except:  # noqa: E722
        return False
    return True


class LEDComponent:
    battery_monitor: BatteryMonitorComponent
    intake: IntakeComponent
    photoeye: PhotoEyeComponent
    manipulator: Manipulator
    
    def __init__(self):
        self._lights = AddressableLED(0)  # PWM port, might be different with CAN
        self._led_length = 100
        self._rainbow_mode = False
        self._lights.setLength(self._led_length)

        led = AddressableLED.LEDData
        self._led_data = [led() for _ in range(self._led_length)]

        self.__rainbowFirstPixelHue = 0
        for l in self._led_data:
            l.setRGB(0, 0, 0)
        self._lights.start()

    def _rainbow(self) -> None:
        # For every pixel
        for i in range(self._led_length):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            hue = int(
                self.__rainbowFirstPixelHue + (i * 180 / self._led_length)
            ) % 180
            # Set the value
            self._led_data[i].setHSV(hue, 255, 128)
        # Increase by to make the rainbow "move"
        self.__rainbowFirstPixelHue += 3
        # Check bounds
        self.__rainbowFirstPixelHue %= 180

    def rainbow(self) -> None:
        # Kick the system into rainbow mode.
        self._rainbow_mode = True

    # Here we can determine what the robot should be doing.
    red = (255, 0, 0)
    green = (0, 255, 0)
    blue = (0, 0, 255)
    magenta = (255, 0, 255)
    cyan = (0, 255, 255)
    orange = (255, 165, 0)
    purple = (128, 0, 128)
    yellow = (255, 255, 0)
    white = (255, 255, 255)
    black = (0, 0, 0)

    def setColor(self, r, g, b) -> None:
        for l in self._led_data:
            l.setRGB(r, g, b)

    def execute(self) -> None:
        if not DriverStation.isDSAttached():
            self.rainbow()
        if self._rainbow_mode:
            self._rainbow()

        manip = self.manipulator

        if (self.intake.direction != IntakeDirection.NONE or
            manip.current_state == manip.coral_prepare_score.name):
            self.setColor(*self.yellow)
        elif (self.photoeye.coral_held):
            self.setColor(*self.green)
        self._lights.setData(self._led_data)
