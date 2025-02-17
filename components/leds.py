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
        self._led_length = 4
        self._rainbow_mode = False
        self._lights.setLength(self._led_length)

        led = AddressableLED.LEDData
        self._led_data = [led() for _ in range(self._led_length)]

        self.__rainbowFirstPixelHue = 0
        for l in self._led_data:
            l.setRGB(0, 0, 0)
        self._lights.start()

    def set_intake_color(self, r, g, b):
        self._led_data[3].setRGB(r, g, b)

    def get_intake_color(self):
        color = self._led_data[3]
        return color.r, color.g, color.b

    def set_pe_color(self, r, g, b):
        self._led_data[2].setRGB(r, g, b)

    def get_pe_color(self):
        color = self._led_data[2]
        return color.r, color.g, color.b

    def set_manip_level_color(self, r, g, b):
        self._led_data[1].setRGB(r, g, b)

    def get_manip_level_color(self):
        color = self._led_data[1]
        return color.r, color.g, color.b

    def set_manip_state_color(self, r, g, b):
        self._led_data[0].setRGB(r, g, b)

    def get_manip_state_color(self):
        color = self._led_data[0]
        return color.r, color.g, color.b

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

    def execute(self) -> None:
        if not DriverStation.isDSAttached():
            self.rainbow()
        else:
            if self.intake.direction == IntakeDirection.CORAL_IN:
                self.set_intake_color(*self.yellow)
            elif self.intake.direction == IntakeDirection.CORAL_SCORE:
                self.set_intake_color(*self.magenta)
            else:
                self.set_intake_color(*self.cyan)

            if self.photoeye.coral_chute:
                self.set_pe_color(*self.cyan)
            elif self.photoeye.coral_held:
                self.set_pe_color(*self.green)
            else:
                self.set_pe_color(*self.white)

            lvl = self.manipulator.coral_scoring_target
            match lvl:
                case Locations.CORAL_REEF_1:
                    self.set_manip_level_color(*self.red)
                case Locations.CORAL_REEF_2:
                    self.set_manip_level_color(*self.yellow)
                case Locations.CORAL_REEF_3:
                    self.set_manip_level_color(*self.green)
                case Locations.CORAL_REEF_4:
                    self.set_manip_level_color(*self.blue)
                case _:
                    self.set_manip_level_color(*self.white)

            ms = self.manipulator.current_state
            if ms == self.manipulator.idling:
                self.set_manip_state_color(*self.blue)
            elif ms == self.manipulator.coral_intake:
                self.set_manip_state_color(*self.yellow)
            elif ms == self.manipulator.coral_score:
                self.set_manip_state_color(*self.magenta)
            else:
                self.set_manip_state_color(*self.white)
            
        self._lights.setData(self._led_data)
