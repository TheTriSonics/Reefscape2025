import magicbot

from wpilib import AddressableLED, DriverStation
from components.battery_monitor import BatteryMonitorComponent


def iterable(obj):
    try:
        iter(obj)
    except:  # noqa: E722
        return False
    return True


class LEDComponent:
    battery_monitor: BatteryMonitorComponent
    
    def __init__(self):
        self._lights = AddressableLED(0)  # PWM port, might be different with CAN
        self._led_length = 135
        self._rainbow_mode = False
        self._lights.setLength(self._led_length)

        led = AddressableLED.LEDData
        self._led_data = [led() for _ in range(self._led_length)]

        self.__rainbowFirstPixelHue = 0
        self.set_colorRGB((0, 255, 0))

        self._lights.setData(self._led_data)
        self._lights.start()

    def set_colorRGB(self, color, blinking=False):
        self._rainbow_mode = False
        self._blinking = blinking
        self._curr_color = color
        if iterable(color[0]):
            n = len(color)
            chunk_size = self._led_length / n
            for i in range(self._led_length):
                idx = int(i // chunk_size)
                c = color[idx]
                self._led_data[i].setRGB(*c)
        else:
            for i in range(self._led_length):
                self._led_data[i].setRGB(*color)

    def set_colorHSV(self, color, blinking=False, brightness=255):
        self._rainbow_mode = False
        self._blinking = blinking
        self._curr_color = color
        for i in range(self._led_length):
            self._led_data[i].setHSV(color, 255, brightness)

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
    def execute(self) -> None:
        if self.battery_monitor.is_stop_active():
            self.set_colorRGB((255, 0, 0), blinking=False)
        elif self.battery_monitor.is_warning_active():
            self.set_colorRGB((255, 224, 32), blinking=False)
        else:
            if DriverStation.isDSAttached():
                self.set_colorRGB((0, 255, 0))
            else:
                self.rainbow()
        
        if self._rainbow_mode:
            self._rainbow()
        self._lights.setData(self._led_data)
