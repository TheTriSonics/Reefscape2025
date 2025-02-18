import wpilib
from wpilib import Color8Bit, MechanismLigament2d
from components.leds import LEDComponent


class LEDSim:
    leds: LEDComponent

    def __init__(self):
        pass

    def setup(self):
        grey = Color8Bit(128, 128, 128)
        # Create a 2D mechanism for the Manipulator
        self.lights2d = wpilib.Mechanism2d(2, 2)
        # Create the base structure
        self.base = self.lights2d.getRoot("base", 0.8, 0.0)
        self.base_pole = self.base.appendLigament(
            "base_pole", 0.5, 90, 8, color=grey
        )
        led_height = 0.005
        spacer_height = 0.0005
        self.lights: list[MechanismLigament2d] = []
        prev_base = self.base_pole
        for idx, _ in enumerate(self.leds._led_data):
            led = prev_base.appendLigament(
                "led", led_height, 0, 8
            )
            self.lights.append(led)
            spacer = led.appendLigament(
                f"spacer_{idx}", spacer_height, 0, 8, color=grey
            )
            prev_base = spacer

        # Send to SmartDashboard
        wpilib.SmartDashboard.putData("LightPole", self.lights2d)

    def execute(self):
        for light, src_led in zip(self.lights, self.leds._led_data):
            light.setColor(Color8Bit(src_led.r, src_led.g, src_led.b))