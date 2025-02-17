import wpilib
from wpilib import Color8Bit
from components.leds import LEDComponent


class LEDSim:
    leds: LEDComponent

    def __init__(self):
        # Create a 2D mechanism for the Manipulator
        self.lights2d = wpilib.Mechanism2d(2, 2)
        # Create the base structure
        self.base = self.lights2d.getRoot("base", 0.8, -0.0)
        # Create elevator tower
        self.light_manip_state = self.base.appendLigament(
            "manip_state", 0.6, 90, 8
        )
        
        self.light_manip_level = self.light_manip_state.appendLigament(
            "manip_level", 0.1, 0, 8
        )
        
        self.light_pe_status = self.light_manip_level.appendLigament(
            "pe_status", 0.1, 0, 8
        )

        self.light_intake_status = self.light_pe_status.appendLigament(
            "intake_status", 0.1, 0, 8
        )

        # Send to SmartDashboard
        wpilib.SmartDashboard.putData("LightPole", self.lights2d)

    def execute(self):
        r, g, b = self.leds.get_manip_state_color()
        self.light_manip_state.setColor(Color8Bit(r, g, b))

        r, g, b = self.leds.get_manip_level_color()
        self.light_manip_level.setColor(Color8Bit(r, g, b))

        r, g, b = self.leds.get_pe_color()
        self.light_pe_status.setColor(Color8Bit(r, g, b))

        r, g, b = self.leds.get_intake_color()
        self.light_intake_status.setColor(Color8Bit(r, g, b))