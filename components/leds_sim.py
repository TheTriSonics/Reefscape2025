import wpilib
from wpilib import Color8Bit
from components.leds import LEDComponent


class LEDSim:
    leds: LEDComponent

    def __init__(self):
        grey = Color8Bit(128, 128, 128)
        # Create a 2D mechanism for the Manipulator
        self.lights2d = wpilib.Mechanism2d(2, 2)
        # Create the base structure
        self.base = self.lights2d.getRoot("base", 0.8, -0.0)
        self.base_pole = self.base.appendLigament(
            "base_pole", 0.5, 90, 8, color=grey
        )
        # Create elevator tower
        self.light_manip_state = self.base_pole.appendLigament(
            "manip_state", 0.1, 0, 8
        )

        self.spacer_1 = self.light_manip_state.appendLigament(
            "spacer_1", 0.02, 0, 8, color=grey
        )
        
        self.light_manip_level = self.spacer_1.appendLigament(
            "manip_level", 0.1, 0, 8
        )

        self.spacer_2 = self.light_manip_level.appendLigament(
            "spacer_2", 0.02, 0, 8, color=grey
        )
        
        self.light_pe_status = self.spacer_2.appendLigament(
            "pe_status", 0.1, 0, 8
        )

        self.spacer_3 = self.light_pe_status.appendLigament(
            "spacer_3", 0.02, 0, 8, color=grey
        )
        
        self.light_intake_status = self.spacer_3.appendLigament(
            "intake_status", 0.1, 0, 8
        )

        self.spacer_4 = self.light_intake_status.appendLigament(
            "spacer_4", 0.02, 0, 8, color=grey
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