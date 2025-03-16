import wpilib
from components import (
    ElevatorComponent, WristComponent, ArmComponent, IntakeComponent
)
from utilities import is_sim


class ManipulatorSim:
    elevator: ElevatorComponent
    arm: ArmComponent
    wrist: WristComponent
    intake: IntakeComponent

    def __init__(self):
        # Create a 2D mechanism for the Manipulator
        self.mech2d = wpilib.Mechanism2d(1, 3)
        # Create the base structure
        self.base = self.mech2d.getRoot("base", 0.5, 0)
        # Create elevator tower
        self.elevator_tower = self.base.appendLigament(
            "elevator_tower", 
            0,  # Length of tower
            90   # Straight up
        )
        
        # Create arm attached to elevator
        self.arm_mech = self.elevator_tower.appendLigament(
            "arm",
            0.4,   # Arm length
            0,    # Initial angle
            4     # Line weight
        )
        
        red = wpilib.Color8Bit(255, 0, 0)
        green = wpilib.Color8Bit(0, 255, 0)
        # Create wrist at end of arm
        self.wrist_mech_coral = self.arm_mech.appendLigament(
            "wrist_coral",
            0.15,    # Wrist length
            0,    # Initial angle
            3,    # Line weight
            color=red
        )
        
        self.wrist_mech_algae = self.arm_mech.appendLigament(
            "wrist_algae",
            0.15,    # Wrist length
            0,    # Initial angle
            3,    # Line weight
            color=green
        )

        self.intake_algae_arrow1 = self.wrist_mech_algae.appendLigament(
            "algae_arrow1",
            0.15,    # Wrist length
            -135,    # Initial angle
            3,    # Line weight
            color=green
        )

        self.intake_algae_arrow2 = self.wrist_mech_algae.appendLigament(
            "algae_arrow2",
            0.15,    # Wrist length
            135,    # Initial angle
            3,    # Line weight
            color=green
        )

        self.intake_coral_arrow1 = self.wrist_mech_coral.appendLigament(
            "coral_arrow1",
            0.15,    # Wrist length
            -45,    # Initial angle
            3,    # Line weight
            color=red
        )

        self.intake_coral_arrow2 = self.wrist_mech_coral.appendLigament(
            "coral_arrow2",
            0.15,    # Wrist length
            45,    # Initial angle
            3,    # Line weight
            color=red
        )
        
        # Send to SmartDashboard
        wpilib.SmartDashboard.putData("Manipulator", self.mech2d)

    def update_mech_sim(self):
        """Update mechanism visualization with current positions"""
        # Get current positions
        elevator_height = self.elevator.get_position()
        # Adjusted for how we think of our coordinates
        arm_angle = self.arm.get_position() - 90
        wrist_angle = self.wrist.get_position()

        # Update the mechanism's values with robot values
        self.elevator_tower.setLength(0.8 + (elevator_height / 60))
        self.arm_mech.setAngle(arm_angle)
        self.wrist_mech_coral.setAngle(wrist_angle)
        self.wrist_mech_algae.setAngle(wrist_angle + 180)

        # Now figure out what the arrows indicating intake should be doing
        intake_v = self.intake.motor_request.output   
        # This gets us the raw voltage applied to the motor. We can use this
        # to scale the size of the arrows.
        if intake_v > 0:
            self.intake_algae_arrow1.setLength(intake_v / 10)
            self.intake_algae_arrow2.setLength(intake_v / 10)
        elif intake_v < 0:
            self.intake_coral_arrow1.setLength(intake_v / 10)
            self.intake_coral_arrow2.setLength(intake_v / 10)
        else:
            self.intake_algae_arrow1.setLength(0)
            self.intake_algae_arrow2.setLength(0)
            self.intake_coral_arrow1.setLength(0)
            self.intake_coral_arrow2.setLength(0)

    def execute(self):
        if is_sim():
            self.update_mech_sim()