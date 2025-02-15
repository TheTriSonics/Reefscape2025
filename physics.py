from __future__ import annotations

import math
import typing

import phoenix6
import phoenix6.unmanaged
import wpilib
import robotpy_apriltag
from photonlibpy.simulation import PhotonCameraSim, SimCameraProperties, VisionSystemSim
from pyfrc.physics.core import PhysicsInterface
from wpimath.geometry import Transform3d
from wpilib.simulation import DCMotorSim
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import kilogram_square_meters

from components.drivetrain import SwerveModule
from components import IntakeComponent
from components.intake import IntakeDirection

from generated.tuner_constants import TunerConstants

if typing.TYPE_CHECKING:
    from robot import MyRobot


class SimpleTalonFXMotorSim:
    def __init__(
        self, motor: phoenix6.hardware.TalonFX, units_per_rev: float, kV: float
    ) -> None:
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.kV = kV  # volt seconds per unit
        self.units_per_rev = units_per_rev

    def update(self, dt: float) -> None:
        voltage = self.sim_state.motor_voltage
        velocity = voltage / self.kV  # units per second
        velocity_rps = velocity * self.units_per_rev
        self.sim_state.set_rotor_velocity(velocity_rps)
        self.sim_state.add_rotor_position(velocity_rps * dt)


class Falcon500MotorSim:
    def __init__(
        self,
        *motors: phoenix6.hardware.TalonFX,
        # Reduction between motor and encoder readings, as output over input.
        # If the mechanism spins slower than the motor, this number should be
        # greater than one.
        gearing: float,
        moi: kilogram_square_meters,
    ):
        self.falcon = DCMotor.falcon500(len(motors))
        self.plant = LinearSystemId.DCMotorSystem(self.falcon, moi, gearing)
        self.gearing = gearing
        self.sim_states = [motor.sim_state for motor in motors]
        for sim_state in self.sim_states:
            sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, self.falcon)

    def update(self, dt: float) -> None:
        voltage = self.sim_states[0].motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        motor_rev_per_mechanism_rad = self.gearing / math.tau
        for sim_state in self.sim_states:
            sim_state.set_raw_rotor_position(
                self.motor_sim.getAngularPosition() * motor_rev_per_mechanism_rad
            )
            sim_state.set_rotor_velocity(
                self.motor_sim.getAngularVelocity() * motor_rev_per_mechanism_rad
            )


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller
        self.robot = robot
        self.cancoder_test_offset = 0

        self.kinematics: SwerveDrive4Kinematics = robot.drivetrain.kinematics
        self.swerve_modules: tuple[
            SwerveModule, SwerveModule, SwerveModule, SwerveModule
        ] = robot.drivetrain.modules
        
        self.apriltag_layout = robotpy_apriltag.AprilTagFieldLayout.loadField(
            robotpy_apriltag.AprilTagField.k2025Reefscape
        )

        # Motors
        self.wheels = [
            SimpleTalonFXMotorSim(
                module.drive,
                units_per_rev=1 / 0.0503,
                kV=2.7,
            )
            for module in robot.drivetrain.modules
        ]
        self.steer = [
            Falcon500MotorSim(
                module.steer,
                gearing=1 / TunerConstants._steer_gear_ratio,
                # measured from MKCad CAD
                moi=0.0009972,
            )
            for module in robot.drivetrain.modules
        ]

        self.manip_motors = []

        """
        if hasattr(robot, "intake"):
            self.intake_motor = Falcon500MotorSim(
                robot.intake.motor, gearing=1, moi=0.001
            )
            self.manip_motors.append(self.intake_motor)

        if hasattr(robot, "wrist"):
            self.wrist_motor = Falcon500MotorSim(
                robot.wrist.motor, gearing=25, moi=0.00001
            )
            self.manip_motors.append(self.wrist_motor)
        
        if hasattr(robot, "arm"):
            self.arm_motor = Falcon500MotorSim(
                robot.arm.motor, gearing=25, moi=0.00001
            )
            self.manip_motors.append(self.arm_motor)
    
        if hasattr(robot, "elevator"):
            self.elevator_motor_left = Falcon500MotorSim(
                robot.elevator.motor_left, gearing=1, moi=0.00001
            )
            self.manip_motors.append(self.elevator_motor_left)

            
            self.elevator_motor_right = Falcon500MotorSim(
                robot.elevator.motor_right, gearing=1, moi=0.00001
            )
            self.manip_motors.append(self.elevator_motor_right)
        """
        
        # Create a 2D mechanism for the Manipulator
        self.mech2d = wpilib.Mechanism2d(25, 30)
        # Create the base structure
        self.base = self.mech2d.getRoot("base", 12.5, 0)
        # Create elevator tower
        self.elevator_tower = self.base.appendLigament(
            "elevator_tower", 
            0,  # Length of tower
            90   # Straight up
        )
        
        # Create arm attached to elevator
        self.arm_mech = self.elevator_tower.appendLigament(
            "arm",
            8,   # Arm length
            0,    # Initial angle
            4     # Line weight
        )
        
        red = wpilib.Color8Bit(255, 0, 0)
        green = wpilib.Color8Bit(0, 255, 0)
        # Create wrist at end of arm
        self.wrist_mech_coral = self.arm_mech.appendLigament(
            "wrist_coral",
            3,    # Wrist length
            0,    # Initial angle
            3,    # Line weight
            color=red
        )
        
        self.wrist_mech_algae = self.arm_mech.appendLigament(
            "wrist_algae",
            3,    # Wrist length
            0,    # Initial angle
            3,    # Line weight
            color=green
        )

        self.intake_algae_arrow1 = self.wrist_mech_algae.appendLigament(
            "algae_arrow1",
            2,    # Wrist length
            -135,    # Initial angle
            3,    # Line weight
            color=green
        )

        self.intake_algae_arrow2 = self.wrist_mech_algae.appendLigament(
            "algae_arrow2",
            2,    # Wrist length
            135,    # Initial angle
            3,    # Line weight
            color=green
        )

        self.intake_coral_arrow1 = self.wrist_mech_coral.appendLigament(
            "coral_arrow1",
            2,    # Wrist length
            -45,    # Initial angle
            3,    # Line weight
            color=red
        )

        self.intake_coral_arrow2 = self.wrist_mech_coral.appendLigament(
            "coral_arrow2",
            2,    # Wrist length
            45,    # Initial angle
            3,    # Line weight
            color=red
        )
        
        # Send to SmartDashboard
        wpilib.SmartDashboard.putData("Manipulator", self.mech2d)

        self.current_yaw = 0.0
        self.gyro = robot.gyro.pigeon.sim_state  # Access the Pigeon 2's sim state
        self.gyro.set_supply_voltage(12.0)  # Set the supply voltage for simulation

        # Photoeye status variables
        self.pe_coral_chute_triggerd_at: float = -1.0
        self.pe_coral_held_at: float = -1.0

        self.intake_coral_in_at: float = -1.0
        self.intake_coral_out_at: float = -1.0

        # self.imu = SimDeviceSim("navX-Sensor", 4)
        # self.imu_yaw = self.imu.getDouble("Yaw")

        self.vision_sim = VisionSystemSim("ardu_cam-1")
        self.vision_sim.addAprilTags(self.apriltag_layout)
        properties = SimCameraProperties.OV9281_1280_720()
        self.camera_fl = PhotonCameraSim(robot.vision.camera_fl, properties)
        self.camera_fl.setMaxSightRange(5.0)
        self.vision_sim.addCamera(
            self.camera_fl,
            self.robot.vision.camera_fl_offset,
        )
        self.camera_center = PhotonCameraSim(robot.vision.camera_center, properties)
        self.camera_center.setMaxSightRange(5.0)
        self.vision_sim.addCamera(
            self.camera_center,
            self.robot.vision.camera_center_offset,
        )

    def update_mech_sim(self):
        if self.robot.manipulator is None:
            return
        """Update mechanism visualization with current positions"""
        # Get current positions
        elevator_height = self.robot.elevator.get_position()
        # Adjusted for how we think of our coordinates
        arm_angle = self.robot.arm.get_position() - 90
        wrist_angle = self.robot.wrist.get_position()

        # Update the mechanism's values with robot values
        self.elevator_tower.setLength(elevator_height / 2 + 15)
        self.arm_mech.setAngle(arm_angle)
        self.wrist_mech_coral.setAngle(wrist_angle)
        self.wrist_mech_algae.setAngle(wrist_angle + 180)

        # Now figure out what the arrows indicating intake should be doing
        intake_v = self.robot.intake.motor.sim_state.motor_voltage
        # This gets us the raw voltage applied to the motor. We can use this
        # to scale the size of the arrows.
        if intake_v > 0:
            self.intake_algae_arrow1.setLength(intake_v)
            self.intake_algae_arrow2.setLength(intake_v)
        elif intake_v < 0:
            self.intake_coral_arrow1.setLength(intake_v)
            self.intake_coral_arrow2.setLength(intake_v)
        else:
            self.intake_algae_arrow1.setLength(0)
            self.intake_algae_arrow2.setLength(0)
            self.intake_coral_arrow1.setLength(0)
            self.intake_coral_arrow2.setLength(0)

    def update_sim(self, now: float, tm_diff: float) -> None:
        # Enable the Phoenix6 simulated devices
        # TODO: delete when phoenix6 integrates with wpilib
        if wpilib.DriverStation.isEnabled():
            phoenix6.unmanaged.feed_enable(0.1)

        for wheel in self.wheels:
            wheel.update(tm_diff)
        for steer in self.steer:
            steer.update(tm_diff)

        for module in self.robot.drivetrain.modules:
            # Set the cancoder to be what the module wants it to be.
            desired_ang = module.state.angle.radians()
            raw = desired_ang / math.tau
            module.encoder.sim_state.set_raw_position(
                raw - module.mag_offset
            )

        for m in self.manip_motors:
            m.update(tm_diff)

        speeds = self.kinematics.toChassisSpeeds((
            self.swerve_modules[0].get(),
            self.swerve_modules[1].get(),
            self.swerve_modules[2].get(),
            self.swerve_modules[3].get(),
        ))

        self.current_yaw += math.degrees(speeds.omega * tm_diff)
        self.gyro.set_raw_yaw(self.current_yaw)

        self.physics_controller.drive(speeds, tm_diff)
        self.vision_sim.update(self.robot.drivetrain.get_pose())
        # self.update_mech_sim()

        # Ok now let's do photoeyes.
        """
        pe: PhotoEyeComponent = self.robot.photoeye
        intake: IntakeComponent = self.robot.intake
        if pe.coral_chute and self.pe_coral_chute_triggerd_at < 0:
            self.pe_coral_chute_triggerd_at = now

        if (intake.direction == IntakeDirection.CORAL_IN
            and
            self.intake_coral_in_at < 0 
        ):
            self.intake_coral_in_at = now

        elif (intake.direction == IntakeDirection.CORAL_SCORE
            and
            self.intake_coral_out_at < 0
        ):
            self.intake_coral_out_at = now
        # If intake isn't running at all set the timers to -1
        if intake.direction == IntakeDirection.NONE:
            self.intake_coral_in_at = -1.0
            self.intake_coral_out_at = -1.0

        # If coral in has been running for X seconds set the coral_held eye
        if (
            intake.direction == IntakeDirection.CORAL_IN
            and self.intake_coral_in_at + 1.6 < now
        ):
            pe.coral_held = True
            self.intake_coral_in_at = -1.0

        # Now if intake has been running for 0.6 seconds clear the coral_held eye
        if (
            intake.direction == IntakeDirection.CORAL_SCORE
            and self.intake_coral_out_at + 2.0 < now
        ):
            pe.coral_held = False
            self.intake_coral_out_at = -1.0

        diff = now - self.pe_coral_chute_triggerd_at
        if pe.coral_chute and self.pe_coral_chute_triggerd_at + 0.5 < now:
            # Untrigger the chute after a half a second.
            self.pe_coral_chute_triggerd_at = -1.0
            pe.coral_chute = False
        """
        pass

