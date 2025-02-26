from __future__ import annotations

import math
import typing
import numpy as np

import phoenix6
import phoenix6.unmanaged
import wpilib
import robotpy_apriltag
from photonlibpy.simulation.visionSystemSim import VisionSystemSim
from photonlibpy.simulation.photonCameraSim import PhotonCameraSim
from photonlibpy.simulation.simCameraProperties import SimCameraProperties
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import DCMotorSim
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import kilogram_square_meters

from components.drivetrain import SwerveModule
from components import IntakeComponent, PhotoEyeComponent
from components.intake import IntakeDirection

from generated.tuner_constants import TunerConstants
from utilities import Waypoints

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
            robotpy_apriltag.AprilTagField.k2025ReefscapeWelded
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

        self.manip_motors: list[Falcon500MotorSim] = []

        self.intake_motor = Falcon500MotorSim(
            robot.intake.motor, gearing=1, moi=0.001
        )
        self.manip_motors.append(self.intake_motor)

        self.wrist_motor = Falcon500MotorSim(
            robot.wrist.motor, gearing=105, moi=0.00001
        )
        self.robot.wrist.encoder.sim_state.set_raw_position(
            self.robot.wrist.target_pos / 360 - self.robot.wrist.mag_offset
        )
        self.manip_motors.append(self.wrist_motor)
        
        self.arm_motor = Falcon500MotorSim(
            robot.arm.motor, gearing=25, moi=0.00001
        )
        self.robot.arm.encoder.sim_state.set_raw_position(
            self.robot.arm.target_pos / 360 - self.robot.arm.mag_offset
        )
        self.manip_motors.append(self.arm_motor)
    
        self.elevator_motor_left = Falcon500MotorSim(
            robot.elevator.motor_left, gearing=1, moi=0.00001
        )
        self.manip_motors.append(self.elevator_motor_left)

        self.elevator_motor_right = Falcon500MotorSim(
            robot.elevator.motor_right, gearing=1, moi=0.00001
        )
        self.manip_motors.append(self.elevator_motor_right)
        
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
        self.camera_fr = PhotonCameraSim(robot.vision.camera_fr, properties)
        self.camera_fr.setMaxSightRange(5.0)
        self.vision_sim.addCamera(
            self.camera_fr,
            self.robot.vision.camera_fr_offset,
        )

    def update_pe_intake_sim(self, now: float, tm_diff: float) -> None:
        # Ok now let's do photoeyes.
        pe: PhotoEyeComponent = self.robot.photoeye
        intake: IntakeComponent = self.robot.intake

        robot_pose = self.robot.drivetrain.get_pose()
        _, ps_dist = Waypoints.closest_ps_tag_id(robot_pose)

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
            and ps_dist < 0.99  # We have to be within a X meters of a pick up station
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
            sigma = (math.radians(0.5) / math.tau) / 2
            raw = desired_ang / math.tau + np.random.normal(loc=0, scale=sigma)
            module.encoder.sim_state.set_raw_position(
                raw - module.mag_offset
            )
        for m in self.manip_motors:
            m.update(tm_diff)

        w = self.robot.wrist
        wpos = w.encoder.get_position().value
        wvel = self.wrist_motor.motor_sim.getAngularVelocity() / 105
        w.encoder.sim_state.set_raw_position(wpos - w.mag_offset + wvel)
        
        a = self.robot.arm
        wpos = a.encoder.get_position().value
        wvel = self.arm_motor.motor_sim.getAngularVelocity() / 255
        a.encoder.sim_state.set_raw_position(wpos - a.mag_offset + wvel)

        speeds = self.kinematics.toChassisSpeeds((
            self.swerve_modules[0].get(),
            self.swerve_modules[1].get(),
            self.swerve_modules[2].get(),
            self.swerve_modules[3].get(),
        ))

        self.current_yaw += math.degrees(speeds.omega * tm_diff)
        sigma = (math.radians(0.5) / math.tau) / 2
        yaw_jitter = np.random.normal(loc=0, scale=sigma)
        self.gyro.set_raw_yaw(self.current_yaw + yaw_jitter)

        self.physics_controller.drive(speeds, tm_diff)
        self.vision_sim.update(self.robot.drivetrain.get_pose())
        # self.update_pe_intake_sim()

