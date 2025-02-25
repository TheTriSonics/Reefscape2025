import math
import wpilib
from phoenix6.hardware import TalonFX, CANcoder
from magicbot import feedback, tunable
from phoenix6.controls import (
    MotionMagicVoltage,
)
from phoenix6.configs import TalonFXConfiguration
from phoenix6 import configs, signals
from utilities.game import ManipLocation
from ids import TalonId, CancoderId

pn = wpilib.SmartDashboard.putNumber


class WristComponent:
    motor = TalonFX(TalonId.MANIP_WRIST.id, TalonId.MANIP_WRIST.bus)
    encoder = CANcoder(CancoderId.MANIP_WRIST.id, CancoderId.MANIP_WRIST.bus)
    default_pos = 135.0
    target_pos = tunable(default_pos)
    motor_request = MotionMagicVoltage(0, override_brake_dur_neutral=True)
    
    def __init__(self):
        enc_config = configs.CANcoderConfiguration()

        enc_config.magnet_sensor.absolute_sensor_discontinuity_point = 0.5
        enc_config.magnet_sensor.sensor_direction = signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        enc_config.magnet_sensor.magnet_offset = 0.4
        self.encoder.configurator.apply(enc_config) # type: ignore


        config = TalonFXConfiguration()
        config.slot0.k_s = 0.0
        config.slot0.k_v = 0.8
        config.slot0.k_a = 0.01
        config.slot0.k_p = 0.5
        config.slot0.k_i = 0
        config.slot0.k_d = 0.1
        config.motion_magic.motion_magic_cruise_velocity = 100
        config.motion_magic.motion_magic_acceleration = 1600
        config.motion_magic.motion_magic_jerk = 4000
        config.feedback.feedback_remote_sensor_id = self.encoder.device_id
        config.feedback.feedback_sensor_source = signals.FeedbackSensorSourceValue.FUSED_CANCODER
        # The sensor is on the output shaft, so the sensor to mechanism ratio is 1
        config.feedback.sensor_to_mechanism_ratio = 1
        # The rotor to sensor should be the ratio of the gears connecting the motor
        # to the wrist pivot.
        config.feedback.rotor_to_sensor_ratio = 1.0 
        self.motor.set_position(self.default_pos)
        self.encoder.set_position(self.default_pos)
        self.motor.configurator.apply(config)  # type: ignore
    
    @feedback
    def get_position(self) -> float:
        return self.motor.get_position().value

    @feedback
    def at_goal(self):
        current_pos = self.get_position()
        target_loc = ManipLocation(0, 0, self.target_pos)
        current_loc = ManipLocation(0, 0, current_pos)
        return current_loc == target_loc

    def execute(self):

        if not self.at_goal():
            req = self.motor_request.with_position(self.target_pos)
            self.motor.set_control(req)