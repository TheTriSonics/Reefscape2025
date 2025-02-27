import wpilib
from magicbot import feedback, tunable
from phoenix6 import configs, signals
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import (
    MotionMagicDutyCycle
)
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs
from utilities.game import ManipLocation
from utilities import norm_deg, is_sim
from ids import TalonId, CancoderId

pn = wpilib.SmartDashboard.putNumber


class WristComponent:
    motor = TalonFX(TalonId.MANIP_WRIST.id, TalonId.MANIP_WRIST.bus)
    encoder = CANcoder(CancoderId.MANIP_WRIST.id, CancoderId.MANIP_WRIST.bus)
    mag_offset = 0.39990234375
    default_pos = 1.0
    target_pos = tunable(default_pos)
    motor_request = MotionMagicDutyCycle(0, override_brake_dur_neutral=True)
    
    def __init__(self):
        enc_config = configs.CANcoderConfiguration()

        enc_config.magnet_sensor.absolute_sensor_discontinuity_point = 0.5
        enc_config.magnet_sensor.sensor_direction = signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        enc_config.magnet_sensor.magnet_offset = self.mag_offset
        self.encoder.configurator.apply(enc_config) # type: ignore

        config = TalonFXConfiguration()
        config.slot0.k_s = 0.0
        config.slot0.k_v = 0.8
        config.slot0.k_a = 0.01
        config.slot0.k_p = 10.0
        config.slot0.k_i = 0.0
        config.slot0.k_d = 0.0
        config.motion_magic.motion_magic_cruise_velocity = 1
        config.motion_magic.motion_magic_acceleration = 50
        config.motion_magic.motion_magic_jerk = 250
        config.feedback.feedback_remote_sensor_id = self.encoder.device_id
        config.feedback.feedback_sensor_source = signals.FeedbackSensorSourceValue.FUSED_CANCODER
        config.feedback.sensor_to_mechanism_ratio = 1
        config.feedback.rotor_to_sensor_ratio = 105
        config.closed_loop_general.continuous_wrap = True
        config_output = MotorOutputConfigs()
        config_output.neutral_mode = signals.NeutralModeValue.BRAKE
        self.motor.configurator.apply(config)  # type: ignore
        self.motor.configurator.apply(config_output)
    
    @feedback
    def get_position(self) -> float:
        ang = self.motor.get_position().value * 360
        return norm_deg(ang)

    @feedback
    def get_encoder_position(self) -> float:
        ang = self.encoder.get_position().value * 360
        return norm_deg(ang)

    @feedback
    def at_goal(self):
        current_pos = self.get_position()
        target_loc = ManipLocation(0, 0, self.target_pos)
        current_loc = ManipLocation(0, 0, current_pos)
        return current_loc == target_loc

    def execute(self):
        if self.target_pos < -180 or self.target_pos > 180:
            self.target_pos = norm_deg(self.target_pos)
        can_coder_target = self.target_pos / 360
        req = self.motor_request.with_position(can_coder_target)
        if is_sim():
            self.motor.set_control(req)