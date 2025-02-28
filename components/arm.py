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


class ArmComponent:
    motor = TalonFX(TalonId.MANIP_ARM.id, TalonId.MANIP_ARM.bus)
    encoder = CANcoder(CancoderId.MANIP_ARM.id, CancoderId.MANIP_ARM.bus)
    mag_offset = 0.26513671875
    default_pos = -80.0
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
        config.slot0.k_p = 6.0
        config.slot0.k_i = 0.0
        config.slot0.k_d = 0.0
        config.motion_magic.motion_magic_cruise_velocity = 1
        config.motion_magic.motion_magic_acceleration = 50
        config.motion_magic.motion_magic_jerk = 250
        config.feedback.feedback_remote_sensor_id = self.encoder.device_id
        config.feedback.feedback_sensor_source = signals.FeedbackSensorSourceValue.FUSED_CANCODER
        config.feedback.sensor_to_mechanism_ratio = 1
        config.feedback.rotor_to_sensor_ratio = 105
        config.closed_loop_general.continuous_wrap = False
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
        target_loc = ManipLocation(0, self.target_pos, 0)
        current_loc = ManipLocation(0, current_pos, 0)
        return current_loc == target_loc

    def execute(self):
        if self.target_pos < -90:
            # Driving the arm below -90 would be bad. Very bad. So don't let
            # anybody do that!
            self.target_pos = -90
        if self.target_pos > 90:
            # Driving the arm above 90 would be bad. Very bad. So don't let
            # anybody do that!
            self.target_pos = 90
        from math import cos, radians
        curr_pos = self.get_position()
        if self.target_pos < -180 or self.target_pos > 180:
            self.target_pos = norm_deg(self.target_pos)
        can_coder_target = self.target_pos / 360
        """
        max_kg = 0.1  # Call it 0.2 volts just to hold position at 0 degrees
        k_g = max_kg * cos(radians(curr_pos))
        # Not sure if we should flip the sign on k_g if we're heading downward.
        if self.target_pos > curr_pos:  # We're heading down
            k_g = -k_g.with_feed_forward(k_g)
        """
        req = self.motor_request.with_position(can_coder_target)
        self.motor.set_control(req)
