import wpilib
from magicbot import feedback, tunable
from phoenix6 import configs, signals
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import (
    DutyCycleOut,
    MotionMagicDutyCycle
)
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs
from utilities.game import ManipLocation
from utilities import norm_deg, is_sim
from ids import TalonId, CancoderId
from components.arm import ArmComponent
import time

class WristComponent:
    motor = TalonFX(TalonId.MANIP_WRIST.id, TalonId.MANIP_WRIST.bus)
    encoder = CANcoder(CancoderId.MANIP_WRIST.id, CancoderId.MANIP_WRIST.bus)
    mag_offset = -0.05517578125
    default_pos = 1.0
    target_pos = tunable(default_pos)
    motor_request = MotionMagicDutyCycle(0, override_brake_dur_neutral=True)
    arm: ArmComponent
    
    # SysID tuning parameters
    sysid_enabled = tunable(False)
    sysid_voltage = tunable(0.0)
    sysid_test_type = tunable("quasistatic") # or "dynamic"
    sysid_rotation = tunable("forward") # or "backward" 
    sysid_data_points = []
    
    def __init__(self):
        # Existing init code...
        self.last_time = time.time()
        self.sysid_voltage_command = DutyCycleOut(0)
        self.log = wpilib.DataLogManager.getLog()

    def do_sysid(self):
        """Run a SysID characterization routine"""
        if not self.sysid_enabled:
            return

        current_time = time.time()
        dt = current_time - self.last_time
        
        # Get current state
        position = self.encoder.get_position().value * 360  # degrees
        velocity = self.encoder.get_velocity().value * 360  # degrees per second
        voltage = self.motor.get_motor_voltage().value
        
        # Determine voltage based on test type
        if self.sysid_test_type == "quasistatic":
            # Gradually increase voltage
            ramp_rate = 0.1  # V/s
            self.sysid_voltage += ramp_rate * dt
        else:  # dynamic
            # Step voltage
            self.sysid_voltage = 7.0  # Fixed test voltage
            
        # Apply direction
        if self.sysid_rotation == "backward":
            self.sysid_voltage = -self.sysid_voltage
            
        # Apply voltage
        self.sysid_voltage_command.output = self.sysid_voltage / 12.0  # Convert to duty cycle
        self.motor.set_control(self.sysid_voltage_command)
        
        # Log data
        self.log.append(f"sysid,{current_time},{position},{velocity},{voltage}")
        
        self.last_time = current_time
        
    def execute(self):
        """Regular execution loop"""
        if self.sysid_enabled:
            self.do_sysid()
            return
            
        # Normal execution code...
        if self.target_pos < -180 or self.target_pos > 180:
            self.target_pos = norm_deg(self.target_pos)
        can_coder_target = self.target_pos / 360
        req = self.motor_request.with_position(can_coder_target)
        self.motor.set_control(req)