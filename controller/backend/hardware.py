from backend.common import Const, Config, State, SpeedCommand, SteeringCommand
from backend.helpers import RateLimitThread
import msgpacketizer as pack
import numpy as np
import serial
import logging
import functools
import odrive
import time


class TeensyBridge:
    def __init__(self, config: Config):
        self.config = config
        self.log = logging.getLogger("steering")
        self.serial = serial.Serial(port=Const.TEENSY_SERIAL_PORT, baudrate=Const.TEENSY_SERIAL_SPEED)
        self.map = np.genfromtxt(Const.STEERING_MAP_PATH, delimiter=",", skip_header=True)
        self.map = self.map * 180 / np.pi

    def push(self, cmd: SteeringCommand):
        if cmd.curvature == 0:
            self.log.info("Straight steering (curvature=0)")
            left_wheel_angle = 0
            right_wheel_angle = 0
        else:
            min_turn_radius = 1 / self.config.max_curvature
            if abs(cmd.radius) < min_turn_radius:
                self.log.warning("Turn radius too small: %.3f", cmd.radius)
                cmd.radius = np.copysign(min_turn_radius, radius)

            length = self.config.wheel_base_length
            width = self.config.wheel_base_width
            inner = np.arctan2(length, abs(cmd.radius) - width / 2)
            outer = np.arctan2(length, abs(cmd.radius) + width / 2)
            if cmd.radius > 0:
                self.log.info("Left turn with R=%.3f", abs(cmd.radius))
                left_wheel_angle = -inner
                right_wheel_angle = outer
            else:
                self.log.info("Right turn with R=%.3f", abs(cmd.radius))
                left_wheel_angle = outer
                right_wheel_angle = -inner
        self.log.info(
            "Wheel angles: left=%.3f, right=%.3f",
            np.rad2deg(left_wheel_angle),
            np.rad2deg(right_wheel_angle),
        )

        left_servo_angle = np.interp(-left_wheel_angle, self.map[:, 0], self.map[:, 1])
        right_servo_angle = np.interp(right_wheel_angle, self.map[:, 0], self.map[:, 1])
        self.log.info(
            "Servo angles: left=%.3f, right=%.3f",
            np.rad2deg(left_servo_angle),
            np.rad2deg(right_servo_angle),
        )

        left_servo_angle = self.config.left_servo_home - left_servo_angle
        right_servo_angle = self.config.right_servo_home - right_servo_angle

        data = (np.rad2deg(left_servo_angle), np.rad2deg(right_servo_angle))
        pack.send(self.serial, Const.MSGPACK_SEND_INDEX, data)
        self.last_left_servo_angle = float(left_servo_angle)
        self.last_right_servo_angle = float(right_servo_angle)


def catch_errors(*args, ignore_existing=False, reraise=True):
    # TODO: Prettify?
    def make_deco(func):
        @functools.wraps(func)
        def deco(self: "Hardware", *args, **kwargs):
            try:
                if not ignore_existing:
                    if self.state.error_state:
                        raise RuntimeError("Hardware is in error state")
                    self._check_errors()
                return func(self, *args, **kwargs)
            except Exception as err:
                self.panic(err)
                if reraise:
                    raise err
                
        return deco
    
    return make_deco(args[0]) if len(args) == 1 else make_deco


class Hardware:
    def __init__(self, config: Config, state: State):
        self.config = config
        self.state = state
        self._log = logging.getLogger("hardware")
        self._odrive = odrive.find_any()
        self._axis = self._get_main_axis()
        self._teensy = TeensyBridge(config)
        self._speed_rlt = RateLimitThread(
            self._set_speed, freq=100
        )
        self._steering_rlt = RateLimitThread(
            self._set_steering, freq=Const.RATE_LIMIT_FREQUENCY
        )
        self.update_config()

    def panic(self, error: Exception):
        message = str(error)
        if self.state.error_state:
            self._log.warning("Error state already set, skipping")
            return
        self._log.error("[PANIC] %s", message, exc_info=error)
        self.state.error_state = True
        self.state.error_description = message
        self.disable()

    @catch_errors(ignore_existing=True)
    def hard_reset(self):
        self._log.info("Starting hard reset sequence...")
        self._odrive.clear_errors()
        self.disable()
        self._reconnect_ordive()
        self._encoder_calibration()
        self.state.error_state = False
        self.state.error_description = ""
        self._log.info("Hard reset completed")

    @catch_errors(ignore_existing=True)
    def soft_reset(self):
        self._log.info("Starting soft reset sequence...")
        self._odrive.clear_errors()
        if not self._axis.encoder.index_found:
            self._encoder_calibration()
        self.state.error_state = False
        self.state.error_description = ""
        self._log.info("Soft reset completed")
    
    @catch_errors
    def enable(self):
        self._log.info("Enable motor")
        self._axis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        self._axis.controller.input_vel = 0
        self.state.motor_enabled = True
    
    @catch_errors(ignore_existing=True)
    def disable(self):
        self._log.info("Disable motor")
        self._axis.requested_state = odrive.enums.AXIS_STATE_IDLE
        self._axis.controller.input_vel = 0
        self.state.motor_enabled = False
        self._check_errors()

    @catch_errors
    def set_speed(self, ratio: float = None, linear: float = None, rpm: float = None):
        cmd = SpeedCommand(self.config, ratio, linear, rpm)
        self._log.info("Pending speed cmd: %s", str(cmd))
        # self._speed_rlt.push(cmd)
        self._axis.controller.input_vel = cmd.rpm

    @catch_errors
    def set_steering(self, ratio: float = None):
        cmd = SteeringCommand(self.config, ratio)
        self._log.info("Pending steering cmd: %s", str(cmd))
        self._steering_rlt.push(cmd)

    @catch_errors
    def update_config(self):
        self.disable()
        self._log.info("Config update")
        rpm_accel = self.config.linear_acceleration * self.config.linear_speed_to_rpm
        self._axis.controller.config.vel_ramp_rate = rpm_accel
        self._log.info(f"Acceleration: {self.config.linear_acceleration:.1f} m/s^2 | {rpm_accel:.1f} rpm/s^2")
        max_rpm = self.config.max_linear_speed * self.config.linear_speed_to_rpm
        self._axis.controller.config.vel_limit = max_rpm
        self._log.info(f"Max speed: {SpeedCommand(self.config, rpm=max_rpm)!s}")

    @catch_errors(reraise=False)
    def _set_speed(self, cmd: SpeedCommand):
        self._log.info("Set speed: %s", str(cmd))
        # self._axis.controller.input_vel = cmd.rpm

    @catch_errors(reraise=False)
    def _set_steering(self, cmd: SteeringCommand):
        self._log.info("Set steering: %s", str(cmd))
        self._teensy.push(cmd)

    def _reconnect_ordive(self):
        try:
            self._odrive.reboot()
        except Exception as err:
            self._log.warning(
                "ODrive reboot exception (expected): %s", type(err).__name__
            )
        time.sleep(0.5)  # Wait for odrive to reconnect...
        self._odrive = odrive.find_any()
        self._axis = self._get_main_axis()

    def _encoder_calibration(self):
        self._axis.requested_state = odrive.enums.AXIS_STATE_ENCODER_INDEX_SEARCH
        while not self._axis.encoder.index_found:
            self._check_errors()
            time.sleep(0.1)
        
    def _get_main_axis(self):
        return getattr(self._odrive, Const.ODRIVE_MAIN_AXIS)

    def _format_err_msg(self, raw_lines):
        clean_lines = []
        axis_block_found = False
        for line in raw_lines:
            if line.startswith("system"):
                clean_lines.append(line)
            elif line.startswith(Const.ODRIVE_MAIN_AXIS):
                axis_block_found = True
            elif axis_block_found and line.startswith("  "):
                clean_lines.append(line.strip())
        return "\n".join(clean_lines).strip()

    def _check_errors(self):
        if self._odrive.error == 0 and self._axis.error == 0:
            return  # No errors (yet)
        # Call to odrive.rich_text.RichText.__repr__
        buff = repr(odrive.utils.format_errors(self._odrive))
        raw_lines = buff.split("\n")
        error_msg = self._format_err_msg(raw_lines)
        error_msg = "ODrive errors detected:\n" + error_msg
        raise RuntimeError(error_msg)
