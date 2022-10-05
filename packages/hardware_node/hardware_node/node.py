from pathlib import PosixPath as Path

import odrive
import pymodel
import rclpy
import rclpy.logging
from ament_index_python.packages import get_package_share_directory
from hardware_node.teensy import TeensyBridge
from rclpy.node import Node
from truck_interfaces.msg import Control, ControlMode, HardwareStatus, HardwareTelemetry


class HardwareNode(Node):
    DEFAULT_LINEAR_ACCEL = 1.0
    DEFAULT_MODEL_CONFIG = ["model", "config/model.yaml"]
    STEERING_CSV_PATH = ["hardware_node", "resource/steering.csv"]

    def __init__(self):
        super().__init__("hardware_node")
        self._log = self.get_logger()
        self._init_ros_params()
        self._init_ros_topics()
        self._init_ros_timers()
        self._teensy = TeensyBridge(
            logger=self._log,
            serial_port=self._teensy_serial_port,
            serial_speed=self._teensy_serial_speed,
            steering_csv_path=self._get_shared_path(*self.STEERING_CSV_PATH),
        )
        self._model = pymodel.Model(self._get_shared_path(*self.DEFAULT_MODEL_CONFIG))
        self._odrive = odrive.find_any()
        self._axis = getattr(self._odrive, self._odrive_axis)
        self._axis.config.enable_watchdog = True
        self._axis.config.watchdog_timeout = self._odrive_timeout
        self._prev_mode = ControlMode.OFF
        self._prev_status: HardwareStatus = None
        self._log.info("Hardware node initialized")

    def _init_ros_params(self):
        self.declare_parameter("odrive_axis", "axis1")
        self.declare_parameter("odrive_timeout", 0.5)
        self.declare_parameter("teensy_serial_port", "/dev/ttyTHS0")
        self.declare_parameter("teensy_serial_speed", 500000)
        self.declare_parameter("status_report_rate", 1.0)
        self.declare_parameter("telemetry_report_rate", 100.0)
        self._odrive_axis = self._get_param("odrive_axis", str)
        self._odrive_timeout = self._get_param("odrive_timeout", float)
        self._teensy_serial_port = self._get_param("teensy_serial_port", str)
        self._teensy_serial_speed = self._get_param("teensy_serial_speed", int)
        self._status_report_rate = self._get_param("status_report_rate", float)
        self._telemetry_report_rate = self._get_param("telemetry_report_rate", float)

    def _init_ros_topics(self):
        self._mode_sub = self.create_subscription(
            ControlMode,
            "/control/mode",
            self._mode_callback,
            qos_profile=1,
        )
        self._command_sub = self.create_subscription(
            Control,
            "/control/command",
            self._command_callback,
            qos_profile=1,
        )
        self._status_pub = self.create_publisher(
            HardwareStatus,
            "/hardware/status",
            qos_profile=1,
        )
        self._telemetry_pub = self.create_publisher(
            HardwareTelemetry,
            "/hardware/telemetry",
            qos_profile=1,
        )

    def _init_ros_timers(self):
        self._status_timer = self.create_timer(
            1 / self._status_report_rate,
            self._push_status,
        )
        self._telemetry_timer = self.create_timer(
            1 / self._telemetry_report_rate,
            self._push_telemetry,
        )

    def _mode_callback(self, msg: ControlMode):
        if msg.mode == self._prev_mode:
            return
        if self._prev_mode == ControlMode.OFF and msg.mode != ControlMode.OFF:
            self._log.info("Mode change: OFF -> ANY - Enabling motor")
            self._enable_motor()
        if self._prev_mode != ControlMode.OFF and msg.mode == ControlMode.OFF:
            self._log.info("Mode change: ANY -> OFF - Disabling motor")
            self._disable_motor()
        self._prev_mode = msg.mode
        self._status_timer.reset()
        self._push_status()

    def _enable_motor(self):
        self._axis.watchdog_feed()
        self._odrive.clear_errors()
        self._axis.controller.input_vel = 0
        self._axis.controller.config.vel_ramp_rate = self._get_default_accel()
        self._axis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL

    def _disable_motor(self):
        self._axis.requested_state = odrive.enums.AXIS_STATE_IDLE

    def _command_callback(self, msg: Control):
        if self._prev_mode == ControlMode.OFF:
            return
        self._axis.watchdog_feed()
        rpm = self._model.linear_velocity_to_motor_rps(msg.velocity)
        self._axis.controller.input_vel = rpm
        twist = pymodel.Twist(msg.curvature, msg.velocity)
        twist = self._model.base_to_rear_twist(twist)
        steering = self._model.rear_twist_to_steering(twist)
        self._log.debug(f"Center curvature: {msg.curvature:.2f}")
        self._log.debug(f"Rear curvature: {twist.curvature:.2f}")
        self._teensy.push(steering.left.radians, steering.right.radians)

    def _push_status(self):
        armed = self._axis.current_state != odrive.enums.AXIS_STATE_IDLE
        errors = []
        if self._odrive.error or self._axis.error:
            # TODO: Prettify
            message = odrive.utils.format_errors(self._odrive)
            errors.extend(repr(message).split("\n"))
        status = HardwareStatus(armed=armed, errors=errors)
        self._status_pub.publish(status)

    def _push_telemetry(self):
        telemetry = HardwareTelemetry(
            current_velocity=self._axis.encoder.vel_estimate,
            target_velocity=self._axis.controller.input_vel,
            battery_voltage=self._odrive.vbus_voltage,
            battery_current=self._odrive.ibus,
        )
        self._telemetry_pub.publish(telemetry)

    def _get_default_accel(self):
        return self._model.linear_velocity_to_motor_rps(self.DEFAULT_LINEAR_ACCEL)

    def _get_param(self, name, type):
        value = self.get_parameter(name).get_parameter_value()
        if type == str:
            return value.string_value
        elif type == float:
            return value.double_value
        elif type == int:
            return value.integer_value
        else:
            raise RuntimeError(f"Unsupported type: {type}")

    def _get_shared_path(self, package, path):
        share = get_package_share_directory(package)
        return Path(share).joinpath(path).as_posix()


def main():
    rclpy.init()
    node = HardwareNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
