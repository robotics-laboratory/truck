import odrive
import pymodel
import rclpy
from hardware_node.teensy import TeensyBridge
from rclpy.node import Node
from truck_interfaces.msg import Control, ControlMode, HardwareStatus, HardwareTelemetry


class HardwareNode(Node):
    def __init__(self):
        super().__init__("hardware_node")
        self._log = self.get_logger()
        self._init_ros_params()
        self._init_ros_topics()
        self._init_ros_timers()
        self._model = pymodel.Model(self._model_config)
        self._init_teensy()
        self._init_odrive()
        self._prev_mode = ControlMode.OFF
        self._log.info("Hardware node initialized")

    def _init_ros_params(self):
        self.declare_parameter("model_config", "")
        self.declare_parameter("steering_config", "")
        self.declare_parameter("odrive_axis", "axis1")
        self.declare_parameter("odrive_timeout", 200)
        self.declare_parameter("teensy_serial_port", "/dev/ttyTHS0")
        self.declare_parameter("teensy_serial_speed", 500000)
        self.declare_parameter("status_report_rate", 1.0)
        self.declare_parameter("telemetry_report_rate", 20.0)
        self._model_config = self._get_param("model_config", str)
        self._steering_config = self._get_param("steering_config", str)
        self._odrive_axis = self._get_param("odrive_axis", str)
        self._odrive_timeout = self._get_param("odrive_timeout", int, "ms")
        self._teensy_serial_port = self._get_param("teensy_serial_port", str)
        self._teensy_serial_speed = self._get_param("teensy_serial_speed", int)
        self._status_rate = self._get_param("status_report_rate", float, "Hz")
        self._telemetry_rate = self._get_param("telemetry_report_rate", float, "Hz")
        if not self._model_config:
            raise ValueError("Model config path not set")
        if not self._steering_config:
            raise ValueError("Steering CSV path not set")

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
            1 / self._status_rate,
            self._push_status,
        )
        self._telemetry_timer = self.create_timer(
            1 / self._telemetry_rate,
            self._push_telemetry,
        )

    def _init_teensy(self):
        self._teensy = TeensyBridge(
            logger=self._log,
            serial_port=self._teensy_serial_port,
            serial_speed=self._teensy_serial_speed,
            steering_csv_path=self._steering_config,
            servo_home_angles={
                "left": self._model.servo_home_angles.left.radians,
                "right": self._model.servo_home_angles.right.radians,
            },
        )

    def _init_odrive(self):
        self._odrive = odrive.find_any()
        self._axis = getattr(self._odrive, self._odrive_axis)
        self._axis.config.enable_watchdog = True
        self._axis.config.watchdog_timeout = self._odrive_timeout / 1000
        accel_mps = self._model.base_acceleration_limits.max
        accel_rps = self._model.linear_velocity_to_motor_rps(accel_mps)
        self._log.info(
            f"Max acceleration: {accel_mps:.1f} m/s^2 | {accel_rps:.1f} turns/s^2"
        )
        self._axis.controller.config.vel_ramp_rate = accel_rps
        self._disable_motor()

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
        self._odrive.clear_errors()
        self._axis.controller.input_vel = 0
        self._axis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL

    def _disable_motor(self):
        self._axis.requested_state = odrive.enums.AXIS_STATE_IDLE

    def _command_callback(self, msg: Control):
        if self._prev_mode == ControlMode.OFF:
            self._disable_motor()
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
            errors = self._parse_odrive_errors()
        status = HardwareStatus(armed=armed, errors=errors)
        status.header.stamp = self.get_clock().now().to_msg()
        self._status_pub.publish(status)

    def _parse_odrive_errors(self):
        root = {}
        root_stack = []
        curr_root = root
        last_child = None
        curr_level = 0
        lines = repr(odrive.utils.format_errors(self._odrive)).split("\n")
        for line in lines:
            level = len(line) - len(line.lstrip())
            name = line.split(":")[0].strip().split(".")[-1]
            if level > curr_level:
                curr_level = level
                root_stack.append(curr_root)
                curr_root = last_child
            elif level < curr_level:
                curr_level = level
                curr_root = root_stack.pop(-1)
            last_child = curr_root[name] = {}
        errors = [f"system.{x}" for x in root["system"]]
        axis_group = root[self._odrive_axis]
        errors += [f"axis.{x}" for x in axis_group["axis"]]
        errors += [f"motor.{x}" for x in axis_group["motor"]]
        errors += [f"drv.{x}" for x in axis_group["DRV fault"]]
        errors += [f"estimator.{x}" for x in axis_group["sensorless_estimator"]]
        errors += [f"encoder.{x}" for x in axis_group["encoder"]]
        errors += [f"controller.{x}" for x in axis_group["controller"]]
        return errors

    def _push_telemetry(self):
        telemetry = HardwareTelemetry(
            current_velocity=self._axis.encoder.vel_estimate,
            target_velocity=self._axis.controller.input_vel,
            battery_voltage=self._odrive.vbus_voltage,
            battery_current=self._odrive.ibus,
        )
        telemetry.header.stamp = self.get_clock().now().to_msg()
        self._telemetry_pub.publish(telemetry)

    def _get_param(self, name, type, unit=""):
        value = self.get_parameter(name).get_parameter_value()
        if type == str:
            value = value.string_value
        elif type == float:
            value = value.double_value
        elif type == int:
            value = value.integer_value
        else:
            raise RuntimeError(f"Unsupported type: {type}")
        readable_name = name.replace('_', ' ').capitalize()
        self._log.info(f"{readable_name}: {value!r} {unit}")
        return value


def main():
    rclpy.init()
    node = HardwareNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
