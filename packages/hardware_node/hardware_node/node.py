from functools import cached_property

import odrive
import pymodel
import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Header
from truck_msgs.msg import Control, ControlMode, HardwareStatus, HardwareTelemetry

from hardware_node.teensy import TeensyBridge


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
        self._target_curvature = 0.0

    def _init_ros_params(self):
        self.declare_parameter("model_config", "")
        self.declare_parameter("steering_config", "")
        self.declare_parameter("odrive_axis", "axis1")
        self.declare_parameter("odrive_timeout", 250)
        self.declare_parameter("teensy_serial_port", "/dev/ttyTHS0")
        self.declare_parameter("teensy_serial_speed", 921600)
        self.declare_parameter("status_report_rate", 1.0)
        self.declare_parameter("telemetry_report_rate", 100.0)
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
        self._odom_pub = self.create_publisher(
            Odometry,
            "/hardware/wheel/odometry",
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
        self._odrive = odrive.find_any(timeout=5)
        self._log.info("odrive found!")
        self._axis = getattr(self._odrive, self._odrive_axis)
        self._axis.config.enable_watchdog = True
        self._axis.config.watchdog_timeout = self._odrive_timeout / 1000
        accel_mps = self._model.max_acceleration
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
        self._target_curvature = msg.curvature

    def _push_status(self):
        armed = self._axis.current_state != odrive.enums.AXIS_STATE_IDLE
        errors = self._parse_odrive_errors()
        status = HardwareStatus(armed=armed, errors=errors)
        status.header.stamp = self.get_clock().now().to_msg()
        self._status_pub.publish(status)

    def _parse_odrive_errors(self):
        errors = []

        def add_errors(prefix, value, enum_type):
            try:
                value = int(value or 0)
            except (TypeError, ValueError):
                return
            if value == 0:
                return

            for item in enum_type:
                item_value = int(item.value)
                if item_value and value & item_value:
                    errors.append(f"{prefix}.{item.name}")
                    value &= ~item_value
            if value:
                errors.append(f"{prefix}.UNKNOWN_0x{value:08X}")

        add_errors("system", getattr(self._odrive, "error", 0), odrive.enums.ODriveError)
        add_errors("axis", getattr(self._axis, "error", 0), odrive.enums.AxisError)
        add_errors(
            "axis.active",
            getattr(self._axis, "active_errors", 0),
            odrive.enums.ODriveError,
        )
        add_errors(
            "axis.disarm",
            getattr(self._axis, "disarm_reason", 0),
            odrive.enums.ODriveError,
        )

        last_drv_fault = int(getattr(self._axis, "last_drv_fault", 0) or 0)
        if last_drv_fault:
            errors.append(f"axis.drv_fault.0x{last_drv_fault:08X}")

        return errors

    def _push_telemetry(self):
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="base")

        rps = self._axis.encoder.vel_estimate
        vel = self._model.motor_rps_to_linear_velocity(rps)
        curv = self._target_curvature
        twist = pymodel.Twist(curv, vel)
        twist = self._model.base_to_rear_twist(twist)
        steering = self._model.rear_twist_to_steering(twist)
        wheel_velocity = self._model.rear_twist_to_wheel_velocity(twist)

        telemetry = HardwareTelemetry(
            header=header,
            current_rps=rps,
            target_rps=self._axis.controller.input_vel,
            battery_voltage=self._odrive.vbus_voltage,
            battery_current=self._odrive.ibus,
            target_left_steering=steering.left.radians,
            current_left_steering=steering.left.radians,
            target_right_steering=steering.right.radians,
            current_right_steering=steering.right.radians,
            rear_left_wheel_velocity=wheel_velocity.rear_left,
            rear_right_wheel_velocity=wheel_velocity.rear_right,
            front_left_wheel_velocity=wheel_velocity.front_left,
            front_right_wheel_velocity=wheel_velocity.front_right,
        )

        self._telemetry_pub.publish(telemetry)

        odom = Odometry(header=header)
        odom.twist.twist.linear = Vector3(x=float(vel), y=0.0, z=0.0)
        odom.twist.covariance = self._odom_covariance
        self._odom_pub.publish(odom)

    @cached_property
    def _odom_covariance(self):
        matrix = [0.0] * 36
        matrix[0] = 0.0001
        return matrix

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
        readable_name = name.replace("_", " ").capitalize()
        self._log.info(f"{readable_name}: {value!r} {unit}")
        return value


def main():
    rclpy.init()
    node = HardwareNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
