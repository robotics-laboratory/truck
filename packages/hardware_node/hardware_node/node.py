import odrive
import pymodel
import rclpy
import rclpy.logging
from hardware_node.teensy import TeensyBridge
from rclpy.node import Node
from truck_interfaces.msg import Control, ControlMode


class HardwareNode(Node):
    # FIXME: Move some of these to external config (node launch config?)
    ODRIVE_MAIN_AXIS = "axis1"
    ODRIVE_WATCHDOG_TIMEOUT = 0.5
    DEFAULT_MODEL_CONFIG = "/truck/packages/model/config/model.yaml"
    DEFAULT_LINEAR_ACCEL = 1.0
    TEENSY_SERIAL_PORT = "/dev/ttyTHS0"
    TEENSY_SERIAL_SPEED = 500000
    STEERING_CSV_PATH = "/truck/packages/hardware_node/resource/steering.csv"

    def __init__(self):
        super().__init__("hardware_node")
        self._log = self.get_logger()
        self._prev_mode = ControlMode.OFF
        self._teensy = TeensyBridge(
            serial_port=self.TEENSY_SERIAL_PORT,
            serial_speed=self.TEENSY_SERIAL_SPEED,
            steering_csv_path=self.STEERING_CSV_PATH,
        )
        self._model = pymodel.Model(self.DEFAULT_MODEL_CONFIG)
        self._odrive = odrive.find_any()
        self._axis = self._get_main_axis()
        self._init_ros_communication()
        self._log.info("Hardware node initialized")

    def _init_ros_communication(self):
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
        self._log.info(f"Set motor speed: {rpm} rpm")
        self._axis.controller.input_vel = rpm
        twist = pymodel.Twist(msg.curvature, msg.velocity)
        twist = self._model.base_to_rear_twist(twist)
        steering = self._model.rear_twist_to_steering(twist)
        self._log.info(
            f"Set wheel angles: "
            f"{steering.left.degrees:.1f} | {steering.right.degrees:.1f}"
        )
        self._teensy.push(steering.left.radians, steering.right.radians)

    def _get_main_axis(self):
        axis = getattr(self._odrive, self.ODRIVE_MAIN_AXIS)
        axis.config.enable_watchdog = True
        axis.config.watchdog_timeout = self.ODRIVE_WATCHDOG_TIMEOUT
        axis.watchdog_feed()
        return axis

    def _get_default_accel(self):
        return self._model.linear_velocity_to_motor_rps(self.DEFAULT_LINEAR_ACCEL)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
