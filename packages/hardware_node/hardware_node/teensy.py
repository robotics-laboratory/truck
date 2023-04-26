import msgpacketizer as pack
import numpy as np
import serial


class TeensyBridge:
    MSGPACK_SEND_INDEX = 1
    MSGPACK_RECV_INDEX = 2

    def __init__(
        self,
        logger,
        serial_port: str,
        serial_speed: int,
        steering_csv_path: str,
        servo_home_angles: dict,
    ):
        self._log = logger
        self._serial = serial.Serial(port=serial_port, baudrate=serial_speed)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        # Steering mapping for RIGHT wheel generated in CAD
        # Column 0 = wheel angle (radians), column 1 = servo angle (radians)
        # Wheel angle (+) = left turn (right wheel moves clockwise, looking from top)
        # Servo angle (-) = left turn (servo arm moves forward, looking from top)
        self._map = np.genfromtxt(steering_csv_path, delimiter=",", skip_header=True)
        self._servo_home_angles = servo_home_angles

    def push(self, left_wheel_angle: float, right_wheel_angle: float):
        self._log.debug(
            f"Input angles: "
            f"{left_wheel_angle / np.pi * 180:.1f} | "
            f"{right_wheel_angle / np.pi * 180:.1f}",
        )
        left_servo_angle = np.interp(
            -left_wheel_angle, self._map[:, 0], self._map[:, 1]
        )
        right_servo_angle = np.interp(
            right_wheel_angle, self._map[:, 0], self._map[:, 1]
        )
        self._log.debug(
            f"Servo angles: "
            f"{left_servo_angle / np.pi * 180:.1f} | "
            f"{right_servo_angle / np.pi * 180:.1f}"
        )
        left_servo_angle = self._servo_home_angles["left"] + left_servo_angle
        right_servo_angle = self._servo_home_angles["right"] - right_servo_angle
        data = (np.rad2deg(left_servo_angle), np.rad2deg(right_servo_angle))
        self._log.debug(f"Servo angles + home: {data[0]:.1f} | {data[1]:.1f}")
        pack.send(self._serial, self.MSGPACK_SEND_INDEX, data)
