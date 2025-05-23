import struct
import time

import numpy as np
import serial
from cobs import cobs


def noexcept(func):
    def deco(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as err:
            print(f"err in func {func.__name__}:", err)

    return deco


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
        timeout: float = 0.05,
    ):
        self._log = logger
        self._serial = serial.Serial(
            port=serial_port,
            baudrate=serial_speed,
            timeout=timeout,
        )
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        # Steering mapping for RIGHT wheel generated in CAD
        # Column 0 = wheel angle (radians), column 1 = servo angle (radians)
        # Wheel angle (+) = left turn (right wheel moves clockwise, looking from top)
        # Servo angle (-) = left turn (servo arm moves forward, looking from top)
        self._map = np.genfromtxt(steering_csv_path, delimiter=",", skip_header=True)
        self._servo_home_angles = servo_home_angles
        self._odom_servo_ts = 0
        self._odom_servo_values = {"left": 0, "right": 0}
        self._odom_wheels_ts = 0
        self._odom_wheels_values = {"left": 0, "right": 0}
        self._target_servo_values = {"left": 0, "right": 0}
        self._magnitometer_values = {"x_axis": 0, "y_axis": 0, "z_axis": 0}
        self._left_wheel_values = []
        self._right_wheel_values = []

    @noexcept
    def _parse_one(self, chunk):
        chunk = chunk.strip(b"\x00")
        data = cobs.decode(chunk)
        msgid, data = data[0], data[1:]
        # self._log.info(f"{msgid}, {data}")
        if msgid == 1:
            l, r = struct.unpack("<ff", data)
            self._odom_wheels_ts = time.perf_counter()
            self._odom_wheels_values = {"left": l, "right": r}
            # self._log.debug(f"wheel: {self._odom_wheels_values}")
        elif msgid == 2:
            l, r = struct.unpack("<ff", data)
            self._odom_servo_ts = time.perf_counter()
            self._odom_servo_values = {"left": l, "right": r}
            # self._log.debug(f"servo: {self._odom_servo_values}")
        elif msgid == 4:
            magn_x, magn_y, magn_z = struct.unpack("<HHH", data)
            self._magnitometer_ts = time.perf_counter()
            self._magnitometer_values["x_axis"] = magn_x
            self._magnitometer_values["y_axis"] = magn_y
            self._magnitometer_values["z_axis"] = magn_z
        elif msgid == 5:
            wheel_tick = struct.unpack("<i", data)
            self._left_wheel_values.append(wheel_tick[0])
        elif msgid == 6:
            wheel_tick = struct.unpack("<i", data)
            self._right_wheel_values.append(wheel_tick[0])

    def pull(self):
        while self._serial.in_waiting:
            chunk = self._serial.read_until(b"\x00")
            if not chunk:
                self._log.info(f"no data available: {self._serial.in_waiting}")
                return
            self._parse_one(chunk)

    def push(self, left_wheel_angle: float, right_wheel_angle: float):
        self._log.debug(
            f"Input angles: "
            f"{left_wheel_angle / np.pi * 180:.1f} | "
            f"{right_wheel_angle / np.pi * 180:.1f}",
        )
        left_servo_angle = np.interp(left_wheel_angle, self._map[:, 0], self._map[:, 1])
        right_servo_angle = np.interp(
            -right_wheel_angle, self._map[:, 0], self._map[:, 1]
        )
        self._log.debug(
            f"Servo angles: "
            f"{left_servo_angle / np.pi * 180:.1f} | "
            f"{right_servo_angle / np.pi * 180:.1f}"
        )
        self._target_servo_values["left"] = np.rad2deg(left_servo_angle)
        self._target_servo_values["right"] = np.rad2deg(right_servo_angle)
        left_servo_angle = self._servo_home_angles["left"] + left_servo_angle
        right_servo_angle = self._servo_home_angles["right"] - right_servo_angle
        data = (np.rad2deg(left_servo_angle), np.rad2deg(right_servo_angle))
        self._log.debug(f"Servo angles + home: {data[0]:.1f} | {data[1]:.1f}")
        packet = struct.pack("BBB", 3, round(data[0]), round(data[1]))
        packet = cobs.encode(packet) + b"\x00"
        self._serial.write(packet)

    def blink(self, activate: bool):
        self._log.info(f"Teensy BLINK - {activate}")
        packet = struct.pack("BB", 5, 1 if (activate) else 0)
        packet = cobs.encode(packet) + b"\x00"
        self._serial.write(packet)


if __name__ == "__main__":
    import logging

    logging.basicConfig(level="INFO")
    logger = logging.getLogger("teensy")
    mcu = TeensyBridge(
        logger=logger,
        serial_port="/dev/ttyTHS0",
        serial_speed=921600,
        steering_csv_path="../resource/steering.csv",
        servo_home_angles={"left": np.deg2rad(98), "right": np.deg2rad(172)},
    )

    import time

    def read_loop():
        while True:
            mcu.pull()

    read_loop()
    # t = Thread(target=read_loop)
    # t.start()

    # flag = False
    # while True:
    #     l, r = 0, 0
    #     if flag: r = np.deg2rad(20)
    #     mcu.push(l, r)
    #     flag = not flag
    #     time.sleep(3)

    # input("enter to send 0, 0")
    # mcu.push(0, 0)

    # input("enter to send 20deg, 0")
    # mcu.push(0, 0.349066)

    # input("enter to send 0, 0")
    # mcu.push(0, 0)

    # t.join()
