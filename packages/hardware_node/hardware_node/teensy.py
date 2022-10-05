from typing import Any, Tuple

import crc8
import msgpack
import numpy as np
import serial
from cobs import cobs


class TeensyBridge:
    MSGPACK_SEND_INDEX = 1
    MSGPACK_RECV_INDEX = 2
    LEFT_SERVO_HOME = float(89 / 180 * np.pi)
    RIGHT_SERVO_HOME = float(96 / 180 * np.pi)

    def __init__(
        self,
        logger,
        serial_port: str,
        serial_speed: int,
        steering_csv_path: str,
    ):
        self._log = logger
        self._serial = serial.Serial(port=serial_port, baudrate=serial_speed)
        self._map = np.genfromtxt(steering_csv_path, delimiter=",", skip_header=True)

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
        left_servo_angle = left_servo_angle + self.LEFT_SERVO_HOME
        right_servo_angle = self.RIGHT_SERVO_HOME - right_servo_angle
        data = (np.rad2deg(left_servo_angle), np.rad2deg(right_servo_angle))
        self._log.debug(f"Servo angles + home: {data[0]:.1f} | {data[1]:.1f}")
        self._send(self._serial, self.MSGPACK_SEND_INDEX, data)

    def _send(self, stream, pkgidx, data):
        packed = self._pack(pkgidx, data)
        stream.write(b"\0" + packed + b"\0")

    def _pack(self, pkgidx: int, datain: Any) -> bytes:
        pkgidx = bytes([pkgidx])
        check = crc8.crc8()
        packed = msgpack.packb(datain)
        check.update(packed)
        return cobs.encode(pkgidx + packed + check.digest())

    def _unpack(self, bytesin: bytes) -> Tuple[int, Any]:
        if bytesin[0] == 0:
            bytesin = bytesin[1:]
        if bytesin[-1] == 0:
            bytesin = bytesin[:-1]
        decoded = cobs.decode(bytesin)
        idx = decoded[0]
        pktcrc = decoded[-1]
        packed = decoded[1:-1]
        check = crc8.crc8()
        check.update(packed)
        expectedcrc = check.digest()[0]
        if expectedcrc != pktcrc:
            raise ValueError("Packet checksum mismatch")
        data = msgpack.unpackb(packed)
        return (idx, data)
