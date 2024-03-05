# pip install msgpacketizer pyserial
import time
from serial import Serial
import msgpacketizer


if __name__ == "__main__":
    SERIAL_PORT = "/dev/serial/by-id/..."
    SERIAL_SPEED = 500000
    MSGPACK_RECV_INDEX = 2
    MSGPACK_SEND_INDEX = 1
    MIN_ANGLE, MAX_ANGLE = 90 - 50, 90 + 50
    DELAY = 0.5 / (MAX_ANGLE - MIN_ANGLE)

    serial = Serial(port=SERIAL_PORT, baudrate=SERIAL_SPEED, timeout=0.5)

    def send_angles(left, right):
        msgpacketizer.send(serial, MSGPACK_SEND_INDEX, (left, right))
        time.sleep(DELAY)
        ret = serial.readline().strip()
        assert ret == b"ok"

    print("Testing left servo...")
    for i in range(90, MIN_ANGLE, -1):
        send_angles(i, 90)
    for i in range(MIN_ANGLE, MAX_ANGLE):
        send_angles(i, 90)
    for i in range(MAX_ANGLE, 90, -1):
        send_angles(i, 90)

    print("Testing right servo...")
    for i in range(90, MIN_ANGLE, -1):
        send_angles(90, i)
    for i in range(MIN_ANGLE, MAX_ANGLE):
        send_angles(90, i)
    for i in range(MAX_ANGLE, 90, -1):
        send_angles(90, i)
