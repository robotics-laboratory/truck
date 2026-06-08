#!/truck/firmware/venv/bin/python3
"""
ROS 2 node that communicates with the ESP32 robot over serial.
Framing is COBS + CRC‑8/SMBUS (data only, index excluded) + 0x00 delimiter.
"""
#import sys
#print(sys.executable)
#exit(0)

import struct
import threading
import time
from dataclasses import dataclass

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, Float32, Float32MultiArray
from std_srvs.srv import Empty

from truck_msgs.msg import Control, ControlMode, HardwareStatus, HardwareTelemetry

from cobs import cobs
from anycrc import Model  # pip install anycrc

# ====== CRC‑8/SMBUS (polynomial 0x07) ======
crc8_model = Model('CRC8-SMBUS')   # poly=0x07, init=0x00, ref_in=False, ref_out=False, xor_out=0x00

# ====== Command / Status dataclasses ======

max_vel = 0.3
max_cri = 0.85

@dataclass
class Command:
    type: int          # uint8, 1/2/3
    steering: float
    speed: float

    def pack(self) -> bytes:
        return struct.pack('<Bff', self.type, self.steering, self.speed)

    @classmethod
    def unpack(cls, data: bytes):
        return cls(*struct.unpack('<Bff', data))


@dataclass
class Status:
    enc1_speed_ticks: float
    enc1_angle_ticks: float
    enc2_speed_ticks: float
    enc2_angle_ticks: float
    active: bool

    @classmethod
    def unpack(cls, data: bytes):
        a, b, c, d, e = struct.unpack('<ffffB', data)
        return cls(a, b, c, d, bool(e))


# ====== Low‑level framing functions ======

def send_packet(ser: serial.Serial, index: int, data: bytes):
    """
    Build a packet with index, compute CRC8 over data only,
    COBS‑encode index+data+CRC, append 0x00 and write to serial.
    """
    payload = bytes([index]) + data
    crc_byte = crc8_model.calc(data).to_bytes(1, 'little')  # CRC over data only!
    frame = cobs.encode(payload + crc_byte) + b'\x00'
    ser.write(frame)
    ser.flush()


def read_packets(ser: serial.Serial, callback):
    """Continuously read serial, reassemble frames, decode and call callback(index, data)."""
    buf = bytearray()
    while True:
        if ser.in_waiting:
            new = ser.read(ser.in_waiting)
            buf.extend(new)
            # Look for 0x00 delimiter
            while True:
                try:
                    delim_idx = buf.index(0x00)
                except ValueError:
                    break   # no delimiter, wait for more data
                if delim_idx > 0:
                    process_frame(buf[:delim_idx], callback)
                buf = buf[delim_idx+1:]
        else:
            time.sleep(0.001)


def process_frame(frame: bytes, callback):
    """Decode a COBS frame, verify CRC, extract index and data."""
    if len(frame) < 3:  # at least index(1) + data(min 1) + CRC(1)
        print("SMALL FRAME ERR")
        return
    #print("FRAME:", frame)
    #try:
    decoded = cobs.decode(frame)
    #except Exception:
    #    print("COBS DECODE ERR")
    #    return
    # decoded = index + data + crc_byte
    if len(decoded) < 3:
        print("SMALL FRAME ERR 2")
        return
    index = decoded[0]
    data  = decoded[1:-1]          # everything between index and CRC
    crc_recv = decoded[-1]
    crc_calc = crc8_model.calc(data)
    if crc_recv != crc_calc:
        # CRC mismatch – discard
        print("CRC ERR")
        return
    callback(index, data)


# ====== ROS 2 node ======

class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_serial_bridge')

        # --- Parameters ---
        port = self.declare_parameter('serial_port', '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B5E134448-if00').value
        baud = self.declare_parameter('baud', 115200).value

        # --- Publishers ---
        self.pub_status = self.create_publisher(Float32MultiArray, "/hardware/debug_status", 10)
        # self.pub_speed    = self.create_publisher(Float32, '/robot/speed', 10)
        # self.pub_steering = self.create_publisher(Float32, '/robot/steering', 10)
        # self.pub_active   = self.create_publisher(Bool, '/robot/active', 10)

        # --- Services ---
        self.create_service(Empty, '/activate', self.srv_activate)
        self.create_service(Empty, '/deactivate', self.srv_deactivate)

        # --- Subscriber ---
        # self.create_subscription(Twist, '/hardware/control_test', self.cmd_vel_cb, 10)
        self.create_subscription(ControlMode, "/control/mode", self.control_mode_cb, qos_profile=1)
        self.create_subscription(Control, "/control/command", self.control_command_cb, qos_profile=1)

        # --- Keep‑alive timer (200 ms) ---
        self.keepalive_timer = self.create_timer(1 / 10, self.keepalive_cb)

        # --- Internal state ---
        self.active = False
        self.target_steering = 0.0
        self.target_speed    = 0.0
        self._prev_mode = ControlMode.OFF

        self.ser = serial.Serial(port, baud, timeout=0)
        self.reader_thread = threading.Thread(target=read_packets, args=(self.ser, self.packet_callback), daemon=True)
        self.reader_thread.start()

        self.get_logger().info(f'Robot bridge started on {port}')

    # ----- Service callbacks -----
    def srv_activate(self, request, response):
        self.send_command(1)  # activate
        self.active = True
        return response

    def srv_deactivate(self, request, response):
        self.send_command(2)  # deactivate
        return response

    # ----- Subscriber callback -----
    # def cmd_vel_cb(self, msg: Twist):
    #     self.target_speed    = min(max(msg.linear.x, -1.0), 1.0)
    #     # self.target_steering = msg.angular.z
    #     # if self.active:
    #     self.send_control()

    def _enable(self):
        self.send_command(1)  # activate

    def _disable(self):
        self.send_command(2)  # deactivate

    def control_mode_cb(self, msg: ControlMode):
        # self.get_logger().info(f"MODE: {msg}")
        if msg.mode == self._prev_mode:
            return
        if self._prev_mode == ControlMode.OFF and msg.mode != ControlMode.OFF:
            self.get_logger().info("Mode change: OFF -> ANY - Enabling motor")
            self._enable()
        if self._prev_mode != ControlMode.OFF and msg.mode == ControlMode.OFF:
            self.get_logger().info("Mode change: ANY -> OFF - Disabling motor")
            self._disable()
        self._prev_mode = msg.mode
        # self._status_timer.reset()
        # self._push_status()

    def control_command_cb(self, msg: Control):
        # self.get_logger().info(f"CMD: {msg}")
        if self._prev_mode == ControlMode.OFF:
            return
            # self._disable_motor()
        self.target_steering = -min(max(msg.curvature / 0.58, -1.0), 1.0) * max_cri
        self.target_speed = min(max(msg.velocity / 1.0, -1.0), 1.0) * max_vel
        if self._prev_mode == ControlMode.AUTO:
            self.target_speed = 0 if msg.velocity < 0.01 else 0.2
        self.send_control()
        # self.get_logger().info(f"c={target_steering:.3f}, v={target_speed:.3f}")
        # self._axis.watchdog_feed()
        # rpm = self._model.linear_velocity_to_motor_rps(msg.velocity)
        # self._axis.controller.input_vel = rpm
        # twist = pymodel.Twist(msg.curvature, msg.velocity)
        # twist = self._model.base_to_rear_twist(twist)
        # steering = self._model.rear_twist_to_steering(twist)
        # self._log.debug(f"Center curvature: {msg.curvature:.2f}")
        # self._log.debug(f"Rear curvature: {twist.curvature:.2f}")
        # self._teensy.push(steering.left.radians, steering.right.radians)
        # self._target_curvature = msg.curvature

    # ----- Keep‑alive timer -----
    def keepalive_cb(self):
        pass
        # print("active:", self.active)
        # if self.active:
        #     self.send_control()

    # ----- Sending helpers -----
    def send_command(self, cmd_type: int):
        cmd = Command(cmd_type, 0.0, 0.0)
        send_packet(self.ser, cmd_type, cmd.pack()[1:])  # drop the leading 'type' byte,
                                                          # the index is the packet index
        # For activate (1) and deactivate (2), the data is two floats (0,0)
        # We must pack the command without the type byte because index already carries it.
        # Command.pack() returns [type, steering, speed] (9 bytes). We slice off the type.
        data_bytes = cmd.pack()[1:]  # steering + speed (8 bytes)
        send_packet(self.ser, cmd_type, data_bytes)

    def send_control(self):
        cmd = Command(3, self.target_steering, self.target_speed)
        data_bytes = cmd.pack() #[1:]
        # print("send", cmd)
        send_packet(self.ser, 3, data_bytes)

    # ----- Incoming packet callback (from reader thread) -----
    def packet_callback(self, index, data):
        if index == 0x10:   # status packet
            self.handle_status(data)

    def handle_status(self, data: bytes):
        # if len(data) != struct.calcsize('<ffB'):
        #     self.get_logger().warn(f'Invalid status length {len(data)}')
        #     return
        try:
            status = Status.unpack(data)
        except Exception as e:
            self.get_logger().error(f'Status unpack error: {e}')
            return
        
        msg = Float32MultiArray()
        # active, curr_speed, curr_steering, enc1_speed, enc1_angle, enc2_speed, enc2_angle
        msg.data = [
            float(status.active),
            float(self.target_speed),
            float(self.target_steering),
            float(status.enc1_speed_ticks),
            float(status.enc1_angle_ticks),
            float(status.enc2_speed_ticks),
            float(status.enc2_angle_ticks),
            # ...
        ]
        self.pub_status.publish(msg)

        #self.get_logger().info(f"STATUS: enc1_speed: {status.enc1_speed_ticks:.2f}, enc1_angle: {status.enc1_angle_ticks:.2f}")
        #self.pub_speed.publish(Float32(data=status.current_speed))
        #self.pub_steering.publish(Float32(data=status.current_steering))
        #self.pub_active.publish(Bool(data=status.active))
        #self.active = status.active   # synchronise with actual robot state

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
