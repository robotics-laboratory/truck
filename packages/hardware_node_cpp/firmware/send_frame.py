import sys
from struct import pack
import subprocess

def create_can_frame_hex(value):
    data_bytes = pack('f', float(value)) * 2
    return ''.join(format(byte, '02X') for byte in data_bytes)

def send_can_frame(can_frame_hex, can_id="555", device="vcan0"):
    command = f"cansend {device} {can_id}#{can_frame_hex}"
    subprocess.run(command, shell=True)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: ./script.py value")
        sys.exit(1)
    value = sys.argv[1]
    try:
        can_frame_hex = create_can_frame_hex(value)
        send_can_frame(can_frame_hex)
    except ValueError as e:
        print("non-valid float")