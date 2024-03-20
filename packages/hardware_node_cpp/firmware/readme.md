## Teensy MCU firmware

Currently, we use teensy 4.0 MCU to control two servo motors for steering. Communication is done via serial and [msgpacketizer](https://github.com/hideakitai/MsgPacketizer) library.

### Quickstart

Install platformio:
```
pip install platformio
```

Build and upload:
```
pio run -t upload
```

Run basic test:
```
python test.py
```
