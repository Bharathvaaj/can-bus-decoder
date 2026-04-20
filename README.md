# CAN Bus Message Decoder

A lightweight Python library for decoding raw CAN (Controller Area Network) frames into human-readable signal values, using a configurable signal database.

Built as a learning and demonstration project drawing on experience in automotive ECU software development (ABS, EBD, traction control systems).

---

## Features

- Decode standard (11-bit) and extended (29-bit) CAN IDs
- Support for **little-endian (Intel)** and **big-endian (Motorola)** byte orders
- Configurable signal database — define your own messages and signals
- Factor/offset conversion from raw bit values to physical units
- Clean, typed Python with `dataclasses` — no external dependencies
- Example database inspired by typical brake ECU signals (wheel speed, brake pressure, ABS status)
- Unit tests with `pytest`

---

## Project structure

```
can_decoder/
├── can_decoder.py        # Core decoder library
├── test_can_decoder.py   # Unit tests (pytest)
└── README.md
```

---

## Quick start

**No installation needed** — pure Python 3.10+, no external dependencies.

```bash
git clone https://github.com/<your-username>/can-decoder.git
cd can-decoder
python can_decoder.py
```

**Example output:**

```
============================================================
  CAN Bus Decoder — Automotive ECU Demo
  Registered message IDs: ['0xc0', '0x1a0', '0x2b0']
============================================================

Message : WheelSpeed (ID: 0x0C0)
DLC     : 8
Raw     : [D0 2E C6 2E D0 2E C6 2E]
Signals :
  WheelSpeed_FL                  raw=11984   value=119.84 km/h
  WheelSpeed_FR                  raw=11974   value=119.74 km/h
  WheelSpeed_RL                  raw=11984   value=119.84 km/h
  WheelSpeed_RR                  raw=11974   value=119.74 km/h

Message : BrakePressure (ID: 0x1A0)
DLC     : 4
Raw     : [C2 0B 01 03]
Signals :
  BrakePressure_Front            raw=450     value=45.0 bar
  BrakePressure_Rear             raw=280     value=28.0 bar
  BrakeLight_Active              raw=1       value=1.0 bool
  ABS_Active                     raw=1       value=1.0 bool
```

---

## Define your own signals

```python
from can_decoder import CANDecoder, Message, Signal

decoder = CANDecoder()

# Define a message
engine_msg = Message(can_id=0x316, name="EngineStatus", dlc=4)
engine_msg.signals = [
    Signal("EngineRPM",   start_bit=0,  length=16, factor=0.25, unit="rpm"),
    Signal("Throttle",    start_bit=16, length=8,  factor=0.4,  unit="%"),
    Signal("EngineReady", start_bit=24, length=1,  unit="bool"),
]

decoder.add_message(engine_msg)

# Decode a raw frame
frame = decoder.decode(0x316, bytes([0x20, 0x1F, 0x96, 0x01]))
print(frame)
```

---

## Run tests

```bash
pip install pytest
pytest test_can_decoder.py -v
```

---

## Background

CAN bus is the backbone communication protocol in automotive ECUs. Each ECU broadcasts messages on the bus at fixed intervals — for example, a brake ECU might send wheel speed data at 10 ms intervals and ABS status at 20 ms intervals. This decoder replicates the signal unpacking step that tools like Vector CANoe perform when monitoring live vehicle data.

**Key concepts:**
| Term | Meaning |
|------|---------|
| CAN ID | Arbitration identifier — determines message priority on the bus |
| DLC | Data Length Code — number of payload bytes (0–8) |
| Signal | A logical value packed into specific bits of the payload |
| Factor / Offset | Converts raw integer to a physical value: `physical = raw × factor + offset` |
| Little-endian | LSB in the lowest bit position (Intel/default in most passenger cars) |
| Big-endian | MSB first (Motorola/used in some legacy ECUs) |

---

## Roadmap

- [ ] Parse `.dbc` files to auto-load signal databases
- [ ] Live decoding from `python-can` (USB-to-CAN adapter support)
- [ ] CSV/JSON logging of decoded frames
- [ ] CLI interface for piping raw CAN logs

---

## Author

**Bharathvaaj Mukunthan**  
Master's student, Communication Systems and Networks — TH Köln  
Former Embedded Software Engineer, Bosch Global Software Technologies  
[LinkedIn](https://linkedin.com/in/) · [Email](mailto:bharathvaaj1@gmail.com)

---

## License

MIT License — free to use, modify, and distribute.
