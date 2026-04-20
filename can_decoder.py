"""
can_decoder.py
CAN Bus Message Decoder for Automotive ECU Signals

Decodes raw CAN frames into human-readable signal values using
a configurable signal database (DBC-inspired). Supports standard
and extended CAN IDs, little-endian and big-endian byte orders.

Author: Bharathvaaj Mukunthan
"""

import struct
from dataclasses import dataclass, field
from typing import Optional


# ---------------------------------------------------------------------------
# Data models
# ---------------------------------------------------------------------------

@dataclass
class Signal:
    """Represents a single signal within a CAN message."""
    name: str
    start_bit: int
    length: int
    factor: float = 1.0
    offset: float = 0.0
    min_val: float = 0.0
    max_val: float = 0.0
    unit: str = ""
    byte_order: str = "little_endian"  # "little_endian" or "big_endian"

    def decode(self, raw_value: int) -> float:
        """Apply factor and offset to convert raw bit value to physical value."""
        physical = (raw_value * self.factor) + self.offset
        return round(physical, 4)


@dataclass
class Message:
    """Represents a CAN message with its ID, name, and signals."""
    can_id: int
    name: str
    dlc: int  # Data Length Code (number of bytes, 0–8)
    signals: list[Signal] = field(default_factory=list)


@dataclass
class DecodedSignal:
    """Result of decoding a single signal from a CAN frame."""
    name: str
    raw_value: int
    physical_value: float
    unit: str


@dataclass
class DecodedFrame:
    """Result of decoding a full CAN frame."""
    can_id: int
    message_name: str
    dlc: int
    raw_data: bytes
    signals: list[DecodedSignal] = field(default_factory=list)

    def __str__(self) -> str:
        raw_hex = " ".join(f"{b:02X}" for b in self.raw_data)
        lines = [
            f"\nMessage : {self.message_name} (ID: 0x{self.can_id:03X})",
            f"DLC     : {self.dlc}",
            f"Raw     : [{raw_hex}]",
            "Signals :",
        ]
        for sig in self.signals:
            lines.append(
                f"  {sig.name:<30} raw={sig.raw_value:<6}  "
                f"value={sig.physical_value} {sig.unit}"
            )
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Bit extraction helpers
# ---------------------------------------------------------------------------

def _extract_bits_little_endian(data: bytes, start_bit: int, length: int) -> int:
    """
    Extract a signal value using little-endian (Intel) byte order.
    start_bit is the LSB position counted from byte 0, bit 0.
    """
    raw = int.from_bytes(data, byteorder="little")
    mask = (1 << length) - 1
    return (raw >> start_bit) & mask


def _extract_bits_big_endian(data: bytes, start_bit: int, length: int) -> int:
    """
    Extract a signal value using big-endian (Motorola) byte order.
    start_bit is the MSB position in DBC convention.
    """
    raw = int.from_bytes(data, byteorder="big")
    total_bits = len(data) * 8
    # Convert DBC big-endian start_bit to MSB offset from the left
    msb_pos = total_bits - 1 - start_bit
    mask = (1 << length) - 1
    return (raw >> (msb_pos - length + 1)) & mask


# ---------------------------------------------------------------------------
# Core decoder
# ---------------------------------------------------------------------------

class CANDecoder:
    """
    Decodes raw CAN frames against a registered message/signal database.

    Usage:
        decoder = CANDecoder()
        decoder.add_message(Message(...))
        frame = decoder.decode(can_id=0x0C0, data=bytes([0x00, 0x10, ...]))
    """

    def __init__(self):
        self._db: dict[int, Message] = {}

    def add_message(self, message: Message) -> None:
        """Register a message definition in the decoder database."""
        self._db[message.can_id] = message

    def decode(self, can_id: int, data: bytes) -> Optional[DecodedFrame]:
        """
        Decode a raw CAN frame.

        Args:
            can_id: CAN arbitration ID (11-bit standard or 29-bit extended).
            data:   Up to 8 bytes of payload.

        Returns:
            DecodedFrame if the CAN ID is known, None otherwise.
        """
        message = self._db.get(can_id)
        if message is None:
            return None

        if len(data) < message.dlc:
            raise ValueError(
                f"Data too short for message '{message.name}': "
                f"expected {message.dlc} bytes, got {len(data)}."
            )

        frame = DecodedFrame(
            can_id=can_id,
            message_name=message.name,
            dlc=message.dlc,
            raw_data=data[:message.dlc],
        )

        for signal in message.signals:
            if signal.byte_order == "big_endian":
                raw_val = _extract_bits_big_endian(data, signal.start_bit, signal.length)
            else:
                raw_val = _extract_bits_little_endian(data, signal.start_bit, signal.length)

            physical_val = signal.decode(raw_val)
            frame.signals.append(
                DecodedSignal(
                    name=signal.name,
                    raw_value=raw_val,
                    physical_value=physical_val,
                    unit=signal.unit,
                )
            )

        return frame

    def known_ids(self) -> list[int]:
        """Return list of all registered CAN IDs."""
        return list(self._db.keys())


# ---------------------------------------------------------------------------
# Example signal database — Brake ECU inspired
# (Values are illustrative; not derived from any proprietary source)
# ---------------------------------------------------------------------------

def build_example_database() -> CANDecoder:
    """
    Build a small example signal database inspired by typical
    automotive brake ECU CAN messages (ABS, wheel speed, brake pressure).
    """
    decoder = CANDecoder()

    # --- Wheel Speed Message (0x0C0) ---
    wheel_speed_msg = Message(can_id=0x0C0, name="WheelSpeed", dlc=8)
    wheel_speed_msg.signals = [
        Signal("WheelSpeed_FL", start_bit=0,  length=16, factor=0.01, unit="km/h"),
        Signal("WheelSpeed_FR", start_bit=16, length=16, factor=0.01, unit="km/h"),
        Signal("WheelSpeed_RL", start_bit=32, length=16, factor=0.01, unit="km/h"),
        Signal("WheelSpeed_RR", start_bit=48, length=16, factor=0.01, unit="km/h"),
    ]
    decoder.add_message(wheel_speed_msg)

    # --- Brake Pressure Message (0x1A0) ---
    brake_msg = Message(can_id=0x1A0, name="BrakePressure", dlc=4)
    brake_msg.signals = [
        Signal("BrakePressure_Front", start_bit=0,  length=12, factor=0.1, unit="bar"),
        Signal("BrakePressure_Rear",  start_bit=12, length=12, factor=0.1, unit="bar"),
        Signal("BrakeLight_Active",   start_bit=24, length=1,  factor=1.0, unit="bool"),
        Signal("ABS_Active",          start_bit=25, length=1,  factor=1.0, unit="bool"),
    ]
    decoder.add_message(brake_msg)

    # --- ABS Status Message (0x2B0) ---
    abs_msg = Message(can_id=0x2B0, name="ABS_Status", dlc=3)
    abs_msg.signals = [
        Signal("ABS_FL_Active",         start_bit=0, length=1, unit="bool"),
        Signal("ABS_FR_Active",         start_bit=1, length=1, unit="bool"),
        Signal("ABS_RL_Active",         start_bit=2, length=1, unit="bool"),
        Signal("ABS_RR_Active",         start_bit=3, length=1, unit="bool"),
        Signal("EBD_Active",            start_bit=4, length=1, unit="bool"),
        Signal("TractionControl_Active",start_bit=5, length=1, unit="bool"),
        Signal("VehicleSpeed_ABS",      start_bit=8, length=16, factor=0.01, unit="km/h"),
    ]
    decoder.add_message(abs_msg)

    return decoder


# ---------------------------------------------------------------------------
# Demo
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    decoder = build_example_database()

    print("=" * 60)
    print("  CAN Bus Decoder — Automotive ECU Demo")
    print(f"  Registered message IDs: {[hex(i) for i in decoder.known_ids()]}")
    print("=" * 60)

    # Wheel speeds: FL=120.00 km/h, FR=119.50 km/h, RL=120.00 km/h, RR=119.50 km/h
    ws_data = bytes([0xD0, 0x2E, 0xC6, 0x2E, 0xD0, 0x2E, 0xC6, 0x2E])
    frame = decoder.decode(0x0C0, ws_data)
    print(frame)

    # Brake pressure: Front=45.0 bar, Rear=28.0 bar, BrakeLight=ON, ABS=ON
    bp_data = bytes([0xC2, 0x81, 0x11, 0x03])
    frame = decoder.decode(0x1A0, bp_data)
    print(frame)

    # ABS: FL active, EBD active, vehicle speed=98.56 km/h
    abs_data = bytes([0x11, 0x80, 0x26])
    frame = decoder.decode(0x2B0, abs_data)
    print(frame)

    # Unknown CAN ID
    result = decoder.decode(0x999, bytes([0xFF] * 8))
    print(f"\n0x999: {'Unknown message (not in database)' if result is None else result}")
