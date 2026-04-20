"""
test_can_decoder.py
Unit tests for the CAN Bus Message Decoder.
"""

import pytest
from can_decoder import (
    CANDecoder, Message, Signal,
    _extract_bits_little_endian, _extract_bits_big_endian,
    build_example_database,
)


class TestBitExtraction:
    def test_little_endian_single_byte(self):
        data = bytes([0b10110100])
        assert _extract_bits_little_endian(data, 2, 4) == 0b1101

    def test_little_endian_multi_byte(self):
        data = bytes([0xD0, 0x2E])  # 0x2ED0 = 11984 → 11984 * 0.01 = 119.84
        assert _extract_bits_little_endian(data, 0, 16) == 11984

    def test_little_endian_single_bit(self):
        data = bytes([0b00000100])
        assert _extract_bits_little_endian(data, 2, 1) == 1
        assert _extract_bits_little_endian(data, 1, 1) == 0


class TestSignalDecode:
    def test_factor_and_offset(self):
        s = Signal("test", 0, 8, factor=0.5, offset=10.0)
        assert s.decode(100) == 60.0

    def test_zero_raw(self):
        s = Signal("test", 0, 8, factor=1.0, offset=0.0)
        assert s.decode(0) == 0.0

    def test_unit_stored(self):
        s = Signal("speed", 0, 8, unit="km/h")
        assert s.unit == "km/h"


class TestCANDecoder:
    def setup_method(self):
        self.decoder = build_example_database()

    def test_unknown_id_returns_none(self):
        result = self.decoder.decode(0x999, bytes(8))
        assert result is None

    def test_wheel_speed_decode(self):
        # FL wheel speed raw = 12000 → 12000 * 0.01 = 120.00 km/h
        data = bytes([0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E])
        frame = self.decoder.decode(0x0C0, data)
        assert frame is not None
        assert frame.message_name == "WheelSpeed"
        fl = next(s for s in frame.signals if s.name == "WheelSpeed_FL")
        assert fl.physical_value == 120.0

    def test_abs_active_flag(self):
        # Byte 0 bit 0 = ABS_FL, bit 4 = EBD; set both
        data = bytes([0x11, 0x00, 0x00])
        frame = self.decoder.decode(0x2B0, data)
        abs_fl = next(s for s in frame.signals if s.name == "ABS_FL_Active")
        ebd = next(s for s in frame.signals if s.name == "EBD_Active")
        assert abs_fl.physical_value == 1.0
        assert ebd.physical_value == 1.0

    def test_data_too_short_raises(self):
        with pytest.raises(ValueError):
            self.decoder.decode(0x0C0, bytes([0x00, 0x01]))  # needs 8, got 2

    def test_known_ids(self):
        ids = self.decoder.known_ids()
        assert 0x0C0 in ids
        assert 0x1A0 in ids
        assert 0x2B0 in ids

    def test_decoded_frame_str(self):
        data = bytes([0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E])
        frame = self.decoder.decode(0x0C0, data)
        output = str(frame)
        assert "WheelSpeed" in output
        assert "0xC0" in output.upper() or "0x0C0" in output.upper() or "C0" in output.upper()
