"""Crossfire (CRSF) protocol driver for MicroPython and CircuitPython.

This module ports the key features of the AlfredoCRSF C++ driver to
MicroPython/CircuitPython so it can run on boards such as the Raspberry Pi Pico 2
and RP2040 boards running Adafruit firmware. It maintains parity with the
original implementation:
It maintains parity with the original implementation:

* Reads CRSF frames from a UART running at 420 kbaud
* Parses RC channel and link statistics payloads
* Tracks failsafe state based on link quality and RSSI thresholds
* Provides battery and custom telemetry frame builders
* Streams outbound telemetry in alternating fashion when data is ready

The code is structured to run on both MicroPython and CircuitPython, with
CPython-friendly fallbacks for unit testing where the hardware-specific
modules (``machine``/``busio``/``utime``) are unavailable.
"""
from __future__ import annotations

import time

try:  # pragma: no cover - MicroPython constant helper
    from micropython import const  # type: ignore
except ImportError:  # pragma: no cover - CPython/CircuitPython fallback
    def const(value: int) -> int:
        return value

try:
    _monotonic_ns = time.monotonic_ns  # type: ignore[attr-defined]
except AttributeError:  # pragma: no cover - older interpreters
    _monotonic_ns = None


def ticks_us() -> int:
    if _monotonic_ns is not None:
        return _monotonic_ns() // 1_000
    return int(time.monotonic() * 1_000_000)


def ticks_ms() -> int:
    if _monotonic_ns is not None:
        return _monotonic_ns() // 1_000_000
    return int(time.monotonic() * 1_000)


def ticks_diff(end: int, start: int) -> int:
    return end - start


def ticks_add(start: int, delta: int) -> int:
    return start + delta

# ---------------------------------------------------------------------------
# Protocol-level constants
# ---------------------------------------------------------------------------
BAUD_RATE = const(420_000)
CRSF_MAX_CHANNELS = const(16)
CRSF_MAX_FRAME_SIZE = const(64)
CRSF_MAX_PAYLOAD_SIZE = const(CRSF_MAX_FRAME_SIZE - 4)
CRSF_SYNC_BYTES = (0xC8, 0xEE)

# Frame type identifiers (align with crsf.h)
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = const(0x16)
CRSF_FRAMETYPE_BATTERY_SENSOR = const(0x08)
CRSF_FRAMETYPE_LINK_STATISTICS = const(0x14)
CRSF_FRAMETYPE_CUSTOM_PAYLOAD = const(0x7F)

# Telemetry round-robin order
CRSF_BATTERY_INDEX = const(0)
CRSF_CUSTOM_PAYLOAD_INDEX = const(1)
TELEMETRY_FRAME_TYPES = const(2)

# Threshold defaults
DEFAULT_LINK_QUALITY_THRESHOLD = const(70)
DEFAULT_RSSI_THRESHOLD = const(105)

# CRSF util conversion (ticks to microseconds) matches macro in header
TICKS_TO_US_NUMERATOR = 5.0
TICKS_TO_US_DENOMINATOR = 8.0
TICKS_TO_US_OFFSET = 1500.0


def ticks_to_us(raw: int) -> float:
    """Convert a CRSF RC channel tick value to microseconds."""

    return (raw - 992.0) * TICKS_TO_US_NUMERATOR / TICKS_TO_US_DENOMINATOR + TICKS_TO_US_OFFSET


def _available_bytes(uart) -> int:
    """Return the number of bytes ready to be read from UART."""

    if hasattr(uart, "any"):
        try:
            value = uart.any()
        except TypeError:
            value = None
        if value is None:
            return 0
        if isinstance(value, bool):
            return 1 if value else 0
        try:
            return int(value)
        except (TypeError, ValueError):
            return 0
    if hasattr(uart, "in_waiting"):
        try:
            value = getattr(uart, "in_waiting")
            return int(value)
        except (TypeError, ValueError, AttributeError):
            return 0
    return 0


class LinkStatistics:
    """Lightweight container mirroring ``link_statistics_t`` from C."""

    __slots__ = ("rssi", "link_quality", "snr", "tx_power")

    def __init__(self, rssi: int = 0, link_quality: int = 0, snr: int = 0, tx_power: int = 0) -> None:
        self.rssi = rssi
        self.link_quality = link_quality
        self.snr = snr
        self.tx_power = tx_power

    def as_tuple(self) -> tuple[int, int, int, int]:
        return self.rssi, self.link_quality, self.snr, self.tx_power


class _TelemetryBattery:
    __slots__ = ("voltage", "current", "capacity", "percent")

    def __init__(self) -> None:
        self.voltage = 0
        self.current = 0
        self.capacity = 0
        self.percent = 0


class _TelemetryCustom:
    __slots__ = ("buffer", "length")

    def __init__(self) -> None:
        self.buffer = bytearray(60)
        self.length = 0


class CRSF:
    """CRSF protocol handler.

    Parameters
    ----------
    uart:
        A ``machine.UART`` instance (or compatible object) configured for
        420000 baud, 8-N-1. For unit tests, any duck-typed object with
        ``read``, ``write``, ``any`` and ``sendbreak``/``flush`` no-ops is
        acceptable.
    link_quality_threshold:
        Minimum acceptable link quality (%). Falling below this value will
        trigger failsafe notifications.
    rssi_threshold:
        Maximum acceptable RSSI value (in dBm * -1). Exceeding this value
        also trips failsafe.
    """

    def __init__(
        self,
        uart,
        *,
        link_quality_threshold: int = DEFAULT_LINK_QUALITY_THRESHOLD,
        rssi_threshold: int = DEFAULT_RSSI_THRESHOLD,
    ) -> None:
        self._uart = uart

        # Callbacks
        self._rc_channels_cb = None
        self._link_stats_cb = None
        self._failsafe_cb = None

        # Runtime state
        self._incoming_frame = bytearray(CRSF_MAX_FRAME_SIZE)
        self._frame_index = 0
        self._frame_length = 0
        self._crc_index = 0
        self._rc_channels = [0] * CRSF_MAX_CHANNELS
        self._link_statistics = LinkStatistics()
        self._failsafe = True
        self._link_quality_threshold = link_quality_threshold
        self._rssi_threshold = rssi_threshold

        # Telemetry buffers
        self._telem_buf = bytearray(CRSF_MAX_FRAME_SIZE)
        self._telem_len = 0
        self._battery = _TelemetryBattery()
        self._custom = _TelemetryCustom()
        self._frame_has_data = [False] * TELEMETRY_FRAME_TYPES
        self._telem_rotation = 0

        # Telemetry CRC lookup shared with inbound processing
        self._crc_table = _CRSFCRC8_TABLE

    # ------------------------------------------------------------------
    # Public API mirrors crsf.h
    # ------------------------------------------------------------------
    @property
    def channels(self):
        """Return the most recent raw RC channel values."""

        return tuple(self._rc_channels)

    @property
    def link_statistics(self) -> LinkStatistics:
        """Return the latest link statistics snapshot."""

        return self._link_statistics

    @property
    def failsafe(self) -> bool:
        """Current failsafe state."""

        return self._failsafe

    def set_on_rc_channels(self, callback=None):
        """Register/unregister the RC channels callback."""

        self._rc_channels_cb = callback

    def set_on_link_statistics(self, callback=None):
        """Register/unregister the link statistics callback."""

        self._link_stats_cb = callback

    def set_on_failsafe(self, callback=None):
        """Register/unregister the failsafe callback."""

        self._failsafe_cb = callback

    def set_link_quality_threshold(self, threshold: int) -> None:
        self._link_quality_threshold = threshold

    def set_rssi_threshold(self, threshold: int) -> None:
        self._rssi_threshold = threshold

    def begin(self) -> None:
        """Initialize UART if it exposes an ``init`` method."""

        if hasattr(self._uart, "init"):
            self._uart.init(baudrate=BAUD_RATE)

    def end(self) -> None:
        """Deinitialize UART if possible."""

        if hasattr(self._uart, "deinit"):
            self._uart.deinit()

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------
    def telem_set_battery_data(self, voltage: int, current: int, capacity: int, percent: int) -> None:
        self._battery.voltage = voltage
        self._battery.current = current
        self._battery.capacity = capacity
        self._battery.percent = percent
        self._frame_has_data[CRSF_BATTERY_INDEX] = True

    def telem_set_custom_payload(self, data: bytes, length=None) -> None:
        if length is None:
            length = len(data)
        if length > len(self._custom.buffer):
            raise ValueError("custom payload length exceeds 60 bytes")
        mv = memoryview(self._custom.buffer)
        mv[:length] = data[:length]
        self._custom.length = length
        self._frame_has_data[CRSF_CUSTOM_PAYLOAD_INDEX] = True

    # ------------------------------------------------------------------
    # Incoming frame processing
    # ------------------------------------------------------------------
    def process_frames(self, timeout_us=None) -> None:
        """Pump the RX UART and send telemetry if ready.

        Parameters
        ----------
        timeout_us:
            Maximum wait for new data in microseconds. ``None`` (default)
            drains all currently buffered data without waiting.
        """

        deadline = None
        if timeout_us is not None:
            deadline = ticks_add(ticks_us(), timeout_us)

        while True:
            if deadline is not None and ticks_diff(deadline, ticks_us()) <= 0:
                break

            available = _available_bytes(self._uart)
            if available <= 0:
                if timeout_us is None:
                    break
                continue

            read_size = available
            if read_size > CRSF_MAX_FRAME_SIZE:
                read_size = CRSF_MAX_FRAME_SIZE
            try:
                chunk = self._uart.read(read_size)
            except TypeError:
                chunk = self._uart.read()
            if not chunk:
                if timeout_us is None:
                    break
                continue
            for byte in chunk:
                self._process_byte(byte)

        self.send_telem()

    def process_byte(self, current_byte: int) -> None:
        """Feed a single byte into the frame parser (testing helper)."""

        self._process_byte(current_byte & 0xFF)

    def _process_byte(self, current_byte: int) -> None:
        if self._frame_index == 0:
            if current_byte not in CRSF_SYNC_BYTES:
                return
            self._incoming_frame[self._frame_index] = current_byte
            self._frame_index = 1
            return

        if self._frame_index == 1:
            self._incoming_frame[self._frame_index] = current_byte
            self._frame_index = 2
            self._frame_length = current_byte
            if self._frame_length < 2 or self._frame_length > (CRSF_MAX_FRAME_SIZE - 2):
                self._frame_index = 0
            else:
                self._crc_index = self._frame_length + 1
            return

        if self._frame_index == self._crc_index:
            # Validate CRC on payload+type
            if self._crc(self._incoming_frame[2 : 2 + self._frame_length - 1]) == current_byte:
                self._incoming_frame[self._frame_index] = current_byte
                self._process_frame_complete()
            self._frame_index = 0
            return

        if self._frame_index < CRSF_MAX_FRAME_SIZE:
            self._incoming_frame[self._frame_index] = current_byte
            self._frame_index += 1
        else:
            self._frame_index = 0  # Overflow guard

    def _process_frame_complete(self) -> None:
        frame_type = self._incoming_frame[2]
        if frame_type == CRSF_FRAMETYPE_LINK_STATISTICS:
            self._process_link_statistics(memoryview(self._incoming_frame)[3:])
        elif frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            self._process_rc_channels(memoryview(self._incoming_frame)[3:])
        # Unknown frame types are ignored but keep resilience

    def _process_rc_channels(self, data: memoryview) -> None:
        value = 0
        bits = 0
        index = 0
        for channel in range(CRSF_MAX_CHANNELS):
            while bits < 11:
                value |= data[index] << bits
                index += 1
                bits += 8
            channel_value = value & 0x7FF
            self._rc_channels[channel] = channel_value
            value >>= 11
            bits -= 11

        if self._rc_channels_cb:
            self._rc_channels_cb(tuple(self._rc_channels))

    _TX_POWER_TABLE = (0, 10, 25, 100, 500, 1000, 2000, 250, 50)

    def _process_link_statistics(self, data: memoryview) -> None:
        uplink_rssi_ant_1 = data[0]
        uplink_rssi_ant_2 = data[1]
        uplink_package_success_rate = data[2]
        uplink_snr = _to_signed_byte(data[3])
        diversity_active_antenna = data[4]
        uplink_tx_power_raw = data[6]

        rssi = uplink_rssi_ant_2 if diversity_active_antenna else uplink_rssi_ant_1
        tx_power = self._TX_POWER_TABLE[uplink_tx_power_raw] if uplink_tx_power_raw < len(self._TX_POWER_TABLE) else 0

        self._link_statistics.rssi = rssi
        self._link_statistics.link_quality = uplink_package_success_rate
        self._link_statistics.snr = uplink_snr
        self._link_statistics.tx_power = tx_power

        self._maybe_update_failsafe()

        if self._link_stats_cb:
            self._link_stats_cb(self._link_statistics)

    def _maybe_update_failsafe(self) -> None:
        new_state = (
            self._link_statistics.link_quality <= self._link_quality_threshold
            or self._link_statistics.rssi >= self._rssi_threshold
        )
        if new_state != self._failsafe:
            self._failsafe = new_state
            if self._failsafe_cb:
                self._failsafe_cb(self._failsafe)

    # ------------------------------------------------------------------
    # Telemetry outbound path
    # ------------------------------------------------------------------
    def send_telem(self) -> None:
        if self._telem_update():
            payload = self._telem_buf[: self._telem_len]
            if hasattr(self._uart, "write"):
                self._uart.write(payload)

    def _telem_update(self) -> bool:
        for _ in range(TELEMETRY_FRAME_TYPES):
            idx = self._telem_rotation % TELEMETRY_FRAME_TYPES
            self._telem_rotation += 1
            if not self._frame_has_data[idx]:
                continue

            self._frame_has_data[idx] = False
            self._begin_frame()
            if idx == CRSF_BATTERY_INDEX:
                self._write_battery_sensor_payload()
            elif idx == CRSF_CUSTOM_PAYLOAD_INDEX:
                self._write_custom_payload()
            else:  # pragma: no cover - safeguard
                continue
            self._end_frame()
            return True
        return False

    def _begin_frame(self) -> None:
        self._telem_len = 0
        self._write_ui8(0xC8)
        self._write_ui8(0)  # placeholder length, filled later

    def _end_frame(self) -> None:
        crc = self._crc(self._telem_buf[2 : self._telem_len])
        self._write_ui8(crc)
        self._telem_buf[1] = self._telem_len - 2

    def _write_battery_sensor_payload(self) -> None:
        self._write_ui8(CRSF_FRAMETYPE_BATTERY_SENSOR)
        self._write_ui16(self._battery.voltage)
        self._write_ui16(self._battery.current)
        self._write_ui24(self._battery.capacity)
        self._write_ui8(self._battery.percent)

    def _write_custom_payload(self) -> None:
        length = self._custom.length
        self._write_ui8(CRSF_FRAMETYPE_CUSTOM_PAYLOAD)
        for i in range(length):
            self._write_ui8(self._custom.buffer[i])

    # ------------------------------------------------------------------
    # Buffer helpers
    # ------------------------------------------------------------------
    def _write_ui8(self, value: int) -> None:
        if self._telem_len >= CRSF_MAX_FRAME_SIZE:
            raise RuntimeError("telemetry buffer overflow")
        self._telem_buf[self._telem_len] = value & 0xFF
        self._telem_len += 1

    def _write_ui16(self, value: int) -> None:
        self._write_ui8((value >> 8) & 0xFF)
        self._write_ui8(value & 0xFF)

    def _write_ui24(self, value: int) -> None:
        self._write_ui8((value >> 16) & 0xFF)
        self._write_ui8((value >> 8) & 0xFF)
        self._write_ui8(value & 0xFF)

    def _crc(self, data: memoryview) -> int:
        crc = 0
        for byte in data:
            crc = self._crc_table[crc ^ byte]
        return crc


def _to_signed_byte(value: int) -> int:
    return value - 256 if value & 0x80 else value


# CRC table matches original implementation
_CRSFCRC8_TABLE = (
    0x00,
    0xD5,
    0x7F,
    0xAA,
    0xFE,
    0x2B,
    0x81,
    0x54,
    0x29,
    0xFC,
    0x56,
    0x83,
    0xD7,
    0x02,
    0xA8,
    0x7D,
    0x52,
    0x87,
    0x2D,
    0xF8,
    0xAC,
    0x79,
    0xD3,
    0x06,
    0x7B,
    0xAE,
    0x04,
    0xD1,
    0x85,
    0x50,
    0xFA,
    0x2F,
    0xA4,
    0x71,
    0xDB,
    0x0E,
    0x5A,
    0x8F,
    0x25,
    0xF0,
    0x8D,
    0x58,
    0xF2,
    0x27,
    0x73,
    0xA6,
    0x0C,
    0xD9,
    0xF6,
    0x23,
    0x89,
    0x5C,
    0x08,
    0xDD,
    0x77,
    0xA2,
    0xDF,
    0x0A,
    0xA0,
    0x75,
    0x21,
    0xF4,
    0x5E,
    0x8B,
    0x9D,
    0x48,
    0xE2,
    0x37,
    0x63,
    0xB6,
    0x1C,
    0xC9,
    0xB4,
    0x61,
    0xCB,
    0x1E,
    0x4A,
    0x9F,
    0x35,
    0xE0,
    0xCF,
    0x1A,
    0xB0,
    0x65,
    0x31,
    0xE4,
    0x4E,
    0x9B,
    0xE6,
    0x33,
    0x99,
    0x4C,
    0x18,
    0xCD,
    0x67,
    0xB2,
    0x39,
    0xEC,
    0x46,
    0x93,
    0xC7,
    0x12,
    0xB8,
    0x6D,
    0x10,
    0xC5,
    0x6F,
    0xBA,
    0xEE,
    0x3B,
    0x91,
    0x44,
    0x6B,
    0xBE,
    0x14,
    0xC1,
    0x95,
    0x40,
    0xEA,
    0x3F,
    0x42,
    0x97,
    0x3D,
    0xE8,
    0xBC,
    0x69,
    0xC3,
    0x16,
    0xEF,
    0x3A,
    0x90,
    0x45,
    0x11,
    0xC4,
    0x6E,
    0xBB,
    0xC6,
    0x13,
    0xB9,
    0x6C,
    0x38,
    0xED,
    0x47,
    0x92,
    0xBD,
    0x68,
    0xC2,
    0x17,
    0x43,
    0x96,
    0x3C,
    0xE9,
    0x94,
    0x41,
    0xEB,
    0x3E,
    0x6A,
    0xBF,
    0x15,
    0xC0,
    0x4B,
    0x9E,
    0x34,
    0xE1,
    0xB5,
    0x60,
    0xCA,
    0x1F,
    0x62,
    0xB7,
    0x1D,
    0xC8,
    0x9C,
    0x49,
    0xE3,
    0x36,
    0x19,
    0xCC,
    0x66,
    0xB3,
    0xE7,
    0x32,
    0x98,
    0x4D,
    0x30,
    0xE5,
    0x4F,
    0x9A,
    0xCE,
    0x1B,
    0xB1,
    0x64,
    0x72,
    0xA7,
    0x0D,
    0xD8,
    0x8C,
    0x59,
    0xF3,
    0x26,
    0x5B,
    0x8E,
    0x24,
    0xF1,
    0xA5,
    0x70,
    0xDA,
    0x0F,
    0x20,
    0xF5,
    0x5F,
    0x8A,
    0xDE,
    0x0B,
    0xA1,
    0x74,
    0x09,
    0xDC,
    0x76,
    0xA3,
    0xF7,
    0x22,
    0x88,
    0x5D,
    0xD6,
    0x03,
    0xA9,
    0x7C,
    0x28,
    0xFD,
    0x57,
    0x82,
    0xFF,
    0x2A,
    0x80,
    0x55,
    0x01,
    0xD4,
    0x7E,
    0xAB,
    0x84,
    0x51,
    0xFB,
    0x2E,
    0x7A,
    0xAF,
    0x05,
    0xD0,
    0xAD,
    0x78,
    0xD2,
    0x07,
    0x53,
    0x86,
    0x2C,
    0xF9,
)
