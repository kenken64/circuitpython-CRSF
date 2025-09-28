import unittest

from crsf import CRSF, ticks_to_us


class DummyUART:
    """Minimal UART stub for exercising the driver under CPython."""

    def __init__(self):
        self.rx = bytearray()
        self.tx = bytearray()
        self.config = {}

    def init(self, **kwargs):
        self.config.update(kwargs)

    def deinit(self):
        self.config.clear()

    def any(self):
        return len(self.rx)

    def read(self, nbytes):
        if not self.rx:
            return b""
        if nbytes is None or nbytes >= len(self.rx):
            data = bytes(self.rx)
            self.rx.clear()
            return data
        data = bytes(self.rx[:nbytes])
        del self.rx[:nbytes]
        return data

    def write(self, data):
        if isinstance(data, memoryview):
            data = data.tobytes()
        if isinstance(data, (bytes, bytearray)):
            self.tx.extend(data)
            return len(data)
        self.tx.append(int(data) & 0xFF)
        return 1

    # Helpers for tests
    def feed(self, data: bytes):
        self.rx.extend(data)


def _pack_channels(channels):
    packed = bytearray()
    accumulator = 0
    bits = 0
    for value in channels:
        accumulator |= (value & 0x7FF) << bits
        bits += 11
        while bits >= 8:
            packed.append(accumulator & 0xFF)
            accumulator >>= 8
            bits -= 8
    return bytes(packed)


def _build_frame(frame_type, payload):
    frame = bytearray()
    frame.append(0xC8)
    frame.append(len(payload) + 2)  # type + crc
    frame.append(frame_type)
    frame.extend(payload)
    driver = CRSF(DummyUART())
    crc = driver._crc(memoryview(frame)[2:])  # type: ignore[attr-defined]
    frame.append(crc)
    return bytes(frame)


class CRSFDriverTestCase(unittest.TestCase):
    def test_rc_channels_callback_invoked(self):
        uart = DummyUART()
        driver = CRSF(uart)

        captured = []

        def on_channels(values):
            captured.append(values)

        driver.set_on_rc_channels(on_channels)

        channels = [172 + i * 10 for i in range(16)]
        payload = _pack_channels(channels)
        frame = _build_frame(0x16, payload)
        uart.feed(frame)

        driver.process_frames()

        self.assertEqual(len(captured), 1)
        self.assertEqual(driver.channels[:4], tuple(channels[:4]))
        self.assertAlmostEqual(ticks_to_us(driver.channels[0]), 988.0, delta=0.5)

    def test_link_statistics_and_failsafe(self):
        uart = DummyUART()
        driver = CRSF(uart)

        events = []
        driver.set_on_link_statistics(lambda stats: events.append(("link", stats.as_tuple())))
        driver.set_on_failsafe(lambda state: events.append(("failsafe", state)))

        good_payload = bytes(
            [
                80,  # uplink_rssi_ant_1
                90,  # uplink_rssi_ant_2
                100,  # uplink package success rate
                10,  # uplink snr (2.5 dB)
                0,  # diversity antenna -> use ant 1
                3,  # rf mode (ignored)
                6,  # tx power enum -> 2000 mW (maps to 2000)
                70,  # downlink rssi
                95,  # downlink success rate
                12,  # downlink snr
            ]
        )
        bad_payload = bytes(
            [
                110,
                90,
                60,
                250,  # -6 dB
                0,
                3,
                4,  # 100 mW
                80,
                90,
                10,
            ]
        )

        uart.feed(_build_frame(0x14, good_payload))
        driver.process_frames()

        uart.feed(_build_frame(0x14, bad_payload))
        driver.process_frames()

        self.assertIn(("failsafe", False), events)
        self.assertIn(("failsafe", True), events)
        self.assertEqual(driver.link_statistics.rssi, 110)
        self.assertEqual(driver.link_statistics.tx_power, 500)

    def test_battery_telemetry_frame(self):
        uart = DummyUART()
        driver = CRSF(uart)

        driver.telem_set_battery_data(1000, 250, 12345, 80)
        driver.send_telem()

        self.assertGreater(len(uart.tx), 0)
        self.assertEqual(uart.tx[0], 0xC8)
        length = uart.tx[1]
        self.assertEqual(length, len(uart.tx) - 2)
        self.assertEqual(uart.tx[2], 0x08)
        voltage = (uart.tx[3] << 8) | uart.tx[4]
        self.assertEqual(voltage, 1000)

    def test_custom_payload_telemetry_frame(self):
        uart = DummyUART()
        driver = CRSF(uart)

        driver.telem_set_custom_payload(b"XYZ")
        driver.send_telem()

        self.assertGreater(len(uart.tx), 0)
        self.assertEqual(uart.tx[2], 0x7F)
        self.assertEqual(bytes(uart.tx[3:6]), b"XYZ")


if __name__ == "__main__":
    unittest.main()
