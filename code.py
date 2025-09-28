from crsf import BAUD_RATE, CRSF, ticks_to_us

# Now set up CRSF
import board
import busio
import time

# Configuration ------------------------------------------------------------
# Adjust these pins to match your wiring. For RP2040 boards running CircuitPython,
# GP4/GP5 (physical pins 6/7) align with the default UART1 port used by ELRS receivers.
UART_TX_PIN = board.GP4
UART_RX_PIN = board.GP5

# Instantiate UART at the CRSF baud rate. Increase receiver_buffer_size if you see
# buffer overflows on high packet rates.
uart = busio.UART(
    UART_TX_PIN,
    UART_RX_PIN,
    baudrate=BAUD_RATE,
    timeout=0,
    receiver_buffer_size=512,
)

def _sleep_ms(ms: int) -> None:
    time.sleep(ms / 1000.0)

def _timestamp_label() -> str:
    return "{:.3f}s".format(time.monotonic())

# Create CRSF driver
crsf = CRSF(uart)


# Shared state for telemetry presentation
latest_channels_us = [0.0] * 10
latest_channels_timestamp = "never"


# Callback handlers -------------------------------------------------------------
def on_channels(channels):
    # Render the first ten RC channels (or fewer if not provided) in microseconds
    global latest_channels_timestamp
    count = min(10, len(channels))
    report = []
    for idx in range(count):
        ch_us = ticks_to_us(channels[idx])
        latest_channels_us[idx] = ch_us
        report.append("RC{}: {:0.1f} µs".format(idx + 1, ch_us))

    # Zero any remaining cached channels when less than ten are received
    for idx in range(count, 10):
        latest_channels_us[idx] = 0.0

    latest_channels_timestamp = _timestamp_label()
    print(" | ".join(report))


def on_link(stats):
    print("Link RSSI:{} dBm  LQ:{}%  TX:{} mW".format(stats.rssi, stats.link_quality, stats.tx_power))


def on_failsafe(state):
    if state:
        print("Failsafe engaged!")
    else:
        print("Link recovered")


crsf.set_on_rc_channels(on_channels)
crsf.set_on_link_statistics(on_link)
crsf.set_on_failsafe(on_failsafe)


# Telemetry example -------------------------------------------------------------
# Populate battery data (values follow CRSF units)
crsf.telem_set_battery_data(
    voltage=1170,  # decivolts
    current=320,   # deciamps
    capacity=1650, # mAh
    percent=82,
)

# Optional: send a short custom payload
crsf.telem_set_custom_payload(b"Pico2 Demo")


# Main loop --------------------------------------------------------------
while True:
    crsf.process_frames()
    _sleep_ms(5)