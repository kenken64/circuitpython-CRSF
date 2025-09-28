"""MicroPython CRSF driver for Raspberry Pi Pico 2.

Example
-------
.. code-block:: python

    from machine import UART, Pin
    from crsf import CRSF, BAUD_RATE

    uart = UART(1, baudrate=BAUD_RATE, tx=Pin(4), rx=Pin(5))
    crsf = CRSF(uart)

    from crsf import ticks_to_us

    def on_channels(channels):
        print("CH1 µs:", ticks_to_us(channels[0]))

    crsf.set_on_rc_channels(on_channels)

    while True:
        crsf.process_frames()
"""

from .core import BAUD_RATE, CRSF, LinkStatistics, ticks_to_us

__all__ = ["BAUD_RATE", "CRSF", "LinkStatistics", "ticks_to_us"]
