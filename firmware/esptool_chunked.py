#!/usr/bin/env python3
"""esptool wrapper: chunk serial writes to survive macOS CDC driver bulk-write corruption.

The macOS CDC-ACM driver corrupts large bulk writes to the board's USB-serial
bridge (stub upload and flash writes fail with checksum/timeout errors, while
reads work). Writing in small chunks with a tiny inter-chunk delay avoids it.
"""
import sys
import time

import serial

CHUNK = 64

_orig_write = serial.Serial.write


def chunked_write(self, data):
    if isinstance(data, (bytes, bytearray)) and len(data) > CHUNK:
        # pace to the wire rate (10 bits/byte) + 20% margin so the bridge
        # chip's FIFO never overflows (macOS CDC driver has no flow control)
        baud = getattr(self, "baudrate", 115200) or 115200
        total = 0
        for i in range(0, len(data), CHUNK):
            chunk = data[i:i + CHUNK]
            total += _orig_write(self, chunk)
            self.flush()
            time.sleep(len(chunk) * 10.0 / baud * 1.2)
        return total
    return _orig_write(self, data)


serial.Serial.write = chunked_write

from esptool import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
