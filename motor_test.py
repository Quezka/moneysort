#!/usr/bin/env python3
"""Motor 1 first-pulse test - direct 3.3V common-cathode (no shifter).

Wiring (common-cathode, HIGH = active):
    GPIO17 -> PUL+     GPIO27 -> DIR+
    all '-' terminals bussed to Pi GND
    ENA disconnected -> driver always enabled

Usage (run on the Pi):
    python3 motor_test.py [pulses] [direction]
        pulses    : step pulses to send (default 50)
        direction : 1 or 0 (default 1)
"""
import sys
from time import sleep
from gpiozero import OutputDevice

PUL = OutputDevice(17)   # HIGH = one step
DIR = OutputDevice(27)   # sets rotation direction


def move(pulses, direction=1, delay=0.0015):
    DIR.value = 1 if direction else 0
    sleep(0.002)                 # let DIR settle before pulsing
    for _ in range(pulses):
        PUL.on()
        sleep(delay)             # ~1.5 ms high
        PUL.off()
        sleep(delay)             # ~330 pulses/sec


n = int(sys.argv[1]) if len(sys.argv) > 1 else 50
d = int(sys.argv[2]) if len(sys.argv) > 2 else 1
print(f"Sending {n} pulses, direction={d} ...")
try:
    move(n, d)
    print("done")
finally:
    PUL.off()
