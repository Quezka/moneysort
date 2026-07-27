#!/usr/bin/env python3
"""Motor 1 test - direct 3.3V common-cathode (no shifter needed).

Wiring (common-cathode, HIGH = active):
    GPIO17 -> PUL+     GPIO27 -> DIR+
    all '-' terminals bussed to Pi GND
    ENA disconnected -> driver always enabled

Usage (run on the Pi):
    python3 motor_test.py [pulses] [direction] [pps]
        pulses    : number of step pulses (default 800)
        direction : 1 or 0 (default 1)
        pps       : target pulses per second at cruise (default 1000)

Includes a linear acceleration ramp so higher speeds don't stall the motor:
it starts slow, ramps up to `pps`, cruises, then ramps back down before stopping.
"""
import sys
from time import sleep
from gpiozero import OutputDevice

PUL = OutputDevice(17)   # HIGH = one step
DIR = OutputDevice(27)   # rotation direction


def move(pulses, direction=1, pps=1000, ramp=250):
    DIR.value = 1 if direction else 0
    sleep(0.002)                          # let DIR settle

    cruise = 1.0 / pps                    # target period between pulses
    start = 1.0 / max(pps * 0.2, 150)     # begin ~5x slower than cruise
    ramp = min(ramp, pulses // 2)         # can't ramp longer than half the move

    for i in range(pulses):
        if ramp and i < ramp:             # speeding up
            period = start + (cruise - start) * (i / ramp)
        elif ramp and i >= pulses - ramp: # slowing down
            j = pulses - 1 - i
            period = start + (cruise - start) * (j / ramp)
        else:                             # cruise
            period = cruise
        half = period / 2
        PUL.on()
        sleep(half)
        PUL.off()
        sleep(half)


n = int(sys.argv[1]) if len(sys.argv) > 1 else 800
d = int(sys.argv[2]) if len(sys.argv) > 2 else 1
pps = int(sys.argv[3]) if len(sys.argv) > 3 else 1000
print(f"Sending {n} pulses, direction={d}, cruise={pps} pulses/sec ...")
try:
    move(n, d, pps)
    print("done")
finally:
    PUL.off()
