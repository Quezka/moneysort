#!/usr/bin/env python3
"""Fast motor test using the Stepper class (lgpio, hardware-timed pulses).

Wiring: GPIO17 -> PUL+, GPIO27 -> DIR+, '-' bus -> GND (direct 3.3V).

Usage (run on the Pi):
    python3 motor_test.py [steps] [pps]
        steps : signed step count (+ / - sets direction), default 3200
        pps   : cruise pulses per second, default 6000
"""
import sys
import time
import lgpio
from stepper import Stepper

STEP_PIN, DIR_PIN = 17, 27

steps = int(sys.argv[1]) if len(sys.argv) > 1 else 3200
pps = int(sys.argv[2]) if len(sys.argv) > 2 else 6000

h = lgpio.gpiochip_open(0)
m = Stepper(h, STEP_PIN, DIR_PIN, max_pps=pps, accel_steps=600)
print(f"tx queue capacity: {lgpio.tx_room(h, STEP_PIN, 0)} entries")
print(f"Moving {steps} steps, cruise {pps} pps ...")
t0 = time.time()
try:
    m.move(steps)
    print(f"done in {time.time() - t0:.2f}s")
finally:
    m.close()
    lgpio.gpiochip_close(h)
