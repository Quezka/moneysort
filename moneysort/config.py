"""Central configuration: the single source of truth for pins and calibration.

Everything hardware-specific lives here so the layers above don't hard-code pin
numbers or step counts. See docs/PINOUT.md for the wiring and how the
calibration numbers were measured.
"""

# --- server ---------------------------------------------------------------
PORT = 8080          # armd HTTP server (dashboard + control API), binds 0.0.0.0
GPIOCHIP = 0         # lgpio.gpiochip_open() index

# --- shared enable line ---------------------------------------------------
# Every driver's ENA+ ties to this one pin. Active-low: pin LOW = ENA opto off
# = drivers ENABLED (motors hold); pin HIGH = disabled. Owned by the Arm (not
# the Steppers) because lgpio can't let three Steppers each claim the same pin.
ENABLE_PIN = 26      # BCM GPIO26 = header pin 37

# --- per-axis config (BCM pins + calibration) -----------------------------
#   step, dir     : BCM gpio for PUL+ / DIR+
#   invert        : flip if positive moves the "wrong" way
#   steps_per_rev : measured steps for a full 360 deg output turn (folds in
#                   microstepping + gearing) -- makes move_degrees() accurate
#   travel        : usable range in steps for a limited joint (home switch)
#   home_pin      : BCM gpio of the home/limit switch (NC to GND, internal pull-up)
#   home_dir      : sign of the step direction that moves TOWARD the switch
# Home switches are normally-closed: not-home = LOW, at-home / broken wire = HIGH
# (fail-safe -- a disconnected switch reads as triggered and stops motion).
JOINTS = {
    "x": {"step": 5,  "dir": 6,  "invert": False, "travel": 33000,
          "steps_per_rev": 132000, "home_pin": 7, "home_dir": 1},             # elbow: 0..-33000 steps = 0..-90 deg, home (0) toward +steps
    "y": {"step": 17, "dir": 27, "invert": False, "travel": 33000,
          "steps_per_rev": 132000, "home_pin": 8, "home_dir": -1},            # shoulder: 0..33000 steps = 0..90 deg, home (0) toward -steps
    "z": {"step": 23, "dir": 24, "invert": False, "steps_per_rev": 157005},   # base: measured via full rev, 360 deg = 157005 steps (90 deg = 39251)
}
