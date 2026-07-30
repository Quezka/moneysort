# Pinout

Raspberry Pi 4B → 2DM442 stepper drivers.

## Wiring scheme

- **Common-cathode, direct 3.3V** (no level shifter). Each driver signal's `+`
  terminal is driven by a Pi GPIO; all `-` terminals bus together to Pi **GND**.
- GPIO **HIGH = active** (pulse / enable asserted).
- 24V motor supply is separate and stays isolated via the drivers' optocouplers.
- **Enable is a single shared line**: every driver's `ENA+` ties to **GPIO26**
  (pin 37). One pin enables/disables all axes at once. Driver default (pin LOW,
  ENA opto off) = enabled/holding; driving it HIGH disables all.
  Verified: driving HIGH releases all motors even sourcing ~3 optos from one
  3.3V pin, so no buffer/transistor is needed.

BCM numbers are the GPIO ids used in code; the pin column is the physical
40-pin header position.

## Y axis (shoulder)

| Signal | Driver terminal | BCM | Header pin |
|--------|-----------------|-----|-----------|
| Enable | ENA+            | GPIO26 | 37 | (shared)
| Pulse  | PUL+            | GPIO17 | 11 |
| Dir    | DIR+            | GPIO27 | 13 |

Shared: `ENA-` / `PUL-` / `DIR-` → GND bus (e.g. header pin 6).

## X axis (elbow)

| Signal | Driver terminal | BCM | Header pin |
|--------|-----------------|-----|-----------|
| Enable | ENA+            | GPIO26 | 37 | (shared)
| Pulse  | PUL+            | GPIO5  | 29 |
| Dir    | DIR+            | GPIO6  | 31 |

## Z axis (base)

| Signal | Driver terminal | BCM | Header pin |
|--------|-----------------|-----|-----------|
| Enable | ENA+            | GPIO26 | 37 | (shared)
| Pulse  | PUL+            | GPIO23 | 16 |
| Dir    | DIR+            | GPIO24 | 18 |

## Calibration & travel

Measured at the output shaft (folds in microstepping + gearing), at 20000 pps
cruise. These live in `arm.py` `JOINTS`.

| Axis | Joint | Full range | Notes |
|------|-------|-----------|-------|
| **z** | base | 360° = **157,005 steps** (measured, full rev) → 90° ≈ 39,251 | continuous rotation, no physical limit |
| **y** | shoulder | **33,000 steps = 90°** → 360° = 132,000 steps/rev | homed; soft-limited to **0..33,000 steps (0..90°)** |
| **x** | elbow | per-degree not measured yet | homed; soft-limited to **−33,000..0 steps** (home at 0, positive end) |

- `z` has `steps_per_rev = 157005`, so `move_degrees()` is accurate for the base.
- `y` has `steps_per_rev = 132000` (33k steps = 90°); `move_degrees("y", ...)` works.
- `y` **soft limits are enforced once homed**: after `find_home`, home = 0 and
  moves are clamped to 0..33,000 steps (0..90°), so it can't overtravel either
  end. Limits are inactive until the axis is homed (no position reference before).
- `x` still commands in raw **steps** until its per-degree is measured.
- A comfortable cruise speed for all three axes is **20000 pps** (the default).

## Home / limit switches

**y and x are both wired and homing works for both.**

Normally-closed switches wired **between the GPIO and GND**, using the Pi's
internal pull-up. NC is fail-safe: a disconnected/broken switch reads as
triggered and stops motion.

| Axis | Joint | GPIO | Header pin | Toward home | Reading |
|------|-------|------|-----------|-------------|---------|
| **y** | shoulder | GPIO8 | 24 | **negative** steps | not-home = LOW, at-home = HIGH | wired ✓ |
| **x** | elbow | GPIO7 | 26 | **positive** steps *(verified)* | not-home = LOW, at-home = HIGH | wired ✓ |

- `z` (base) has no switch — continuous rotation.
- **Homing** (`find_home`): coarse jog toward the switch until HIGH, back off
  until released, slow fine approach, then set position 0. Aborts if it travels
  past ~1.3× the axis range (guards a wrong `home_dir` or a stuck/broken switch).
  Trigger via the dashboard **⌂ Home Y** button or `arm_test.py home y`.
- **x homes toward +steps** (verified 2026-07-30); its zero sits right at the
  switch edge, so `at_home("x")` can flicker LOW at rest — benign. A larger
  `backoff`/`fine` in `find_home` would seat it more firmly if it matters.
