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
| **z** | base | 360° ≈ **150,000 steps** | continuous rotation, no physical limit |
| **y** | shoulder | ≈ **33,000 steps** end-to-end | **home switch at start** (homing TODO) |
| **x** | elbow | not calibrated yet | — |

- `z` has `steps_per_rev = 150000`, so `move_degrees()` is accurate for the base.
- `y`/`x` still command in raw **steps** until their per-degree is measured.
- `y` soft limits + homing against the start switch are **not implemented yet**;
  position resets to 0 at daemon start, so there's no overtravel protection until
  homing lands.
- A comfortable cruise speed for all three axes is **20000 pps** (the default).
