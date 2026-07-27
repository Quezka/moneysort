# Pinout

Raspberry Pi 4B → 2DM442 stepper drivers.

## Wiring scheme

- **Common-cathode, direct 3.3V** (no level shifter). Each driver signal's `+`
  terminal is driven by a Pi GPIO; all `-` terminals bus together to Pi **GND**.
- GPIO **HIGH = active** (pulse / enable asserted).
- 24V motor supply is separate and stays isolated via the drivers' optocouplers.
- **Enable is a single shared line**: every driver's `ENA+` ties to **GPIO22**
  (pin 15). One pin enables/disables all axes at once. Driver default (pin LOW,
  ENA opto off) = enabled/holding; driving it HIGH disables all.
  Caveat: HIGH must source ~3 optos in parallel from one 3.3V pin, which is
  current-marginal — verify it actually releases before relying on disable.

BCM numbers are the GPIO ids used in code; the pin column is the physical
40-pin header position.

## Y axis

| Signal | Driver terminal | BCM | Header pin |
|--------|-----------------|-----|-----------|
| Enable | ENA+            | GPIO22 | 15 |
| Pulse  | PUL+            | GPIO17 | 11 |
| Dir    | DIR+            | GPIO27 | 13 |

Shared: `ENA-` / `PUL-` / `DIR-` → GND bus (e.g. header pin 6).

## X axis

| Signal | Driver terminal | BCM | Header pin |
|--------|-----------------|-----|-----------|
| Enable | ENA+            | GPIO22 | 15 | (shared)
| Pulse  | PUL+            | GPIO23 | 16 |
| Dir    | DIR+            | GPIO24 | 18 |

## Z axis

| Signal | Driver terminal | BCM | Header pin |
|--------|-----------------|-----|-----------|
| Enable | ENA+            | GPIO22 | 15 | (shared)
| Pulse  | PUL+            | GPIO5  | 29 |
| Dir    | DIR+            | GPIO6  | 31 |
