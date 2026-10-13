# Gripper research: suction cup for coins

Notes from a web search (2026-10-14) on how to pick up coins. Nothing here is
built yet; this records the options and the decision so far.

## Decision (so far)

A **small DC diaphragm vacuum pump + a solenoid release valve**, both switched by
logic-level MOSFETs from two Pi GPIOs. Not a venturi ejector: it needs
compressed air (a compressor and regulator), which is out of scale for a desk arm.

## How much suction a coin needs (our estimate, not from a source)

A coin weighs 2-9 g. A 5 mm cup at -30 kPa holds about 60 g (F = P x A), so the
margin is 5x or more. Coins are flat and seal well: the cup (small, soft lip or
bellows) matters more than pump flow.

Reference: the uArm Swift's built-in pump uses 5-10 mm cups, reaches 33 kPa and
lifts 1 kg ([datasheet](https://www.mouser.com/datasheet/2/813/uArm-Swift-Specifications-en-1223866.pdf)).

## Venturi ejectors (rejected)

- [Anver JV miniature venturi](https://anver.com/venturi%20vacuum%20pumps.htm): 16 g, no moving parts, up to 26" Hg,
  millisecond attach and release. Needs compressed air; air use is the main cost.
- [Riegler SCP 101-M mini-pump](https://riegler.de/de/en/mini-compact-ejector-mini-pump-nozzle-size-1-mm-nc-108397-1.html):
  built-in vacuum switch, 85% evacuation, about 639 EUR ex VAT. Out of proportion.

## Pumps found

- [SparkFun 12 V diaphragm pump (ROB-10398)](https://sparkfun.com/products/10398): 1000 h rated life. Works, but bigger than needed.
- [Evelta 12 V brushless piston pump](https://evelta.com/12v-mini-brushless-piston-vacuum-pump-for-suction-systems-max-flow-26l-min/)
  (18 or 26 L/min): made for pick-and-place, oversized for coins.
- Better: a micro diaphragm pump of the kind built into small arms. No specific
  buyable model found yet (open item).

## Release valve is required

Switching the pump off does not release the grip
([SparkFun community thread](https://community.sparkfun.com/t/an-open-source-robot-arm-project/27897/13)).
Put a solenoid valve on the line that vents the cup to atmosphere. So two outputs:
pump and valve. A normally-open valve vents on power loss (a coin drops rather
than stays held on a dead arm).

## Driving it from the Pi

- GPIO is 3.3 V only: switch the pump and valve through **logic-level MOSFETs**
  (gate threshold below ~2.5 V) with a flyback diode across each load, GND common
  with the Pi. Avoid IRF520 modules: they need ~10 V on the gate.
- Choosing a **24 V** pump and valve lets them share the stepper PSU (no converter).
  A 12 V part needs a 24 -> 12 V buck.

## Open items

- Pick a concrete pump + valve + cup (5-8 mm, soft lip or bellows) available in Italy.
- Choose two free GPIO pins (check `PINOUT.md`).
- Software: pins in `config.py`, a `Gripper` in `hardware/`, `POST /grip` with a
  dead-man like `/jog` (never leave the pump running on a lost connection).
