# Vacuum pickup: the pump, the valve and the spring-loaded nozzle

Engineering of the SMD-pick-and-place-style pickup for coins: one small diaphragm pump, one release valve, a
vacuum sensor and a **spring-loaded nozzle** at the tip. It turns the decisions in [`GRIPPER.md`](GRIPPER.md)
into a build. Nothing here is fitted yet. Numbers marked *estimate* are calculated, not measured; the list at the
end says what to measure first.

Files: [`cad/pickup_nozzle.py`](../cad/pickup_nozzle.py) (printable nozzle, replaces `nozzle_holder.stl`),
[`moneysort/hardware/gripper.py`](../moneysort/hardware/gripper.py) (pump + valve driver, tested against a fake lgpio),
pins in [`config.py`](../moneysort/config.py) and [`PINOUT.md`](PINOUT.md).

## 1. What it has to do

| Requirement | Why |
|-------------|-----|
| Hold the heaviest euro coin (2 EUR, 8.5 g) through a lift and a move, with margin | the job |
| Seal on a flat field of coins from 16 to 26 mm, with a raised rim and relief | euro coins are not smooth |
| Forgive a few mm of height error | the kinematic fit is good to ~1 cm rms |
| Add almost no weight at the tip | the shoulder already stalls fully extended; the camera adds 110 g |
| Drop the coin when anything fails (power, Pi, e-stop) | a coin held on a dead arm is the worse failure |
| Say whether the pick worked | a sorter must not carry empty air to the bin |

## 2. Force budget (*estimate*)

Holding force is `F = dP x A` over the area the cup really seals. A 6 mm bellows cup seals on its lip: take an
effective diameter of 4 mm (A = 12.6 mm2), which is conservative.

| dP | Force on 4 mm | Grams-force | x the 2 EUR coin's weight |
|----|---------------|-------------|---------------------------|
| -30 kPa | 0.38 N | 38 g | 4.5 |
| -40 kPa | 0.50 N | 51 g | 6.0 |
| -60 kPa | 0.75 N | 77 g | 9.0 |

Sideways the cup holds by friction: with mu = 0.3 and -40 kPa that is 0.15 N, which carries an 8.5 g coin at
1.8 g of lateral acceleration. Keep a 2x margin: **the arm should not accelerate a carried coin above ~0.9 g**, which
the trapezoidal planner stays far below. Target: **at least -40 kPa at the cup with a coin on it.**

## 3. Pneumatic circuit

```
   cup ---- brass tube (4 mm, slides in the nozzle) ---- silicone hose 3 x 5 mm ----+
 (6 mm bellows)                                                                   |
                                                                          T ------+---- vacuum sensor (I2C, gauge)
                                                                          |
                                                                 COM  [ 3/2 valve ]  NO --> open air (vent)
                                                                          |  (energised: COM <-> NC)
                                                                         NC
                                                                          |
                                                          [ diaphragm pump, inlet ]   exhaust --> open air
```

- **Valve de-energised = cup open to the air** (the normally-open vent). Energised = cup connected to the pump.
  Power loss, a crashed Pi or a pulled plug all release the coin. This costs ~2 W of valve coil while holding, so
  the valve must be rated for continuous duty (most mini 3/2 valves are; check).
- The sensor sits on the cup side of the valve, so it sees the cup pressure, vented or not.
- Hose from the base to the tip: 3 mm bore, ~0.8 m = **~6 mL** of volume. Laminar flow loss at 1 L/min is ~120 Pa
  (Hagen-Poiseuille), nothing. Pump-down to -40 kPa in **~0.4 s** with a 1 L/min pump (*estimate*); venting takes
  tens of ms.
- Pick-up time, not hold, is the cycle's bottleneck: the pump must not be sized for flow, only for the leak
  of a coin seal (~0.1-0.3 L/min, *estimate*) and for the final vacuum.

### Where the parts go

**The pump, valve, sensor and driver board live at the base, not on the arm.** Only the nozzle (about 12 g
printed + 3 g of tube, spring and collar) and the hose reach the tip. A 12 V R385-class pump weighs 60-80 g
and its vibration would shake the camera. Leave a loop of hose at the shoulder and the elbow so neither axis pulls
on it, and route it along the arm with the existing cable-tie habit. The base turns +-180 deg: either mount the
pod on the rotating part, or leave a hose loop that unwinds through the column.

## 4. Parts (by specification; check the datasheet or measure before trusting a listing)

| Part | Specification | Note |
|------|---------------|------|
| Pump | 12 V brushed diaphragm, **at least -50 kPa** (better -60), 1-3 L/min free flow, < 0.6 A | the 24 EUR HamPaesk in `GRIPPER.md` does not state its vacuum: ask or skip it. Search "R385 12V vacuum pump -60kPa" |
| Valve | 12 V mini 3/2 solenoid, **normally open to vent**, 3-4 mm barbs, continuous duty | 12 V so it shares the pump's buck |
| Supply | 24 V -> 12 V buck, 2 A or more, fixed output | the stepper PSU stays the only source; a 6 V kit pump would need a second buck |
| Cup | 6 mm bellows, silicone or nitrile, neck that pushes onto a 4 mm tube | bellows give 2-3 mm of extra compliance; an M5-threaded cup needs an M5 nipple on the tube instead |
| Tube | brass tube **4.0 mm OD x 0.45 wall**, cut to the length in section 6 | hobby shop |
| Hose | silicone 3 x 5 mm (ID x OD), about 1 m | stretches over 4 mm; do not use PVC (it stiffens and kinks) |
| Spring | compression, OD 7, free length 18, solid height <= 6, rate 0.1-0.2 N/mm | soft; press it by hand: it should take 1 N to move 8 mm |
| Sensor | I2C gauge vacuum sensor, 0 to -100 kPa, 3.3 V, barbed port (e.g. the XGZP6847D family: confirm range, address and 3.3 V on the datasheet) | not an absolute-pressure breakout: those are open to the air and not sealed |
| Driver | 2x logic-level N-MOSFET, 2x 1N5819 (or 1N4007), 2x 100 ohm, 2x 10 k, 100 nF, 100 uF, small screw-terminal board | schematic below |
| Printed | `pickup_housing.stl`, `pickup_collar.stl` | PETG |
| Hardware | 2x M3 x 12 bolt + 2 nuts (same as the old holder), 1x M3 x 6 grub screw + nut (collar) | |

## 5. Electronics

```
 12 V (buck) ---------+------------------+-------------+
                      |                  |             |
                  [ pump ]  <--|<--      [ valve ] <--|<--     (flyback diode across each load,
                      |        1N5819        |        1N5819    stripe/cathode to +12 V)
                      +---+                  +---+
                          | D                    | D
 GPIO16 --[100R]--+-------| G  MOSFET       +----| G  MOSFET
 (pin 36)         |       | S                    | S
                [10k]     |                  [10k]
                  |       |                    |
 Pi GND ----------+-------+--------------------+---- 12 V buck GND (common)
 GPIO20 --[100R]--(to the valve gate in the same way, pin 38)

 pump terminals: 100 nF ceramic across the motor + 100 uF near the board (brush noise would reset the Pi otherwise)
```

- The **10 k pull-downs** keep both gates low while the Pi boots, so nothing runs before armd owns the pins and the
  valve vents.
- Logic level: check that RDS(on) is specified at Vgs = 2.5-3.3 V (an IRL-series part or an AO3400-class SMD;
  **not** an IRF520, which needs ~10 V). At 0.5 A almost any such part is cool.
- Common ground with the Pi, as with the stepper drivers. The sensor goes on I2C (GPIO2/3, pins 3/5), 3.3 V.
- **On/off only, no PWM on these pins.** lgpio's PWM shares the pulse thread the steppers use, and a PWM record
  there reopens the "TX queue stuck" wedge (`hardware/stepper.TxAnchor`).

| Signal | BCM | Header pin |
|--------|-----|------------|
| Pump MOSFET gate | GPIO16 | 36 |
| Valve MOSFET gate | GPIO20 | 38 |
| Vacuum sensor SDA/SCL | GPIO2 / GPIO3 | 3 / 5 |

## 6. The spring-loaded nozzle

```
 plate / block
 ------+--------- hose slips over the tube end, ABOVE the plate (TOP_REST)
       |  tube (rest)                     s = 0   underside of the block
   [flange]----------+
       | bearing 7   |                   the tube slides in two 7 mm printed bearings,
       |------- spring (soft)  ---+      pushed down by the spring against the collar
       | collar (M3 grub)  <-- TRAVEL 9 mm: the tube goes UP when the cup touches
       |------- rest stop
       | bearing 7   |
   [housing]---------+
       | tube protrudes 10 mm, the cup sits on it
      cup
```

Run `python cad/pickup_nozzle.py` for the numbers; its asserts check that the spring never goes solid and keeps
a preload. With the defaults: housing 35 mm below the flange, **the cup end 45 mm below the block, the same as the
old holder**, so the fitted kinematics and the camera-mount calculations (nozzle out of the depth view) do not
change. The tube is cut about 55 mm long; the collar is clamped about 34 mm from its top end.

- Contact force: spring preload ~0.3 N at rest, ~1.4 N at full travel (*estimate* with a 0.12 N/mm spring):
  enough to seat the cup, soft enough not to push the coin around.
- **Assembly:** ream both bearings with a 4 mm drill; drop the spring in through the side window, thread the tube
  through, fit the collar (nut in its pocket) with the tube at rest, tighten the grub. Slide the tube by hand: it
  must move freely and snap back. Fit the cup last.
- Put a drop of silicone grease on the tube. If it binds, the bearings are too tight (print 0.1 mm bigger).
- The nozzle stays out of the camera's view: the housing is higher than the old tube end, so its angle from the lens
  axis is larger (>29 deg).

## 7. The pick sequence

1. Descend to the learned touch height **plus the travel**: the arm may overshoot by up to 9 mm and the spring
   absorbs it. The existing `--calibra` learns the touch height with a bare tip; after fitting the nozzle, run it
   again so the 'touch' is where the tube starts to retract.
2. `grip()`: valve closes the vent, pump runs.
3. With the sensor: wait until the cup reads -25 kPa or lower (or 300 ms without it), then rise. A reading that
   stays near 0 means no coin: try again.
4. Carry. During the carry the reading should stay below -25 kPa; a rise means the coin slipped.
5. At the drop spot `release()`: the pump stops first, then the valve opens to the air.

## 8. What is built and what is next

Built and tested (79 unit tests pass): the `Gripper` class (`grip(ttl)`, `keepalive()`, `release()`, dead-man timer),
the pin constants, the CAD model.

Not done (and not touched, on purpose: it changes a real arm and the dashboard has your uncommitted work):

1. `POST /grip` and `/release` in `interface/server.py` with the dead-man refresh like `/jog`; show `gripping` in `/status`.
2. **E-stop must call `Gripper.release()`** (the coin drops; decide if you want that or a held coin).
3. `pick()` / `release()` in `tools/center_coin.py` calling those endpoints; then `PUMP_FITTED = True`.
4. The vacuum-sensor reader (`hardware/vacuum.py`) and the pick check of section 7.
5. A Gripper switch in the dashboard (Italian + English strings).

## 9. Measure first (30 minutes with the parts on a table)

1. The pump's real vacuum: pump, valve and sensor on a tee with the end plugged. If it is under -50 kPa, change the pump.
2. A coin on the cup: the vacuum with a 2 EUR coin and with a 1 cent coin. Both must reach -40 kPa.
3. Weigh a coin with the cup on a kitchen scale pulling up: the number that replaces the estimate in section 2.
4. The time from `grip()` to -40 kPa and from `release()` to 0.
5. A cheap-cup check: some uxcell cups leak at the neck: push on, plug the tip, and watch the sensor.
