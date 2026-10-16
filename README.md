# Money Sorter

Vision-guided 3-axis stepper robot arm on a Raspberry Pi 4, working toward a
coin/note sorter: a camera picks a target -> inverse kinematics -> the arm moves.

The arm runs as a single daemon (`armd`) that owns the GPIO and serves a
touch-friendly dashboard + JSON control API on port **8080**.

## Quick links

- **Dashboard:** `http://money-sorter.local:8080` (from any device on the LAN)
- **Architecture:** [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) - layers, modules, mechanisms
- **Wiring & calibration:** [`docs/PINOUT.md`](docs/PINOUT.md)
- **Picking coins:** [`docs/PICKING.md`](docs/PICKING.md) - `tools/center_coin.py`: detect, lock on, centre, descend, drop, calibration, settings, recovery
- **Gripper research:** [`docs/GRIPPER.md`](docs/GRIPPER.md) - suction cup options for coins
- **Commanding the arm:** [`docs/USAGE.md`](docs/USAGE.md) - `arm_test.py` + curl API
- **Project context / conventions:** [`CLAUDE.md`](CLAUDE.md)

## Hardware

Raspberry Pi 4B -> 3x 2DM442 open-loop stepper drivers, 24 V PSU. Common-cathode
direct-3.3 V wiring (no level shifter). Shared enable on GPIO26 (active-low).

| Axis | Joint | STEP / DIR | Home switch | Range |
|------|-------|-----------|-------------|-------|
| **x** | elbow | GPIO5 / GPIO6 | GPIO7 | 0 to -90 deg |
| **y** | shoulder | GPIO17 / GPIO27 | GPIO8 | 0 to 80 deg |
| **z** | base | GPIO23 / GPIO24 | none | continuous |

Measured arm geometry: upper arm 220 mm, forearm 310 mm, shoulder pivot 120 mm
above the base plane. Full details in `docs/PINOUT.md`.

## Layout

```
moneysort/            layered package (interface -> app -> hardware -> domain)
  domain/             pure logic: planning, kinematics, config (unit-tested)
  hardware/stepper.py lgpio STEP/DIR driver
  app/                Arm + ArmController (motion policy, e-stop)
  interface/          server (daemon+HTTP), dashboard, cli
armd.py, arm_test.py  thin root shims into the package
tests/                pure-domain unit tests (no hardware)
deploy/               deploy.sh, setup.sh, systemd unit, kiosk autostart
```

## Run it

The daemon runs on the Pi as a systemd service (`moneysort-arm`), on the Pi's
**system** Python (which provides `lgpio`). The dashboard auto-starts fullscreen
via `kiosk.sh`.

Command the arm from a terminal (see `docs/USAGE.md` for the full API):

```sh
python3 arm_test.py z 40000            # base +40000 steps
python3 arm_test.py home y             # home the shoulder
python3 arm_test.py x:-11000 y:5000    # two axes together
# from any PC:
curl -s -X POST http://money-sorter.local:8080/return_zero
```

## Tests

Pure-domain tests, no hardware needed:

```sh
python3 -m unittest discover tests
```

## Deploy

The repo is the single source of truth - **never edit or run code directly on
the Pi.** Author locally -> commit/push -> on the Pi:

```sh
bash ~/projects/moneysort/deploy/deploy.sh   # git pull + venv + pip + restart
```

## Pi access (LAN only)

The Pi is reachable on the local network only (not internet-exposed).

- **Host:** `money-sorter.local` (mDNS) or `money-sorter.lan`
- **User:** `money-sorter`
- **Password:** `moneysort`

```sh
ssh money-sorter@money-sorter.local      # password: moneysort
```

For convenience, contributors can set up key auth so deploys/diagnostics run
without the password:

```sh
ssh-copy-id money-sorter@money-sorter.local
```

## Status

Motion, all-axis calibration + homing, emergency-disable-aborts-motion, and the
dashboard are done. Kinematics (FK/IK) is implemented and unit-tested, with the
elbow verified as an absolute joint on the real hardware. Next: wire a
"move to (x, y, z)" path, then Intel RealSense camera integration. See
`CLAUDE.md` for the live status.
