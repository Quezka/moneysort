# Money Sorter — project context

Vision-guided 3-axis stepper robot arm on a Raspberry Pi 4, working toward a
money sorter (camera picks a target → inverse kinematics → move). This file is
the portable source of truth for a fresh session; deeper detail is in `docs/`.

## Workflow rule (important)

**Never create, edit, or run code directly on the Pi** (`money-sorter@money-sorter.local`).
The repo is the single source of truth:

- Author/Edit files **only** in this local repo → commit/push.
- Deploy = `bash ~/projects/moneysort/deploy/deploy.sh` on the Pi (git pull →
  venv → pip → `sudo systemctl restart moneysort-arm`). This is the one command
  allowed to run over SSH.
- Anything that would otherwise be created on the Pi (systemd units, autostart,
  config) lives in the repo as files + a documented install script the **user**
  runs — not `sudo tee`'d onto the device.
- SSH is otherwise limited to **read-only diagnostics** (`pinctrl get`, `curl`,
  measurements) and **only with the user's explicit go-ahead**. The user runs
  scripts; the assistant does not.

## Architecture

Layered `moneysort/` package; **dependencies point inward** (interface → app →
hardware → domain; `config` is the shared innermost). `armd.py` and `arm_test.py`
at the repo root are thin shims into the package (kept so the systemd unit and
the documented CLI paths don't change).

- **`domain/`** — pure logic, no hardware or HTTP (unit-testable off-Pi):
  - `planning.py` — trapezoidal motion profile (`plan_segments`, `ramp_segs`);
    cruise chunked into ~40ms bursts so an e-stop drains fast (no re-enable lurch).
  - `kinematics.py` — FK/IK (**stub**, to fill next; needs measured link lengths).
  - `config.py` (package root) — single source of truth: `PORT`, `ENABLE_PIN`,
    `GPIOCHIP`, `JOINTS` (pins + calibration).
- **`hardware/stepper.py`** (`Stepper`) — per-axis STEP/DIR via `lgpio.tx_pulse`
  (hardware-timed), consumes `domain.planning`.
- **`app/`** — orchestration/policy, no HTTP:
  - `arm.py` (`Arm`) — `move`, `move_many` (coordinated), homing (`find_home`,
    `home_all`), `return_zero`, `zero`, shared `enable`/`disable` with an `abort`
    Event, homed-aware soft limits.
  - `controller.py` (`ArmController`) — serializes motion (one move at a time),
    latched e-stop, assembles `status()`.
- **`interface/`** — delivery:
  - `server.py` — long-running daemon (systemd `moneysort-arm`) that OWNS all
    GPIO for its lifetime (so the e-stop is latched). Serves dashboard + JSON API
    on **:8080** (binds `0.0.0.0`, reachable from any LAN PC).
  - `dashboard.py` — the HTML `PAGE` + stdlib system-metric helpers (no GPIO).
  - `cli.py` — CLI HTTP client (see `docs/USAGE.md`).
- **`tests/`** — `python3 -m unittest discover tests` (pure domain; no hardware).
- **`deploy/`** — `deploy.sh`, `setup.sh`, systemd unit, kiosk autostart.

## Hardware (summary; full wiring in `docs/PINOUT.md`)

Pi 4B → 3× 2DM442 open-loop stepper drivers, 24V PSU. Common-cathode direct-3.3V
wiring (no level shifter). Shared enable on **GPIO26** (active-low: LOW=enabled).
Axes: **x**=elbow (STEP5/DIR6), **y**=shoulder (STEP17/DIR27), **z**=base
(STEP23/DIR24). NC home switches (internal pull-up): x=GPIO7, y=GPIO8; z has none
(continuous). not-home=LOW, at-home/broken=HIGH (fail-safe).

## Calibration & limits (all axes done)

| Axis | Joint | Steps/rev | Range | Home |
|------|-------|-----------|-------|------|
| **x** | elbow | 132,000 | 0 … −90° (0 … −33,000 steps) | switch GPIO7, home_dir +1 |
| **y** | shoulder | 132,000 | 0 … 90° (0 … 33,000 steps) | switch GPIO8, home_dir −1 |
| **z** | base | 157,005 | continuous | no switch |

Comfortable cruise = **20000 pps**. Soft limits (x/y) apply only once homed; an
e-stop un-homes every axis (positions unknown after an abrupt stop) → re-home
before relying on limits again. Steps↔degrees table in `docs/USAGE.md`.

## Status & next steps

Done: motion, all-axis calibration + homing (debounced fine approach so an axis
resting on the switch edge seats solidly), emergency-disable-aborts-motion,
dashboard (return-to-zero, Home in Settings, toasts). Note: the homing seek
*timeout* was deliberately removed — no runaway guard, relies on wired switches.

Next: **inverse kinematics** (camera xy → joint angles), then **camera
integration**. A setpoint-tracking controller for visual servoing is the eventual
model but deferred.

## Docs

- `docs/PINOUT.md` — full wiring, calibration, home switches.
- `docs/USAGE.md` — commanding the arm from a terminal (`arm_test.py` + curl API).
