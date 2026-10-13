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
  - `kinematics.py` — FK/IK for a base-yaw + 2-link arm with a **levelling
    wrist** (forearm tilts, tool stays level → constant reach). Empirically fit
    to a 3×3 on-arm tip grid (rms ~1 cm); `forward()`/`inverse()` + `OutOfReach`.
  - `calibration.py` — camera→base rigid transform (`CameraCalib.to_base`,
    `at_survey`), loaded from the git-ignored `camera_calib.json`; armd's
    `/detect` adds `base_mm` per coin and `at_survey` (valid only at the survey pose).
  - `jog.py` — continuous-jog maths: `slew` (bounded accel), `take_steps` (fractional
    carry), `cart_to_joint_rates` (numeric Jacobian of the FK).
  - `config.py` (package root) — single source of truth: `PORT`, `ENABLE_PIN`,
    `GPIOCHIP`, `JOINTS` (pins + calibration).
- **`hardware/stepper.py`** (`Stepper`) — per-axis STEP/DIR via `lgpio.tx_pulse`
  (hardware-timed), consumes `domain.planning`.
- **`hardware/camera.py`** (`Camera`) — Intel RealSense D435 owned by the daemon
  (one process holds the device, like GPIO). Background thread keeps the latest
  aligned depth+colour frame; `jpeg()` feeds the MJPEG stream, `detect()` runs
  `vision`. USB3 profile first, USB2-safe low-fps fallback. Imports are guarded:
  without `pyrealsense2`/OpenCV or a device it comes up `ok=False` and armd runs
  arm-only.
- **`vision.py`** — OpenCV Hough coin detection, hardware-free (caller supplies
  `depth_at`/`deproject`); returns each coin's pixel + camera-frame 3D point (mm).
- **`app/`** — orchestration/policy, no HTTP:
  - `arm.py` (`Arm`) — `move`, `move_many` (coordinated), homing (`find_home`,
    `home_all`), `return_zero`, `zero`, shared `enable`/`disable` with an `abort`
    Event, homed-aware soft limits.
  - `ArmController.jog` — hold-to-move with a dead-man (`POST /jog`, ~0.25 s TTL): a
    jog thread feeds 40 ms step slices (`Stepper.queue_slice`) so motion is
    continuous and stops by itself if heartbeats stop. Joint (deg/s) or cartesian (mm/s).
  - `controller.py` (`ArmController`) — serializes motion (one move at a time),
    latched e-stop, assembles `status()`.
- **`interface/`** — delivery:
  - `server.py` — long-running daemon (systemd `moneysort-arm`) that OWNS all
    GPIO **and the camera** for its lifetime (so the e-stop is latched). Serves
    dashboard + JSON API on **:8080** (binds `0.0.0.0`, reachable from any LAN PC),
    incl. the camera feed (`/camera`, `/camera?detect=1`), `/detected`, `POST /detect`.
  - `dashboard.py` — the HTML `PAGE` + stdlib system-metric helpers (no GPIO).
    The page is in Italian; every on-screen string goes through `t()` with its Italian in the `IT`
    table (add `?lang=en` to the address for English). Add both when adding UI text.
  - `cli.py` — CLI HTTP client (see `docs/USAGE.md`).
- **`tests/`** — `python3 -m unittest discover tests` (pure domain; no hardware).
- **`tools/`** — run on the Pi, all thin HTTP clients of armd (never touch GPIO or
  the camera): `jog.py` (keyboard teleop, joint/cartesian), `coin_detect.py`,
  `calibrate_camera.py` (camera→base rigid transform at a survey pose →
  `camera_calib.json`), `realsense_test.py`.
- **`deploy/`** — `deploy.sh`, `setup.sh`, `install_realsense.sh` (pyrealsense2 on
  the Pi: pip wheel or long librealsense source build), systemd unit, kiosk
  autostart (`kiosk.sh`, `cog` fullscreen on labwc).

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
| **y** | shoulder | 132,000 | 0 … 80° (0 … 29,333 steps; `SHOULDER_MAX_DEG`) | switch GPIO8, home_dir −1 |
| **z** | base | 157,005 | continuous | no switch |

The shoulder is limited to 80° (it stalls fully extended; the mechanical stroke is 90°).
Comfortable cruise = **20000 pps**; return-to-zero runs at `RETURN_ZERO_PPS` (6000). Soft limits (x/y) apply only once homed; an
e-stop un-homes every axis (positions unknown after an abrupt stop) → re-home
before relying on limits again. Steps↔degrees table in `docs/USAGE.md`.

## Status & next steps

Done: motion, all-axis calibration + homing (debounced fine approach so an axis
resting on the switch edge seats solidly), emergency-disable-aborts-motion,
dashboard (return-to-zero, Home in Settings, toasts). Note: the homing seek
*timeout* was deliberately removed — no runaway guard, relies on wired switches.

**Kinematics done + `move_to` wired** (`domain/kinematics.py`, `Arm.move_to` →
`inverse` → `move_many`, `POST /move_to`, `arm_test.py moveto X Y Z`). The arm is
a base-yaw + 2-link arm with a **levelling wrist**: the forearm tilts with the
elbow but a passive pivot keeps the tool level, so the tool adds a *constant*
horizontal reach. Model (mm, Z from base plate):
`r = L1·sin(y) + LF·cos(x) + LT`, `h = L1·cos(y) + LF·sin(x) + D`
(L1 161, LF 210, LT 132, D 168), empirically fit to a 3×3 on-arm tip grid,
rms ~1 cm / max ~2 cm. `move_to` requires x & y homed. The base (z) is hard-limited
to ±180° from zero.

**Camera done:** RealSense D435 (eye-in-hand) lives in armd with a live MJPEG feed
on the dashboard and coin detection (`/detect`); `tools/calibrate_camera.py`
produces the camera→base transform (valid for its survey pose; re-run if the mount
moves). `pyrealsense2` on the Pi was the risky install (`deploy/install_realsense.sh`).

**Next:** re-verify a couple of `moveto` targets on the arm, then the
visual-servoing controller (deferred) and the actual sorting logic (pick a
detected coin → base frame → `move_to`).

## Docs

- `docs/ARCHITECTURE.md` — the layered design, every module, runtime model, and
  the key mechanisms (motion planning, e-stop, homing, soft limits).
- `docs/PINOUT.md` — full wiring, calibration, home switches.
- `docs/GRIPPER.md` — suction-cup research for picking coins (pump + release valve; not built yet).
- `docs/USAGE.md` — commanding the arm from a terminal (`arm_test.py` + curl API).
