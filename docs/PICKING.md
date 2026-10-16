# Picking coins: `tools/center_coin.py`

The pick cycle. It runs on the Pi as a thin HTTP client of `armd` (it never touches GPIO or the
camera) and uses everything built so far: coin detection, kinematics, calibration and the
recovery logic. The suction pump is not fitted yet, so `pick()` and `release()` are empty hooks.

## The cycle

```
detect ──> choose the top coin of the pile ──> lock on it (magenta TARGET ring on the dashboard)
   ──> centre it by pixels (one or two moves) ──> descend in ONE smooth move, folding the last
   correction into it ──> pick() ──> rise ──> carry to the drop spot ──> release()
   ──> go back to where the coin was picked (the rest of the pile is right there) ──> next coin
```

Between two coins it asks you to continue (you take the coin away by hand while there is no pump).
With `--auto`, or `PUMP_FITTED = True` once the pump exists, it carries on by itself until it sees
no more coins.

## Running it

Prerequisites: `armd` running, camera `ok` (dashboard says `USB3` ideally), x and y **homed**, base
(z) at zero, a coin in view 20-60 cm below the camera, a hand near the e-stop.

```
python3 ~/projects/moneysort/tools/center_coin.py            # asks before each next coin
python3 ~/projects/moneysort/tools/center_coin.py --auto     # no questions, until the pile is empty
python3 ~/projects/moneysort/tools/center_coin.py --drop X Y Z   # set the drop spot and run
python3 ~/projects/moneysort/tools/center_coin.py --calibra  # calibrate (see below), then exit
```

The rest pose (all axes at zero) is the very corner of the workspace: at that height the arm cannot
move sideways at all. The script first moves the tool to a **working pose** with room to move
(about X392 Z294 mm, asks for a go-ahead), and works from there.

At the prompt between coins: `Enter` next, `m` the tip was not right (teaches the calibration),
`d` set the drop spot where the tip is now, `q` quit. After a failed attempt: `r` retry the **same**
coin, `n` give up on it and pick another, `q` quit.

## How it works

### Seeing the coins (`vision.py`, `hardware/camera.py`)

- Hough circles on **brightness and on colour saturation**, merged: a silver coin among gold ones
  has almost no brightness edge but stands out in saturation. `min_dist` 22 px (coins in a pile
  overlap), radius 18-45 px (drops big false circles on the cardboard).
- A coin's depth is the **median of a 13x13 patch**, not one pixel (shiny metal makes holes).
  Without a valid depth it is drawn **yellow** (the D435 cannot read closer than ~20 cm, and the USB2
  fallback profile is noisy) - it can still be centred, just not descended to.
- A coin that flickers out of one detection is **held** for two calls while the scene is static
  (orange ring, `held: true`).
- Which coin of a pile: the one **closest to the camera** (the top one). Depths within 6 mm count
  as level (a coin is ~2 mm thick, the sensor wobbles by a few mm); among those, the nearest to the
  target pixel wins.

### Lock-on

The chosen coin is remembered (`lock`) and shown with a **magenta TARGET ring** on the dashboard
preview (`POST /target`, expires after 60 s without updates). The script never switches to another
coin until this one is done or you answer `n`. If the coin is lost it steps back to where it was
last seen and retries with half-size steps.

### Centring by pixels

The camera is on the arm and sees nothing in 3D below ~20 cm, so centring works on pixels. The arm
is moved a little (two probe moves of 12 mm, once per run) to learn the 2x2 **image Jacobian**
(pixels per mm of tool motion, `domain/centering.py`); each step is then `J^-1 x pixel error`.

The probes and the tracking after each move use the **whole picture's shift** between two frames
(phase correlation: `POST /ref` marks a reference frame, `POST /shift` measures how far everything
moved), not "the nearest circle". In a pile neighbouring coins look alike and sit ~20 px apart, so
following the nearest circle jumped to the wrong coin, which produced a wrong Jacobian and a
diverging arm. If the correlation is weak the script falls back to following the circle.

When the coin is within `FINISH_PX` it stops centring and folds the last correction into the descent.

### The descent

The coin surface is `depth` below the tool, so the target Z is `tip.z - depth + margin`. The lowest Z
the arm can actually reach at that x, y is computed on the kinematics (no trial moves) and then
**one coordinated `move_to`** does the x/y correction and the descent together.

What the arm can reach is limited (elbow 0..-90 deg, shoulder 0..80 deg):

| Tool Z (mm) | Reachable distance from the base (mm) |
|-------------|---------------------------------------|
| 295 (working pose) | 342-396 |
| 59   | 258-448 |
| 20   | 282-404 |
| 3    | 288-372 |
| -10  | 290-330 |

The coin surface is around Z -10..0, so to touch coins the pile must be **30-35 cm from the base**.
When it cannot go lower the script says so and stops at the lowest reachable Z.

### Drop spot and return

After the pick the arm rises, moves to the **drop spot**, calls `release()`, and returns to the spot
above where it picked, not to a fixed home: the rest of the pile is next. The default drop spot is
15 cm to the right (-Y) of the working pose, lowered if unreachable; set your own with `--drop X Y Z`,
`d` at the prompt, or `center_settings.py drop here`.

## Calibration

`center_coin.py --calibra` runs the camera->base calibration **at the working pose** (the transform
is only valid at the pose it was made at) and learns where the nozzle sits in the image:

1. Put ONE coin in view. The script detects it from the working pose.
2. Jog the tool to **touch** that coin (`tools/jog.py` in another terminal), press Enter. Repeat for
   at least 4 spots (`f` to finish).
3. It solves `base = R cam + t` (same Kabsch maths as `tools/calibrate_camera.py`, written to
   `camera_calib.json`), and from the same touches the **pixel where the nozzle is** and the **height
   at which it touches** (`nozzle_calib.json`).

Afterwards the script loads them by itself when the working pose matches (within 15 mm / 3 deg):
the first move goes straight to the coin, centring aims at the nozzle pixel instead of the image
centre, and the descent uses the learned touch height (+3 mm).

**It keeps learning from your answers.** After a miss, answer `m`, jog the tool to the right spot
and press Enter: the script measures the offset, moves the learned nozzle pixel and touch height
50% toward it, and saves. Redo `--calibra` when the camera mount or the working pose changes, and
when the suction cup is fitted (the tool tip changes).

## Settings

`tools/center_settings.py` edits the tunables; `center_coin.py` reads them at its next start.

```
python3 ~/projects/moneysort/tools/center_settings.py             # menu
python3 .../center_settings.py show
python3 .../center_settings.py set GAIN 0.8
python3 .../center_settings.py drop 392 -150 294      # or: drop here
python3 .../center_settings.py reset
```

| Key | Default | Meaning |
|-----|---------|---------|
| `PPS` | 6000 | max pulse rate of the moves (above 8000 raises the risk of queue wedges) |
| `SETTLE_S` | 0.25 | wait after a move before looking (slow camera -> higher) |
| `GAIN` | 1.0 | fraction of the pixel error corrected per step |
| `MAX_STEP_MM` | 60 | biggest correction in one step |
| `TOL_PX` | 10 | "centred" within this many pixels of the target |
| `FINISH_PX` | 30 | within this, fold the correction into the descent (0 = never) |
| `TRACK_GATE_PX` | 50 | how far from the prediction a coin still counts as the same one |
| `PROBE_MM` | 12 | size of the two probe moves |
| `APPROACH_MARGIN_MM` | 15 | distance kept above the coin when there is no calibration |
| `MAX_DROP_MM` | 50 | how far it may sink while centring to reach a far coin |
| `PUMP_FITTED` | no | carry on by itself after each coin |

Definitions live in `tools/center_params.py` (one table: default, type, range, description).

## Files

| File | In git? | What |
|------|---------|------|
| `camera_calib.json` | no | camera->base transform at the working pose |
| `nozzle_calib.json` | no | nozzle pixel + touch height, refined by your `m` answers |
| `center_settings.json` | no | tunables and the drop spot |

## When the pulse queue wedges ("TX queues stuck")

`lgpio`'s software pulse queue sometimes stops draining: every axis shows `room=0 busy=1` and moves
fail with `TX queues stuck: no room for 5.0s`. A cancel (`tx_pulse(0,0)`) empties the queue, but the
next move can wedge again; only restarting the process reliably fixes it. The cause is not proven.
Seen so far: it happens on **tiny moves** (a few mm, mostly right after another move) and during
fast jogging, both of which queue very short bursts. What is in place:

- tiny bursts are **merged** (no burst under 8 cycles, `domain/planning.py`), keeping the queue feed
  away from lgpio's idle/wake window;
- every lgpio `tx_*` call and the home-switch read goes through **one lock** (HTTP status polls used
  to run concurrently with the move thread);
- on a stuck queue `armd` cancels it and logs the raw `room/busy` values; a **second wedge within
  3 minutes**, or an idle axis with no free slot after a cancel, **restarts the daemon** (motors stay
  enabled so the arm holds its pose; axes come back un-homed);
- `center_coin.py` waits for `armd`, **re-homes** if it lost the position, goes back to the working pose
  and re-runs the centring by itself (up to 3 times);
- manual: `POST /recover`, or the "Recupera (riavvia)" button on the dashboard (shown only while
  pulses are still running).

If it keeps happening, read `journalctl -u moneysort-arm` for the `tx_room/tx_busy` lines and the
`RECOVERY:` entries. A power glitch can cause the same symptoms (check `vcgencmd get_throttled`).

## API used (all on `armd`, :8080)

| Method | Path | Purpose |
|--------|------|---------|
| POST | `/detect` `{require_depth:false}` | coins in view (`px, py, r, depth_ok, depth_m, cam_mm, base_mm?, held?`) + frame size |
| POST | `/ref` | mark the current frame as the reference for `/shift` |
| POST | `/shift` | `{dx, dy, response}`: how far the picture moved since `/ref` (response ~1 reliable) |
| POST | `/target` `{px, py}` / `{clear:true}` | draw the TARGET ring on the dashboard preview |
| POST | `/move_to` `{x,y,z,pps}` | tool tip to a point (all limits apply) |
| POST | `/home` | home x/y, z to zero |
| POST | `/recover` | restart the daemon (frees a wedged queue) |
| GET | `/status`, `/detected`, `/camera` | state, annotated frame, MJPEG feed |

## Where the pump plugs in

`pick(coin)` runs with the tool at its lowest reachable Z over the coin; `release()` runs at the drop
spot. See [`GRIPPER.md`](GRIPPER.md) for the hardware and the planned `Gripper` / `POST /grip`.
Then set `PUMP_FITTED = yes` and redo `--calibra`.
