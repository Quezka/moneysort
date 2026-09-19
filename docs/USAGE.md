# Commanding the arm (CLI)

The arm is driven by the **`armd`** daemon, which owns all the GPIO and serves an
HTTP API (and the dashboard) on **port 8080**. You never touch the GPIO directly
— you send it commands over HTTP. Two ways to do that from a terminal:

- **`arm_test.py`** — a small helper, runs **on the Pi** (targets `localhost`).
- **`curl`** — the raw API, works from **any PC on the same network**.

Motion is serialized and refused while e-stopped; every command returns the arm
status as JSON.

---

## `arm_test.py` (on the Pi)

```
python3 arm_test.py <x|y|z> [steps] [pps]        # single axis
python3 arm_test.py x:400 y:-800 z:1200 [pps]    # several axes at once (arg with ':')
python3 arm_test.py home <x|y|z>                 # seek the home switch (x/y only)
```

- `steps` — signed step count; **sign sets direction** (default `800`).
- `pps` — cruise pulses/sec (default `20000`, a comfortable speed for all axes).

Examples:

```
python3 arm_test.py z 40000            # base: +40000 steps
python3 arm_test.py y -11000 12000     # shoulder: -11000 steps at 12000 pps
python3 arm_test.py x:-11000 z:20000   # elbow and base together
python3 arm_test.py home y             # home the shoulder
```

> `arm_test.py` covers **move** and **home** only. For zeroing, return-to-zero,
> e-stop, etc. use the HTTP API below (or the dashboard buttons).

---

## HTTP API with `curl` (from any PC)

Replace the host with the Pi's address — `money-sorter.local` (mDNS) or its IP
(e.g. `192.168.1.13`). All bodies are JSON; all are `POST` except `/status`.

```sh
PI=http://money-sorter.local:8080

# live status (system + arm): joints in degrees, moving, enabled, estopped, homed
curl -s $PI/status

# move one axis (signed steps set direction)
curl -s -X POST $PI/move -H 'Content-Type: application/json' \
     -d '{"axis":"z","steps":40000,"pps":20000}'

# move several axes at once
curl -s -X POST $PI/move -H 'Content-Type: application/json' \
     -d '{"moves":{"x":-11000,"z":20000},"pps":20000}'

# home a switch axis (x or y): fast seek -> back off -> slow re-approach -> zero
curl -s -X POST $PI/find_home -H 'Content-Type: application/json' -d '{"axis":"y"}'

# home everything: x and y seek switches, z drives back to zero
curl -s -X POST $PI/home

# return every axis to its tracked zero (no switch seek; no-op if already there)
curl -s -X POST $PI/return_zero

# set the current position as the new zero reference
curl -s -X POST $PI/zero

# EMERGENCY DISABLE: cut torque, abort motion in flight, un-home (latched)
curl -s -X POST $PI/disable

# clear the e-stop and re-energize
curl -s -X POST $PI/enable

# system control
curl -s -X POST $PI/reboot
curl -s -X POST $PI/poweroff
```

Endpoint reference:

| Method | Path          | Body                                            | Effect |
|--------|---------------|-------------------------------------------------|--------|
| GET    | `/status`     | —                                               | system + arm state (JSON) |
| GET    | `/`           | —                                               | dashboard page |
| POST   | `/move`       | `{axis,steps,pps}` or `{moves:{..},pps}`        | move one / several axes |
| POST   | `/find_home`  | `{axis}`                                         | home one switch axis |
| POST   | `/home`       | `{pps?}`                                          | home all (x/y switch, z→0) |
| POST   | `/return_zero`| `{pps?}`                                          | drive all axes to zero |
| POST   | `/zero`       | —                                               | define current pos as 0 |
| POST   | `/disable`    | —                                               | latched e-stop + abort |
| POST   | `/enable`     | —                                               | clear e-stop |
| POST   | `/reboot` / `/poweroff` | —                                     | system control |

---

## Steps ↔ degrees

Commands take **steps**; the dashboard shows **degrees**. Conversions (see
`docs/PINOUT.md` for the source):

| Axis | Steps/rev | Steps per degree | Usable range |
|------|-----------|------------------|--------------|
| **x** elbow    | 132,000 | 366.67 | 0 … −90° (0 … −33,000 steps) |
| **y** shoulder | 132,000 | 366.67 | 0 … 90° (0 … 33,000 steps) |
| **z** base     | 157,005 | 436.13 | continuous |

e.g. 45° on x or y ≈ `366.67 × 45 ≈ 16,500` steps; 90° on z ≈ `39,251` steps.

> Soft limits (x/y) are enforced **only once the axis is homed**; before that
> there's no position reference and moves aren't clamped. An e-stop un-homes
> every axis, so re-home before relying on the limits again.
