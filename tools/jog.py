#!/usr/bin/env python3
"""Interactive keyboard teleop for the arm -- run over SSH to shake everything out.

Drives the arm through armd's HTTP API (localhost:8080), so armd stays the sole
GPIO owner and all its guards apply: x/y soft limits (once homed), the base
+/-180 deg hard limit, and the latched e-stop. This tool never touches GPIO.

    python3 ~/projects/moneysort/tools/jog.py        # on the Pi, in an SSH shell

Hold a key to move, release to stop (POST /jog: a dead-man in armd -- if the key
repeats stop arriving, the arm decelerates by itself within ~0.25 s). Two modes
(toggle with 'm'):
  JOINT      - one axis at a time, in degrees/second
  CARTESIAN  - the tool tip in mm/second, relative to where the arm points (up/down =
               extend/retract, left/right = swing the base), via the kinematics Jacobian

A terminal reports key *presses* (and auto-repeat) but never releases, so the
first press holds for FIRST_HOLD (about the OS repeat delay) and each repeat keeps
it alive for REPEAT_HOLD. Keys are single presses (no Enter); needs a real TTY.
"""
import json
import math
import os
import sys
import select
import termios
import time
import tty
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # repo root on path

from moneysort.config import PORT
from moneysort.domain import kinematics

BASE = f"http://localhost:{PORT}"
JOINT_SPEEDS = [1, 2, 5, 10, 20]              # deg/s
CART_SPEEDS = [5, 10, 20, 40, 80]             # mm/s
FIRST_HOLD = 0.5                              # s a first press keeps moving (>= OS repeat delay)
REPEAT_HOLD = 0.15                            # s each auto-repeat keeps it alive
BEAT = 0.08                                   # s between /jog heartbeats

# key -> (axis, sign) for JOINT mode
JOINT_KEYS = {"q": ("x", +1), "a": ("x", -1),
              "w": ("y", +1), "s": ("y", -1),
              "e": ("z", +1), "d": ("z", -1)}


# --- HTTP -----------------------------------------------------------------
def _post(path, payload=None, timeout=180):
    data = json.dumps(payload or {}).encode()
    req = urllib.request.Request(BASE + path, data=data,
                                 headers={"Content-Type": "application/json"}, method="POST")
    try:
        return json.load(urllib.request.urlopen(req, timeout=timeout)), None
    except urllib.error.HTTPError as e:
        try:
            return None, json.load(e).get("error", "?")
        except Exception:
            return None, e.read().decode()[:80]
    except Exception as e:
        return None, str(e)


def _status():
    try:
        return json.load(urllib.request.urlopen(BASE + "/status", timeout=10))
    except Exception:
        return None


# --- terminal -------------------------------------------------------------
def _keys(fd, wait):
    """Wait up to `wait` s for input; return the list of keys pressed.

    Reads the raw fd (NOT sys.stdin: its buffer swallows the rest of an arrow
    key's escape sequence, which select() then can't see -- the old reader took
    that for a lone Esc and quit). Arrows -> 'UP'/'DOWN'/'LEFT'/'RIGHT'.
    """
    if not select.select([fd], [], [], wait)[0]:
        return []
    data = os.read(fd, 64)
    out, i = [], 0
    while i < len(data):
        ch = data[i:i + 1]
        if ch == b"\x1b":
            if data[i + 1:i + 2] in (b"[", b"O") and i + 2 < len(data):
                out.append({b"A": "UP", b"B": "DOWN", b"C": "RIGHT", b"D": "LEFT"}.get(data[i + 2:i + 3], "?"))
                i += 3
                continue
            out.append("ESC")
        else:
            out.append(ch.decode("latin-1"))
        i += 1
    return out


HELP = """\
 MODE  m: toggle joint/cartesian      SPEED  ]: faster   [: slower
 HOLD a key to move, release to stop
 JOINT     q/a elbow x +/-   w/s shoulder y +/-   e/d base z +/-
 CARTESIAN arrows: up/down = extend/retract, left/right = swing base     w/s: up/down
 h: home all   k: return to zero   SPACE: E-STOP   o: re-enable   Esc: stop   Shift-Q: quit"""


def _render(st, mode, speed, msg, held):
    lines = ["  ARM JOG  (armd @ %s)" % BASE, ""]
    if st is None:
        lines.append("  \x1b[31m! can't reach armd -- is moneysort-arm running?\x1b[0m")
    else:
        j = st["arm"]["joints"]
        homed = ",".join(st["arm"].get("homed", [])) or "none"
        tip = kinematics.forward(kinematics.JointAngles(x=j["x"], y=j["y"], z=j["z"]))
        state = "\x1b[31mE-STOPPED\x1b[0m" if st.get("estopped") else (
            "enabled" if st.get("motors_enabled") else "disabled")
        unit = "deg/s" if mode == "JOINT" else "mm/s"
        lines += [
            "  mode  \x1b[1m%-9s\x1b[0m   speed %-8s   %s%s" % (
                mode, "%g %s" % (speed, unit), state, "   \x1b[32mMOVING\x1b[0m" if held else ""),
            "  homed %-6s" % homed,
            "",
            "  joints   x %7.1f    y %7.1f    z %7.1f   (deg)" % (j["x"], j["y"], j["z"]),
            "  tip      X %7.1f    Y %7.1f    Z %7.1f   (mm, r=%.0f)" % (
                tip.x, tip.y, tip.z, math.hypot(tip.x, tip.y)),
        ]
    lines += ["", "  " + (msg or ""), "", HELP]
    print("\x1b[H" + "\x1b[K\n".join(lines) + "\x1b[K\x1b[J", end="", flush=True)


def _jog_payload(mode, key, speed):
    """Key -> /jog body, or None if it is not a movement key."""
    if mode == "JOINT" and key in JOINT_KEYS:
        ax, sign = JOINT_KEYS[key]
        return {"rates": {ax: sign * speed}}
    if mode == "CARTESIAN" and key in ("UP", "DOWN", "LEFT", "RIGHT", "w", "s"):
        v = {"x": 0, "y": 0, "z": 0}
        if key in ("UP", "DOWN"):
            v["x"] = speed if key == "UP" else -speed
        elif key in ("RIGHT", "LEFT"):
            v["y"] = speed if key == "RIGHT" else -speed
        else:
            v["z"] = speed if key == "w" else -speed
        return {"cart": v, "frame": "arm"}
    return None


def main():
    if not sys.stdin.isatty():
        print("jog.py needs an interactive terminal (a real SSH shell).")
        return
    mode = "JOINT"
    sidx = {"JOINT": 1, "CARTESIAN": 2}            # per-mode speed index
    msg = "ready -- press h to home x & y before using CARTESIAN"
    active = None                                  # /jog body being held
    active_key, last_key_t, hold_until, last_beat = None, 0.0, 0.0, 0.0
    last_render = 0.0

    def speed():
        return (JOINT_SPEEDS if mode == "JOINT" else CART_SPEEDS)[sidx[mode]]

    def stop():
        nonlocal active, active_key
        if active is not None:
            _post("/jog", {}, timeout=3)
        active, active_key = None, None

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)            # cbreak: no echo/line-buffer, but Ctrl-C still works
        print("\x1b[2J", end="")
        while True:
            for k in _keys(fd, 0.03):
                now = time.monotonic()
                if k in ("Q", "\x03"):
                    stop()
                    return
                body = _jog_payload(mode, k, speed())
                if body is not None:
                    repeat = k == active_key and now - last_key_t < 0.7
                    last_key_t = now
                    hold_until = now + (REPEAT_HOLD if repeat else FIRST_HOLD)
                    if not repeat or body != active:
                        active, active_key, last_beat = body, k, 0.0
                    continue
                stop()                                       # any other key: stop first
                if k == "ESC":
                    msg = "stopped"
                elif k == "m":
                    mode = "CARTESIAN" if mode == "JOINT" else "JOINT"
                    msg = "mode -> %s" % mode
                elif k in ("]", "["):
                    n = len(JOINT_SPEEDS if mode == "JOINT" else CART_SPEEDS)
                    sidx[mode] = max(0, min(n - 1, sidx[mode] + (1 if k == "]" else -1)))
                    msg = "speed %s" % ("faster" if k == "]" else "slower")
                elif k == " ":
                    _post("/disable")
                    msg = "E-STOP -- press o to re-enable, then re-home"
                elif k == "o":
                    _post("/enable")
                    msg = "re-enabled (axes un-homed by an e-stop -- re-home)"
                elif k == "h":
                    _render(_status(), mode, speed(), "homing all ...", False)
                    _, err = _post("/home")
                    msg = "home failed: %s" % err if err else "homed"
                elif k == "k":
                    _render(_status(), mode, speed(), "returning to zero ...", False)
                    _, err = _post("/return_zero")
                    msg = "return_zero failed: %s" % err if err else "at zero"
            now = time.monotonic()
            if active is not None:
                if now >= hold_until:                        # key released
                    stop()
                    msg = "stopped"
                elif now - last_beat >= BEAT:                # dead-man heartbeat
                    _, err = _post("/jog", active, timeout=3)
                    last_beat = now
                    if err:
                        active, active_key = None, None
                        msg = "rejected: %s" % err
            if now - last_render >= 0.2:
                _render(_status(), mode, speed(), msg, active is not None)
                last_render = now
    except KeyboardInterrupt:
        pass
    finally:
        try:
            _post("/jog", {}, timeout=3)                     # never leave it running
        except Exception:
            pass
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n\x1b[0mbye.")


if __name__ == "__main__":
    main()
