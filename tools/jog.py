#!/usr/bin/env python3
"""Interactive keyboard teleop for the arm -- run over SSH to shake everything out.

Drives the arm through armd's HTTP API (localhost:8080), so armd stays the sole
GPIO owner and all its guards apply: x/y soft limits (once homed), the base
+/-180 deg hard limit, and the latched e-stop. This tool never touches GPIO.

    python3 ~/projects/moneysort/tools/jog.py        # on the Pi, in an SSH shell

Two modes (toggle with 'm'):
  JOINT      - jog one axis at a time, in degrees
  CARTESIAN  - jog the tool tip in millimetres (base frame) via move_to / IK

Keys are single presses (no Enter). It reads a raw terminal, so it needs a real
TTY -- an interactive SSH session is fine; a piped/non-tty stdin is not.
"""
import json
import math
import os
import sys
import termios
import tty
import select
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # repo root on path

from moneysort.config import JOINTS, PORT
from moneysort.domain import kinematics

BASE = f"http://localhost:{PORT}"
JOG_PPS = 8000                       # gentle jog speed (cruise is ~20000)
SPR = {ax: JOINTS[ax]["steps_per_rev"] for ax in ("x", "y", "z")}

JOINT_STEPS = [0.5, 1.0, 2.0, 5.0, 10.0]      # degrees
CART_STEPS = [1, 2, 5, 10, 20]                # millimetres

# key -> (axis, sign) for JOINT mode
JOINT_KEYS = {"q": ("x", +1), "a": ("x", -1),
              "w": ("y", +1), "s": ("y", -1),
              "e": ("z", +1), "d": ("z", -1)}


# --- HTTP -----------------------------------------------------------------
def _post(path, payload=None):
    data = json.dumps(payload or {}).encode()
    req = urllib.request.Request(BASE + path, data=data,
                                 headers={"Content-Type": "application/json"}, method="POST")
    try:
        return json.load(urllib.request.urlopen(req, timeout=180)), None
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
def _read_key():
    """One keypress; arrow keys come back as 'UP'/'DOWN'/'LEFT'/'RIGHT', Esc as 'ESC'."""
    ch = sys.stdin.read(1)
    if ch != "\x1b":
        return ch
    if not select.select([sys.stdin], [], [], 0.02)[0]:
        return "ESC"
    if sys.stdin.read(1) != "[":
        return "ESC"
    if not select.select([sys.stdin], [], [], 0.02)[0]:
        return "ESC"
    return {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}.get(sys.stdin.read(1), "ESC")


HELP = """\
 MODE  m: toggle joint/cartesian      STEP  ]: bigger   [: smaller
 JOINT     q/a elbow x +/-   w/s shoulder y +/-   e/d base z +/-
 CARTESIAN arrows: forward/back = X, left/right = Y     w/s: Z up/down
 h: home all   k: return to zero   SPACE: E-STOP   o: re-enable   Q: quit"""


def _render(st, mode, jidx, cidx, target, msg):
    out = ["\x1b[H\x1b[2J", "  ARM JOG  (armd @ %s)" % BASE, ""]
    if st is None:
        out.append("  \x1b[31m! can't reach armd -- is moneysort-arm running?\x1b[0m")
        print("\n".join(out) + "\n" + HELP, end="", flush=True)
        return
    j = st["arm"]["joints"]
    homed = ",".join(st["arm"].get("homed", [])) or "none"
    estop = st.get("estopped")
    en = st.get("motors_enabled")
    tip = kinematics.forward(kinematics.JointAngles(x=j["x"], y=j["y"], z=j["z"]))
    r = math.hypot(tip.x, tip.y)
    state = "\x1b[31mE-STOPPED\x1b[0m" if estop else ("enabled" if en else "disabled")
    step = ("%.1f deg" % JOINT_STEPS[jidx]) if mode == "JOINT" else ("%d mm" % CART_STEPS[cidx])
    out += [
        "  mode  \x1b[1m%-9s\x1b[0m   step %-8s   %s" % (mode, step, state),
        "  homed %-6s" % homed,
        "",
        "  joints   x %7.1f    y %7.1f    z %7.1f   (deg)" % (j["x"], j["y"], j["z"]),
        "  tip      X %7.1f    Y %7.1f    Z %7.1f   (mm, r=%.0f)" % (tip.x, tip.y, tip.z, r),
    ]
    if mode == "CARTESIAN" and target is not None:
        out.append("  target   X %7.1f    Y %7.1f    Z %7.1f   (mm)" % (target.x, target.y, target.z))
    out += ["", "  " + (msg or ""), "", HELP]
    print("\n".join(out), end="", flush=True)


def main():
    if not sys.stdin.isatty():
        print("jog.py needs an interactive terminal (a real SSH shell).")
        return
    mode = "JOINT"
    jidx, cidx = 1, 2                 # default step: 1 deg / 5 mm
    target = None                    # current cartesian target (a Point)
    msg = "ready -- press h to home x & y before using CARTESIAN"

    def seed_target():
        st = _status()
        if not st:
            return None
        j = st["arm"]["joints"]
        return kinematics.forward(kinematics.JointAngles(x=j["x"], y=j["y"], z=j["z"]))

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)            # cbreak: no echo/line-buffer, but Ctrl-C still works
        _render(_status(), mode, jidx, cidx, target, msg)
        while True:
            k = _read_key()
            if k in ("ESC", "Q") or k == "\x03":         # Esc / Shift-Q / Ctrl-C quit
                break
            act = None                                   # (path, payload) to send

            if k == "m":
                mode = "CARTESIAN" if mode == "JOINT" else "JOINT"
                target = seed_target() if mode == "CARTESIAN" else None
                msg = "mode -> %s" % mode
            elif k == "]":
                if mode == "JOINT":
                    jidx = min(jidx + 1, len(JOINT_STEPS) - 1)
                else:
                    cidx = min(cidx + 1, len(CART_STEPS) - 1)
                msg = "step bigger"
            elif k == "[":
                if mode == "JOINT":
                    jidx = max(jidx - 1, 0)
                else:
                    cidx = max(cidx - 1, 0)
                msg = "step smaller"
            elif k == " ":
                _post("/disable"); msg = "E-STOP -- press o to re-enable, then re-home"
                target = None
            elif k == "o":
                _post("/enable"); msg = "re-enabled (axes un-homed by an e-stop -- re-home)"
            elif k == "h":
                msg = "homing all ..."
                _render(_status(), mode, jidx, cidx, target, msg)
                _, err = _post("/home")
                msg = "home failed: %s" % err if err else "homed"
                target = seed_target() if mode == "CARTESIAN" else target
            elif k == "k":
                msg = "returning to zero ..."
                _render(_status(), mode, jidx, cidx, target, msg)
                _, err = _post("/return_zero")
                msg = "return_zero failed: %s" % err if err else "at zero"
                target = seed_target() if mode == "CARTESIAN" else target
            elif mode == "JOINT" and k in JOINT_KEYS:
                ax, sign = JOINT_KEYS[k]
                steps = round(sign * JOINT_STEPS[jidx] / 360.0 * SPR[ax])
                act = ("/move", {"axis": ax, "steps": steps, "pps": JOG_PPS})
                msg = "%s %+g deg" % (ax, sign * JOINT_STEPS[jidx])
            elif mode == "CARTESIAN" and k in ("UP", "DOWN", "LEFT", "RIGHT", "w", "s"):
                if target is None:
                    target = seed_target()
                if target is None:
                    msg = "no status -- can't seed target"
                else:
                    d = CART_STEPS[cidx]
                    dx = {"UP": d, "DOWN": -d}.get(k, 0)
                    dy = {"RIGHT": d, "LEFT": -d}.get(k, 0)
                    dz = {"w": d, "s": -d}.get(k, 0)
                    nt = kinematics.Point(target.x + dx, target.y + dy, target.z + dz)
                    res, err = _post("/move_to", {"x": nt.x, "y": nt.y, "z": nt.z, "pps": JOG_PPS})
                    if err:
                        msg = "rejected: %s" % err          # keep old target
                    else:
                        target = nt
                        msg = "-> X%.0f Y%.0f Z%.0f" % (nt.x, nt.y, nt.z)

            if act:
                _, err = _post(*act)
                if err:
                    msg = "rejected: %s" % err
            _render(_status(), mode, jidx, cidx, target, msg)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n\x1b[0mbye.")


if __name__ == "__main__":
    main()
