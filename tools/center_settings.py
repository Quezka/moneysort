#!/usr/bin/env python3
"""View and change the settings of center_coin.py (speed, tolerances, margins, drop spot...).

    python3 ~/projects/moneysort/tools/center_settings.py              # interactive menu
    python3 .../center_settings.py show                                # print everything
    python3 .../center_settings.py set GAIN 0.8                        # change one value
    python3 .../center_settings.py drop 392 -150 294                   # drop spot (mm, base frame; -Y = right)
    python3 .../center_settings.py drop here                           # drop spot = where the tool is now
    python3 .../center_settings.py reset                               # back to the defaults

Values are saved in center_settings.json (git-ignored) and read by center_coin.py at its next start.
"""
import json
import os
import sys
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import center_params as cp
from moneysort.config import PORT
from moneysort.domain import kinematics
from moneysort.domain.kinematics import JointAngles


def tool_tip():
    """Where the tool is now, from armd (needs the daemon up)."""
    with urllib.request.urlopen(f"http://localhost:{PORT}/status", timeout=5) as r:
        j = json.load(r)["arm"]["joints"]
    return kinematics.forward(JointAngles(x=j["x"], y=j["y"], z=j["z"]))


def show(data):
    vals = cp.values(data)
    print("\nImpostazioni di center_coin.py (* = diverso dal default)\n")
    for i, (key, (default, kind, lo, hi, desc)) in enumerate(cp.PARAMS.items(), 1):
        mark = "*" if vals[key] != default else " "
        print(f"{i:2d}. {mark} {key:<19} = {vals[key]!s:<7} (default {default!s:<6}) {desc}")
    d = data.get("drop")
    print("\n d.   DROP (scarico)        = " + (f"X{d[0]:.0f} Y{d[1]:.0f} Z{d[2]:.0f} mm" if d else
                                              "default: 15 cm a destra della posa di lavoro"))


def set_param(data, key, text):
    key = key.upper()
    if key not in cp.PARAMS:
        raise ValueError(f"impostazione sconosciuta: {key}")
    value = cp.parse(key, text)
    data.setdefault("params", {})[key] = value
    print(f"  {key} = {value}")


def set_drop(data, args):
    if args and args[0] == "here":
        p = tool_tip()
        xyz = [p.x, p.y, p.z]
    else:
        if len(args) != 3:
            raise ValueError("servono X Y Z (mm), oppure 'here'")
        xyz = [float(a) for a in args]
    try:
        kinematics.inverse(kinematics.Point(*xyz))
    except kinematics.OutOfReach:
        print("  attenzione: quel punto non e' raggiungibile; center_coin lo abbassera' a quanto serve")
    data["drop"] = xyz
    print(f"  scarico = X{xyz[0]:.0f} Y{xyz[1]:.0f} Z{xyz[2]:.0f} mm")


def menu():
    data = cp.load()
    keys = list(cp.PARAMS)
    while True:
        show(data)
        a = input("\nNumero = cambia, d = scarico, r = ripristina i default, q = esci e salva: ").strip().lower()
        try:
            if a in ("q", ""):
                break
            if a == "r":
                data.pop("params", None)
                print("  default ripristinati (lo scarico resta)")
            elif a == "d":
                b = input("  scarico: 'qui' = dove sta la punta ora, oppure 'X Y Z' in mm: ").strip().lower()
                set_drop(data, ["here"] if b in ("qui", "here") else b.split())
            elif a.isdigit() and 1 <= int(a) <= len(keys):
                key = keys[int(a) - 1]
                default, kind, lo, hi, desc = cp.PARAMS[key]
                rng = "si/no" if kind is bool else f"{lo}..{hi}"
                set_param(data, key, input(f"  {key} ({desc})\n  nuovo valore [{rng}]: "))
            else:
                print("  non capisco")
        except (ValueError, OSError) as e:
            print(f"  errore: {e}")
        cp.save(data)
    cp.save(data)
    print("salvato in", cp.PATH)


def main():
    args = sys.argv[1:]
    data = cp.load()
    if not args:
        return menu()
    cmd, rest = args[0], args[1:]
    try:
        if cmd == "show":
            show(data)
            return
        if cmd == "set" and len(rest) == 2:
            set_param(data, rest[0], rest[1])
        elif cmd == "drop":
            set_drop(data, rest)
        elif cmd == "reset":
            data.pop("params", None)
            print("default ripristinati (lo scarico resta)")
        else:
            sys.exit(__doc__)
    except (ValueError, OSError) as e:
        sys.exit(f"errore: {e}")
    cp.save(data)


if __name__ == "__main__":
    main()
