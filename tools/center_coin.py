#!/usr/bin/env python3
"""Centre a coin in the camera image, ask for confirmation, then wait for the next.

Run ON THE PI (armd running, camera ok, x & y homed):
    python3 ~/projects/moneysort/tools/center_coin.py

The depth camera can't read depth closer than ~0.2 m, so this works on PIXELS
only, like a person lining something up by eye:
  1. detect a coin (circle, depth not required),
  2. probe: nudge the tool +12 mm in base X, then +12 mm in base Y, and watch how
     the coin moves in the image -> the image Jacobian (domain/centering.py), which
     absorbs camera yaw / mirroring / scale without any calibration file,
  3. iterate small moves (damped, <= 20 mm each) until the coin sits at the image
     centre,
  4. ask you to confirm. Then you take the coin away by hand and say "ok": the arm
     goes back to where it started and looks for the next one.

It moves the tool in X/Y at a constant height, sinking at most 50 mm only when a step
is out of reach (move_to through armd, so
every limit and the e-stop still apply). Where the pump will go, `pick()` is the
hook. Keep a hand near the e-stop.
"""
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from moneysort.config import PORT
from moneysort.domain import centering, kinematics
from moneysort.domain.kinematics import JointAngles, Point

ARMD = f"http://localhost:{PORT}"
PPS = 12000             # small moves never reach this: it only lifts the ramp ceiling
PROBE_MM = 12.0         # size of the two probe moves (done once, then the Jacobian is reused)
TOL_PX = 10             # "centred" when the coin is within this many pixels of the target
MAX_ITERS = 12
SETTLE_S = 0.35         # wait after a move so the camera frame is fresh and still
GAIN = 0.9              # fraction of the pixel error corrected per step
MAX_STEP_MM = 35.0      # biggest single correction
TRACK_GATE_PX = 50      # a coin further than this from where we predicted it isn't ours
jac_cache = None        # image Jacobian learned on the first coin, reused after
TARGET_OFFSET_PX = (0, 0)   # nozzle-vs-image-centre offset: set once the pump is mounted


def call(path, body=None, timeout=60):
    if body is None:
        req = urllib.request.Request(ARMD + path)
    else:
        req = urllib.request.Request(ARMD + path, data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        try:
            msg = json.load(e).get("error", "?")
        except Exception:
            msg = str(e)
        raise RuntimeError(msg)


def tool_tip():
    j = call("/status")["arm"]["joints"]
    return kinematics.forward(JointAngles(x=j["x"], y=j["y"], z=j["z"]))


def move_to(p):
    call("/move_to", {"x": p.x, "y": p.y, "z": p.z, "pps": PPS}, timeout=120)
    time.sleep(SETTLE_S)


def _unreachable(e):
    return "range" in str(e).lower() or "reach" in str(e).lower()


def try_move(dx, dy, scales=(1.0,), z_floor=None):
    """Move the tool by (dx, dy) mm, shrinking the step if the target is out of reach.

    If that fails and z_floor is given, also try going lower, a little at a time down to
    z_floor: the arm reaches further out the lower it is (at the rest height the radial
    band is only ~55 mm wide). Returns the (dx, dy) really moved, or None if nothing worked.
    """
    p = tool_tip()
    drops = [0.0]
    if z_floor is not None:
        drops += [d for d in DROPS_MM if p.z - d >= z_floor]
    for drop in drops:
        for sc in scales:
            try:
                move_to(Point(p.x + dx * sc, p.y + dy * sc, p.z - drop))
                if drop:
                    print(f"  (abbasso la punta di {drop:.0f} mm per arrivarci)")
                q = tool_tip()
                return (q.x - p.x, q.y - p.y)
            except (RuntimeError, kinematics.OutOfReach) as e:
                if not _unreachable(e):
                    raise
    return None


DROPS_MM = (10.0, 20.0, 30.0, 40.0, 50.0)   # how far below the working pose the tool may sink
MAX_DROP_MM = 50.0
floor_z = None          # lowest Z allowed while centring; set from the working pose in main()
ROOM_MM = 50.0     # free travel wanted in every X/Y direction around the working pose


def has_room(p, room=ROOM_MM):
    """True if the tool can still move `room` mm each way in X and Y at p's height."""
    for dx, dy in ((room, 0), (-room, 0), (0, room), (0, -room)):
        try:
            kinematics.inverse(Point(p.x + dx, p.y + dy, p.z))
        except kinematics.OutOfReach:
            return False
    return True


def ready_pose(p):
    """Nearest pose to p with room to move sideways at constant height, or None.

    The rest pose is the very corner of the workspace (shoulder and elbow at their
    zero stops): at that height nothing else is reachable, so a few mm out and down
    are needed before the centring can nudge the tool around.
    """
    if has_room(p):
        return p
    cands = sorted(((dr * dr + dz * dz, dr, dz)
                    for dr in range(-100, 101, 5) for dz in range(0, 101, 5)))
    for _, dr, dz in cands:
        q = Point(p.x + dr, p.y, p.z - dz)       # dr along X: the arm points along +X at yaw 0
        if has_room(q):
            return q
    return None


def detect():
    """-> (coins, (W, H)). Coins without depth are included (require_depth=False)."""
    r = call("/detect", {"require_depth": False})
    return r.get("coins", []), r.get("frame")


def pick(coin):
    """Hook for the suction pump (not fitted yet): lower, pump on, lift."""


def learn_jacobian(coin):
    """Two probe moves -> image Jacobian. Returns (jac, coin_now) or (None, message)."""
    seen = [coin]
    probes = []
    for dx, dy in ((PROBE_MM, 0.0), (0.0, PROBE_MM)):
        for sign in (1, -1):                  # try the other way if one is out of reach
            moved = try_move(sign * dx, sign * dy)
            if moved:
                break
        else:
            return None, "fuori portata in ogni direzione: porta il braccio piu' al centro dell'area di lavoro"
        coins, _ = detect()
        c = centering.track(coins, (seen[-1]["px"], seen[-1]["py"]), 200)
        if c is None:
            return None, "ho perso la moneta durante la calibrazione dei movimenti"
        probes.append((moved, (c["px"] - seen[-1]["px"], c["py"] - seen[-1]["py"])))
        seen.append(c)
    try:
        return centering.solve_image_jacobian(probes), seen[-1]
    except ValueError as e:
        return None, f"calibrazione immagine non riuscita: {e}"


def centre_coin(prev=None, relearned=False):
    """Move until the coin is at the image centre. Returns (ok, message).

    With several coins in view (a pile) it goes for the one nearest the target point, then
    follows that same coin: each step predicts where it should land and only accepts a coin
    close to that spot.
    """
    global jac_cache
    coins, frame = detect()
    if not frame:
        return False, "nessun frame dalla camera"
    target = (frame[0] / 2 + TARGET_OFFSET_PX[0], frame[1] / 2 + TARGET_OFFSET_PX[1])
    coin = centering.nearest(coins, prev) if prev else centering.pick_target(coins, target)
    if coin is None:
        return False, "nessuna moneta in vista"
    if len(coins) > 1:
        d = f", a {coin['depth_m'] * 100:.0f} cm" if coin.get("depth_ok") else ""
        print(f"  vedo {len(coins)} monete: prendo quella piu' vicina alla camera (in cima){d}")

    # --- the image Jacobian: learned once (two probe moves), then reused -----
    if jac_cache is None:
        jac, res = learn_jacobian(coin)
        if jac is None:
            return False, res
        jac_cache, coin = jac, res
    jac = jac_cache

    # --- close the loop -----------------------------------------------------
    best = None
    for i in range(MAX_ITERS):
        err = centering.error_px(coin, target)
        dist = math.hypot(*err)
        print(f"  passo {i}: errore {dist:5.1f} px")
        if centering.centered(err, TOL_PX):
            return True, f"centrata (errore {dist:.0f} px)"
        if best is not None and dist > best * 1.5:
            jac_cache = None                  # the learned Jacobian no longer fits: relearn
            if not relearned:
                print("  l'errore sale: ricalibro i movimenti e riprovo")
                return centre_coin(prev, relearned=True)
            return False, "l'errore sta aumentando: mi fermo"
        best = dist if best is None else min(best, dist)
        dx, dy = centering.step_mm(jac, err, gain=GAIN, max_mm=MAX_STEP_MM)
        moved = try_move(dx, dy, scales=(1.0, 0.5, 0.25), z_floor=floor_z)
        if not moved:
            return False, "la moneta e' al limite dell'area raggiungibile: avvicinala al centro"
        coins, _ = detect()
        expected = centering.predict_px(jac, (coin["px"], coin["py"]), moved)
        nxt = centering.track(coins, expected, TRACK_GATE_PX)
        if nxt is None:
            return False, "ho perso la moneta"
        coin = nxt
    return False, "non riesco a centrarla in tempo"


def ask(prompt, choices):
    while True:
        a = input(prompt).strip().lower()
        if a in choices:
            return a


def main():
    try:
        st = call("/status")
    except Exception as e:
        sys.exit(f"armd non risponde su {ARMD} ({e})")
    if not st.get("camera", {}).get("ok"):
        sys.exit(f"camera non pronta: {st.get('camera', {}).get('error')}")
    if st.get("estopped"):
        sys.exit("e-stop attivo: riabilita i motori prima")
    if not {"x", "y"} <= set(st["arm"].get("homed", [])):
        sys.exit("fai prima l'homing di x e y (serve per move_to)")

    home = tool_tip()
    if abs(home.y) > 30:
        sys.exit("braccio ruotato: rimetti la base a zero (guarda verso +X) prima di iniziare")
    ready = ready_pose(home)
    if ready is None:
        sys.exit("non trovo una posa di lavoro raggiungibile da qui: muovi il braccio verso il centro")
    if ready is not home:
        print(f"\nDal riposo non c'e' margine per muoversi di lato: porto la punta a X{ready.x:.0f} "
              f"Z{ready.z:.0f} mm (da X{home.x:.0f} Z{home.z:.0f}).")
        input("Spazio libero sotto la camera? Invio per spostare il braccio... ")
        move_to(ready)
        home = ready
    global floor_z
    floor_z = home.z - MAX_DROP_MM
    print(f"\nPosa di partenza: X{home.x:.0f} Y{home.y:.0f} Z{home.z:.0f} mm "
          "(ci torno tra una moneta e l'altra).")
    print("Il braccio si muove solo in X/Y, a quota costante. Tieni la mano vicino all'e-stop.")
    input("Metti una moneta sotto la camera. Invio per iniziare... ")

    while True:
        print("\nCerco e centro la moneta...")
        try:
            ok, msg = centre_coin()
        except (RuntimeError, kinematics.OutOfReach) as e:
            ok, msg = False, f"errore: {e}"
        print(("OK: " if ok else "NO: ") + msg)

        if ok:
            a = ask("Confermi? [s = si', r = riprova, q = esci] ", {"s", "r", "q"})
            if a == "s":
                coins, _ = detect()
                if coins:
                    pick(coins[0])
                input("Togli la moneta a mano, poi premi Invio per procedere... ")
        else:
            a = ask("Riprovo? [r = riprova, q = esci] ", {"r", "q"})
        if a == "q":
            break
        print("Torno alla posa di partenza...")
        try:
            move_to(home)
        except (RuntimeError, kinematics.OutOfReach) as e:
            print(f"non riesco a tornare: {e}")
            break
        if ok and a == "s":
            input("Metti la prossima moneta. Invio per continuare... ")
    print("fine.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\ninterrotto (il braccio resta dov'e').")
