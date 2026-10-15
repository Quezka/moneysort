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
PPS = 6000              # gentle; high rates load lgpio's software-timed pulses and have jammed its queue
PROBE_MM = 12.0         # size of the two probe moves (done once, then the Jacobian is reused)
TOL_PX = 10             # "centred" when the coin is within this many pixels of the target
MAX_ITERS = 12
SETTLE_S = 0.35         # wait after a move so the camera frame is fresh and still
GAIN = 0.9              # fraction of the pixel error corrected per step
MAX_STEP_MM = 35.0      # biggest single correction
TRACK_GATE_PX = 50      # a coin further than this from where we predicted it isn't ours
jac_cache = None        # image Jacobian learned on the first coin, reused after
lock = None             # (px, py) of the coin we are committed to; we never switch to another
LOCK_GATE_PX = 140      # how far from the remembered spot we still accept it as the same coin
SERVO = False           # centre with continuous jog velocity (smooth) instead of step moves
KP = 2.0                # servo: mm/s of tool speed per mm of remaining error (1/s)
MAX_SPEED_MM_S = 60.0
LOOP_S = 0.08           # pause between servo updates (a detect call takes about as long again)
STALL_S = 2.5           # no progress for this long -> sink the tool (more reach) or give up
SINK_MM_S = 15.0
SERVO_TIMEOUT_S = 30.0
SERVO_GATE_PX = 80      # tracking gate while the arm is moving
APPROACH_MARGIN_MM = 15.0   # stay this far above the coin surface when descending (nozzle offset)
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
    global lock
    before = tool_tip()
    for attempt in (0, 1):
        try:
            call("/move_to", {"x": p.x, "y": p.y, "z": p.z, "pps": PPS}, timeout=120)
            break
        except RuntimeError as e:
            if "TX queue" not in str(e) or attempt:
                raise
            print("  coda impulsi bloccata: aspetto il recupero automatico e riprovo...")
            time.sleep(5.0)                     # armd cancels the queue (or restarts) by itself
    time.sleep(SETTLE_S)
    if lock and jac_cache:                      # the image shifts as the camera moves: follow it
        after = tool_tip()
        lock = centering.predict_px(jac_cache, lock, (after.x - before.x, after.y - before.y))
        set_target(lock)


def set_target(xy):
    """Tell armd which coin we are locked on, so the dashboard preview draws a magenta ring on it."""
    try:
        call("/target", {"px": xy[0], "py": xy[1]}, timeout=3)
    except Exception:
        pass


def lock_to(coin):
    global lock
    lock = (coin["px"], coin["py"])
    set_target(lock)


def unlock():
    global lock
    lock = None
    try:
        call("/target", {"clear": True}, timeout=3)
    except Exception:
        pass


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
                q = Point(p.x + dx * sc, p.y + dy * sc, p.z - drop)
                kinematics.inverse(q)           # check here: armd would log every refusal as a dashboard warning
                move_to(q)
                if drop:
                    print(f"  (abbasso la punta di {drop:.0f} mm per arrivarci)")
                now = tool_tip()
                return (now.x - p.x, now.y - p.y)
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


def jog(vx, vy, vz=0.0, ttl=0.35):
    """Hold-to-move velocity (base frame, mm/s). Zeros stop. armd stops by itself after ttl."""
    call("/jog", {"cart": {"x": vx, "y": vy, "z": vz}, "ttl": ttl}, timeout=5)


def stop_jog():
    try:
        call("/jog", {"cart": {"x": 0, "y": 0, "z": 0}}, timeout=5)
    except Exception:
        pass


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
        lock_to(c)
    try:
        return centering.solve_image_jacobian(probes), seen[-1]
    except ValueError as e:
        return None, f"calibrazione immagine non riuscita: {e}"


def servo_centre(coin, target, jac):
    """Smooth centring: send a tool velocity proportional to the pixel error, keep updating it.

    Returns (ok, message, coin_now). If the error stops shrinking (the arm is at the edge of its
    reach) it sinks the tool, which reaches further, down to floor_z; then gives up.
    """
    t_start = last_t = best_t = time.monotonic()
    v = (0.0, 0.0)
    dist = math.hypot(*centering.error_px(coin, target))
    best, inside, lost, sinking = dist, 0, 0, False
    try:
        while time.monotonic() - t_start < SERVO_TIMEOUT_S:
            coins, _ = detect()
            now = time.monotonic()
            dt = now - last_t
            expected = centering.predict_px(jac, (coin["px"], coin["py"]), (v[0] * dt, v[1] * dt))
            nxt = centering.track(coins, expected, SERVO_GATE_PX)
            last_t = now
            if nxt is None:
                lost += 1
                v = (0.0, 0.0)
                jog(0.0, 0.0)
                if lost >= 4:
                    return False, "ho perso la moneta", coin
                time.sleep(LOOP_S)
                continue
            lost, coin = 0, nxt
            lock_to(coin)
            err = centering.error_px(coin, target)
            dist = math.hypot(*err)
            print(f"  errore {dist:5.1f} px", end="\r", flush=True)
            if centering.centered(err, TOL_PX):
                inside += 1
                if inside >= 2:
                    break
            else:
                inside = 0
            if dist < best * 0.9:
                best, best_t, sinking = dist, now, False
            vz = 0.0
            if now - best_t > STALL_S:                  # stuck: nearly out of reach at this height
                if tool_tip().z - 5.0 > floor_z:
                    vz, sinking = -SINK_MM_S, True
                else:
                    return False, "la moneta e' al limite dell'area raggiungibile: avvicinala al centro", coin
            v = centering.servo_velocity(jac, err, KP, MAX_SPEED_MM_S)
            jog(v[0], v[1], vz)
            time.sleep(LOOP_S)
        else:
            return False, "non riesco a centrarla in tempo", coin
    finally:
        stop_jog()
    print()
    time.sleep(SETTLE_S + 0.2)                           # let it stop, then check with a still frame
    coins, _ = detect()
    c = centering.track(coins, (coin["px"], coin["py"]), SERVO_GATE_PX)
    if c is None:
        return False, "ho perso la moneta", coin
    d = math.hypot(*centering.error_px(c, target))
    if d > TOL_PX * 1.5:
        return False, f"si e' fermata a {d:.0f} px dal centro", c
    return True, f"centrata (errore {d:.0f} px)", c


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
    if lock:                                    # already committed to a coin: only that one
        coin = centering.track(coins, lock, LOCK_GATE_PX)
        if coin is None:
            return False, ("non trovo piu' la moneta scelta (non passo a un'altra): rimettila in vista, "
                           "oppure 'n' per sceglierne un'altra")
    else:
        coin = centering.nearest(coins, prev) if prev else centering.pick_target(coins, target)
        if coin is None:
            return False, "nessuna moneta in vista"
        if len(coins) > 1:
            d = f", a {coin['depth_m'] * 100:.0f} cm" if coin.get("depth_ok") else ""
            print(f"  vedo {len(coins)} monete: scelgo quella piu' vicina alla camera (in cima){d}")
        print("  la bloccata (anello viola sulla preview): non vado su altre finche' non e' presa.")
    lock_to(coin)

    # --- the image Jacobian: learned once (two probe moves), then reused -----
    if jac_cache is None:
        jac, res = learn_jacobian(coin)
        if jac is None:
            return False, res
        jac_cache, coin = jac, res
    jac = jac_cache

    if SERVO:
        ok, msg, coin = servo_centre(coin, target, jac)
        centre_coin.last = coin
        return ok, msg

    # --- close the loop (step moves; the old way) ---------------------------
    best = None
    step_scale = 1.0                            # halved after losing the coin: smaller, safer steps
    for i in range(MAX_ITERS):
        err = centering.error_px(coin, target)
        dist = math.hypot(*err)
        print(f"  passo {i}: errore {dist:5.1f} px")
        if centering.centered(err, TOL_PX):
            centre_coin.last = coin
            return True, f"centrata (errore {dist:.0f} px)"
        if best is not None and dist > best * 1.5:
            jac_cache = None                  # the learned Jacobian no longer fits: relearn
            if not relearned:
                print("  l'errore sale: ricalibro i movimenti e riprovo")
                return centre_coin(prev, relearned=True)
            return False, "l'errore sta aumentando: mi fermo"
        best = dist if best is None else min(best, dist)
        dx, dy = centering.step_mm(jac, err, gain=GAIN * step_scale, max_mm=MAX_STEP_MM * step_scale)
        moved = try_move(dx, dy, scales=(1.0, 0.5, 0.25), z_floor=floor_z)
        if not moved:
            return False, "la moneta e' al limite dell'area raggiungibile: avvicinala al centro"
        coins, _ = detect()
        expected = centering.predict_px(jac, (coin["px"], coin["py"]), moved)
        nxt = centering.track(coins, expected, TRACK_GATE_PX)
        for _ in range(2):                      # a missed detection is often just one bad frame
            if nxt is not None:
                break
            time.sleep(0.4)
            coins, _ = detect()
            nxt = centering.track(coins, expected, TRACK_GATE_PX * 2)
        if nxt is None:                         # still gone: go back to where it was last seen
            print("  persa: torno dove l'avevo vista e riprovo con passi piu' piccoli")
            if try_move(-moved[0], -moved[1], z_floor=floor_z) is None:
                return False, "ho perso la moneta e non riesco a tornare indietro"
            coins, _ = detect()
            nxt = centering.track(coins, (coin["px"], coin["py"]), LOCK_GATE_PX)
            if nxt is None:
                return False, "ho perso la moneta"
            step_scale = max(step_scale * 0.5, 0.25)
        coin = nxt
        lock_to(coin)
    centre_coin.last = coin
    return False, "non riesco a centrarla in tempo"


def approach(coin, margin=APPROACH_MARGIN_MM):
    """Sink the tool straight down toward the coin (x, y fixed). Returns (Z reached, pose it started from).

    Depth comes from the camera (valid down to ~0.2 m): the coin surface is about depth below
    the tool, so the target is tip.z - depth + margin. Stops early where the arm can't go lower.
    """
    try:                                        # a fresh reading of the locked coin (its depth)
        coins, _ = detect()
        fresh = centering.track(coins, lock or ((coin or {}).get("px", 0), (coin or {}).get("py", 0)), 60)
        if fresh is not None:
            coin = fresh
    except RuntimeError:
        pass
    if not coin or not coin.get("depth_ok"):
        print("  nessuna profondita' valida: non scendo (alza la camera oltre 20 cm e riprova).")
        return None
    start = tool_tip()
    target_z = start.z - coin["depth_m"] * 1000.0 + margin
    print(f"  moneta a {coin['depth_m'] * 100:.0f} cm sotto la camera: scendo da Z{start.z:.0f} "
          f"verso Z{target_z:.0f} mm (margine {margin:.0f} mm).")
    # Lowest Z actually reachable at this x, y (checked on the kinematics, no trial moves),
    # then ONE coordinated move: nothing to re-check on the way down, so it runs smoothly.
    z = target_z
    while z < start.z:
        try:
            kinematics.inverse(Point(start.x, start.y, z))
            break
        except kinematics.OutOfReach:
            z += 1.0
    if z >= start.z - 1.0:
        print("  non posso scendere a questa distanza dalla base: avvicina la moneta.")
        return start.z, start
    if z > target_z + 1.0:
        print(f"  piu' in basso di Z{z:.0f} mm non arrivo a questa distanza dalla base.")
    move_to(Point(start.x, start.y, z))
    return z, start


def trouble(e):
    """True for failures armd recovers from by itself: a stuck pulse queue, or armd restarting."""
    return isinstance(e, OSError) or "TX queue" in str(e)


def wait_armd(timeout=120):
    """Poll /status until armd answers (it may be restarting after a queue wedge). Returns the status."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try:
            st = call("/status", timeout=3)
            if st.get("camera", {}).get("ok"):
                return st
        except Exception:
            pass
        print(".", end="", flush=True)
        time.sleep(2.0)
    raise RuntimeError("armd non torna disponibile")


def recover_job(home):
    """After a stuck queue / daemon restart: wait for armd, re-home if it lost its position, go back
    to the working pose, so the interrupted centring can simply be run again."""
    print("\n  Problema con il braccio (coda impulsi): aspetto il recupero di armd", end="", flush=True)
    time.sleep(4.0)                                     # let its own cancel/restart finish first
    st = wait_armd()
    print()
    if st.get("estopped"):
        raise RuntimeError("e-stop attivo: riabilita i motori e rilancia")
    if not {"x", "y"} <= set(st["arm"].get("homed", [])):
        print("  armd si e' riavviato e ha perso la posizione: rifaccio l'homing da solo...")
        call("/home", {}, timeout=300)
    unlock()                                            # image position of the old lock is meaningless now
    print("  torno alla posa di lavoro e rifaccio la centratura.")
    move_to(home)


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
        try:
            move_to(ready)
        except (RuntimeError, OSError) as e:
            if not trouble(e):
                raise
            recover_job(ready)                       # ends at the working pose
        home = ready
    global floor_z
    floor_z = home.z - MAX_DROP_MM
    print(f"\nPosa di partenza: X{home.x:.0f} Y{home.y:.0f} Z{home.z:.0f} mm "
          "(ci torno tra una moneta e l'altra).")
    print("Il braccio si muove solo in X/Y, a quota costante. Tieni la mano vicino all'e-stop.")
    input("Metti una moneta sotto la camera. Invio per iniziare... ")

    tries = 0
    while True:
        print("\nCerco e centro la moneta...")
        try:
            ok, msg = centre_coin()
        except (RuntimeError, OSError, kinematics.OutOfReach) as e:
            if trouble(e) and tries < 3:               # armd wedged/restarted: heal and run it again
                tries += 1
                recover_job(home)
                continue
            ok, msg = False, f"errore: {e}"
        tries = 0
        print(("OK: " if ok else "NO: ") + msg)
        redo = False

        if ok:
            a = ask("Confermi? [s = si', r = riprova, n = un'altra moneta, q = esci] ", {"s", "r", "n", "q"})
            if a == "s":
                coin = getattr(centre_coin, "last", None)
                input("Scendo verso la moneta (mano all'e-stop). Invio... ")
                try:
                    reached = approach(coin)
                    if reached is not None:
                        z_low, up_pose = reached
                        pick(coin)
                        input(f"Sono a Z{z_low:.0f} mm. Invio per risalire... ")
                        move_to(up_pose)                  # back up to where we started the descent
                except (RuntimeError, OSError) as e:
                    if not trouble(e):
                        raise
                    recover_job(home)                     # position may be lost: centre again
                    redo, a = True, "r"
                if not redo:
                    input("Togli la moneta a mano, poi premi Invio per procedere... ")
                    unlock()
        else:
            a = ask("Riprovo? [r = riprova, n = un'altra moneta, q = esci] ", {"r", "n", "q"})
        if a == "n":
            unlock()                                   # drop the locked coin: next round picks anew
        if a == "q":
            unlock()
            break
        if redo:
            continue
        print("Torno alla posa di partenza...")
        try:
            move_to(home)
        except (RuntimeError, OSError, kinematics.OutOfReach) as e:
            if trouble(e):
                recover_job(home)                    # heals armd, re-homes if needed, ends at home
            else:
                print(f"non riesco a tornare: {e}")
                break
        if ok and a == "s":
            input("Metti la prossima moneta. Invio per continuare... ")
    print("fine.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        stop_jog()
        unlock()
        print("\ninterrotto (il braccio resta dov'e').")
