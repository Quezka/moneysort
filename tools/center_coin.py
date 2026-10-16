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
import center_params as cp                       # tools/center_params.py: settings shared with center_settings.py
from moneysort.domain import calibration as camcalib
from moneysort.domain import centering, kinematics
from moneysort.domain.kinematics import JointAngles, Point

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAMERA_CALIB = os.path.join(ROOT, "camera_calib.json")     # camera -> base (tools/calibrate_camera.py)
NOZZLE_CALIB = os.path.join(ROOT, "nozzle_calib.json")     # where the nozzle is in the image + touch height

ARMD = f"http://localhost:{PORT}"
PPS = 6000              # gentle; high rates load lgpio's software-timed pulses and have jammed its queue
PROBE_MM = 12.0         # size of the two probe moves (done once, then the Jacobian is reused)
TOL_PX = 10             # "centred" when the coin is within this many pixels of the target
MAX_ITERS = 12
SETTLE_S = 0.25         # wait after a move so the camera frame is fresh and still
GAIN = 1.0              # fraction of the pixel error corrected per step (the Jacobian is measured, not guessed)
MAX_STEP_MM = 60.0      # biggest single correction
FINISH_PX = 30          # this close: stop centring, fold the last correction into the descent itself
FAST_FINISH = True
PUMP_FITTED = False     # True: after a coin it carries on by itself; False: asks first (you remove the coin)
SETTINGS = os.path.join(ROOT, "center_settings.json")      # where to drop the coins (git-ignored)
DROP_OFFSET_MM = (0.0, -150.0)   # default drop spot: this far from the working pose (base frame; -Y = to the right)
drop = None             # Point: where a picked coin is released
TRACK_GATE_PX = 50      # a coin further than this from where we predicted it isn't ours
jac_cache = None        # image Jacobian learned on the first coin, reused after
last_shift = None      # (dx, dy) measured by the camera for the last move (None = unusable)
MIN_RESPONSE = 0.15     # phase-correlation confidence below this: ignore the measurement
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
TARGET_OFFSET_PX = (0, 0)   # nozzle-vs-image-centre offset (replaced by the learned nozzle_px)
SURVEY_TOL_MM = 15.0        # a calibration is only trusted while the working pose is this close
LEARN_RATE = 0.5            # how much one corrected cycle moves the learned nozzle pixel / margin
nozzle = None               # {"px": [u, v], "margin": mm, "pose": [x, y, z], "n": updates} or None
camcal = None               # CameraCalib valid at the working pose, or None
last_cycle = None           # what the last descent did, so a correction can refine the calibration


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


def mark_ref():
    try:
        call("/ref", {}, timeout=3)
    except Exception:
        pass


def measure_shift():
    """Picture shift since mark_ref() as (dx, dy), or None if it can't be trusted."""
    try:
        r = call("/shift", {}, timeout=3)
    except Exception:
        return None
    if not r.get("ok") or r.get("response", 0.0) < MIN_RESPONSE:
        return None
    return (r["dx"], r["dy"])


def move_to(p):
    global lock, last_shift
    before = tool_tip()
    mark_ref()
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
    last_shift = measure_shift()
    if lock:                                    # the image shifts as the camera moves: follow it
        if last_shift:
            lock = (lock[0] + last_shift[0], lock[1] + last_shift[1])
        elif jac_cache:
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
        prev = seen[-1]
        if last_shift:                          # whole-picture shift: no coin identity needed
            expected = (prev["px"] + last_shift[0], prev["py"] + last_shift[1])
            c = centering.track(coins, expected, 40)
            shift = last_shift
        else:                                   # no texture to correlate: follow the coin itself
            c = centering.track(coins, (prev["px"], prev["py"]), 200)
            shift = None if c is None else (c["px"] - prev["px"], c["py"] - prev["py"])
        if c is None:
            return None, "ho perso la moneta durante la calibrazione dei movimenti"
        probes.append((moved, shift))
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
    centre_coin.finish = (0.0, 0.0)             # correction still to apply, folded into the descent
    coins, frame = detect()
    if not frame:
        return False, "nessun frame dalla camera"
    target = (tuple(nozzle["px"]) if nozzle
              else (frame[0] / 2 + TARGET_OFFSET_PX[0], frame[1] / 2 + TARGET_OFFSET_PX[1]))
    centre_coin.target_used = target
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

    # --- calibrated first move: the camera->base transform says where the tool must go ----
    if camcal and coin.get("depth_ok") and coin.get("cam_mm") and camcal.at_survey(
            call("/status")["arm"]["joints"], tol_deg=3.0):          # valid only at the survey pose
        b = camcal.to_base(coin["cam_mm"])
        tip = tool_tip()
        moved = try_move(b[0] - tip.x, b[1] - tip.y, scales=(1.0, 0.5))
        if moved:
            print(f"  mossa calibrata verso la moneta ({moved[0]:+.0f},{moved[1]:+.0f} mm)")
            coins, _ = detect()
            expected = (coin["px"] + last_shift[0], coin["py"] + last_shift[1]) if last_shift else (coin["px"], coin["py"])
            nxt = centering.track(coins, expected, LOCK_GATE_PX)
            if nxt is None:
                return False, "ho perso la moneta dopo la mossa calibrata"
            coin = nxt
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
        if FAST_FINISH and dist <= FINISH_PX:    # close enough: don't stop, correct while descending
            centre_coin.finish = centering.step_mm(jac, err, gain=1.0, max_mm=FINISH_PX)
            centre_coin.last = coin
            return True, f"quasi al centro ({dist:.0f} px): correggo durante la discesa"
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
        if last_shift:
            expected = (coin["px"] + last_shift[0], coin["py"] + last_shift[1])
        else:
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


def approach(coin, margin=None, dxy=(0.0, 0.0)):
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
    if margin is None:                          # learned touch height (+3 mm safety), else the default
        margin = nozzle["margin"] + 3.0 if nozzle else APPROACH_MARGIN_MM
    approach.margin_used = margin
    start = tool_tip()
    over = Point(start.x + dxy[0], start.y + dxy[1], start.z)       # the spot right above the coin
    target_z = start.z - coin["depth_m"] * 1000.0 + margin
    print(f"  moneta a {coin['depth_m'] * 100:.0f} cm sotto la camera: scendo da Z{start.z:.0f} "
          f"verso Z{target_z:.0f} mm (margine {margin:.0f} mm).")
    # Lowest Z actually reachable at this x, y (checked on the kinematics, no trial moves),
    # then ONE coordinated move: nothing to re-check on the way down, so it runs smoothly.
    z = target_z
    while z < start.z:
        try:
            kinematics.inverse(Point(over.x, over.y, z))
            break
        except kinematics.OutOfReach:
            z += 1.0
    if z >= start.z - 1.0:
        print("  non posso scendere a questa distanza dalla base: avvicina la moneta.")
        return start.z, over
    if z > target_z + 1.0:
        print(f"  piu' in basso di Z{z:.0f} mm non arrivo a questa distanza dalla base.")
    move_to(Point(over.x, over.y, z))           # x, y correction and descent in one smooth move
    return z, over


def _read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def load_calibrations(home):
    """Use the saved calibrations if they were made at (about) this working pose."""
    global nozzle, camcal
    d = _read_json(NOZZLE_CALIB)
    if d and math.dist(d["pose"][:2], (home.x, home.y)) <= SURVEY_TOL_MM and abs(d["pose"][2] - home.z) <= 10:
        nozzle = d
        print(f"  calibrazione punta caricata (pixel {d['px'][0]:.0f},{d['px'][1]:.0f}; "
              f"margine {d['margin']:.0f} mm; {d.get('n', 0)} aggiornamenti)")
    elif d:
        print("  calibrazione punta salvata per un'altra posa di lavoro: la ignoro (rifai --calibra).")
    c = camcalib.load(CAMERA_CALIB)
    j = call("/status")["arm"]["joints"]
    if c and c.at_survey(j, tol_deg=3.0):
        camcal = c
        print(f"  calibrazione camera->base attiva (rms {c.rms_mm:.1f} mm): prima mossa diretta sulla moneta")
    elif c:
        print("  calibrazione camera->base fatta per un'altra posa: la ignoro (rifai --calibra).")


def save_nozzle():
    with open(NOZZLE_CALIB, "w") as f:
        json.dump(nozzle, f, indent=2)


def learn_from_correction(cycle):
    """The user fixed the tool position by hand after a miss: refine the nozzle calibration.

    The tool ended at D but the right spot is T2 (user-corrected). Centring had put the coin at
    pixel P, so the pixel the coin SHOULD be at is P + J*(T2-D) (moving the tool by e shifts the
    coin by J e); the descent margin shifts by how much lower/higher the user put the tip.
    """
    global nozzle
    if jac_cache is None or cycle is None:
        print("  niente da correggere (nessun ciclo recente o calibrazione immagine mancante).")
        return
    input("Porta la punta SOPRA/SULLA moneta nel punto giusto (tools/jog.py in un altro terminale), poi Invio... ")
    t2 = tool_tip()
    d = cycle["down"]
    ex, ey = t2.x - d.x, t2.y - d.y
    du, dv = (jac_cache[0][0] * ex + jac_cache[0][1] * ey, jac_cache[1][0] * ex + jac_cache[1][1] * ey)
    new_px = [cycle["target"][0] + du, cycle["target"][1] + dv]
    new_margin = cycle["margin"] - (d.z - t2.z)
    if nozzle is None:
        nozzle = {"px": new_px, "margin": new_margin, "n": 1}
    else:
        r = LEARN_RATE
        nozzle["px"] = [nozzle["px"][0] * (1 - r) + new_px[0] * r, nozzle["px"][1] * (1 - r) + new_px[1] * r]
        nozzle["margin"] = nozzle["margin"] * (1 - r) + new_margin * r
        nozzle["n"] = nozzle.get("n", 0) + 1
    nozzle["pose"] = [cycle["home"].x, cycle["home"].y, cycle["home"].z]
    save_nozzle()
    print(f"  imparato: la punta e' in immagine a ({nozzle['px'][0]:.0f},{nozzle['px'][1]:.0f}) px, "
          f"margine di discesa {nozzle['margin']:.0f} mm (spostamento mm: {ex:+.0f},{ey:+.0f},{t2.z - d.z:+.0f}). "
          "Salvato: vale dalla prossima moneta.")


def calibrate(home):
    """--calibra: camera->base transform at the working pose + where the nozzle is in the image.

    For each of >= 4 coin spots: the script detects the coin from the working pose, you jog the
    tool to TOUCH it (tools/jog.py in another terminal) and press Enter; the arm goes back to the
    working pose. Solves base = R cam + t (same maths as tools/calibrate_camera.py) and, from the
    same touches, the pixel where the nozzle sits plus the height at which it touches.
    """
    try:
        import numpy as np
        import calibrate_camera as cal
    except ImportError as e:
        sys.exit(f"serve numpy per la calibrazione ({e})")
    global jac_cache, nozzle
    print("\n=== calibrazione (camera->base + punta) alla posa di lavoro ===")
    print("Per ogni punto: UNA moneta sotto la camera. La punta la porti tu a toccarla (jog.py).")
    cam_pts, base_pts, pxs, dzs = [], [], [], []
    while True:
        a = input(f"[punto {len(cam_pts) + 1}] metti UNA moneta e premi Invio ({'f = finisci, ' if len(cam_pts) >= 4 else ''}q = annulla)... ").strip().lower()
        if a == "q":
            return
        if a == "f" and len(cam_pts) >= 4:
            break
        move_to(home)
        coins, _ = detect()
        coins = [c for c in coins if c.get("depth_ok") and c.get("cam_mm")]
        if len(coins) != 1:
            print(f"  vedo {len(coins)} monete con profondita': ne serve esattamente UNA a 20-60 cm. Riprova.")
            continue
        coin = coins[0]
        if jac_cache is None:
            jac, res = learn_jacobian(coin)
            if jac is None:
                print("  " + res)
                continue
            jac_cache = jac
            move_to(home)
            coins, _ = detect()
            coins = [c for c in coins if c.get("depth_ok") and c.get("cam_mm")]
            if len(coins) != 1:
                continue
            coin = coins[0]
        print(f"  moneta a ({coin['px']:.0f},{coin['py']:.0f}) px, {coin['depth_m'] * 100:.0f} cm sotto la camera.")
        input("  Porta la punta a TOCCARE la moneta (jog.py), poi Invio... ")
        t = tool_tip()
        tp = np.array([t.x, t.y, t.z], float)
        delta = (t.x - home.x, t.y - home.y)
        px = (coin["px"] + jac_cache[0][0] * delta[0] + jac_cache[0][1] * delta[1],
              coin["py"] + jac_cache[1][0] * delta[0] + jac_cache[1][1] * delta[1])
        cam_pts.append(np.array(coin["cam_mm"], float))
        base_pts.append(tp)
        pxs.append(px)
        dzs.append(t.z - (home.z - coin["depth_m"] * 1000.0))
        print(f"  registrato ({len(cam_pts)}): punta a ({t.x:.0f},{t.y:.0f},{t.z:.0f}) mm, "
              f"pixel punta {px[0]:.0f},{px[1]:.0f}, quota di contatto {dzs[-1]:+.0f} mm sopra la moneta")
    R, tr = cal.rigid_transform(cam_pts, base_pts)
    resid = [float(np.linalg.norm(base_pts[i] - (R @ cam_pts[i] + tr))) for i in range(len(cam_pts))]
    rms = float(np.sqrt(np.mean(np.square(resid))))
    j = call("/status")["arm"]["joints"]
    with open(CAMERA_CALIB, "w") as f:
        json.dump({"R": R.tolist(), "t": tr.tolist(), "survey_pose": {k: j[k] for k in ("x", "y", "z")},
                   "n_points": len(cam_pts), "residual_rms_mm": rms,
                   "note": "base_mm = R @ cam_mm + t ; valid at the working pose (center_coin.py --calibra)"}, f, indent=2)
    nozzle = {"px": [sum(p[0] for p in pxs) / len(pxs), sum(p[1] for p in pxs) / len(pxs)],
              "margin": sum(dzs) / len(dzs), "pose": [home.x, home.y, home.z], "n": len(pxs)}
    save_nozzle()
    print(f"\nSalvato: camera->base (rms {rms:.1f} mm su {len(cam_pts)} punti) e punta in immagine a "
          f"({nozzle['px'][0]:.0f},{nozzle['px'][1]:.0f}) px, margine {nozzle['margin']:.0f} mm.")
    print("Da ora center_coin.py li usa da solo e continua a migliorarli con le tue correzioni.")


def release():
    """Hook for the suction pump (not fitted yet): pump off + vent valve open, the coin drops."""


def reachable_pose(p, max_drop=150.0):
    """p itself, or p lowered (10 mm at a time) until the arm can reach it; None if it never can."""
    z = p.z
    while z >= p.z - max_drop:
        q = Point(p.x, p.y, z)
        try:
            kinematics.inverse(q)
            return q
        except kinematics.OutOfReach:
            z -= 10.0
    return None


def save_drop():
    data = cp.load()                              # keep the other settings
    data["drop"] = [drop.x, drop.y, drop.z]
    cp.save(data)


def apply_settings():
    """Override the module constants with the values saved by tools/center_settings.py."""
    g = globals()
    for key, val in cp.values().items():
        if g.get(key) != val:
            print(f"  impostazione {key} = {val} (da center_settings.json)")
        g[key] = val


def load_drop(home):
    """Drop spot: --drop X Y Z (saved), else the saved one, else DROP_OFFSET_MM from the working pose."""
    global drop
    a = sys.argv
    if "--drop" in a:
        i = a.index("--drop")
        try:
            drop = Point(float(a[i + 1]), float(a[i + 2]), float(a[i + 3]))
        except (IndexError, ValueError):
            sys.exit("uso: --drop X Y Z   (mm, sistema di riferimento della base)")
        want = drop
        drop = reachable_pose(want)
        if drop is None:
            sys.exit(f"(X{want.x:.0f}, Y{want.y:.0f}, Z{want.z:.0f}) non e' raggiungibile")
        save_drop()
        return
    d = (_read_json(SETTINGS) or {}).get("drop")
    want = Point(*d) if d else Point(home.x + DROP_OFFSET_MM[0], home.y + DROP_OFFSET_MM[1], home.z)
    drop = reachable_pose(want)
    if drop is None:
        sys.exit("non trovo un punto di scarico raggiungibile: usa --drop X Y Z")
    if drop.z != want.z:
        print(f"  scarico abbassato a Z{drop.z:.0f} mm per essere raggiungibile")


def set_drop_here():
    """d: the tool tip is where the coins should be dropped from now on."""
    global drop
    input("Porta la punta dove vuoi lasciare le monete (jog.py), poi Invio... ")
    drop = tool_tip()
    save_drop()
    print(f"  scarico impostato a X{drop.x:.0f} Y{drop.y:.0f} Z{drop.z:.0f} mm (salvato).")


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
    apply_settings()
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
    load_calibrations(home)
    if "--calibra" in sys.argv:
        calibrate(home)
        return
    load_drop(home)
    auto = PUMP_FITTED or "--auto" in sys.argv
    print(f"\nPosa di lavoro: X{home.x:.0f} Y{home.y:.0f} Z{home.z:.0f} mm. "
          f"Scarico: X{drop.x:.0f} Y{drop.y:.0f} Z{drop.z:.0f} mm (cambialo con --drop X Y Z o col tasto d).")
    print("Per ogni moneta: la centra, scende, la prende, risale, la porta allo scarico e torna dov'era, "
          "tutto da solo. Tieni la mano vicino all'e-stop.")
    input("Metti una moneta sotto la camera. Invio per iniziare... ")

    tries = 0
    last_cycle = None
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
            a = "s"                                        # the whole cycle runs by itself, no questions
            coin = getattr(centre_coin, "last", None)
            try:
                reached = approach(coin, dxy=getattr(centre_coin, "finish", (0.0, 0.0)))   # down to the coin
                if reached is not None:
                    z_low, up_pose = reached
                    last_cycle = {"home": home, "target": centre_coin.target_used,
                                  "margin": approach.margin_used,
                                  "down": Point(up_pose.x, up_pose.y, z_low)}
                    print(f"  giu' a Z{z_low:.0f} mm: prendo la moneta")
                    pick(coin)                             # <- the pump goes here
                    print("  risalgo e la porto allo scarico")
                    move_to(up_pose)                       # back up to where the descent started
                    move_to(drop)
                    release()                              # <- pump off / valve here
                    print("  scaricata; torno dove l'ho presa (le altre sono li' vicino)")
                    move_to(up_pose)
                unlock()
            except (RuntimeError, OSError) as e:
                if not trouble(e):
                    raise
                recover_job(home)                          # position may be lost: centre again
                redo, a = True, "r"
        else:
            if auto and msg.startswith("nessuna moneta"):
                print("Finito: non vedo piu' monete.")
                break
            a = ask("Riprovo? [r = riprova, n = un'altra moneta, q = esci] ", {"r", "n", "q"})
        if a == "n":
            unlock()                                   # drop the locked coin: next round picks anew
        if a == "q":
            unlock()
            break
        if redo:
            continue
        if not (ok and a == "s"):                    # a failed attempt: back to the working pose
            print("Torno alla posa di lavoro...")
            try:
                move_to(home)
            except (RuntimeError, OSError, kinematics.OutOfReach) as e:
                if trouble(e):
                    recover_job(home)                # heals armd, re-homes if needed, ends at home
                else:
                    print(f"non riesco a tornare: {e}")
                    break
        elif not auto:                               # no pump yet: you take the coin away, then go on
            while True:
                go = input("Moneta fatta (resto qui sopra, dove l'ho presa). Invio = prossima (togli prima questa "
                           "se non c'e' ancora la pompa), m = la punta non era giusta, d = imposta lo scarico "
                           "qui, q = esci... ").strip().lower()
                if go == "m":
                    learn_from_correction(last_cycle)
                elif go == "d":
                    set_drop_here()
                else:
                    break
            if go == "q":
                break
    print("fine.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        stop_jog()
        unlock()
        print("\ninterrotto (il braccio resta dov'e').")
