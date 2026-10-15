"""Visual centering maths: bring a coin to the middle of the image -- pure, no I/O.

The camera is eye-in-hand and the depth sensor is blind below ~0.2 m, so we can't
turn a close coin into a 3D point. Instead we close the loop on PIXELS: move the
tool a little, see how the coin shifts in the image, and learn the 2x2 image
Jacobian J (pixels per mm of tool motion, base frame). That absorbs the unknown
camera yaw, mirroring and scale without any depth or calibration file. Then each
step is  d = J^-1 * (target_px - coin_px)  (damped and clipped).
"""
import math

MIN_DET = 1e-6          # |det J| below this (px^2/mm^2) = probes didn't move the image


def solve_image_jacobian(probes):
    """probes: [((dx_mm, dy_mm), (du_px, dv_px)), ...] (>= 2, not collinear).

    Least squares J with  (du, dv) = J @ (dx, dy).  Raises ValueError if the
    probes are degenerate (coin lost, or the image did not respond).
    """
    if len(probes) < 2:
        raise ValueError("need at least 2 probes")
    # J = U D^T (D D^T)^-1 with D = moves (2xn), U = pixel shifts (2xn)
    ddt = [[0.0, 0.0], [0.0, 0.0]]
    udt = [[0.0, 0.0], [0.0, 0.0]]
    for (dx, dy), (du, dv) in probes:
        d = (dx, dy)
        u = (du, dv)
        for i in range(2):
            for j in range(2):
                ddt[i][j] += d[i] * d[j]
                udt[i][j] += u[i] * d[j]
    inv = _inv2(ddt)
    j = [[sum(udt[r][k] * inv[k][c] for k in range(2)) for c in range(2)] for r in range(2)]
    if abs(j[0][0] * j[1][1] - j[0][1] * j[1][0]) < MIN_DET:
        raise ValueError("image did not respond to the probe moves")
    return j


def _inv2(m):
    det = m[0][0] * m[1][1] - m[0][1] * m[1][0]
    if abs(det) < 1e-12:
        raise ValueError("degenerate probes")
    return [[m[1][1] / det, -m[0][1] / det], [-m[1][0] / det, m[0][0] / det]]


def step_mm(jac, err_px, gain=0.7, max_mm=20.0):
    """Tool move (dx, dy) in mm, base frame, that shifts the coin by ~err_px.

    err_px = (target_px - coin_px). Damped by `gain` (< 1 so a wrong J still
    converges) and clipped to `max_mm` so one bad reading can't fling the arm.
    """
    inv = _inv2(jac)
    dx = gain * (inv[0][0] * err_px[0] + inv[0][1] * err_px[1])
    dy = gain * (inv[1][0] * err_px[0] + inv[1][1] * err_px[1])
    n = math.hypot(dx, dy)
    if n > max_mm:
        dx, dy = dx * max_mm / n, dy * max_mm / n
    return dx, dy


def servo_velocity(jac, err_px, kp=2.0, max_mm_s=60.0):
    """Tool velocity (vx, vy) in mm/s, base frame, that pulls the coin toward the target.

    Continuous version of step_mm: speed is proportional to the pixel error (kp [1/s] times
    the mm that would close it), so the arm slows smoothly into the target instead of
    stopping and starting; capped at max_mm_s.
    """
    inv = _inv2(jac)
    vx = kp * (inv[0][0] * err_px[0] + inv[0][1] * err_px[1])
    vy = kp * (inv[1][0] * err_px[0] + inv[1][1] * err_px[1])
    n = math.hypot(vx, vy)
    if n > max_mm_s:
        vx, vy = vx * max_mm_s / n, vy * max_mm_s / n
    return vx, vy


def error_px(coin, target):
    """(target - coin) in pixels; coin is a dict with px, py."""
    return target[0] - coin["px"], target[1] - coin["py"]


def centered(err, tol_px):
    return math.hypot(*err) <= tol_px


def predict_px(jac, px, moved_mm):
    """Where a coin that was at `px` should appear after the tool moved `moved_mm` (dx, dy):
    tool +d shifts the coin by J d (J is the learned image Jacobian)."""
    return (px[0] + jac[0][0] * moved_mm[0] + jac[0][1] * moved_mm[1],
            px[1] + jac[1][0] * moved_mm[0] + jac[1][1] * moved_mm[1])


def track(coins, expected, gate_px):
    """The coin nearest to where we expect ours, or None if none is within `gate_px`.

    In a pile a plain "nearest to where it was" can jump to a neighbour once the camera
    has moved; predicting with the Jacobian first, and gating, keeps us on the same coin.
    """
    c = nearest(coins, expected)
    if c is None or math.hypot(c["px"] - expected[0], c["py"] - expected[1]) > gate_px:
        return None
    return c


DEPTH_BAND_M = 0.006    # depths within this of the closest count as "the same level" (noise ~ 2-5 mm)


def pick_target(coins, point, band_m=DEPTH_BAND_M):
    """Which coin of a pile to go for: the one closest to the camera, i.e. the top one.

    The depth sensor wobbles by a few mm, and a coin is only ~2 mm thick, so coins whose depth is
    within `band_m` of the closest are treated as level and the one nearest `point` (the image
    target) wins among them. Coins without a depth reading are used only if none has one.
    """
    with_depth = [c for c in coins if c.get("depth_ok") and c.get("depth_m")]
    if not with_depth:
        return nearest(coins, point)
    top = min(c["depth_m"] for c in with_depth)
    return nearest([c for c in with_depth if c["depth_m"] <= top + band_m], point)


def nearest(coins, point):
    """The coin whose pixel centre is closest to `point`, or None."""
    if not coins:
        return None
    return min(coins, key=lambda c: math.hypot(c["px"] - point[0], c["py"] - point[1]))
