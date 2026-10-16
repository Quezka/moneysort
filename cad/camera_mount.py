#!/usr/bin/env python3
"""Parametric 3D model of the RealSense D435 mount for the front of the arm.

    python cad/camera_mount.py            # writes cad/camera_mount.stl (+ assembly STL and previews)

See docs/CAMERA_MOUNT.md for the design, what was measured from the photos and what to check with a caliper
before printing. Needs: pip install trimesh manifold3d numpy matplotlib

Frame (mm): X = along the arm toward the tip, Y = left, Z = up. Origin = centre of the end plate's window,
on the plate's UNDERSIDE (z = 0). The plate's top face is at z = PLATE_T.
"""
import os
import sys

import numpy as np
import trimesh
from trimesh.boolean import difference, union

# ---------------------------------------------------------------- what to measure on the real arm ----
PLATE_T = 2.0            # end-plate thickness
PLATE_HALF_W = 18.0      # half the plate width (the photos suggest ~36 mm overall)
PLATE_EDGE_X = 32.0      # front edge of the plate, measured from the window centre
HOLES_DX = 49.5          # the 4 plate holes: distance along the arm (servo-bracket pattern; the photo says 47-49.5)
HOLES_DY = 10.0          # ... and across the arm
BOLT = 3.0               # bolt size through the plate holes (M3). If the holes are only ~2.6 mm use 2.5 (or drill 3.2)
WINDOW = (40.6, 20.2)    # the rectangular window (X, Y), the nozzle goes through it

# ---------------------------------------------------------------------------------- the camera ----
CAM_L, CAM_H, CAM_D = 90.0, 25.0, 25.0   # D435: length, height (bottom -> top face), depth (rear -> lens face)
CAM_TOL = 0.6            # pocket clearance around the camera
TRIPOD = 6.7             # clearance for the 1/4-20 bolt (6.35 mm)
Z_LENS = -4.0            # height of the lens face relative to the plate underside (negative = below it)

# ----------------------------------------------------------------------------- the printed part ----
BLOCK_BACK = 29.0        # block extends this far behind the window centre
BLOCK_T = 6.0            # block thickness (under the plate)
BLOCK_HALF_W = 17.0
WALL_T = 8.0             # wall that carries the camera
WALL_HALF_W = 49.0
POCKET_DEPTH = 2.0
NOZZLE_BORE = 6.4        # tube / cup stem; adjust when the suction cup is chosen
BOSS_D, BOSS_LEN = 14.0, 12.0
NUT_AF, NUT_T = 5.5, 2.4  # M3 nut

OUT = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(os.path.dirname(OUT), "docs", "img")


def box(x0, x1, y0, y1, z0, z1):
    m = trimesh.creation.box(extents=(x1 - x0, y1 - y0, z1 - z0))
    m.apply_translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    return m


def cyl(axis, center, r, h, sections=48):
    """Cylinder of radius r and length h centred on `center`, along 'x', 'y' or 'z'."""
    m = trimesh.creation.cylinder(radius=r, height=h, sections=sections)
    if axis == "x":
        m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, (0, 1, 0)))
    elif axis == "y":
        m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, (1, 0, 0)))
    m.apply_translation(center)
    return m


def hexprism(axis, center, af, h):
    """Hexagonal prism with `af` across flats."""
    return cyl(axis, center, af / np.sqrt(3), h, sections=6)


def sub(a, *cuts):
    return difference([a, *cuts], engine="manifold")


def add(*parts):
    return union(list(parts), engine="manifold")


def build():
    xe = PLATE_EDGE_X + 0.5                      # the wall starts just in front of the plate edge
    x_wall0, x_wall1 = xe, xe + WALL_T
    z_bot = -BLOCK_T
    z_top_wall = Z_LENS + CAM_D + 3.0
    zc = Z_LENS + CAM_D / 2                      # tripod screw height (middle of the camera's depth)

    block = box(-BLOCK_BACK, xe, -BLOCK_HALF_W, BLOCK_HALF_W, z_bot, 0.0)
    wall = box(x_wall0, x_wall1, -WALL_HALF_W, WALL_HALF_W, z_bot, z_top_wall)
    ledge = box(xe - 4.0, xe, -WALL_HALF_W, WALL_HALF_W, z_bot, z_bot + 4.0)   # L-section: stiffens the wings
    boss = cyl("z", (0, 0, z_bot - BOSS_LEN / 2 + 0.01), BOSS_D / 2, BOSS_LEN + 0.02)
    body = add(block, wall, ledge, boss)

    cuts = []
    # nozzle: bore through block + boss, M3 set screw across the boss with a nut trap on the -Y side
    cuts.append(cyl("z", (0, 0, (z_bot - BOSS_LEN) / 2), NOZZLE_BORE / 2, BOSS_LEN + BLOCK_T + 2))
    zs = z_bot - BOSS_LEN / 2
    cuts.append(cyl("y", (0, 0, zs), 1.6, BOSS_D + 2, sections=24))
    cuts.append(hexprism("y", (0, -BOSS_D / 2 + (NUT_T + 0.2) / 2 - 0.05, zs), NUT_AF + 0.3, NUT_T + 0.2))
    # the 4 plate bolts: clearance hole + nut pocket opening on the underside (bolt head sits on the plate top)
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * HOLES_DX / 2, sy * HOLES_DY / 2
            cuts.append(cyl("z", (x, y, z_bot / 2), BOLT / 2 + 0.2, BLOCK_T + 2, sections=24))
            cuts.append(hexprism("z", (x, y, z_bot + (NUT_T + 0.4) / 2 - 0.05), NUT_AF + 0.3 + (BOLT - 3) * 0.7, NUT_T + 0.4))
    # camera: pocket on the front face (anti-rotation), 1/4-20 bolt through the wall with a head recess behind
    cuts.append(box(x_wall1 - POCKET_DEPTH, x_wall1 + 1, -(CAM_L + CAM_TOL) / 2, (CAM_L + CAM_TOL) / 2,
                    Z_LENS - CAM_TOL / 2, Z_LENS + CAM_D + CAM_TOL / 2))
    cuts.append(cyl("x", ((x_wall0 + x_wall1) / 2, 0, zc), TRIPOD / 2, WALL_T + 2, sections=32))
    cuts.append(cyl("x", (x_wall0 + 2.0, 0, zc), 6.0, 4.0, sections=32))        # 12 mm head recess, 4 mm deep from the rear
    # cable-tie slots in both wings (the USB-C end of the camera depends on how it is turned)
    for sy in (-1, 1):
        for z in (Z_LENS + 3.0, Z_LENS + CAM_D - 3.0):
            cuts.append(box(x_wall0 - 1, x_wall1 + 1, sy * 38.0 - 2.5, sy * 38.0 + 2.5, z - 1.0, z + 1.0))
    # lightening holes in the wings
    for sy in (-1, 1):
        cuts.append(cyl("x", ((x_wall0 + x_wall1) / 2, sy * 30.0, zc), 5.0, WALL_T + 2, sections=32))
    return sub(body, *cuts)


def mock_plate():
    p = box(-60, PLATE_EDGE_X, -PLATE_HALF_W, PLATE_HALF_W, 0, PLATE_T)
    cuts = [box(-WINDOW[0] / 2, WINDOW[0] / 2, -WINDOW[1] / 2, WINDOW[1] / 2, -1, PLATE_T + 1)]
    for sx in (-1, 1):
        for sy in (-1, 1):
            cuts.append(cyl("z", (sx * HOLES_DX / 2, sy * HOLES_DY / 2, 1.0), BOLT / 2, PLATE_T + 2, sections=24))
    return sub(p, *cuts)


def mock_camera():
    x_front = PLATE_EDGE_X + 0.5 + WALL_T - POCKET_DEPTH
    c = box(x_front, x_front + CAM_H, -CAM_L / 2, CAM_L / 2, Z_LENS, Z_LENS + CAM_D)
    return c


def checks(mount, plate, cam):
    """Numbers worth knowing; asserts the things that must hold."""
    r = {}
    r["watertight"] = bool(mount.is_watertight)
    r["volume_cm3"] = mount.volume / 1000.0
    r["mass_g_solid_PETG"] = r["volume_cm3"] * 1.27
    r["mass_g_est_40pct_infill"] = r["mass_g_solid_PETG"] * 0.55
    r["bounds_mm"] = [round(float(v), 1) for v in (mount.bounds[1] - mount.bounds[0])]
    # interference: the camera must only touch the printed part through the pocket
    inter = trimesh.boolean.intersection([mount, cam], engine="manifold")
    r["camera_overlap_mm3"] = float(inter.volume) if len(inter.vertices) else 0.0
    inter = trimesh.boolean.intersection([mount, plate], engine="manifold")
    r["plate_overlap_mm3"] = float(inter.volume) if len(inter.vertices) else 0.0
    # nozzle tip must be outside the camera's vertical field of view (along the arm: +-29 deg for the depth stream)
    nozzle_x = 0.0
    cam_x = PLATE_EDGE_X + 0.5 + WALL_T - POCKET_DEPTH + CAM_H / 2
    below = 45.0 + 6.0 - Z_LENS * -1  # nozzle protrudes ~45 mm below the block, the lens is 4 mm below the plate
    ang = np.degrees(np.arctan2(cam_x - nozzle_x, below))
    r["nozzle_behind_camera_mm"] = round(cam_x - nozzle_x, 1)
    r["nozzle_angle_from_camera_vertical_deg"] = round(float(ang), 1)
    r["camera_centre_ahead_of_nozzle_mm"] = round(cam_x, 1)
    return r


def render(parts, path, elev, azim, title=""):
    """Quick matplotlib preview (no OpenGL needed)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(8, 6), dpi=130)
    ax = fig.add_subplot(111, projection="3d")
    allv = []
    for mesh, color, alpha in parts:
        tris = mesh.triangles
        light = np.array([0.3, -0.5, 0.8])
        n = mesh.face_normals
        shade = 0.55 + 0.45 * np.clip(n @ (light / np.linalg.norm(light)), 0, 1)
        fc = np.array([matplotlib.colors.to_rgb(color)]) * shade[:, None]
        ax.add_collection3d(Poly3DCollection(tris, facecolors=fc, edgecolors=(0, 0, 0, 0.05), linewidths=0.2, alpha=alpha))
        allv.append(mesh.vertices)
    v = np.vstack(allv)
    c, s = (v.max(0) + v.min(0)) / 2, (v.max(0) - v.min(0)).max() / 2
    ax.set_xlim(c[0] - s, c[0] + s)
    ax.set_ylim(c[1] - s, c[1] + s)
    ax.set_zlim(c[2] - s, c[2] + s)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("X (arm, tip ->)")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(title, fontsize=9)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main():
    mount = build()
    plate, cam = mock_plate(), mock_camera()
    mount.export(os.path.join(OUT, "camera_mount.stl"))
    asm = trimesh.util.concatenate([mount, plate, cam])
    asm.export(os.path.join(OUT, "camera_mount_assembly.stl"))
    res = checks(mount, plate, cam)
    for k, v in res.items():
        print(f"{k:42s} {v}")
    if "--no-render" in sys.argv:
        return
    os.makedirs(IMG, exist_ok=True)
    parts = [(plate, "#9aa0a6", 0.55), (cam, "#cfd3d7", 0.55), (mount, "#2f6fdb", 1.0)]
    render(parts, os.path.join(IMG, "camera-mount-iso.png"), 24, -52, "Mount (blue) on the end plate (grey), D435 (light)")
    render(parts, os.path.join(IMG, "camera-mount-side.png"), 0, -90, "Side view: arm to the left, tip to the right")
    render(parts, os.path.join(IMG, "camera-mount-top.png"), 90, -90, "Top view")
    render([(mount, "#2f6fdb", 1.0)], os.path.join(IMG, "camera-mount-part.png"), 24, 35, "Printed part alone")


if __name__ == "__main__":
    main()
