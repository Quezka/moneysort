#!/usr/bin/env python3
"""Parametric 3D model of the spring-loaded pickup nozzle (replaces cad/nozzle_holder.stl).

    python cad/pickup_nozzle.py       # writes pickup_housing.stl, pickup_collar.stl, pickup_nozzle_assembly.stl

A pick-and-place style head: a hollow 4 mm brass tube slides in two printed bearings and is pushed down by a
light spring. When the cup touches the coin the tube retracts (TRAVEL mm) instead of the arm having to land
within a millimetre, and the contact force stays near 1 N. See docs/PUMP.md. Needs the same libraries as
cad/camera_mount.py (trimesh manifold3d numpy matplotlib).

Frame (mm): Z = the nozzle axis, s = distance DOWN from the underside of the camera-mount block (the flange
face). The housing is printed flange-down (like nozzle_holder.stl) and installed flipped, tube hanging down.
"""
import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_mount import (HOLDER_BOLTS_X, HOLDER_FLANGE, NUT_AF, NUT_T, BLOCK_T, PLATE_T, add, box, cyl,  # noqa: E402
                          hexprism, sub)

# ------------------------------------------------------------------ parts to buy ----
SHAFT_OD = 4.0           # brass tube 4.0 x 0.45 (hobby-shop tube); the cup's neck pushes on its lower end
SPRING_OD, SPRING_FREE, SPRING_SOLID = 7.0, 18.0, 6.0   # soft compression spring, 0.1-0.2 N/mm
# ---------------------------------------------------------------------- the design ----
TRAVEL = 9.0             # how far the tube can retract (the bellows cup gives 2-3 mm more)
GUIDE_BORE = SHAFT_OD + 0.4   # printed holes come out ~0.2 small: this leaves ~0.2 running clearance (ream with a 4 mm drill)
BEARING = 7.0            # length of each bearing; the span between them is what keeps the tube straight
COLLAR_T, COLLAR_OD = 6.0, 10.0
CAVITY_D = 12.0          # spring + collar chamber
TUBE_OD = 17.0
WINDOW_W = 7.0           # side opening on +Y to reach the collar's grub screw and fit the spring
PROTRUDE = 9.0           # shaft end below the housing: the cup's neck sits on it (cup tip = nominal tip height)
TOP_REST = -12.0         # shaft top at rest, above the plate top (s = -(BLOCK_T + PLATE_T)) so the hose clears it
GRUB = 3.0               # M3 grub screw clamps the collar to the tube

SPAN = BEARING + (TRAVEL + COLLAR_T + SPRING_SOLID + 1.0) + BEARING   # housing length below the flange face
CAV0 = BEARING                                  # cavity starts here (s)
CAV1 = SPAN - BEARING                           # ... and ends here: the collar rests on this wall at rest
OUT = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(os.path.dirname(OUT), "docs", "img")


def build_housing():
    """Print orientation: flange at z = 0..ft, tube above it. Installed it is the mirror image (tube down)."""
    fx, fy, ft = HOLDER_FLANGE
    z = lambda s: s            # print z == s measured from the flange face (z = 0)
    flange = box(-fx / 2, fx / 2, -fy / 2, fy / 2, 0, ft)
    tube = cyl("z", (0, 0, SPAN / 2), TUBE_OD / 2, SPAN)
    body = add(flange, tube)
    cuts = [cyl("z", (0, 0, SPAN / 2), GUIDE_BORE / 2, SPAN + 2, sections=40),                 # both bearings
            cyl("z", (0, 0, (CAV0 + CAV1) / 2), CAVITY_D / 2, CAV1 - CAV0, sections=48)]       # spring chamber
    for sx in (-1, 1):                                                                         # same bolts as the old holder
        cuts.append(cyl("z", (sx * HOLDER_BOLTS_X, 0, ft / 2), 1.7, ft + 2, sections=24))
        cuts.append(hexprism("z", (sx * HOLDER_BOLTS_X, 0, (NUT_T + 0.4) / 2 - 0.05), NUT_AF + 0.3, NUT_T + 0.4))
    # window on +Y over the whole chamber: grub-screw access and room to drop the spring in
    cuts.append(box(-WINDOW_W / 2, WINDOW_W / 2, 0, TUBE_OD, CAV0 + 0.5, CAV1 - 0.5))
    return sub(body, *cuts)


def build_collar():
    """Clamps on the tube with one M3 grub screw (from +Y); the nut is trapped in a pocket open to the spring side."""
    body = cyl("z", (0, 0, COLLAR_T / 2), COLLAR_OD / 2, COLLAR_T, sections=48)
    yn = SHAFT_OD / 2 + 2.2                     # nut centre, between the tube and the outside
    zn = COLLAR_T / 2
    cuts = [cyl("z", (0, 0, COLLAR_T / 2), SHAFT_OD / 2 - 0.05, COLLAR_T + 2, sections=32),    # snug: the grub does the clamping
            cyl("y", (0, COLLAR_OD / 4 + 1.5, zn), GRUB / 2 + 0.1, COLLAR_OD / 2 + 2, sections=24),
            hexprism("y", (0, yn, zn), NUT_AF + 0.3, NUT_T + 0.4),
            box(-(NUT_AF + 0.3) / 2, (NUT_AF + 0.3) / 2, yn - (NUT_T + 0.4) / 2, yn + (NUT_T + 0.4) / 2, zn, COLLAR_T + 1)]
    return sub(body, *cuts)


def shaft_mock():
    """Brass tube at rest: top at s = TOP_REST, bottom PROTRUDE below the housing. Housing frame: installed (flipped)."""
    length = (SPAN + PROTRUDE) - TOP_REST
    return cyl("z", (0, 0, -(TOP_REST + SPAN + PROTRUDE) / 2), SHAFT_OD / 2, length, sections=32), length


def installed(m):
    """Housing/collar from print orientation to installed: flange face at z = 0, tube hanging down (z = -s)."""
    m = m.copy()
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi, (1, 0, 0)))
    return m


def main():
    housing, collar = build_housing(), build_collar()
    shaft, shaft_len = shaft_mock()
    h_in = installed(housing)
    c_in = collar.copy()
    c_in.apply_translation((0, 0, -(CAV1 - COLLAR_T) - COLLAR_T))      # collar at rest: resting on the lower wall (s = CAV1)
    c_in.apply_transform(trimesh.transformations.rotation_matrix(0, (1, 0, 0)))
    assembly = trimesh.util.concatenate([h_in, c_in, shaft])
    housing.export(os.path.join(OUT, "pickup_housing.stl"))
    collar.export(os.path.join(OUT, "pickup_collar.stl"))
    assembly.export(os.path.join(OUT, "pickup_nozzle_assembly.stl"))

    inter = trimesh.boolean.intersection([h_in, shaft], engine="manifold")
    spring_rest = (CAV1 - COLLAR_T) - CAV0                     # spring length at rest
    spring_full = spring_rest - TRAVEL                         # ... with the tube fully retracted
    tip_below_block = SPAN + PROTRUDE
    assert spring_full >= SPRING_SOLID, "spring would go solid before the travel ends"
    assert spring_rest <= SPRING_FREE, "spring too short to be preloaded"
    assert SPRING_OD < CAVITY_D - 1.0 and SPRING_OD > SHAFT_OD + 1.0
    report = {
        "housing_watertight": bool(housing.is_watertight), "collar_watertight": bool(collar.is_watertight),
        "housing_len_below_flange_mm": SPAN, "housing_bounds_mm": [round(float(v), 1) for v in housing.bounds[1] - housing.bounds[0]],
        "housing_mass_g_est": round(housing.volume / 1000 * 1.27 * 0.7, 1),
        "tube_cut_length_mm": round(shaft_len, 1),
        "collar_from_tube_top_mm": round((CAV1 - COLLAR_T) - TOP_REST, 1),
        "spring_length_rest/full_mm": (spring_rest, spring_full),
        "spring_preload_mm": SPRING_FREE - spring_rest,
        "shaft_end_below_block_mm": tip_below_block,
        "shaft_top_above_plate_top_mm": round(-TOP_REST - (BLOCK_T + PLATE_T), 1),
        "housing_shaft_overlap_mm3": float(inter.volume) if len(inter.vertices) else 0.0,
    }
    for k, v in report.items():
        print(f"{k:34s} {v}")
    return report


if __name__ == "__main__":
    main()
