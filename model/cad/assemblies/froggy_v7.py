"""Froggy v7, "base unit + net cage": the complete machine.

Datums: Z 0 = deck rail top, X and Y zero on the rotor axis, rotor axis Z.

frame   : 300 x 300 deck of six 15x15 rails (perimeter Z -15..0, cross rails at X
          +-45), a 300 x 300 bottom frame (Z -110..-95) and four 80 mm legs at the
          corners (Z -95..-15). Tovi 3-way corner brackets in all eight plinth
          corners; Tovi chocolate gussets lying flat in the deck plane in the outer
          bays where the cross rails meet the perimeter rails; Leon Matthews T plates
          on the deck top over those junctions; idig3D corner plates under the deck
          at the same junctions on the middle-bay side.
feet    : four 14 x 14 x 10 placeholder feet under the bottom frame corners.
plinth  : 3 mm black sheets: four side panels captured 4 mm deep in the leg and rail
          slots, notched to the silhouettes of the 3-way brackets and idig plates (no
          see-through windows, panel_coverage), 1 mm slot-entry chamfers, vent slots
          on the left and right panels; a bottom panel on
          the bottom frame (46 mm hole under the motor), three deck infill panels in
          the deck rails' side slots (grommets G1, G2, 30 mm coupling hole);
          C47 cable clips on the bottom rails mark the cable routes.
electrical: CL42T, LRS-50-24 and the controller carrier (Pico, buck, delay relay)
          on the bottom panel, panel-mount e-stop (box behind the front panel,
          collar and head proud on the yellow fp7_estop_legend ring), rear-flange
          IEC inlet and USB-C flush in the back panel; Qi TX board + hall sensor in the pedestal; the
          rotor's Qi RX, Pi, LEDs, LED driver and two magnets.
drive_base: v6 motor, plate and jaw coupling under the deck; fp7_pedestal with the
          v6 608 pair on the cross rails.
rotor   : the v6 rotor stack plus the second magnet, the LED driver, fp7_wire_spine
          at -Y and the two red-eyed tree frogs outside the drum.
cage    : stationary: four fp7_cage_seat segments on the deck, the bottom Keiran
          220 hoop pair in their groove (Z 3..15), the top pair at Z 252..264 held by
          three fp7_cage_strut, a mesh sleeve and a top mesh between them.

Install steps (driver_path_clear counts bodies installed at or before a joint):
0 frame, feet; 1 motor, plate, coupling, clips; 2 pedestal and its stack; 3 spacer,
hub; 4 carrier and its electrics, spine; 5 camera carrier, camera, Pi; 6 drum, disc,
ring, frogs; 7 panels and plinth electrics; 8 cage.
"""
import copy
import itertools
import math
import os
from functools import lru_cache

import numpy as np
from build123d import Align, Box, Compound, Cylinder, Location, Plane, Solid, extrude, import_step

from assemblies.drum_base import R_MOTOR, _motor_solids, motor_length
from assemblies.frame import _location
from lib import hardware as HW
from lib import profile_1515 as P
from lib import represent as RP
from lib import vendor as V
from lib.assembly import Assembly
from lib.joints import _seat, bolt_joint, hex_nut_joint, insert_joint, tapped_joint
from parts import datums as D0
from parts import fp6_datums as D6
from parts import (fp6_drum_carrier, fp6_motor_plate, fp7_cage_seat, fp7_cage_strut, fp7_controller_carrier,
                   fp7_estop_legend, fp7_pedestal, fp7_wire_spine, fp_camera_carrier, fp_disc_clamp_ring, s_drum_tube, s_top_disc)

W = P.W
HALF = 150.0                    # 300 x 300 outer
RC = HALF - W / 2               # 142.5 perimeter rail centre
RIN = HALF - W                  # 135 perimeter rail inner face
CROSS_X = 45.0
BOT_Z0, BOT_Z1 = -110.0, -95.0
LEG_Z0, LEG_Z1 = BOT_Z1, -W
FOOT_H, FOOT_W = 10.0, 14.0
PANEL_T = 3.0
OV = "opensource/"
THREEWAY_DIR = OV + "tovi-1515-3way-corner-bracket"
THREEWAY_STEP = os.path.join(V.VENDOR, THREEWAY_DIR, "3-Way Corner Bracket.step")
CHOC_DIR = OV + "tovi-1515-chocolate-corner-bracket"
CHOC_STEP = os.path.join(V.VENDOR, CHOC_DIR, "Chocolate Styled Corner Bracket.step")
IDIG_DIR = OV + "idig3d-1515-corner-plate"
IDIG_STL = os.path.join(V.VENDOR, IDIG_DIR, "1515_short_-_base_M3.stl")
IDIG_SOCKET_TOP = 7.3
IDIG_HOLES = [(0.0, 7.5), (0.0, 26.5), (19.0, 7.5)]     # (x, z) in the STL, axis +Y, plate back face y -7.7
LEON_DIR = OV + "leonmatthews-openbeam-t-bracket"
LEON_STL = os.path.join(V.VENDOR, LEON_DIR, "OpenBeam-T-Bracket.stl")
LEON_REAM_R = 1.7
CLIP_DIR = OV + "c47-1515-cable-clips"
CLIP_STL = os.path.join(V.VENDOR, CLIP_DIR, "1515 clip_A2_100.stl")
HOOP_DIR = OV + "keiran-embroidery-hoop-220"
HOOP_IN_STL = os.path.join(V.VENDOR, HOOP_DIR, "EmbroideryHoop_Inner_220mm_M5.stl")
HOOP_OUT_STL = os.path.join(V.VENDOR, HOOP_DIR, "EmbroideryHoop_Outter_220mm_M5.stl")
HOOP_LUG_DEG = 225.0           # clamp lug at a seat-segment joint
HOOP_Z = {"bottom": 3.0, "top": 252.0}
MESH_R = (108.2, 109.2)          # 1.0 wall: a 0.5 shell tessellated badly in Fusion
MESH_Z = (15.0, 252.0)
MESH_TOP_R, MESH_TOP_Z = 105.9, (264.0, 264.5)
STRUT_ANGLES = (90.0, 210.0, 330.0)
FROG_DIR = OV + "3square-tree-frog"
# decimated for Fusion (2000 -> 600 and 300 -> 68 triangles, 90 faces after the body cut, trimesh quadric) from the 2k / 300 files beside them
FROG_BODY_STL = os.path.join(V.VENDOR, FROG_DIR, "agalychnis-eyeless-decimated-600.stl")
FROG_EYES_STL = os.path.join(V.VENDOR, FROG_DIR, "agalychnis-eyes-decimated-70.stl")
HUB_STEP = os.path.join(V.VENDOR, "coupling/gobilda-1310-0016-0008-hyper-hub-8mm.step")
HUB = "v_gobilda_1310_hub_8mm"
COUPLING_STEP = os.path.join(V.VENDOR, "coupling/jaw-coupling-bore5-to-8-envelope.step")
PI_STEP = os.path.join(V.VENDOR, "electronics/rpi-zero-2w.step")
CL42T_STEP = os.path.join(V.VENDOR, "drive/stepperonline-cl42t-v41-driver.step")
ESTOP_STEP = os.path.join(V.VENDOR, "hardware/estop-22mm-mushroom.step")
MOTOR = "v_nema17_17hs19_2004d_e1k"
TUBE_OR = D0.DRUM_OD / 2
DISC_TOP_Z = D6.DRUM_TOP_Z + D0.ACRYLIC_T
FROG_CLEAR = 0.3
FROG_WALL_Z = 186.0            # bbox centre; head 4.07 under the clamp ring underside (Z 224.31)
FROG_DISC_SPARE = 0.5          # disc frog outermost vertex inside the bezel limit, mm
FROG_RPM = (400, 600)          # load-test speeds for the foot load note
FROG_RING_GAP = 4.0            # wall frog top to clamp ring underside, minimum
FROG_NET_GAP = 8.0             # wall frog to the net sleeve inner radius, minimum
FROG_MESH_TOP_GAP = 10.0       # disc frog top to s_mesh_top underside, minimum
# Frog mesh native frame (measured): head (eyes centroid (0, -22.5, 2.5)) at local -Y, up +Z, belly -Z.
# plinth electrics (bottom panel top Z -92)
FLOOR_Z = BOT_Z1 + PANEL_T          # -92
CL42T_X0 = -116.0
CL42T_EAR_T = 5.0
PSU_BOX = (34.0, 116.0, 5.0, 104.0, 30.0)      # x0, x1, y0, y1, h
ESTOP_X, PANEL_DEV_Z, ESTOP_Z = 75.0, -55.0, -57.0
IEC = (80.0, 27.4, 31.0, 112.0, 144.0)          # x, w, h, y0, y1: C14 + fuse drawer body, front flush
USB = (-60.0, 20.0, 12.0, 124.0, 144.0)          # front flush with the back panel's outer face
G1, G2, G_D = (-24.0, -30.0), (100.0, 0.0), 8.0


def _sg(v):
    return "p" if v > 0 else "n"


def _c(sx, sy):
    return f"{_sg(sx)}x{_sg(sy)}y"


def _R(cols):
    R = np.column_stack([np.array(c, float) for c in cols])
    assert abs(np.linalg.det(R) - 1) < 1e-9, R
    return R


def _rail_hit(A, cands, p):
    return next((m for m in cands if all(A.bbox(m)[0][k] - 1e-6 <= p[k] <= A.bbox(m)[1][k] + 1e-6
                                         for k in range(3))), None)


def _box(x0, x1, y0, y1, z0, z1):
    return Box(x1 - x0, y1 - y0, z1 - z0, align=(Align.MIN, Align.MIN, Align.MIN)).moved(Location((x0, y0, z0)))


def _solid(s):
    sols = s.solids()
    assert len(sols) == 1, f"expected one solid, got {len(sols)}"
    return sols[0]


# ------------------------------------------------------------------ vendor bracket bases

@lru_cache(maxsize=None)
def _open_counterbores(path):
    """Vendor STEP bracket with every M3 counterbore opened to 7.2 from the end of the
    3.4 clearance hole outward (hex-key driver clearance, and a flat head seat)."""
    s = V._load(path)
    cuts = []
    holes = V.cyl_holes(s, 1.6, 1.8)
    cbs = V.cyl_holes(s, 3.0, 3.55)
    done = set()
    for h in holes:
        ax, p0 = h["axis"], h["p0"]
        key = (tuple(np.round(p0 - ax * (p0 @ ax), 2)), tuple(np.round(np.abs(ax), 3)))
        if key in done:
            continue
        done.add(key)
        hv = [(np.array(tuple(v)) - p0) @ ax for v in h["face"].vertices()]
        a0, a1 = min(hv), max(hv)
        side = []
        for c in cbs:
            if abs(abs(c["axis"] @ ax) - 1) > 1e-4 or np.linalg.norm(np.cross(c["p0"] - p0, ax)) > 0.05:
                continue
            cv = [(np.array(tuple(v)) - p0) @ ax for v in c["face"].vertices()]
            side.append((min(cv), max(cv)))
        if not side:
            continue
        hi_side = [b for b in side if b[0] >= a1 - 1.0]
        lo_side = [b for b in side if b[1] <= a0 + 1.0]
        if hi_side:
            z0, z1 = a1, max(b[1] for b in hi_side) + 1.0
        elif lo_side:
            z0, z1 = min(b[0] for b in lo_side) - 1.0, a0
        else:
            continue
        o = p0 + ax * z0
        cuts.append(Plane(origin=tuple(o), z_dir=tuple(ax)) * Cylinder(3.6, z1 - z0, align=(Align.CENTER, Align.CENTER,
                                                                                             Align.MIN)))
    for c in cuts:
        s = s - c
    return _solid(s)


def _holes(solid):
    out, seen = [], set()
    for h in V.bracket_holes(solid):
        k = tuple(np.round(h["centre"], 2))
        if k not in seen:
            seen.add(k)
            out.append(h)
    return out


def _bracket_joints(A, name, mates, prefix, module, screw="M3x8", step=0):
    for i, h in enumerate(_holes(A[name])):
        at, axis = h["centre"], h["axis"]
        rail = _rail_hit(A, mates, at + axis * 1.0)
        tag = A.meta_role.get(rail, rail.replace("s_rail_1515_", "r"))
        bolt_joint(A, [name, rail], at, axis, screw=screw, nut="slot_nut", name=f"{prefix}_{tag}_{i}",
                   module=module, evaluate=False, step=step)


# ------------------------------------------------------------------ frame

def _frame(A):
    A.meta_role = {}
    z = -W / 2
    zb = (BOT_Z0 + BOT_Z1) / 2
    specs = [  # name role, length, axis, centre
        ("deck_x_back", 2 * HALF, "x", (0, RC, z)), ("deck_x_front", 2 * HALF, "x", (0, -RC, z)),
        ("deck_y_right", 2 * RIN, "y", (RC, 0, z)), ("deck_y_left", 2 * RIN, "y", (-RC, 0, z)),
        ("cross_right", 2 * RIN, "y", (CROSS_X, 0, z)), ("cross_left", 2 * RIN, "y", (-CROSS_X, 0, z)),
        ("bot_x_back", 2 * HALF, "x", (0, RC, zb)), ("bot_x_front", 2 * HALF, "x", (0, -RC, zb)),
        ("bot_y_right", 2 * RIN, "y", (RC, 0, zb)), ("bot_y_left", 2 * RIN, "y", (-RC, 0, zb)),
    ]
    for sx in (1, -1):
        for sy in (1, -1):
            specs.append((f"leg_{_c(sx, sy)}", LEG_Z1 - LEG_Z0, "z", (sx * RC, sy * RC, (LEG_Z0 + LEG_Z1) / 2)))
    names = {}
    for n, (role, L, ax, c) in enumerate(specs, 1):
        s = P.rail(L, ax, c, n=n)
        A.add(s.label, s, "rail", "frame", length_mm=L, axis=ax, centre=c, role=role)
        names[role] = s.label
        A.meta_role[s.label] = role
    A.meta_rails = names
    return names


def _threeway(A, R_, lvl, sx, sy):
    """Tovi 3-way corner bracket in the inside corner of a plinth corner."""
    base = _open_counterbores(THREEWAY_STEP)
    d_loc = np.column_stack([(1, 0, 0), (0, -1, 0), (0, 0, 1)]).astype(float)
    vz = 1.0 if lvl == "bottom" else -1.0
    w_dirs = [np.array((-sx, 0, 0.0)), np.array((0, -sy, 0.0)), np.array((0, 0, vz))]
    R = None
    for perm in itertools.permutations(range(3)):
        Wm = np.column_stack([w_dirs[i] for i in perm])
        cand = Wm @ d_loc.T
        if np.linalg.det(cand) > 0.5:
            R = cand
            break
    zc = (BOT_Z0 + BOT_Z1) / 2 if lvl == "bottom" else -W / 2
    cube_w = np.array([sx * RC, sy * RC, zc])
    t = cube_w - R @ np.array([-7.5, -7.5, -7.5])
    nm = f"v_tovi_3way_{lvl}_{_c(sx, sy)}"
    s = copy.deepcopy(base).moved(_location(R, t))
    if lvl == "bottom":
        mates = [R_["bot_x_back" if sy > 0 else "bot_x_front"], R_["bot_y_right" if sx > 0 else "bot_y_left"],
                 R_[f"leg_{_c(sx, sy)}"]]
    else:
        mates = [R_["deck_x_back" if sy > 0 else "deck_x_front"], R_["deck_y_right" if sx > 0 else "deck_y_left"],
                 R_[f"leg_{_c(sx, sy)}"]]
    for m in mates:                   # the bracket's 0.4 corner beads assume radiused rail edges: trim them
        s = s - A[m]
    s = _solid(s)
    A.add(nm, s, "bracket", "frame", mates=mates, step=0)
    A.vendor.append(V.vendor_entry(nm, THREEWAY_DIR))
    _bracket_joints(A, nm, mates, f"j_3way_{lvl}_{_c(sx, sy)}", "frame")


def _choc_side(sx, sy):
    """+1 / -1: the X direction of the bay the chocolate gusset sits in (the idig plate takes the other side)."""
    return sx if sx * sy < 0 else -sx


def _chocolate(A, R_, sx, sy):
    """Tovi chocolate gusset lying flat in the deck plane (Z -15..0) in an inside corner
    where the cross rail at X sx*45 meets the perimeter rail at Y sy*142.5."""
    base = _open_counterbores(CHOC_STEP)
    o = _choc_side(sx, sy)
    rx, rz = np.array((o, 0, 0.0)), np.array((0, -sy, 0.0))
    ry = np.cross(rz, rx)
    R = _R([rx, ry, rz])
    t = np.array([sx * CROSS_X + o * W / 2, sy * RIN, 0.0 if ry[2] > 0 else -W])
    nm = f"v_tovi_chocolate_{_c(sx, sy)}"
    mates = [R_["cross_right" if sx > 0 else "cross_left"], R_["deck_x_back" if sy > 0 else "deck_x_front"]]
    A.add(nm, copy.deepcopy(base).moved(_location(R, t)), "bracket", "frame", mates=mates, step=1)
    A.vendor.append(V.vendor_entry(nm, CHOC_DIR))
    for i, h in enumerate(_holes(A[nm])):
        at, axis = h["centre"], h["axis"]
        loc = np.linalg.solve(R, at - t)
        if (abs(axis @ rz) > 0.5 and abs(loc[0] - 9.0) < 0.5) or (abs(axis @ rx) > 0.5 and abs(loc[2] - 9.0) < 0.5):
            continue          # the two screws nearest the corner: their key paths run over the other rail's screw heads
        rail = _rail_hit(A, mates, at + axis * 1.0)
        bolt_joint(A, [nm, rail], at, axis, screw="M3x8", nut="slot_nut",
                   name=f"j_chocolate_{_c(sx, sy)}_{A.meta_role[rail]}_{i}", module="frame", evaluate=False, step=1)


def _leon(A, R_, sx, sy):
    F = V.stl_plate_features(LEON_STL)
    t = F["slab_top"]
    R = _R([(sy, 0, 0), (0, -sy, 0), (0, 0, -1)])
    origin = np.array([sx * CROSS_X, sy * RC, 0.0]) - R @ np.array([0.0, 7.5, t])
    nm = f"v_leon_openbeam_t_plate_{_c(sx, sy)}"
    mates = [R_["deck_x_back" if sy > 0 else "deck_x_front"], R_["cross_right" if sx > 0 else "cross_left"]]
    A.add(nm, V.stl_plate(LEON_STL, label=nm, location=_location(R, origin), ream_r=LEON_REAM_R, tol=0.25), "bracket", "frame",
          mates=mates, step=0)
    A.vendor.append(_rep_entry(nm, LEON_DIR, "Leon Matthews STL"))
    ax = R @ np.array([0, 0, 1.0])            # rail side is local z = t: screws point down
    for h in F["holes"]:
        x, y = h["centre"]
        at = R @ np.array([x, y, t]) + origin
        host = _rail_hit(A, mates, at + ax * 1.0)
        bolt_joint(A, [nm, host], at, ax, screw="M3x8", nut="slot_nut",
                   name=f"j_t_plate_{_c(sx, sy)}_{A.meta_role[host]}_{int(round(x)):02d}_{int(round(y)):02d}",
                   module="frame", evaluate=False, contact="faces", step=0, tongue_top=1.0)


def _rep_entry(nm, rel_dir, what):
    e = V.vendor_entry(nm, rel_dir)
    e["note"] = f"representative solid derived from the {what}"
    return e


def _idig_rep():
    """Representative idig3D plate from the STL's own sections: the plate
    outline (Y -11.5..-7.7), the L socket walls (Y -7.7..7.3) with their
    notches at Y -1.6..1.9 as the narrowest (Y 0) section."""
    m = V.stl_mesh(IDIG_STL)

    def loops(y):
        return [RP.simplify(c, 0.1) for c in RP.section_loops(m, 1, y) if not RP.circle(c)]
    plate = RP.prism(max(loops(-9.4), key=lambda c: abs(RP.area(c))), -11.5, -7.7, axis=1)
    wall = loops(-4.0)[0]
    s = plate + RP.prism(wall, -7.7, -1.6, axis=1) + RP.prism(wall, 1.9, 7.3, axis=1)
    for c in loops(0.0):
        s = s + RP.prism(c, -1.6, 1.9, axis=1)
    return _solid(s)


@lru_cache(maxsize=None)
def _idig_base():
    """idig3D 1515 short corner plate as a solid, its three plate holes re-cut as
    true 3.44 cylinders (the STL facets inscribe the printed 3.4 hole)."""
    s = _idig_rep()
    s = s - _box(-50, 50, IDIG_SOCKET_TOP, 50, -50, 80)     # socket top trimmed 0.4 so it sits flush with the deck top
    for x, z in IDIG_HOLES:
        s = s - Plane(origin=(x, -13.0, z), z_dir=(0, 1, 0)) * Cylinder(1.72, 6.5, align=(Align.CENTER, Align.CENTER,
                                                                                           Align.MIN))
    return _solid(s)


def _idig(A, R_, sx, sy):
    """idig3D corner plate under the deck at the cross-rail junction, socket in the
    middle-bay inside corner, 3 x M3 x 8 up into the rails' bottom slots."""
    d = -_choc_side(sx, sy)                              # = sy: the only side where the plate lands under the deck
    R = _R([(d, 0, 0), (0, 0, 1), (0, -sy, 0)])          # plate x along the perimeter rail, z along the cross rail
    t = np.array([sx * CROSS_X, sy * RC, -W]) - R @ np.array([0.0, -7.7, 7.5])
    nm = f"v_idig3d_corner_plate_{_c(sx, sy)}"
    mates = [R_["deck_x_back" if sy > 0 else "deck_x_front"], R_["cross_right" if sx > 0 else "cross_left"]]
    A.add(nm, copy.deepcopy(_idig_base()).moved(_location(R, t)), "bracket", "frame", mates=mates, step=0)
    A.vendor.append(_rep_entry(nm, IDIG_DIR, "idig3D STL"))
    up = np.array([0, 0, 1.0])
    for x, z in IDIG_HOLES:
        at = R @ np.array([x, -7.7, z]) + t
        seat = R @ np.array([x, -11.5, z]) + t
        host = _rail_hit(A, mates, at + up)
        bolt_joint(A, [nm, host], at, up, screw="M3x8", nut="slot_nut",
                   name=f"j_idig_{_c(sx, sy)}_{A.meta_role[host]}_{int(x)}_{int(z)}", module="frame", evaluate=False,
                   contact="faces", seat=seat, step=0)


@lru_cache(maxsize=None)
def _clip_base():
    """C47 1515 cable clip A2, its snap barbs trimmed to the 3.2 slot opening for
    the first 1.45 mm (they flex through the lip on install)."""
    loop = next(c for c in RP.section_loops(V.stl_mesh(CLIP_STL), 2, 2.5) if not RP.circle(c))
    s = RP.prism(RP.simplify(loop, 0.1), 0.0, 10.0, axis=2)        # representative: the barbed profile, full height
    for sy in (1, -1):
        s = s - _box(-1.45, 0.0, 1.6 if sy > 0 else -3.0, 3.0 if sy > 0 else -1.6, -1, 11)
    return _solid(s)


def _clips(A, R_):
    """Eight cable clips on the inner faces of the bottom rails (cable routes)."""
    k = 1
    zc = (BOT_Z0 + BOT_Z1) / 2
    for role, n, along in (("bot_x_front", (0, 1, 0), (1, 0, 0)), ("bot_x_back", (0, -1, 0), (1, 0, 0)),
                           ("bot_y_left", (1, 0, 0), (0, 1, 0)), ("bot_y_right", (-1, 0, 0), (0, 1, 0))):
        n, a = np.array(n, float), np.array(along, float)
        if np.linalg.det(np.column_stack([n, (0, 0, 1), a])) < 0:
            a = -a
        R = _R([n, (0, 0, 1), a])
        face = -n * RIN
        for s in (-60.0, 60.0):
            org = face + np.array(along, float) * s + np.array([0, 0, zc]) - a * 5.0
            nm = f"v_c47_cable_clip_{k}"
            A.add(nm, copy.deepcopy(_clip_base()).moved(_location(R, org)), "bracket", "plinth", mates=[R_[role]],
                  step=1)
            A.vendor.append(_rep_entry(nm, CLIP_DIR, "C47 STL"))
            k += 1


def _feet(A, R_):
    for sx in (1, -1):
        for sy in (1, -1):
            c = _c(sx, sy)
            x, y = sx * RC, sy * RC
            f = Box(FOOT_W, FOOT_W, FOOT_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
            f = f - Cylinder(1.7, 40) - Cylinder(3.6, 8.0)    # 7.2 counterbore 4 deep from the bottom
            f = _solid(f).moved(Location((x, y, BOT_Z0 - FOOT_H)))
            nm = f"v_foot_placeholder_{c}"
            A.add(nm, f, "bracket", "feet", step=0, note="placeholder foot, PETG; Sawyer picks the real foot later")
            A.vendor.append(V.manifest_entry(nm, "v_foot_placeholder"))
            rail = R_["bot_x_back" if sy > 0 else "bot_x_front"]
            bolt_joint(A, [nm, rail], (x, y, BOT_Z0), (0, 0, 1), screw="M3x10", nut="slot_nut", name=f"j_foot_{c}_rail",
                       module="feet", evaluate=False, contact="faces", seat=(x, y, BOT_Z0 - FOOT_H + 4.0), step=0,
                       hw_tag=f"j_leveller_{c}_rail")


# ------------------------------------------------------------------ plinth panels
#
# Side panels are 3 mm laser-cut sheets in the slot plane of the legs and rails (centre RC), engaged
# SLOT_ENG deep in the legs' slots and in the bottom rails' top slots and deck rails' bottom slots.
# Each panel is drawn as a 2D profile in (h, z) as seen from outside (h to the viewer's right, z up)
# and extruded 3 mm outward from its inner face.
# Notches: every frame bracket, screw and nut that crosses the panel slab (the Tovi 3-way corners, the
# idig3D plates under the deck rail and their screw heads) is cut by its own silhouette in the slab
# grown by NOTCH_C (arcs at the panel's inside corners); at the 3-way corners the cut is also closed by
# NOTCH_CLOSE so no thin panel finger is left between the bracket's flanges (those gaps are backed by
# the bracket's body). panel_coverage() measures what stays see-through.

SIDES = {"front": ((0, -1, 0), (1, 0, 0)), "back": ((0, 1, 0), (-1, 0, 0)),
         "right": ((1, 0, 0), (0, 1, 0)), "left": ((-1, 0, 0), (0, -1, 0))}    # outward normal, h direction
PANEL_IN = RC - PANEL_T / 2         # 141: panel inner face distance from the axis
SLOT_ENG = 4.0
NOTCH_C, NOTCH_CLOSE, SLOT_CHAMFER = 0.25, 2.5, 1.0
ESTOP_HOLE_D, ESTOP_KEY_W, ESTOP_KEY_R = 22.3, 3.2, 24.1 - 22.3 / 2      # IEC 60947-5-1 22 mm hole + key
LEGEND_OD, LEGEND_T = 45.0, 3.0
DEV_CLEAR, DEV_R = 0.3, 1.5
IEC_PITCH, IEC_FLANGE = 40.0, (50.0, 34.0, 3.0)    # rear flange w, h, t (behind the panel)
VENT_W, VENT_L, VENT_PITCH, VENT_N, VENT_Z = 3.0, 40.0, 6.0, 15, (-88.0, -48.0)
VENT_H = {"right": 55.0, "left": 0.0}               # slot-row centre h: beside the PSU (+X) and the CL42T (-X)
VENT_GAP = 3.0


def _plane(side):
    n, h = SIDES[side]
    return Plane(origin=tuple(np.array(n, float) * PANEL_IN), x_dir=h, z_dir=n)


def _hv(side, p):
    """World point -> (h, z) on the panel of `side`."""
    return float(np.array(SIDES[side][1], float) @ np.asarray(p, float)), float(p[2])


def _rrect(h0, h1, z0, z1, r):
    from build123d import RectangleRounded
    f = RectangleRounded(h1 - h0, z1 - z0, r).faces()[0] if r > 0 else _rect(h0, h1, z0, z1)
    return f.moved(Location(((h0 + h1) / 2, (z0 + z1) / 2, 0))) if r > 0 else f


def _rect(h0, h1, z0, z1):
    from build123d import Rectangle
    return Rectangle(h1 - h0, z1 - z0).faces()[0].moved(Location(((h0 + h1) / 2, (z0 + z1) / 2, 0)))


def _circ(h, z, r):
    from build123d import Circle
    return Circle(r).faces()[0].moved(Location((h, z, 0)))


def _slot_keepouts(A, side):
    """Silhouettes (2D faces in (h, z)) of every frame bracket, screw and nut where it crosses the panel
    slab (+-0.5), from the actual solids: brackets by their sections at the slab faces and mid plane
    (they are prismatic through the slab), screws and nuts by their projected bbox (axis-aligned
    cylinders and blocks, whose widest section is on their axis). Returns [(name, face, (h0, h1, z0, z1))]."""
    n = np.array(SIDES[side][0], float)
    pl = _plane(side)
    lo, hi = PANEL_IN - 0.5, PANEL_IN + PANEL_T + 0.5
    k = int(np.argmax(np.abs(n)))
    sgn = n[k]
    out = []
    for nm, p in A.parts.items():
        if p.module != "frame" or p.kind not in ("bracket", "screw", "nut"):
            continue
        b0, b1 = A.bbox(nm)
        a0, a1 = sorted((sgn * b0[k], sgn * b1[k]))
        if a1 < lo or a0 > hi:
            continue
        faces = []
        if p.kind == "bracket":
            for d in (lo, (lo + hi) / 2, hi):
                rng = [(-400.0, 400.0)] * 3
                rng[k] = tuple(sorted((sgn * (d - 0.01), sgn * (d + 0.01))))
                sec = A[nm] & _box(rng[0][0], rng[0][1], rng[1][0], rng[1][1], rng[2][0], rng[2][1])
                if sec is None:
                    continue
                for f in sec.faces():
                    if abs(abs(np.array(tuple(f.normal_at())) @ n) - 1) < 1e-6 and \
                            abs(sgn * tuple(f.center())[k] - (d + 0.01)) < 1e-3:
                        g = pl.to_local_coords(f)
                        faces.append(g.moved(Location((0, 0, -g.center().Z))))
        else:
            rng = [(-400.0, 400.0)] * 3
            rng[k] = tuple(sorted((sgn * lo, sgn * hi)))
            cut = A[nm] & _box(rng[0][0], rng[0][1], rng[1][0], rng[1][1], rng[2][0], rng[2][1])
            if cut is None or not cut.solids():
                continue
            bb = cut.bounding_box()
            hs = [_hv(side, (x, y, 0))[0] for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y)]
            faces.append(_rect(min(hs), max(hs), bb.min.Z, bb.max.Z))
        if not faces:
            continue
        f = faces[0].fuse(*faces[1:]).clean() if len(faces) > 1 else faces[0]
        fb = f.bounding_box()
        if fb.min.Y > LEG_Z1 + SLOT_ENG or fb.max.Y < BOT_Z1 - SLOT_ENG:
            continue                  # above or below the sheet (deck-top T plates)
        out.append((nm, f, (fb.min.X, fb.max.X, fb.min.Y, fb.max.Y)))
    return out


def _outline(side):
    """Panel outline: the bay plus SLOT_ENG into the leg and rail slots; the four outer corners (where a
    leg slot and a rail slot would meet under the leg) clipped at 45 deg. The 3-way brackets are taken
    out by the silhouette notch."""
    from build123d import Polygon
    e, zb, zt = RIN + SLOT_ENG, BOT_Z1 - SLOT_ENG, LEG_Z1 + SLOT_ENG
    hc = RIN - SLOT_ENG
    pts = [(-hc, zb), (hc, zb), (e, BOT_Z1 + SLOT_ENG), (e, LEG_Z1 - SLOT_ENG), (hc, zt), (-hc, zt),
           (-e, LEG_Z1 - SLOT_ENG), (-e, BOT_Z1 + SLOT_ENG)]
    return Polygon(*pts, align=None).faces()[0], pts


def _profile(A, side):
    """2D panel profile (h, z) and its report: notches, cutouts, vents."""
    from build123d import Kind, offset
    face, pts = _outline(side)
    keep = _slot_keepouts(A, side)
    rep = dict(side=side)
    # notch = keep-out silhouettes grown by NOTCH_C + NOTCH_CLOSE, shrunk by NOTCH_CLOSE (a closing):
    # NOTCH_C clearance everywhere, R NOTCH_C at the panel's inside corners, and no panel finger
    # narrower than 2 x NOTCH_CLOSE between neighbouring keep-outs (those gaps are backed by the bracket)
    for corner in (True, False):      # 3-way corners: closed (gaps backed by the bracket); idig: plain offset
        fs = [g for nm, f, _ in keep if ("3way" in nm) == corner for g in f.faces()]
        if not fs:
            continue
        U = fs[0].fuse(*fs[1:]).clean() if len(fs) > 1 else fs[0]
        close = NOTCH_CLOSE if corner else 0.0
        grown = [offset(f, NOTCH_C + close, kind=Kind.ARC).faces()[0] for f in U.faces()]
        G = grown[0].fuse(*grown[1:]).clean() if len(grown) > 1 else grown[0]
        cut = [offset(f, -close, kind=Kind.ARC) for f in G.faces()] if close else [G]
        for c in cut:
            for f in c.faces():
                face = face - f
        face = max(face.faces(), key=lambda q: q.area)       # drop slivers cut off in the slots
    rep["notched_parts"] = sorted({nm for nm, _, _ in keep if A.parts[nm].kind == "bracket"})
    rep["notch_keepouts"] = len(keep)
    # the corners where the sheet enters a slot (on the slot-engaged outer edges): 1 x 45 deg chamfer
    e, zb, zt = RIN + SLOT_ENG, BOT_Z1 - SLOT_ENG, LEG_Z1 + SLOT_ENG
    on_edge = [(v.X, v.Y) for v in face.vertices()
               if abs(abs(v.X) - e) < 0.01 or abs(v.Y - zb) < 0.01 or abs(v.Y - zt) < 0.01]
    from build123d import Polygon
    sv, tris = [], []
    for x, y in on_edge:               # cut the corner with the triangle on its two edges (convex corners only)
        p0 = np.array((x, y))
        dirs = []
        for ed in face.edges():
            a, b = np.array(tuple(ed.start_point()))[:2], np.array(tuple(ed.end_point()))[:2]
            if np.linalg.norm(a - p0) < 1e-4:
                dirs.append(np.array(tuple(ed.tangent_at(0)))[:2])
            elif np.linalg.norm(b - p0) < 1e-4:
                dirs.append(-np.array(tuple(ed.tangent_at(1)))[:2])
        if len(dirs) != 2:
            continue
        q1, q2 = p0 + SLOT_CHAMFER * dirs[0], p0 + SLOT_CHAMFER * dirs[1]
        mid = (q1 + q2) / 2
        if not face.is_inside((mid[0], mid[1], 0)):
            continue                   # an inside corner: no relief needed
        tris.append(Polygon(tuple(p0), tuple(q1), tuple(q2), align=None).faces()[0])
        sv.append((round(x, 2), round(y, 2)))
    for t_ in tris:
        face = face - t_
    face = max(face.faces(), key=lambda q: q.area)
    rep["slot_corner_chamfers"] = len(sv)
    # device cutouts and vents
    holes = []
    if side == "front":
        h, z = _hv(side, (ESTOP_X, 0, ESTOP_Z))
        key = _rrect(h - ESTOP_KEY_W / 2, h + ESTOP_KEY_W / 2, z, z + ESTOP_KEY_R, 0) + \
            _circ(h, z + ESTOP_KEY_R - ESTOP_KEY_W / 2, ESTOP_KEY_W / 2)
        holes.append(_circ(h, z, ESTOP_HOLE_D / 2) + key)
        rep["estop_hole"] = dict(d=ESTOP_HOLE_D, key_w=ESTOP_KEY_W, key_to_centre=round(ESTOP_KEY_R, 2),
                                 centre_hz=[h, z])
    if side == "back":
        for nm, (x, w, hh) in (("iec", IEC[:3]), ("usb", USB[:3])):
            h, _ = _hv(side, (x, 0, 0))
            z = PANEL_DEV_Z
            cw, ch = w + 2 * DEV_CLEAR, hh + 2 * DEV_CLEAR
            holes.append(_rrect(h - cw / 2, h + cw / 2, z - ch / 2, z + ch / 2, DEV_R))
            rep[f"{nm}_cutout"] = dict(w=round(cw, 2), h=round(ch, 2), r=DEV_R, centre_hz=[h, z])
        h, _ = _hv(side, (IEC[0], 0, 0))
        for s in (-1, 1):
            holes.append(_circ(h + s * IEC_PITCH / 2, PANEL_DEV_Z, 1.7))
        rep["iec_holes"] = dict(d=3.4, pitch=IEC_PITCH)
    if side in VENT_H:
        hc = VENT_H[side]
        for i in range(VENT_N):
            h = hc + (i - (VENT_N - 1) / 2) * VENT_PITCH
            holes.append(_rrect(h - VENT_W / 2, h + VENT_W / 2, VENT_Z[0], VENT_Z[1], VENT_W / 2 - 1e-3))
        a1 = VENT_W * (VENT_L - VENT_W) + math.pi * (VENT_W / 2) ** 2
        rep["vents"] = dict(n=VENT_N, w=VENT_W, l=VENT_L, pitch=VENT_PITCH, open_mm2=round(VENT_N * a1, 1),
                            h=[round(hc - (VENT_N - 1) / 2 * VENT_PITCH - VENT_W / 2, 2),
                               round(hc + (VENT_N - 1) / 2 * VENT_PITCH + VENT_W / 2, 2)], z=list(VENT_Z))
    for f in holes:
        face = face - f
    face = max(face.faces(), key=lambda q: q.area)       # drop slivers cut off in the slots
    _PANEL_HOLES[side] = holes          # module-level: OCC faces stay out of the pickled assembly
    return _clean_face(face), rep


def _clean_face(face, tol=0.02):
    """Rebuild a planar profile from lines and three-point arcs only: offset curves become polylines
    (0.05 chord), edges shorter than `tol` are merged into their neighbour. STEP readers (Fusion) heal
    such a face into one valid solid; raw boolean / offset output did not."""
    from build123d import Edge, Face, Vector, Wire

    def rebuild(w):
        segs = []
        for e in w.order_edges():
            p0, p1 = Vector(e.position_at(0)), Vector(e.position_at(1))
            if e.geom_type.name == "LINE":
                segs.append(("L", [p0, p1]))
            elif e.geom_type.name == "CIRCLE" and e.length > 4 * tol:
                if (p1 - p0).length < tol:          # full circle: two half arcs
                    pm = Vector(e.position_at(0.5))
                    segs.append(("A", [p0, Vector(e.position_at(0.25)), pm]))
                    segs.append(("A", [pm, Vector(e.position_at(0.75)), p1]))
                else:
                    segs.append(("A", [p0, Vector(e.position_at(0.5)), p1]))
            else:
                n = max(2, int(math.ceil(e.length / 0.5)))
                pts = [Vector(e.position_at(i / n)) for i in range(n + 1)]
                segs += [("L", [pts[i], pts[i + 1]]) for i in range(n)]
        out = []
        for kind, pts in segs:                # drop tiny pieces: the next segment starts where the last ended
            if (pts[-1] - pts[0]).length < tol and kind == "L":
                continue
            if out:
                pts = [out[-1][1][-1]] + pts[1:]
            out.append((kind, pts))
        if out:
            k, pts = out[0]
            out[0] = (k, [out[-1][1][-1]] + pts[1:])
        edges = []
        for kind, pts in out:
            if kind == "L":
                edges.append(Edge.make_line(pts[0], pts[1]))
            else:
                try:
                    edges.append(Edge.make_three_point_arc(*pts))
                except Exception:             # degenerate after snapping: chords
                    edges += [Edge.make_line(pts[0], pts[1]), Edge.make_line(pts[1], pts[2])]
        return Wire(edges)
    outer = rebuild(face.outer_wire())
    inner = [rebuild(w) for w in face.inner_wires()]
    f = Face(outer, inner)
    assert f.is_valid and abs(f.area - face.area) < 0.01 * face.area, "profile clean-up changed the face"
    return f


_PANEL_HOLES = {}

COV_RES, COV_BAND = 0.1, 40.0


def _raster_tris(mask, h0, z0, P, tris):
    """OR into `mask` (rows z, cols h, pixel COV_RES, origin (h0, z0)) the 2D triangles P[tris]: a pixel
    counts as covered when its centre is inside a triangle. Triangles over already-covered pixels are skipped."""
    nz, nh = mask.shape
    P = np.asarray(P, float)
    for a, b, c in tris:
        pa, pb, pc = P[a], P[b], P[c]
        d = (pb[0] - pa[0]) * (pc[1] - pa[1]) - (pc[0] - pa[0]) * (pb[1] - pa[1])
        if abs(d) < 1e-9:
            continue
        i0 = max(int(math.floor((min(pa[0], pb[0], pc[0]) - h0) / COV_RES)), 0)
        i1 = min(int(math.ceil((max(pa[0], pb[0], pc[0]) - h0) / COV_RES)), nh)
        j0 = max(int(math.floor((min(pa[1], pb[1], pc[1]) - z0) / COV_RES)), 0)
        j1 = min(int(math.ceil((max(pa[1], pb[1], pc[1]) - z0) / COV_RES)), nz)
        if i0 >= i1 or j0 >= j1 or mask[j0:j1, i0:i1].all():
            continue
        H = h0 + (np.arange(i0, i1) + 0.5)[None, :] * COV_RES
        Z = z0 + (np.arange(j0, j1) + 0.5)[:, None] * COV_RES
        w1 = ((pb[0] - H) * (pc[1] - Z) - (pc[0] - H) * (pb[1] - Z)) / d
        w2 = ((pc[0] - H) * (pa[1] - Z) - (pa[0] - H) * (pc[1] - Z)) / d
        e = -1e-9
        mask[j0:j1, i0:i1] |= (w1 >= e) & (w2 >= e) & (1.0 - w1 - w2 >= e)


def panel_coverage(A, holes, masks=None):
    """Uncovered bay area (mm2) per side panel, seen straight on from outside: the bay rectangle
    (legs' inner faces, bottom rail top to deck rail bottom) minus the projections of the panel and of
    every frame rail, bracket, screw and nut within COV_BAND of the outer face, minus the intended
    cutouts (`holes`: side -> 2D faces in (h, z): e-stop hole, IEC / USB cutouts and screw holes, vents)."""
    from lib.mesh import fast_tessellate
    out = {}
    for side, (n, hv) in SIDES.items():
        n, hv = np.array(n, float), np.array(hv, float)
        h0, h1, z0, z1 = -RIN, RIN, BOT_Z1, LEG_Z1
        mask = np.zeros((int(round((z1 - z0) / COV_RES)), int(round((h1 - h0) / COV_RES))), bool)
        names = [f"s_panel_{side}"] + [m for m in A.parts if m != f"s_panel_{side}"]
        for nm in names:
            p = A.parts[nm]
            if nm != f"s_panel_{side}" and not (p.module == "frame" and p.kind in ("rail", "bracket", "screw", "nut")):
                continue
            b0, b1 = A.bbox(nm)
            k = int(np.argmax(np.abs(n)))
            if max(n[k] * b0[k], n[k] * b1[k]) < HALF - COV_BAND:
                continue
            v, f = fast_tessellate(p.solid, (0.05, 0.2))
            if not len(f):
                continue
            P = np.column_stack([v @ hv, v[:, 2]])
            _raster_tris(mask, h0, z0, P, f)
        cut = np.zeros_like(mask)
        for face in holes.get(side, []):
            vs, ts = face.tessellate(0.01)
            P = np.array([(q.X, q.Y) for q in vs], float)
            _raster_tris(cut, h0, z0, P, np.array(ts, int))
        open_ = ~mask & ~cut
        out[side] = round(float(open_.sum()) * COV_RES ** 2, 1)
        if masks is not None:
            masks[side] = open_
    return out


def _vent_clearance(A):
    """Smallest gap (mm, bbox-based, conservative) from each side's vent field, taken through the sheet
    and VENT_GAP further inside, to any other component; must be >= VENT_GAP."""
    out = {}
    for side, hc in VENT_H.items():
        n, hv = np.array(SIDES[side][0], float), np.array(SIDES[side][1], float)
        k, kh = int(np.argmax(np.abs(n))), int(np.argmax(np.abs(hv)))
        h0 = hc - (VENT_N - 1) / 2 * VENT_PITCH - VENT_W / 2
        h1 = hc + (VENT_N - 1) / 2 * VENT_PITCH + VENT_W / 2
        lo = np.zeros(3)
        hi = np.zeros(3)
        a0, a1 = sorted((hv[kh] * h0, hv[kh] * h1))
        lo[kh], hi[kh] = a0, a1
        lo[2], hi[2] = VENT_Z
        b0, b1 = sorted((n[k] * (PANEL_IN - VENT_GAP), n[k] * (PANEL_IN + PANEL_T)))
        lo[k], hi[k] = b0, b1
        best, who = 1e9, None
        for nm, p in A.parts.items():
            if nm == f"s_panel_{side}":
                continue
            c0, c1 = map(np.array, A.bbox(nm))
            gap = max(max(lo[i] - c1[i], c0[i] - hi[i]) for i in range(3))
            if gap < best:
                best, who = gap, nm
        out[side] = (round(float(best), 2), who)
        assert best >= VENT_GAP - 1e-6, f"{side} vents {best:.2f} mm from {who}"
    return out


def _panels(A):
    notch = RIN - 24.5                 # bottom / deck panels: corners clear of the 3-way brackets (23 mm arms)
    ext = RIN + 4.0
    out = {}
    _PANEL_HOLES.clear()
    A.meta_panels = {}
    for side in SIDES:
        face, rep = _profile(A, side)
        out[side] = _solid(extrude(_plane(side) * face, amount=PANEL_T))
        A.meta_panels[side] = rep
    A.meta_vent_clear = _vent_clearance(A)

    # bottom panel on the bottom frame, corners clear of the legs and 3-way brackets
    e = RC - 1.6
    p = _box(-e, e, -e, e, BOT_Z1, FLOOR_Z)
    for sx in (1, -1):
        for sy in (1, -1):
            xa, xb = sorted((sx * notch, sx * 200))
            ya, yb = sorted((sy * notch, sy * 200))
            p = p - _box(xa, xb, ya, yb, BOT_Z1 - 1, FLOOR_Z + 1)
    p = p - Cylinder(23.0, 20)
    x0, x1, y0, y1, _ = PSU_BOX
    for x, y in [(x, y) for x in (x0 + 8, x1 - 8) for y in (y0 + 8, y1 - 8)]:
        p = p - Cylinder(1.7, 20).moved(Location((x, y, BOT_Z1)))
    for x, y in fp7_controller_carrier.INSERTS:
        wx, wy, _ = fp7_controller_carrier.world(x, y)
        p = p - Cylinder(1.7, 20).moved(Location((wx, wy, BOT_Z1)))
    for x, y in _cl42t_holes():
        p = p - Cylinder(2.25, 20).moved(Location((x, y, BOT_Z1)))
    out["bottom"] = _solid(p)
    # deck infill panels in the deck rails' side slots, Z -9..-6
    dz0, dz1 = -9.0, -6.0
    cuts = []
    for sx in (1, -1):
        for sy in (1, -1):
            cuts.append(_box(*sorted((sx * notch, sx * 200)), *sorted((sy * notch, sy * 200)), dz0 - 1, dz1 + 1))
            o = _choc_side(sx, sy)
            cuts.append(_box(*sorted((sx * CROSS_X + o * (W / 2 - 5.0), sx * CROSS_X + o * (W / 2 + 50.5))),
                             *sorted((sy * (RIN - 50.5), sy * 200)), dz0 - 1, dz1 + 1))
            cuts.append(_box(*sorted((sx * CROSS_X - o * (W / 2 - 5.0), sx * CROSS_X - o * (W / 2 + 21.0))),
                             *sorted((sy * (RIN - 20.5), sy * 200)), dz0 - 1, dz1 + 1))
    for side in ("left", "middle", "right"):
        if side == "left":
            xa, xb = -ext, -(CROSS_X + W / 2) + 4.0
        elif side == "right":
            xa, xb = (CROSS_X + W / 2) - 4.0, ext
        else:
            xa, xb = -(CROSS_X - W / 2) - 4.0, (CROSS_X - W / 2) + 4.0
        p = _box(xa, xb, -ext, ext, dz0, dz1)
        for c in cuts:
            p = p - c
        if side == "middle":
            p = p - Cylinder(15.0, 40)
            p = p - Cylinder(G_D / 2, 40).moved(Location((G1[0], G1[1], 0)))
        if side == "right":
            p = p - Cylinder(G_D / 2, 40).moved(Location((G2[0], G2[1], 0)))
        out[f"deck_{side}"] = _solid(p)
    return out


def _grommet(z_top, t, label):
    """8 mm rubber grommet in a sheet of thickness t whose top face is z_top: 7.9 OD
    neck (5 ID) through the hole, 11 OD x 1 lips either side."""
    neck = HW.ring(7.9, 5.0, t, label).moved(Location((0, 0, z_top - t)))
    lo = HW.ring(11.0, 5.0, 1.0, label).moved(Location((0, 0, z_top - t - 1.0)))
    hi = HW.ring(11.0, 5.0, 1.0, label).moved(Location((0, 0, z_top)))
    s = _solid(neck + lo + hi)
    s.label = label
    return s


# ------------------------------------------------------------------ drive (from v6)

def _motor(A):
    body, shaft = _motor_solids()
    loc = _location(R_MOTOR, (0, 0, D6.MOTOR_FACE_Z))
    A.add(MOTOR, Compound(children=[copy.deepcopy(s).moved(loc) for s in body]), "motor", "drive_base", step=1)
    A.add("v_nema17_shaft", copy.deepcopy(shaft).moved(loc), "shaft", "drive_base", step=1, spins=True,
          note="motor shaft (vendor STEP solid), turns with the coupling")
    A.vendor.append(V.manifest_entry(MOTOR, "v_nema17"))


@lru_cache(maxsize=None)
def _coupling_src():
    return V._load(COUPLING_STEP)


def _coupling(A):
    s = _coupling_src()
    bb = s.bounding_box()
    R = _R([(0, 0, 1), (0, 1, 0), (-1, 0, 0)])
    t = np.array([0, 0, D6.COUPLING_Z0]) - R @ np.array([bb.min.X, (bb.min.Y + bb.max.Y) / 2, (bb.min.Z + bb.max.Z) / 2])
    A.add("v_coupling_jaw_5_8mm", copy.deepcopy(s).moved(_location(R, t)), "coupling", "drive_base", step=1,
          spins=True, includes="MECCANIXITY 5 -> 8 mm jaw coupling, D25 L30 (24 x 32 envelope model)")
    A.vendor.append(V.manifest_entry("v_coupling_jaw_5_8mm", "v_coupling_jaw_5_8"))


@lru_cache(maxsize=None)
def _hub_parts():
    sols = import_step(HUB_STEP).solids()
    hub = max(sols, key=lambda s: s.volume)
    screws = [s for s in sols if s is not hub]
    hb = hub.bounding_box()
    cx, cy = (hb.min.X + hb.max.X) / 2, (hb.min.Y + hb.max.Y) / 2
    for dx in (-8, 8):
        for dy in (-8, 8):
            hub = hub - Cylinder(1.65, 12.0).moved(Location((cx + dx, cy + dy, hb.min.Z + 5.0)))
    hub = hub.solids()[0]
    return hub, screws, (cx, cy, hb.min.Z)


def _hub(A):
    hub, screws, (cx, cy, z_face) = _hub_parts()
    R = _R([(1, 0, 0), (0, -1, 0), (0, 0, -1)])
    t = np.array([0, 0, D6.HUB_TOP_Z]) - R @ np.array([cx, cy, z_face])
    loc = _location(R, t)
    comp = Compound(children=[copy.deepcopy(hub).moved(loc)] + [copy.deepcopy(s).moved(loc) for s in screws])
    A.add(HUB, comp, "hub", "rotor", step=3, includes="1310-0016-0008 Hyper Hub + its two M4 clamp screws")
    A.vendor.append(V.manifest_entry(HUB, "v_gobilda_1310_hub_8mm"))


@lru_cache(maxsize=None)
def _pi_src():
    """Representative Pi Zero 2 W: the vendor PCB solid, every component as its box."""
    return RP.box_envelope(import_step(PI_STEP), keep_main_faces=400, vmin=2.0)


def _pi(A, z0):
    s = _pi_src()
    R = _R([(1, 0, 0), (0, 0, 1), (0, -1, 0)])
    t = np.array([0.0, 45.0, z0 + D6.PI_DZ]) - R @ np.array([0.0, -0.8, 0.0])
    A.add("v_rpi_zero_2w", copy.deepcopy(s).moved(_location(R, t)), "pcb", "electrical", step=5, spins=True)
    A.vendor.append(dict(V.manifest_entry("v_rpi_zero_2w", "v_rpi_zero_2w"),
                         note="representative solid derived from the vendor STEP (PCB kept, parts as boxes)"))


def _hall_module(label):
    """A3144 hall breakout (HiLetgo class): PCB 14 x 32 x 1.6, the TO-92 lying flat
    sensing face up at its +X side (4 x 3 x 1.52), a 3-pin header under the -Y end."""
    pcb = Box(14, 32, 1.6, align=(Align.CENTER, Align.CENTER, Align.MIN))
    sen = Box(4.0, 3.0, 1.52, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((D6.MAGNET_R - fp7_pedestal.HALL_C[0], 0, 1.6)))
    hdr = Box(2.5, 7.6, 6.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(
        Location((0, fp7_pedestal.HALL_HEADER[0], 0)))
    c = Compound([Solid(pcb.wrapped), Solid(sen.wrapped), Solid(hdr.wrapped)])
    c.label = label
    return c


# ------------------------------------------------------------------ frogs

@lru_cache(maxsize=None)
def _frog_verts():
    b = np.asarray(V.stl_mesh(FROG_BODY_STL).vertices, float)
    e = np.asarray(V.stl_mesh(FROG_EYES_STL).vertices, float)
    return np.vstack([b, e])


@lru_cache(maxsize=None)
def _frog_eyes_local():
    """Eye bodies with the body subtracted (the decimated eyes dip 0.1 mm into their sockets)."""
    e = V._mesh_solid(FROG_EYES_STL) - V._mesh_solid(FROG_BODY_STL)
    sols = e.solids()
    return sols[0] if len(sols) == 1 else Compound(sols)


def _frog_wall_pose():
    """Outside the drum tube at the +X azimuth (centre y 0), belly on the tube so the
    fisheye inside sees belly and face through the acrylic, head up.
    Columns are the world images of local x, y, z: local -Y (head) -> +Z,
    local -Z (belly) -> -X (toward the axis), local x = y cross z -> -Y."""
    R = _R([(0, -1, 0), (0, 0, -1), (1, 0, 0)])
    v = _frog_verts() @ R.T
    c = (v.min(0) + v.max(0)) / 2
    # every vertex at x >= TUBE_OR + FROG_CLEAR (0.3 off the tangent plane at +X), so the
    # bbox stays outside the drum radius; the closest vertex is a splayed foot 2.7 mm off
    # the curved wall (sliding in until it touched would push the limbs past x 76).
    t = np.array([TUBE_OR + FROG_CLEAR - v[:, 0].min(), -c[1], FROG_WALL_Z - c[2]])
    w = v + t
    r = np.hypot(w[:, 0], w[:, 1])
    ring_under = D6.DRUM_TOP_Z - fp_disc_clamp_ring.COLLAR_H
    assert w[:, 2].max() <= ring_under - FROG_RING_GAP, "wall frog too close to the clamp ring"
    assert r.max() <= MESH_R[0] - FROG_NET_GAP, "wall frog too close to the net sleeve"
    assert r.min() >= TUBE_OR + FROG_CLEAR - 1e-6, "wall frog inside the tube"
    return R, t


def _frog_disc_pose():
    """On the top disc at the +X azimuth, belly down, body TANGENTIAL (long axis, local Y,
    along world Y) so the feet sit at similar radii and share the centripetal shear;
    head (eyes at local -Y) toward +Y, which the tilted fisheye sees better than head -Y
    (more eye surface unoccluded by the head, smaller off-axis angle). Columns:
    local x -> -X, local y -> -Y (head -> +Y), local z -> +Z.
    Slid out along +X to the largest centre radius with every vertex FROG_DISC_SPARE
    inside the bezel limit."""
    R = _R([(-1, 0, 0), (0, -1, 0), (0, 0, 1)])
    v = _frog_verts() @ R.T
    c = (v.min(0) + v.max(0)) / 2
    lim = fp_disc_clamp_ring.BEZEL_ID / 2 - 3.0
    z = DISC_TOP_Z + FROG_CLEAR - v[:, 2].min()
    lo, hi = 0.0, lim
    for _ in range(50):
        mid = (lo + hi) / 2
        if np.hypot(v[:, 0] - c[0] + mid, v[:, 1] - c[1]).max() <= lim - FROG_DISC_SPARE:
            lo = mid
        else:
            hi = mid
    t = np.array([math.floor(lo * 100) / 100 - c[0], -c[1], z])
    w = v + t
    assert np.hypot(w[:, 0], w[:, 1]).max() <= lim, "disc frog hits the bezel"
    assert w[:, 2].max() <= MESH_TOP_Z[0] - FROG_MESH_TOP_GAP, "disc frog too close to the net top"
    return R, t


def frog_foot_loads():
    """Min / max vertex radius of each frog and the centripetal g at FROG_RPM there."""
    out = {}
    for k, (R, t) in (("disc", _frog_disc_pose()), ("wall", _frog_wall_pose())):
        w = _frog_verts() @ R.T + t
        r = np.hypot(w[:, 0], w[:, 1])
        ri, ro = float(r.min()), float(r.max())
        g = {n: [(2 * math.pi * n / 60) ** 2 * x / 1000 / 9.81 for x in (ri, ro)] for n in FROG_RPM}
        out[k] = (ri, ro, g, ro / ri)
    return out


def _foot_load_note(label, ri, ro, g, ratio):
    sp = "; ".join(f"{n} RPM {g[n][0]:.1f} g / {g[n][1]:.1f} g" for n in FROG_RPM)
    return f"{label}: inner r {ri:.1f} mm outer r {ro:.1f} mm; {sp}; ratio {ratio:.2f}"


def frog_report():
    """Frog centres and the gaps the placement is built around (mm)."""
    rw, tw = _frog_wall_pose()
    rd, td = _frog_disc_pose()
    w = _frog_verts() @ rw.T + tw
    d = _frog_verts() @ rd.T + td
    r = np.hypot(w[:, 0], w[:, 1])
    cw, cd = (w.min(0) + w.max(0)) / 2, (d.min(0) + d.max(0)) / 2
    return dict(wall_centre=[round(float(x), 2) for x in cw],
                wall_z=[round(float(w[:, 2].min()), 2), round(float(w[:, 2].max()), 2)],
                disc_centre=[round(float(x), 2) for x in cd],
                gap_clamp_ring=round(float(D6.DRUM_TOP_Z - fp_disc_clamp_ring.COLLAR_H - w[:, 2].max()), 2),
                gap_net=round(float(MESH_R[0] - r.max()), 2),
                gap_tube=round(float(r.min() - TUBE_OR), 2),
                gap_mesh_top=round(float(MESH_TOP_Z[0] - d[:, 2].max()), 2))


def camera_pov():
    """Rotor fisheye point of view for the camera_view render: lens datum, optical axis."""
    lens = fp_camera_carrier.lens_datum(D6.DECK_TOP_Z)
    ax = np.array(fp_camera_carrier.OPTICAL_AXIS, float)
    ax /= np.linalg.norm(ax)
    return dict(eye=tuple(float(x) for x in lens), axis=tuple(float(x) for x in ax), fov=120.0,
                hide=("v_camera_fisheye_imx219_160", "fp_camera_carrier"),
                closeup_focus=(60.0, 0.0, 205.0))


def frog_offaxis(A):
    lens = np.array(fp_camera_carrier.lens_datum(D6.DECK_TOP_Z))
    ax = np.array(fp_camera_carrier.OPTICAL_AXIS, float)
    ax /= np.linalg.norm(ax)
    out = {}
    for n in ("v_frog_wall_body", "v_frog_disc_body"):
        mn, mx = (np.array(v) for v in A.bbox(n))
        d = (mn + mx) / 2 - lens
        out[n] = round(math.degrees(math.acos(float(d @ ax / np.linalg.norm(d)))), 1)
    return out


def frog_centres(A):
    return {n: [round(float(x), 2) for x in (np.array(A.bbox(n)[0]) + np.array(A.bbox(n)[1])) / 2]
            for n in ("v_frog_wall_body", "v_frog_disc_body")}


# ------------------------------------------------------------------ cage

HOOP_INNER_K = 0.997            # inner ring shrunk 0.3 % (rib r 108.7 -> 108.4) so it clears the outer ring's groove


@lru_cache(maxsize=None)
def _hoop_src(path, k=1.0):
    """Representative ring: revolved from the Keiran STL's measured cross
    section plus its clamp lug as one block (the faceted mesh was 2,200 to
    2,900 faces a ring)."""
    from build123d import scale
    s = RP.ring_from_mesh(V.stl_mesh(path), 110.0, 110.0)
    return scale(s, by=k) if k != 1.0 else s


def _hoop(A, which, ring, path):
    R = _R([(math.cos(math.radians(HOOP_LUG_DEG - 90)), math.sin(math.radians(HOOP_LUG_DEG - 90)), 0),
            (-math.sin(math.radians(HOOP_LUG_DEG - 90)), math.cos(math.radians(HOOP_LUG_DEG - 90)), 0), (0, 0, 1)])
    k = HOOP_INNER_K if ring == "inner" else 1.0
    t = np.array([0, 0, HOOP_Z[which] + (0.4 * (1 - k) if ring == "inner" else 0.0)]) - R @ np.array([110.0 * k, 110.0 * k, 0.0])
    nm = f"v_keiran_hoop_{which}_{ring}"
    A.add(nm, copy.deepcopy(_hoop_src(path, k)).moved(_location(R, t)), "printed", "cage", step=8,
          rmin=100.0 * k - 0.1 if ring == "inner" else 105.9,
          includes="Keiran 220 mm embroidery hoop ring (Printables 254483), clamps the mesh")
    A.vendor.append(_rep_entry(nm, HOOP_DIR, "Keiran STL"))


def _cage(A, R_):
    rails = {0: R_["deck_y_right"], 90: R_["deck_x_back"], 180: R_["deck_y_left"], 270: R_["deck_x_front"]}
    seats = {}
    for i, ang in enumerate((0, 90, 180, 270), 1):
        nm = f"fp7_cage_seat_{i}"
        A.add(nm, fp7_cage_seat.build(ang), "printed", "cage", step=8, rmin=fp7_cage_seat.R_IN - 0.01)
        seats[ang] = nm
        for k, (x, y) in enumerate(fp7_cage_seat.TAB_HOLES, 1):
            wx, wy = fp7_cage_seat.rot(ang, x, y)
            bolt_joint(A, [nm, rails[ang]], (wx, wy, 0.0), (0, 0, -1), screw="M3x12", nut="slot_nut",
                       name=f"j_seat_{i}_rail_{k}", module="cage", evaluate=False, contact="faces", step=8)
    for which in ("bottom", "top"):
        _hoop(A, which, "inner", HOOP_IN_STL)
        _hoop(A, which, "outer", HOOP_OUT_STL)
    sleeve = HW.ring(2 * MESH_R[1], 2 * MESH_R[0], MESH_Z[1] - MESH_Z[0], "s_mesh_sleeve").moved(
        Location((0, 0, MESH_Z[0])))
    A.add("s_mesh_sleeve", sleeve, "mesh", "cage", step=8, rmin=MESH_R[0] - 0.01,
          note="insect net sleeve (modelled as a 0.5 shell), clamped in both hoop pairs")
    top = Cylinder(MESH_TOP_R, MESH_TOP_Z[1] - MESH_TOP_Z[0], align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, MESH_TOP_Z[0])))
    A.add("s_mesh_top", _solid(top), "mesh", "cage", step=8, note="insect net top, clamped in the top hoop pair")
    host = {90: seats[90], 210: seats[180], 330: seats[0]}
    for i, ang in enumerate(STRUT_ANGLES, 1):
        nm = f"fp7_cage_strut_{i}"
        A.add(nm, fp7_cage_strut.build(ang), "printed", "cage", step=8)
        x, y = fp7_cage_strut.screw_xy(ang)
        insert_joint(A, [nm, host[ang]], (x, y, fp7_cage_strut.Z_FOOT), (0, 0, -1), screw="M3x12",
                     name=f"j_strut_{i}_seat", module="cage", evaluate=False, step=8)


# ------------------------------------------------------------------ plinth electrics

@lru_cache(maxsize=None)
def _cl42t_src():
    """Representative CL42T: the vendor housing solid (ears and mount holes), terminals and parts as boxes."""
    return RP.box_envelope(import_step(CL42T_STEP), keep_main_faces=400, vmin=2.0)


@lru_cache(maxsize=None)
def _estop_src():
    return import_step(ESTOP_STEP)


ESTOP_BOX_FRONT = 56.5          # vendor STEP: box front face (local +Y = actuator axis), collar beyond it
ESTOP_AXIS_XZ = (17.5, -13.5)   # vendor STEP: actuator axis


def _estop_frame():
    """(R, t) of the vendor e-stop frame for an operator offset of 0: local Y 56.5 (box front) on the front
    panel's inner face, actuator axis at (ESTOP_X, ESTOP_Z), local +Y outward (world -Y)."""
    R = _R([(1, 0, 0), (0, -1, 0), (0, 0, -1)])
    t = np.array([ESTOP_X - ESTOP_AXIS_XZ[0], -PANEL_IN + ESTOP_BOX_FRONT, ESTOP_Z + ESTOP_AXIS_XZ[1]])
    return R, t


def _estop_panel_mount():
    src = _estop_src()
    sols = sorted(src.solids(), key=lambda q: q.volume)
    lid, body = sols[0], sols[-1]
    gap = PANEL_T + LEGEND_T                     # operator moved out by the panel plus the legend ring
    big = 400.0
    rear = body & _box(-big, big, -big, ESTOP_BOX_FRONT, -big, big)
    front = (body & _box(-big, big, ESTOP_BOX_FRONT, big, -big, big)).moved(Location((0, gap, 0)))
    ax, az = ESTOP_AXIS_XZ
    from build123d import RegularPolygon
    bush = extrude(Plane(origin=(ax, ESTOP_BOX_FRONT - 0.01, az), x_dir=(1, 0, 0), z_dir=(0, 1, 0))
                   * RegularPolygon(11.0, 16), amount=gap + 0.02)
    s = _solid(rear + bush + front)
    R, t = _estop_frame()
    loc = _location(R, t)
    return Compound(children=[copy.deepcopy(s).moved(loc), copy.deepcopy(lid).moved(loc)])


def _dev_body(x, w, h, y0, y1):
    """Back-panel device body along Y, its corners rounded R (DEV_R - DEV_CLEAR) to follow the cutout."""
    from build123d import RectangleRounded
    f = RectangleRounded(w, h, DEV_R - DEV_CLEAR)
    return _solid(extrude(Plane(origin=(x, y0, PANEL_DEV_Z), x_dir=(1, 0, 0), z_dir=(0, 1, 0)) * f, amount=y1 - y0))


def _legend_location():
    """fp7_estop_legend (axis +Z, back face Z 0) on the front panel's outer face, axis on the e-stop."""
    return _location(_R([(1, 0, 0), (0, 0, 1), (0, -1, 0)]), (ESTOP_X, -(PANEL_IN + PANEL_T), ESTOP_Z))


def _electrics(A):
    # CL42T lying flat, 116 along Y, terminals toward the motor (+X)
    R = _R([(0, 1, 0), (0, 0, 1), (1, 0, 0)])
    t = np.array([CL42T_X0, 0.0, FLOOR_Z])
    A.add("v_cl42t_driver", copy.deepcopy(_cl42t_src()).moved(_location(R, t)), "driver", "electrical", step=7,
          includes="STEPPERONLINE CL42T-V41 closed-loop driver (representative of the vendor STEP)")
    A.vendor.append(dict(V.manifest_entry("v_cl42t_driver", "v_cl42t"),
                         note="representative solid derived from the vendor STEP (housing kept, terminals as boxes)"))
    x0, x1, y0, y1, h = PSU_BOX
    psu = _box(x0, x1, y0, y1, FLOOR_Z, FLOOR_Z + h)
    A.meta_psu_holes = [(x, y) for x in (x0 + 8, x1 - 8) for y in (y0 + 8, y1 - 8)]
    for x, y in A.meta_psu_holes:
        psu = psu - Cylinder(1.25, 6.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y, FLOOR_Z)))
    A.add("v_psu_lrs_50_24", _solid(psu), "enclosure", "electrical", step=7,
          note="MEAN WELL LRS-50-24 envelope 99 x 82 x 30, 4 x M3 tapped in the base")
    A.vendor.append(V.manifest_entry("v_psu_lrs_50_24", "v_psu_lrs_50_24"))
    A.add("fp7_controller_carrier", fp7_controller_carrier.build(), "printed", "electrical", step=7)
    cc = fp7_controller_carrier
    px, py = cc.PICO_C
    A.add("v_pico", HW.pcb_box(21, 51, label="v_pico").moved(Location(cc.world(px, py, cc.T + cc.BOSS_H))), "pcb",
          "electrical", step=7)
    A.vendor.append(V.manifest_entry("v_pico", "v_pico"))
    bx, by = cc.BUCK_C
    A.add("v_buck_module", _solid(Box(43, 21, 14, align=(Align.CENTER, Align.CENTER, Align.MIN))).moved(
        Location(cc.world(bx, by, cc.T))), "pcb", "electrical", step=7)
    A.vendor.append(V.manifest_entry("v_buck_module", "v_buck_module"))
    rx, ry = cc.RELAY_C
    A.add("v_relay_delay", _solid(Box(50, 26, 18, align=(Align.CENTER, Align.CENTER, Align.MIN))).moved(
        Location(cc.world(rx, ry, cc.T))), "pcb", "electrical", step=7)
    A.vendor.append(V.manifest_entry("v_relay_delay", "v_relay_delay"))
    # e-stop, panel-mount: the vendor STEP split at its box front (local Y 56.5); the box seats on the front
    # panel's inner face, the operator (collar, neck, mushroom) sits on the legend ring outside, joined
    # by a 22 mm bushing (16-gon, inside the 22.3 hole) through panel and ring
    A.add("v_estop_22mm", _estop_panel_mount(), "switch", "electrical", step=7,
          includes="22 mm mushroom e-stop (FreeCAD-library STEP), panel mount: box behind the front panel, "
                   "collar and head proud of it on the legend ring")
    A.vendor.append(V.manifest_entry("v_estop_22mm", "v_estop_22mm"))
    A.add("fp7_estop_legend", fp7_estop_legend.build().moved(_legend_location()), "printed", "electrical", step=7,
          note="yellow PETG legend ring behind the e-stop collar, clamped by the e-stop ring nut")
    x, w, hh, ya, yb = IEC
    fw, fh, ft = IEC_FLANGE
    iec = _dev_body(x, w, hh, ya, yb)
    iec = iec + _box(x - fw / 2, x + fw / 2, PANEL_IN - ft, PANEL_IN, PANEL_DEV_Z - fh / 2, PANEL_DEV_Z + fh / 2)
    for sx in (-1, 1):
        iec = iec - Plane(origin=(x + sx * IEC_PITCH / 2, PANEL_IN + 1, PANEL_DEV_Z), z_dir=(0, -1, 0)) * Cylinder(
            1.25, ft + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    A.add("v_iec_inlet", _solid(iec), "switch", "electrical", step=7,
          note=f"IEC C14 inlet with fuse drawer, rear-mount: {w} x {hh} body flush with the back panel's outer face, "
               f"{fw} x {fh} x {ft} flange behind the panel, 2 x M3 tapped at {IEC_PITCH:.0f} mm pitch "
               "(representative envelope)")
    A.vendor.append(V.manifest_entry("v_iec_inlet", "v_iec_inlet"))
    x, w, hh, ya, yb = USB
    A.add("v_usb_panel", _solid(_dev_body(x, w, hh, ya, yb)),
          "switch", "electrical", step=7,
          note="USB-C panel-mount extension through the back panel, front flush with its outer face")
    A.vendor.append(V.manifest_entry("v_usb_panel", "v_usb_panel"))


def _panel_notes(A):
    cov = panel_coverage(A, _PANEL_HOLES)
    A.meta_panel_coverage = cov
    A.notes.append("panel exterior coverage: " + " ".join(f"{k} {cov[k]}" for k in ("front", "back", "left", "right"))
                   + " mm2 uncovered (bay between the legs' inner faces and bottom rail top to deck rail bottom, "
                   "seen straight on, minus the projections of the panel and the frame rails, brackets, screws and "
                   "nuts, minus the e-stop hole, IEC / USB cutouts, IEC screw holes and vent slots; 0.1 mm raster)")
    P_ = A.meta_panels
    A.notes.append(f"panel notches: silhouette of each Tovi 3-way and idig3D plate (+ screws and nuts) in the "
                   f"panel slab, {NOTCH_C} clearance, closed by {NOTCH_CLOSE} at the 3-way corners (gaps backed by "
                   f"the bracket); slot-entry corners {SLOT_CHAMFER} x 45 deg: "
                   + ", ".join(f"{k} {v['slot_corner_chamfers']}" for k, v in P_.items()))
    es = A.bbox("v_estop_22mm")
    proud = -(PANEL_IN + PANEL_T) - es[0][1]
    lg = A.bbox("fp7_estop_legend")
    A.notes.append(f"e-stop: axis at x {ESTOP_X} z {ESTOP_Z}, 70 x 70 box behind the front panel (y {-PANEL_IN:.1f}.."
                   f"{es[1][1]:.1f}, front on the panel's inner face), collar front {LEGEND_T + 7.0:.1f} and head tip "
                   f"{proud:.1f} proud of the panel's outer face, collar seated on the legend ring; hole "
                   f"{ESTOP_HOLE_D} + {ESTOP_KEY_W} key to {ESTOP_KEY_R:.2f} from centre; legend ring "
                   f"OD {fp7_estop_legend.OD} ID {fp7_estop_legend.ID} T {fp7_estop_legend.T} "
                   f"(bbox y {lg[0][1]:.1f}..{lg[1][1]:.1f})")
    b = P_["back"]
    A.notes.append(f"back panel: IEC cutout {b['iec_cutout']['w']} x {b['iec_cutout']['h']} R{DEV_R}, 2 x 3.4 at "
                   f"{IEC_PITCH:.0f} pitch; USB-C cutout {b['usb_cutout']['w']} x {b['usb_cutout']['h']} R{DEV_R}; "
                   "both device fronts flush with the outer face")
    v = P_["right"]["vents"]
    A.notes.append(f"vents: {v['n']} slots {VENT_W} x {VENT_L} R{VENT_W / 2} at {VENT_PITCH} pitch per side panel "
                   f"(right beside the PSU, left beside the CL42T), {v['open_mm2']} mm2 open per side, "
                   f"{2 * v['open_mm2']:.1f} total; clearance to the nearest part through the sheet + {VENT_GAP}: "
                   + ", ".join(f"{k} {g} ({n})" for k, (g, n) in A.meta_vent_clear.items()))


def _cl42t_holes():
    """Mount holes of the CL42T in world XY (source X +-54.5 -> world Y, source Z 21.7 / 49.7 -> world X)."""
    return [(CL42T_X0 + z, y) for z in (21.7, 49.7) for y in (-54.5, 54.5)]


# ------------------------------------------------------------------ build

def build():
    A = Assembly("froggy_v7")
    R_ = _frame(A)
    for lvl in ("bottom", "top"):
        for sx in (1, -1):
            for sy in (1, -1):
                _threeway(A, R_, lvl, sx, sy)
    for sx in (1, -1):
        for sy in (1, -1):
            _chocolate(A, R_, sx, sy)
            _leon(A, R_, sx, sy)
            _idig(A, R_, sx, sy)
    _feet(A, R_)
    _clips(A, R_)

    # drive base (step 1: motor, plate, coupling)
    _motor(A)
    A.add("fp6_motor_plate", fp6_motor_plate.build(), "printed", "drive_base", step=1)
    _coupling(A)
    # step 2: pedestal, bearings, collar, shaft, TX coil, TX board, hall, grommet
    A.add("fp7_pedestal", fp7_pedestal.build(), "printed", "drive_base", step=2)
    for nm, z in (("v_bearing_608_2rs_lower", D6.LOWER_BRG_Z0), ("v_bearing_608_2rs_upper", D6.UPPER_BRG_Z0)):
        A.add(nm, HW.bearing_608_2rs(nm).moved(Location((0, 0, z))), "bearing", "drive_base", step=2)
        A.vendor.append(V.manifest_entry(nm, "v_bearing_608"))
    A.add("v_shaft_collar_8mm", HW.shaft_collar_8mm().moved(Location((0, 0, D6.COLLAR_Z0))), "collar", "rotor", step=2)
    A.vendor.append(V.manifest_entry("v_shaft_collar_8mm", "v_shaft_collar_8mm"))
    shaft = Cylinder(D6.SHAFT_D / 2, D6.SHAFT_Z1 - D6.SHAFT_Z0).moved(Location((0, 0, (D6.SHAFT_Z0 + D6.SHAFT_Z1) / 2)))
    A.add("s_shaft_8mm", shaft.solids()[0], "shaft", "rotor", step=2)
    A.vendor.append(V.manifest_entry("s_shaft_8mm", "s_shaft_8mm"))
    A.add("v_qi_tx_coil", HW.ring(D6.COIL_OD, D6.COIL_ID, D6.COIL_T, "v_qi_tx_coil").moved(Location((0, 0, D6.TX_Z0))),
          "coil", "electrical", step=2)
    A.vendor.append(V.manifest_entry("v_qi_tx_coil", "v_qi_kit"))
    A.add("v_qi_pcb_tx", HW.pcb_box(*fp7_pedestal.TX_BOARD[:2], label="v_qi_pcb_tx").moved(
        Location(fp7_pedestal.tx_board_origin())), "pcb", "electrical", step=2,
        note="Qi TX board on two M2.5 bosses inside the pedestal cavity")
    A.vendor.append(V.manifest_entry("v_qi_pcb_tx", "v_qi_kit"))
    A.add("v_hall_a3144", _hall_module("v_hall_a3144").moved(Location(fp7_pedestal.hall_origin())), "sensor",
          "electrical", step=2, bodies=[("v_hall_a3144_pcb", "pcb_green"), ("v_hall_a3144_sensor", "black_plastic"),
                                        ("v_hall_a3144_header", "black_plastic")])
    A.vendor.append(V.manifest_entry("v_hall_a3144", "v_hall_a3144_v7"))
    gx, gy = fp7_pedestal.CABLE_XY
    A.add("v_grommet_pedestal", _grommet(fp7_pedestal.T, fp7_pedestal.T, "v_grommet_pedestal").moved(
        Location((gx, gy, 0))), "grommet", "drive_base", step=2)
    A.vendor.append(V.manifest_entry("v_grommet_pedestal", "v_grommet_8mm"))
    # step 3: spacer + hub
    A.add("v_washer_8mm_spacer", HW.ring(12.0, 8.2, D6.WASHER_T, "v_washer_8mm_spacer").moved(
        Location((0, 0, D6.WASHER_Z0))), "washer", "rotor", step=3)
    A.vendor.append(V.manifest_entry("v_washer_8mm_spacer", "v_washer_8mm"))
    _hub(A)
    # step 4: carrier + its electrics + spine
    A.add("fp6_drum_carrier", fp6_drum_carrier.build(), "printed", "rotor", step=4)
    A.add("v_qi_rx_coil", HW.ring(D6.COIL_OD, D6.COIL_ID, D6.COIL_T, "v_qi_rx_coil").moved(Location((0, 0, D6.RX_Z0))),
          "coil", "electrical", step=4, spins=True)
    A.vendor.append(V.manifest_entry("v_qi_rx_coil", "v_qi_kit"))
    for nm, mx in (("v_magnet_6x3_index", D6.MAGNET_R), ("v_magnet_6x3_balance", -D6.MAGNET_R)):
        A.add(nm, HW.magnet_6x3().moved(Location((mx, 0, fp6_drum_carrier.MAGNET_Z0_V7))), "magnet", "electrical",
              step=4, spins=True)
        A.vendor.append(V.manifest_entry(nm, "v_magnet_6x3"))
    led_z0 = D6.CARRIER_LED_TOP_Z - D6.LED_LEN
    for i, (x, y) in enumerate(fp6_drum_carrier.led_positions(), 1):
        nm = f"v_ir_led_{i:02d}"
        A.add(nm, HW.ir_led_5mm(nm).moved(Location((x, y, led_z0))), "led", "electrical", step=4, spins=True,
              note="carrier rim, edge-lights the tube from its end")
        A.vendor.append(V.manifest_entry(nm, "v_ir_led_5mm"))
    A.add("v_led_driver", HW.pcb_box(20, 25, label="v_led_driver").moved(Location((-53.0, 0.0, D6.DECK_TOP_Z))),
          "pcb", "electrical", step=4, spins=True, note="IR LED driver on the carrier deck's -X spoke")
    A.vendor.append(V.manifest_entry("v_led_driver", "v_led_driver"))
    A.add("fp7_wire_spine", fp7_wire_spine.build(), "printed", "rotor", step=4)
    # step 5: camera carrier, camera, Pi, Qi RX board
    z0 = D6.DECK_TOP_Z
    A.add("fp_camera_carrier", fp_camera_carrier.build(z0), "printed", "rotor", step=5)
    cam = HW.camera_fisheye_imx219_160("v_camera_fisheye_imx219_160").moved(fp_camera_carrier.camera_location(z0))
    A.add("v_camera_fisheye_imx219_160", cam, "camera", "rotor", step=5,
          bodies=[("v_camera_fisheye_imx219_160_pcb", "pcb_green"), ("v_camera_fisheye_imx219_160_lens", "glass_lens")])
    A.vendor.append(V.manifest_entry("v_camera_fisheye_imx219_160", "v_camera_fisheye"))
    _pi(A, z0)
    A.add("v_qi_pcb_rx", HW.pcb_box(30, 20, label="v_qi_pcb_rx").moved(Location((0, -42.5, z0 + 4.0))), "pcb",
          "electrical", step=5, spins=True, note="Qi RX board on the camera carrier's -Y tray")
    A.vendor.append(V.manifest_entry("v_qi_pcb_rx", "v_qi_kit"))
    # step 6: drum, disc, clamp ring, ring LEDs, frogs
    dz = D6.DRUM_FLOOR_Z - D0.DRUM_FLOOR_Z
    A.add("s_drum_tube", s_drum_tube.build().moved(Location((0, 0, dz))), "stock", "rotor", step=6, opacity=0.35,
          rmin=TUBE_OR - D0.DRUM_WALL - 0.01)
    A.add("s_top_disc", s_top_disc.build().moved(Location((0, 0, dz))), "stock", "rotor", step=6, opacity=0.35)
    A.add("fp_disc_clamp_ring", fp_disc_clamp_ring.build(D6.DRUM_TOP_Z), "printed", "rotor", step=6)
    for i, loc in enumerate(fp_disc_clamp_ring.led_locations(D6.DRUM_TOP_Z), 13):
        nm = f"v_ir_led_{i:02d}"
        A.add(nm, HW.ir_led_5mm(nm).moved(loc), "led", "electrical", step=6, spins=True,
              note="clamp ring edge, edge-lights the top disc")
        A.vendor.append(V.manifest_entry(nm, "v_ir_led_5mm"))
    for nm, (R, t) in (("v_frog_wall", _frog_wall_pose()), ("v_frog_disc", _frog_disc_pose())):
        for part in ("body", "eyes"):
            src = V._mesh_solid(FROG_BODY_STL) if part == "body" else _frog_eyes_local()
            A.add(f"{nm}_{part}", copy.deepcopy(src).moved(_location(R, t)), "specimen",
                  "rotor", step=6, includes="Agalychnis callidryas, 45 mm snout-vent (mesh x 19.5, decimated)")
            A.vendor.append(V.vendor_entry(f"{nm}_{part}", FROG_DIR))

    # step 7: plinth panels, grommets, plinth electrics
    pan = _panels(A)
    for k, s in pan.items():
        A.add(f"s_panel_{k}", s, "panel", "plinth", step=7, note="3 mm black ABS sheet, laser cut")
    for nm, (x, y) in (("v_grommet_g1", G1), ("v_grommet_g2", G2)):
        A.add(nm, _grommet(-6.0, 3.0, nm).moved(Location((x, y, 0))), "grommet", "plinth", step=7)
        A.vendor.append(V.manifest_entry(nm, "v_grommet_8mm"))
    _electrics(A)
    # step 8: cage
    _cage(A, R_)

    # ---------------------------------------------------------------- joints
    down, up = (0, 0, -1), (0, 0, 1)
    cross = {45: R_["cross_right"], -45: R_["cross_left"]}
    for x, y in fp6_motor_plate.RAIL_HOLES:
        bolt_joint(A, ["fp6_motor_plate", cross[int(x)]], (x, y, D6.PLATE_Z0 + D6.PLATE_T), up, screw="M3x10",
                   nut="slot_nut", name=f"j_motor_plate_{_c(x, y)}_cross_rail", module="drive_base",
                   evaluate=False, contact="faces", step=1)
    for x, y in fp6_motor_plate.MOTOR_HOLES:
        tapped_joint(A, ["fp6_motor_plate", MOTOR], (x, y, D6.MOTOR_FACE_Z), down, screw="M3x10",
                     name=f"j_motor_face_{_c(x, y)}", module="drive_base", evaluate=False, step=1)
    for x, y in fp7_pedestal.RAIL_HOLES:
        bolt_joint(A, ["fp7_pedestal", cross[int(x)]], (x, y, 0.0), down, screw="M3x10", nut="slot_nut",
                   name=f"j_pedestal_flange_{_c(x, y)}", module="drive_base", evaluate=False, contact="faces", step=2)
    for x, y in fp6_drum_carrier.HUB_HOLES:
        tapped_joint(A, ["fp6_drum_carrier", HUB], (x, y, D6.CARRIER_Z0), down, screw="M4x14",
                     name=f"j_hub_carrier_{_c(x, y)}", module="rotor", evaluate=False, step=4, head="csk",
                     seat=(x, y, D6.DECK_TOP_Z))
    for x, y in fp7_wire_spine.FOOT_HOLES:
        insert_joint(A, ["fp7_wire_spine", "fp6_drum_carrier"], (x, y, D6.DECK_TOP_Z), down, screw="M3x8",
                     name=f"j_spine_carrier_{_sg(x)}x", module="rotor", evaluate=False, step=4)
    for x, y in fp_camera_carrier.MOUNT_HOLES:
        insert_joint(A, ["fp_camera_carrier", "fp6_drum_carrier"], (x, y, D6.DECK_TOP_Z), down, screw="M3x8",
                     name=f"j_camera_carrier_{_c(x, y)}", module="rotor", evaluate=False, step=5)
    for k, (x, y) in enumerate(A.meta_psu_holes, 1):
        tapped_joint(A, ["s_panel_bottom", "v_psu_lrs_50_24"], (x, y, FLOOR_Z), up, screw="M3x8",
                     name=f"j_psu_panel_{k}", module="electrical", evaluate=False, step=7)
    for k, (x, y) in enumerate(fp7_controller_carrier.INSERTS, 1):
        wx, wy, _ = fp7_controller_carrier.world(x, y)
        insert_joint(A, ["s_panel_bottom", "fp7_controller_carrier"], (wx, wy, FLOOR_Z), up, screw="M3x7",
                     name=f"j_controller_panel_{k}", module="electrical", evaluate=False, step=7)
    for k, (x, y) in enumerate(_cl42t_holes(), 1):
        at = np.array([x, y, FLOOR_Z])
        seat = at + np.array([0, 0, CL42T_EAR_T])          # ear top face (5 mm flange)
        L = int(math.ceil(seat[2] - BOT_Z1 + HW.HEX_NUT["M4"]["m"] + 1.0))
        hex_nut_joint(A, ["v_cl42t_driver", "s_panel_bottom"], at, down, screw=f"M4x{L}", name=f"j_cl42t_panel_{k}",
                      module="electrical", evaluate=False, step=7, seat=seat)
    for k, sx in enumerate((-1, 1), 1):          # IEC inlet: 2 x M3 from outside into its tapped rear flange
        tapped_joint(A, ["s_panel_back", "v_iec_inlet"], (IEC[0] + sx * IEC_PITCH / 2, PANEL_IN, PANEL_DEV_Z),
                     (0, -1, 0), screw="M3x6", name=f"j_iec_panel_{k}", module="electrical", evaluate=False, step=7)
    _panel_notes(A)
    A.notes.append(f"motor: {motor_length()}")
    A.notes.append(f"frog off-axis from the fisheye optical axis (deg): {frog_offaxis(A)}")
    A.notes.append(f"frog centres (bbox, mm): {frog_centres(A)}")
    A.notes.append(f"frog gaps (mm): {frog_report()}")
    fl = frog_foot_loads()
    A.notes.append(_foot_load_note("disc frog foot loads", *fl["disc"]))
    A.notes.append(_foot_load_note("wall frog belly-to-back loads", *fl["wall"]))
    A.camera_pov = camera_pov()
    return A
