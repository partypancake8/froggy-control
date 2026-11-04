"""Frame stage: 340 x 250 rectangle of 15x15 rails, two cross rails at X -45 / +45,
eight Tovi corner brackets in the inside corners, sixteen M3x8 slot-nut joints.

Datums: Z 0 = rail top face, rails occupy Z -15..0, X and Y zero at the frame centre.
"""
import os

import numpy as np
from build123d import Location
from OCP.gp import gp_Trsf

from lib import profile_1515 as P
from lib import vendor as V
from lib.assembly import Assembly
from lib.joints import bolt_joint

MODULE = "frame"
OX, OY = 340.0, 250.0          # outer frame size
CROSS_X = 45.0                 # cross rail centres at X -45, +45
W = P.W
ZC = -W / 2
INNER_Y = OY / 2 - W           # inner face of the long X rails (110)
INNER_X = OX / 2 - W           # inner face of the end Y rails (155)
TOVI_DIR = "opensource/tovi-1515-corner-bracket"
TOVI_STEP = os.path.join(V.VENDOR, TOVI_DIR, "1515 Corner Bracket.step")


def _sgn(s):
    return "p" if s > 0 else "n"


def _location(R, t):
    tr = gp_Trsf()
    tr.SetValues(R[0, 0], R[0, 1], R[0, 2], t[0], R[1, 0], R[1, 1], R[1, 2], t[1],
                 R[2, 0], R[2, 1], R[2, 2], t[2])
    return Location(tr)


def _place_bracket(name, a, a_face, b, b_face):
    """Rotate the bracket so its two mount-face normals (read from its own hole
    axes) point along world a and b, then translate so the mount faces lie on
    the rail faces a_face (along a) and b_face (along b); width spans Z -15..0."""
    base = V.import_step_instance(TOVI_STEP)
    n0, n1 = (h["mount_normal"] for h in V.bracket_holes(base))
    src = np.column_stack([n0, n1, np.cross(n0, n1)])
    a, b = np.array(a, float), np.array(b, float)
    for ta, tb in ((a, b), (b, a)):
        R = np.column_stack([ta, tb, np.cross(ta, tb)]) @ src.T
        if np.linalg.det(R) > 0.5:
            break
    moved = base.moved(_location(R, (0, 0, 0)))
    bb = moved.bounding_box()
    mn, mx = np.array(tuple(bb.min)), np.array(tuple(bb.max))
    t = np.zeros(3)
    for n, face in ((a, a_face), (b, b_face)):
        ext = (mx if n.sum() > 0 else mn) @ np.abs(n)
        t += np.abs(n) * (face - ext)
    t[2] = -W - mn[2]
    return V.import_step_instance(TOVI_STEP, label=name, location=_location(R, t))


def build():
    return add_frame(Assembly("frame"))


def add_frame(A):
    """Add the 6 rails, 8 Tovi brackets and 16 slot-nut joints (module frame,
    install step 0) to assembly A and return it."""
    rails = [  # (length, axis, centre)
        (OX, "x", (0, OY / 2 - W / 2, ZC)),
        (OX, "x", (0, -(OY / 2 - W / 2), ZC)),
        (OY - 2 * W, "y", (OX / 2 - W / 2, 0, ZC)),
        (OY - 2 * W, "y", (-(OX / 2 - W / 2), 0, ZC)),
        (OY - 2 * W, "y", (CROSS_X, 0, ZC)),
        (OY - 2 * W, "y", (-CROSS_X, 0, ZC)),
    ]
    role = {}
    for i, (L, ax, c) in enumerate(rails, 1):
        s = P.rail(L, ax, c, n=i)
        A.add(s.label, s, "rail", MODULE, length_mm=L, axis=ax, centre=c)
    long_rail = {1: "s_rail_1515_1", -1: "s_rail_1515_2"}
    end_rail = {1: "s_rail_1515_3", -1: "s_rail_1515_4"}
    cross_rail = {1: "s_rail_1515_5", -1: "s_rail_1515_6"}

    specs = []
    for sx in (1, -1):
        for sy in (1, -1):
            specs.append((f"v_tovi_bracket_corner_{_sgn(sx)}x{_sgn(sy)}y", (0, sy, 0), sy * INNER_Y,
                          (sx, 0, 0), sx * INNER_X, (long_rail[sy], end_rail[sx])))
    for sx in (1, -1):
        for sy in (1, -1):
            specs.append((f"v_tovi_bracket_cross_{_sgn(sx)}x{_sgn(sy)}y", (0, sy, 0), sy * INNER_Y,
                          (sx, 0, 0), sx * (CROSS_X - W / 2), (long_rail[sy], cross_rail[sx])))
    for name, a, af, b, bf, mates in specs:
        A.add(name, _place_bracket(name, a, af, b, bf), "bracket", MODULE, mates=list(mates))
        A.vendor.append(V.vendor_entry(name, TOVI_DIR))
        role[name] = mates

    records = []
    for name, mates in role.items():
        for h in V.bracket_holes(A[name]):
            at, axis = h["centre"], h["axis"]
            # the rail this hole bolts into: the mate whose box holds a point 1 mm past the face
            probe = at + axis * 1.0
            rail = next(m for m in mates if all(A.bbox(m)[0][k] - 1e-6 <= probe[k] <= A.bbox(m)[1][k] + 1e-6
                                                for k in range(3)))
            jn = f"j_{name.replace('v_tovi_bracket_', '')}_{rail.replace('s_rail_1515_', 'r')}"
            records.append(bolt_joint(A, [name, rail], at, axis, screw="M3x8", nut="slot_nut",
                                      name=jn, module=MODULE, evaluate=False))
    return A
