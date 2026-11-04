"""fp7_estop_legend: yellow legend ring behind the 22 mm e-stop on the plinth front panel (v7).

A plain flat ring, OD 45 x ID 22.6 x 3, no engraving: it sits on the front panel's outer face around
the 22.3 mm panel hole, the e-stop collar seats on its face and the e-stop's ring nut clamps ring and
panel together. 22.6 bore clears the 22 mm bushing. The outer front edge carries a 0.6 x 45 deg
chamfer; printed flat on its back face (Z 0), yellow PETG.
Part frame: axis +Z, back face (panel side) Z 0, front face Z 3.
"""
from build123d import Align, Axis, Cylinder

from parts._util import one

OD, ID, T, CHAMFER = 45.0, 22.6, 3.0, 0.6

SPEC = dict(
    name="fp7_estop_legend",
    function="Legend ring behind the e-stop collar on the plinth front panel (safety yellow, plain).",
    mates=[dict(part="s_panel_front", feature="back face Z 0 on the panel's outer face, bore on the 22.3 hole",
                kind="clamped"),
           dict(part="v_estop_22mm", feature="front face Z 3 under the collar, ring nut clamps", kind="clamped")],
    loads=[dict(where=[0, 16, 3], direction=[0, 0, -1], newton=50, note="palm strike on the mushroom")],
    print_face="-z (back face, Z 0)",
    print_rotate=(0, 0, 0),
    material="PETG",
    nonstructural=[],
)


def build():
    s = Cylinder(OD / 2, T, align=(Align.CENTER, Align.CENTER, Align.MIN)) - Cylinder(ID / 2, 3 * T)
    s = one(s)
    top = [e for e in s.edges() if e.center().Z > T - 1e-6 and e.radius > ID / 2 + 1]
    s = one(s.chamfer(CHAMFER, None, top))
    s.label = "fp7_estop_legend"
    return s
