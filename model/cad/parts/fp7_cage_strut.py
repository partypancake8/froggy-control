"""fp7_cage_strut: one of three stationary cage struts (v7), 12 x 12 section.

A C-shaped PETG strut outside the net at r 122..134 (Z 15..252). Its foot (r
110..134, Z 8..15) stands on the seat ring beside the bottom outer hoop ring and
is held by 1 x M3 x 12 into the seat's heat-set insert at r 116; its head (r
110..134, Z 252..264) lies against the top outer ring and a 3.5 mm hook (r
106..134, Z 264..267.5) clips over that ring, holding the top hoop pair down.
build(angle) places it at `angle` deg (the part is drawn on +X). Printed lying on
its side, turned 45 deg on the bed (259 long).
"""
import math

from build123d import Align, Axis, Box, Cylinder, Location

from parts._util import one

W = 12.0
R_COL = (122.0, 134.0)
R_CLIP0 = 110.05                        # 0.05 off the outer hoop ring (r 110)
Z_FOOT, Z_COL0, Z_HEAD0, Z_HEAD1 = 8.0, 15.0, 252.0, 264.0
HOOK = (106.0, 3.5)                     # hook inner radius, thickness
SCREW_R = 116.0
HOLE_D = 3.4

SPEC = dict(
    name="fp7_cage_strut",
    function="Holds the top hoop pair of the net cage 237 mm above the bottom pair; one of three.",
    mates=[dict(part="fp7_cage_seat", feature="foot underside Z 8, 1 x M3 x 12 into the seat insert", kind="heat_set_insert"),
           dict(part="v_keiran_hoop_top_outer", feature="head face r 110, hook over the ring top Z 264", kind="clip")],
    loads=[dict(where=[128, 0, 140], direction=[0, 1, 0], newton=5, note="a knock on the cage")],
    print_face="side (lying flat, 12 mm thick)",
    print_rotate=(90, 0, 45),
    material="PETG",
    nonstructural=[],
)


def _base():
    b = Box(R_COL[1] - R_CLIP0, W, Z_COL0 - Z_FOOT, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
        Location((R_CLIP0, 0, Z_FOOT)))
    b = b + Box(R_COL[1] - R_COL[0], W, Z_HEAD0 - Z_COL0 + 1, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
        Location((R_COL[0], 0, Z_COL0 - 0.5)))
    b = b + Box(R_COL[1] - R_CLIP0, W, Z_HEAD1 - Z_HEAD0, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
        Location((R_CLIP0, 0, Z_HEAD0)))
    b = b + Box(R_COL[1] - HOOK[0], W, HOOK[1], align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
        Location((HOOK[0], 0, Z_HEAD1)))
    b = b - Cylinder(HOLE_D / 2, 40).moved(Location((SCREW_R, 0, Z_FOOT)))
    return one(b)


def build(angle=0.0):
    s = _base()
    if angle:
        s = s.rotate(Axis.Z, angle)
    s.label = "fp7_cage_strut"
    return s


def screw_xy(angle):
    a = math.radians(angle)
    return (SCREW_R * math.cos(a), SCREW_R * math.sin(a))
