"""fp7_cage_seat: stationary seat ring for the net cage (v7), four identical segments.

Each segment is a quarter of a ring r 90..121.5, 8 tall (Z 0..8), lying on the
deck, centred on one axis (build(angle) rotates the +X segment). An 11 wide x 5
deep groove (r 99.5..110.5, floor Z 3) takes the bottom Keiran 220 hoop pair
(inner ring r 100..106 and outer ring r 106..110 sit side by side). A 30 wide tab
on the segment's axis reaches the perimeter rail (r 150) and carries 3 x M3 x 12
into top-slot nuts. Every segment end carries half a notch for the hoop clamp
lug (the lug sits at a segment joint); through notches at +-45 mm off the axis
clear the Leon Matthews T plates where the cross rails meet the perimeter rails
(they only matter on the +-Y segments); three M3 heat-set insert pockets at r 116
(0 and +-30 deg) take the cage struts' foot screws.
"""
import math

from build123d import Align, Box, Cylinder, Location, Polygon, Rot, extrude

from lib import hardware as HW
from parts._util import one

R_IN, R_OUT, H = 90.0, 121.5, 8.0
GROOVE = (99.5, 110.5, 3.0)
TAB_X0, TAB_X1, TAB_W = 112.0, 150.0, 30.0
RAIL_X = 142.5
TAB_HOLES = [(RAIL_X, y) for y in (-9.0, 0.0, 9.0)]
HOLE_D = 3.4
LEON_ANG = (17.0, 27.2, 110.5)        # angular band (deg) and min radius of the T-plate clearance notches
LUG_HALF_DEG, LUG_R0 = 11.0, 100.0     # clamp lug: 39.6 wide, r 109.3..135
INSERT_R = 116.0
INSERT_ANGLES = (-30.0, 0.0, 30.0)

SPEC = dict(
    name="fp7_cage_seat",
    function="Stationary seat ring: locates the bottom hoop pair of the net cage on the deck and anchors the struts.",
    mates=[dict(part="perimeter rail", feature="tab underside Z 0, 3 x M3 x 12 into top-slot nuts", kind="slot_nut x3"),
           dict(part="v_keiran_hoop_bottom", feature="groove r 99.5..110.5, floor Z 3", kind="seat"),
           dict(part="fp7_cage_strut", feature="M3 heat-set insert at r 116", kind="heat_set_insert")],
    loads=[dict(where=[116, 0, 8], direction=[0, 0, -1], newton=10, note="cage weight + a knock")],
    print_face="-z (underside, Z 0)",
    print_rotate=(0, 0, 0),
    material="PETG",
    nonstructural=[],
)


def _ring(r0, r1, z0, z1, a0=-45.0, a1=45.0):
    ring = Cylinder(r1, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN)) - Cylinder(r0, 100)
    big = 400.0
    pts = [(0, 0)] + [(big * math.cos(math.radians(a)), big * math.sin(math.radians(a)))
                      for a in (a0, (a0 + a1) / 2 - 30, (a0 + a1) / 2, (a0 + a1) / 2 + 30, a1)]
    wedge = extrude(Polygon(*pts, align=None), amount=200).moved(Location((0, 0, -50)))
    return (ring & wedge).moved(Location((0, 0, z0)))


def _base():
    b = _ring(R_IN, R_OUT, 0, H)
    b = b + Box(TAB_X1 - TAB_X0, TAB_W, H, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(Location((TAB_X0, 0, 0)))
    g0, g1, gz = GROOVE
    b = b - _ring(g0, g1, gz, H + 1, -50, 50)
    for x, y in TAB_HOLES:
        b = b - Cylinder(HOLE_D / 2, 40).moved(Location((x, y, 0)))
    a0, a1, rmin = LEON_ANG
    for s in (1, -1):
        b = b - _ring(rmin, 200, -1, H + 1, min(s * a0, s * a1), max(s * a0, s * a1))
    for a in (-45.0, 45.0):
        b = b - _ring(LUG_R0, 200, gz, H + 1, a - LUG_HALF_DEG, a + LUG_HALF_DEG)
    for a in INSERT_ANGLES:
        x, y = INSERT_R * math.cos(math.radians(a)), INSERT_R * math.sin(math.radians(a))
        b = b - Cylinder(HW.INSERT["M3"]["pocket_d"] / 2, 40).moved(Location((x, y, 0)))
    return one(b)


def build(angle=0.0):
    s = _base()
    if angle:
        s = s.rotate(__import__("build123d").Axis.Z, angle)
    s.label = "fp7_cage_seat"
    return s


def rot(angle, x, y):
    c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    return (c * x - s * y, s * x + c * y)
