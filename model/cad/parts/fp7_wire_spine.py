"""fp7_wire_spine: co-rotating wire channel up the outside of the drum at -Y (v7).

A 12 x 10 PETG channel (r 84..94, 6 x 4 bore) from the carrier rim (Z 80.31) to
the clamp ring top (Z 236.71). Its 30 x 16 x 4 foot sits on the carrier's -Y lug
with 2 x M3 x 8 into heat-set inserts; the top 12.4 mm is the key tongue that
runs in the clamp ring's -Y notch, so the ring, disc and drum cannot turn on the
carrier. Wires for the clamp-ring LEDs run in the bore. Prints upright on its foot.
"""
from build123d import Align, Box, Cylinder, Location

from parts import fp6_datums as D6
from parts._util import one

Y0, Y1, W = -94.0, -84.0, 12.0          # channel radial extent (at -Y) and width
BORE = (6.0, 4.0)
FOOT = (30.0, -96.0, -80.0, 4.0)        # width, y0, y1, thickness
FOOT_HOLES = [(sx * 10.0, -88.0) for sx in (1, -1)]
Z0 = D6.DECK_TOP_Z                      # 80.31
TOP_Z = 236.71                          # clamp ring top
HOLE_D = 3.4

SPEC = dict(
    name="fp7_wire_spine",
    function="Carries the clamp-ring LED wires down the outside of the drum and keys the ring to the carrier.",
    mates=[dict(part="fp6_drum_carrier", feature="foot on the -Y lug, 2 x M3 x 8 into inserts", kind="heat_set_insert x2"),
           dict(part="fp_disc_clamp_ring", feature="tongue in the -Y notch", kind="key")],
    loads=[dict(where=[0, -89, 150], direction=[0, -1, 0], newton=3, note="centrifugal at 600 rpm")],
    print_face="-z (foot underside, Z 80.31)",
    print_rotate=(0, 0, 0),
    material="PETG",
    nonstructural=[],
)


def build():
    fw, fy0, fy1, ft = FOOT
    b = Box(fw, fy1 - fy0, ft, align=(Align.CENTER, Align.MIN, Align.MIN)).moved(Location((0, fy0, Z0)))
    b = b + Box(W, Y1 - Y0, TOP_Z - Z0 - ft + 0.5, align=(Align.CENTER, Align.MIN, Align.MIN)).moved(
        Location((0, Y0, Z0 + ft - 0.5)))
    b = b - Box(BORE[0], BORE[1], 400).moved(Location((0, (Y0 + Y1) / 2, Z0)))
    for x, y in FOOT_HOLES:
        b = b - Cylinder(HOLE_D / 2, 40).moved(Location((x, y, Z0)))
    b = one(b)
    b.label = "fp7_wire_spine"
    return b
