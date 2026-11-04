"""s_drum_tube: stock cast acrylic tube od x wall x h long (cut to length; defaults DRUM_OD 152.4 x DRUM_WALL 3.2 x DRUM_H 152,
v9 TUBE_H 100; V12 passes DRUM12_OD 150 x DRUM12_WALL 2.5, I036)."""
from build123d import Align, Cylinder, Location

from parts import datums as D

SPEC = dict(name="s_drum_tube", function="Clear drum wall the frog sits on (normal test on the outside at +X).",
            mates=[dict(part="fp5_drum_carrier", feature="tube end in the V groove", kind="seat"),
                   dict(part="fp_disc_clamp_ring", feature="tube top in the collar", kind="seat")],
            loads=[], print_face="n/a (stock)", material="cast acrylic, clear", stock=True)


def build(h=None, od=None, wall=None):
    h = D.DRUM_H if h is None else h
    r = (D.DRUM_OD if od is None else od) / 2
    wall = D.DRUM_WALL if wall is None else wall
    t = Cylinder(r, h, align=(Align.CENTER, Align.CENTER, Align.MIN)) - Cylinder(r - wall, 3 * h)
    t = t.solids()[0].moved(Location((0, 0, D.DRUM_FLOOR_Z)))
    t.label = "s_drum_tube"
    return t
