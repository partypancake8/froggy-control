"""s_top_disc: stock 3.2 cast acrylic disc, 160 OD (laser cut)."""
from build123d import Align, Cylinder, Location

from parts import datums as D

SPEC = dict(name="s_top_disc", function="Shear-test floor for the frog on top of the drum, edge-lit by the ring.",
            mates=[dict(part="s_drum_tube", feature="rests on the tube top", kind="seat"),
                   dict(part="fp_disc_clamp_ring", feature="3.2 bezel rebate", kind="clamp")],
            loads=[], print_face="n/a (stock)", material="cast acrylic 3.2, clear", stock=True)


def build():
    d = Cylinder(D.DISC_OD / 2, D.ACRYLIC_T, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, D.DRUM_TOP_Z)))
    d.label = "s_top_disc"
    return d
