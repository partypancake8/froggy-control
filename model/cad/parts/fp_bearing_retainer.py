"""fp_bearing_retainer: upper 6814 inner-race retaining sleeve (ported from the
accepted fusion-pipeline fp_bearing_retainer, Run C3).

Kept from the accepted part: the OD 76 retaining sleeve that bears on the
upper inner race. Changed: bore 70.2 for the tower OD 69.95 (was 71.2 for
the old cup), sleeve Z 36..41 and OD 78.4 so the wall is 4.1 mm (3.1 under its 1 mm top fillet); the old
four-screw cap is replaced by the motor plate, which clamps the sleeve top
with the same four lug screws.
"""
from build123d import Align, Cylinder, Location

from parts._util import fillet_safe, one

Z0, Z1 = 36.0, 41.0
ID, OD = 70.2, 78.4

SPEC = dict(
    name="fp_bearing_retainer",
    function="Captures the upper 6814 inner race against the stacked lower bearing and tower shoulder.",
    mates=[dict(part="v_bearing_6814_upper", feature="underside Z 36 on the inner race r 35..37.5", kind="seat"),
           dict(part="fp5_motor_tower", feature="bore 70.2 on the OD 69.95", kind="slip"),
           dict(part="fp5_motor_plate", feature="top face Z 41 clamped by the plate", kind="clamp")],
    loads=[dict(where=[36.5, 0, 36], direction=[0, 0, 1], newton=15, note="bearing preload")],
    print_face="-z (underside, Z 36)",
    print_rotate=(0, 0, 0),
    material="PETG",
    nonstructural=[],
)


def build():
    body = Cylinder(OD / 2, Z1 - Z0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, Z0))) - \
        Cylinder(ID / 2, 40).moved(Location((0, 0, Z0)))
    body = one(body)
    top = [e for e in body.edges() if e.geom_type.name == "CIRCLE" and abs(e.center().Z - Z1) < 1e-6
           and abs(e.radius - OD / 2) < 1e-3]
    body, _ = fillet_safe(body, 1.0, top)
    body = one(body)
    body.label = "fp_bearing_retainer"
    return body
