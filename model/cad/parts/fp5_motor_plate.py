"""fp5_motor_plate: motor plate (custom part 2 of 3).

A 6.5 mm disc, OD 80, sitting on the tower top (Z 41..47.5). It clamps the
fp_bearing_retainer sleeve down onto the upper 6814 inner race, takes four
M3 x 12 into the tower lug inserts (r 27.4 at 0/90/180/270 deg) and four
M3 x 10 down into the NEMA 17 face holes (31.0 square). The NEMA 17 hangs
face-up under it, pilot in the 22.5 centre bore. Prints flat on its underside.
"""
import math

from build123d import Align, Cone, Cylinder, Location

from parts import fp5_motor_tower as TOWER
from parts._util import fillet_safe, one

Z0, T = 41.0, 6.5
OD, PILOT_D, HOLE_D = 80.0, 22.5, 3.4
MOTOR_HOLES = [(sx * 15.5, sy * 15.5) for sx in (1, -1) for sy in (1, -1)]
LUG_HOLES = TOWER.INSERTS

SPEC = dict(
    name="fp5_motor_plate",
    function="Hangs the NEMA 17 by its face from the tower lugs and clamps the bearing retainer.",
    mates=[dict(part="fp5_motor_tower", feature="underside on the tower top Z 41, four lug inserts", kind="heat_set_insert x4"),
           dict(part="v_nema17", feature="underside on the motor face, 22.5 pilot bore, 31 square", kind="tapped x4"),
           dict(part="fp_bearing_retainer", feature="underside clamps the sleeve top Z 41", kind="clamp")],
    loads=[dict(where=[0, 0, 41], direction=[0, 0, -1], newton=8, note="motor weight 0.5 kg x 1.6 shock"),
           dict(where=[15.5, 15.5, 41], direction=[0, 1, 0], newton=10, note="reaction torque 0.5 N m at r 22 / 4 screws")],
    print_face="-z (underside, Z 41)",
    print_rotate=(0, 0, 0),
    material="PETG",
    nonstructural=[],
)


def build():
    body = Cylinder(OD / 2, T, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, Z0)))
    body = body - Cylinder(PILOT_D / 2, 40).moved(Location((0, 0, Z0)))
    # 50 deg chamfer on the underside of the pilot bore: clears the R1 fillet at
    # the root of the motor's 22 mm pilot (measured: r 12 at the face, r 11 at +1)
    body = body - Cone(PILOT_D / 2 + 1.25, PILOT_D / 2, 1.5, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, Z0 - 1e-3)))
    for x, y in MOTOR_HOLES + LUG_HOLES:
        body = body - Cylinder(HOLE_D / 2, 40).moved(Location((x, y, Z0)))
    body = one(body)
    top = [e for e in body.edges() if e.geom_type.name == "CIRCLE" and abs(e.center().Z - (Z0 + T)) < 1e-6
           and abs(e.radius - OD / 2) < 1e-3]
    body, _ = fillet_safe(body, 1.0, top)
    body = one(body)
    body.label = "fp5_motor_plate"
    return body
