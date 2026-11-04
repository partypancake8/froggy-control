"""fp5_motor_tower: the fixed motor tower (custom part 1 of 3).

Base flange 120 x 70 x 6 (Z 0..6) on the two cross rails, four M3 x 8
counterbored down through it at (X +-45, Y +-20) into slot nuts. A ring rises
from the flange around the hanging NEMA 17 (ID 60.5): lower shoulder OD 71.6
(Z 6..16) under the lower 6814 inner race, bearing seat OD 69.95 (Z 16..36,
two 6814 stacked face to face), plain OD 69.95 (Z 36..41) under the
fp_bearing_retainer sleeve. Four lugs inside the ring top (Z 22..41, on the
motor flats at 0/90/180/270 deg, inner face r 22.2, 51 deg underside) carry
M3 heat-set inserts at r 27.4 for the motor plate. Prints flange-down.
"""
import math

from build123d import Align, Box, Cylinder, Location, Plane, Polygon, extrude

from lib import hardware as HW
from parts._util import fillet_safe, one, revolve_rz

FLANGE = (120.0, 70.0, 6.0)
RING_ID = 60.5
SHOULDER_OD, SHOULDER_Z = 71.6, (6.0, 16.0)
SEAT_OD = 69.95
TOP_Z = 41.0
SCREWS = [(sx * 45.0, sy * 20.0) for sx in (1, -1) for sy in (1, -1)]
CBORE_D, CBORE_DEPTH, HOLE_D = 6.5, 2.0, 3.4
LUG_ANGLES = (0, 90, 180, 270)
LUG_R_IN, LUG_R_OUT, LUG_HALF_W = 22.2, 31.0, 5.5
LUG_Z_FULL, LUG_Z_LOW = 33.0, 22.0
INSERT_R = 27.4
INSERTS = [(INSERT_R * math.cos(math.radians(a)), INSERT_R * math.sin(math.radians(a))) for a in LUG_ANGLES]

SPEC = dict(
    name="fp5_motor_tower",
    function="Fixed motor tower: bolts to the two cross rails, carries both 6814 inner races on its OD and "
             "hangs nothing itself; the motor plate on its top lugs carries the NEMA 17.",
    mates=[dict(part="s_rail_1515_5/6", feature="flange underside on the cross-rail tops", kind="slot_nut x4"),
           dict(part="v_bearing_6814 x2", feature="OD 69.95 seat Z 16..36, shoulder face Z 16", kind="seat"),
           dict(part="fp_bearing_retainer", feature="plain OD 69.95 Z 36..41", kind="slip"),
           dict(part="fp5_motor_plate", feature="four lug inserts at r 27.4, top face Z 41", kind="heat_set_insert x4")],
    loads=[dict(where=[0, 0, 16], direction=[0, 0, -1], newton=40, note="rotor weight + preload through the shoulder"),
           dict(where=[0, 0, 41], direction=[1, 0, 0], newton=10, note="motor reaction torque via the plate")],
    print_face="-z (flange underside, Z 0)",
    print_rotate=(0, 0, 0),
    material="PETG",
    inserts=dict(thread="M3", pocket_d=HW.INSERT["M3"]["pocket_d"], positions=INSERTS, top_z=TOP_Z),
    nonstructural=[],
)


def build():
    fx, fy, ft = FLANGE
    flange = Box(fx, fy, ft, align=(Align.CENTER, Align.CENTER, Align.MIN))
    flange = fillet_safe(flange, 5.0, flange.edges().filter_by(lambda e: abs(e.tangent_at(0).Z) > 0.99))[0]
    ring = revolve_rz([(RING_ID / 2, 0), (SHOULDER_OD / 2, 0), (SHOULDER_OD / 2, SHOULDER_Z[1]),
                       (SEAT_OD / 2, SHOULDER_Z[1]), (SEAT_OD / 2, TOP_Z), (RING_ID / 2, TOP_Z)])
    body = flange + ring
    # lugs: full block Z 33..41 plus a 51 deg wedge underneath, inside the ring
    xz = Plane(origin=(0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
    prof = Polygon((LUG_R_IN, TOP_Z), (LUG_R_OUT, TOP_Z), (LUG_R_OUT, LUG_Z_LOW), (LUG_R_IN, LUG_Z_FULL), align=None)
    lug = extrude(xz * prof, amount=LUG_HALF_W, both=True)
    for a in LUG_ANGLES:
        body = body + lug.rotate(_z(), a)
    body = one(body)
    # flange opening for the motor body
    body = body - Cylinder(RING_ID / 2, 40, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    # counterbored M3 holes down through the flange
    for x, y in SCREWS:
        body = body - Cylinder(HOLE_D / 2, 20).moved(Location((x, y, 0)))
        body = body - Cylinder(CBORE_D / 2, CBORE_DEPTH + 1, align=(Align.CENTER, Align.CENTER, Align.MIN)) \
            .moved(Location((x, y, ft - CBORE_DEPTH)))
    # heat-set insert pockets in the lugs
    ins = HW.INSERT["M3"]
    for x, y in INSERTS:
        body = body - Cylinder(ins["pocket_d"] / 2, ins["pocket_depth"] + 1,
                               align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((x, y, TOP_Z - ins["pocket_depth"])))
    body = one(body)
    # 1 mm fillets: flange-ring root (concave circle at Z 6) and the flange top outside edges
    root = [e for e in body.edges() if e.geom_type.name == "CIRCLE" and abs(e.center().Z - ft) < 1e-6
            and abs(e.radius - SHOULDER_OD / 2) < 1e-3]
    body, _ = fillet_safe(body, 1.0, root)
    top_out = [e for e in body.edges() if abs(e.bounding_box().min.Z - ft) < 1e-6 and abs(e.bounding_box().max.Z - ft) < 1e-6
               and (abs(abs(e.center().X) - fx / 2) < 5.01 or abs(abs(e.center().Y) - fy / 2) < 5.01)
               and max(abs(e.center().X) - fx / 2 + 5, abs(e.center().Y) - fy / 2 + 5) > -1e-3
               and math.hypot(e.center().X, e.center().Y) > SHOULDER_OD / 2 + 1]
    body, _ = fillet_safe(body, 1.0, top_out)
    body = one(body)
    body.label = "fp5_motor_tower"
    return body


def _z():
    from build123d import Axis
    return Axis.Z
