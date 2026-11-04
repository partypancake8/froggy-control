"""fp6_motor_plate: NEMA 17 hanger under the deck (custom part 1 of 3, v6).

120 x 70 x 6 PETG plate at Z -21..-15, screwed UP into the bottom slots of
the two cross rails (X +-45) with four M3 x 10 into slot nuts at Y +-25. The
STEPPERONLINE 17HS19-2004D-E1K hangs face-up under it: pilot in the 22.5 bore
(a 60 deg relief clears the pilot's R1 root fillet), four M3 x 10 from above
tapped into the motor face (31 square), each on a 10 mm, 0.4 pad so the screw tip
stops inside the motor's 3.75 modelled thread. Above the pilot the bore opens
on a 45 deg cone to 25.5 so the 24 OD jaw coupling sinks into the plate.
Prints flat on its underside (Z -21), no supports.
"""
from build123d import Align, Box, Cone, Cylinder, Location

from parts import fp6_datums as D6
from parts._util import fillet_safe, one

L, W, T = 120.0, 70.0, D6.PLATE_T
Z0 = D6.PLATE_Z0
PILOT_D, HOLE_D, PAD_D = 22.5, 3.4, 10.0
BORE_D = 25.5
MOTOR_HOLES = [(sx * 15.5, sy * 15.5) for sx in (1, -1) for sy in (1, -1)]
RAIL_HOLES = [(sx * 45.0, sy * 25.0) for sx in (1, -1) for sy in (1, -1)]

SPEC = dict(
    name="fp6_motor_plate",
    function="Hangs the NEMA 17 face-up under the cross rails and lets the jaw coupling sink into its bore.",
    mates=[dict(part="s_rail_1515_5/6 (cross rails)", feature="top face on the rail bottoms, 4 x M3 x 10 slot nuts", kind="slot_nut x4"),
           dict(part="v_nema17", feature="underside on the motor face, 22.5 pilot bore, 31 square", kind="tapped x4")],
    loads=[dict(where=[0, 0, -21], direction=[0, 0, -1], newton=8, note="motor 0.5 kg x 1.6 shock"),
           dict(where=[15.5, 15.5, -21], direction=[0, 1, 0], newton=10, note="0.5 N m reaction over 4 screws")],
    print_face="-z (underside, Z -21)",
    print_rotate=(0, 0, 0),
    material="PETG",
    nonstructural=[],
)


def build():
    body = Box(L, W, T, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, Z0)))
    vert = [e for e in body.edges() if e.geom_type.name == "LINE" and abs(e.tangent_at(0).Z) > 0.99]
    body, _ = fillet_safe(body, 6.0, vert)
    for x, y in MOTOR_HOLES:
        body = body + Cylinder(PAD_D / 2, D6.PLATE_PAD, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((x, y, Z0 + T)))
    zc = Z0 + 2.0                                  # top of the motor pilot
    body = body - Cylinder(PILOT_D / 2, 10).moved(Location((0, 0, Z0)))
    # 60 deg relief for the pilot's root fillet (r 12 at the face, r 11 at +1)
    body = body - Cone(12.1, PILOT_D / 2, 1.5, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, Z0 - 1e-3)))
    rise = (BORE_D - PILOT_D) / 2
    body = body - Cone(PILOT_D / 2, BORE_D / 2, rise, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, zc)))
    body = body - Cylinder(BORE_D / 2, T, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, zc + rise)))
    for x, y in MOTOR_HOLES + RAIL_HOLES:
        body = body - Cylinder(HOLE_D / 2, 40).moved(Location((x, y, Z0)))
    body = one(body)
    body.label = "fp6_motor_plate"
    return body
