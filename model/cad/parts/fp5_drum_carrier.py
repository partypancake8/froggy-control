"""fp5_drum_carrier: rotating drum carrier (custom part 3 of 3), from the
accepted fusion-pipeline fp_drum_carrier.

Skirt ID 90.4 over both 6814 outer races (Z 14..55.4), shoulder ring ID 84
(Z 36..53.5) on the upper outer race, flat deck top Z 64.5 (print bed face)
carrying the camera carrier on four M3 inserts at (+-24, +-24), the acrylic
tube in a 51 deg V groove (tube end at Z 60.5, self-centring, prints without
supports), twelve 5.2 LED wells under the tube end, the 6.2 x 3.3 index
magnet pocket at +X, a pocket keying the ServoCity hub flange (24 square, R4)
with four M3 x 16 screws and hex nuts through the hub's flange holes, and a
set-screw access channel along -X. The coil pocket of the accepted part is
dropped: the hub now occupies its centre.
"""
import math

from build123d import Align, Cylinder, Location, Plane, Polygon, extrude

from lib import hardware as HW
from parts import datums as D
from parts._util import fillet_safe, one, revolve_rz

R_OUT = 82.9          # OD 165.8: 3.5 mm rim outside the V groove
SKIRT_IN, SKIRT_OUT, SKIRT_Z0 = 45.2, 52.0, 14.0
SHOULDER_IN, SHOULDER_Z = 42.0, 36.0
OUTER_DECK_Z0 = 55.4
V_R, V_HALF, V_DEPTH = 74.6, 4.8, 6.0
SHAFT_HOLE_D = 8.0
HUB_POCKET, HUB_POCKET_R = 24.3, 4.15
HUB_HOLES = [(sx * 8.0, sy * 8.0) for sx in (1, -1) for sy in (1, -1)]
CAM_INSERTS = [(sx * 24.0, sy * 24.0) for sx in (1, -1) for sy in (1, -1)]
LED_D, LED_ANGLES = 5.2, [15 + 30 * i for i in range(12)]
MAGNET_D, MAGNET_DEPTH, MAGNET_Z = 6.2, 3.3, 21.5
CHANNEL_HALF_W, CHANNEL_HALF_H = 1.6, 1.85

SPEC = dict(
    name="fp5_drum_carrier",
    function="Rotor carrier: rides both 6814 outer races, takes motor torque from the hub flange key and "
             "screws, carries the acrylic drum, the camera carrier and the LED wells.",
    mates=[dict(part="v_bearing_6814 x2", feature="skirt ID 90.4 Z 14..36, shoulder face Z 36 on the upper outer race", kind="slip/seat"),
           dict(part="v_servocity_1308 hub", feature="24.3 square pocket Z 53.5..57.5 + 4 x M3 x 16 + hex nuts", kind="hex_nut x4"),
           dict(part="s_drum_tube", feature="51 deg V groove r 69.8..79.4, tube end at Z 60.5", kind="seat"),
           dict(part="fp_camera_carrier", feature="four M3 inserts at (+-24, +-24) flush with Z 64.5", kind="heat_set_insert x4")],
    loads=[dict(where=[74.6, 0, 60.5], direction=[0, 0, -1], newton=15, note="drum + disc + ring weight x 1.5"),
           dict(where=[0, 76, 61], direction=[1, 0, 0], newton=8, note="drum spin-up inertia at the tube"),
           dict(where=[8, 8, 57.5], direction=[0, 1, 0], newton=30, note="0.5 N m motor torque over the hub key")],
    print_face="+z (deck top, Z 64.5, printed upside down)",
    print_rotate=(180, 0, 0),
    material="PETG",
    inserts=dict(thread="M3", pocket_d=HW.INSERT["M3"]["pocket_d"], positions=CAM_INSERTS, top_z=D.DECK_TOP_Z),
    nonstructural=[],
)


def _profile():
    t = D.DECK_TOP_Z
    return [(SHAFT_HOLE_D / 2, D.DECK_BOTTOM_Z), (SHOULDER_IN, D.DECK_BOTTOM_Z), (SHOULDER_IN, SHOULDER_Z),
            (SKIRT_IN, SHOULDER_Z), (SKIRT_IN, SKIRT_Z0), (SKIRT_OUT, SKIRT_Z0), (SKIRT_OUT, OUTER_DECK_Z0),
            (R_OUT, OUTER_DECK_Z0), (R_OUT, t), (V_R + V_HALF, t), (V_R, t - V_DEPTH), (V_R - V_HALF, t),
            (SHAFT_HOLE_D / 2, t)]


FILLETS = {}


def build():
    body = revolve_rz(_profile())
    # 1 mm fillets on the clean revolve: skirt bottom edges, outer deck underside edge, skirt-to-deck root
    def circ(z, r):
        return [e for e in body.edges() if e.geom_type.name == "CIRCLE" and abs(e.center().Z - z) < 1e-6
                and abs(e.radius - r) < 1e-3]
    for key, edges in (("skirt_bottom", circ(SKIRT_Z0, SKIRT_OUT) + circ(SKIRT_Z0, SKIRT_IN)),
                       ("deck_edge", circ(OUTER_DECK_Z0, R_OUT)), ("skirt_root", circ(OUTER_DECK_Z0, SKIRT_OUT))):
        body, FILLETS[key] = fillet_safe(body, 1.0, edges)
    # hub flange key pocket (flange-up hub: flange Z 53.5..57.5)
    pocket = extrude(_rounded_square(HUB_POCKET, HUB_POCKET_R, D.DECK_BOTTOM_Z - 1),
                     amount=D.HUB_TOP_Z - D.DECK_BOTTOM_Z + 1)
    body = body - pocket
    for x, y in HUB_HOLES:
        body = body - Cylinder(1.7, 40).moved(Location((x, y, D.DECK_TOP_Z)))
    ins = HW.INSERT["M3"]
    for x, y in CAM_INSERTS:
        body = body - Cylinder(ins["pocket_d"] / 2, 40).moved(Location((x, y, D.DECK_TOP_Z)))
    # LED wells: through the floor under the V
    for a in LED_ANGLES:
        x, y = V_R * math.cos(math.radians(a)), V_R * math.sin(math.radians(a))
        body = body - Cylinder(LED_D / 2, 20).moved(Location((x, y, OUTER_DECK_Z0)))
    # index magnet pocket, radial at +X in the skirt
    mp = Plane(origin=(SKIRT_OUT + 1, 0, MAGNET_Z), x_dir=(0, 1, 0), z_dir=(-1, 0, 0))
    body = body - mp * Cylinder(MAGNET_D / 2, MAGNET_DEPTH + 1, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # set-screw access channel: rhombus section (49 deg faces), along -X at the set screw height
    ch = Plane(origin=(-2.0, 0, 0), x_dir=(0, 1, 0), z_dir=(-1, 0, 0))
    zc = D.HUB_FLANGE_BOTTOM_Z
    rh = Polygon((CHANNEL_HALF_W, zc), (0, zc + CHANNEL_HALF_H), (-CHANNEL_HALF_W, zc), (0, zc - CHANNEL_HALF_H),
                 align=None)
    body = body - extrude(ch * rh, amount=60)
    body = one(body)
    body.label = "fp5_drum_carrier"
    return body


def _rounded_square(size, r, z):
    from build123d import RectangleRounded
    return Plane.XY.offset(z) * RectangleRounded(size, size, r)
