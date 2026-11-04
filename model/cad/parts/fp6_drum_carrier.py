"""fp6_drum_carrier: rotor plate (custom part 3 of 3, v6).

OD 165, PETG. The 6 mm centre (Z 74.31..80.31) sits on the goBILDA 1310 hub's
tapped lobe face and is held by four M4 x 14 countersunk screws (16 mm
pattern) in 50 deg countersinks, heads flush with the deck so nothing stands
inside the camera cradle. Underside: 40 x 34 recess for the Qi RX coil
(coaxial, 3 mm over the TX coil), a 6 x 3 magnet pocket in a 1 mm boss at
r 48 (+X) that passes 1.5 mm over the A3144. The rim (r 66..82.5) is 9 thick
for the 51 deg V groove that centres the acrylic tube (tube end 4 under the
deck) with twelve 5.2 IR LED wells through its floor; eight windows between
14 mm spokes keep it light. Deck top = print bed face (printed upside down);
camera carrier on four M3 inserts at (+-24, +-24).
"""
import math

from build123d import Align, Box, Cone, Cylinder, Location, Rot

from lib import hardware as HW
from parts import fp6_datums as D6
from parts._util import one, revolve_rz

R_OUT, R_RIM_IN = 82.5, 66.0
HUB_HOLES = [(sx * 8.0, sy * 8.0) for sx in (1, -1) for sy in (1, -1)]
CAM_INSERTS = [(sx * 24.0, sy * 24.0) for sx in (1, -1) for sy in (1, -1)]
M4_HOLE_R = 2.25
CSK_K = 1 / math.tan(math.radians(50))   # countersink dr/dz (50 deg wall, head is 45: seats on its rim)
CSK_R_AT_HEAD_EDGE = 3.73                # countersink radius 0.29 under the deck: head rim bears on the cone (gap 0.015)
LED_D, LED_ANGLES = 5.2, [15 + 30 * i for i in range(12)]
RX_RECESS = (D6.COIL_ID / 2 - 0.1, D6.COIL_OD / 2 + 0.1, D6.COIL_T + 0.05)
MAGNET_POCKET = (3.05, 3.05)
WIN_R0, WIN_R1, SPOKE_W = 26.0, 62.0, 14.0
# v7 edits: two magnet bosses (+X index, -X balance) reaching down to Z 69.5 so the
# magnets pass 1.5 over the A3144 face (pedestal top Z 68); a 32 wide lug at -Y
# (r 80..97) with 2 M3 inserts and a 6 x 4 wire slot for fp7_wire_spine; a matching
# lug at +Y with 4 M3 tapped trim holes (balance weights)
MAGNET_Z0_V7 = 69.5
MAGNET_XS = (D6.MAGNET_R, -D6.MAGNET_R)
LUG_W, LUG_R0, LUG_R1 = 32.0, 80.0, 97.0
SPINE_INSERTS = [(sx * 10.0, -88.0) for sx in (1, -1)]
SPINE_SLOT = (6.0, 4.0, -89.0)
TRIM_HOLES = [(x, 88.5) for x in (-11.0, -4.0, 4.0, 11.0)]
TRIM_D = 2.5

SPEC = dict(
    name="fp6_drum_carrier",
    function="Rotor plate: takes torque from the 8 mm hub, carries the drum tube, camera carrier, Qi RX coil, "
             "index magnet and 12 IR LEDs.",
    mates=[dict(part="v_gobilda_1310_hub_8mm", feature="centre underside on the lobe face, 4 x M4 x 14 csk", kind="tapped x4"),
           dict(part="s_drum_tube", feature="51 deg V groove r 69.8..79.4, tube end 4 under the deck", kind="seat"),
           dict(part="fp_camera_carrier", feature="four M3 inserts at (+-24, +-24) flush with the deck", kind="heat_set_insert x4"),
           dict(part="v_qi_rx_coil", feature="recess r 16.4..20.1 in the underside", kind="seat"),
           dict(part="v_magnet_6x3", feature="6.1 x 3.05 pocket at (48, 0) in a 1 mm boss", kind="press")],
    loads=[dict(where=[74.6, 0, 78.3], direction=[0, 0, -1], newton=15, note="drum + disc + ring x 1.5"),
           dict(where=[8, 8, 74.3], direction=[0, 1, 0], newton=40, note="0.5 N m over the M4 pattern")],
    print_face="+z (deck top, printed upside down)",
    print_rotate=(180, 0, 0),
    material="PETG",
    inserts=dict(thread="M3", pocket_d=HW.INSERT["M3"]["pocket_d"], positions=CAM_INSERTS, top_z=D6.DECK_TOP_Z),
    nonstructural=[dict(box=[-10.0, -10.0, D6.CARRIER_Z0 - 0.1, 10.0, 10.0, D6.CARRIER_Z0 + D6.HUB_PILOT_H + 0.4],
                        reason="2.3 mm locating lip between the hub pilot-boss pocket (r 7.3) and the M4 clearance "
                               "holes (16 mm pattern leaves 1.8 mm); the 3.7 mm plate above it carries the load")],
)


def _profile():
    z0, t, rz = D6.CARRIER_Z0, D6.DECK_TOP_Z, D6.RIM_Z0
    return [(0, z0), (R_RIM_IN, z0), (R_RIM_IN, rz), (R_OUT, rz), (R_OUT, t), (D6.V_R + D6.V_HALF, t),
            (D6.V_R, t - D6.V_DEPTH), (D6.V_R - D6.V_HALF, t), (0, t)]


def build():
    t, z0 = D6.DECK_TOP_Z, D6.CARRIER_Z0
    body = revolve_rz(_profile())
    for mx in MAGNET_XS:
        body = body + Cylinder(6.5, z0 - MAGNET_Z0_V7 + 0.5, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((mx, 0.0, MAGNET_Z0_V7)))
    for sy in (1, -1):
        body = body + Box(LUG_W, LUG_R1 - LUG_R0, t - D6.RIM_Z0, align=(Align.CENTER, Align.MIN, Align.MIN)).moved(
            Location((0, LUG_R0 if sy > 0 else -LUG_R1, D6.RIM_Z0)))
    # windows between 14 mm spokes (spokes at 0, 45, ... deg carry the inserts and the magnet)
    win = Cylinder(WIN_R1, 40) - Cylinder(WIN_R0, 50)
    for a in (0, 45, 90, 135):
        win = win - Rot(0, 0, a) * Box(2 * WIN_R1 + 10, SPOKE_W, 60)
    body = body - win.moved(Location((0, 0, (z0 + t) / 2)))
    # Qi RX coil recess (underside)
    ri, ro, h = RX_RECESS
    body = body - (Cylinder(ro, h + 1, align=(Align.CENTER, Align.CENTER, Align.MIN)) -
                   Cylinder(ri, 3 * h + 5)).moved(Location((0, 0, z0 - 1)))
    # pocket for the hub's 14 mm x 2 pilot boss (centres the carrier, underside sits on the lobe face)
    body = body - Cylinder(D6.HUB_PILOT_R + 0.3, D6.HUB_PILOT_H + 0.3 + 1,
                           align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, z0 - 1)))
    # magnet pocket
    for mx in MAGNET_XS:
        body = body - Cylinder(MAGNET_POCKET[0], MAGNET_POCKET[1], align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((mx, 0.0, MAGNET_Z0_V7)))
    for x, y in SPINE_INSERTS:
        body = body - Cylinder(HW.INSERT["M3"]["pocket_d"] / 2, 40).moved(Location((x, y, t)))
    body = body - Box(SPINE_SLOT[0], SPINE_SLOT[1], 40).moved(Location((0, SPINE_SLOT[2], t)))
    for x, y in TRIM_HOLES:
        body = body - Cylinder(TRIM_D / 2, 40).moved(Location((x, y, t)))
    # hub screws: M4 clearance + 50 deg countersink
    zc = t - 0.29 - (CSK_R_AT_HEAD_EDGE - M4_HOLE_R) / CSK_K
    r_top = CSK_R_AT_HEAD_EDGE + CSK_K * (0.29 + 1.0)
    for x, y in HUB_HOLES:
        body = body - Cylinder(M4_HOLE_R, 40).moved(Location((x, y, t)))
        body = body - Cone(M4_HOLE_R, r_top, t + 1.0 - zc, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((x, y, zc)))
    ins = HW.INSERT["M3"]
    for x, y in CAM_INSERTS:
        body = body - Cylinder(ins["pocket_d"] / 2, 40).moved(Location((x, y, t)))
    for a in LED_ANGLES:
        x, y = D6.V_R * math.cos(math.radians(a)), D6.V_R * math.sin(math.radians(a))
        body = body - Cylinder(LED_D / 2, 40).moved(Location((x, y, t)))
    body = one(body)
    body.label = "fp6_drum_carrier"
    return body


def led_positions():
    return [(D6.V_R * math.cos(math.radians(a)), D6.V_R * math.sin(math.radians(a))) for a in LED_ANGLES]
