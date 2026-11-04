"""fp7_pedestal: hollow bearing pedestal on the cross rails (v7, replaces fp6_bearing_housing).

Derived from fp6_bearing_housing: the same bore (coupling pocket r 12.5 to Z 14.5,
51 deg print cone, lower 608 at Z 16.31..23.31, 18 bore, upper 608 at Z
46.31..53.31, 33 ID hub cup) and the same Qi TX coil seat (coil Z 69.01..71.31 on
the cup wall, a 4.5 mm lip round it to Z 71.31, 3.0 under the carrier centre).
New: 120 x 120 x 6 flange on both cross rails (4 x M3 x 10 into top-slot nuts at
X +-45, Y +-50); a 110 OD body with 6 mm walls; an annular cavity r 19.5..49 whose
roof is a 51 deg cone (prints upright with no supports) holding the Qi TX board
on two M2.5 bosses at Z 20; a top plate to Z 68 with a 33 x 15 hall pocket
(A3144 face flush at Z 68, header down a slot into the cavity); an 8 mm
grommeted cable hole through the flange at (-24, -30).
"""
from build123d import Align, Box, Cylinder, Location, Plane, Polygon, extrude

from parts import fp6_datums as D6
from parts._util import fillet_safe, one, revolve_rz

L = 120.0
T = D6.FLANGE_T                      # 6
R_OUT, WALL = 55.0, 6.0
R_CAV_IN, R_CAV_OUT = 19.5, R_OUT - WALL
TOP_Z, CAV_TOP = 68.0, 61.5
CONE_K = 1.25
LIP_R_IN, LIP_R_OUT = 20.5, 25.0
LIP_TOP = D6.TX_Z1                   # 71.31
HOLE_D = 3.4
RAIL_HOLES = [(sx * 45.0, sy * 50.0) for sx in (1, -1) for sy in (1, -1)]
CABLE_XY, CABLE_D = (-24.0, -30.0), 8.0
TX_BOSSES = [(-35.0, -10.0), (-35.0, 10.0)]
TX_BOSS_D, TX_BOARD_Z = 9.0, 20.0
TX_BOARD = (20.0, 30.0, 1.6)         # x, y, t; centred (-35, 0)
HALL_C = (47.0, 0.0)                 # pocket centre; the sensor sits at X 48 (magnet radius)
HALL_POCKET = (15.0, 33.0, 3.12)
HALL_HEADER = (-12.0, 3.5, 8.6)      # slot under the board's -Y end: y offset, x width, y length
ACCESS_HALF_W, ACCESS_HALF_H = 2.0, 2.4

SPEC = dict(
    name="fp7_pedestal",
    function="Holds the 608-2RS pair 30 mm apart on the cross rails, seats the Qi TX coil, the TX board "
             "and the A3144 hall sensor.",
    mates=[dict(part="cross rails", feature="flange underside Z 0, 4 x M3 x 10 into top-slot nuts", kind="slot_nut x4"),
           dict(part="v_bearing_608 x2", feature="22.1 pockets Z 16.31..23.31 and 46.31..53.31", kind="press"),
           dict(part="v_qi_tx_coil", feature="cup wall top Z 69.01, lip to 71.31", kind="seat"),
           dict(part="v_qi_pcb_tx", feature="two M2.5 bosses, board at Z 20", kind="seat"),
           dict(part="v_hall_a3144", feature="33 x 15 x 3.12 pocket at X 47, sensor face flush at Z 68", kind="seat")],
    loads=[dict(where=[0, 0, 46.3], direction=[0, 0, -1], newton=25, note="rotor 1.2 kg x 2 shock on the upper shoulder"),
           dict(where=[0, 0, 50], direction=[1, 0, 0], newton=15, note="rotor imbalance at 600 rpm")],
    print_face="-z (flange underside, Z 0)",
    print_rotate=(0, 0, 0),
    material="PETG",
    nonstructural=[dict(box=[-60, -ACCESS_HALF_W - 1.0, D6.COLLAR_Z0 + D6.COLLAR_W / 2 - 4,
                             -D6.MID_BORE_R + 0.5, ACCESS_HALF_W + 1.0, D6.UPPER_BRG_Z0 + 0.1],
                        reason="bridge over the 4 mm set-screw access window (collar set screw); the shoulder ring "
                               "is carried by the solid either side of the window")],
)


def _bore():
    r0, z = D6.POCKET_R, D6.POCKET_TOP
    lb, ub, bw = D6.LOWER_BRG_Z0, D6.UPPER_BRG_Z0, D6.BRG_W
    rb, rm = D6.BRG_POCKET_R, D6.MID_BORE_R
    return [(0, -1), (r0, -1), (r0, z), (rb, lb), (rb, lb + bw), (rm, lb + bw + CONE_K * (rb - rm)), (rm, ub), (rb, ub),
            (rb, ub + bw), (D6.CUP_R_IN, ub + bw), (D6.CUP_R_IN, LIP_TOP + 5), (0, LIP_TOP + 5)]


def _outer():
    return [(0, T - 0.5), (R_OUT, T - 0.5), (R_OUT, TOP_Z), (LIP_R_OUT, TOP_Z), (LIP_R_OUT, LIP_TOP),
            (LIP_R_IN, LIP_TOP), (LIP_R_IN, D6.TX_Z0), (0, D6.TX_Z0)]


def _cavity():
    return [(R_CAV_IN, T), (R_CAV_OUT, T), (R_CAV_OUT, CAV_TOP),
            (R_CAV_IN, CAV_TOP - CONE_K * (R_CAV_OUT - R_CAV_IN))]


def build():
    body = Box(L, L, T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    vert = [e for e in body.edges() if e.geom_type.name == "LINE" and abs(e.tangent_at(0).Z) > 0.99]
    body, _ = fillet_safe(body, 8.0, vert)
    body = body + revolve_rz(_outer())
    body = body - revolve_rz(_cavity())
    for x, y in TX_BOSSES:
        body = body + Cylinder(TX_BOSS_D / 2, TX_BOARD_Z - T, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((x, y, T)))
        body = body - Cylinder(1.1, 10.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(
            Location((x, y, TX_BOARD_Z)))
    body = body - revolve_rz(_bore())
    for x, y in RAIL_HOLES:
        body = body - Cylinder(HOLE_D / 2, 40).moved(Location((x, y, 0)))
    body = body - Cylinder(CABLE_D / 2, 40).moved(Location((CABLE_XY[0], CABLE_XY[1], 0)))
    # hall pocket (sensor face flush with the top) + header slot down into the cavity
    px, py, pd = HALL_POCKET
    x0 = HALL_C[0] - px / 2                 # open to the rim: the board slides in radially
    body = body - Box(R_OUT + 5 - x0, py, pd + 5, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
        Location((x0, HALL_C[1], TOP_Z - pd)))
    hy, hw, hl = HALL_HEADER
    body = body - Box(hw, hl, 12.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(
        Location((HALL_C[0], hy, TOP_Z - pd + 0.1)))
    # set-screw window for the shaft collar (diamond, 50 deg faces), along -X through the wall
    zc = D6.COLLAR_Z0 + D6.COLLAR_W / 2
    pl = Plane(origin=(-D6.MID_BORE_R + 1, 0, zc), x_dir=(0, 1, 0), z_dir=(-1, 0, 0))
    dia = Polygon((ACCESS_HALF_W, 0), (0, ACCESS_HALF_H), (-ACCESS_HALF_W, 0), (0, -ACCESS_HALF_H), align=None)
    body = body - extrude(pl * dia, amount=R_OUT + 5, dir=pl.z_dir)
    body = one(body)
    body.label = "fp7_pedestal"
    return body


def hall_origin():
    """Board underside centre of the A3144 module in the pocket."""
    return (HALL_C[0], HALL_C[1], TOP_Z - HALL_POCKET[2])


def tx_board_origin():
    return (TX_BOSSES[0][0], 0.0, TX_BOARD_Z)
