"""Froggy v8 stage C, "rotor power bank": the stage B rotor cartridge (2020 frame, KFL08 flange units on
flat laser-cut plates) with the Qi link and the hall index removed. The rotor carries its own certified
USB-C power bank in a printed cradle, balanced by a steel bar under the disc; drum speed comes from the
closed-loop stepper's encoder (direct drive). KFL08 units and M5 standoffs are modelled from the datasheet.

Datums: Z 0 = deck rail top, X and Y zero on the rotor axis, rotor axis Z.

frame   : 17 rails of 2020 (lib.profile_2020). Deck ring Z -20..0: 2 x 300 along X (Y +-140)
          and 2 x 260 along Y (X +-140) between them; cross rails 2 x 260 along Y at X +-45;
          bottom ring the same at Z -110..-90; four 70 mm legs at (+-140, +-140), Z -90..-20; three
          cage posts (POST_L) standing on the deck ring top at (0, +140), (-140, +70), (+140, +70).
          Every frame joint is a bought cast 2020 corner bracket (plain envelope, 23 in all) with
          one M5 x 8 button head + drop-in T-nut per leg.
feet    : four rubber bumpers (20 x 8) under the bottom ring corners, M5 x 8 + T-nut each.
plinth  : 3 mm black ABS laser-cut sheets, face-mounted: four side panels, a 300 x 300 deck sheet
          on the deck ring top (Z 0..3) and a bottom panel under the bottom ring (Z -113..-110).
drive_base (stage B): lp_motor_plate (aluminium, PLATE_T) under the cross rails, 4 x M5 + T-nut;
          the motor hangs under it on four 7.2 mm spacers (4 x M3 x 14 from above); jaw coupling;
          the rotor cartridge: lp_plate_a on the deck sheet (4 x M5 x 12 through the sheet into the
          cross-rail T-nuts), four 40 mm M5 hex standoffs, lp_plate_b, a KFL08 flange unit on each
          plate (2 x M4 x 12 + hex nut).
rotor   : 8 mm shaft through both KFL08 units, collar on the upper inner ring, goBILDA 1310 hub,
          lp_carrier_disc (aluminium 186, 4 x M4 csk into the hub), Pi / LED driver on the disc,
          printed camera wedge with the Arducam UC-261, fp8_bank_cradle with the Anker 321 power bank
          (-Y), v_balance_bar under the disc (+Y), charge port and rocker switch through the disc rim
          outside the tube facing down; acrylic tube in lp_ring_b, top disc clamped between lp_ring_d
          (under, round the tube top), lp_ring_e (bezel round the disc) and lp_ring_c (on top) by
          3 x M3 x 16 + hex nut; 24 IR LEDs.
cage    : Keiran hoop pairs and the mesh on six printed post clips, raised with the rotor.

Install steps: 0 frame, feet; 1 motor plate, spacers, motor, coupling; 2 deck sheet, then the rotor
cartridge (built on the bench: plate A, lower KFL08, standoffs, plate B, upper KFL08, shaft, collar)
dropped on and bolted down through plate B's driver holes; 3 hub; 4 carrier disc, balance bar, bank
cradle, charge port, switch, LEDs, ring B; 5 camera wedge, Pi, power bank; 6 camera, drum, rings, frogs;
7 side and bottom panels, plinth electrics; 8 cage clips; 9 hoops and mesh.
"""
import copy
import math

import numpy as np
from build123d import Align, Box, Compound, Cylinder, Location, Plane, Solid, extrude

from assemblies import froggy_v7 as V7
from assemblies.drum_base import R_MOTOR, _motor_solids, motor_length
from assemblies.frame import _location
from lib import hardware as HW
from lib import profile_2020 as P
from lib import vendor as V
from lib.assembly import Assembly
from lib.joints import bolt_joint, hex_nut_joint, insert_joint, tapped_joint
from parts import datums as D0
from parts import fp6_datums as D6
from parts import fp6_drum_carrier, fp7_controller_carrier, fp7_estop_legend, s_drum_tube, s_top_disc

_R, _box, _solid, _c, _sg = V7._R, V7._box, V7._solid, V7._c, V7._sg

W = P.W                          # 20
HALF = 150.0                     # 300 x 300 outer
RC = HALF - W / 2                # 140 ring rail centre
RIN = HALF - W                   # 130 ring rail inner face
CROSS_X = 45.0
DECK_Z0, DECK_Z1 = -W, 0.0       # deck ring Z -20..0
BOT_Z0, BOT_Z1 = -110.0, -90.0   # bottom ring
LEG_Z0, LEG_Z1 = BOT_Z1, DECK_Z0  # legs -90..-20 (70)
POSTS = {"back": (0.0, RC), "left": (-RC, 70.0), "right": (RC, 70.0)}
PANEL_T = 3.0
SHEET_Z0, SHEET_Z1 = 0.0, PANEL_T          # deck sheet on the deck ring top
BOTP_Z0, BOTP_Z1 = BOT_Z0 - PANEL_T, BOT_Z0  # bottom panel under the bottom ring
DZ_ROT = SHEET_Z1                # stage A rotor shift (+3), kept as the cage reference
M5_HOLE = 5.5
G_D = V7.G_D
FOOT_D, FOOT_H, FOOT_CB_D, FOOT_CB_H = 20.0, 8.0, 10.5, 5.0

# ------------------------------------------------------------------ stage B: rotor cartridge datums
PLATE_T = 3.175                  # every lp_plate_* / lp_motor_plate / lp_carrier_disc (1/8 in aluminium);
                                 # 6.0 here makes the acrylic prototype set
CENTRE_HOLE_D = 30.0
# drive: plate under the cross rails, motor on spacers under the plate. The spacers drop the motor
# so the 32 mm jaw coupling (motor face + 3) ends under the lower KFL08 (Z PA_Z1); with the motor
# face on the plate the coupling would run 5.5 into the lower bearing unit.
MPLATE_Z1 = DECK_Z0                          # -20: cross-rail underside
MPLATE_Z0 = MPLATE_Z1 - PLATE_T
MPLATE_X, MPLATE_Y = 120.0, 70.0
MPLATE_RAIL_HOLES = [(sx * CROSS_X, sy * 25.0) for sx in (1, -1) for sy in (1, -1)]
MOTOR_HOLES = [(sx * 15.5, sy * 15.5) for sx in (1, -1) for sy in (1, -1)]
MOTOR_SPACER = 7.2                           # M3 x 14 leaves 3.6 in the 3.75 tapped face holes
MOTOR_FACE_Z = MPLATE_Z0 - MOTOR_SPACER      # -30.375 (6 mm plate: -33.2)
COUPLING_BORE_D = 26.0                       # the coupling passes the plate
COUPLING_L, COUPLING_IN = 32.0, 10.5
COUPLING_Z0 = MOTOR_FACE_Z + 3.0
# cartridge
PA_Z0 = SHEET_Z1                             # plate A on the deck sheet
PA_Z1 = PA_Z0 + PLATE_T
PA_RAIL_HOLES = [(sx * CROSS_X, sy * 15.0) for sx in (1, -1) for sy in (1, -1)]
STANDOFF_XY = [(sx * 45.0, sy * 45.0) for sx in (1, -1) for sy in (1, -1)]
STANDOFF_L, STANDOFF_AF = 40.0, 8.0
PB_Z0 = PA_Z1 + STANDOFF_L
PB_Z1 = PB_Z0 + PLATE_T
KFL_J = HW.KFL08["J"]
KFL_HOLES = [(sx * KFL_J / 2, 0.0) for sx in (1, -1)]
KFL_H = HW.KFL08["H"]
DRIVER_HOLE_D = 8.0                          # plate B access for the plate A rail bolts
SHEET_KFL_HOLE_D = 50.0                      # deck sheet centre hole: coupling + the lower KFL08 nuts
SHEET_HEAD_HOLE_D = 10.5                     # deck sheet clearance for the standoff screw heads
# shaft, collar, hub, disc
COLLAR_Z0, COLLAR_W = PB_Z1 + KFL_H, 8.0
HUB_H = 22.0
HUB_TOP_Z = COLLAR_Z0 + COLLAR_W + HUB_H     # pilot boss top
HUB_LOBE_Z = HUB_TOP_Z - D6.HUB_PILOT_H
DISC_Z0, DISC_Z1, DISC_D = HUB_LOBE_Z, HUB_LOBE_Z + PLATE_T, 186.0
HUB_HOLES = [(sx * 8.0, sy * 8.0) for sx in (1, -1) for sy in (1, -1)]
DRUM_FLOOR = DISC_Z1
DZF = DRUM_FLOOR - D6.DRUM_FLOOR_Z           # drum and everything on it vs v7
SHAFT_Z0 = COUPLING_Z0 + COUPLING_L - COUPLING_IN
SHAFT_Z1 = HUB_TOP_Z - 0.8
LED_R = D6.V_R
LED_ANGLES = [15.0 + 30.0 * i for i in range(12)]
LED_HOLE_D = 5.2
# acrylic rings
RING_T = 3.0
RING_B = (153.0, 180.0)
RING_B_R, RING_SCREW_ANG = 84.0, (60.0, 180.0, 300.0)
TUBE_TOP = DRUM_FLOOR + D6.DRUM_H
TOP_DISC_T = D0.ACRYLIC_T
RING_D = (153.0, 186.0)
RING_E = (160.6, 186.0)                      # bezel round the top disc, same sheet thickness as the disc
RING_C = (150.0, 186.0)
CLAMP_R = 86.5
LED2_TIP_R = D0.DISC_OD / 2 + 0.4            # ring LEDs lie radially, dome 0.4 off the disc edge
LED2_SLOT = (80.2, 89.8, 6.2)                # r0, r1, width: lasered through C, E and D
# camera (Arducam UC-261 class, 36 x 36 board; 28.5 mm M2 hole pitch assumed, not measured)
CAM_HOLE_PITCH = 28.5
OPTICAL_AXIS = (0.5, 0.0, math.cos(math.radians(30)))
LENS_ABOVE_FLOOR = 23.0
LENS = (0.0, 0.0, DRUM_FLOOR + LENS_ABOVE_FLOOR)
CAM_STACK = 13.6                              # PCB underside to lens front along the axis
WEDGE_XY = (-26.0, 6.0, -16.0, 16.0)
WEDGE_SCREWS = [(-3.0, 11.0), (-3.0, -11.0), (-21.0, 5.0), (-21.0, -5.0)]
WEDGE_CB_D, WEDGE_CB_FLOOR = 8.0, 3.0
PI_STANDOFF = 6.0
PI_HOLES = [(sx * 29.0, 45.0 + sy * 11.5) for sx in (1, -1) for sy in (1, -1)]
# ------------------------------------------------------------------ stage C: rotor power (no Qi, no slip ring)
# A certified USB-C power bank (Anker 321 / PowerCore 5K, A1112: 96 x 45.2 x 23.1, 118 g, 5200 mAh) in a
# printed cradle on the -Y half of the disc, standing on its long side (96 along X, 23.1 radial, 45.2 up):
# lying flat it does not fit beside the camera board inside the tube (IR 73). Bank USB-A out -> switch ->
# Pi micro-USB; bank USB-C (its input) <- the panel-mount charge port. A steel balance bar under the disc
# at +Y squares the bank's moment (the bank cannot sit on the axis: the camera wedge is there).
BANK_L, BANK_W, BANK_H, BANK_EDGE_R = 96.0, 23.1, 45.2, 5.0
BANK_MASS_G = 118.0
CR_T, CR_FLOOR, CR_GAP, CR_LIP, CR_LIP_T = 2.5, 3.0, 0.3, 1.5, 1.7
CR_Y1 = -19.5                                # cradle axis-side wall face: 1.5 off the camera board edge (y -18)
BANK_X = 0.0
BANK_Y = CR_Y1 - CR_T - CR_GAP - BANK_W / 2  # -33.85
BANK_Z0 = DRUM_FLOOR + CR_FLOOR + CR_GAP     # bank underside
CR_X = BANK_L / 2 + CR_GAP + CR_T            # 50.8: cradle end wall outer faces at BANK_X +- CR_X
CR_Y0 = BANK_Y - BANK_W / 2 - CR_GAP - CR_T  # -48.2: outer (radial) wall face
CR_Z1 = BANK_Z0 + BANK_H + CR_GAP + CR_LIP_T  # lip top
CR_EAR_X1, CR_EAR_HY = 58.0, 8.35            # ears: x CR_X..58, y BANK_Y +- 8.35, CR_FLOOR thick
CR_SCREWS = [(sx * 54.5 + BANK_X, BANK_Y + sy * 5.0) for sx in (1, -1) for sy in (1, -1)]
# balance bar: bright steel flat 16 x 12 under the disc at +Y, 2 x M3 x 8 from the disc top into it
BAR_H, BAR_W = 16.0, 12.0
BAR_L = 56.0
BAR_X, BAR_Y = -3.5, 61.0
BAR_TAP_DEPTH = 8.0
BAR_SCREWS = [(BAR_X + sx * (BAR_L / 2 - 8.0), BAR_Y) for sx in (1, -1)]
# charge port (Adafruit 6069 class round USB-C panel mount) and power switch (KCD1-101 rocker), both through
# the disc outside the tube (tube OR 76.2) facing down, between LED holes and ring B screws
PWR_R = 85.5
PORT_ANG, SWITCH_ANG = 240.0, 0.0
PORT_FLANGE_D, PORT_FLANGE_T, PORT_BODY_D, PORT_NUT_D, PORT_NUT_T = 19.0, 2.5, 12.0, 14.0, 4.0
PORT_HOLE_D = 12.4
SW_BEZEL, SW_BEZEL_T, SW_BODY, SW_BODY_H = (15.0, 21.0), 4.0, (12.8, 18.9), 11.5   # (radial, tangential)
SW_CUT = (13.2, 19.4)
SW_SPOT = (15.0, 23.0, 0.7)                  # spot face in the disc top: the KCD1 clips grip 0.8..3.0 panels
WIRE_HOLE_D = 7.0
WIRE_HOLES = [(PWR_R, PORT_ANG + 9.5), (66.0, PORT_ANG + 5.0), (PWR_R, SWITCH_ANG - 9.5), (66.0, SWITCH_ANG - 5.0)]
# power budget (5 V rail) and bank rating
PI_MA, CAM_MA, LED_MA = 400, 150, 160        # Pi Zero 2 W streaming on Wi-Fi; UC-261; 8 strings x 3 IR LEDs at 20 mA
BANK_WH = 5.2 * 3.7                          # 5200 mAh at 3.7 V nominal = 19.24 Wh (Anker rated)
BANK_EFF = 0.85                              # cell -> 5 V boost inside the bank (assumed)
CARTRIDGE = ["lp_plate_a", "lp_plate_b", "v_kfl08_lower", "v_kfl08_upper"] + \
    [f"v_standoff_m5_{k}" for k in range(1, 5)]
HOOP_RISE = math.ceil(DZF - DZ_ROT)          # cage top hoop and posts follow the taller rotor
POST_L = 270.0 + HOOP_RISE

# Cast 2020 corner bracket, plain envelope. Source: the common die-cast zinc / aluminium "2020
# corner bracket" datasheet class (e.g. OpenBuilds cast corner bracket, Misumi HBLFSN5-class):
# 20 wide, two 28 x 28 legs about 4 thick, one 5.5 hole per leg on the slot centre. The stiffening
# rib is left out of the envelope; dimensions are the class datasheet's, not one part number's.
BR_W, BR_L, BR_T, BR_HOLE_AT = 20.0, 28.0, 4.0, 16.0

# cage
HOOP_Z = {"bottom": 13.0, "top": 255.0 + HOOP_RISE}
HOOP_H = 12.0
HOOP_OUT_R = 110.0
MESH_R = V7.MESH_R
MESH_Z = (HOOP_Z["bottom"] + HOOP_H, HOOP_Z["top"])
MESH_TOP_R = V7.MESH_TOP_R
MESH_TOP_Z = (HOOP_Z["top"] + HOOP_H, HOOP_Z["top"] + HOOP_H + 0.5)
CLIP_T_BACK = 5.0                # clip wall between the post face and the counterbore floor
CLIP_GAP = 0.1                   # concave face radius over the hoop's outer radius
CLIP_LEDGE = 3.0                 # ledge under the hoop pair
CLIP_TOP = 2.0                   # clip above the hoop pair
CLIP_CB_D = 10.5                 # counterbore for the M5 button head (9.5) and its key
LEDGE_R = {"bottom": 104.0, "top": MESH_R[1] + 0.3}        # top ledge stays outside the mesh sleeve

# plinth electrics (bottom panel top Z -110)
FLOOR_Z = BOTP_Z1
CL42T_X0 = V7.CL42T_X0
CL42T_EAR_T = V7.CL42T_EAR_T
PSU_BOX = (34.0, 116.0, 0.0, 99.0, 30.0)     # v7 (34, 116, 5, 104) moved -5 in Y: clears the bottom corner bracket
CC_DX = 8.0                                  # controller carrier moved +8 in X: clears the bottom corner bracket
CC_DZ = FLOOR_Z - fp7_controller_carrier.Z0
PANEL_IN = HALF                              # side panels' inner face on the rails' outer faces
ESTOP_X, PANEL_DEV_Z, ESTOP_Z = 65.0, -55.0, -55.0   # e-stop box Z -90..-20 fills the bay, X clears the leg brackets
IEC = (80.0, 27.4, 31.0, PANEL_IN + PANEL_T - 32.0, PANEL_IN + PANEL_T)
USB = (-60.0, 20.0, 12.0, PANEL_IN + PANEL_T - 20.0, PANEL_IN + PANEL_T)
G2 = V7.G2
SIDES = V7.SIDES
VENT_W, VENT_L, VENT_PITCH, VENT_N, VENT_Z = 3.0, 40.0, 6.0, 15, (-85.0, -45.0)
VENT_H, VENT_GAP = V7.VENT_H, V7.VENT_GAP
ESTOP_HOLE_D, ESTOP_KEY_W, ESTOP_KEY_R = V7.ESTOP_HOLE_D, V7.ESTOP_KEY_W, V7.ESTOP_KEY_R
LEGEND_T = V7.LEGEND_T
DEV_CLEAR, DEV_R = V7.DEV_CLEAR, V7.DEV_R
IEC_PITCH, IEC_FLANGE = V7.IEC_PITCH, V7.IEC_FLANGE
PANEL_SCREW_H, PANEL_SCREW_Z = 100.0, ((DECK_Z0 + DECK_Z1) / 2, (BOT_Z0 + BOT_Z1) / 2)
BOTP_SCREWS = [(sx * 60.0, sy * RC) for sx in (1, -1) for sy in (1, -1)] + \
              [(sx * RC, sy * 60.0) for sx in (1, -1) for sy in (1, -1)]
SHEET_SCREWS = [(sx * 100.0, sy * RC) for sx in (1, -1) for sy in (1, -1)]
M5 = dict(nut="slot_nut", head="button", evaluate=False)


def _up(s, dz):
    return s.moved(Location((0, 0, dz))) if dz else s


def _vec(*a):
    return np.array(a, float)


# ------------------------------------------------------------------ frame

def _frame(A):
    A.meta_role = {}
    z, zb = (DECK_Z0 + DECK_Z1) / 2, (BOT_Z0 + BOT_Z1) / 2
    specs = [("deck_x_back", 2 * HALF, "x", (0, RC, z)), ("deck_x_front", 2 * HALF, "x", (0, -RC, z)),
             ("deck_y_right", 2 * RIN, "y", (RC, 0, z)), ("deck_y_left", 2 * RIN, "y", (-RC, 0, z)),
             ("cross_right", 2 * RIN, "y", (CROSS_X, 0, z)), ("cross_left", 2 * RIN, "y", (-CROSS_X, 0, z)),
             ("bot_x_back", 2 * HALF, "x", (0, RC, zb)), ("bot_x_front", 2 * HALF, "x", (0, -RC, zb)),
             ("bot_y_right", 2 * RIN, "y", (RC, 0, zb)), ("bot_y_left", 2 * RIN, "y", (-RC, 0, zb))]
    for sx in (1, -1):
        for sy in (1, -1):
            specs.append((f"leg_{_c(sx, sy)}", LEG_Z1 - LEG_Z0, "z", (sx * RC, sy * RC, (LEG_Z0 + LEG_Z1) / 2)))
    names = {}
    for n, (role, L, ax, c) in enumerate(specs, 1):
        s = P.rail(L, ax, c, n=n)
        A.add(s.label, s, "rail", "frame", length_mm=L, axis=ax, centre=c, role=role, step=0)
        names[role] = s.label
        A.meta_role[s.label] = role
    for k, (key, (x, y)) in enumerate(POSTS.items(), 1):
        s = P.rail(POST_L, "z", (x, y, POST_L / 2), n=f"post_{key}")
        role = f"post_{key}"
        A.add(s.label, s, "rail", "frame", length_mm=POST_L, axis="z", centre=(x, y, POST_L / 2), role=role,
              step=0, note="cage post, stands on the deck ring top")
        names[role] = s.label
        A.meta_role[s.label] = role
    A.meta_rails = names
    return names


def _bracket_solid():
    leg_a = Box(BR_L, BR_W, BR_T, align=(Align.MIN, Align.CENTER, Align.MIN))       # on face A (local z = nA)
    leg_b = Box(BR_T, BR_W, BR_L, align=(Align.MIN, Align.CENTER, Align.MIN))       # on face B (local x = nB)
    s = leg_a + leg_b
    s = s - Cylinder(M5_HOLE / 2, 3 * BR_T).moved(Location((BR_HOLE_AT, 0, 0)))
    s = s - Plane(origin=(0, 0, BR_HOLE_AT), z_dir=(1, 0, 0)) * Cylinder(M5_HOLE / 2, 3 * BR_T)
    return _solid(s)


_BRACKET = None


def _bracket(A, name, C, nA, nB, rail_a, rail_b, prefix):
    """Cast 2020 corner bracket in the inside corner C between face A (outward normal nA, of rail_a)
    and face B (nB, of rail_b); centred on C across its width. One M5 x 8 + T-nut per leg."""
    global _BRACKET
    if _BRACKET is None:
        _BRACKET = _bracket_solid()
    C, nA, nB = (np.asarray(v, float) for v in (C, nA, nB))
    w = np.cross(nA, nB)
    R = _R([nB, w, nA])
    A.add(name, copy.deepcopy(_BRACKET).moved(_location(R, C)), "bracket", "frame", mates=[rail_a, rail_b], step=0,
          note="bought cast 2020 corner bracket (envelope), zinc plated; 20 wide, 28 x 28 legs, 2 x 5.5")
    A.vendor.append(dict(component=name, source_url=None, author="catalogue (2020 cast corner bracket class)",
                         licence=None, file=None, price_usd=0.6,
                         note="plain envelope from the 2020 cast corner bracket datasheet class"))
    for leg, at, axis, rail in (("a", C + nB * BR_HOLE_AT, -nA, rail_a), ("b", C + nA * BR_HOLE_AT, -nB, rail_b)):
        bolt_joint(A, [name, rail], at, axis, screw="M5x8", name=f"{prefix}_{leg}_{A.meta_role[rail]}",
                   module="frame", contact="faces", step=0, **M5)


def _brackets(A, R_):
    zd, zb = (DECK_Z0 + DECK_Z1) / 2, (BOT_Z0 + BOT_Z1) / 2
    for sx in (1, -1):
        for sy in (1, -1):
            c = _c(sx, sy)
            for lvl, z, xr, yr in (("deck", zd, "deck_x", "deck_y"), ("bot", zb, "bot_x", "bot_y")):
                rx = R_[f"{xr}_{'back' if sy > 0 else 'front'}"]
                ry = R_[f"{yr}_{'right' if sx > 0 else 'left'}"]
                _bracket(A, f"v_cast_bracket_2020_{lvl}_corner_{c}", (sx * RIN, sy * RIN, z), (0, -sy, 0), (-sx, 0, 0),
                         rx, ry, f"j_{lvl}_corner_{c}")
            leg = R_[f"leg_{c}"]
            for end, z, nA, xr in (("top", DECK_Z0, (0, 0, -1), "deck_x"), ("bottom", BOT_Z1, (0, 0, 1), "bot_x")):
                rx = R_[f"{xr}_{'back' if sy > 0 else 'front'}"]
                _bracket(A, f"v_cast_bracket_2020_leg_{end}_{c}", (sx * RIN, sy * RC, z), nA, (-sx, 0, 0), rx, leg,
                         f"j_leg_{end}_{c}")
            cross = R_["cross_right" if sx > 0 else "cross_left"]
            rx = R_[f"deck_x_{'back' if sy > 0 else 'front'}"]
            _bracket(A, f"v_cast_bracket_2020_cross_end_{c}", (sx * (CROSS_X + W / 2), sy * RIN, zd), (0, -sy, 0),
                     (sx, 0, 0), rx, cross, f"j_cross_end_{c}")
    for key, (x, y) in POSTS.items():
        post = R_[f"post_{key}"]
        if key == "back":
            host, C, nB = R_["deck_x_back"], (x + W / 2, y, 0.0), (1, 0, 0)
        else:
            host = R_["deck_y_right" if x > 0 else "deck_y_left"]
            C, nB = (x, y - W / 2, 0.0), (0, -1, 0)
        _bracket(A, f"v_cast_bracket_2020_post_{key}", C, (0, 0, 1), nB, host, post, f"j_post_{key}")


def _feet(A, R_):
    for sx in (1, -1):
        for sy in (1, -1):
            c = _c(sx, sy)
            x, y = sx * RC, sy * RC
            z0 = BOT_Z0 - FOOT_H
            f = Cylinder(FOOT_D / 2, FOOT_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
            f = f - Cylinder(M5_HOLE / 2, 3 * FOOT_H) - Cylinder(FOOT_CB_D / 2, 2 * FOOT_CB_H)
            nm = f"v_foot_bumper_{c}"
            A.add(nm, _solid(f).moved(Location((x, y, z0))), "foot", "feet", step=0,
                  note="bought rubber bumper foot 20 x 8 with a recessed washer (envelope), M5 x 8 into a T-nut")
            A.vendor.append(dict(component=nm, source_url=None, author="catalogue (rubber bumper foot 20 x 8, M5)",
                                 licence=None, file=None, price_usd=0.5))
            rail = R_["bot_x_back" if sy > 0 else "bot_x_front"]
            bolt_joint(A, [nm, rail], (x, y, BOT_Z0), (0, 0, 1), screw="M5x8", name=f"j_foot_{c}_rail",
                       module="feet", contact="faces", step=0, seat=(x, y, z0 + FOOT_CB_H), **M5)


# ------------------------------------------------------------------ plinth panels (face-mounted)

def _plane(side):
    n, h = SIDES[side]
    return Plane(origin=tuple(np.array(n, float) * PANEL_IN), x_dir=h, z_dir=n)


def _hv(side, p):
    return float(np.array(SIDES[side][1], float) @ np.asarray(p, float)), float(p[2])


def _side_profile(side):
    """2D (h, z) profile of a face-mounted side panel: the full plinth face (front / back reach over
    the side panels' edges), 4 x 5.5 screw holes, and the v7 device cutouts and vents."""
    e = HALF + PANEL_T if side in ("front", "back") else HALF
    face = V7._rect(-e, e, BOTP_Z0, SHEET_Z1)
    holes = [V7._circ(h, z, M5_HOLE / 2) for h in (-PANEL_SCREW_H, PANEL_SCREW_H) for z in PANEL_SCREW_Z]
    rep = dict(side=side, screws=4, mount="face-mounted on the rails' outer faces, 4 x M5 x 8 button head + T-nut")
    if side == "front":
        h, z = _hv(side, (ESTOP_X, 0, ESTOP_Z))
        key = V7._rrect(h - ESTOP_KEY_W / 2, h + ESTOP_KEY_W / 2, z, z + ESTOP_KEY_R, 0) + \
            V7._circ(h, z + ESTOP_KEY_R - ESTOP_KEY_W / 2, ESTOP_KEY_W / 2)
        holes.append(V7._circ(h, z, ESTOP_HOLE_D / 2) + key)
        rep["estop_hole"] = dict(d=ESTOP_HOLE_D, key_w=ESTOP_KEY_W, centre_hz=[h, z])
    if side == "back":
        for nm, (x, w, hh) in (("iec", IEC[:3]), ("usb", USB[:3])):
            h, _ = _hv(side, (x, 0, 0))
            cw, ch = w + 2 * DEV_CLEAR, hh + 2 * DEV_CLEAR
            holes.append(V7._rrect(h - cw / 2, h + cw / 2, PANEL_DEV_Z - ch / 2, PANEL_DEV_Z + ch / 2, DEV_R))
            rep[f"{nm}_cutout"] = dict(w=round(cw, 2), h=round(ch, 2), r=DEV_R, centre_hz=[h, PANEL_DEV_Z])
        h, _ = _hv(side, (IEC[0], 0, 0))
        for s in (-1, 1):
            holes.append(V7._circ(h + s * IEC_PITCH / 2, PANEL_DEV_Z, 1.7))
    if side in VENT_H:
        hc = VENT_H[side]
        for i in range(VENT_N):
            h = hc + (i - (VENT_N - 1) / 2) * VENT_PITCH
            holes.append(V7._rrect(h - VENT_W / 2, h + VENT_W / 2, VENT_Z[0], VENT_Z[1], VENT_W / 2 - 1e-3))
        a1 = VENT_W * (VENT_L - VENT_W) + math.pi * (VENT_W / 2) ** 2
        rep["vents"] = dict(n=VENT_N, w=VENT_W, l=VENT_L, pitch=VENT_PITCH, open_mm2=round(VENT_N * a1, 1),
                            z=list(VENT_Z))
    for f in holes:
        face = face - f
    face = max(face.faces(), key=lambda q: q.area)
    return V7._clean_face(face), rep


def _vent_clearance(A):
    out = {}
    for side, hc in VENT_H.items():
        n, hv = np.array(SIDES[side][0], float), np.array(SIDES[side][1], float)
        k, kh = int(np.argmax(np.abs(n))), int(np.argmax(np.abs(hv)))
        h0 = hc - (VENT_N - 1) / 2 * VENT_PITCH - VENT_W / 2
        h1 = hc + (VENT_N - 1) / 2 * VENT_PITCH + VENT_W / 2
        lo, hi = np.zeros(3), np.zeros(3)
        lo[kh], hi[kh] = sorted((hv[kh] * h0, hv[kh] * h1))
        lo[2], hi[2] = VENT_Z
        lo[k], hi[k] = sorted((n[k] * (PANEL_IN - VENT_GAP), n[k] * (PANEL_IN + PANEL_T)))
        best, who = 1e9, None
        for nm in A.parts:
            if nm == f"s_panel_{side}":
                continue
            c0, c1 = map(np.array, A.bbox(nm))
            gap = max(max(lo[i] - c1[i], c0[i] - hi[i]) for i in range(3))
            if gap < best:
                best, who = gap, nm
        out[side] = (round(float(best), 2), who)
        assert best >= VENT_GAP - 1e-6, f"{side} vents {best:.2f} mm from {who}"
    return out


def _sheet_notches():
    """Deck sheet notches (x0, x1, y0, y1) round each cage post and its bracket, 0.5 clear, open to the edge."""
    out = []
    c = 0.5
    for key, (x, y) in POSTS.items():
        if key == "back":
            out.append((x - W / 2 - c, x + W / 2 + BR_L + c, y - W / 2 - c, HALF + 1))
        else:
            x0, x1 = sorted((x - np.sign(x) * (W / 2 + c), np.sign(x) * (HALF + 1)))
            out.append((x0, x1, y - W / 2 - BR_L - c, y + W / 2 + c))
    return out


def _deck_sheet():
    p = _box(-HALF, HALF, -HALF, HALF, SHEET_Z0, SHEET_Z1)
    for x0, x1, y0, y1 in _sheet_notches():
        p = p - _box(x0, x1, y0, y1, SHEET_Z0 - 1, SHEET_Z1 + 1)
    p = p - Cylinder(SHEET_KFL_HOLE_D / 2, 40)
    p = p - Cylinder(G_D / 2, 40).moved(Location((G2[0], G2[1], 0)))
    for x, y in PA_RAIL_HOLES + SHEET_SCREWS:
        p = p - Cylinder(M5_HOLE / 2, 40).moved(Location((x, y, 0)))
    for x, y in STANDOFF_XY:
        p = p - Cylinder(SHEET_HEAD_HOLE_D / 2, 40).moved(Location((x, y, 0)))
    return _solid(p)


def _cl42t_holes():
    return V7._cl42t_holes()


def _cc_world(x, y, dz=0.0):
    wx, wy, wz = fp7_controller_carrier.world(x, y, dz)
    return (wx + CC_DX, wy, wz + CC_DZ)


def _bottom_panel():
    p = _box(-HALF, HALF, -HALF, HALF, BOTP_Z0, BOTP_Z1)
    n = RIN - 1.5                       # corner notches round the feet (r 10 at 140)
    for sx in (1, -1):
        for sy in (1, -1):
            p = p - _box(*sorted((sx * n, sx * 200)), *sorted((sy * n, sy * 200)), BOTP_Z0 - 1, BOTP_Z1 + 1)
    p = p - Cylinder(23.0, 20).moved(Location((0, 0, BOTP_Z0)))
    x0, x1, y0, y1, _ = PSU_BOX
    for x, y in [(x, y) for x in (x0 + 8, x1 - 8) for y in (y0 + 8, y1 - 8)]:
        p = p - Cylinder(1.7, 20).moved(Location((x, y, BOTP_Z0)))
    for x, y in fp7_controller_carrier.INSERTS:
        wx, wy, _ = _cc_world(x, y)
        p = p - Cylinder(1.7, 20).moved(Location((wx, wy, BOTP_Z0)))
    for x, y in _cl42t_holes():
        p = p - Cylinder(2.25, 20).moved(Location((x, y, BOTP_Z0)))
    for x, y in BOTP_SCREWS:
        p = p - Cylinder(M5_HOLE / 2, 20).moved(Location((x, y, BOTP_Z0)))
    return _solid(p)


def _panels(A, R_):
    A.meta_panels = {}
    for side in SIDES:
        face, rep = _side_profile(side)
        A.add(f"s_panel_{side}", _solid(extrude(_plane(side) * face, amount=PANEL_T)), "panel", "plinth", step=7,
              note="3 mm black ABS sheet, laser cut, face-mounted on the rails' outer faces")
        A.meta_panels[side] = rep
        n = np.array(SIDES[side][0], float)
        hv = np.array(SIDES[side][1], float)
        for z, lvl in zip(PANEL_SCREW_Z, ("deck", "bot")):
            for h in (-PANEL_SCREW_H, PANEL_SCREW_H):
                at = n * PANEL_IN + hv * h + _vec(0, 0, z)
                role = f"{lvl}_{'x' if abs(n[1]) > 0.5 else 'y'}_{side}"
                bolt_joint(A, [f"s_panel_{side}", R_[role]], at, -n, screw="M5x8",
                           name=f"j_panel_{side}_{lvl}_{'p' if h > 0 else 'n'}", module="plinth", contact="faces",
                           step=7, **M5)
    A.add("s_panel_deck", _deck_sheet(), "panel", "plinth", step=2,
          note="3 mm black ABS deck sheet, laser cut, on the deck ring top (Z 0..3); notched round the cage posts")
    for k, (x, y) in enumerate(SHEET_SCREWS, 1):
        role = "deck_x_back" if y > 0 else "deck_x_front"
        bolt_joint(A, ["s_panel_deck", R_[role]], (x, y, 0.0), (0, 0, -1), screw="M5x8", name=f"j_deck_sheet_{k}",
                   module="plinth", contact="faces", step=2, **M5)
    A.add("s_panel_bottom", _bottom_panel(), "panel", "plinth", step=7,
          note="3 mm black ABS bottom panel, laser cut, face-mounted under the bottom ring (Z -113..-110), "
               "notched round the feet")
    for k, (x, y) in enumerate(BOTP_SCREWS, 1):
        role = ("bot_x_back" if y > 0 else "bot_x_front") if abs(y) > 100 else ("bot_y_right" if x > 0 else "bot_y_left")
        bolt_joint(A, ["s_panel_bottom", R_[role]], (x, y, BOT_Z0), (0, 0, 1), screw="M5x8", name=f"j_bottom_panel_{k}",
                   module="plinth", contact="faces", step=7, **M5)



# ------------------------------------------------------------------ drive and rotor cartridge (stage B)

def _holes(s, pts, d, z0=-200.0, h=600.0):
    for x, y in pts:
        s = s - Cylinder(d / 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y, z0)))
    return s


def _plate(x, y, z0, label):
    return _box(-x / 2, x / 2, -y / 2, y / 2, z0, z0 + PLATE_T)


def _motor_plate():
    p = _plate(MPLATE_X, MPLATE_Y, MPLATE_Z0, "lp_motor_plate")
    p = _holes(p, [(0, 0)], COUPLING_BORE_D)
    p = _holes(p, MPLATE_RAIL_HOLES, M5_HOLE)
    p = _holes(p, MOTOR_HOLES, 3.4)
    return _solid(p)


def _plate_a():
    p = _plate(120.0, 120.0, PA_Z0, "lp_plate_a")
    p = _holes(p, [(0, 0)], CENTRE_HOLE_D)
    p = _holes(p, PA_RAIL_HOLES + STANDOFF_XY, M5_HOLE)
    p = _holes(p, KFL_HOLES, 4.5)
    return _solid(p)


def _plate_b():
    p = _plate(120.0, 120.0, PB_Z0, "lp_plate_b")
    p = _holes(p, [(0, 0)], CENTRE_HOLE_D)
    p = _holes(p, STANDOFF_XY, M5_HOLE)
    p = _holes(p, KFL_HOLES, 4.5)
    p = _holes(p, PA_RAIL_HOLES, DRIVER_HOLE_D)
    return _solid(p)


def _carrier_disc():
    from build123d import Cone
    t, z0 = PLATE_T, DISC_Z0
    p = Cylinder(DISC_D / 2, t, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, z0)))
    p = _holes(p, [(0, 0)], 2 * D6.HUB_PILOT_R + 0.6)
    k, r_edge, r_hole = fp6_drum_carrier.CSK_K, fp6_drum_carrier.CSK_R_AT_HEAD_EDGE, fp6_drum_carrier.M4_HOLE_R
    zc = DISC_Z1 - 0.29 - (r_edge - r_hole) / k
    r_top = r_edge + k * (0.29 + 1.0)
    for x, y in HUB_HOLES:
        p = p - Cylinder(r_hole, 40).moved(Location((x, y, z0)))
        p = p - Cone(r_hole, r_top, DISC_Z1 + 1.0 - zc, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((x, y, zc)))
    p = _holes(p, _polar(LED_R, LED_ANGLES), LED_HOLE_D)
    p = _holes(p, CR_SCREWS, 3.4)
    p = _holes(p, BAR_SCREWS, 3.4)
    p = _holes(p, [_pol(PWR_R, PORT_ANG)], PORT_HOLE_D)
    p = _holes(p, [_pol(r, a) for r, a in WIRE_HOLES], WIRE_HOLE_D)

    p = p - _dev_rect(SWITCH_ANG, *SW_CUT, DISC_Z0 - 1, DISC_Z1 + 1)
    p = p - _dev_rect(SWITCH_ANG, SW_SPOT[0], SW_SPOT[1], DISC_Z1 - SW_SPOT[2], DISC_Z1 + 1)
    p = _holes(p, _polar(RING_B_R, RING_SCREW_ANG), 3.4)
    p = _holes(p, WEDGE_SCREWS, 2.5)                         # M3 tapped
    p = _holes(p, PI_HOLES, 2.7)
    return _solid(p)


def _polar(r, angles):
    return [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in angles]


def _ring(id_, od, z0, t, label, holes=(), slots=False):
    s = HW.ring(od, id_, t, label).moved(Location((0, 0, z0)))
    s = _holes(s, holes, 3.4)
    if slots:
        r0, r1, w = LED2_SLOT
        for a in LED_ANGLES:
            R = _R([(math.cos(math.radians(a)), math.sin(math.radians(a)), 0),
                    (-math.sin(math.radians(a)), math.cos(math.radians(a)), 0), (0, 0, 1)])
            s = s - copy.deepcopy(_box(r0, r1, -w / 2, w / 2, z0 - 1, z0 + t + 1)).moved(_location(R, (0, 0, 0)))
    s = _solid(s)
    s.label = label
    return s


def _pol(r, ang):
    return (r * math.cos(math.radians(ang)), r * math.sin(math.radians(ang)))


def _rotz(ang):
    c, s_ = math.cos(math.radians(ang)), math.sin(math.radians(ang))
    return _R([(c, s_, 0), (-s_, c, 0), (0, 0, 1)])


def _dev_rect(ang, radial, tangential, z0, z1, r=PWR_R):
    """Box radial x tangential centred at radius r on the ray at `ang`, Z z0..z1."""
    b = _box(r - radial / 2, r + radial / 2, -tangential / 2, tangential / 2, z0, z1)
    return b.moved(_location(_rotz(ang), (0, 0, 0)))


def _ring_b():
    s = HW.ring(RING_B[1], RING_B[0], RING_T, "lp_ring_b").moved(Location((0, 0, DRUM_FLOOR)))
    s = _holes(s, _polar(RING_B_R, RING_SCREW_ANG), 3.4)
    # notches open to the outer edge; a 1.5 to 2 mm band stays on the inner edge round the tube foot
    s = s - _dev_rect(PORT_ANG, 16.5, PORT_NUT_D + 1.5, DRUM_FLOOR - 1, DRUM_FLOOR + RING_T + 1, r=78.5 + 8.25)
    s = s - _dev_rect(SWITCH_ANG, 15.5, SW_SPOT[1], DRUM_FLOOR - 1, DRUM_FLOOR + RING_T + 1, r=78.0 + 7.75)
    for r, a in WIRE_HOLES:
        if r > 80:
            s = s - Cylinder(WIRE_HOLE_D / 2 + 0.5, 20).moved(Location((*_pol(r, a), DRUM_FLOOR)))
    s = _solid(s)
    s.label = "lp_ring_b"
    return s


def _bank_solid():
    """Anker 321 Power Bank (PowerCore 5K, A1112): 96 x 45.2 x 23.1 bar with R5 long edges and R1.5
    end edges; port end (-X): USB-C (in/out) and USB-A (out) receptacles, the status LED window and the
    power button. Local frame: lying on its long side, L on X, 23.1 thickness on Y, 45.2 on Z, centred."""
    from build123d import Axis, RectangleRounded, fillet
    b = Box(BANK_L, BANK_W, BANK_H)
    b = fillet(b.edges().filter_by(Axis.X), BANK_EDGE_R)
    b = fillet(b.edges().filter_by_position(Axis.X, -BANK_L / 2 - 1e-3, -BANK_L / 2 + 1e-3)
               + b.edges().filter_by_position(Axis.X, BANK_L / 2 - 1e-3, BANK_L / 2 + 1e-3), 1.5)
    face = Plane(origin=(-BANK_L / 2 - 0.1, 0, 0), x_dir=(0, 0, 1), z_dir=(1, 0, 0))
    for zc, w, h, depth, r in ((11.0, 8.94, 3.26, 7.0, 1.6), (-6.0, 12.5, 5.12, 10.0, 0.4)):
        pl = Plane(origin=(-BANK_L / 2 - 0.1, 0, zc), x_dir=(0, 0, 1), z_dir=(1, 0, 0))
        b = b - extrude(pl * RectangleRounded(w, h, r), amount=depth + 0.1)
    b = b - extrude(Plane(origin=(-BANK_L / 2 - 0.1, 0, -17.0), x_dir=(0, 0, 1), z_dir=(1, 0, 0))
                    * RectangleRounded(6.0, 1.6, 0.7), amount=0.8)          # LED window
    _ = face
    btn = Plane(origin=(-BANK_L / 2 + 12.0, 0, BANK_H / 2 + 0.1), z_dir=(0, 0, -1)) * Cylinder(
        2.5, 0.6, align=(Align.CENTER, Align.CENTER, Align.MIN))
    b = b - btn                                                             # power button recess
    return _solid(b)


def _cradle():
    """fp8_bank_cradle: PETG tray on the disc holding the bank on its long side: 3 floor, 2.5 walls, a
    1.5 snap lip along both long walls over the bank's top edge, the -X end open for the USB-A / USB-C
    leads, a thumb window in the +X end, two ears per end with 2 x M3 x 10 + nut through the disc."""
    from build123d import Axis, fillet
    x0, x1 = BANK_X - CR_X, BANK_X + CR_X
    y0, y1 = CR_Y0, CR_Y1
    z0 = DRUM_FLOOR
    b = Box(x1 - x0, y1 - y0, CR_Z1 - z0, align=(Align.MIN, Align.MIN, Align.MIN)).moved(Location((x0, y0, z0)))
    b = fillet(b.edges().filter_by(Axis.Z), 3.0)
    ix, iy = BANK_L / 2 + CR_GAP, BANK_W / 2 + CR_GAP
    lip_z = BANK_Z0 + BANK_H + CR_GAP
    b = b - _box(BANK_X - ix, BANK_X + ix, BANK_Y - iy, BANK_Y + iy, z0 + CR_FLOOR, lip_z)
    b = b - _box(BANK_X - ix, BANK_X + ix, BANK_Y - iy + CR_LIP, BANK_Y + iy - CR_LIP, lip_z - 1, CR_Z1 + 1)
    b = b - _box(x0 - 1, x0 + CR_T + 1, BANK_Y - 8.0, BANK_Y + 8.0, z0 + CR_FLOOR + 3.0, CR_Z1 + 1)   # port end open
    b = b - _box(x1 - CR_T - 1, x1 + 1, BANK_Y - 6.0, BANK_Y + 6.0, z0 + 20.0, CR_Z1 + 1)            # thumb window
    for sx in (1, -1):
        xa, xb = sorted((BANK_X + sx * (CR_X - 1.0), BANK_X + sx * CR_EAR_X1))
        ear = Box(xb - xa, 2 * CR_EAR_HY, CR_FLOOR, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
            Location((xa, BANK_Y, z0)))
        b = b + ear
    b = _holes(b, CR_SCREWS, 3.4)
    return _solid(b)


def _charge_port():
    """Adafruit 6069 class round USB-C panel-mount extension: 20 flange under the disc (USB-C receptacle in
    its face, facing down), 12 threaded body through the disc, 14 plastic nut on the disc top, cable boot.
    Local Z up, flange face at z = -PORT_FLANGE_T, panel underside at z = 0."""
    from build123d import RectangleRounded
    fl = Cylinder(PORT_FLANGE_D / 2, PORT_FLANGE_T, align=(Align.CENTER, Align.CENTER, Align.MAX))
    body = Cylinder(PORT_BODY_D / 2, PLATE_T + 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    nut = Cylinder(PORT_NUT_D / 2, PORT_NUT_T, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, PLATE_T)))
    boot = Cylinder(3.5, 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, PLATE_T + 10.0)))
    s = fl + body + nut + boot
    s = s - extrude(Plane(origin=(0, 0, -PORT_FLANGE_T - 0.1)) * RectangleRounded(8.94, 3.26, 1.6), amount=6.5)
    return _solid(s)


def _power_switch():
    """KCD1-101 SPST mini rocker: 21 x 15 bezel (under the disc, rocker facing down), 18.9 x 12.8 body
    through the disc cutout up to 11.5 behind the panel face, two 4.8 quick-connect tabs. Local X radial,
    Y tangential, panel underside at z = 0."""
    from build123d import Axis, fillet
    bz = Box(*SW_BEZEL, SW_BEZEL_T, align=(Align.CENTER, Align.CENTER, Align.MAX))
    bz = fillet(bz.edges().filter_by(Axis.Z), 1.5)
    rw, rt = SW_BEZEL[0] - 4.0, (SW_BEZEL[1] - 4.0) / 2
    rock = Box(rw, rt, 2.5, align=(Align.CENTER, Align.MIN, Align.MAX)).moved(Location((0, 0, -SW_BEZEL_T + 0.01))) + \
        Box(rw, rt, 1.2, align=(Align.CENTER, Align.MAX, Align.MAX)).moved(Location((0, 0, -SW_BEZEL_T + 0.01)))
    rock = fillet(rock.edges().filter_by(Axis.Z), 1.0)                       # rocker paddle, ON side proud
    body = Box(*SW_BODY, SW_BODY_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
    s = bz + rock + body
    for sy in (-1, 1):
        s = s + Box(0.8, 4.8, 7.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((0, sy * 4.5, SW_BODY_H)))
    return _solid(s)


def _balance_bar():
    b = _box(BAR_X - BAR_L / 2, BAR_X + BAR_L / 2, BAR_Y - BAR_W / 2, BAR_Y + BAR_W / 2, DISC_Z0 - BAR_H, DISC_Z0)
    for x, y in BAR_SCREWS:
        b = b - Cylinder(1.25, BAR_TAP_DEPTH, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(
            Location((x, y, DISC_Z0)))
    return _solid(b)


def _dev_loc(ang, z):
    return _location(_rotz(ang), np.array([*_pol(PWR_R, ang), z]))


def _power(A):
    A.add("fp8_bank_cradle", _cradle(), "printed", "rotor", step=4,
          note="PETG cradle for the power bank on its long side, snap lip, 4 x M3 x 10 + nut through the disc")
    A.add("v_power_bank", _bank_solid().moved(Location((BANK_X, BANK_Y, BANK_Z0 + BANK_H / 2))), "power_bank",
          "electrical", step=5, spins=True,
          includes="Anker 321 Power Bank (PowerCore 5K, A1112) 96 x 45.2 x 23.1, 118 g, 5200 mAh, USB-C in/out + "
                   "USB-A out, 12 W; port end -X")
    A.vendor.append(V.vendor_entry("v_power_bank", "electronics/anker-321-power-bank-a1112"))
    A.add("v_balance_bar", _balance_bar(), "weight", "rotor", step=4,
          note=f"bright steel flat {BAR_H:g} x {BAR_W:g} x {BAR_L:g} under the disc at +Y, 2 x M3 tapped 8 deep: "
               "balances the bank's moment")
    A.vendor.append(dict(component="v_balance_bar", source_url=None, author="catalogue (bright mild steel flat bar "
                         "16 x 12, cut and tapped)", licence=None, file=None, price_usd=2.0))
    A.add("v_charge_port", _charge_port().moved(_dev_loc(PORT_ANG, DISC_Z0)), "switch", "electrical", step=4,
          spins=True, note=f"round USB-C panel-mount extension through the disc at r {PWR_R:g}, {PORT_ANG:g} deg, "
                           "receptacle facing down; its cable goes down the outer wire hole, under the disc and up "
                           "inside the tube to the bank's USB-C input")
    A.vendor.append(V.vendor_entry("v_charge_port", "electronics/adafruit-6069-usb-c-round-panel-mount"))
    A.add("v_power_switch", _power_switch().moved(_dev_loc(SWITCH_ANG, DISC_Z0)), "switch", "electrical", step=4,
          spins=True, note=f"KCD1-101 rocker in the 5 V lead (bank USB-A -> switch -> Pi micro-USB) at r {PWR_R:g}, "
                           f"{SWITCH_ANG:g} deg, rocker facing down; disc spot-faced to 2.5 for the snap clips")
    A.vendor.append(V.vendor_entry("v_power_switch", "electronics/kcd1-101-rocker"))


def _cam_frame():
    a = np.array(OPTICAL_AXIS, float)
    a /= np.linalg.norm(a)
    t = np.array([a[2], 0.0, -a[0]])
    p0 = np.array(LENS) - a * CAM_STACK          # PCB underside centre
    return a, t, p0


def _camera_wedge():
    """fp8_camera_wedge: one PETG block on the disc centre, its top face square to OPTICAL_AXIS
    (30 deg toward +X) under the UC-261 board, 4 x M2 holes at CAM_HOLE_PITCH, 4 counterbored
    M3 x 6 into the disc's tapped holes."""
    a, t, p0 = _cam_frame()
    x0, x1, y0, y1 = WEDGE_XY
    s = _box(x0, x1, y0, y1, DRUM_FLOOR, DRUM_FLOOR + 60.0)
    cut = Plane(origin=tuple(p0), x_dir=tuple(t), z_dir=tuple(a)) * Box(200, 200, 100,
                                                                          align=(Align.CENTER, Align.CENTER, Align.MIN))
    s = s - cut
    s = _holes(s, WEDGE_SCREWS, 3.4)
    for x, y in WEDGE_SCREWS:
        s = s - Cylinder(WEDGE_CB_D / 2, 80, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((x, y, DRUM_FLOOR + WEDGE_CB_FLOOR)))
    v = np.array([0.0, 1.0, 0.0])
    h = CAM_HOLE_PITCH / 2
    for su in (-1, 1):
        for sv in (-1, 1):
            c = p0 + t * su * h + v * sv * h
            s = s - Plane(origin=tuple(c + a * 0.5), x_dir=tuple(t), z_dir=tuple(-a)) * Cylinder(
                0.9, 6.5, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return _solid(s)


def _drive(A):
    body, shaft = _motor_solids()
    loc = _location(R_MOTOR, (0, 0, MOTOR_FACE_Z))
    A.add(V7.MOTOR, Compound(children=[copy.deepcopy(s).moved(loc) for s in body]), "motor", "drive_base", step=1,
          note="v_nema17 vendor STEP (17HS19 closed loop) stands in for the 17HS08 + encoder; face Z "
               f"{MOTOR_FACE_Z:g} on {MOTOR_SPACER:g} spacers")
    A.add("v_nema17_shaft", copy.deepcopy(shaft).moved(loc), "shaft", "drive_base", step=1, spins=True,
          note="motor shaft (vendor STEP solid), turns with the coupling")
    A.vendor.append(V.manifest_entry(V7.MOTOR, "v_nema17"))
    A.add("lp_motor_plate", _motor_plate(), "plate", "drive_base", step=1,
          note=f"laser / waterjet cut aluminium {PLATE_T} plate {MPLATE_X:g} x {MPLATE_Y:g} under the cross rails: "
               f"4 x M5 + T-nut, NEMA 17 31 mm M3 pattern, {COUPLING_BORE_D:g} coupling bore")
    for k, (x, y) in enumerate(MOTOR_HOLES, 1):
        nm = f"v_motor_spacer_{k}"
        A.add(nm, HW.ring(6.0, 3.4, MOTOR_SPACER, nm).moved(Location((x, y, MOTOR_FACE_Z))), "standoff",
              "drive_base", step=1, note=f"aluminium round spacer 6 OD x 3.4 ID x {MOTOR_SPACER:g}")
        A.vendor.append(dict(component=nm, source_url=None, author="catalogue (M3 round spacer)", licence=None,
                             file=None, price_usd=0.2))
    s = V7._coupling_src()
    bb = s.bounding_box()
    R = _R([(0, 0, 1), (0, 1, 0), (-1, 0, 0)])
    t = np.array([0, 0, COUPLING_Z0]) - R @ np.array([bb.min.X, (bb.min.Y + bb.max.Y) / 2, (bb.min.Z + bb.max.Z) / 2])
    A.add("v_coupling_jaw_5_8mm", copy.deepcopy(s).moved(_location(R, t)), "coupling", "drive_base", step=1,
          spins=True, includes="MECCANIXITY 5 -> 8 mm jaw coupling, D25 L30 (24 x 32 envelope model)")
    A.vendor.append(V.manifest_entry("v_coupling_jaw_5_8mm", "v_coupling_jaw_5_8"))

    # cartridge
    A.add("lp_plate_a", _plate_a(), "plate", "drive_base", step=2,
          note=f"aluminium {PLATE_T} plate 120 x 120 on the deck sheet: 4 x M5 into the cross-rail T-nuts, "
               "4 standoffs, lower KFL08, 30 centre hole")
    A.add("lp_plate_b", _plate_b(), "plate", "drive_base", step=2,
          note=f"aluminium {PLATE_T} plate 120 x 120 on the standoffs: upper KFL08, "
               "8 mm driver holes over the plate A rail bolts")
    kfl = HW.kfl08()
    for nm, z in (("v_kfl08_lower", PA_Z1), ("v_kfl08_upper", PB_Z1)):
        A.add(nm, copy.deepcopy(kfl).moved(Location((0, 0, z))), "bearing", "drive_base", step=2,
              includes="KFL08 2-bolt flange unit, 8 mm KP08 insert, zinc die-cast housing (catalogue drawing: oval "
                       "flange, housing boss, spherical insert band, inner-ring collar with 2 set screws)")
        A.vendor.append(dict(V.vendor_entry(nm, "motion/kfl08"), file="lib/hardware.py:kfl08"))
    for k, (x, y) in enumerate(STANDOFF_XY, 1):
        nm = f"v_standoff_m5_{k}"
        A.add(nm, HW.hex_standoff(STANDOFF_AF, STANDOFF_L, 4.2, nm, chamfer_mm=0.5).moved(Location((x, y, PA_Z1))), "standoff",
              "drive_base", step=2, note="M5 female-female hex standoff 8 AF x 40 (hexagonal prism, 0.5 end chamfers), tapped through")
        A.vendor.append(dict(component=nm, source_url=None, author="catalogue (M5 x 40 F-F hex standoff)",
                             licence=None, file=None, price_usd=0.5))
    A.add("v_shaft_collar_8mm", HW.shaft_collar_8mm().moved(Location((0, 0, COLLAR_Z0))), "collar", "rotor", step=2,
          note="on the upper KFL08 inner ring, under the hub")
    A.vendor.append(V.manifest_entry("v_shaft_collar_8mm", "v_shaft_collar_8mm"))
    shaft = Cylinder(D6.SHAFT_D / 2, SHAFT_Z1 - SHAFT_Z0).moved(Location((0, 0, (SHAFT_Z0 + SHAFT_Z1) / 2)))
    A.add("s_shaft_8mm", shaft.solids()[0], "shaft", "rotor", step=2,
          note=f"8 mm shaft Z {SHAFT_Z0:.2f}..{SHAFT_Z1:.2f} ({SHAFT_Z1 - SHAFT_Z0:.1f} long): {COUPLING_IN} in the "
               "coupling, through both KFL08 units and the collar into the hub")
    A.vendor.append(V.manifest_entry("s_shaft_8mm", "s_shaft_8mm"))

def _rotor(A):
    hub, screws, (cx, cy, z_face) = V7._hub_parts()
    R = _R([(1, 0, 0), (0, -1, 0), (0, 0, -1)])
    t = np.array([0, 0, HUB_TOP_Z]) - R @ np.array([cx, cy, z_face])
    loc = _location(R, t)
    A.add(V7.HUB, Compound(children=[copy.deepcopy(hub).moved(loc)] + [copy.deepcopy(s).moved(loc) for s in screws]),
          "hub", "rotor", step=3, includes="1310-0016-0008 Hyper Hub + its two M4 clamp screws")
    A.vendor.append(V.manifest_entry(V7.HUB, "v_gobilda_1310_hub_8mm"))
    A.add("lp_carrier_disc", _carrier_disc(), "plate", "rotor", step=4,
          note=f"aluminium {PLATE_T} disc {DISC_D:g}: 4 x M4 csk into the hub, 12 LED holes r {LED_R}, ring B, "
               "camera wedge, bank cradle and balance bar screws, Pi standoff holes, charge port hole, switch cutout "
               "with a 0.7 spot face, 4 wire holes")
    led_z0 = DRUM_FLOOR - D6.LED_LEN
    for i, (x, y) in enumerate(_polar(LED_R, LED_ANGLES), 1):
        nm = f"v_ir_led_{i:02d}"
        A.add(nm, HW.ir_led_5mm(nm).moved(Location((x, y, led_z0))), "led", "electrical", step=4, spins=True,
              note="in a 5.2 disc hole, dome flush with the disc top under the tube end, glued")
        A.vendor.append(V.manifest_entry(nm, "v_ir_led_5mm"))
    A.add("v_led_driver", HW.pcb_box(20, 25, label="v_led_driver").moved(Location((-53.0, 0.0, DRUM_FLOOR))),
          "pcb", "electrical", step=4, spins=True, note="IR LED driver on the disc top, -X")
    A.vendor.append(V.manifest_entry("v_led_driver", "v_led_driver"))
    A.add("lp_ring_b", _ring_b(), "plate", "rotor", step=4,
          note="3 mm clear acrylic ring on the disc top round the tube foot, 3 x M3 + nut; cut out round the "
               "charge port nut, the switch body and their wire holes")
    _power(A)
    A.add("fp8_camera_wedge", _camera_wedge(), "printed", "rotor", step=5,
          note=f"PETG wedge on the disc centre, face 30 deg to +X, UC-261 M2 holes at {CAM_HOLE_PITCH} pitch (assumed)")
    for k, (x, y) in enumerate(PI_HOLES, 1):
        nm = f"v_standoff_m2p5_{k}"
        A.add(nm, HW.hex_standoff(5.0, PI_STANDOFF, 2.05, nm).moved(Location((x, y, DRUM_FLOOR))), "standoff",
              "electrical", step=5, spins=True, note="M2.5 x 6 F-F standoff under the Pi Zero")
        A.vendor.append(dict(component=nm, source_url=None, author="catalogue (M2.5 x 6 hex standoff)", licence=None,
                             file=None, price_usd=0.1))
    s = V7._pi_src()
    R = _R([(1, 0, 0), (0, 0, 1), (0, -1, 0)])
    t = np.array([0.0, 45.0, DRUM_FLOOR + PI_STANDOFF]) - R @ np.array([0.0, -0.8, 0.0])
    A.add("v_rpi_zero_2w", copy.deepcopy(s).moved(_location(R, t)), "pcb", "electrical", step=5, spins=True)
    A.vendor.append(dict(V.manifest_entry("v_rpi_zero_2w", "v_rpi_zero_2w"),
                         note="representative solid derived from the vendor STEP (PCB kept, parts as boxes)"))
    a, tt, p0 = _cam_frame()
    cam = HW.camera_uc261("v_camera_uc261").moved(Location(Plane(origin=tuple(p0), x_dir=tuple(tt), z_dir=tuple(a))))
    A.add("v_camera_uc261", cam, "camera", "rotor", step=6,
          bodies=[("v_camera_uc261_pcb", "pcb_green"), ("v_camera_uc261_lens", "glass_lens")],
          note="Arducam UC-261 class envelope: PCB 36 x 36 x 1.6, lens barrel 14 x 12")
    A.vendor.append(dict(component="v_camera_uc261", source_url=None, author="catalogue (Arducam UC-261 envelope)",
                         licence=None, file="lib/hardware.py:camera_uc261", price_usd=None))
    ddz = DRUM_FLOOR - D0.DRUM_FLOOR_Z
    A.add("s_drum_tube", s_drum_tube.build().moved(Location((0, 0, ddz))), "stock", "rotor", step=6, opacity=0.35,
          rmin=V7.TUBE_OR - D0.DRUM_WALL - 0.01)
    A.add("s_top_disc", s_top_disc.build().moved(Location((0, 0, ddz))), "stock", "rotor", step=6, opacity=0.35)
    clamp = _polar(CLAMP_R, RING_SCREW_ANG)
    A.add("lp_ring_d", _ring(*RING_D, TUBE_TOP - RING_T, RING_T, "lp_ring_d", clamp, slots=True), "plate", "rotor",
          step=6, rmin=RING_D[0] / 2 - 0.01, note="3 mm clear acrylic ring round the tube top, under the disc overhang")
    A.add("lp_ring_e", _ring(*RING_E, TUBE_TOP, TOP_DISC_T, "lp_ring_e", clamp, slots=True), "plate", "rotor",
          step=6, rmin=RING_E[0] / 2 - 0.01,
          note=f"{TOP_DISC_T} clear acrylic bezel round the top disc: spacer between C and D, LED slots")
    A.add("lp_ring_c", _ring(*RING_C, TUBE_TOP + TOP_DISC_T, RING_T, "lp_ring_c", clamp, slots=True), "plate",
          "rotor", step=6, rmin=RING_C[0] / 2 - 0.01, note="3 mm clear acrylic ring on the top disc")
    zm = TUBE_TOP + TOP_DISC_T / 2
    for i, ang in enumerate(LED_ANGLES, 13):
        nm = f"v_ir_led_{i:02d}"
        c, s_ = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        org = (c * (LED2_TIP_R + D6.LED_LEN), s_ * (LED2_TIP_R + D6.LED_LEN), zm)
        loc = Location(Plane(origin=org, x_dir=(0, 0, 1), z_dir=(-c, -s_, 0)))
        A.add(nm, HW.ir_led_5mm(nm).moved(loc), "led", "electrical", step=6, spins=True,
              note="lying radially in the C / E / D slots, edge-lights the top disc")
        A.vendor.append(V.manifest_entry(nm, "v_ir_led_5mm"))
    for nm, (R, t) in (("v_frog_wall", _frog_wall_pose()), ("v_frog_disc", _frog_disc_pose())):
        for part in ("body", "eyes"):
            src = V._mesh_solid(V7.FROG_BODY_STL) if part == "body" else V7._frog_eyes_local()
            A.add(f"{nm}_{part}", copy.deepcopy(src).moved(_location(R, t)), "specimen",
                  "rotor", step=6, includes="Agalychnis callidryas, 45 mm snout-vent (mesh x 19.5, decimated)")
            A.vendor.append(V.vendor_entry(f"{nm}_{part}", V7.FROG_DIR))


def _frog_wall_pose():
    R, t = V7._frog_wall_pose()
    return R, np.asarray(t, float) + _vec(0, 0, DZF)


def _frog_disc_pose():
    """v7 disc pose rule (tangential, belly down, slid out along +X) against ring C's inner edge,
    keeping v7's 3 mm margin: limit RING_C ID / 2 - 3."""
    R = _R([(-1, 0, 0), (0, -1, 0), (0, 0, 1)])
    v = V7._frog_verts() @ R.T
    c = (v.min(0) + v.max(0)) / 2
    lim = RING_C[0] / 2 - 3.0
    z = TUBE_TOP + TOP_DISC_T + V7.FROG_CLEAR - v[:, 2].min()
    lo, hi = 0.0, lim
    for _ in range(50):
        mid = (lo + hi) / 2
        if np.hypot(v[:, 0] - c[0] + mid, v[:, 1] - c[1]).max() <= lim - V7.FROG_DISC_SPARE:
            lo = mid
        else:
            hi = mid
    t = np.array([math.floor(lo * 100) / 100 - c[0], -c[1], z])
    return R, t



# ------------------------------------------------------------------ cage

def _hoop(A, which, ring, path):
    a = math.radians(V7.HOOP_LUG_DEG - 90)
    R = _R([(math.cos(a), math.sin(a), 0), (-math.sin(a), math.cos(a), 0), (0, 0, 1)])
    k = V7.HOOP_INNER_K if ring == "inner" else 1.0
    t = np.array([0, 0, HOOP_Z[which] + (0.4 * (1 - k) if ring == "inner" else 0.0)]) - R @ np.array([110.0 * k,
                                                                                                    110.0 * k, 0.0])
    nm = f"v_keiran_hoop_{which}_{ring}"
    A.add(nm, copy.deepcopy(V7._hoop_src(path, k)).moved(_location(R, t)), "printed", "cage", step=9,
          rmin=100.0 * k - 0.1 if ring == "inner" else 105.9,
          includes="Keiran 220 mm embroidery hoop ring (Printables 254483), clamps the mesh")
    A.vendor.append(V7._rep_entry(nm, V7.HOOP_DIR, "Keiran STL"))


def _post_face(key):
    """(face centre XY on the post's inner face, inward normal, width direction) for a cage post."""
    x, y = POSTS[key]
    if key == "back":
        return _vec(x, y - W / 2, 0), _vec(0, -1, 0), _vec(1, 0, 0)
    s = np.sign(x)
    return _vec(x - s * W / 2, y, 0), _vec(-s, 0, 0), _vec(0, 1, 0)


def _clip(key, which):
    """fp8_cage_clip: PETG block on the post's inner face, its front cut concave to the hoop's outer
    radius (+0.1) over the hoop pair, a 3 mm ledge under the pair, one M5 x 10 counterbored through it."""
    F, n, w = _post_face(key)
    z0 = HOOP_Z[which] - CLIP_LEDGE
    z1 = HOOP_Z[which] + HOOP_H + CLIP_TOP
    depth = 70.0
    w = np.cross(_vec(0, 0, 1), n)
    R = _R([n, w, _vec(0, 0, 1)])
    body = Box(depth, P.W, z1 - z0, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
        _location(R, F + _vec(0, 0, z0)))
    body = body - Cylinder(HOOP_OUT_R + CLIP_GAP, z1 - HOOP_Z[which] + 1, align=(Align.CENTER, Align.CENTER, Align.MIN)
                           ).moved(Location((0, 0, HOOP_Z[which])))
    body = body - Cylinder(LEDGE_R[which], z1 - z0 + 2, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, z0 - 1)))
    zm = HOOP_Z[which] + HOOP_H / 2
    at = F + _vec(0, 0, zm)
    pl = Plane(origin=tuple(at - n * 1.0), z_dir=tuple(n))
    body = body - pl * Cylinder(M5_HOLE / 2, depth + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    pl = Plane(origin=tuple(at + n * CLIP_T_BACK), z_dir=tuple(n))
    body = body - pl * Cylinder(CLIP_CB_D / 2, depth, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return _solid(body), at, -n


def _cage(A, R_):
    for key in POSTS:
        for which in ("bottom", "top"):
            nm = f"fp8_cage_clip_{key}_{which}"
            s, at, axis = _clip(key, which)
            A.add(nm, s, "clip", "cage", step=8,
                  note="printed PETG clip: hoop pair seat on a cage post, M5 x 10 + T-nut into the post's inner slot")
            bolt_joint(A, [nm, R_[f"post_{key}"]], at, axis, screw="M5x10", name=f"j_clip_{key}_{which}",
                       module="cage", contact="faces", step=8, **M5)
    for which in ("bottom", "top"):
        _hoop(A, which, "inner", V7.HOOP_IN_STL)
        _hoop(A, which, "outer", V7.HOOP_OUT_STL)
    sleeve = HW.ring(2 * MESH_R[1], 2 * MESH_R[0], MESH_Z[1] - MESH_Z[0], "s_mesh_sleeve").moved(
        Location((0, 0, MESH_Z[0])))
    A.add("s_mesh_sleeve", sleeve, "mesh", "cage", step=9, rmin=MESH_R[0] - 0.01,
          note="insect net sleeve (modelled as a 1.0 shell), clamped in both hoop pairs")
    top = Cylinder(MESH_TOP_R, MESH_TOP_Z[1] - MESH_TOP_Z[0], align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, MESH_TOP_Z[0])))
    A.add("s_mesh_top", _solid(top), "mesh", "cage", step=9, note="insect net top, clamped in the top hoop pair")


# ------------------------------------------------------------------ plinth electrics (v7 on the new floor)

def _estop_frame():
    R = _R([(1, 0, 0), (0, -1, 0), (0, 0, -1)])
    t = np.array([ESTOP_X - V7.ESTOP_AXIS_XZ[0], -PANEL_IN + V7.ESTOP_BOX_FRONT, ESTOP_Z + V7.ESTOP_AXIS_XZ[1]])
    return R, t


def _estop_panel_mount():
    src = V7._estop_src()
    sols = sorted(src.solids(), key=lambda q: q.volume)
    lid, body = sols[0], sols[-1]
    gap = PANEL_T + LEGEND_T
    big = 400.0
    bf = V7.ESTOP_BOX_FRONT
    rear = body & _box(-big, big, -big, bf, -big, big)
    front = (body & _box(-big, big, bf, big, -big, big)).moved(Location((0, gap, 0)))
    ax, az = V7.ESTOP_AXIS_XZ
    from build123d import RegularPolygon
    bush = extrude(Plane(origin=(ax, bf - 0.01, az), x_dir=(1, 0, 0), z_dir=(0, 1, 0))
                   * RegularPolygon(11.0, 16), amount=gap + 0.02)
    s = _solid(rear + bush + front)
    R, t = _estop_frame()
    loc = _location(R, t)
    return Compound(children=[copy.deepcopy(s).moved(loc), copy.deepcopy(lid).moved(loc)])


def _dev_body(x, w, h, y0, y1):
    from build123d import RectangleRounded
    f = RectangleRounded(w, h, DEV_R - DEV_CLEAR)
    return _solid(extrude(Plane(origin=(x, y0, PANEL_DEV_Z), x_dir=(1, 0, 0), z_dir=(0, 1, 0)) * f, amount=y1 - y0))


def _electrics(A):
    R = _R([(0, 1, 0), (0, 0, 1), (1, 0, 0)])
    A.add("v_cl42t_driver", copy.deepcopy(V7._cl42t_src()).moved(_location(R, np.array([CL42T_X0, 0.0, FLOOR_Z]))),
          "driver", "electrical", step=7,
          includes="STEPPERONLINE CL42T-V41 closed-loop driver (representative of the vendor STEP)")
    A.vendor.append(dict(V.manifest_entry("v_cl42t_driver", "v_cl42t"),
                         note="representative solid derived from the vendor STEP (housing kept, terminals as boxes)"))
    x0, x1, y0, y1, h = PSU_BOX
    psu = _box(x0, x1, y0, y1, FLOOR_Z, FLOOR_Z + h)
    A.meta_psu_holes = [(x, y) for x in (x0 + 8, x1 - 8) for y in (y0 + 8, y1 - 8)]
    for x, y in A.meta_psu_holes:
        psu = psu - Cylinder(1.25, 6.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y, FLOOR_Z)))
    A.add("v_psu_lrs_50_24", _solid(psu), "enclosure", "electrical", step=7,
          note="MEAN WELL LRS-50-24 envelope 99 x 82 x 30, 4 x M3 tapped in the base")
    A.vendor.append(V.manifest_entry("v_psu_lrs_50_24", "v_psu_lrs_50_24"))
    cc = fp7_controller_carrier
    A.add("fp7_controller_carrier", cc.build().moved(Location((CC_DX, 0, CC_DZ))), "printed", "electrical", step=7)
    A.add("v_pico", HW.pcb_box(21, 51, label="v_pico").moved(Location(_cc_world(*cc.PICO_C, cc.T + cc.BOSS_H))), "pcb",
          "electrical", step=7)
    A.vendor.append(V.manifest_entry("v_pico", "v_pico"))
    A.add("v_buck_module", _solid(Box(43, 21, 14, align=(Align.CENTER, Align.CENTER, Align.MIN))).moved(
        Location(_cc_world(*cc.BUCK_C, cc.T))), "pcb", "electrical", step=7)
    A.vendor.append(V.manifest_entry("v_buck_module", "v_buck_module"))
    A.add("v_relay_delay", _solid(Box(50, 26, 18, align=(Align.CENTER, Align.CENTER, Align.MIN))).moved(
        Location(_cc_world(*cc.RELAY_C, cc.T))), "pcb", "electrical", step=7)
    A.vendor.append(V.manifest_entry("v_relay_delay", "v_relay_delay"))
    A.add("v_estop_22mm", _estop_panel_mount(), "switch", "electrical", step=7,
          includes="22 mm mushroom e-stop (FreeCAD-library STEP), panel mount: box behind the front panel, "
                   "collar and head proud of it on the legend ring")
    A.vendor.append(V.manifest_entry("v_estop_22mm", "v_estop_22mm"))
    A.add("fp7_estop_legend", fp7_estop_legend.build().moved(
        _location(_R([(1, 0, 0), (0, 0, 1), (0, -1, 0)]), (ESTOP_X, -(PANEL_IN + PANEL_T), ESTOP_Z))),
        "printed", "electrical", step=7,
        note="yellow PETG legend ring behind the e-stop collar, clamped by the e-stop ring nut")
    x, w, hh, ya, yb = IEC
    fw, fh, ft = IEC_FLANGE
    iec = _dev_body(x, w, hh, ya, yb)
    iec = iec + _box(x - fw / 2, x + fw / 2, PANEL_IN - ft, PANEL_IN, PANEL_DEV_Z - fh / 2, PANEL_DEV_Z + fh / 2)
    for sx in (-1, 1):
        iec = iec - Plane(origin=(x + sx * IEC_PITCH / 2, PANEL_IN + 1, PANEL_DEV_Z), z_dir=(0, -1, 0)) * Cylinder(
            1.25, ft + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    A.add("v_iec_inlet", _solid(iec), "switch", "electrical", step=7,
          note=f"IEC C14 inlet with fuse drawer, rear-mount: {w} x {hh} body flush with the back panel's outer face, "
               f"{fw} x {fh} x {ft} flange behind the panel, 2 x M3 tapped at {IEC_PITCH:.0f} mm pitch "
               "(representative envelope)")
    A.vendor.append(V.manifest_entry("v_iec_inlet", "v_iec_inlet"))
    x, w, hh, ya, yb = USB
    A.add("v_usb_panel", _solid(_dev_body(x, w, hh, ya, yb)), "switch", "electrical", step=7,
          note="USB-C panel-mount extension through the back panel, front flush with its outer face")
    A.vendor.append(V.manifest_entry("v_usb_panel", "v_usb_panel"))
    A.add("v_grommet_g2", V7._grommet(SHEET_Z1, PANEL_T, "v_grommet_g2").moved(Location((G2[0], G2[1], 0))),
          "grommet", "plinth", step=7)
    A.vendor.append(V.manifest_entry("v_grommet_g2", "v_grommet_8mm"))


# ------------------------------------------------------------------ notes

def camera_pov():
    ax = np.array(OPTICAL_AXIS, float)
    ax /= np.linalg.norm(ax)
    return dict(eye=tuple(float(x) for x in LENS), axis=tuple(float(x) for x in ax), fov=120.0,
                hide=("v_camera_uc261", "fp8_camera_wedge"), closeup_focus=(60.0, 0.0, 205.0 + DZF))


def frog_report(A):
    """Frog gaps (mm) in the stage B placement against the v8 cage and ring C."""
    out = {}
    w0, w1 = (np.array(v) for v in A.bbox("v_frog_wall_body"))
    d0, d1 = (np.array(v) for v in A.bbox("v_frog_disc_body"))
    rw, tw = _frog_wall_pose()
    w = V7._frog_verts() @ rw.T + tw
    rd, td = _frog_disc_pose()
    d = V7._frog_verts() @ rd.T + td
    r = np.hypot(w[:, 0], w[:, 1])
    out["gap_net"] = round(float(MESH_R[0] - r.max()), 2)
    out["gap_mesh_top"] = round(float(MESH_TOP_Z[0] - d[:, 2].max()), 2)
    out["gap_tube"] = round(float(r.min() - V7.TUBE_OR), 2)
    out["gap_ring_d"] = round(float(TUBE_TOP - RING_T - w[:, 2].max()), 2)
    out["gap_ring_c"] = round(float(RING_C[0] / 2 - np.hypot(d[:, 0], d[:, 1]).max()), 2)
    out["wall_centre"] = [round(float(x), 2) for x in (w0 + w1) / 2]
    out["disc_centre"] = [round(float(x), 2) for x in (d0 + d1) / 2]
    assert out["gap_net"] >= V7.FROG_NET_GAP, "wall frog too close to the net sleeve"
    assert out["gap_mesh_top"] >= V7.FROG_MESH_TOP_GAP, "disc frog too close to the net top"
    assert out["gap_ring_d"] >= V7.FROG_RING_GAP, "wall frog too close to ring D"
    return out


def frog_offaxis(A):
    lens = np.array(camera_pov()["eye"])
    ax = np.array(OPTICAL_AXIS, float)
    ax /= np.linalg.norm(ax)
    out = {}
    for n in ("v_frog_wall_body", "v_frog_disc_body"):
        mn, mx = (np.array(v) for v in A.bbox(n))
        dv = (mn + mx) / 2 - lens
        out[n] = round(math.degrees(math.acos(float(dv @ ax / np.linalg.norm(dv)))), 1)
    return out


def rotor_inertia(A):
    """Izz (kg m2) and mass (kg) of everything that turns, from each body's volume inertia about Z
    times its material density (lib.materials)."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from lib import materials as M
    izz, mass = 0.0, 0.0
    for n, p in A.parts.items():
        if not (p.module == "rotor" or p.meta.get("spins")):
            continue
        rho = M.MATERIALS[p.material]["density"] * 1e-6          # kg / mm3
        for sol in p.solid.solids():
            g = GProp_GProps()
            BRepGProp.VolumeProperties_s(sol.wrapped, g)
            m = g.MatrixOfInertia()
            c = g.CentreOfMass()
            vol = abs(g.Mass())
            i = abs(m.Value(3, 3)) + vol * (c.X() ** 2 + c.Y() ** 2)       # about the world Z axis
            izz += i * rho * 1e-6                                          # kg mm2 -> kg m2
            mass += vol * rho
    return izz, mass


def rotor_cg(A):
    """Centre of mass (x, y, z mm) and mass (kg) of everything that turns (module rotor plus parts
    tagged spins), from each body's volume and centroid times its sheet density (lib.materials)."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from lib import materials as M
    m_tot, mom = 0.0, np.zeros(3)
    for n, p in A.parts.items():
        if not (p.module == "rotor" or p.meta.get("spins")):
            continue
        rho = M.MATERIALS[p.material]["density"] * 1e-6          # kg / mm3
        for sol in p.solid.solids():
            g = GProp_GProps()
            BRepGProp.VolumeProperties_s(sol.wrapped, g)
            c = g.CentreOfMass()
            m = abs(g.Mass()) * rho
            m_tot += m
            mom += m * np.array([c.X(), c.Y(), c.Z()])
    return mom / m_tot, m_tot


def power_notes(A):
    from lib import materials as M
    cg, m = rotor_cg(A)
    off = math.hypot(cg[0], cg[1])
    A.meta_rotor_cg = dict(x=round(float(cg[0]), 3), y=round(float(cg[1]), 3), z=round(float(cg[2]), 2),
                           mass_kg=round(m, 4))
    bank_v = A["v_power_bank"].volume
    bank_g = bank_v * M.MATERIALS["power_bank"]["density"] * 1e-3
    tot = PI_MA + CAM_MA + LED_MA
    watts = tot * 5.0 / 1000
    hours = BANK_WH * BANK_EFF / watts
    idle = 100 + CAM_MA
    return [
        "rotor power (stage C): no Qi coils, no slip ring, no loose lithium cell. A certified USB-C power bank "
        f"(Anker 321 / PowerCore 5K, A1112: {BANK_L:g} x {BANK_H:g} x {BANK_W:g}, {BANK_MASS_G:g} g, 5200 mAh, "
        "12 W) stands on its long side in fp8_bank_cradle on the -Y half of the disc (lying flat it does not fit "
        "beside the camera board inside the tube). Wiring, cable-tied to the disc: bank USB-A out -> short lead -> "
        f"v_power_switch (KCD1-101 rocker in the 5 V lead, r {PWR_R:g} at {SWITCH_ANG:g} deg, rocker facing down) -> "
        f"Pi micro-USB; bank USB-C (its input) <- v_charge_port (round USB-C panel mount, r {PWR_R:g} at "
        f"{PORT_ANG:g} deg, facing down). Both are under the disc rim outside the tube: reachable with the cage "
        "lifted and the drum left on. Modelled bank mass "
        f"{bank_g:.1f} g (material power_bank density chosen so the modelled outline weighs the published 118 g)",
        "drum speed: direct drive, so the closed-loop stepper's own encoder (17HS19-2004D-E1K on the CL42T) reads "
        "drum speed; no index sensor: the A3144 hall, its clip and the two 6 x 3 magnets are removed",
        f"rotor balance: rotor_cg_offset_mm={off:.2f} (CG x {cg[0]:.2f}, y {cg[1]:.2f}, z {cg[2]:.1f}; turning mass "
        f"{m:.3f} kg, every body in module rotor plus the spinning electrics, volume x sheet density). The bank "
        "cannot sit on the axis (camera wedge), so v_balance_bar (steel "
        f"{BAR_H:g} x {BAR_W:g} x {BAR_L:g} under the disc at y {BAR_Y:g}, x {BAR_X:g}) squares its moment with the "
        "Pi at +Y",
        f"rotor_power_budget={PI_MA} mA (Pi Zero 2 W, Wi-Fi streaming) + {CAM_MA} mA (UC-261 camera) + {LED_MA} mA "
        f"(24 IR LEDs as 8 series strings of 3 at 20 mA) = {tot} mA at 5 V ({watts:.2f} W), {BANK_WH:.2f} Wh bank "
        f"(rated 5200 mAh x 3.7 V) => {hours:.1f} h (assumes {BANK_EFF * 100:.0f} % cell-to-5 V conversion in the "
        "bank)",
        f"bank auto-off: Anker does not publish the A1112 low-current cut-off (typically about 50 to 100 mA for "
        f"Anker banks, unverified); the rotor's idle draw (Pi idle about 100 mA + camera {CAM_MA} mA = {idle} mA, LEDs "
        "off) is above that, so the bank should stay on without a keep-alive load; if it ever drops out, its "
        "trickle-charging mode (double-press the button) holds the output on for low loads",
    ]


# ------------------------------------------------------------------ build

def build():
    A = Assembly("froggy_v8")
    R_ = _frame(A)
    _brackets(A, R_)
    _feet(A, R_)
    _drive(A)
    _rotor(A)
    _panels(A, R_)
    _electrics(A)
    _cage(A, R_)
    A.meta_vent_clear = _vent_clearance(A)

    down, up = (0, 0, -1), (0, 0, 1)
    cross = {45: R_["cross_right"], -45: R_["cross_left"]}
    for x, y in MPLATE_RAIL_HOLES:
        bolt_joint(A, ["lp_motor_plate", cross[int(x)]], (x, y, MPLATE_Z1), up, screw="M5x8",
                   name=f"j_motor_plate_{_c(x, y)}_cross_rail", module="drive_base", contact="faces", step=1, **M5)
    for k, (x, y) in enumerate(MOTOR_HOLES, 1):
        tapped_joint(A, ["lp_motor_plate", V7.MOTOR], (x, y, MOTOR_FACE_Z), down, screw="M3x14",
                     name=f"j_motor_face_{_c(x, y)}", module="drive_base", evaluate=False, step=1,
                     stack=[(f"v_motor_spacer_{k}", MOTOR_SPACER)])
    for x, y in PA_RAIL_HOLES:
        bolt_joint(A, ["lp_plate_a", cross[int(x)]], (x, y, 0.0), down, screw="M5x12",
                   name=f"j_plate_a_{_c(x, y)}_cross_rail", module="drive_base", step=2,
                   stack=[("s_panel_deck", PANEL_T)], **M5)
    for k, (x, y) in enumerate(STANDOFF_XY, 1):
        so = f"v_standoff_m5_{k}"
        tapped_joint(A, ["lp_plate_a", so], (x, y, PA_Z1), up, screw="M5x12", head="button",
                     name=f"j_standoff_{k}_plate_a", module="drive_base", evaluate=False, step=2,
                     bench=["lp_plate_a", so, "v_kfl08_lower"])
        tapped_joint(A, ["lp_plate_b", so], (x, y, PB_Z0), down, screw="M5x12", head="button",
                     name=f"j_standoff_{k}_plate_b", module="drive_base", evaluate=False, step=2)
    for nm, plate, z, bench in (("v_kfl08_lower", "lp_plate_a", PA_Z1, ["lp_plate_a", "v_kfl08_lower"]),
                                ("v_kfl08_upper", "lp_plate_b", PB_Z1, None)):
        for x, y in KFL_HOLES:
            hex_nut_joint(A, [nm, plate], (x, y, z), down, screw="M4x12", name=f"j_{nm[2:]}_{_sg(x)}x",
                          module="drive_base", evaluate=False, step=2, bench=bench)
    for k, (x, y) in enumerate(CR_SCREWS, 1):
        hex_nut_joint(A, ["fp8_bank_cradle", "lp_carrier_disc"], (x, y, DISC_Z1), down, screw="M3x10",
                      name=f"j_bank_cradle_{k}", module="rotor", evaluate=False, step=4)
    for k, (x, y) in enumerate(BAR_SCREWS, 1):
        tapped_joint(A, ["lp_carrier_disc", "v_balance_bar"], (x, y, DISC_Z0), down, screw="M3x8",
                     name=f"j_balance_bar_{k}", module="rotor", evaluate=False, step=4)
    for x, y in HUB_HOLES:
        tapped_joint(A, ["lp_carrier_disc", V7.HUB], (x, y, DISC_Z0), down, screw="M4x12",
                     name=f"j_hub_disc_{_c(x, y)}", module="rotor", evaluate=False, step=4, head="csk",
                     seat=(x, y, DISC_Z1))
    for k, (x, y) in enumerate(_polar(RING_B_R, RING_SCREW_ANG), 1):
        hex_nut_joint(A, ["lp_ring_b", "lp_carrier_disc"], (x, y, DISC_Z1), down, screw="M3x10",
                      name=f"j_ring_b_{k}", module="rotor", evaluate=False, step=4)
    for k, (x, y) in enumerate(WEDGE_SCREWS, 1):
        tapped_joint(A, ["fp8_camera_wedge", "lp_carrier_disc"], (x, y, DISC_Z1), down, screw="M3x6",
                     name=f"j_camera_wedge_{k}", module="rotor", evaluate=False, step=5)
    for k, (x, y) in enumerate(_polar(CLAMP_R, RING_SCREW_ANG), 1):
        hex_nut_joint(A, ["lp_ring_c", "lp_ring_d"], (x, y, TUBE_TOP), down, screw="M3x16",
                      name=f"j_ring_clamp_{k}", module="rotor", evaluate=False, step=6,
                      stack=[("lp_ring_e", TOP_DISC_T)])
    for k, (x, y) in enumerate(A.meta_psu_holes, 1):
        tapped_joint(A, ["s_panel_bottom", "v_psu_lrs_50_24"], (x, y, FLOOR_Z), up, screw="M3x8",
                     name=f"j_psu_panel_{k}", module="electrical", evaluate=False, step=7)
    for k, (x, y) in enumerate(fp7_controller_carrier.INSERTS, 1):
        wx, wy, _ = _cc_world(x, y)
        insert_joint(A, ["s_panel_bottom", "fp7_controller_carrier"], (wx, wy, FLOOR_Z), up, screw="M3x7",
                     name=f"j_controller_panel_{k}", module="electrical", evaluate=False, step=7)
    for k, (x, y) in enumerate(_cl42t_holes(), 1):
        at = np.array([x, y, FLOOR_Z])
        seat = at + np.array([0, 0, CL42T_EAR_T])
        L = int(math.ceil(seat[2] - BOTP_Z0 + HW.HEX_NUT["M4"]["m"] + 0.5))
        hex_nut_joint(A, ["v_cl42t_driver", "s_panel_bottom"], at, down, screw=f"M4x{L}", name=f"j_cl42t_panel_{k}",
                      module="electrical", evaluate=False, step=7, seat=seat)
    for k, sx in enumerate((-1, 1), 1):
        tapped_joint(A, ["s_panel_back", "v_iec_inlet"], (IEC[0] + sx * IEC_PITCH / 2, PANEL_IN, PANEL_DEV_Z),
                     (0, -1, 0), screw="M3x6", name=f"j_iec_panel_{k}", module="electrical", evaluate=False, step=7)

    # ---------------------------------------------------------------- notes
    n_br = len(A.names("bracket"))
    A.notes.append(f"frame: 17 x 2020 rails (lib.profile_2020: slot {P.SLOT_OPEN}, depth {P.SLOT_DEPTH}, cavity "
                   f"{P.CAVITY} under a {P.LIP} lip, bore {P.BORE}), {n_br} cast corner brackets (envelope), every rail "
                   "joint M5 button head + 2020 drop-in T-nut; cage posts "
                   f"{POST_L:g} (+{HOOP_RISE} with the taller rotor)")
    A.notes.append(f"stage B datums: PLATE_T {PLATE_T} (all lp_plate_* / lp_motor_plate / lp_carrier_disc, aluminium "
                   f"6061); motor plate Z {MPLATE_Z0:g}..{MPLATE_Z1:g}, MOTOR_FACE_Z {MOTOR_FACE_Z:g} on {MOTOR_SPACER:g} "
                   f"spacers, coupling Z {COUPLING_Z0:g}..{COUPLING_Z0 + COUPLING_L:g}; plate A Z {PA_Z0:g}..{PA_Z1:g}, "
                   f"standoffs {STANDOFF_L:g}, plate B Z {PB_Z0:g}..{PB_Z1:g}; KFL08 mounting faces Z {PA_Z1:g} and "
                   f"{PB_Z1:g}; collar Z {COLLAR_Z0:g}..{COLLAR_Z0 + COLLAR_W:g}; hub top {HUB_TOP_Z:g} (lobe face "
                   f"{HUB_LOBE_Z:g}); carrier disc Z {DISC_Z0:g}..{DISC_Z1:g} = drum floor; tube top {TUBE_TOP:g}; "
                   f"shaft Z {SHAFT_Z0:g}..{SHAFT_Z1:g}")
    kz = [(A.bbox(n)[0][2] + A.bbox(n)[1][2]) / 2 for n in ("v_kfl08_lower", "v_kfl08_upper")]
    A.notes.append(f"bearing spread (KFL08 centres): {kz[1] - kz[0]:.2f} mm")
    A.notes.extend(power_notes(A))
    izz, m = rotor_inertia(A)
    A.meta_rotor = dict(izz_kg_m2=round(izz, 6), mass_kg=round(m, 3))
    A.notes.append(f"rotor inertia: Izz {izz:.5f} kg m2, mass {m:.3f} kg (everything that turns; v7 note "
                   "0.0038 kg m2 for the printed carrier rotor)")
    A.notes.append(f"camera: Arducam UC-261 class on fp8_camera_wedge, optical axis 30 deg to +X, lens front "
                   f"{LENS_ABOVE_FLOOR:g} above the drum floor at Z {LENS[2]:.2f}; M2 hole pitch {CAM_HOLE_PITCH} assumed")
    A.notes.append("bottom panel: face-mounted under the bottom ring (Z -113..-110, 8 x M5), feet bolt to the rail "
                   "through corner notches; electrics sit on it at Z -110")
    A.notes.append(f"cage: hoop pairs Z {HOOP_Z['bottom']:g}..{HOOP_Z['bottom'] + HOOP_H:g} and "
                   f"{HOOP_Z['top']:g}..{HOOP_Z['top'] + HOOP_H:g} on 6 fp8_cage_clip (3 posts x 2); mesh sleeve Z "
                   f"{MESH_Z[0]:g}..{MESH_Z[1]:g}, top Z {MESH_TOP_Z[0]:g}")
    A.notes.append("vents: clearance through the sheet + 3: " + ", ".join(
        f"{k} {g} ({n})" for k, (g, n) in A.meta_vent_clear.items()))
    A.notes.append(f"motor: {motor_length()}")
    A.notes.append(f"frog off-axis from the camera optical axis (deg): {frog_offaxis(A)}")
    A.notes.append(f"frog gaps (mm): {frog_report(A)}")
    A.camera_pov = camera_pov()
    A.rotor_top = dict(z_cut=150.0, z_min=DISC_Z0 - 1.0, r_max=135.0, hide=("s_drum_tube",))
    return A
