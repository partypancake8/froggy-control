"""Froggy V12: complete V10 machine with improved motor engagement and end retention.

ICD-V12.md supersedes drive datums only. Motor plate 64 x 120 x 6, four stock
20 mm spacers; motor face -23. Motor pulley hub up. Stock 72 mm REX shaft,
M4 end retainers and catalogue spacer/shim stack. Frame, rotor, cage, wiring
representations retain V11 geometry. V12 adds flat-tread Ender wheels, revised supports, and metal deck hardpoints. Not load-qualified.
"""
import copy
import math
import os
from functools import lru_cache

import numpy as np
from build123d import Align, Box, Compound, Cylinder, GeomType, Location, Plane, Solid, extrude, import_step, scale

from assemblies import froggy_v7 as V7
from assemblies import froggy_v8 as V8
from assemblies.frame import _location
from lib import hardware as HW
from lib import profile_2020 as P
from lib import vendor as V
from lib.assembly import Assembly
from lib.design_constants import LASER, PRINTED, min_edge_margin
from lib.joints import bolt_joint, hex_nut_joint, insert_joint, tapped_joint
from parts import drive_v11 as BD
from parts import rollers_v12 as RW
from parts import datums as D0
from parts import fp6_datums as D6
from parts import fp6_drum_carrier, fp7_controller_carrier, fp9_cage as CG, fp9_estop_cradle, s_drum_tube, s_top_disc

_R, _box, _solid, _c, _sg = V7._R, V7._box, V7._solid, V7._c, V7._sg

# S2.3 / S2.4 (2026-10-12): legs 80 (Z -100..-20) and the bottom ring at Z -120..-100, so the 17HS19 cartridge with a
# penetration-safe coupling fits; bottom panel, feet and plinth electrics follow 10 lower. The v8 plinth builders read
# these module datums when they run, so v9 sets them on V8 once, here, before anything is built.
V8.BOT_Z0, V8.BOT_Z1 = -120.0, -100.0
V8.LEG_Z0 = V8.BOT_Z1
V8.BOTP_Z0, V8.BOTP_Z1 = V8.BOT_Z0 - V8.PANEL_T, V8.BOT_Z0
V8.FLOOR_Z = V8.BOTP_Z1
V8.CC_DZ = V8.FLOOR_Z - fp7_controller_carrier.Z0
# I020 / I021 (2026-10-16 sponsor meeting, DL29): the base square is gone; the panels hang from the deck rails (2 x M5
# per panel at the deck rail centre line). I029 (2026-10-21): four 2020 legs at the deck corners carry the machine down to
# the plinth floor (LEG_TOP..LEG_BOT below); the panels also hang from the legs' outer slots. BOT_Z0 stays the plinth floor
# datum (bottom panel top face, electrics on it); V8.BOT_Z1 / V8.LEG_Z0 are no longer frame datums.
V8.PANEL_SCREW_Z = ((V8.DECK_Z0 + V8.DECK_Z1) / 2,)
# DL21 (2026-10-12): the right panel's vent row centred on the panel (h 0) like the left panel's, not beside the PSU
V8.VENT_H = {**V8.VENT_H, "right": 0.0}

# ------------------------------------------------------------------ frame (v8, posts per ICD S3.1)
W = P.W
HALF, RC, RIN = V8.HALF, V8.RC, V8.RIN
CROSS_X = V8.CROSS_X
DECK_Z0, DECK_Z1 = V8.DECK_Z0, V8.DECK_Z1
BOT_Z0, BOT_Z1 = V8.BOT_Z0, V8.BOT_Z1
LEG_Z0, LEG_Z1 = V8.LEG_Z0, V8.LEG_Z1
PANEL_T = V8.PANEL_T
SHEET_Z0, SHEET_Z1 = V8.SHEET_Z0, V8.SHEET_Z1
M5_HOLE = 5.5
BR_L = V8.BR_L
G_D, G2 = V8.G_D, V8.G2
SHEET_SCREWS = V8.SHEET_SCREWS
M5 = dict(nut="slot_nut", head="button", evaluate=False)

# ------------------------------------------------------------------ legs, plinth shell and clamp tabs (DL29, I021, I023, I029)
# I029 (2026-10-21, Sawyer: "no that will not work, sitting on the arylic, maybe 4 posts out of 8020 ? as legs and the plinth
# adds rigidity?"): four 2020 legs at (+-RC, +-RC) from the deck rail underside down to the bottom panel's top face. Each
# leg hangs from the deck X rail on one cast bracket (inboard X face); its bottom end tap takes the foot bolt, which clamps
# the bumper foot and the bottom panel under the leg end. The panels are skin: 2 x M5 x 8 + T-nut per leg edge into the
# legs' outer slots plus the deck rail pair; acrylic and PETG carry nothing (DL29).
LEG_TOP, LEG_BOT = DECK_Z0, BOT_Z0          # -20 (deck rail underside) .. -120 (bottom panel top face)
LEG_L = LEG_TOP - LEG_BOT                   # 100
LEG_XY = RC                                 # 140: leg axes, the deck rail centre lines (outer faces on the panel faces)
LEG_TAP_DEPTH = 12.0                        # M5 tapped 12 deep in the leg's 4.2 centre bore at the bottom end (assumed)
LEG_PANEL_Z = (LEG_TOP - 15.0, LEG_BOT + 15.0)   # -35 / -105: panel screws into each leg's outer slots
FOOT_SCREW = "M5x16"                        # head in the foot's 10.5 x 5 counterbore, 3 rubber + 3 sheet, 10 into the leg tap
CT_T = 6.0                                  # lp_clamp_tab: 6061 PLATE_T off the Fablight
CT_Z0 = PANEL_T                             # on the deck sheet top (Z 3), over the deck rail corner
CT_HOLE = HALF - 31.0                       # 119: one M5 hole on each deck rail's top-slot line (+-140)
CT_EDGE = HALF + 1.0                        # 151: outer edges 11 = 2 x d from the slot-line holes (DL8), inside the 153 panel faces
CT_ARM_END = CT_HOLE - 11.0                 # 108: arm ends 11 past the holes (sheet screw heads at 100 + 4.75 stay clear)
CT_ARM_IN = RC - 11.0                       # 129: arm inner edges 11 inside the slot line
# assumed jaw (I023, recorded in the sheet notes): F clamp or wood quick clamp, pad 25 wide, reach 30 in from the panel
# face, 10 thick; opening at least 180 (tab top Z 9 to the underside of a 40 thick bench top under the feet)
CLAMP_JAW = dict(reach=30.0, thick=10.0, pad=25.0, opening_min=180.0)

# ------------------------------------------------------------------ S1 datums
PLATE_T = 6.0           # lp_plate_a (ICD S1.4 PLATE_T) and the carrier disc: 6 mm 6061
DISC_T = 6.0
DISC_D = 186.0
FAB_T = 3.175           # 1/8 in 6061 off the Fablight: rings B / C / D / E (motor plate is 6 mm) (rule 1)
# S1.1 / S1.2 motor plate and motor (the drive base is a cartridge on plate A, S1.26)
SPACER_L, SPACER_OD, SPACER_ID = 20.0, 10.0, 5.3  # RAF M0669-5-AL, S1.1-V12
SPACER_XY = [(sx * 21.0, y) for sx in (1, -1) for y in (1.0, -45.0)]
SPACER_NAMES = [f"v_spacer_raf_m0669_5_al_{k}" for k in range(1, 5)]
SPACER_SCREW_L = 40.0  # 32 mm grip + nyloc; S1.1-V12

MPLATE_T = 6.0
MPLATE_Z1 = SHEET_Z1 - SPACER_L  # -17
MPLATE_Z0 = MPLATE_Z1 - MPLATE_T  # -23
MPLATE_X, MPLATE_Y = 64.0, 120.0            # long side along Y, passes between the cross rail inner faces (X +-35)
MOTOR_HOLES = BD.motor_holes()  # V12: square pattern at (0, 30), tension slots
MOTOR_PILOT_D = 24.0                        # ICD S1.2 (ds): drawing boss 22 -0.05, the STEP root fillet reaches 23.5
MPLATE_BELT_SLOT_W = 20.0                   # I035: 16 left the slot edge 1.56 from the belt; 20 gives >= 3.0 (BRepExtrema, C 28.5..30.5)
MPLATE_BELT_SLOT_R = 2.0                    # the four slot-to-circle cusps rounded (DL8 inside corner look)
MOTOR_FACE_Z = MPLATE_Z0
MOTOR = "v_nema17_17hs19_2004d_e1k"
MOTOR_DIR = "motion/stepperonline_17hs19_2004d_e1k"
MOTOR_STEP = MOTOR_DIR + "/17HS19-2004D-E1K.step"
MOTOR_SHAFT_L = 24.0                        # drawing 24 +-1, D-cut 15 from the tip; STEP shaft local Y -4..24
MOTOR_SHAFT_TIP = MOTOR_FACE_Z + MOTOR_SHAFT_L  # +1 nominal, 0..2 with vendor tolerance
MOTOR_SCREW = "M3x10"  # 10 - 0.5 washer - 6 plate = 3.5 mm engagement
DECK_OPEN = (66.0, 126.0)                   # S2.5 centre opening X x Y, between the cross rail inner faces
# V12 S1.3: drive_v11 holds vendor pulley datums and closed belt geometry.
# S1.4 / S1.5 plate A and pillow block
PA_Z0 = SHEET_Z1                            # 3
PA_Z1 = PA_Z0 + PLATE_T                     # 9
PA_SIZE = 120.0
PA_RAIL_HOLES = [(sx * CROSS_X, sy * 40.0) for sx in (1, -1) for sy in (1, -1)]
PA_CENTRE_D = 16.0                          # ICD text 30: the block's 16 mm M4 pattern (r 11.3) lies inside 30; 16 clears the 9.95 race spacer by 3.0 radial (fix A)
BLOCK_HOLES = [(sx * 8.0, sy * 8.0) for sx in (1, -1) for sy in (1, -1)]
BLOCK_HOLE_D = 4.3
BLOCK_H = 16.0
BLOCK_Z0, BLOCK_Z1 = PA_Z1, PA_Z1 + BLOCK_H  # 9..25
# S1.7 hub
HUB_ANG = 30.0                              # hub turned so its REX bore flats meet the shaft (and the block's)
HUB_BOSS_Z = BLOCK_Z1                       # 25: 11 dia boss on the upper inner race
HUB_FLANGE_TOP = HUB_BOSS_Z + 20.0          # 45
HUB_PILOT_D, HUB_PILOT_H = 14.0, 2.0
HUB_H = 22.0                                # measured vendor STEP: 0.6 boss + clamp + shoulder + 6 flange + 2 pilot, 25..47
# S1.6 shaft retention, fix A (I033, 2026-10-28; interface-risks rows 2, 4, 5, 9, 17b, 17c): the 2920 collar is gone.
# On the lower inner race a 1522 spacer (8.05 ID x 9.95 OD x 6.00, measured) runs down through plate A's 16 hole and
# SHIM_N 2807 shims (8.00 x 11.0 x 1.00, measured) hang under plate A; the 3422 REX pulley (18.0, hub up) sits directly
# under the shims and its two M5 set screws (axis 5.0 below the pulley's hub-end tip, measured) are the lower axial
# stop. The block is captured between the hub boss (25) and the spacer column (9): set by pushing the pulley up and
# tightening its set screws on the bench. Measure then choose: SHIM_N is the nominal count only.
SHAFT_L = 72.0  # stock goBILDA 2106-4008-0720, 72.00 measured
SHAFT_SQUEEZE = 1.0  # use nominal vendor geometry, no invented fit clearance
SHAFT_PROUD = 0.2                           # shaft end 0.1..0.3 proud of the hub pilot (interface-risks rows 4, 17c)
SHAFT_Z1 = HUB_FLANGE_TOP + HUB_PILOT_H + SHAFT_PROUD   # 47.2: the upper end washer bears on the shaft, not the hub
SHAFT_Z0 = SHAFT_Z1 - SHAFT_L               # -24.8
RACE_SPACER_H, SHIM_H = 6.0, 1.0            # goBILDA 1522-0010-0060, 2807-0811-1000 (measured STEPs)
SHIM_N = 3                                  # shim_count_nominal (pulley hub top 1.3 under the block screw heads)
RACE_SPACER_Z0 = BLOCK_Z0 - RACE_SPACER_H   # 3 = plate A underside
SHIM_Z0 = RACE_SPACER_Z0 - SHIM_N * SHIM_H  # 0
REX_H = 18.0                                # 3422 overall, measured
REX_TOP = SHIM_Z0                           # 0: the pulley's hub-end tip on the lowest shim
REX_GRUB_FROM_TOP = 5.0                     # 3422 STEP: set screw axis 5.0 from the hub-end tip (holes 2.5..7.5)
REX_GRUB_Z = REX_TOP - REX_GRUB_FROM_TOP    # -5.0
BACKUP_SPACER_Z0 = SHAFT_Z0                 # lower 1522 on the lower end washer, -24.8..-18.8, 0.8 under the pulley
RACE_SPACER = "v_retainer_race_spacer_8x6"
RACE_SHIMS = [f"v_retainer_race_shim_8x1_{k}" for k in range(1, SHIM_N + 1)]
BD.REX_TOP = REX_TOP
BD.BELT_Z = REX_TOP - 13.0                  # -13.0: the groove centre 13.0 under the hub-end tip (was -12.75)
BD.MOTOR_ANGLE = 180.0                      # motor turned 180 about its axis: lead glands face +Y, away from the spindle
#                                             (interface-risks 17a); the M3 square and the D flat turn with it
_RACE_NOTE = ("fix A (I033): on the lower inner race, through plate A's 16 hole; spacer + shims + REX pulley set screws "
              "are the lower axial stop; measure then choose the shim count")
BD.RETAINER_NAMES = (BD.UPPER_WASHER, BD.LOWER_WASHER, BD.LOWER_SPACER, RACE_SPACER, *RACE_SHIMS)
BD.RETAINER_WASHERS = ((BD.UPPER_WASHER, SHAFT_Z1), (BD.LOWER_WASHER, SHAFT_Z0 - 1.0))
BD.RETAINER_STOCK = (
    (BD.LOWER_SPACER, "hardware/gobilda_1522_8x6", "1522-0010-0060.step", BACKUP_SPACER_Z0,
     "fix A (I033): backup retainer under the REX pulley, sits on the lower end washer, 0.8 under the pulley, never "
     "clamped; it covers the e-clip groove (remove the factory e-clip)"),
    (RACE_SPACER, "hardware/gobilda_1522_8x6", "1522-0010-0060.step", RACE_SPACER_Z0, _RACE_NOTE),
    *((nm, "hardware/gobilda_2807_8x1", "2807-0811-1000.step", SHIM_Z0 + k * SHIM_H, _RACE_NOTE)
      for k, nm in enumerate(RACE_SHIMS)))                # shim 1 lowest (on the pulley), shim N under the spacer
BD.RETAINER_ENDS = (("upper", BD.UPPER_WASHER, SHAFT_Z1, (0, 0, -1), SHAFT_Z1 + 1.0),
                    ("lower", BD.LOWER_WASHER, SHAFT_Z0, (0, 0, 1), SHAFT_Z0 - 1.0))
BOTP_Z1_V8 = V8.BOTP_Z1                     # bottom panel top face (-110)
# S1.8 carrier disc
DISC_Z0, DISC_Z1 = HUB_FLANGE_TOP, HUB_FLANGE_TOP + DISC_T
DRUM_FLOOR = DISC_Z1                        # 51
HUB_R32 = 16.0 * math.sqrt(2.0)
HUB_HOLE_ANG = [45.0 + HUB_ANG + 90.0 * k for k in range(4)]   # 75, 165, 255, 345
DZF = DRUM_FLOOR - D6.DRUM_FLOOR_Z          # drum and everything on it vs v7
# S1.27 rim rollers: support the rim at r 90, set touching the disc underside; S1.28 running band r 85.95..92.05
ROLLER_R, ROLLER_ANG = 89.0, (90.0, 210.0, 330.0)
TYRE_OD, TYRE_W = 24.1, 10.2                 # Creality standard Ender wheel: 688-2RS (8 x 16 x 5) with a bonded 85A polyurethane tread
BRG_ID, BRG_OD, BRG_W = 5.0, 16.0, 11.0
ROLLER_Z = DISC_Z0 - TYRE_OD / 2            # 34: axle height with the tread top at the disc underside (zero gap)
ROLLER_NAMES = [f"v_rim_roller_creality_4001070002_{k}" for k in range(1, 4)]
RBLOCK_NAMES = [f"fp12_roller_block_{k}" for k in range(1, 4)]
AXLE_SPACER_NAMES = [f"v_spacer_axle_10x5x1_{k}" for k in range(1, 4)]
# M5x25 axle head outboard, two external 1 mm precision shims and DIN985 nut inboard.
AXLE_SPACER_L = 1.0
RB_BASE_T = 4.0                             # printed block base under the M4 heads (M4 x 16: 4 + 6 plate A + 5 nyloc + 1)
RB_X0, RB_X1 = 67.0, ROLLER_R + BRG_W / 2 + AXLE_SPACER_L + 4.0   # base r 67 .. 99.5 (outboard wall 4 thick)
RB_WALL_X0 = ROLLER_R + BRG_W / 2 + AXLE_SPACER_L                # 95.5: outboard wall inner face (2.5 off the disc edge)
RB_HALF_W, RB_SIDE_Y0 = 17.0, 13.05          # block +-17 wide; side walls from 12.5 (tyre +-11)
RB_SIDE_TOP = DISC_Z0 - 3.0                 # 42: sides stop 3 under the disc, the tyre's top 3 mm stays exposed
RB_WALL_TOP = DISC_Z1 - 1.0                 # 50: the outboard wall stands beside the disc edge (r 95.5 vs 93)
RB_SLOT_HALF, RB_SLOT_W = 2.0, 5.4          # vertical axle slot: centre Z 34 +- 2, 8.2 wide (8 mm shoulder)
RB_HOLES = [(76.0, -8.0), (76.0, 8.0)]       # 2 x M4 x 16 + nyloc to plate A (block-local r, t)
ARM_X1, ARM_HALF_W = 100.0, 19.0            # plate A arms out to r 100, 38 wide, under each block
DECK_NUT_HOLE_D = 12.0                      # deck-sheet clearance under the six block nylocs (they hang below plate A)
# S2.10 e-stop: the complete BACO LBX10510 station (LBX single-position box 74 x 74 x 47.9, yellow cover, 40 mm red
# push-turn-reset mushroom, 1 NC contact block), all 26 solids of BACO's STEP, recessed in the front panel (option A):
# the box passes through a 76 x 76 opening, its yellow cover face ES_PROUD outside the panel, its back on
# fp9_estop_cradle behind the panel. Clear band behind the panel: deck rail underside Z -20 to bottom ring top Z -100.
ESTOP = "v_estop_baco_lbx10510"
ES_STEP = "vendor/electrical/baco_lbx10510/lbx10510.stp"   # BACO's own STEP AP214 (26 solids, all used)
ES_W = 74.0                                  # box outline (STEP base / cover), corners about R 3
ES_DEPTH = 47.9                              # STEP: base back pads (local Y -0.7) to cover face (local Y 47.2)
ES_FACE_LOCAL_Y, ES_PAD_LOCAL_Y = 47.2, -0.7
ES_BOX_C_LOCAL = (0.0, -33.0)                # box centre (X, Z) in the STEP
ES_XC, ES_ZC = 0.0, -60.0                    # box centre on the front panel: centred in the 80 band (3 to each rail);
#                                              X 0: the cradle back is 75 in front of the motor (Y -31), no move needed
ES_Y0 = -(V8.PANEL_IN + PANEL_T)             # -153: front panel outer face
ES_PANEL_IN = -V8.PANEL_IN                   # -150: front panel inner face, the cradle flange on it
ES_PROUD = 5.0                               # yellow cover face 5 outside the panel's outer face
ES_FACE_Y = ES_Y0 - ES_PROUD                 # -158
ES_BACK_Y = ES_FACE_Y + ES_DEPTH             # -110.1: the base's back pads on the cradle back plate
ES_OPEN, ES_OPEN_R = 76.0, 4.0               # panel opening: 1 clearance per side, corners R 3 + 1
# The station is turned 90 deg about the panel normal from the STEP's upright pose so its two M16/20 knockouts face
# +-X: upright, the bottom (and top) knockout opens straight onto the bottom ring (deck rail) 3 below the box, where no
# gland fits. Turned, the bottom knockout faces -X into the open plinth, the head's axis sits 5 above the box centre.
# The base's 2 slots, world (X, Z) relative to the box centre after the turn: (29, 20.5) and (-30.5, -22).
ES_HOLES_REL = [(29.0, 20.5), (-30.5, -22.0)]
ES_HOLES = [(ES_XC + x, ES_ZC + z) for x, z in ES_HOLES_REL]
ES_FLOOR = 3.7                               # base back wall at the slots: pads (STEP Y -0.7) to inner floor (Y 3.0)
ES_CR_SCREWS = [(ES_XC + x, ES_ZC - y) for x, y in fp9_estop_cradle.INSERTS]   # 4 x M3 x 10 into the cradle inserts
# S1.9 .. S1.13 drum stack D1 (2026-10-12): flat Fablight rings + printed seats, no machining. Load path: hub -> carrier
# disc (6061) -> ring B clamped flat on it (6061); the PETG seats only locate and bond the light tube, and the top
# seat only carries the top disc's clamp.
RING_T = FAB_T                              # ring B and the top bezel: 3.175 6061 (rule 1)
# I036 (2026-10-28): drum tube 150 OD x 2.5 wall (Rebower 145 x 150 x 14 in) replaces the 152.4 x 3.2 tube
TUBE_OD, TUBE_WALL = D0.DRUM12_OD, D0.DRUM12_WALL
TUBE_OR, TUBE_IR = TUBE_OD / 2, TUBE_OD / 2 - TUBE_WALL   # 75.0, 72.5
SEAT_RADIAL = 0.3                           # seat bore clearance per side: print the seats at measured OD + 0.6
RING_B = (144.0, 186.0)                     # ID 144 (r 72): the tube foot r 72.5..75 sits fully on aluminium; clears the
#                                             switch nut (r 70.6) and the port nut (r 70.2)
RING_TOP = (147.0, 186.0)                   # lp_ring_top bezel: hole margin 9.5 to the ID, 10 to the OD (rule 6)
STACK_R = 83.0                              # one 6 x M4 screw circle for both ends: outside the tube (r 75.0),
STACK_ANG = tuple(60.0 * k for k in range(6))   # inside the r 86 running band; 30 deg off the two drum posts
STACK_HOLE_D = 4.5                          # M4 clearance (Fablight through holes)
STACK_TAP_D = 3.3                           # M4 tap drill in the 6 mm disc, tapped through
SEAT_FLOOR = 0.0                            # the tube foot stands on ring B (aluminium); the seat is a socket round it
SEAT_CH_R = (TUBE_IR - SEAT_RADIAL, TUBE_OR + SEAT_RADIAL)   # 72.2, 75.3: tube r 72.5..75.0, 0.3 each side (bore 150.6)
# S1.32 drum posts (2026-10-12, Sawyer 15:05): the structure that carries the top ring, the silicone is a seal
# Posts INSIDE the tube (2026-10-13, I014: "put the supports for the drum internal ... supported by the 3D printed
# piece instead of on the external where the frog might hit them"). r 66: body r 61..71 (10 OD), 1.5 clear of the tube bore (r 72.5).
# Diametric pair (point mirror about the rotor axis, DL27; equal masses 180 deg apart keep the rotor CG on the axis).
# 51.25 / 231.25 deg: +-Y is taken by the Pi (+Y, standoffs at r 66.2, 116 / 64 deg) (the -Y LED driver standoffs, r 57.5,
# are gone since I034), and the switch (140 deg) and port (214.7 deg) nuts close the -X gaps; at 51.25 the ring B lobe
# (r 11 round the stud) clears the nearest Pi standoff and at 231.25 the port nut (margins in the sheet notes).
POST_ANG, POST_R = (51.25, 231.25), 66.0
# 140 long (I017): no 8 OD x 140 listing was found (8 OD stock jumps 120 -> 150), the 10 OD x 140 is a listed stock size.
POST_D, POST_TAP_DEPTH = 10.0, 12.0         # M5 F-F round aluminium standoff 10 x 140 (listing); POST_L = TUBE_H below
POST_VENDOR = "hardware/standoff_m5_10x140_al"
POST_HOLE_D, M5_TAP_D = 8.4, 4.2
STUD = "M5x16"                               # bottom: threaded stud, post -> ring B -> tapped disc, no head below
POST_SCREW = "M5x14"                         # top: button head through the bezel and the top disc into the post
POST_LOBE_R = min_edge_margin(M5_HOLE)      # 11: ring B / bezel lobe round each post hole (DL8: centre 2 x d from edges)
SEAT_B_OUT_R = 90.0
SEAT_B_T = 6.0                              # bottom seat flange height (counterbored M4 heads)
SEAT_CB_D, SEAT_CB_FLOOR = 8.0, 1.2         # bottom seat counterbores: M4 BHCS head 7.6 x 2.2 (DL10: one M4 button family on the drum), 1.2 PETG floor
SEAT_T_T = 6.0                              # top seat ring under the top disc, bore r 75.3 over the tube top
SEAT_FILLET = 2.0
RING_B_Z1 = DRUM_FLOOR + RING_T             # 54.175
SEAT_B_Z0 = RING_B_Z1
TUBE_FOOT = SEAT_B_Z0 + SEAT_FLOOR          # 54.175: the tube foot on ring B
SEAT_B_Z1 = SEAT_B_Z0 + SEAT_B_T            # 60.175: wall band starts here
# ICD S1.10 (I017, 2026-10-13): TUBE_H = band floor (bottom seat flange top) + 8 clear + the 100 tall wall-frog
# placeholder (largest sponsor frog, frogs 2 to 10 cm) + 8 clear under the top seat, rounded to the 140 stock cut
# (Sawyer: "14cm prob"; the band is 128); camera board top >= 40 under the disc.
TUBE_H = 140.0
TUBE_TOP = TUBE_FOOT + TUBE_H               # 194.175 = drum post top
SEAT_T_Z0 = TUBE_TOP - SEAT_T_T             # 188.175: band ceiling
POST_L = TUBE_H                             # S1.32: the post runs ring B top (= tube foot) to the top disc underside
WALL_FROG_H = 100.0                         # S1.10 wall-frog placeholder height (largest sponsor frog, 10 cm)
TUBE_NAME = f"acrylic tube {TUBE_OD:g} x {TUBE_WALL:g} x {TUBE_H:g} (Rebower 145 ID x 150 OD x 14 in, cut to length)"
TUBE_DZ = TUBE_H - D6.DRUM_H                # tube height vs the 152 tube (-52)
TOP_DISC_T = D0.ACRYLIC_T
TOP_DISC_D = 186.0                          # grows to the bezel OD so the 6 clamp screws pass through it
STACK_SCREW_B, STACK_SCREW_T = "M4x10", "M4x18"
CAGE_TOP_SCREW = "M5x18"     # S3.5 cage top tab bolt from above: 6.85 ring stack + 4 bracket leg + 4.7 nut = 15.55,
#                              plus 2 full threads past the nut (the joint check's engagement rule): the shortest stock length
# S1.29 white WS2812B 16 LED ring on the camera bridge (vendor/electronics/ws2812_ring_16, I032; replaces the
# Spinel IRXS10 850 nm IR ring of I015). Vendor STEP: Adafruit NeoPixel Ring 16 (product 1463), the footprint the
# Amazon rings (Sparkleiot B0C77XPKSL, 45 OD / 32 hole) share. Read off the STEP: board 44.45 OD x 31.75 hole x 1.6,
# 16 x 5050 (5.0 x 5.0 x 1.7) on r 19, 22.5 deg pitch, 3.3 overall.
LEDR_STEP = "vendor/electronics/ws2812_ring_16/neopixel_ring_16.step"
LEDR_OD, LEDR_ID, LEDR_T = 44.45, 31.75, 1.6
LEDR_N, LEDR_PKG_H = 16, 1.7
LEDR_Z = 4.3                                # board underside 4.3 in front of the camera board underside (FFC top 4.1),
#                                             the plane the IR board occupied
LEDR_RECESS_D = 45.0                        # 44.45 board + 0.55 (the Amazon ring is listed 45 OD)
LEDR_RECESS_DEPTH = LEDR_T + LEDR_PKG_H     # 3.3: the 5050 tops sit flush with the collar top
COLLAR_OUT, COLLAR_IN = 57.0, 37.0          # 57 square collar round the camera board kept (I032 changes the recess only)
LEDR_MA_5V = 300.0                          # allowance: 16 x 5050 white at about 18 mA each (about 30 % brightness cap,
#                                             about 290 mA); full white is up to 16 x 60 = 960 mA (Adafruit)
# S1.30 index magnets, S1.31 hall bracket (2 pulses per rev)
MAG_R, MAG_ANG = 76.0, (0.0, 180.0)         # r 70 (brief) lies inside the swept switch head (r 69.5) and port flange
MAG_D, MAG_T, MAG_HOLE_D = 6.0, 3.0, 6.1    # (r 72.7); r 76 is the free annulus between them and the band
HALL_AZ = 45.0                              # plate A's square corner reaches r 81.5 on the diagonal
HALL_GAP = 2.0
# S1.16 .. S1.20 camera bridge over the bank (Anker 321, A1112: 96 x 45.2 x 23.1, 118 g) lying flat on the axis
BANK_L, BANK_W, BANK_H = V8.BANK_L, V8.BANK_W, V8.BANK_H     # 96 (along X), 23.1 (lying: height), 45.2 (along Y)
BANK_MASS_G = V8.BANK_MASS_G
BANK_Z0 = DRUM_FLOOR
BR_GAP = 0.3
BR_TOP_T = 3.0
BR_TOP_Z0 = BANK_Z0 + BANK_W + BR_GAP       # 74.4: top plate over the bank
BR_TOP_Z1 = BR_TOP_Z0 + BR_TOP_T            # 77.4
BR_X = 40.0                                 # bridge spans x -40..40 (the bank ends stick out 8 each side)
BANK_OFF = (4.0, -3.1)                      # bank + bridge centre: the axis moved by the layout balance (see notes);
#                                             +2.1 in X on 2026-10-13 (I015) for the heavier flat ring + 57 collar
BR_CHEEK_Y = (BANK_H / 2 + BR_GAP, 31.5)     # cheek inner / outer faces from the bank centre line
BR_SCREWS = [(BANK_OFF[0] + sx * 34.0, BANK_OFF[1] + sy * 27.2) for sx in (1, -1) for sy in (1, -1)]
BR_INSERT_D, BR_INSERT_L = 4.1, 6.0         # I028: M3 heat-set pocket in each lug from the disc face (M3 x 10 button from
#                                             under the disc: 6 disc + 4 insert), so the mount top has no driver bores
BR_LIP = 1.2                                # snap lips along the bank's long top edges
CAM_HOLE_PITCH = 28.5                       # UC-261 M2 pitch: assumed (no drawing found)
OPTICAL_AXIS = (0.5, 0.0, math.cos(math.radians(30)))
# I028 (2026-10-21, Sawyer: "i think the camera internal stand needs rework as well, not sure how it got so high? looks
# like it's in a weird spot"): the I018 riser (lens 88) is gone. The camsim sweep at the 140 tube (lens 48..88, tilt
# 20..40, mount left out) sees 99 % of the top disc and 65 / 63 % of the disc / wall frog at 48, as much as at 88
# (the rest is the frogs' own legs and the drum posts), at 6.8 / 9.7 px/mm; so the collar seats straight on the
# saddle's top plate again: lens front on the axis 48 above the disc, the lowest seat the board clears the plate at.
LENS_ABOVE_FLOOR = 48.0                     # lens front on the axis above the disc
LENS = (0.0, 0.0, DRUM_FLOOR + LENS_ABOVE_FLOOR)
CAM_STACK = 13.6
WEDGE_HALF = 19.0
# S1.18 Pi (the S1.23 LED driver is removed in V12, I034: the WS2812 ring runs straight from the Pi)
PI_C = (0.0, 48.0)
PI_HOLES = [(PI_C[0] + sx * 29.0, PI_C[1] + sy * 11.5) for sx in (1, -1) for sy in (1, -1)]
PI_STANDOFF = 6.0
# S1.21 / S1.22 switch and port (r 85.5, outside the tube)
# S1.28: the switch and the port leave the r 86..93 running band. Between the tube (r 75.0) and r 86 there are 11 mm,
# less than either body (12 through the disc, a 19 flange and 16.2 nut), so both move inside the tube on the -X side
# (never in the camera's +X view): switch at (-48, +40) between the Pi and the bridge, port at (-52, -36) beside the
# bank's USB-C end; both still face down, reached under the disc rim when the cage is off.
SWITCH_XY, PORT_XY = (-48.0, 40.0), (-52.0, -36.0)
SWITCH_ANG = math.degrees(math.atan2(SWITCH_XY[1], SWITCH_XY[0]))
PORT_ANG = math.degrees(math.atan2(PORT_XY[1], PORT_XY[0]))
SWITCH_R, PORT_R = math.hypot(*SWITCH_XY), math.hypot(*PORT_XY)
PORT_FLANGE_D, PORT_FLANGE_T, PORT_BODY_D, PORT_NUT_D, PORT_NUT_T = 19.0, 2.5, 12.0, 14.0, 4.0
PORT_HOLE_D = 12.4
# TRU Components LAS4GQH-11 latching (Conrad 1661896) drawing: head 14 dia x 3.2, button 7.4, M12 x 1 body 36.35
# behind the head, panel up to 10, nut 16.2 across corners (thickness 2.0 assumed), 1NO1NC 48 V 2 A
SW_HEAD_D, SW_HEAD_T, SW_BTN_D, SW_BODY_L, SW_THREAD_D = 14.0, 3.2, 7.4, 36.35, 12.0
SW_NUT_AC, SW_NUT_T = 16.2, 2.0
SW_HOLE_D = 12.2
WIRE_HOLE_D = 7.0
WIRE_HOLES = []                              # both devices inside the tube: their leads stay on the disc top
# S1.24 trim
TRIM_R, TRIM_ANG = 80.0, (45.0, 135.0, 225.0, 315.0)
TRIM_TAP_D = 4.2
PI_MA, CAM_MA = V8.PI_MA, V8.CAM_MA
LED_MA = round(LEDR_MA_5V)                  # S1.29 WS2812 ring straight off the 5 V rail (no boost, I032)
BANK_WH, BANK_EFF = V8.BANK_WH, V8.BANK_EFF

# ------------------------------------------------------------------ S3 cage: tube-post cube, magnet-on mesh panels (ICD S3.1 .. S3.6)
# I030 (2026-10-21, Sawyer: "really the cage should be a structure that is put over some alu posts or soemthing like
# that, that forms a cube ideally the panels have magnets on them so you can pull any one off like a mesh door thing";
# "yeah frame footprint sounds good"): four 12 x 1 6061 tube posts at the deck corners in printed foot corners, a top
# square of four tubes in printed 3-way corners, five magnet-on mesh panels (parts/fp9_cage.py). The posts stand inboard
# of the clamp tabs (arm ends at 108, pads 124..151 stay free) and of the deck rail T-nut rows; each foot's tongue runs
# out to one M5 x 12 button head into a T-nut in the deck Y rail at (+-140, +-97), 3 clear of the tab arm end.
CAGE_P = 99.0                                # post axes at (+-99, +-99): node faces (panel plane) at 108.7
CAGE_FACE = CAGE_P + CG.NH                   # 108.7: inner face of every panel, outer faces of the corner nodes
CAGE_POST_L = 256.0                          # cut length: the top nodes' top face lands at Z 300 (the proposal's 300 tall cube)
CAGE_POST_Z0 = SHEET_Z1 + CG.FOOT_SOCK_Z     # 23: tube foot on the foot socket floor
CAGE_POST_Z1 = CAGE_POST_Z0 + CAGE_POST_L    # 279
CAGE_ZC = CAGE_POST_Z1 + CG.NODE_BOT         # 290.3: top node centre (the post top on the down socket's floor)
CAGE_TOP = CAGE_ZC + CG.NH                   # 300: top node top face = the top panel's inner face
TOP_RAIL_Z = CAGE_ZC + CG.ARM_DZ             # 290.9: the horizontal sockets sit 0.6 over the node centre
TOP_RAIL_L = 2 * (CAGE_P - CG.NH)            # 178.6: node face to node face (tube ends on the socket floors)
PANEL_Z0 = SHEET_Z1 + CG.EDGE_GAP            # 3.1: side panels' bottom edge, 0.1 over the deck sheet
CAGE_THUMBS = [(sx * RC, sy * (CAGE_P + CG.TONGUE_HOLE[1])) for sx in (1, -1) for sy in (1, -1)]   # (+-140, +-97)
CAGE_SIDES = {"back": (0.0, 1.0), "front": (0.0, -1.0), "right": (1.0, 0.0), "left": (-1.0, 0.0)}
DISC_TOP_Z = TUBE_TOP + TOP_DISC_T
CAGE_PULL_PAIR_KG = 0.8                      # assumed face-to-face pull of one D52 pair across the 0.1 gap (K&J case 1: 1.52 kg to steel)
DISC_FROG_H = 22.4                           # disc frog: top disc top to frog top (0.3 clear + mesh height)
VM = "motion/"


def _vec(*a):
    return np.array(a, float)


def _pol(r, ang):
    return (r * math.cos(math.radians(ang)), r * math.sin(math.radians(ang)))


def _polar(r, angles):
    return [_pol(r, a) for a in angles]


def _rotz(ang):
    c, s_ = math.cos(math.radians(ang)), math.sin(math.radians(ang))
    return _R([(c, s_, 0), (-s_, c, 0), (0, 0, 1)])


def _rz(ang):
    c, s_ = math.cos(math.radians(ang)), math.sin(math.radians(ang))
    return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])


RY_UP = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)   # vendor local +Y (axis) -> world +Z


def _finish_print(s, print_face, outer=None, inner=0.0, foot=0.5, log=None):
    """DL7 finish for a printed part (cadkit.idioms.finish): every remaining convex edge filleted (outer, default the
    design-language radius for the part size, R 2 / R 4 over 40 mm), concave edges at `inner` (0 where a mating part
    sits in the corner), and the elephant-foot chamfer on the print face. Hole rims and print-plane edges are left to
    the chamfer; a failed edge keeps the previous valid solid (finish never raises)."""
    from cadkit.idioms import finish
    from lib.design_constants import fillet_printed
    s = _solid(s)
    bb = s.bounding_box()
    r = fillet_printed(max(bb.size.X, bb.size.Y, bb.size.Z)) if outer is None else outer
    out = finish(s, outer_r=r, inner_r=inner, print_face=print_face, foot=foot, log=log if log is not None else [])
    return _solid(out)


def _holes(s, pts, d, z0=-300.0, h=800.0):
    for x, y in pts:
        s = s - Cylinder(d / 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y, z0)))
    return s



# ------------------------------------------------------------------ frame, brackets, feet (v8)

def _frame(A):
    A.meta_role = {}
    z = (DECK_Z0 + DECK_Z1) / 2
    specs = [("deck_x_back", 2 * HALF, "x", (0, RC, z)), ("deck_x_front", 2 * HALF, "x", (0, -RC, z)),
             ("deck_y_right", 2 * RIN, "y", (RC, 0, z)), ("deck_y_left", 2 * RIN, "y", (-RC, 0, z)),
             ("cross_right", 2 * RIN, "y", (CROSS_X, 0, z)), ("cross_left", 2 * RIN, "y", (-CROSS_X, 0, z))]
    # DL29 / I020 / I021 (2026-10-16): the 2020 is the load-carrying core only, the deck square and the two cross rails
    # that carry the rotor, the drive cartridge and the cage; the base square is gone. I029 (2026-10-21): four legs at
    # the deck corners carry the deck to the plinth floor (the panels and the bottom sheet are skin).
    for sx in (1, -1):
        for sy in (1, -1):
            specs.append((f"leg_{_c(sx, sy)}", LEG_L, "z", (sx * LEG_XY, sy * LEG_XY, (LEG_TOP + LEG_BOT) / 2)))
    names = {}
    for n, (role, L, ax, c) in enumerate(specs, 1):
        s = P.rail(L, ax, c, n=role if role.startswith("leg_") else n)
        A.add(s.label, s, "rail", "frame", length_mm=L, axis=ax, centre=c, role=role, step=0)
        names[role] = s.label
        A.meta_role[s.label] = role
    A.meta_rails = names
    return names


def _cast_bracket(A, *args):
    """V8._bracket with the vendor row read from vendor/hardware/cast_bracket_2020_28/source.json (DL3)."""
    V8._bracket(A, *args)
    m = V.source("hardware/cast_bracket_2020_28")
    A.vendor[-1] = dict(component=args[0], source_url=m.get("url"), author=m.get("author"), licence=m.get("licence"),
                        file=m.get("file"), price_usd=m.get("price_usd"),
                        note="vendor/hardware/cast_bracket_2020_28/source.json (envelope from the listing dimensions)")


def _brackets(A, R_):
    zd = (DECK_Z0 + DECK_Z1) / 2
    for sx in (1, -1):
        for sy in (1, -1):
            c = _c(sx, sy)
            rx = R_[f"deck_x_{'back' if sy > 0 else 'front'}"]
            ry = R_[f"deck_y_{'right' if sx > 0 else 'left'}"]
            _cast_bracket(A, f"v_cast_bracket_2020_deck_corner_{c}", (sx * RIN, sy * RIN, zd), (0, -sy, 0),
                        (-sx, 0, 0), rx, ry, f"j_deck_corner_{c}")
            cross = R_["cross_right" if sx > 0 else "cross_left"]
            rx = R_[f"deck_x_{'back' if sy > 0 else 'front'}"]
            _cast_bracket(A, f"v_cast_bracket_2020_cross_end_{c}", (sx * (CROSS_X + W / 2), sy * RIN, zd), (0, -sy, 0),
                          (sx, 0, 0), rx, cross, f"j_cross_end_{c}")
            # I029: the leg hangs from the deck X rail's underside, one cast bracket on the leg's inboard X face (the key
            # says "upright", not "leg", so only the four rails are leg parts)
            _cast_bracket(A, f"v_cast_bracket_2020_upright_top_{c}", (sx * RIN, sy * LEG_XY, LEG_TOP), (0, 0, -1),
                          (-sx, 0, 0), rx, R_[f"leg_{c}"], f"j_upright_top_{c}")


# ------------------------------------------------------------------ plinth (v8 panels, deck sheet with the S1.4 holes)

def _hardpoint(A, x, y, module):
    name = f"v_deck_hardpoint_{x:g}_{y:g}"
    shape = Cylinder(5,3,align=(Align.CENTER,Align.CENTER,Align.MIN))-Cylinder(2.5,3,align=(Align.CENTER,Align.CENTER,Align.MIN))
    A.add(name,_solid(shape.moved(Location((x,y,0)))),"standoff",module,step=2,
          note="Stock aluminum spacer 5 ID / 10 OD / 3. Direct plate-to-2020 load path; acrylic footprint relieved.")
    A.vendor.append(dict(component=name,source_url="https://www.filastruder.com/products/m5-spacers-3mm-tall-10mm-od",author="Filastruder dimensions",file="assemblies/froggy_v12.py:_hardpoint",licence=None,price_usd=.398))
    return name


def _deck_sheet():
    """Deck sheet, cast acrylic 3 (navy, opaque), 300 x 300 on the deck ring top (S2.5): 66 x 126 centre opening between the cross rail inner
    faces (the S1.26 cartridge's motor plate, motor and spacers pass through it; plate A covers it in X, 3 of it stays
    open at each Y end), grommet hole, S1.4 plate A holes, 4 sheet screws, and the four S3.2 cage-foot screw
    holes 5.5 at (+-140, +-97) (no notches: the foot corners stand on it). The S1.1 spacers pass inside the opening (no separate holes)."""
    p = _box(-HALF, HALF, -HALF, HALF, SHEET_Z0, SHEET_Z1)
    ox, oy = DECK_OPEN
    p = p - _box(-ox / 2, ox / 2, -oy / 2, oy / 2, SHEET_Z0 - 1, SHEET_Z1 + 1)   # S2.5 cartridge opening
    for az in ROLLER_ANG:                                        # S1.27 block nylocs under plate A's arms
        for x, y in _az_pts(RB_HOLES, az):
            p = p - Cylinder(DECK_NUT_HOLE_D / 2, 40).moved(Location((x, y, 0)))
    p = p - Cylinder(G_D / 2, 40).moved(Location((G2[0], G2[1], 0)))
    clamp = [(sx * CT_HOLE, sy * RC) for sx in (1, -1) for sy in (1, -1)] + \
            [(sx * RC, sy * CT_HOLE) for sx in (1, -1) for sy in (1, -1)]           # S2.13 clamp tab bolts
    for x, y in PA_RAIL_HOLES + SHEET_SCREWS + CAGE_THUMBS + clamp:
        p = p - Cylinder(M5_HOLE / 2, 40).moved(Location((x, y, 0)))
    # Offset the actual rounded outer wires: rectangular proxies miss the filled concave fillets.
    from build123d import Face, Kind
    def relief(shape):
        candidates=[f for f,nrm,c in V.planar_faces(shape) if abs(abs(nrm[2])-1)<1e-6]
        face=max(candidates,key=lambda f:f.area)
        outline=Face(face.outer_wire().offset_2d(.5,kind=Kind.ARC))
        outline=outline.moved(Location((0,0,-1-outline.center().Z)))
        return extrude(outline,5)
    p -= relief(_plate_a())
    tab=relief(_clamp_tab_body())
    for angle in (0,90,180,270):
        p -= _az(tab,angle)
    return _solid(p)


def _panels(A, R_):
    A.meta_panels = {}
    for side in V8.SIDES:
        face, rep = _front_profile() if side == "front" else V8._side_profile(side)
        n = np.array(V8.SIDES[side][0], float)
        hv = np.array(V8.SIDES[side][1], float)
        panel = _solid(extrude(V8._plane(side) * face, amount=PANEL_T))
        for h in (-LEG_XY, LEG_XY):                 # I029: 2 x M5 per leg edge into the legs' outer slots
            for z in LEG_PANEL_Z:
                at = n * (V8.PANEL_IN - 1.0) + hv * h + _vec(0, 0, z)
                panel = panel - Plane(origin=tuple(at), z_dir=tuple(n)) * Cylinder(
                    M5_HOLE / 2, PANEL_T + 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        rep = dict(rep, screws=6, mount="skin (I029): hung from the deck rail's outer face (2 x M5 x 8 button head + "
                                        "T-nut) and the two corner legs' outer slots (2 x M5 x 8 + T-nut per leg)")
        A.add(f"s_panel_{side}", _solid(panel), "panel", "plinth", step=7,
              note="3 mm navy cast acrylic sheet, laser cut, skin only (I029): hung from the deck rail's outer face and "
                   "the two corner legs' outer slots; carries no load")
        A.meta_panels[side] = rep
        n = np.array(V8.SIDES[side][0], float)
        hv = np.array(V8.SIDES[side][1], float)
        for z, lvl in zip(V8.PANEL_SCREW_Z, ("deck",)):
            for h in (-V8.PANEL_SCREW_H, V8.PANEL_SCREW_H):
                at = n * V8.PANEL_IN + hv * h + _vec(0, 0, z)
                role = f"{lvl}_{'x' if abs(n[1]) > 0.5 else 'y'}_{side}"
                bolt_joint(A, [f"s_panel_{side}", R_[role]], at, -n, screw="M5x8",
                           name=f"j_panel_{side}_{lvl}_{'p' if h > 0 else 'n'}", module="plinth", contact="faces",
                           step=7, **M5)
        for h in (-LEG_XY, LEG_XY):
            for z, lvl in zip(LEG_PANEL_Z, ("top", "bot")):
                at = n * V8.PANEL_IN + hv * h + _vec(0, 0, z)
                c = _c(1 if at[0] > 0 else -1, 1 if at[1] > 0 else -1)
                bolt_joint(A, [f"s_panel_{side}", R_[f"leg_{c}"]], at, -n, screw="M5x8",
                           name=f"j_panel_{side}_leg_{c}_{lvl}", module="plinth", contact="faces", step=7, **M5)
    A.add("s_panel_deck", _deck_sheet(), "panel", "plinth", step=2,
          note="3 mm navy cast acrylic deck sheet, laser cut, on the deck ring top (Z 0..3); 50 centre hole, S1.4 holes, 4 cage-foot "
               "screw holes (S3.2)")
    for k, (x, y) in enumerate(SHEET_SCREWS, 1):
        role = "deck_x_back" if y > 0 else "deck_x_front"
        bolt_joint(A, ["s_panel_deck", R_[role]], (x, y, 0.0), (0, 0, -1), screw="M5x8", name=f"j_deck_sheet_{k}",
                   module="plinth", contact="faces", step=2, **M5)
    A.add("s_panel_bottom", _bottom_panel(), "panel", "plinth", step=7,
          note="3 mm navy cast acrylic bottom panel, laser cut, 300 x 300 inside the side panels (Z -123..-120): the faux "
               "bench-top floor (I021) that carries the plinth electrics, skin only (I029): clamped under the four leg "
               "ends by the foot bolts")


def _bottom_panel():
    """The v8 bottom panel without the base-rail screws and the foot notches: a full 300 x 300 sheet inside the side
    panels, clamped under the four leg ends by the foot bolts (one M5 x 16 each through the 5.5 hole at (+-140, +-140),
    S2.8, I029)."""
    p = _box(-HALF, HALF, -HALF, HALF, V8.BOTP_Z0, V8.BOTP_Z1)
    p = p - Cylinder(23.0, 20).moved(Location((0, 0, V8.BOTP_Z0)))
    x0, x1, y0, y1, _ = V8.PSU_BOX
    for x, y in [(x, y) for x in (x0 + 8, x1 - 8) for y in (y0 + 8, y1 - 8)]:
        p = p - Cylinder(1.7, 20).moved(Location((x, y, V8.BOTP_Z0)))
    for x, y in fp7_controller_carrier.INSERTS:
        wx, wy, _ = V8._cc_world(x, y)
        p = p - Cylinder(1.7, 20).moved(Location((wx, wy, V8.BOTP_Z0)))
    for x, y in V8._cl42t_holes():
        p = p - Cylinder(2.25, 20).moved(Location((x, y, V8.BOTP_Z0)))
    for sx in (1, -1):
        for sy in (1, -1):
            p = p - Cylinder(M5_HOLE / 2, 20).moved(Location((sx * LEG_XY, sy * LEG_XY, V8.BOTP_Z0)))
    return _solid(p)


def _feet(A, R_):
    """S2.8 feet (I029, DL29): per leg one bought rubber bumper foot 20 x 8 under the bottom panel on the leg axis, its
    5.5 through hole counterbored 10.5 x 5 from below, and one M5 x 16 button head up through the foot (3 of rubber under
    the head) and the bottom panel (3) into the leg's end tap (10 of thread in the 12 deep tap). The bolt clamps the
    sheet and the foot to the leg end: the load path to the table is aluminium and steel, the rubber only in compression;
    the head sits 2.25 inside the counterbore (DL1). Bench order: flip the frame, bottom panel on the leg ends, feet."""
    for sx in (1, -1):
        for sy in (1, -1):
            c = _c(sx, sy)
            fx, fy = sx * LEG_XY, sy * LEG_XY
            z0 = V8.BOTP_Z0 - V8.FOOT_H
            f = Cylinder(V8.FOOT_D / 2, V8.FOOT_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
            f = f - Cylinder(M5_HOLE / 2, 3 * V8.FOOT_H)
            f = f - Cylinder(V8.FOOT_CB_D / 2, V8.FOOT_CB_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
            nm = f"v_foot_bumper_{c}"
            A.add(nm, _solid(f).moved(Location((fx, fy, z0))), "foot", "feet", step=7,
                  note="bought rubber bumper foot 20 x 8 with a 5.5 through hole counterbored 10.5 x 5 (envelope, "
                       "assumed), under the bottom panel on the leg axis")
            A.vendor.append(dict(component=nm, source_url=None, author="catalogue (rubber bumper foot 20 x 8, M5 "
                                 "counterbored)", licence=None, file=None, price_usd=0.5))
            tapped_joint(A, [nm, R_[f"leg_{c}"]], (fx, fy, LEG_BOT), (0, 0, 1), screw=FOOT_SCREW, head="button",
                         seat=(fx, fy, z0 + V8.FOOT_CB_H), stack=[("s_panel_bottom", PANEL_T)],
                         name=f"j_foot_{c}_leg", module="feet", evaluate=False, step=7, tap_depth=LEG_TAP_DEPTH)


def _clamp_tab_body():
    """lp_clamp_tab at the back right deck corner (mirrored for the others): an L of two 22 wide arms along the deck
    rails' top-slot lines, 6061 6 mm (PLATE_T) off the Fablight, through holes only, outside corners R 3, the inside
    corner R 2 (DL8). The open corner square (x, y 124..151, 27 x 27) is the clamp pad: no head stands in it."""
    from build123d import Face, Kind, Polygon
    e, a, w = CT_EDGE, CT_ARM_END, CT_ARM_IN
    f = Polygon((a, w), (w, w), (w, a), (e, a), (e, e), (a, e), align=None)
    wire = f.faces()[0].outer_wire() if hasattr(f, "faces") else f.outer_wire()
    r_in, r_out = LASER["corner_inside"], LASER["corner_outside_min"]
    for d in (r_in, -r_in, -r_out, r_out):          # closing rounds the inside corner, opening the outside ones
        wire = wire.offset_2d(d, kind=Kind.ARC)
    s = _solid(extrude(Face(wire).moved(Location((0, 0, CT_Z0))), CT_T))
    return s


def _clamp_tabs(A, R_):
    """S2.13 quick-clamp points (I023, DL29): four lp_clamp_tab L plates on the deck sheet over the deck rail corners,
    each 2 x M5 x 14 button head down through the tab and the sheet into T-nuts in the two deck rails' top slots (the
    S1.4 stack). An F clamp or wood clamp presses the pad square onto the 2020 core and grips under the bench top; its
    bar stands outside the plinth corner, the jaw passes over the panel top edges (Z 3) and lands on the tab (Z 9)."""
    from build123d import Plane as _P
    for sx in (1, -1):
        for sy in (1, -1):
            c = _c(sx, sy)
            nm = f"lp_clamp_tab_{c}"
            t = _clamp_tab_body()
            if sx < 0:
                t = t.mirror(_P.YZ)
            if sy < 0:
                t = t.mirror(_P.XZ)
            holes = [((sx * CT_HOLE, sy * RC), R_[f"deck_x_{'back' if sy > 0 else 'front'}"]),
                     ((sx * RC, sy * CT_HOLE), R_[f"deck_y_{'right' if sx > 0 else 'left'}"])]
            t = _holes(t, [h for h, _ in holes], M5_HOLE)
            A.add(nm, _solid(t), "plate", "frame", step=2,
                  note=f"laser part, 6061 {CT_T:g} mm Fablight quick-clamp tab (S2.13, I023): L arms along the deck rail "
                       f"slot lines, pad square 27 x 27 at the corner for an F clamp or wood clamp jaw (assumed reach "
                       f"{CLAMP_JAW['reach']:g}, thick {CLAMP_JAW['thick']:g}, pad {CLAMP_JAW['pad']:g})")
            for i, ((x, y), rail) in enumerate(holes, 1):
                hp = _hardpoint(A,x,y,"frame")
                bolt_joint(A, [nm, rail], (x, y, 0.0), (0, 0, -1), screw="M5x14", name=f"j_clamp_tab_{c}_{i}",
                           module="frame", step=2, stack=[(hp, PANEL_T)], **M5)


def _electrics(A):
    """v8 plinth electrics; the printed carrier and legend ring renamed fp9_ (DESIGN-RULES 1).
    fp9_controller_carrier - justification: a tray with Pico standoff bosses and insert pockets on the plinth floor.
    fp9_estop_cradle - justification: in parts/fp9_estop_cradle.py (holds the recessed e-stop station behind the panel).
    The e-stop is the complete BACO LBX10510 station (S2.10) recessed through the front panel's 76 x 76 opening on
    fp9_estop_cradle; the v8 panel e-stop and its printed legend ring are gone."""
    R = _R([(0, 1, 0), (0, 0, 1), (1, 0, 0)])
    A.add("v_cl42t_driver", copy.deepcopy(V7._cl42t_src()).moved(_location(R, np.array([V8.CL42T_X0, 0.0, V8.FLOOR_Z]))),
          "driver", "electrical", step=7,
          includes="STEPPERONLINE CL42T-V41 closed-loop driver (representative of the vendor STEP)")
    A.vendor.append(dict(V.manifest_entry("v_cl42t_driver", "v_cl42t"),
                         note="representative solid derived from the vendor STEP (housing kept, terminals as boxes)"))
    x0, x1, y0, y1, h = V8.PSU_BOX
    psu = _box(x0, x1, y0, y1, V8.FLOOR_Z, V8.FLOOR_Z + h)
    A.meta_psu_holes = [(x, y) for x in (x0 + 8, x1 - 8) for y in (y0 + 8, y1 - 8)]
    for x, y in A.meta_psu_holes:
        psu = psu - Cylinder(1.25, 6.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y, V8.FLOOR_Z)))
    A.add("v_psu_lrs_50_24", _solid(psu), "enclosure", "electrical", step=7,
          note="MEAN WELL LRS-50-24 envelope 99 x 82 x 30, 4 x M3 tapped in the base")
    A.vendor.append(V.manifest_entry("v_psu_lrs_50_24", "v_psu_lrs_50_24"))
    cc = fp7_controller_carrier
    A.add("fp9_controller_carrier", _finish_print(cc.build(), "-Z", outer=2.0).moved(Location((V8.CC_DX, 0, V8.CC_DZ))), "printed", "electrical", step=7,
          note="PETG tray (v7 fp7_controller_carrier geometry); justification: Pico bosses + insert pockets, no stock tray")
    A.add("v_pico", HW.pcb_box(21, 51, label="v_pico").moved(Location(V8._cc_world(*cc.PICO_C, cc.T + cc.BOSS_H))), "pcb",
          "electrical", step=7)
    A.vendor.append(V.manifest_entry("v_pico", "v_pico"))
    A.add("v_buck_module", _solid(Box(43, 21, 14, align=(Align.CENTER, Align.CENTER, Align.MIN))).moved(
        Location(V8._cc_world(*cc.BUCK_C, cc.T))), "pcb", "electrical", step=7)
    A.vendor.append(V.manifest_entry("v_buck_module", "v_buck_module"))
    A.add("v_relay_delay", _solid(Box(50, 26, 18, align=(Align.CENTER, Align.CENTER, Align.MIN))).moved(
        Location(V8._cc_world(*cc.RELAY_C, cc.T))), "pcb", "electrical", step=7)
    A.vendor.append(V.manifest_entry("v_relay_delay", "v_relay_delay"))
    A.add("fp9_estop_cradle", _estop_cradle(), "printed", "electrical", step=7,
          note="PETG open frame behind the front panel: flange on the panel's inner face (4 x M3 x 10 button head from "
               "outside into heat-set inserts), walls hugging the station, back gussets carrying its 2 base slots "
               "(2 x M4 x 12 button head + nyloc), -X cable window at the M16/20 knockout; prints flat on the flange")
    station, st_bodies, screws, sc_bodies = _estop_parts()
    cover = next(c for c in station.children if c.label == f"{ESTOP}_cover")
    A.meta_es_proud = ES_Y0 - cover.bounding_box().min.Y          # cover face outside the panel's outer face
    A.add(ESTOP, station, "switch", "electrical", step=7, bodies=st_bodies,
          includes="BACO LBX10510 IP69K one-hole 22 mm control station, manufacturer STEP: base (2 x 4.5 slots, "
                   "diagonal), yellow cover, 22 mm operator with the 40 mm red mushroom head, 1 NC contact block; "
                   f"recessed through the front panel's {ES_OPEN:g} x {ES_OPEN:g} opening at X {ES_XC:g}, Z {ES_ZC:g}, "
                   f"cover face {ES_PROUD:g} proud, back on fp9_estop_cradle (S2.10). One component: the STEP's own "
                   "bodies overlap (snap fits, threads), so they stay together")
    A.add(ESTOP + "_cover_screws", screws, "switch", "electrical", step=12, bodies=sc_bodies,
          note="BACO LBX station: 4 cover screws and their 4 captive nuts, from the STEP (S2.10)")
    for nm in (ESTOP, ESTOP + "_cover_screws"):
        A.vendor.append(V.vendor_entry(nm, "electrical/baco_lbx10510"))
    x, w, hh, ya, yb = V8.IEC
    fw, fh, ft = V8.IEC_FLANGE
    iec = V8._dev_body(x, w, hh, ya, yb)
    iec = iec + _box(x - fw / 2, x + fw / 2, V8.PANEL_IN - ft, V8.PANEL_IN, V8.PANEL_DEV_Z - fh / 2, V8.PANEL_DEV_Z + fh / 2)
    for sx in (-1, 1):
        iec = iec - Plane(origin=(x + sx * V8.IEC_PITCH / 2, V8.PANEL_IN + 1, V8.PANEL_DEV_Z), z_dir=(0, -1, 0)) * Cylinder(
            1.25, ft + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    A.add("v_iec_inlet", _solid(iec), "switch", "electrical", step=7,
          note="IEC C14 inlet with fuse drawer, rear-mount (representative envelope)")
    A.vendor.append(V.manifest_entry("v_iec_inlet", "v_iec_inlet"))
    x, w, hh, ya, yb = V8.USB
    A.add("v_usb_panel", _solid(V8._dev_body(x, w, hh, ya, yb)), "switch", "electrical", step=7,
          note="USB-C panel-mount extension through the back panel")
    A.vendor.append(V.manifest_entry("v_usb_panel", "v_usb_panel"))
    A.add("v_grommet_g2", V7._grommet(SHEET_Z1, PANEL_T, "v_grommet_g2").moved(Location((G2[0], G2[1], 0))),
          "grommet", "plinth", step=7)
    A.vendor.append(V.manifest_entry("v_grommet_g2", "v_grommet_8mm"))


# ------------------------------------------------------------------ S1 vendor parts (goBILDA STEPs)

@lru_cache(maxsize=None)
def _step_solids(rel):
    return import_step(V.os.path.join(V.VENDOR, rel)).solids()


def _axial_loc(src_axis_pt, z, ang=0.0, src_axis_end=None):
    """Location taking a vendor part whose axis is its local Y to world Z: the local point src_axis_pt (on the axis,
    at the face that lands at world Z z) goes to (0, 0, z); turned `ang` deg about Z."""
    R = _rz(ang) @ RY_UP
    t = np.array([0.0, 0.0, z]) - R @ np.asarray(src_axis_pt, float)
    return _location(R, t)


def _block_parts():
    """goBILDA 1614-0016-4008 STEP: housing (566 faces, 16 mm M4 pattern both ends, four oval slots), two bearings,
    four shields. Axis local Y, ends at Y 8.894 and 24.894."""
    sols = _step_solids(VM + "gobilda_1614_pillow_block/1614-0016-4008.step")
    housing = max(sols, key=lambda s: s.volume)
    hb = housing.bounding_box()
    c = ((hb.min.X + hb.max.X) / 2, hb.min.Y, (hb.min.Z + hb.max.Z) / 2)
    rest = [s for s in sols if s is not housing]
    lower = [s for s in rest if s.bounding_box().max.Y < (hb.min.Y + hb.max.Y) / 2]
    upper = [s for s in rest if s not in lower]
    return housing, lower, upper, c


def _squeeze_y_axis(sol, k, axis_pt):
    """Scale a local-Y-axis part by k across its axis (running clearance on the REX section)."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_GTransform
    from OCP.gp import gp_GTrsf, gp_Mat, gp_XYZ
    g = gp_GTrsf()
    g.SetVectorialPart(gp_Mat(k, 0, 0, 0, 1, 0, 0, 0, k))
    g.SetTranslationPart(gp_XYZ(axis_pt[0] * (1 - k), 0, axis_pt[2] * (1 - k)))
    return Solid(BRepBuilderAPI_GTransform(sol.wrapped, g, True).Shape())


@lru_cache(maxsize=None)
def _shaft_src():
    """goBILDA 2106-4008-0720 STEP: REX shaft. The STEP ships the e-clip installed; only the shaft solid is kept (the
    factory e-clip comes off before assembly: the groove end is down, inside the lower 1522 backup spacer)."""
    sols = _step_solids(VM + "gobilda_2106_rex_shaft_72/2106-4008-0720.step")
    sh = max(sols, key=lambda s: s.volume)
    b = sh.bounding_box()
    c = ((b.min.X + b.max.X) / 2, b.min.Y, (b.min.Z + b.max.Z) / 2)
    return copy.deepcopy(sh), c  # preserve analytic end faces and tapped bores


@lru_cache(maxsize=None)
def _hub_src():
    sols = _step_solids(VM + "gobilda_1313_hyper_hub/1313-1632-4008.step")
    return sols                                             # body + 2 pinch bolts; flange face Z 0, boss bottom Z -20


# ------------------------------------------------------------------ S1 laser parts

def _plate(x, y, z0):
    return _box(-x / 2, x / 2, -y / 2, y / 2, z0, z0 + PLATE_T)


def _motor_plate():
    """6061 6 mm plate, 64 x 120, spacer holes at (+/-21,+1/-45); raised to Z -23..-17.
    ICD-V12 S1.2: motor center (0,30), unrotated 31 mm square mounting pattern.
    Four 3.4 mm wide slots and 24 mm pilot slot run Y -1.5..+.5 about nominal.
    Four spacer holes 5.5 at (+-21,+1/-45), S1.1. Install motor loose on bench,
    loop belt over pulleys, tension, tighten M3 x 10 before cartridge insertion. Spindle clearance hole 24 mm at (0,0), S1.6c-V12.
    I035: a 20 wide through slot along Y (X +-10, Y 0..28.5) joins the spindle hole to the pilot slot, cusps R 2, so
    the belt runs over air (BRepExtrema 3.36 to the plate at C 28.5 / 30.0 / 30.5; was 1.0 to the web).
    """
    p = _box(-MPLATE_X / 2, MPLATE_X / 2, -MPLATE_Y / 2, MPLATE_Y / 2, MPLATE_Z0, MPLATE_Z1)
    p = p - BD.slot(*BD.MOTOR_XY, MOTOR_PILOT_D, MPLATE_Z0-1, MPLATE_Z1+1)
    p = p - BD.cylinder(12, MPLATE_T + 2, MPLATE_Z0 - 1)
    # I035 belt channel: one MPLATE_BELT_SLOT_W through slot along Y joins the 24 spindle hole to the 24 pilot slot,
    # so the belt's straight runs (X +-6.8, bottom Z -16) pass over air, not over the web 1.0 below them
    w = MPLATE_BELT_SLOT_W / 2
    p = p - _box(-w, w, 0.0, BD.MOTOR_XY[1], MPLATE_Z0 - 1, MPLATE_Z1 + 1)
    p = _solid(p)
    yj = math.sqrt((MOTOR_PILOT_D / 2) ** 2 - w ** 2)      # where the slot sides meet the two 24 circles
    cusp = [(sx * w, y) for sx in (1, -1) for y in (yj, BD.MOTOR_XY[1] + BD.TRAVEL[0] - yj)]
    edges = [e for e in p.edges() if abs(e.length - MPLATE_T) < 1e-3
             and any(math.hypot(e.center().X - x, e.center().Y - y) < 0.05 for x, y in cusp)]
    assert len(edges) == 4, f"motor plate belt slot: expected 4 cusp edges, got {len(edges)}"
    p = p.fillet(MPLATE_BELT_SLOT_R, edges)
    p = _holes(p, SPACER_XY, M5_HOLE)
    for x, y in MOTOR_HOLES:
        p = p - BD.slot(x, y, 3.4, MPLATE_Z0-1, MPLATE_Z1+1)
    return _solid(p)


PA_R_OUT = 8.0          # lp_plate_a outline: every convex corner R 8 (arm ends, square corners)
PA_R_IN = 6.0           # every concave corner (arm roots) R 6


def _round_outline(face, r_out, r_in):
    """2D outline rounding for a laser plate: closing by r_in (offset out, then back in) rounds every concave corner to
    r_in, opening by r_out (in, then out) rounds every convex corner to r_out. Arcs and lines only (DXF friendly)."""
    from build123d import Face, Kind
    w = face.outer_wire()
    for d in (r_in, -r_in, -r_out, r_out):
        w = w.offset_2d(d, kind=Kind.ARC)
    return Face(w)


def _plate_a():
    """lp_plate_a, 6061 6.0, 120 x 120 on the deck sheet, the cartridge base plate (S1.26). Fablight cut: a 2D outline
    and through holes only. Outline: the 120 square and three arms 38 wide out to r 100 at 90 / 210 / 330 deg (S1.27),
    every convex corner R 8 (PA_R_OUT) and every concave corner where an arm meets the square R 6 (PA_R_IN). Hole table:
    (0, 0) 16 for the shaft and the 9.95 OD race spacer (fix A, I033), S1.4 and S1.6;
    (+-45, +-40) 5.5 x 4 through the sheet into the cross-rail T-nuts (the only cartridge-to-frame fasteners), S1.4;
    (+-21, +1/-45) 5.5 x 4 spacer bolts M5 x 40 down through the spacers to the motor plate, S1.1;
    (+-8, +-8) 4.3 x 4 pillow block M4 from below, S1.5;
    each arm 4.5 x 2 at block-local (r 76, t +-8): fp12_roller_block M4 x 16 + nyloc under the plate (the nylocs hang
    through 12 holes in the deck sheet), S1.27;
    2 x 2.5 tapped M3 through at (u 67.3, v 0.6 / 7.0) in the 45 deg frame, i.e. (47.2, 48.0) and (42.6, 52.5):
    fp9_hall_bracket, S1.31 (edge margin 7.5).
    Edge margins: rail holes 15 / 20 to the square's edges, arm holes 11 to the arm sides and 24 to the arm end."""
    from build123d import Face, Polygon
    pts_sq = [(-PA_SIZE / 2, -PA_SIZE / 2), (PA_SIZE / 2, -PA_SIZE / 2), (PA_SIZE / 2, PA_SIZE / 2),
              (-PA_SIZE / 2, PA_SIZE / 2)]
    face = Polygon(*pts_sq, align=None).faces()[0]
    for az in ROLLER_ANG:                                        # S1.27: three arms out to the rim rollers
        arm = _az_pts([(40.0, -ARM_HALF_W), (ARM_X1, -ARM_HALF_W), (ARM_X1, ARM_HALF_W), (40.0, ARM_HALF_W)], az)
        face = face + Polygon(*arm, align=None).faces()[0]
    face = max(face.faces(), key=lambda q: q.area)
    face = _round_outline(face, PA_R_OUT, PA_R_IN)
    p = extrude(face.moved(Location((0, 0, PA_Z0))), PLATE_T)
    p = _holes(p, [q for az in ROLLER_ANG for q in _az_pts(RB_HOLES, az)], 4.5)
    p = _holes(p, [(0, 0)], PA_CENTRE_D)
    p = _holes(p, PA_RAIL_HOLES, M5_HOLE)
    p = _holes(p, SPACER_XY, M5_HOLE)
    p = _holes(p, BLOCK_HOLES, BLOCK_HOLE_D)
    p = _holes(p, [_hall_xyz(u, v, 0)[:2] for u, v in HB_HOLES], 2.5)   # S1.31 tapped M3
    return _solid(p)


# BACO LBX10510 STEP solids (file order) -> (body label, material). Indices read off the file 2026-10-12.
ES_STATION_BODIES = [
    (1, "base", "black_plastic"), (0, "cover", "estop_yellow"), (10, "operator_ring", "black_plastic"),
    (11, "operator_flange", "black_plastic"), (12, "operator_body", "black_plastic"),
    (13, "operator_plunger", "black_plastic"), (14, "head_40mm", "estop_red"), (15, "head_insert", "black_plastic"),
    (16, "contact_block", "black_plastic"), (17, "contact_block_inner", "black_plastic"),
    (18, "terminal_screw_1", "steel_zinc"), (19, "terminal_screw_2", "steel_zinc"), (20, "terminal_1", "brass"),
    (21, "terminal_2", "brass"), (22, "terminal_clamp_1", "steel_zinc"), (23, "terminal_clamp_2", "steel_zinc"),
    (24, "contact_plunger", "black_plastic"), (25, "contact_bridge", "estop_red")]
ES_SCREW_BODIES = [(i, f"cover_screw_{i - 1}", "steel_zinc") for i in (2, 3, 4, 5)] + \
                  [(i, f"cover_nut_{i - 5}", "steel_zinc") for i in (6, 7, 8, 9)]


def _estop_loc():
    """STEP -> world for the BACO station: box centre to (ES_XC, ES_ZC), head toward the operator (-Y, 180 deg about
    Z), then 90 deg about Y so the knockouts face +-X (bottom knockout to -X), cover face at ES_FACE_Y."""
    bx, bz = ES_BOX_C_LOCAL
    return (Location((ES_XC, ES_FACE_Y + ES_FACE_LOCAL_Y, ES_ZC)) * Location((0, 0, 0), (0, 90, 0))
            * Location((0, 0, 0), (0, 0, 180)) * Location((-bx, 0, -bz)))


def _estop_parts():
    """BACO LBX10510 control station (S2.10) from the manufacturer's STEP (vendor/electrical/baco_lbx10510), all 26
    solids: base, yellow cover, 22 mm operator with the 40 mm red head, NC contact block and terminals, 4 cover screws
    and their nuts. Placed by _estop_loc: recessed through the front panel's 76 x 76 opening, cover face ES_PROUD
    outside the panel, back pads on fp9_estop_cradle at ES_BACK_Y. Returns (station compound, its bodies, cover screws
    + nuts compound, their bodies). Two components (station, screws) because BACO's own bodies overlap (snap fits,
    threads) inside each group."""
    path = os.path.join(V.ROOT, ES_STEP)
    raw = V.import_step_solids(path)
    assert len(raw) == 26, f"BACO STEP: expected 26 solids, got {len(raw)}"
    loc = _estop_loc()

    def group(rows):
        sols = [raw[i].moved(loc) for i, _, _ in rows]
        for s, (_, lbl, _) in zip(sols, rows):
            s.label = f"{ESTOP}_{lbl}"
        return Compound(children=sols), [(f"{ESTOP}_{lbl}", mat) for _, lbl, mat in rows]
    st, st_b = group(ES_STATION_BODIES)
    sc, sc_b = group(ES_SCREW_BODIES)
    used = sorted(i for i, _, _ in ES_STATION_BODIES + ES_SCREW_BODIES)
    assert used == list(range(26)), "BACO STEP: every solid must be placed exactly once"
    return st, st_b, sc, sc_b


def _estop_cradle():
    """fp9_estop_cradle (parts/fp9_estop_cradle.py) placed: flange on the front panel's inner face round the opening,
    local Y = world -Z, local Z = world +Y (depth into the plinth)."""
    pl = Plane(origin=(ES_XC, ES_PANEL_IN, ES_ZC), x_dir=(1, 0, 0), z_dir=(0, 1, 0))
    return _solid(pl * _finish_print(fp9_estop_cradle.build(), "-Z", outer=2.0))   # DL7 finish, flange face on the bed


def _front_profile():
    """Front panel (S2.6 / S2.10): the v8 front panel outline and 4 screw holes, with the 76 x 76 (R 4) opening for the
    recessed BACO station centred at (X ES_XC, Z ES_ZC) and 4 x 3.4 holes for the M3 x 10 button heads into the
    fp9_estop_cradle inserts. The cradle's lip (its flange edge 0.8 inside the opening, 0.2 off the box) closes the
    1 mm clearance gap from behind; the cable leaves the station through its M16/20 knockout on the -X side."""
    from build123d import RectangleRounded
    e = HALF + PANEL_T
    face = V7._rect(-e, e, V8.BOTP_Z0, SHEET_Z1)
    holes = [V7._circ(h, z, M5_HOLE / 2) for h in (-V8.PANEL_SCREW_H, V8.PANEL_SCREW_H) for z in V8.PANEL_SCREW_Z]
    holes += [V7._circ(x, z, 3.4 / 2) for x, z in ES_CR_SCREWS]           # front: h = +X, so (h, z) = (X, Z)
    holes += [RectangleRounded(ES_OPEN, ES_OPEN, ES_OPEN_R).faces()[0].moved(Location((ES_XC, ES_ZC, 0)))]
    for f in holes:
        face = face - f
    face = max(face.faces(), key=lambda q: q.area)
    rep = dict(side="front", screws=4, mount="face-mounted on the rails' outer faces, 4 x M5 x 8 button head + T-nut",
               estop=dict(part=ESTOP, opening=[ES_OPEN, ES_OPEN], opening_r=ES_OPEN_R, centre=[ES_XC, ES_ZC],
                          cradle="fp9_estop_cradle", cradle_screws=[list(h) for h in ES_CR_SCREWS],
                          cover_proud_mm=ES_PROUD, cable="M16/20 knockout on the station's -X side, cradle window"))
    return V7._clean_face(face), rep


def _az(solid, az):
    """Turn a part built on the +X axis to azimuth `az` (deg, CCW about Z)."""
    return solid.moved(_location(_rotz(az), (0, 0, 0)))


def _az_pts(pts, az):
    c, s_ = math.cos(math.radians(az)), math.sin(math.radians(az))
    return [(x * c - y * s_, x * s_ + y * c) for x, y in pts]


def _xcyl(r, x0, x1, z, y=0.0):
    """Cylinder of radius r along +X from x0 to x1, axis at (y, z)."""
    return Plane(origin=(x0, y, z), z_dir=(1, 0, 0)) * Cylinder(r, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def _roller(az):
    """Official Creality POM wheel, two 625 bearings and internal shim; radial horizontal axle."""
    return [_az(q.moved(Location((ROLLER_R, 0, ROLLER_Z))), az) for q in RW.kit()]


def _axle_spacer(az):
    x0 = ROLLER_R + BRG_W / 2
    return _az(_solid(_xcyl(5, x0, x0 + 1, ROLLER_Z) - _xcyl(2.5, x0 - 1, x0 + 2, ROLLER_Z)), az)


def _axle_hw(az):
    """M5x25 ISO4762 screw, 1 mm precision inner washer, DIN985 M5 nyloc.
    Simplified nominal fastener surfaces; thread helices omitted, socket included.
    Stack: support 4 + outer shim 1 + kit 11 + inner shim 1 + nut 5 = 22.
    """
    from build123d import RegularPolygon
    z, xw = ROLLER_Z, RB_X1
    head = _xcyl(4.25, xw, xw+5, z)
    head -= Plane(origin=(xw+2,0,z), z_dir=(1,0,0))*extrude(RegularPolygon(2/math.cos(math.pi/6),6),amount=4)
    screw = head + _xcyl(2.5, xw-25, xw, z)
    xn = ROLLER_R-BRG_W/2
    washer = _xcyl(5,xn-1,xn,z)-_xcyl(2.5,xn-2,xn+1,z)
    nut = Plane(origin=(xn-6,0,z),z_dir=(1,0,0))*extrude(RegularPolygon(4/math.cos(math.pi/6),6),amount=5)
    nut -= _xcyl(2.5,xn-7,xn,z)
    return [_az(_solid(q),az) for q in (screw,washer,nut)]


RB_R_PLAN = 4.0         # fp12_roller_block: the four outer vertical corners (plan) R 4
RB_R_GUARD = 8.0        # the pinch-guard side walls' top inboard corner (side view) R 8, blending them down to the base
RB_R_TOP = 1.5          # outboard wall top edges
RB_R_EDGE = 1.0         # side-wall top edges and the base's exposed top edges


def _roller_block(az):
    """PETG light-contact rim support and guard, ICD-V12 S1.27.
    Print base flat on bed. Base R67..99.5, width34, thickness4. Wall R95.5..99.5.
    5.4 wide full-round slot center Z32.95 +/-2, M5 axle. Side clearance1mm.
    Two diameter4.5 mounting holes at local (R76,t+/-8), M4x16 plus 5 mm nylocs.
    Justification: integrated axle adjustment and guard for light stabilizing contact only;
    it does not carry the rotor's normal gravity load. Physical print creep unqualified.
    """
    from build123d import Axis, Polygon
    z0, zb = PA_Z1, PA_Z1 + RB_BASE_T
    xg0 = RB_WALL_X0 - 11.5                                      # 84: side walls' inboard end

    def plan(x0, x1, y0, y1, z_lo, z_hi, corners):
        f = Polygon((x0, y0), (x1, y0), (x1, y1), (x0, y1), align=None).faces()[0]
        vs = [v for v in f.vertices() if (round(v.X, 3), round(v.Y, 3)) in corners]
        f = f.fillet_2d(RB_R_PLAN, vs) if vs else f
        return extrude(f.moved(Location((0, 0, z_lo))), z_hi - z_lo)

    yo = RB_HALF_W
    base = plan(RB_X0, RB_X1, -yo, yo, z0, zb, {(RB_X0, -yo), (RB_X0, yo), (RB_X1, -yo), (RB_X1, yo)})
    wall = plan(RB_WALL_X0, RB_X1, -yo, yo, zb - 0.01, RB_WALL_TOP, {(RB_X1, -yo), (RB_X1, yo)})
    b = base + wall
    for sy in (1, -1):                                           # side walls from an X-Z profile, R 8 top inboard
        prof = Polygon((xg0, zb - 0.01), (RB_WALL_X0, zb - 0.01), (RB_WALL_X0, RB_SIDE_TOP), (xg0, RB_SIDE_TOP),
                       align=None).faces()[0]
        prof = prof.fillet_2d(RB_R_GUARD, [v for v in prof.vertices()
                                           if abs(v.X - xg0) < 1e-6 and abs(v.Y - RB_SIDE_TOP) < 1e-6])
        y0, y1 = sorted((sy * RB_SIDE_Y0, sy * RB_HALF_W))
        side = extrude(Plane.XZ.from_local_coords(prof) if False else
                       prof.moved(Location((0, y1, 0), (90, 0, 0))), y1 - y0)
        b = b + side
    b = _solid(b).clean()
    top = [e for e in b.edges() if abs(e.bounding_box().min.Z - RB_WALL_TOP) < 1e-3
           and abs(e.bounding_box().max.Z - RB_WALL_TOP) < 1e-3]
    b = b.fillet(RB_R_TOP, top)

    def on_side_wall(e):            # edge of a side wall's top or inboard end (faces t +-12.5 / +-17, r < 95.5)
        bb = e.bounding_box()
        return (bb.max.X < RB_WALL_X0 + 1e-3 and bb.min.X > xg0 - 1e-3 and bb.max.Z > zb + 1e-3
                and min(abs(abs(bb.min.Y) - RB_SIDE_Y0), abs(abs(bb.min.Y) - RB_HALF_W)) < 1e-3
                and abs(bb.max.Y - bb.min.Y) < 1e-3
                and not (abs(bb.min.X - RB_WALL_X0) < 1e-3 and abs(bb.max.X - RB_WALL_X0) < 1e-3))

    def on_base_top(e):             # exposed top edges of the base (Z zb), inboard of the side walls
        bb = e.bounding_box()
        return abs(bb.min.Z - zb) < 1e-3 and abs(bb.max.Z - zb) < 1e-3 and bb.max.X < xg0 + 1e-3

    edges = [e for e in b.edges() if on_side_wall(e) or on_base_top(e)]
    b = b.fillet(RB_R_EDGE, edges)
    corner = [e for e in b.edges() if abs(e.center().X - RB_WALL_X0) < 1e-3 and abs(abs(e.center().Y) - RB_HALF_W) < 1e-3
              and e.center().Z > RB_SIDE_TOP]       # DL7: the outboard wall's inboard corners above the side walls (last: they end in the R 1 side-wall blends)
    for rr in (1.0, 0.75, -1.0):
        try:
            b = (b.fillet(rr, corner) if rr > 0 else b.chamfer(1.0, None, corner)) if corner else b
            break
        except ValueError:
            continue
    w = RB_SLOT_W / 2
    b = b - _box(RB_WALL_X0 - 1, RB_X1 + 1, -w, w, ROLLER_Z - RB_SLOT_HALF, ROLLER_Z + RB_SLOT_HALF)
    for dz in (-RB_SLOT_HALF, RB_SLOT_HALF):
        b = b - _xcyl(w, RB_WALL_X0 - 1, RB_X1 + 1, ROLLER_Z + dz)
    b = _holes(b, RB_HOLES, 4.5)
    b = _finish_print(b, "-Z", outer=2.0, inner=0.0)     # DL7: slot rims, wall corners R 2 (R 1 where 4 thick), bed chamfer
    return _az(_solid(b), az)


def _carrier_disc():
    """lp_carrier_disc, 6061 6.0, 186 dia, no cutouts other than this hole table (centre, dia, ICD row):
    (0, 0) 14.3 hub pilot boss, S1.8; r 22.63 at 75/165/255/345 deg 4.5 + 90 deg csk x 4, hub M4, S1.8;
    r 83 at 0 + 60k deg 3.3 x 6 tapped M4 through: ring B + bottom seat, S1.9 (screw tips stop 0.4 short of the
    underside: nothing below the disc); r 66 at 51.25 / 231.25 deg 4.2 x 2 tapped M5 through: drum post studs, S1.32 (stud
    tips 0.4 short of the underside);
    (4.0 +- 34, -3.1 +- 27.2) 3.4 x 4 camera mount (M3 x 10 button from below), S1.16; (+-29, 48 +- 11.5) 2.7 x 4 Pi standoffs, S1.18;
    (-48, 40) 12.2 switch bushing, S1.21; (-52, -36) 12.4 charge port, S1.22 (both inside the tube, S1.28);
    (the S1.23 LED driver holes are gone, I034); r 80 at 45/135/225/315 deg 4.2 x 4 tapped M5 trim, S1.24;
    r 76 at 0 / 180 deg 6.1 x 2 through: index magnets pressed and glued in from below, flush with the underside,
    capped on top by ring B, S1.30. (The 12 S1.14 LED holes are gone, 2026-10-12.)
    The underside band r 86..93 is the S1.27 rim-roller track (S1.28): nothing protrudes there."""
    t, z0 = DISC_T, DISC_Z0
    from build123d import Cone
    p = Cylinder(DISC_D / 2, t, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, z0)))
    p = _holes(p, [(0, 0)], HUB_PILOT_D + 0.3)
    k, r_edge, r_hole = 1.0, 4.19, 2.25  # ICD-V12: 90 degree,8.96 maximum top diameter,2.23 depth
    zc = DISC_Z1 - 0.29 - (r_edge - r_hole) / k
    r_top = r_edge + k * (0.29 + 1.0)
    for x, y in _polar(HUB_R32, HUB_HOLE_ANG):
        p = p - Cylinder(r_hole, 40).moved(Location((x, y, z0)))
        p = p - Cone(r_hole, r_top, DISC_Z1 + 1.0 - zc, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((x, y, zc)))
    p = _holes(p, _polar(STACK_R, STACK_ANG), STACK_TAP_D)
    p = _holes(p, BR_SCREWS, 3.4)
    p = _holes(p, PI_HOLES, 2.7)
    p = _holes(p, [PORT_XY], PORT_HOLE_D)
    p = _holes(p, [SWITCH_XY], SW_HOLE_D)
    p = _holes(p, _polar(TRIM_R, TRIM_ANG), TRIM_TAP_D)
    p = _holes(p, _polar(MAG_R, MAG_ANG), MAG_HOLE_D)
    p = _holes(p, _polar(POST_R, POST_ANG), M5_TAP_D)         # S1.32 tapped M5 for the post studs
    return _solid(p)


def _lobed_ring(od, id_, z0, label):
    """Flat Fablight ring (3.175 6061) with a round lobe of r POST_LOBE_R inward round each S1.32 post hole (posts at
    r POST_R inside the tube, I014). The lobe sides run radially, tangent to the lobe circle, out into the ring; the
    two inside corners where they meet the bore are R 2 (DL8 corner_inside). Through-cut only."""
    s = HW.ring(od, id_, RING_T, label).moved(Location((0, 0, z0)))
    for ang in POST_ANG:
        x, y = _pol(POST_R, ang)
        lobe = Cylinder(POST_LOBE_R, RING_T, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y, z0)))
        L = id_ / 2 - POST_R + 4.0                              # tongue from the hole centre out past the bore
        tongue = Box(L, 2 * POST_LOBE_R, RING_T, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
            Location((x, y, z0), (0, 0, ang)))
        s = s + lobe + tongue
    s = _solid(s).clean()
    rb = id_ / 2
    corner = [e for e in s.edges() if e.geom_type == GeomType.LINE and abs(e.bounding_box().size.Z - RING_T) < 1e-4
              and abs(math.hypot(e.bounding_box().center().X, e.bounding_box().center().Y) - rb) < 0.05]
    s = _solid(s.fillet(LASER["corner_inside"], corner))
    return s


def _ring_b():
    """lp_ring_b, 6061 3.175 (Fablight, through holes only), ID 144 OD 186 (I036: the 150 x 2.5 tube foot r 72.5..75 fully on it), flat on the disc top under the bottom
    tube seat. Hole table: r 83 at 0 + 60k deg 4.5 x 6, the M4 x 10 BHCS from the seat counterbores into the disc
    threads, S1.9; r 66 at the POST_ANG pair 5.5 x 2 in two R 11 lobes inside the bore, the drum post studs, S1.32. Margins 11 to the ID, 10 to the OD (rule 6). Covers the two magnet holes (S1.30) and the trim
    holes from above."""
    s = _lobed_ring(RING_B[1], RING_B[0], DRUM_FLOOR, "lp_ring_b")
    s = _holes(s, _polar(STACK_R, STACK_ANG), STACK_HOLE_D)
    s = _holes(s, _polar(POST_R, POST_ANG), M5_HOLE)            # S1.32 post studs, in the two lobes
    s = _solid(s)
    s.label = "lp_ring_b"
    return s


def _ring_top():
    """lp_ring_top, 6061 3.175 (Fablight, through holes only), bezel ID 147 OD 186 on the top disc. Hole table:
    r 83 at 0 + 60k deg 4.5 x 6, M4 x 18 button head down through the bezel, the top disc and the top seat into hex
    nuts under the seat, S1.13; r 66 at the POST_ANG pair 5.5 x 2 in two R 11 lobes, M5 x 14 button into the drum posts, S1.32. Button heads, not countersunk: no countersinks in 3 mm sheet (rule 4) and the
    Fablight cuts through holes only. Margins 9.5 to the ID, 10 to the OD (rule 6)."""
    z0 = TUBE_TOP + TOP_DISC_T
    s = _lobed_ring(RING_TOP[1], RING_TOP[0], z0, "lp_ring_top")
    s = _holes(s, _polar(STACK_R, STACK_ANG), STACK_HOLE_D)
    s = _holes(s, _polar(POST_R, POST_ANG), M5_HOLE)            # S1.32 post screws, in the two lobes
    s = _solid(s)
    s.label = "lp_ring_top"
    return s


def _top_disc():
    """s_top_disc, cast acrylic 3.2, 186 OD (laser cut), on the tube rim and the top seat; 6 x 4.5 at r 83 for the
    S1.13 clamp screws and 2 x 5.5 at r 66 (POST_ANG) for the S1.32 post screws. The frog stands inside the bezel ID 147."""
    s = Cylinder(TOP_DISC_D / 2, TOP_DISC_T, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, TUBE_TOP)))
    s = _holes(s, _polar(STACK_R, STACK_ANG), STACK_HOLE_D)
    s = _holes(s, _polar(POST_R, POST_ANG), M5_HOLE)
    s = _solid(s)
    s.label = "s_top_disc"
    return s


def _revolve_profile(pts, fillets=()):
    """Solid of revolution about Z from a closed (r, z) polyline; fillets: [((r, z), radius), ...] rounds those
    profile corners (a revolved fillet on the ring's circular edge)."""
    from build123d import Axis, Polyline, make_face, revolve
    f = make_face(Polyline(*[(r, 0.0, z) for r, z in pts], close=True)).faces()[0]
    for (r, z), rad in fillets:
        vs = [v for v in f.vertices() if abs(v.X - r) < 1e-6 and abs(v.Z - z) < 1e-6]
        f = f.fillet_2d(rad, vs)
    return _solid(revolve(f, Axis.Z, 360))


def _tube_seat_bottom():
    """fp9_tube_seat_bottom (PETG). Justification: the socket that locates the tube foot and holds its silicone seal
    cannot be cut in flat Fablight stock (no machining, D1) and no vendor part fits a 150 tube; it carries no
    rotor load (the tube stands on ring B, the drum posts carry the top).
    Print orientation: flat on its outer (bottom) face, the face on ring B, counterbores open upward, no supports.
    Ring r 75.3..90 x 6 on ring B (Z 54.175..60.175, the wall band floor); its bore 150.6 (0.3 radial) is the 6 deep
    socket round the tube foot, silicone bead in the bore. (The 2 mm channel's inner wall is dropped so the tube foot
    stands on ring B and the posts are a stock 100 length.) Top outer and bore edges R 2 (the bore's is the lead-in).
    Hole table: r 83 at 0 + 60k deg: 4.5 through + 8.0 counterbore from the top to a 1.2 floor (M4 x 10 BHCS head 2.6
    below the top), S1.9. (The S1.32 drum posts stand inside the tube since 2026-10-13, I014: no post holes here.)
    Install: step 4, after ring B, the six M4 x 10 through it and ring B into the disc."""
    z0 = SEAT_B_Z0
    s = _revolve_profile([(SEAT_CH_R[1], z0), (SEAT_B_OUT_R, z0), (SEAT_B_OUT_R, SEAT_B_Z1), (SEAT_CH_R[1], SEAT_B_Z1)],
                         [((SEAT_B_OUT_R, SEAT_B_Z1), SEAT_FILLET), ((SEAT_CH_R[1], SEAT_B_Z1), SEAT_FILLET)])
    pts = _polar(STACK_R, STACK_ANG)
    s = _holes(s, pts, STACK_HOLE_D)
    for x, y in pts:
        s = s - Cylinder(SEAT_CB_D / 2, 20, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((x, y, z0 + SEAT_CB_FLOOR)))
    return _finish_print(s, "-Z", outer=2.0)              # DL7: bed-face chamfer on the ring B face


def _tube_seat_top():
    """fp9_tube_seat_top (PETG). Justification: the bore that captures and bonds the tube top cannot be cut in flat
    Fablight stock (no machining, D1); it carries only the top disc's clamp, no rotor load.
    Print orientation: flat on its outer (top) face, the face under the top disc, no supports (a plain ring).
    Ring r 75.3..93 x 6 under the top disc (Z 188.175..194.175); its bore (150.6, 0.3 radial to the tube) captures the
    tube's top 6 mm, silicone bead in the bore; the tube rim is flush with the seat top so the disc bears on both.
    No inner channel wall: anything inside r 72.5 under the disc hides the top disc edge from the camera (camsim).
    Bottom outer and bore edges R 2 (the bore's is the tube lead-in). Hole table: r 83 at 30 + 60k deg 4.5 x 6, M4 x 18
    button + hex nut on the seat underside (outside the tube, inside r 86), S1.13 (the S1.32 posts stand inside the
    tube since 2026-10-13, I014: no post holes here). Install: step 6, slid over the
    tube top before the top disc."""
    from build123d import fillet
    z0, z1 = SEAT_T_Z0, TUBE_TOP
    s = _revolve_profile([(SEAT_CH_R[1], z0), (TOP_DISC_D / 2, z0), (TOP_DISC_D / 2, z1), (SEAT_CH_R[1], z1)],
                         [((TOP_DISC_D / 2, z0), SEAT_FILLET), ((SEAT_CH_R[1], z0), SEAT_FILLET)])
    s = _holes(s, _polar(STACK_R, STACK_ANG), STACK_HOLE_D)
    return _finish_print(s, "+Z", outer=2.0)              # DL7: bed-face chamfer on the top (print) face


def _drum_post():
    """M5 female-female round aluminium standoff POST_D OD x POST_L (vendor POST_VENDOR, listing dims), M5
    tapped 12 deep from each end (modelled at the 4.2 tap drill). Local: bottom face z 0, axis +Z."""
    s = Cylinder(POST_D / 2, POST_L, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for z0 in (-0.01, POST_L - POST_TAP_DEPTH):
        s = s - Cylinder(M5_TAP_D / 2, POST_TAP_DEPTH + 0.01, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((0, 0, z0)))
    return _solid(s)


def _magnet():
    """6 x 3 N42 disc magnet (vendor/hardware/magnet_6x3_n42, Total Element listing), axial, faces z 0 and 3."""
    return _solid(Cylinder(MAG_D / 2, MAG_T, align=(Align.CENTER, Align.CENTER, Align.MIN)))


def _led_ring():
    """WS2812B 16 LED ring from the Adafruit NeoPixel Ring 16 vendor STEP (vendor/electronics/ws2812_ring_16, I032):
    all 17 solids, board 44.45 OD x 31.75 hole x 1.6 then 16 x 5050 on r 19. Local (as in the STEP): board underside
    z 0, LEDs to +z. Returns (compound, bodies)."""
    raw = V.import_step_solids(os.path.join(V.ROOT, LEDR_STEP))
    assert len(raw) == 1 + LEDR_N, f"NeoPixel Ring 16 STEP: expected {1 + LEDR_N} solids, got {len(raw)}"
    names = ["v_led_ring_ws2812_board"] + [f"v_led_ring_ws2812_led_{i}" for i in range(1, LEDR_N + 1)]
    for sol, n in zip(raw, names):
        sol.label = n
    return Compound(children=list(raw)), [(names[0], "pcb_black")] + [(n, "smd_led_white") for n in names[1:]]


# KY-003 class A3144 hall breakout (Joy-it SEN-KY003HMS; listing 15 x 29 x 7 with the header): board 15 x 18.5 x 1.6,
# a 3.2 hole 4.5 from the board's inboard edge and 3 above its bottom edge, the A3144 (TO-92S 4.1 x 3.0 x 1.52) on its
# leads at the top edge, bent so the branded (sensing) face looks up; the 3 pads wired direct (header not fitted).
KY_W, KY_H, KY_T = 15.0, 18.5, 1.6
KY_HOLE = (4.5, 3.0)
TO92 = (4.1, 3.0, 1.52)


def _hall_frame():
    """(origin, u radial, v tangential) of the hall bracket frame at HALL_AZ."""
    c, s_ = math.cos(math.radians(HALL_AZ)), math.sin(math.radians(HALL_AZ))
    return np.array([c, s_, 0.0]), np.array([-s_, c, 0.0])


def _hall_xyz(u, v, z):
    ur, vt = _hall_frame()
    return tuple(ur * u + vt * v + np.array([0, 0, z]))


HB_BASE = (63.0, 79.6, -4.0, 10.0)           # fp9_hall_bracket base footprint (u0, u1, v0, v1) on plate A, R 3 plan corners
HB_BASE_T = 4.0
HB_COL = (71.0, 79.6, KY_T, 10.0)             # column; the board's face sits on its -v face (v = KY_T)
HB_COL_TOP = 39.0
HB_R_PLAN, HB_R_COL = 3.0, 2.0                # base plan corners, column vertical corners (DL7: outside R >= 2)
HB_HEAD_CLR_R = 5.5                           # scallop round the S1.4 M5 button head (ISO 7380 dk 9.5 + 0.75)
HB_R_BASE_TOP = 1.5                           # base top outer edges (4 thick base: R 1.5 + the 0.5 bed chamfer)
HB_R_LEDGE = 0.5                              # the 1.6 deep ledge's edges (the board's bottom edge sits on it)
HB_INSERT_D, HB_INSERT_L = 4.1, 6.0           # M3 heat-set pocket (cadkit table: M3 short insert 4.0 OD x 5.7)
HB_HOLES = [(67.3, 0.6), (67.3, 7.0)]         # 2 x M3 x 10 button into M3 threads tapped in plate A
SENSOR_U = MAG_R                              # the A3144 centre right under the magnet circle
KY_U0 = SENSOR_U - 5.0                        # board inboard edge (u 71), its top edge at the sensor
KY_Z1 = DISC_Z0 - HALL_GAP - TO92[2] - 0.48   # board top edge 41.0 (leads 0.48 to the TO-92 underside)
KY_Z0 = KY_Z1 - KY_H


def _convex_vertical(solid, e):
    """True when the vertical edge e of solid is an outside (convex) corner: a point just outside the corner
    along the bisector of its two faces' outward normals is outside the solid."""
    fs = [f for f in solid.faces() if any(e.is_same(x) for x in f.edges())]
    if len(fs) != 2:
        return False
    c = e.center()
    n = [f.normal_at(f.center()) for f in fs]
    d = (n[0] + n[1])
    if d.length < 1e-6:
        return False
    d = d.normalized()
    p = c + d * 0.3
    return not solid.is_inside(p)


def _hall_bracket():
    """fp9_hall_bracket (PETG). Justification: holds a 15 x 18.5 breakout on edge 2 mm under the disc at r 76 with
    a ledge against rotation and an M3 insert; a flat sheet bracket would need a bend and a tapped boss.
    Reference: the printed sensor stands of open-source encoder mounts (one base plate with two screws, one upright
    column, a support ledge): a rounded-plate base, a rounded column, every edge blended.
    Print orientation: flat on its base (the plate A face) on the bed, column up, no supports: the ledge under the
    board has a 45 deg underside; bed edges carry a 0.5 elephant-foot chamfer.
    Frame: u radial at 45 deg, v tangential. Base u 63..79.6, v -4..10, 4 thick, plan corners R 3, top edges R 1.5
    (rebuilt 2026-10-12, DL7: the old base put the M3 holes 0.6 from its edges and the insert pocket broke out of the
    column; a R 5.5 scallop clears plate A's S1.4 M5 button head at (45, 40)); column u 71..79.6, v 1.6..10 up to Z 39 (2.2 under the lowest swept rotor part there), flush with the base
    on its +u and +v faces, vertical corners R 2, top edges R 2; ledge v 0..1.6 at Z 20.5..22.5 under the board's bottom edge with a 45 deg underside (with the screw it stops
    the board turning). Column-to-base junction R 1 inside. Every other outside edge R 2 via cadkit finish().
    Hole table (u, v): (67.3, 0.6) and (67.3, 7.0) 3.4 x 2, M3 x 10 button into plate A (tapped M3), S1.31; column face
    (u 75.5, Z 25.5) M3 heat-set insert pocket 4.1 x 6 along +v (2.4 wall behind), M3 x 6 button through the board,
    S1.31. Install: on plate A on the bench (step 2, with the roller blocks), board on it before the disc goes on."""
    from build123d import Polygon
    ur, vt = _hall_frame()
    loc = _location(np.column_stack([ur, vt, [0, 0, 1]]), (0, 0, 0))

    u0, u1, v0, v1 = HB_BASE
    zb = PA_Z1 + HB_BASE_T
    cu0, cu1, cv0, cv1 = HB_COL
    # base and column flush on their +u and +v faces (one face each, no ledge round the column's back), then the
    # vertical corners rounded: R 3 on the shared back corners and the base's front corners, R 2 on the column's
    b = _solid(_box(u0, u1, v0, v1, PA_Z1, zb) + _box(cu0, cu1, cv0, cv1, zb - 0.01, HB_COL_TOP)).clean()
    ur_, vt_ = _hall_frame()                         # scallop round plate A's S1.4 M5 button head at (45, 40)
    hx, hy = next(p for p in PA_RAIL_HOLES if p[0] > 0 and p[1] > 0)
    hu, hv = hx * ur_[0] + hy * ur_[1], hx * vt_[0] + hy * vt_[1]
    b = _solid(b - Cylinder(HB_HEAD_CLR_R, 40).moved(Location((hu, hv, PA_Z1))))
    for rr in (HB_R_PLAN, HB_R_COL):
        vert = [e for e in b.edges() if e.geom_type.name == "LINE" and abs(e.tangent_at(0).Z) > 0.99
                and e.length > 1.0 and (rr == HB_R_COL or e.bounding_box().min.Z < PA_Z1 + 1e-3)
                and (rr == HB_R_PLAN or e.bounding_box().min.Z > PA_Z1 + 1e-3)]
        vert = [e for e in vert if _convex_vertical(b, e)]
        try:
            b = b.fillet(rr, vert)
        except ValueError:
            pass
    # ledge: v 0..cv0, top at the board's bottom edge, 45 deg underside, on the column face's flat between its R 2 corners
    prof = Polygon((0.0, KY_Z0), (cv0 + 0.01, KY_Z0), (cv0 + 0.01, KY_Z0 - 2.0 - cv0), (0.0, KY_Z0 - 2.0), align=None)
    led = extrude(Plane(origin=(cu0 + HB_R_COL + 0.5, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)) * prof.faces()[0],
                  cu1 - cu0 - 2 * HB_R_COL - 1.0,
                  dir=(1, 0, 0))
    b = _solid(b + led).clean()

    def flat_at(e, z):
        bb = e.bounding_box()
        return abs(bb.min.Z - z) < 1e-3 and abs(bb.max.Z - z) < 1e-3

    def in_col(e):
        return cu0 - 1e-3 <= e.center().X <= cu1 + 1e-3 and cv0 - 0.2 <= e.center().Y <= cv1 + 1e-3
    base_top = [e for e in b.edges() if flat_at(e, zb) and not in_col(e)]
    b = b.fillet(HB_R_BASE_TOP, base_top)
    top = [e for e in b.edges() if abs(e.bounding_box().min.Z - HB_COL_TOP) < 1e-3
           and abs(e.bounding_box().max.Z - HB_COL_TOP) < 1e-3]
    b = b.fillet(HB_R_COL, top)
    try:
        b = b.fillet(PRINTED["fillet_inside"], [e for e in b.edges() if flat_at(e, zb) and in_col(e)])
    except ValueError:
        pass
    ledge = [e for e in b.edges() if e.center().Y < cv0 - 1e-3 and KY_Z0 - 2.0 - cv0 - 0.1 < e.center().Z < KY_Z0 + 0.1
             and e.center().Z > zb + 1]
    for rr in (HB_R_LEDGE, HB_R_LEDGE / 2):
        try:
            b = b.fillet(rr, ledge)
            break
        except ValueError:
            continue
    b = _finish_print(b, "-Z", outer=HB_R_COL, inner=0.0)     # base top edges R 2, ledge edges, bed chamfer
    for u, v in HB_HOLES:
        b = b - Cylinder(1.7, 40).moved(Location((u, v, 0)))
    hu, hz = KY_U0 + KY_HOLE[0], KY_Z0 + KY_HOLE[1]
    b = b - (Plane(origin=(hu, cv0 - 0.01, hz), x_dir=(1, 0, 0), z_dir=(0, 1, 0)) *
             Cylinder(HB_INSERT_D / 2, HB_INSERT_L, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    return _solid(b).moved(loc)


def _hall_board():
    """KY-003 board + the A3144 in the hall frame. Returns [pcb, sensor (TO-92 + leads)]."""
    ur, vt = _hall_frame()
    loc = _location(np.column_stack([ur, vt, [0, 0, 1]]), (0, 0, 0))
    pcb = _box(KY_U0, KY_U0 + KY_W, 0.0, KY_T, KY_Z0, KY_Z1)
    hu, hz = KY_U0 + KY_HOLE[0], KY_Z0 + KY_HOLE[1]
    pcb = pcb - (Plane(origin=(hu, -1, hz), x_dir=(1, 0, 0), z_dir=(0, 1, 0)) *
                 Cylinder(1.6, 5, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    tw, tl, tt = TO92
    z0 = KY_Z1 + 0.48
    body = _box(SENSOR_U - tl / 2, SENSOR_U + tl / 2, KY_T / 2 - tw / 2, KY_T / 2 + tw / 2, z0, z0 + tt)
    leads = _box(SENSOR_U - 0.5, SENSOR_U + 0.5, KY_T / 2 - 1.5, KY_T / 2 + 1.5, KY_Z1 - 0.01, z0 + 0.01)
    return [_solid(pcb).moved(loc), _solid(body + leads).moved(loc)]


# ------------------------------------------------------------------ S1 printed parts

def _cam_frame():
    a = np.array(OPTICAL_AXIS, float)
    a /= np.linalg.norm(a)
    t = np.array([a[2], 0.0, -a[0]])
    p0 = np.array(LENS) - a * CAM_STACK
    return a, t, p0


BR_LUG_R = 5.7                              # corner lugs round the four S1.16 screws: 7.4 counterbore + 2 wall
BR_R_SADDLE = 4.0                           # saddle top corners (YZ section) and the X-end plan corners
BR_R_COLLAR = 4.0                           # collar (47 square) corner radius
BR_X_NEG = 44.0                             # saddle -X end 44 from the bank centre (the +X end stays 40): the 30 deg
#                                             collar's foot reaches x -39.6 at the top plate, so the plate runs past it


def _camera_bridge():
    """fp9_camera_wedge_bridge (PETG), the camera mount: the only printed rotor part, one body. Justification: one
    saddle that straddles the power bank lying flat on the axis, holds it down (top plate + two snap lips over its long
    top edges) and carries the 30 deg camera seat for the 36 x 36 board with four M2 inserts; no sheet or vendor part
    does both.
    Reference: printed camera mounts with a rounded saddle (GoPro-style cradles, Arducam case collars): one rounded
    U-section, round lugs at the screws, a rounded collar that follows the board; every edge blended (DL7).
    I028 (2026-10-21, "not sure how it got so high? looks like it's in a weird spot"): the I018 riser that stood the
    collar 40 above the top plate is gone; the collar seats straight on the top plate as a wedge (lens 48 above the
    disc, camsim at the 140 tube: top disc 99 %, disc frog 65 %, wall frog 63 %, 6.8 / 9.7 px/mm), its -X foot cut
    flush with the saddle's -X end so nothing leans past the saddle, and the four screws come up from under the disc
    into heat-set inserts, so no driver bore cuts the wedge or the lug tops.
    Form: the saddle is one inverted-U section (cheeks |y| 22.9..31.5 about the bank centre line, top plate Z 74.4..77.4,
    the snap lips 1.2 x 1.5 under the plate with an R 0.6 nose) with R 4 top corners, extruded x -44..40 about the bank
    centre with R 4 plan corners; four round lugs R 5.7 at the screws (clipped at the cheeks' inner faces, so the bank
    space is untouched) up to Z 72.9, solid, with a 4.1 x 6 insert pocket from the bed face; the collar (57 square,
    R 4 corners) runs along the optical axis from the top plate to 7.8 in front of the board and is the camera seat
    itself. Every outside edge R 2 (R 1 on the collar rim) by cadkit finish().
    Print orientation: flat on the cheek and lug feet (the disc face, -Z) on the bed, collar up; the top plate spans
    the bank as a 45 bridge, the collar's 30 deg lean is self-supporting; 0.5 bed chamfer.
    Kept exactly: the lens on the axis 48 above the disc (0, 0, 99), the 30 deg tilt, the board plane and its four M2
    insert pockets on 28.5, the 52.4 ring-light recess 4.3 in front of the board, the 25 ribbon slot, the snap lips.
    Hole pattern (S1.16): (+-34, +-27.2) about the bank centre (4.0, -3.1), M3 heat-set inserts 4.1 x 6 from the disc
    face (M3 x 10 button up through the 3.4 disc holes)."""
    from build123d import Polygon, RectangleRounded
    a, t, p0 = _cam_frame()
    z0 = DRUM_FLOOR
    yi, yo = BR_CHEEK_Y
    ox, oy = BANK_OFF
    zl = BR_TOP_Z0 - 1.5
    # saddle: inverted-U section with the snap lips (local x = Y about the bank centre, local y = Z), along +X
    sec = Polygon((-yo, z0), (-yi, z0), (-yi, zl), (-yi + BR_LIP, zl), (-yi + BR_LIP, BR_TOP_Z0),
                  (yi - BR_LIP, BR_TOP_Z0), (yi - BR_LIP, zl), (yi, zl), (yi, z0), (yo, z0), (yo, BR_TOP_Z1),
                  (-yo, BR_TOP_Z1), align=None).faces()[0]
    sec = sec.fillet_2d(BR_R_SADDLE, [q for q in sec.vertices() if abs(q.Y - BR_TOP_Z1) < 1e-6])
    sec = sec.fillet_2d(0.6, [q for q in sec.vertices() if abs(q.Y - zl) < 1e-6 and abs(abs(q.X) - (yi - BR_LIP)) < 1e-6])
    x0, x1 = ox - BR_X_NEG, ox + BR_X
    sad = extrude(Plane(origin=(x0, oy, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)) * sec, x1 - x0, dir=(1, 0, 0))
    plan = extrude(RectangleRounded(x1 - x0, 2 * yo, BR_R_SADDLE).moved(Location(((x0 + x1) / 2, oy, z0 - 1))),
                   BR_TOP_Z1 - z0 + 2)
    s = _solid(sad & plan)
    ends = [e for e in s.edges() if min(abs(e.bounding_box().min.X - x0), abs(e.bounding_box().max.X - x1)) < 1e-3
            and abs(e.bounding_box().max.X - e.bounding_box().min.X) < 1e-3 and e.bounding_box().max.Z > z0 + 1e-3]
    for rr in (PRINTED["fillet_outside_min"], 1.5, 1.0):        # the X-end faces' outline (top, underside, lips)
        try:
            s = _solid(s.fillet(rr, ends))
            break
        except ValueError:
            continue
    s = _finish_print(s, "-Z", outer=PRINTED["fillet_outside_min"])
    lug_top = BR_TOP_Z1 - BR_R_SADDLE - 0.5
    for x, y in BR_SCREWS:                                     # round lugs at the four screws, outside the bank space
        sy = 1 if y > oy else -1
        lug = Cylinder(BR_LUG_R, lug_top - z0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y, z0)))
        y_in = oy + sy * yi
        lug = lug & _box(x - 10, x + 10, *sorted((y_in, y_in + sy * 20)), z0 - 1, lug_top + 1)
        # foot=0: the lug's own bed chamfer met the saddle's chamfer on the cheek face in two BSPLINE slivers that
        # the mesher could not triangulate (the bridge vanished in renders, I013); the final finish chamfers the bed
        s = s + _finish_print(lug, "-Z", outer=PRINTED["fillet_outside_min"], foot=0.0)
    # S1.29 collar: a 47 square (R 4) prism along the optical axis from the top plate up to 5.9 in front of the board
    # underside plane, pocketed 37 square for the camera board, with the 45 recess the WS2812 ring board sits in
    # (floor 4.3 in front of the board underside) and a 25 wide slot in its +Y wall for the camera ribbon.
    cf = Plane(origin=tuple(p0), x_dir=tuple(t), z_dir=tuple(a))
    top = LEDR_Z + LEDR_RECESS_DEPTH
    col = cf * extrude(RectangleRounded(COLLAR_OUT, COLLAR_OUT, BR_R_COLLAR).moved(Location((0, 0, -60.0))), top + 60.0)
    zc = BR_TOP_Z1                                           # collar foot: on the saddle's top plate (I028)
    col = col & _box(x0 + 0.01, 200, -200, 200, zc - 0.01, 300)   # I028: the wedge's -X foot stops at the saddle end
    col = col - cf * Box(COLLAR_IN, COLLAR_IN, 20, align=(Align.CENTER, Align.CENTER, Align.MIN))
    col = col - cf * Box(25.0, 20.0, 20, align=(Align.CENTER, Align.MIN, Align.MIN)).moved(Location((0, 15.0, 0)))
    col = _finish_print(col, None, outer=1.0)
    # I032: the 45 recess is cut after the collar finish; its rim meets the ribbon slot walls 0.2 past the pocket
    # wall, and filleting that sliver crashes OCCT's fillet builder (segfault), so the recess rim stays sharp here
    col = col - cf * Cylinder(LEDR_RECESS_D / 2, 20, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, LEDR_Z)))
    s = _solid(s + col).clean()
    for x, y in BR_SCREWS:                                     # I028: M3 heat-set pockets from the bed face (screws come
        s = s - Cylinder(BR_INSERT_D / 2, BR_INSERT_L, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((x, y, z0 - 0.01)))                       # up through the disc), so no driver bore cuts the wedge
    v = np.array([0.0, 1.0, 0.0])
    hp = CAM_HOLE_PITCH / 2
    for su in (-1, 1):
        for sv in (-1, 1):
            c = p0 + t * su * hp + v * sv * hp
            s = s - Plane(origin=tuple(c + a * 0.5), x_dir=tuple(t), z_dir=tuple(-a)) * Cylinder(
                1.6, 4.5, align=(Align.CENTER, Align.CENTER, Align.MIN))       # M2 heat-set pocket 3.2
    return _finish_print(s, "-Z", outer=PRINTED["fillet_outside_min"])   # anything the unions left sharp


def _charge_port():
    """Adafruit 6069 class round USB-C panel mount: 19 flange under the disc, 12 body through the 6 mm disc,
    14 nut on the disc top, cable boot. Panel underside at z 0."""
    from build123d import RectangleRounded
    fl = Cylinder(PORT_FLANGE_D / 2, PORT_FLANGE_T, align=(Align.CENTER, Align.CENTER, Align.MAX))
    body = Cylinder(PORT_BODY_D / 2, DISC_T + 8.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    nut = Cylinder(PORT_NUT_D / 2, PORT_NUT_T, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, DISC_T)))
    boot = Cylinder(3.5, 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, DISC_T + 8.0)))
    s = fl + body + nut + boot
    s = s - extrude(Plane(origin=(0, 0, -PORT_FLANGE_T - 0.1)) * RectangleRounded(8.94, 3.26, 1.6), amount=6.5)
    return _solid(s)


def _power_switch():
    """TRU Components LAS4GQH-11 latching pushbutton (Conrad 1661896), from its data-sheet drawing: 14 dia x 3.2
    stainless head with a 7.4 button 0.6 proud, M12 x 1 threaded body (drawn at 12.0) 36.35 behind the head, a
    hex nut 16.2 across corners x 2.0 (thickness assumed), three 0.65 solder tabs. Local: panel underside at z 0,
    head below (facing -Z), body up through the panel; the nut goes on at z = nut_z."""
    from build123d import RegularPolygon
    head = Cylinder(SW_HEAD_D / 2, SW_HEAD_T, align=(Align.CENTER, Align.CENTER, Align.MAX))
    btn = Cylinder(SW_BTN_D / 2, 0.6, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(Location((0, 0, -SW_HEAD_T)))
    body = Cylinder(SW_THREAD_D / 2 - 0.02, SW_BODY_L - 5.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    tail = Cylinder(5.0, 5.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, SW_BODY_L - 5.0)))
    s = head + btn + body + tail
    for x in (-2.5, 0.0, 2.5):
        s = s + Box(0.65, 2.0, 4.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, 0, SW_BODY_L)))
    nut = extrude(RegularPolygon(SW_NUT_AC / 2, 6, rotation=30.0), amount=SW_NUT_T) - Cylinder(SW_THREAD_D / 2, 10)
    nut = nut.moved(Location((0, 0, DISC_T)))                  # flats across the radius (14.0 radial)
    return _solid(s), _solid(nut)


R_MOTOR19 = np.array([[-1.0, 0, 0], [0, 0, 1.0], [0, 1.0, 0]])   # STEP local +Y (shaft) -> world +Z, local -Z (cable
#                                                                 exit) -> world -Y, local X -> world -X


@lru_cache(maxsize=None)
def _motor_src():
    """STEPPERONLINE 17HS19-2004D-E1K (vendor STEP 17HS19-2004D-E1K.STEP; drawing 17HS19-2004D-E1K_Dimension.svg:
    42.3 square, 67 +-1 long with the 20 +-1 encoder, 22 x 2 boss, shaft 5 x 24 with a 15 D-cut, 4 x M3 x 4.5 deep on
    31). STEP frame: axis local +Y, mounting face Y 0, boss to Y 2, encoder cover to Y -68.0, rear screw heads to
    -68.5, cable-exit block on the local -Z side. The two cables are drawn as straight stubs out to local Z -74: the
    flexible cables beyond the 9.6 cable-exit block (local Z < -31) are dropped. The shaft is its own solid so it can turn.
    Returns (body solids, shaft solid) in source coordinates."""
    sols = _step_solids(MOTOR_STEP)

    def is_shaft(q):
        b = q.bounding_box()
        return max(abs(b.min.X), abs(b.max.X), abs(b.min.Z), abs(b.max.Z)) <= 2.51 and b.max.Y > 20
    shaft = next(q for q in sols if is_shaft(q))
    keep = _box(-21.2, 21.2, -69.0, 2.1, -31.0, 21.2)
    kb = keep.bounding_box()
    body = []
    for q in sols:
        if q is shaft:
            continue
        b = q.bounding_box()
        if b.min.X >= kb.min.X and b.max.X <= kb.max.X and b.min.Y >= kb.min.Y and b.max.Y <= kb.max.Y and \
                b.min.Z >= kb.min.Z and b.max.Z <= kb.max.Z:
            body.append(q)
        elif b.max.Z > kb.min.Z and b.min.Z < kb.max.Z:
            body.extend((q & keep).solids())
    # the STEP's four M3 holes stop 3.75 deep; the drawing says DEPTH 4.5: tap-drill them to the drawing depth
    for i, q in enumerate(body):
        b = q.bounding_box()
        if b.max.Y > -0.5 and b.min.Y < -5.0 and b.max.X > 15:          # the front end cap carrying the holes
            for hx, hz in ((15.5, 15.5), (15.5, -15.5), (-15.5, 15.5), (-15.5, -15.5)):
                q = q - (Plane(origin=(hx, 0.01, hz), z_dir=(0, -1, 0)) *
                         Cylinder(1.25, 4.51, align=(Align.CENTER, Align.CENTER, Align.MIN)))
            body[i] = _solid(q)
    return body, shaft


def drive_cartridge(A):
    """Names of every body in the S1.26 cartridge (its parts plus the fasteners made on the bench)."""
    core = [*BD.RETAINER_NAMES, MOTOR, "v_nema17_shaft", "lp_motor_plate", *SPACER_NAMES, *BD.PART_NAMES, "lp_plate_a",
            "v_gobilda_1614_pillow_block", "v_gobilda_1614_bearing_lower", "v_gobilda_1614_bearing_upper",
            "v_gobilda_rex_shaft_2106_72", "v_gobilda_1313_hub",
            *RBLOCK_NAMES, *ROLLER_NAMES, *AXLE_SPACER_NAMES, "fp9_hall_bracket", "v_index_hall_ky003"]
    hw = [n for n, p in A.parts.items()
          if str(p.meta.get("joint", "")).startswith(("j_s1_1_", "j_s1_2_", "j_s1_5_", "j_s1_27_", "j_s1_31_", "j_s1_6_"))]
    return [n for n in core if n in A.parts] + hw


def _drive(A):
    """S1.26 drive-base cartridge: motor plate (with the offset motor and its direct pulley) hung under plate A on four
    spacers, plate A carrying the pillow block, REX shaft, race spacer + shims and REX pulley; built and spun on the bench,
    dropped through the S2.5 deck opening and held by the four S1.4 screws only."""
    body, shaft = _motor_src()
    loc = _location(BD.rz(BD.MOTOR_ANGLE) @ R_MOTOR19, (*BD.MOTOR_XY, MOTOR_FACE_Z))
    A.add(MOTOR, Compound(children=[copy.deepcopy(q).moved(loc) for q in body]), "motor", "drive_base", step=1,
          note=f"STEPPERONLINE 17HS19-2004D-E1K closed-loop motor, 52 Ncm (73.64 oz.in) with 1000 PPR encoder (vendor "
               f"STEP), face on the motor plate underside Z {MOTOR_FACE_Z:g}, uncut 24 mm shaft, turned {BD.MOTOR_ANGLE:g} deg about its axis so the lead glands face +Y, away from the spindle")
    A.add("v_nema17_shaft", copy.deepcopy(shaft).moved(loc), "shaft", "drive_base", step=1, spins=True, spin_origin=(*BD.MOTOR_XY, 0),
          note="motor shaft (vendor STEP, uncut), 5 with a 15 D-cut, tip Z %.2f in the direct 5 mm bore motor pulley" % MOTOR_SHAFT_TIP)
    A.vendor.append(V.vendor_entry(MOTOR, MOTOR_DIR))
    A.add("lp_motor_plate", _motor_plate(), "plate", "drive_base", step=1,
          note=f"laser part, 6061 {MPLATE_T} Fablight plate {MPLATE_X:g} x {MPLATE_Y:g} (long along Y) hung 20 under "
               "plate A on four spacers, between the cross rails (S1.1, S1.2); "
               f"{MPLATE_BELT_SLOT_W:g} wide belt slot along Y (X +-{MPLATE_BELT_SLOT_W / 2:g}, Y 0..{BD.MOTOR_XY[1] + BD.TRAVEL[0]:g}) "
               f"joins the 24 spindle hole to the 24 pilot slot, four cusps R {MPLATE_BELT_SLOT_R:g} (I035); 5.5 support holes "
               "keep 8.25 wall to the slot, 6.27 to the spindle hole")
    for nm, (x, y) in zip(SPACER_NAMES, SPACER_XY):
        A.add(nm, HW.ring(SPACER_OD, SPACER_ID, SPACER_L, nm).moved(Location((x, y, MPLATE_Z1))), "standoff",
              "drive_base", step=2, note="RAF M0669-5-AL aluminium round spacer 10 OD x 5.3 ID x 20 (S1.1)")
        A.vendor.append(V.vendor_entry(nm, "hardware/raf_m0669_5_al_spacer"))
    BD.add(A)  # V12 S1.3: complete toothed pulleys and closed-loop belt
    A.add("lp_plate_a", _plate_a(), "plate", "drive_base", step=2,
          note=f"laser part, 6061 {PLATE_T} plate 120 x 120 on the deck sheet, the cartridge base plate (ICD S1.4, S1.5, "
               "S1.26)")
    housing, lower, upper, c = _block_parts()
    loc = _axial_loc(c, BLOCK_Z0)
    A.add("v_gobilda_1614_pillow_block", copy.deepcopy(housing).moved(loc), "bearing", "drive_base", step=2,
          includes="goBILDA 1614-0016-4008 dual-bearing pillow block housing (vendor STEP), M4 tapped 16 mm pattern")
    A.vendor.append(V.vendor_entry("v_gobilda_1614_pillow_block", VM + "gobilda_1614_pillow_block"))
    for nm, grp in (("v_gobilda_1614_bearing_lower", lower), ("v_gobilda_1614_bearing_upper", upper)):
        A.add(nm, Compound(children=[copy.deepcopy(s).moved(loc) for s in grp]), "bearing", "drive_base", step=2,
              spins=True, includes="the block's REX-bore ball bearing and its shields (vendor STEP)")
    sh, sc = _shaft_src()
    A.add("v_gobilda_rex_shaft_2106_72", copy.deepcopy(sh).moved(_axial_loc(sc, SHAFT_Z0)), "shaft", "rotor", step=2,
          includes=f"goBILDA 2106-4008-0720 REX shaft 8 x 72, Z {SHAFT_Z0:g}..{SHAFT_Z1:g}")
    A.vendor.append(V.vendor_entry("v_gobilda_rex_shaft_2106_72", VM + "gobilda_2106_rex_shaft_72"))
    BD.add_retainers(A)
    for k, az in enumerate(ROLLER_ANG):                                               # S1.27 rim rollers
        A.add(RBLOCK_NAMES[k], _roller_block(az), "printed", "drive_base", step=2,
              note=f"PETG rim-roller block at {az:g} deg on plate A's arm: axle slot + pinch guard (S1.27); "
                   "justification in its builder docstring")
        rn = ROLLER_NAMES[k]
        A.add(rn, Compound(children=_roller(az)), "bearing", "drive_base", step=4,
              bodies=[(f"{rn}_body_{i}", "pom_black" if i==0 else "steel_bearing") for i in range(30)],
              includes=f"Creality standard Ender POM wheel kit: two 625 bearings and 1 mm internal shim; flat tread at R{ROLLER_R:g}, Z45")
        A.vendor.append(V.vendor_entry(rn, "motion/creality_ender_vwheel"))
        A.add(AXLE_SPACER_NAMES[k], _axle_spacer(az), "standoff", "drive_base", step=4,
              note="Steel precision shim 5 ID x 10 OD x 1; between bearing inner race and support wall")
        jn = f"j_s1_27_axle_{k + 1}"
        for nm, kind, sol in zip((f"hw_m5x25_shcs_{jn}", f"hw_washer_m5_{jn}", f"hw_nyloc_m5_{jn}"),
                                 ("screw", "washer", "nut"), _axle_hw(az)):
            A.add(nm, sol, kind, "drive_base", step=4, joint=jn,
                  note="M5x25 axle head outboard; 5/10/1 precision washer each side; M5 nyloc inboard. Internal 1 mm shim included in wheel kit.")



def _hall(A):
    """S1.31: the stationary index sensor on plate A (part of the S1.26 cartridge)."""
    A.add("fp9_hall_bracket", _hall_bracket(), "printed", "drive_base", step=2,
          note=f"PETG hall bracket on plate A's corner at {HALL_AZ:g} deg (S1.31); justification in its builder docstring")
    pcb, sen = _hall_board()
    A.add("v_index_hall_ky003", Compound(children=[pcb, sen]), "sensor", "drive_base", step=2,
          bodies=[("v_index_hall_ky003_breakout", "pcb_green"), ("v_index_hall_ky003_a3144", "black_plastic")],
          note=f"Joy-it SEN-KY003HMS (A3144 hall switch breakout) on edge, sensing face {HALL_GAP:g} under the disc at "
               f"r {SENSOR_U:g}, {HALL_AZ:g} deg, under the index magnet circle (S1.31)")
    A.vendor.append(V.vendor_entry("v_index_hall_ky003", "electronics/hall_ky003_joyit"))


def _rotor(A):
    hub = _hub_src()
    loc = _location(_rz(HUB_ANG), (0, 0, HUB_FLANGE_TOP))
    A.add("v_gobilda_1313_hub", Compound(children=[copy.deepcopy(s).moved(loc) for s in hub]), "hub", "rotor", step=3,
          includes="goBILDA 1313-1632-4008 Hyper Hub (vendor STEP) with its two M4 pinch bolts")
    A.vendor.append(V.vendor_entry("v_gobilda_1313_hub", VM + "gobilda_1313_hyper_hub"))
    A.add("lp_carrier_disc", _carrier_disc(), "plate", "rotor", step=4,
          note=f"laser part, 6061 {DISC_T} disc {DISC_D:g} (ICD S1.8 .. S1.24)")
    for k, ang in enumerate(MAG_ANG, 1):                              # S1.30 index magnets, flush underneath
        nm = f"v_index_magnet_6x3_n42_{k}"
        x, y = _pol(MAG_R, ang)
        A.add(nm, _magnet().moved(Location((x, y, DISC_Z0))), "magnet", "rotor", step=4, spins=True,
              note=f"6 x 3 N42 index magnet pressed and glued into the 6.1 hole at r {MAG_R:g}, {ang:g} deg, flush with "
                   "the disc underside, south pole down (S1.30)")
        A.vendor.append(V.vendor_entry(nm, "hardware/magnet_6x3_n42"))
    A.add("lp_ring_b", _ring_b(), "plate", "rotor", step=4, rmin=RING_B[0] / 2 - 0.01,
          note="laser part, 3.175 6061 flat ring on the disc under the bottom tube seat (S1.9)")
    A.add("fp9_tube_seat_bottom", _tube_seat_bottom(), "printed", "rotor", step=4, rmin=SEAT_CH_R[1] - 0.01,
          note=f"PETG bottom tube seat on ring B, bore {2 * SEAT_CH_R[1]:.1f} socket for the tube foot (S1.9, S1.10); justification in its "
               "builder docstring")
    # Pi standoffs (bench: on the disc before it goes on the hub)
    for k, (x, y) in enumerate(PI_HOLES, 1):
        nm = f"v_pi_standoff_m2p5_{k}"
        A.add(nm, HW.hex_standoff(5.0, PI_STANDOFF, 2.05, nm).moved(Location((x, y, DRUM_FLOOR))), "standoff",
              "electrical", step=4, spins=True, note="M2.5 x 6 F-F hex standoff under the Pi Zero (S1.18)")
        A.vendor.append(dict(component=nm, source_url=None, author="catalogue (M2.5 x 6 F-F hex standoff)",
                             licence=None, file=None, price_usd=0.1))
    # power bank flat on the axis under the camera bridge (S1.16, S1.19)
    R = _R([(1, 0, 0), (0, 0, 1), (0, -1, 0)])        # bank local Y (23.1) -> world Z, local Z (45.2) -> world Y
    A.add("v_power_bank", V8._bank_solid().moved(_location(R, (BANK_OFF[0], BANK_OFF[1], BANK_Z0 + BANK_W / 2))),
          "power_bank",
          "electrical", step=5, spins=True,
          includes="Anker 321 Power Bank (PowerCore 5K, A1112) 96 x 45.2 x 23.1, 118 g, lying flat on the axis, "
                   "long axis along X, ports at -X")
    A.vendor.append(V.vendor_entry("v_power_bank", "electronics/anker-321-power-bank-a1112"))
    A.add("fp9_camera_wedge_bridge", _camera_bridge(), "printed", "rotor", step=5,
          note="PETG camera mount: one wedge saddle over the bank, lens 48 above the disc (S1.16, S1.19, I028); "
               "justification in its builder docstring")
    A.add("v_charge_port", _charge_port().moved(_dev_loc(PORT_XY, DISC_Z0)), "switch", "electrical", step=4,
          spins=True, note=f"round USB-C panel mount through the disc at ({PORT_XY[0]:g}, {PORT_XY[1]:g}) (r {PORT_R:.1f}, "
                           "inside the tube beside the bank's USB-C end), facing down (S1.22, S1.28)")
    A.vendor.append(V.vendor_entry("v_charge_port", "electronics/adafruit-6069-usb-c-round-panel-mount"))
    sw, nut = _power_switch()
    A.add("v_power_switch", Compound(children=[sw.moved(_dev_loc(SWITCH_XY, DISC_Z0)),
                                               nut.moved(_dev_loc(SWITCH_XY, DISC_Z0))]), "switch", "electrical",
          step=4, spins=True, note=f"TRU LAS4GQH-11 12 mm latching pushbutton through the disc at ({SWITCH_XY[0]:g}, "
                                   f"{SWITCH_XY[1]:g}) (r {SWITCH_R:.1f}, inside the tube), button facing down, nut on the "
                                   "disc top (S1.21, S1.28)")
    A.vendor.append(V.vendor_entry("v_power_switch", "electronics/tru-las4gqh-11-latching-12mm"))
    # Pi (S1.18)
    s = V7._pi_src()
    R = _R([(1, 0, 0), (0, 0, 1), (0, -1, 0)])
    t = np.array([PI_C[0], PI_C[1], DRUM_FLOOR + PI_STANDOFF]) - R @ np.array([0.0, -0.8, 0.0])
    npi = len(s.solids())          # box_envelope: the vendor PCB solid first, every component as its box after it
    A.add("v_rpi_zero_2w", copy.deepcopy(s).moved(_location(R, t)), "pcb", "electrical", step=5, spins=True,
          bodies=[("v_rpi_zero_2w_pi_pcb", "pcb_soldermask")]
                 + [(f"v_rpi_zero_2w_pi_part_{i}", "solder_tin") for i in range(1, npi)])   # I015: silver parts
    A.vendor.append(dict(V.manifest_entry("v_rpi_zero_2w", "v_rpi_zero_2w"),
                         note="representative solid derived from the vendor STEP (PCB kept, parts as boxes)"))
    a, tt, p0 = _cam_frame()
    # S1.17: the board's local X (the sensor's long, 194 deg image axis) lies along tt, in the X-Z tilt plane, so the
    # image width runs from the -X edge of the top disc to the +X wall frog; the FFC connector on the local +Y edge
    # sends the ribbon toward world +Y (the Pi side).
    cam = HW.camera_uc261("v_camera_uc261", connector=True, glass=True).moved(
        Location(Plane(origin=tuple(p0), x_dir=tuple(tt), z_dir=tuple(a))))
    A.add("v_camera_uc261", cam, "camera", "rotor", step=6,
          bodies=[("v_camera_uc261_pcb", "pcb_soldermask"), ("v_camera_uc261_lens", "lens_housing"),
                  ("v_camera_uc261_fpc", "black_plastic"), ("v_camera_uc261_glass", "lens_glass_dark")],
          note="Arducam UC-261 class envelope: PCB 36 x 36 x 1.6, lens barrel 14 x 12, FFC connector 22 x 5.5 x 2.5; "
               "sensor long axis in the X-Z tilt plane, ribbon leaves toward +Y (S1.17)")
    A.vendor.append(dict(component="v_camera_uc261", source_url=None, author="catalogue (Arducam UC-261 envelope)",
                         licence=None, file="lib/hardware.py:camera_uc261", price_usd=None))
    ledr, ledr_b = _led_ring()
    ledr = ledr.moved(Location(Plane(origin=tuple(p0 + a * LEDR_Z), x_dir=tuple(tt), z_dir=tuple(a))))
    A.add("v_led_ring_ws2812", ledr, "pcb", "rotor", step=6, spins=True, bodies=ledr_b,
          note=f"WS2812B 16 LED white ring, Adafruit NeoPixel Ring 16 footprint {LEDR_OD:g} x {LEDR_ID:g} x {LEDR_T:g} "
               f"(Amazon Sparkleiot B0C77XPKSL class), 5 V, coaxial round the UC-261 lens in the bridge collar's "
               f"{LEDR_RECESS_D:g} recess, LEDs along the optical axis, 3 wires (5 V, GND, DIN) to the Pi (S1.29, I032)")
    A.vendor.append(V.vendor_entry("v_led_ring_ws2812", "electronics/ws2812_ring_16"))
    ddz = TUBE_FOOT - D0.DRUM_FLOOR_Z
    A.add("s_drum_tube", s_drum_tube.build(TUBE_H, TUBE_OD, TUBE_WALL).moved(Location((0, 0, ddz))), "stock", "rotor", step=6,
          opacity=0.35, rmin=TUBE_IR - 0.01,
          note=f"{TUBE_NAME}, foot in the bottom seat channel, top in the top seat bore, silicone at both ends (S1.10); "
               f"OD tolerance unknown (assumed nominal {TUBE_OD:g}): measure then print the seats at measured OD + "
               f"{2 * SEAT_RADIAL:g} (I036)")
    A.vendor.append(dict(V.vendor_entry("s_drum_tube", "stock/acrylic_tube_rebower_145x150"), file="parts/s_drum_tube.py"))
    A.add("fp9_tube_seat_top", _tube_seat_top(), "printed", "rotor", step=6, rmin=SEAT_CH_R[1] - 0.01,
          note="PETG top tube seat round the tube top under the top disc (S1.12, S1.13); justification in its builder "
               "docstring")
    A.add("s_top_disc", _top_disc(), "stock", "rotor", step=6, opacity=0.35,
          note=f"acrylic top disc {TOP_DISC_D:g} x {TOP_DISC_T:g} on the tube rim and the top seat (S1.12)")
    A.add("lp_ring_top", _ring_top(), "plate", "rotor", step=6, rmin=RING_TOP[0] / 2 - 0.01,
          note="laser part, 3.175 6061 bezel ring on the top disc, 6 x M4 to the top seat (S1.13)")
    for k, ang in enumerate(POST_ANG, 1):                              # S1.32 drum posts
        x, y = _pol(POST_R, ang)
        A.add(f"v_drum_post_{k}", _drum_post().moved(Location((x, y, RING_B_Z1))), "standoff", "rotor", step=6,
              spins=True, note=f"M5 female-female round aluminium standoff {POST_D:g} x {POST_L:g} at r {POST_R:g}, {ang:g} deg, "
                               "inside the tube on the ring B lobe (M5 stud in the 5.5 hole), top face under the top "
                               "disc: carries the top ring (S1.32, I014)")
        A.vendor.append(V.vendor_entry(f"v_drum_post_{k}", POST_VENDOR))
        st = Cylinder(M5_TAP_D / 2 - 0.01, 16.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((x, y, DISC_Z0 + 0.4)))
        A.add(f"hw_m5x16_stud_j_s1_32_stud_{k}", _solid(st), "set_screw", "rotor", step=6, spins=True,
              joint=f"j_s1_32_stud_{k}", note="M5 x 16 threaded stud (DIN 913 set screw, full thread): 7.2 in the post, "
                                              "through ring B, 5.6 in the disc, tip 0.4 short of the underside (S1.32)")
    sy = _wall_frog_sy()
    for nm, (R, t) in (("v_frog_wall", _frog_wall_pose()), ("v_frog_disc", _frog_disc_pose())):
        for part in ("body", "eyes"):
            src = V._mesh_solid(V7.FROG_BODY_STL) if part == "body" else V7._frog_eyes_local()
            src = copy.deepcopy(src)
            inc = "Agalychnis callidryas, 45 mm snout-vent (mesh x 19.5, decimated)"
            if nm == "v_frog_wall":              # S1.10 placeholder: stretched head-to-tail to WALL_FROG_H (I017)
                src = scale(src, by=(1.0, sy, 1.0))
                inc += f", stretched x {sy:.3f} head-to-tail to the {WALL_FROG_H:g} tall largest sponsor frog"
            A.add(f"{nm}_{part}", src.moved(_location(R, t)), "specimen", "rotor", step=6, includes=inc)
            A.vendor.append(V.vendor_entry(f"{nm}_{part}", V7.FROG_DIR))


def _dev_loc(xy, z):
    ang = math.degrees(math.atan2(xy[1], xy[0]))
    return _location(_rotz(ang), np.array([xy[0], xy[1], z]))


def _wall_frog_sy():
    """Head-to-tail (local Y) stretch that makes the 45 mm mesh frog WALL_FROG_H tall in the wall pose (I017: frogs
    2 to 10 cm from the sponsor). Depth and width stay the 45 mm frog's, so the net gap is not the 10 cm frog's."""
    v = V7._frog_verts()
    return WALL_FROG_H / float(v[:, 1].max() - v[:, 1].min())


def _wall_frog_verts():
    return V7._frog_verts() * np.array([1.0, _wall_frog_sy(), 1.0])


def _frog_wall_pose():
    """v7 pose (belly on the tube at +X, head up), centred in the wall band between the two tube seats (S1.10)."""
    R = _R([(0, -1, 0), (0, 0, -1), (1, 0, 0)])
    v = _wall_frog_verts() @ R.T
    c = (v.min(0) + v.max(0)) / 2
    zc = (SEAT_B_Z1 + SEAT_T_Z0) / 2
    return R, np.array([TUBE_OR + V7.FROG_CLEAR - v[:, 0].min(), -c[1], zc - c[2]])


def _frog_disc_pose():
    R = _R([(-1, 0, 0), (0, -1, 0), (0, 0, 1)])
    v = V7._frog_verts() @ R.T
    c = (v.min(0) + v.max(0)) / 2
    lim = RING_TOP[0] / 2 - 3.0
    z = TUBE_TOP + TOP_DISC_T + V7.FROG_CLEAR - v[:, 2].min()
    lo, hi = 0.0, lim
    for _ in range(50):
        mid = (lo + hi) / 2
        if np.hypot(v[:, 0] - c[0] + mid, v[:, 1] - c[1]).max() <= lim - V7.FROG_DISC_SPARE:
            lo = mid
        else:
            hi = mid
    return R, np.array([math.floor(lo * 100) / 100 - c[0], -c[1], z])


# ------------------------------------------------------------------ S3 cage: tube-post cube, magnet-on mesh panels

_CORNER_ROT = {"pxpy": ((1, 0, 0), (0, 1, 0)), "nxpy": ((0, 1, 0), (-1, 0, 0)),
               "nxny": ((-1, 0, 0), (0, -1, 0)), "pxny": ((0, -1, 0), (1, 0, 0))}
_FOOT_HAND = {"pxpy": ("a", "pxpy"), "nxny": ("a", "nxny"), "pxny": ("b", "pxpy"), "nxpy": ("b", "nxny")}


def _rot_cols(c):
    a, b = _CORNER_ROT[c]
    return _R([a, b, (0, 0, 1)])


@lru_cache(maxsize=None)
def _cage_src(which):
    """Cached local cage parts (parts/fp9_cage.py): 'top', 'foot_a', 'foot_b', 'mesh_side', 'mesh_top', and the panels
    'lower', 'lower_notch', 'upper', 'top_panel'."""
    if which == "top":
        return CG.corner_top()
    if which.startswith("foot_"):
        return CG.corner_foot(which[-1])
    if which == "lower":
        return CG.panel("lower", CAGE_POST_L)
    if which == "lower_notch":
        return CG.panel("lower", CAGE_POST_L, notch=True)
    if which == "upper":
        return CG.panel("upper", CAGE_POST_L)
    if which == "top_panel":
        return CG.panel("top", CAGE_POST_L)
    kind = "top" if which == "mesh_top" else "side"
    w, h = CG.panel_size(CAGE_POST_L, kind)
    return _solid(extrude(CG.mesh_outline(kind, w, h, CAGE_POST_L).moved(Location((0, 0, CG.BAR_T))), CG.MESH_T))


def _place(src, R, t, label):
    s = copy.deepcopy(src).moved(_location(np.asarray(R, float), np.asarray(t, float)))
    s.label = label
    return s


def _panel_frame(side):
    """(R, t) mapping panel-local coordinates (x across, y up, z 0 outer .. BAR_T inner) to world."""
    if side == "top":
        w, _ = CG.panel_size(CAGE_POST_L, "top")
        return _R([(1, 0, 0), (0, -1, 0), (0, 0, -1)]), _vec(0.0, w / 2, CAGE_TOP + CG.BAR_T)
    nx, ny = CAGE_SIDES[side]
    n = _vec(nx, ny, 0.0)
    t = _vec(ny, -nx, 0.0)                         # n x Z: (t, Z, -n) is right-handed
    return _R([t, (0, 0, 1), -n]), n * (CAGE_FACE + CG.BAR_T) + _vec(0, 0, PANEL_Z0)


def _cage_magnet(label):
    m = Cylinder(CG.MAG_D / 2, CG.MAG_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    m.label = label
    return m


def _add_magnet(A, name, centre, normal, sub_step, note):
    """One 8 x 3 N42 disc bonded flush (0.05 under the face) in its pocket; `normal` = the face's outward normal,
    `centre` = the face point on the pocket axis."""
    n = _vec(*normal)
    n = n / np.linalg.norm(n)
    x = _vec(0, 0, 1) if abs(n[2]) < 0.9 else _vec(1, 0, 0)
    x = x - n * (x @ n)
    x = x / np.linalg.norm(x)
    # local Z of the disc along -n (into the part), its outer face CG.MAG_RECESS under the face
    zl = -n
    yl = np.cross(zl, x)
    R = _R([x, yl, zl])
    t = _vec(*centre) + zl * CG.MAG_RECESS
    A.add(name, _cage_magnet(name).moved(_location(R, t)), "magnet", "cage", step=sub_step, note=note)
    A.vendor.append(dict(V.vendor_entry(name, "hardware/magnet_8x3_n42"), note="K&J D52, 40 in the cage"))


def _cage(A, R_):
    """S3 cage (I030): a tube-post cube on the deck with five magnet-on mesh panels (ICD S3.1 .. S3.6).
    Install (DESIGN-RULES 5):
      8  bench, the tube frame: four posts pushed into the four foot corners (12.1 sockets, 15 deep) and the four top
         corners, the four top rails into the top corners' horizontal sockets (a dab of CA if they rattle)
      9  bench, each panel: the two printed halves lapped with 2 x M3 x 8 button head + hex nut, the netting splined
         into both halves' V grooves (one sheet per side), four 8 x 3 N42 bonded in the corner holes (pole facing in
         matched to the corner magnet it meets)
     10  the tube frame onto the deck sheet LAST: four M5 x 12 button heads (3 mm key) through the foot tongues and
         the sheet into T-nuts left in the deck Y rails; the frame comes off first (four screws out, lift it by the top
         rails), the T-nuts stay in the rails
     11  the five panels onto the cube: each one snaps onto its 4 corner magnets (20 in the corners, 20 in the panels);
         any panel pulls off by hand like a mesh door (pull per panel in the S3 note)"""
    names = {}
    # S3.1 posts, S3.2 foot corners
    for sx in (1, -1):
        for sy in (1, -1):
            c = _c(sx, sy)
            x, y = sx * CAGE_P, sy * CAGE_P
            nm = f"s_cage_post_{c}"
            A.add(nm, HW.ring(CG.TUBE_OD, CG.TUBE_OD - 2 * CG.TUBE_WALL, CAGE_POST_L, nm).moved(Location((x, y, CAGE_POST_Z0))),
                  "rail", "cage", length_mm=CAGE_POST_L, axis="z", centre=(x, y, (CAGE_POST_Z0 + CAGE_POST_Z1) / 2),
                  role=f"cage_post_{c}", step=8,
                  note=f"cage post, 6061 round tube {CG.TUBE_OD:g} x {CG.TUBE_WALL:g} x {CAGE_POST_L:g} (stock, cut to length), "
                       f"Z {CAGE_POST_Z0:g}..{CAGE_POST_Z1:g}, push-fit 15 into the foot corner and the top corner (S3.1)")
            A.meta_role[nm] = f"cage_post_{c}"
            A.vendor.append(dict(V.vendor_entry(nm, "stock/aluminium_tube_12x1"), note="stock tube, cut from 1 m"))
            hand, rot = _FOOT_HAND[c]
            fn = f"fp9_cage_corner_foot_{c}"
            A.add(fn, _place(_cage_src(f"foot_{hand}"), _rot_cols(rot), (x, y, SHEET_Z1), fn), "printed", "cage",
                  step=8, note=f"PETG foot corner (S3.2, hand {hand}): 19.4 node on the deck sheet, 12.1 x 15 socket for "
                               "the post, magnet pockets on its two outer faces, tongue to the M5 foot screw")
            names[c] = fn
            for k, (nx, ny) in enumerate(((sx, 0), (0, sy))):
                fc = _vec(x + nx * CG.NH, y + ny * CG.NH, SHEET_Z1 + CG.FOOT_MAG_Z)
                _add_magnet(A, f"hw_magnet_8x3_corner_foot_{c}_{'xy'[k]}", fc, (nx, ny, 0), 8,
                            "8 x 3 N42 disc (K&J D52) bonded in the foot corner's outer-face pocket (S3.5)")
            # S3.2 lift-off joint: M5 x 12 button head (the frame family, DL10) through the tongue and the deck sheet
            # into a T-nut in the deck Y rail
            hx, hy = sx * RC, sy * (CAGE_P + CG.TONGUE_HOLE[1])
            jn = f"j_s3_2_foot_{c}"
            bolt_joint(A, [fn, R_["deck_y_right" if sx > 0 else "deck_y_left"]], (hx, hy, 0.0), (0, 0, -1),
                       screw="M5x12", head="button", nut="slot_nut", name=jn, module="cage", step=10, evaluate=False,
                       stack=[("s_panel_deck", PANEL_T)])
            A.parts[f"hw_slot_nut_m5_{jn}"].module = "frame"            # the T-nut stays in the deck rail on lift-off
    # S3.3 top corners and top rails
    for sx in (1, -1):
        for sy in (1, -1):
            c = _c(sx, sy)
            x, y = sx * CAGE_P, sy * CAGE_P
            nm = f"fp9_cage_corner_top_{c}"
            A.add(nm, _place(_cage_src("top"), _rot_cols(c), (x, y, CAGE_ZC), nm), "printed", "cage", step=8,
                  note="PETG 3-way top corner (S3.3): 19.4 node, three 12.1 x 15 sockets (post down, two top rails), "
                       "a magnet pocket on each outer face; prints corner-down on its facet")
            for k, nv in enumerate(((sx, 0, 0), (0, sy, 0), (0, 0, 1))):
                fc = _vec(x, y, CAGE_ZC) + _vec(*nv) * CG.NH
                _add_magnet(A, f"hw_magnet_8x3_corner_top_{c}_{'xyz'[k]}", fc, nv, 8,
                            "8 x 3 N42 disc (K&J D52) bonded in the top corner's outer-face pocket (S3.5)")
    for side, (nx, ny) in CAGE_SIDES.items():
        nm = f"s_cage_top_{side}"
        ax = "x" if nx == 0 else "y"
        cx, cy = nx * CAGE_P, ny * CAGE_P
        tube = HW.ring(CG.TUBE_OD, CG.TUBE_OD - 2 * CG.TUBE_WALL, TOP_RAIL_L, nm)
        if ax == "x":
            tube = tube.moved(Location((-TOP_RAIL_L / 2, cy, TOP_RAIL_Z), (0, 1, 0), 90))
        else:
            tube = tube.moved(Location((cx, -TOP_RAIL_L / 2, TOP_RAIL_Z), (1, 0, 0), -90))
        A.add(nm, tube, "rail", "cage", length_mm=TOP_RAIL_L, axis=ax, centre=(cx, cy, TOP_RAIL_Z), role=f"cage_top_{side}",
              step=8, note=f"cage top rail, 6061 round tube {CG.TUBE_OD:g} x {CG.TUBE_WALL:g} x {TOP_RAIL_L:g}, push-fit 15 "
                           "into two top corners (S3.3)")
        A.meta_role[nm] = f"cage_top_{side}"
        A.vendor.append(dict(V.vendor_entry(nm, "stock/aluminium_tube_12x1"), note="stock tube, cut from 1 m"))
    # S3.4 panels, S3.5 magnets, S3.6 netting
    for side in list(CAGE_SIDES) + ["top"]:
        R, t = _panel_frame(side)
        if side == "top":
            w, h = CG.panel_size(CAGE_POST_L, "top")
            parts = [("fp9_cage_panel_top", "top_panel", "top")]
        else:
            w, h = CG.panel_size(CAGE_POST_L, "side")
            low = "lower_notch" if side in ("right", "left") else "lower"
            parts = [(f"fp9_cage_panel_{side}_lower", low, "lower"), (f"fp9_cage_panel_{side}_upper", "upper", "upper")]
        for nm, src, part in parts:
            A.add(nm, _place(_cage_src(src), R, t, nm), "printed", "cage", step=9,
                  note=f"PETG mesh panel frame ({side}, {part}; S3.4): bars {CG.BAR_W:g} x {CG.BAR_T:g}, 45 deg V spline "
                       f"groove on the inner face, 4 magnet holes at the corners")
            for k, (mx, my) in enumerate(CG.magnet_points(part, w, h, side=side != "top")):
                fc = R @ _vec(mx, my, CG.BAR_T) + t
                inward = R @ _vec(0, 0, 1.0)
                mn = f"hw_magnet_8x3_panel_{side}_{part}_{k + 1}"
                _add_magnet(A, mn, fc, inward, 9, f"8 x 3 N42 disc (K&J D52) bonded flush in the {side} panel's "
                                                    "corner hole, face on the inner face (S3.5)")
        mname = f"s_cage_mesh_{side}"
        mesh = _place(_cage_src("mesh_top" if side == "top" else "mesh_side"), R, t, mname)
        A.add(mname, mesh, "mesh", "cage", step=9,
              note=f"Sailrite No-See-Um netting, {side} panel (modelled {CG.MESH_T:g} on the inner face), splined into "
                   "the V grooves; cut round the corner nodes and the lap screws (S3.6)")
        A.vendor.append(V.vendor_entry(mname, "textile/sailrite_no_see_um_netting"))
        if side != "top":
            for i, (lx, ly) in enumerate(CG.lap_screws(w, h), 1):
                at = R @ _vec(lx, ly, CG.BAR_T / 2) + t
                axis = R @ _vec(0, 0, 1.0)
                hex_nut_joint(A, [f"fp9_cage_panel_{side}_lower", f"fp9_cage_panel_{side}_upper"], tuple(at),
                              tuple(axis), screw="M3x8", head="button", name=f"j_s3_4_lap_{side}_{i}", module="cage",
                              evaluate=False, step=9)
    A.meta_cage = dict(names=names)
    # render specs (lib/render.py): the cage page explodes the panels out along their normals
    ex = {}
    for n, p in A.parts.items():
        if p.module != "cage" and not n.startswith("hw_slot_nut_m5_j_s3"):
            continue
        side = next((s for s in list(CAGE_SIDES) + ["top"] if f"_{s}" in n and ("panel" in n or "mesh" in n
                                                                                or "lap_" in n)), None)
        if side == "top":
            ex[n] = (0.0, 0.0, 120.0)
        elif side:
            nx, ny = CAGE_SIDES[side]
            ex[n] = (nx * 110.0, ny * 110.0, 0.0)
        elif "_j_s3_2_foot_" in n:
            ex[n] = (0.0, 0.0, -50.0)
        else:
            ex[n] = (0.0, 0.0, 0.0)
    A.cage_explode = ex


BOM_S3 = [  # part, qty, kind, unit price USD
    ("6061 round tube 12 x 1, 1 m length (4 posts %g + 4 top rails %g = %g mm, stock)"
     % (CAGE_POST_L, TOP_RAIL_L, 4 * CAGE_POST_L + 4 * TOP_RAIL_L), 2, "stock", 6.0),
    ("fp9_cage_corner_top / fp9_cage_corner_foot PETG corners", 8, "printed", 0.5),
    ("fp9_cage_panel PETG mesh panel frames (4 sides x 2 halves + 1 top)", 9, "printed", 1.2),
    ("K&J Magnetics D52 5/16 x 1/8 in (7.94 x 3.17) N42 disc magnet", 40, "vendor", 0.54),
    ("Sailrite No-See-Um netting 60 in, 1 yd (5 panels)", 1, "vendor", 4.95),
    ("screen spline cord 3 mm, about 6 m (V grooves)", 1, "vendor", 4.0),
    ("M5 x 12 button head, black (foot corners to the deck Y rails)", 4, "vendor", 0.15),
    ("M5 2020 drop-in T-nut", 4, "vendor", 0.2),
    ("M3 x 8 button head + M3 hex nut (panel laps)", 8, "vendor", 0.1),
    ("CA glue (magnets in their pockets)", 1, "vendor", 3.0),
]


def s3_notes(A):
    fr = frog_report(A)
    tot = sum(q * p for _, q, _, p in BOM_S3)
    pull = CAGE_PULL_PAIR_KG
    return [
        f"S3 cage clearances: frog_wall_to_mesh_mm={fr['gap_net']:.2f} frog_disc_to_top_mesh_mm={fr['gap_mesh_top']:.2f} "
        f"drum_top_to_top_mesh_mm={CAGE_TOP - CG.MESH_T - DISC_TOP_Z:.2f} rotor_to_side_mesh_mm="
        f"{CAGE_FACE - CG.MESH_T - DISC_D / 2:.2f} mesh_slits=0",
        f"S3 cage (I030): tube-post cube on the deck, posts 4 x 6061 tube {CG.TUBE_OD:g} x {CG.TUBE_WALL:g} x {CAGE_POST_L:g} at "
        f"(+-{CAGE_P:g}, +-{CAGE_P:g}) Z {CAGE_POST_Z0:g}..{CAGE_POST_Z1:g}, top rails 4 x {TOP_RAIL_L:g} at Z {TOP_RAIL_Z:g}, 8 PETG corners "
        f"(4 foot, 4 three-way top, 19.4 nodes, 12.1 x 15 push-fit sockets); five mesh panels on the node faces (side "
        f"panels {2 * CAGE_FACE:g} x {CAGE_TOP - CG.EDGE_GAP - PANEL_Z0:g} in two halves lapped with 2 x M3 x 8, top "
        f"{2 * CAGE_FACE:g} square in one piece), bars {CG.BAR_W:g} x {CG.BAR_T:g}; cube {2 * (CAGE_FACE + CG.BAR_T):g} square, "
        f"top at Z {CAGE_TOP + CG.BAR_T:g}",
        f"S3 magnets: 40 x 8 x 3 N42 (K&J D52, 7.94 x 3.17, case 1 pull 1.52 kg to a thick steel plate): 20 in the "
        f"corners (3 per top corner, 2 per foot), 20 in the panels (4 each); face to face across a 0.1 gap about "
        f"{pull:g} kg per pair (assumed), so {4 * pull:g} kg to pull a whole panel off square, {pull:g} kg to peel it "
        "from one corner",
        f"S3 lift-off: any panel pulls off its magnets by hand; the whole cube comes off by four M5 x 12 button heads at "
        f"(+-{RC:g}, +-{CAGE_P + CG.TONGUE_HOLE[1]:g}) into the deck Y rails (heads 3 clear of the clamp tab arm ends); the "
        "clamp pads (124..151) stay free",
        "bom_s3=" + "; ".join(f"{n} | {q} | {k} | ${p:.2f}" for n, q, k, p in BOM_S3) + f"; total ${tot:.2f}",
    ]


# ------------------------------------------------------------------ S1 joints

def _s1_joints(A, R_):
    down, up = (0, 0, -1), (0, 0, 1)
    cross = {45: R_["cross_right"], -45: R_["cross_left"]}
    bench_a = [*BD.RETAINER_NAMES, MOTOR, "v_nema17_shaft", "lp_motor_plate", *BD.PART_NAMES, "lp_plate_a",
               "v_gobilda_1614_pillow_block", "v_gobilda_1614_bearing_lower", "v_gobilda_1614_bearing_upper",
               "v_gobilda_rex_shaft_2106_72", *SPACER_NAMES, *RBLOCK_NAMES]
    for nm, (x, y) in zip(SPACER_NAMES, SPACER_XY):                                   # S1.1 (bench)
        hex_nut_joint(A, ["lp_plate_a", "lp_motor_plate"], (x, y, MPLATE_Z1), down, screw=f"M5x{SPACER_SCREW_L:g}",
                      head="shcs", seat=(x, y, PA_Z1), stack=[(nm, SPACER_L)], name=f"j_s1_1_spacer_{_c(x, y)}",
                      module="drive_base", evaluate=False, step=2, bench=bench_a)
    for k, (x, y) in enumerate(MOTOR_HOLES, 1):                                       # S1.2 (bench)
        tapped_joint(A, ["lp_motor_plate", MOTOR], (x, y, MOTOR_FACE_Z), down, screw=MOTOR_SCREW, washer=True,
                     name=f"j_s1_2_motor_{k}", module="drive_base", evaluate=False, step=1,
                     min_engagement=3.5, bench=["lp_motor_plate", MOTOR, "v_nema17_shaft"])
    for k, az in enumerate(ROLLER_ANG):                                               # S1.27
        blk = RBLOCK_NAMES[k]
        for i, (x, y) in enumerate(_az_pts(RB_HOLES, az), 1):                          # bench: block on plate A
            hex_nut_joint(A, [blk, "lp_plate_a"], (x, y, PA_Z1), down, screw="M4x16",
                          seat=(x, y, PA_Z1 + RB_BASE_T), name=f"j_s1_27_block_{k + 1}_{i}", module="drive_base",
                          evaluate=False, step=2, bench=bench_a)
    for x, y in PA_RAIL_HOLES:                                                       # S1.4
        hp = _hardpoint(A,x,y,"drive_base")
        bolt_joint(A, ["lp_plate_a", cross[int(x)]], (x, y, 0.0), down, screw="M5x14",
                   name=f"j_s1_4_plate_a_{_c(x, y)}", module="drive_base", step=2,
                   stack=[(hp, PANEL_T)], **M5)
    for x, y in BLOCK_HOLES:                                                         # S1.5 (bench)
        tapped_joint(A, ["lp_plate_a", "v_gobilda_1614_pillow_block"], (x, y, PA_Z1), up, screw="M4x12", head="button",
                     name=f"j_s1_5_block_{_c(x, y)}", module="drive_base", evaluate=False, step=2,
                     bench=["lp_plate_a", "v_gobilda_1614_pillow_block"])
    for x, y in _polar(HUB_R32, HUB_HOLE_ANG):                                       # S1.8
        tapped_joint(A, ["lp_carrier_disc", "v_gobilda_1313_hub"], (x, y, DISC_Z0), down, screw="M4x12", head="csk",
                     seat=(x, y, DISC_Z1), name=f"j_s1_8_hub_disc_{int(round(math.degrees(math.atan2(y, x)))) % 360}",
                     module="rotor", evaluate=False, step=4)
    for k, (x, y) in enumerate(_polar(STACK_R, STACK_ANG), 1):                       # S1.9: seat + ring B
        tapped_joint(A, ["fp9_tube_seat_bottom", "lp_carrier_disc"], (x, y, DISC_Z1), down, screw=STACK_SCREW_B, head="button",
                     seat=(x, y, SEAT_B_Z0 + SEAT_CB_FLOOR), stack=[("lp_ring_b", RING_T)], min_engagement=5.6,
                     name=f"j_s1_9_seat_ring_b_{k}", module="rotor", evaluate=False, step=4)
    for k, (x, y) in enumerate(_polar(STACK_R, STACK_ANG), 1):                       # S1.13: bezel + disc + seat
        hex_nut_joint(A, ["lp_ring_top", "fp9_tube_seat_top"], (x, y, TUBE_TOP), down, screw=STACK_SCREW_T,
                      head="button", stack=[("s_top_disc", TOP_DISC_T)], name=f"j_s1_13_top_clamp_{k}",
                      module="rotor", evaluate=False, step=6)
    for k, (x, y) in enumerate(_polar(POST_R, POST_ANG), 1):                          # S1.32 top post screws
        tapped_joint(A, ["lp_ring_top", f"v_drum_post_{k}"], (x, y, TUBE_TOP), down, screw=POST_SCREW, head="button",
                     stack=[("s_top_disc", TOP_DISC_T)], name=f"j_s1_32_post_top_{k}", module="rotor",
                     evaluate=False, step=6)
    bench_h = bench_a + ["fp9_hall_bracket"]
    for i, (u, v) in enumerate(HB_HOLES, 1):                                          # S1.31 (bench)
        tapped_joint(A, ["fp9_hall_bracket", "lp_plate_a"], _hall_xyz(u, v, PA_Z1), down, screw="M3x10",
                     head="button", seat=_hall_xyz(u, v, PA_Z1 + HB_BASE_T), name=f"j_s1_31_hall_bracket_{i}",
                     module="drive_base", evaluate=False, step=2, bench=bench_h)
    ur, vt = _hall_frame()
    insert_joint(A, ["v_index_hall_ky003", "fp9_hall_bracket"], _hall_xyz(KY_U0 + KY_HOLE[0], KY_T, KY_Z0 + KY_HOLE[1]),
                 tuple(vt), screw="M3x6", head="button", name="j_s1_31_hall_board", module="drive_base", evaluate=False, step=2,
                 bench=bench_h + ["v_index_hall_ky003"])
    for x, y in BR_SCREWS:                                                           # S1.16
        # I028: bench, from below: button head under the disc, into an M3 insert in the mount's lug (no top bores)
        insert_joint(A, ["lp_carrier_disc", "fp9_camera_wedge_bridge"], (x, y, DISC_Z1), up, screw="M3x10",
                     head="button", name=f"j_s1_16_bridge_{_c(x, y)}", module="rotor", evaluate=False, step=5,
                     bench=["lp_carrier_disc", "fp9_camera_wedge_bridge", "v_power_bank"])
    for k, (x, y) in enumerate(PI_HOLES, 1):                                         # S1.18 (bench, from below)
        so = f"v_pi_standoff_m2p5_{k}"
        tapped_joint(A, ["lp_carrier_disc", so], (x, y, DISC_Z1), up, screw="M2.5x10", name=f"j_s1_18_pi_standoff_{k}",
                     module="rotor", evaluate=False, step=4, bench=["lp_carrier_disc", so])


def _plinth_joints(A):
    up = (0, 0, 1)
    for k, (x, z) in enumerate(ES_HOLES, 1):                                          # S2.10 station on the cradle
        # bench, cover off: head on the base floor inside the station, nyloc on the cradle's back gusset
        hex_nut_joint(A, [ESTOP, "fp9_estop_cradle"], (x, ES_BACK_Y, z), (0, 1, 0), screw="M4x12", head="button",
                      seat=(x, ES_BACK_Y - ES_FLOOR, z), name=f"j_s2_10_estop_{k}", module="electrical",
                      evaluate=False, step=7, bench=["fp9_estop_cradle"])
    for k, (x, z) in enumerate(ES_CR_SCREWS, 1):                                      # S2.10 cradle to the panel
        insert_joint(A, ["s_panel_front", "fp9_estop_cradle"], (x, ES_PANEL_IN, z), (0, 1, 0), screw="M3x10",
                     head="button", name=f"j_s2_10_cradle_{k}", module="electrical", evaluate=False, step=7,
                     bench=["s_panel_front", "fp9_estop_cradle", ESTOP])
    for k, (x, y) in enumerate(A.meta_psu_holes, 1):
        tapped_joint(A, ["s_panel_bottom", "v_psu_lrs_50_24"], (x, y, V8.FLOOR_Z), up, screw="M3x8",
                     name=f"j_psu_panel_{k}", module="electrical", evaluate=False, step=7)
    for k, (x, y) in enumerate(fp7_controller_carrier.INSERTS, 1):
        wx, wy, _ = V8._cc_world(x, y)
        insert_joint(A, ["s_panel_bottom", "fp9_controller_carrier"], (wx, wy, V8.FLOOR_Z), up, screw="M3x7",
                     name=f"j_controller_panel_{k}", module="electrical", evaluate=False, step=7)
    for k, (x, y) in enumerate(V8._cl42t_holes(), 1):
        at = np.array([x, y, V8.FLOOR_Z])
        seat = at + np.array([0, 0, V8.CL42T_EAR_T])
        # DL1: flipped, M4 button head from under the bottom panel (2.2, inside the feet's clearance), nut on the
        # driver ear inside the plinth; length = panel + ear + nut, rounded up (tail flush with the nut)
        L = int(math.ceil(seat[2] - V8.BOTP_Z0 + HW.HEX_NUT["M4"]["m"]))
        hex_nut_joint(A, ["s_panel_bottom", "v_cl42t_driver"], at, (0, 0, 1), screw=f"M4x{L}", head="button",
                      name=f"j_cl42t_panel_{k}", module="electrical", evaluate=False, step=7,
                      seat=(at[0], at[1], V8.BOTP_Z0), far=seat)
    for k, sx in enumerate((-1, 1), 1):
        tapped_joint(A, ["s_panel_back", "v_iec_inlet"], (V8.IEC[0] + sx * V8.IEC_PITCH / 2, V8.PANEL_IN, V8.PANEL_DEV_Z),
                     (0, -1, 0), screw="M3x6", name=f"j_iec_panel_{k}", module="electrical", evaluate=False, step=7)


# ------------------------------------------------------------------ balance, power, notes

TRIM_NOTE = "trim: 4 x M5 x 6 set screws, equal (flush in the 6 mm disc: ring B caps the holes, nothing below)"   # S1.24: the tapped holes are modelled empty


def _turning(A):
    # Rotor CG excludes the offset motor shaft/pulley, which turn on their own axis.
    return [(n, p) for n, p in A.parts.items()
            if (p.module == "rotor" or p.meta.get("spins"))
            and not any(p.meta.get("spin_origin", (0, 0, 0))[:2])]


def rotor_mass_props(A):
    """(CG xyz mm, mass kg, Izz kg m2, {part: (mass g, x, y)}) of everything that turns, from each body's volume and
    centroid times its sheet density (lib.materials)."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from lib import materials as M
    m_tot, mom, izz, per = 0.0, np.zeros(3), 0.0, {}
    for n, p in _turning(A):
        rho = M.MATERIALS[p.material]["density"] * 1e-6
        pm, pmom = 0.0, np.zeros(3)
        for sol in p.solid.solids():
            g = GProp_GProps()
            BRepGProp.VolumeProperties_s(sol.wrapped, g)
            c = g.CentreOfMass()
            vol = abs(g.Mass())
            m = vol * rho
            pm += m
            pmom += m * np.array([c.X(), c.Y(), c.Z()])
            izz += (abs(g.MatrixOfInertia().Value(3, 3)) + vol * (c.X() ** 2 + c.Y() ** 2)) * rho * 1e-6
        m_tot += pm
        mom += pmom
        if pm:
            per[n] = (round(pm * 1000, 2), round(pmom[0] / pm, 2), round(pmom[1] / pm, 2))
    return mom / m_tot, m_tot, izz, per


BOM_S1 = [  # part, qty, kind, unit price USD (vendor pages fetched 2026-10-11 / 12; laser parts and prints at material cost)
    ("lp_motor_plate 6061 6 x 64 x 120 (Fablight)", 1, "laser", 4.0),
    ("lp_plate_a 6061 6 x 120 x 120 with three roller arms to r 100", 1, "laser", 14.0),
    ("lp_carrier_disc 6061 6 x 186", 1, "laser", 22.0),
    (f"lp_ring_b 6061 3.175, {RING_B[0]:g} / {RING_B[1]:g} with two R 11 post lobes (Fablight)", 1, "laser", 3.0),
    ("lp_ring_top 6061 3.175, 147 / 186 bezel with two R 11 post lobes (Fablight)", 1, "laser", 3.0),
    ("fp9_tube_seat_bottom PETG", 1, "printed", 2.0),
    ("fp9_tube_seat_top PETG", 1, "printed", 1.5),
    ("clear RTV silicone, tube bead at both seats", 1, "vendor", 6.0),
    ("goBILDA 1614-0016-4008 dual-bearing pillow block", 1, "vendor", 11.99),
    ("goBILDA 1313-1632-4008 Hyper Hub 8 mm REX", 1, "vendor", 12.99),
    ("goBILDA 2106-4008-0720 REX shaft 8 x 72", 1, "vendor", 4.89),
    ("goBILDA 1522-0010-0060 spacer (4-pack, two used: race spacer + backup)", 1, "vendor", 3.49),
    (f"goBILDA 2807-0811-1000 shim (12-pack, {SHIM_N} used nominal, measure then choose)", 1, "vendor", 2.69),
    ("M4 x 8 button + ISO 7093 washer (budget allowance)", 2, "vendor", 0.3),
    ("goBILDA 3422-4008-0020 REX 20T GT2 pulley", 1, "vendor", 7.99),
    ("Olimex GT2-5T20-6MM 5 mm motor pulley (USD budget allowance)", 1, "vendor", 2.0),
    ("Olimex GT2-BELT100-6MM closed loop (USD budget allowance)", 1, "vendor", 2.0),
    ("STEPPERONLINE 17HS19-2004D-E1K closed-loop motor 52 Ncm, 1000 PPR", 1, "vendor", 19.7),
    ("RAF M0669-5-AL aluminium spacer 10 x 20, M5 clear", 4, "vendor", 1.2),
    ("Creality 4001070002 POM wheel kit with 2x625 bearings", 3, "vendor", 3.0),
    ("fp12_roller_block PETG", 3, "printed", 1.5),
    ("M5x25 SHCS + M5 DIN985 nyloc + two 5/10/1 precision shims", 3, "vendor", 1.6),
    ("Deck metal hardpoint spacer 5 ID / 10 OD / 3", 12, "vendor", .398),
    ("Anker 321 power bank A1112", 1, "vendor", 19.99),
    ("Raspberry Pi Zero 2 W", 1, "vendor", 15.0),
    ("Arducam UC-261 camera", 1, "vendor", 25.0),
    ("WS2812B 16 LED ring 45 OD, 5 V (Sparkleiot B0C77XPKSL 5 pack, 1 used; price assumed)", 1, "vendor", 11.99),
    ("Total Element 6 x 3 N42 disc magnet (120 pack, 2 used)", 1, "vendor", 24.79),
    ("Joy-it SEN-KY003HMS A3144 hall breakout", 1, "vendor", 3.5),
    ("fp9_hall_bracket PETG + M3 heat-set insert", 1, "printed", 0.6),
    ("M3 x 10 button (hall bracket) + M3 x 6 button (hall board)", 3, "vendor", 0.1),
    ("TRU Components LAS4GQH-11 12 mm latching pushbutton (Conrad 1661896)", 1, "vendor", 6.0),
    ("Adafruit 6069 round USB-C panel mount", 1, "vendor", 4.95),
    (TUBE_NAME, 1, "vendor", 18.0),
    ("acrylic top disc 186 x 3.2, 6 x 4.5 holes", 1, "laser", 4.0),
    ("M2.5 x 6 F-F hex standoff", 4, "vendor", 0.1),
    ("M5 x 6 trim set screw", 4, "vendor", 0.15),
    ("fp9_camera_wedge_bridge PETG", 1, "printed", 2.0),
    ("M5 x 40 socket + M5 nyloc (spacer bolts)", 4, "vendor", 0.4),
    ("M5 x 14 button + 2020 T-nut (cartridge to frame)", 4, "vendor", 0.3),
    ("M4 x 16 SHCS + M4 nyloc (roller blocks)", 6, "vendor", 0.15),
    ("M4 x 12 button (block) + M4 x 12 csk (hub)", 8, "vendor", 0.15),
    ("M3 x 10 SHCS + M3 washer (motor)", 4, "vendor", 0.1),
    ("M3 x 10 button + M3 heat-set insert (camera mount, from under the disc)", 4, "vendor", 0.15),
    ("M4 x 10 BHCS (bottom seat + ring B into the disc)", 6, "vendor", 0.12),
    ("M4 x 18 button + M4 hex nut (bezel + top disc + top seat)", 6, "vendor", 0.15),
    ("M2.5 x 10 SHCS", 4, "vendor", 0.08),
]

BOM_DRIVE = [  # the S1.26 cartridge only
    ("lp_plate_a 6061 6, 120 x 120 + three arms to r 100", 1, 14.0),
    ("lp_motor_plate 6061 6, 64 x 120", 1, 4.0),
    ("STEPPERONLINE 17HS19-2004D-E1K closed-loop motor 52 Ncm (73.64 oz.in), 1000 PPR", 1, 19.7),
    ("goBILDA 3422-4008-0020 REX 20T GT2 pulley", 1, 7.99),
    ("Olimex GT2-5T20-6MM 5 mm motor pulley (USD budget allowance)", 1, 2.0),
    ("Olimex GT2-BELT100-6MM closed loop (USD budget allowance)", 1, 2.0),
    ("goBILDA 1614-0016-4008 dual-bearing pillow block", 1, 11.99),
    ("goBILDA 2106-4008-0720 REX shaft 8 x 72", 1, 4.89),
    ("goBILDA 1522-0010-0060 spacer (4-pack, two used: race spacer + backup)", 1, 3.49),
    (f"goBILDA 2807-0811-1000 shim (12-pack, {SHIM_N} used nominal, measure then choose)", 1, 2.69),
    ("M4 x 8 button + ISO 7093 washer (budget allowance)", 2, 0.3),
    ("goBILDA 1313-1632-4008 Hyper Hub", 1, 12.99),
    ("RAF M0669-5-AL spacer 10 x 5.3 x 20", 4, 1.2),
    ("M5 x 40 socket head + M5 nyloc", 4, 0.4),
    ("M3 x 10 SHCS + M3 washer (motor)", 4, 0.1),
    ("M4 x 12 button head (block)", 4, 0.1),
    ("Creality 4001070002 POM wheel kit with 2x625 bearings", 3, 3.0),
    ("fp12_roller_block PETG", 3, 1.5),
    ("M5x25 SHCS + M5 DIN985 nyloc + two 5/10/1 precision shims", 3, 1.6),
    ("Plate A metal hardpoint spacer 5 ID / 10 OD / 3", 4, .398),
    ("M4 x 16 SHCS + M4 nyloc (blocks to plate A)", 6, 0.15),
    ("M5 x 14 button head + 2020 T-nut (cartridge to frame, the only four)", 4, 0.3),
]

# S1.25 stop profile inputs (vendor curve, vendor/motion/stepperonline_17hs19_2004d_e1k/source.json)
STOP_RPM = 400.0
T_CURVE_400 = 0.315          # N m: 31.5 N cm pull-out at 400 RPM, 24 V, 2.0 A (vendor torque curve)
TORQUE_MARGIN = 1.2          # the curve torque is 1.2 x the torque the decel limit asks for
J_MOTOR = 8.2e-6             # kg m2 motor rotor (82 g cm2, 17HS19 catalogue, assumed)
ROLLER_J = 0.013 * .01205**2   # conservative inertia upper bound treating entire 13 g kit as rim mass; not a speed qualification
# S1.27 / S1.28
DISC_TRACK_FLATNESS = 0.1    # mm over r 86..93, Fablight 6061 6 mm (assumed, ICD)
BLOCK_PLAY_PER_BEARING = 0.018   # mm radial: 688-class C0 internal clearance 0.010 + REX bore fit 0.008 (assumed)
TREAD_RUNOUT = 0.03          # unverified POM tread runout assumption; not acceptance evidence
WHEEL_LIMIT_RPM = None       # no rating established for supplied kit


def stop_profile(A):
    _, _, izz, _ = rotor_mass_props(A)
    ratio = ROLLER_R / (TYRE_OD / 2)
    j_tot = izz + J_MOTOR + 3 * ROLLER_J * ratio ** 2
    w = STOP_RPM * 2 * math.pi / 60
    t_used = T_CURVE_400 / TORQUE_MARGIN
    a = t_used / j_tot
    t = w / a
    revs = w * t / 2 / (2 * math.pi)
    return dict(torque_curve_nm=T_CURVE_400, torque_used_nm=t_used, decel_rad_s2=a, stop_time_s=t, stop_revs=revs,
                izz_kg_m2=izz, j_total_kg_m2=j_tot)


def bearing_wobble():
    """Disc-edge wobble (r 93): block alone = tilt from the radial play of both 1614 bearings over their 11 span; with
    the S1.27 rollers set touching, the disc plane is held by the three tread tops, so what is left is the track
    flatness plus tread runout, carried out to the edge."""
    lo, up = BLOCK_Z0 + 2.5, BLOCK_Z1 - 2.5
    span = up - lo
    block_only = (DISC_D / 2) * 2 * BLOCK_PLAY_PER_BEARING / span
    rollers = (DISC_TRACK_FLATNESS + TREAD_RUNOUT) * (DISC_D / 2) / ROLLER_R
    return dict(span=span, block_only=block_only, rollers=rollers)


def hall_sweep_clearance(A):
    """Clearance in the (r, Z) plane between the stationary hall parts (S1.31) and the envelope every turning part
    sweeps under the carrier disc. Points: tessellation vertices plus face samples (0.3 mm)."""
    def pts(solid, z_lo, z_hi):
        out = []
        for f in solid.faces():
            vs, _ = f.tessellate(0.05, 0.2)
            out.extend((v.X, v.Y, v.Z) for v in vs)
        a = np.array(out) if out else np.zeros((0, 3))
        return a[(a[:, 2] >= z_lo) & (a[:, 2] <= z_hi)] if len(a) else a
    z_lo, z_hi = PA_Z1, DISC_Z0 + 0.01
    rot = []
    for n, p in _turning(A):
        if n == "lp_carrier_disc" or A.bbox(n)[0][2] > DISC_Z0 - 0.01:
            continue
        q = pts(p.solid, z_lo, DISC_Z0 - 0.01)
        if len(q):
            rot.append((n, np.column_stack([np.hypot(q[:, 0], q[:, 1]), q[:, 2]])))
    res = {}
    for hn in ("v_index_hall_ky003", "fp9_hall_bracket"):
        h = pts(A[hn], z_lo, z_hi)
        hr = np.column_stack([np.hypot(h[:, 0], h[:, 1]), h[:, 2]])
        best = (1e9, None)
        for n, q in rot:
            d = np.sqrt(((hr[:, None, :] - q[None, :, :]) ** 2).sum(-1)).min() if len(q) * len(hr) < 4e7 else 1e9
            if d < best[0]:
                best = (float(d), n)
        res[hn] = (round(best[0], 2), best[1], round(float(DISC_Z0 - h[:, 2].max()), 2))
    return res


def spindle_fix_a(A):
    """Fix A (I033) measurements: BRepExtrema gap from each pillow-block screw to every rotating solid (each turned
    through 360 deg in 15 deg steps about its spin axis, the belt static), the REX pulley set-screw Z, the motor
    pulley grub on the D flat and the e-clip groove inside the backup spacer. Returns (meta dict, sheet note lines)."""
    from build123d import Axis
    spin = [n for n, q in A.parts.items() if q.meta.get("spins") or q.kind == "belt"]
    heads = {}
    for n, q in A.parts.items():
        if q.kind != "screw" or not str(q.meta.get("joint", "")).startswith("j_s1_5_"):
            continue
        (hx0, hy0, hz0), (hx1, hy1, hz1) = A.bbox(n)
        best = (1e9, None)
        for r in spin:
            (rx0, ry0, rz0), (rx1, ry1, rz1) = A.bbox(r)
            o = A.parts[r].meta.get("spin_origin", (0, 0, 0))
            rad = max(math.hypot(x - o[0], y - o[1]) for x in (rx0, rx1) for y in (ry0, ry1))
            dxy = math.hypot((hx0 + hx1) / 2 - o[0], (hy0 + hy1) / 2 - o[1]) - math.hypot(hx1 - hx0, hy1 - hy0) / 2
            if dxy - rad > 15.0 or rz0 - hz1 > 15.0 or hz0 - rz1 > 15.0:
                continue
            angs = [0.0] if A.parts[r].kind == "belt" else [15.0 * k for k in range(24)]
            ax = Axis((o[0], o[1], 0), (0, 0, 1))
            for a in angs:
                d = A[n].distance_to(A[r].rotate(ax, a) if a else A[r])
                if d < best[0]:
                    best = (float(d), r)
        heads[n] = dict(gap_mm=round(best[0], 3), nearest=best[1])
    rex = A[BD.REX_NAME].solids()
    grubs = [g for g in rex if g.volume < 60.0]
    grub_z = [round((g.bounding_box().min.Z + g.bounding_box().max.Z) / 2, 3) for g in grubs]
    grub_top = max(g.bounding_box().max.Z for g in grubs)
    mp = A[BD.MOTOR_NAME].solids()
    mgrub = mp[1].bounding_box()                                   # motor_pulley_grub_flat (bodies order)
    mgz = (mgrub.min.Z + mgrub.max.Z) / 2
    tip = A.bbox("v_nema17_shaft")[1][2]
    flat = (tip - 15.0, tip)
    mpb = A.bbox(BD.MOTOR_NAME)
    groove = (SHAFT_Z0 + 3.2, SHAFT_Z0 + 4.0)
    bsp = A.bbox(BD.LOWER_SPACER)
    meta = dict(shim_count_nominal=SHIM_N, shim_rule="measure then choose", shaft_proud_mm=SHAFT_PROUD,
                block_screw_head_gaps=heads, rex_set_screw_z=grub_z, rex_set_screw_hole_top_z=round(grub_top, 3),
                rex_set_screw_below_plate_a_mm=round(PA_Z0 - max(grub_z), 3), belt_z=BD.BELT_Z,
                motor_pulley_z=[round(mpb[0][2], 3), round(mpb[1][2], 3)], motor_grub_z=round(mgz, 3),
                motor_flat_z=[round(flat[0], 3), round(flat[1], 3)],
                motor_grub_flat_margin_mm=[round(mgz - 1.5 - flat[0], 3), round(flat[1] - (mgz + 1.5), 3)],
                eclip_groove_z=[round(groove[0], 3), round(groove[1], 3)],
                backup_spacer_z=[round(bsp[0][2], 3), round(bsp[1][2], 3)],
                backup_gap_to_pulley_mm=round(A.bbox(BD.REX_NAME)[0][2] - bsp[1][2], 3),
                motor_turned_deg=BD.MOTOR_ANGLE)
    gaps = ", ".join(f"{k.split('_j_')[1]} {v['gap_mm']:.3f} to {v['nearest']}" for k, v in heads.items())
    notes = [
        "spindle_fix_a=" + __import__("json").dumps(meta, sort_keys=True),
        f"shaft retention (fix A, I033): no 2-piece set-screw collar. Lower inner race: 1522 race spacer Z {RACE_SPACER_Z0:g}.."
        f"{BLOCK_Z0:g} through plate A's 16 hole (3.0 radial), shim_count_nominal={SHIM_N} x 2807 shim Z {SHIM_Z0:g}.."
        f"{RACE_SPACER_Z0:g}, REX pulley hub up Z {REX_TOP - REX_H:g}..{REX_TOP:g}; its two M5 set screws (axis Z "
        f"{REX_GRUB_Z:g}, {PA_Z0 - REX_GRUB_Z:g} under plate A's underside) are the lower axial stop. Shim rule: measure then "
        f"choose (hub 22.0, block 16.0, pulley 18.0, spacer 6.0, shim 1.0 nominal); keep the pulley hub top at least 1.0 "
        f"under the block screw heads and the shaft end {SHAFT_PROUD:g} (0.1..0.3) proud of the hub pilot",
        "block screws (S1.5): 4 x M4 x 12 ISO 7380 button head up from under plate A, NO washers on either face; "
        f"BRepExtrema head gap to the nearest rotating solid (swept 360 deg): {gaps}",
        f"remove the factory e-clip before assembly: the groove (Z {groove[0]:.1f}..{groove[1]:.1f}, 6.0 dia) sits inside "
        f"the lower 1522 backup spacer (Z {bsp[0][2]:.1f}..{bsp[1][2]:.1f}, 8.05 ID); a fitted 12.3 OD e-clip would stop it",
        "bench only: the hub pinch bolts (2 x M4, 3 mm key) and the REX pulley set screws (2 x M5, 2.5 key, from the side) "
        "are tightened on the bench before the cartridge goes into the deck; in the deck the 2020 cross and deck rails "
        "(Z -20..0) block every horizontal key approach at the set-screw plane Z -5 (ray test at 36 azimuths), so the "
        "pulley stop is bench-only",
        f"belt tension travel: motor slots give C {BD.CENTER + BD.TRAVEL[0]:g}..{BD.CENTER + BD.TRAVEL[1]:g}, i.e. -3 belt "
        "path to fit the belt and only +1.0 (C +0.5) to tension it; reject any belt from the pack that does not reach "
        "tension by C 30.3",
        f"motor turned {BD.MOTOR_ANGLE:g} deg about its axis: the lead glands face +Y, away from the lower end screw "
        f"(interface-risks 17a); motor pulley Z {mpb[0][2]:.2f}..{mpb[1][2]:.2f}, grub axis Z {mgz:.2f} on the D flat "
        f"Z {flat[0]:.2f}..{flat[1]:.2f}",
    ]
    return meta, notes


def s1_notes(A):
    A.meta_hall_clear = hall_sweep_clearance(A)
    cg, m, izz, per = rotor_mass_props(A)
    off = math.hypot(cg[0], cg[1])
    A.meta_rotor_cg = dict(x=round(float(cg[0]), 3), y=round(float(cg[1]), 3), z=round(float(cg[2]), 2), mass_kg=round(m, 4))
    A.meta_rotor = dict(izz_kg_m2=round(izz, 6), mass_kg=round(m, 3))
    A.meta_rotor_parts = per
    tot = PI_MA + CAM_MA + LED_MA
    watts = tot * 5.0 / 1000
    hours = BANK_WH * BANK_EFF / watts
    bom_total = sum(q * p for _, q, _, p in BOM_S1)
    sp = stop_profile(A)
    wb = bearing_wobble()
    roller_rpm = 600.0 * ROLLER_R / (TYRE_OD / 2)
    mot_bot = A.bbox(MOTOR)[0][2]
    drive_total = sum(q * p for _, q, p in BOM_DRIVE)
    steps = [
        f"Motor on lp_motor_plate at (0,30), turned {BD.MOTOR_ANGLE:g} deg so the lead glands face +Y, away from the spindle; four M3 x 10 + washers loosely in Y tension slots",
        f"20T 5 mm bore motor pulley directly on D shaft, hub up, first grub against the flat at Z {BD.BELT_Z + 8.0:g}, groove center Z {BD.BELT_Z:g}",
        "Pillow block on plate A: 4 x M4 x 12 button up from under plate A, no washers",
        "Three fp12_roller_blocks on plate A's arms: 2 x M4 x 16 down through each block and plate A, nylocs under; "
        "fp9_hall_bracket on plate A's 45 deg corner (2 x M3 x 10 button into plate A's tapped holes), the KY-003 hall "
        "board on its column face (M3 x 6 into the insert, bottom edge on the ledge)",
        f"72 mm REX shaft up through the block (factory e-clip removed, groove end down); from below: the 1522 race spacer up "
        f"through plate A onto the lower inner race, {SHIM_N} x 2807 shims (nominal, measure then choose)",
        "Loop the GT2 belt over the shaft below plate A BEFORE the REX pulley is slid on; the belt cannot pass over a pulley flange at any slot position (minimum centre distance for that is 26.6, the slot stops at 28.3)",
        "Then the REX pulley hub up through the belt loop against the shims, set screws snug only; lower 1522 backup spacer "
        "(never clamped), washer and M4 x 8 end screw",
        "Four M5 x 40 socket heads down through plate A, a RAF M0669-5-AL spacer on each from below",
        "Motor plate onto four spacer bolts with belt around both pulleys, four M5 nylocs; align grooves, tension motor in slots and tighten M3 screws on bench",
        f"Hyper Hub on the shaft top, shaft {SHAFT_PROUD:g} proud of the pilot (0.1..0.3); hub pinch bolts tightened on the "
        "bench, then push the REX pulley up against the shims and tighten its two M5 set screws on the bench (the lower "
        "axial stop): no axial play and no preload at the block; upper washer and M4 x 8 retainer on the shaft end only; "
        "bare carrier disc on hub "
        "(4 x M4 x 12 csk); spin by hand",
        "Bench note: after the pulley is set, feeler-check the gap between the backup 1522 spacer and the pulley bottom; "
        "if under 0.3 mm replace the backup 1522 with five 2807 shims",
        "Rim rollers: fit each assembled Creality wheel between two 1 mm precision shims on M5x25 screw, head outboard. Raise flat tread to just touch underside; hold height and tighten M5 nyloc; wheel must spin freely",
        "Carrier disc off the hub again (4 x M4 csk), the rollers stay locked",
        "Cartridge onto the deck: the motor plate, motor and spacers go down through the 66 x 126 opening, the six "
        "block nylocs into their 12 holes, plate A onto the deck sheet",
        "Four M5 x 14 button heads down through plate A and the sheet into the cross-rail T-nuts (the only "
        "cartridge-to-frame fasteners)",
        "Populated carrier disc on the hub (4 x M4 x 12 csk); repeat the paper-drag test at the three rollers",
    ]
    A.meta_install_s1 = steps
    return [
        f"rotor balance: rotor_cg_offset_mm={off:.2f} (CG x {cg[0]:.2f}, y {cg[1]:.2f}, z {cg[2]:.1f}; turning mass "
        f"{m:.3f} kg, Izz {izz:.5f} kg m2) by layout alone: the bank lies flat under the camera bridge, the Pi at +Y "
        f"(the -Y LED driver is gone, I034), the switch at ({SWITCH_XY[0]:g}, {SWITCH_XY[1]:g}) and the port at "
        f"({PORT_XY[0]:g}, {PORT_XY[1]:g}) sit inside the tube, and the bank + bridge centre sits at ({BANK_OFF[0]:g}, "
        f"{BANK_OFF[1]:g}) so their moment squares what the Pi, port, switch, frogs and camera leave; no balance parts",
        TRIM_NOTE + f" in the tapped holes at r {TRIM_R:g}, {'/'.join(f'{a:g}' for a in TRIM_ANG)} deg (S1.24); "
        "modelled empty, fitted only after a spin test",
        f"rotor_power_budget={PI_MA} mA (Pi Zero 2 W, Wi-Fi streaming) + {CAM_MA} mA (UC-261 camera) + {LED_MA} mA "
        f"(the WS2812B 16 LED white ring straight on 5 V, {LEDR_MA_5V:g} mA allowance with the brightness capped "
        f"near 30 %; it replaces the old 850 nm ring's 150 mA at 12 V allowance) = {tot} mA at 5 V ({watts:.2f} W), {BANK_WH:.2f} Wh bank => "
        f"{hours:.1f} h (assumes {BANK_EFF * 100:.0f} % cell-to-5 V conversion in the bank)",
        "bom_s1=" + "; ".join(f"{n} | {q} | {k} | ${p:.2f}" for n, q, k, p in BOM_S1) +
        f"; total ${bom_total:.2f}",
        f"motor: STEPPERONLINE 17HS19-2004D-E1K closed-loop NEMA 17, 52 Ncm (73.64 oz.in) holding, 2.0 A, 1000 PPR "
        f"encoder (Amazon B0C8984QFM; vendor STEP, drawing and torque curve in {MOTOR_DIR}); 67 +-1 long with the "
        f"encoder on the drawing, 68.5 over the rear screws in the STEP; face Z {MOTOR_FACE_Z:g}, bottom Z {mot_bot:.1f} "
        f"({mot_bot - BOTP_Z1_V8:+.1f} to the bottom panel top at Z {BOTP_Z1_V8:g}); shaft 5 x 24 uncut with a 15 D-cut, "
        f"directly into the 5 mm bore 20T motor pulley; its grub bears on the D flat, motor axis (0,30), body rotated {BD.MOTOR_ANGLE:g} deg (lead glands +Y)",
        f"motor_only_stop_estimate_NOT_belt_qualified: torque_used_nm={sp['torque_used_nm']:.4f} decel_rad_s2={sp['decel_rad_s2']:.1f} "
        f"stop_time_s={sp['stop_time_s']:.3f} stop_revs={sp['stop_revs']:.2f} izz_kg_m2={sp['izz_kg_m2']:.5f} "
        f"(from {STOP_RPM:g} RPM; vendor curve {T_CURVE_400 * 100:.1f} N cm at 400 RPM, 24 V, 2.0 A, divided by "
        f"{TORQUE_MARGIN:g} for the 20 % margin; inertia {sp['j_total_kg_m2']:.5f} kg m2 = rotor Izz from the sheet + "
        f"motor rotor {J_MOTOR:g} + three rim wheels reflected at ({ROLLER_R:g} / {TYRE_OD / 2:g})^2; constant decel, "
        "the cage S3 is the catch during the stop, the motor is not a brake)",
        f"bearing_play_wobble_mm={wb['rollers']:.2f} at the disc edge (r {DISC_D / 2:g}) with the S1.27 rim rollers set "
        f"touching: track flatness {DISC_TRACK_FLATNESS} + tread runout {TREAD_RUNOUT} (assumed) carried to the edge; "
        f"the 1614 block alone would give {wb['block_only']:.2f} (radial play {BLOCK_PLAY_PER_BEARING} per bearing "
        f"assumed: C0 internal clearance 0.010 + REX bore fit 0.008, over the {wb['span']:g} bearing-centre span)",
        f"rim rollers (S1.27): 3 x Creality standard Ender wheel at r {ROLLER_R:g}, {'/'.join(f'{a:g}' for a in ROLLER_ANG)} deg, "
        f"axle Z {ROLLER_Z:g}, tread top Z {ROLLER_Z + TYRE_OD / 2:g} = disc underside (gap 0); "
        f"roller_rpm_at_600={roller_rpm:.0f} (disc surface at r {ROLLER_R:g} over the {TYRE_OD / 2:g} tread radius) "
        "rpm is kinematic only; no loaded speed approved for the supplied bearings or POM contact; "
        f"disc_track_flatness_mm={DISC_TRACK_FLATNESS} (Fablight 6061 6 mm, assumed); running band r 85.95..92.05 under the "
        "disc kept empty (S1.28): the six S1.9 M4 x 10 end 0.4 short of the disc underside, the switch and port moved "
        "inside the tube, the magnets sit flush",
        "install_order_s1=" + " ".join(f"{i}. {t}." for i, t in enumerate(steps, 1)),
        "bom_drive=" + "; ".join(f"{n} x {q} at ${p:.2f}" for n, q, p in BOM_DRIVE) + f"; cartridge total ${drive_total:.2f}",
        f"S1 cartridge stack (Z): motor bottom {mot_bot:.1f}, motor face {MOTOR_FACE_Z:g}, motor plate {MPLATE_Z0:g}.."
        f"{MPLATE_Z1:g}, spacers {MPLATE_Z1:g}..{PA_Z0:g}, belt center Z {BD.BELT_Z:g} (offset motor shaft tip "
        f"{MOTOR_SHAFT_TIP:.2f}, REX shaft end {SHAFT_Z0:.2f}), plate A {PA_Z0:g}..{PA_Z1:g}, REX pulley "
        f"{REX_TOP - REX_H:g}..{REX_TOP:g} (set screws Z {REX_GRUB_Z:g}), {SHIM_N} shims {SHIM_Z0:g}..{RACE_SPACER_Z0:g}, "
        f"race spacer {RACE_SPACER_Z0:g}..{BLOCK_Z0:g}, block {BLOCK_Z0:g}..{BLOCK_Z1:g} (bearing centres 11 apart), REX shaft "
        f"{SHAFT_Z0:g}..{SHAFT_Z1:g}, hub {HUB_BOSS_Z:g}..{HUB_FLANGE_TOP + HUB_PILOT_H:g}, disc {DISC_Z0:g}..{DISC_Z1:g} "
        f"= drum floor, rim rollers axle {ROLLER_Z:g}, tube top {TUBE_TOP:g}",
        "V12 belt qualification pending: 100 mm loop, 6 mm wide, 20/20 teeth, 30 mm centers. "
        "Belt working tension, motor radial bearing limit and actual pulley dimensions must be verified; "
        "do not treat the inherited motor-only stopping calculation as a belt-drive rating.",
        "S1 against the ICD: spacer bolts M5 x 40, with 32 mm grip (6 + 20 + 6), then the 5 mm nyloc; "
        "roller blocks hold with M4 x 16 + nyloc, and the nylocs hang below plate A through six 12 holes in the deck "
        "sheet; the S1.27 axle is an ISO 7379 M5 x 25 socket screw with an M6 thread, head outboard and the nyloc "
        "inboard (outboard, its tip would reach the net sleeve); deck opening 126 long leaves 3 open at each Y end of "
        "plate A; motor-plate spacer holes 11 from the long edges (64 wide at X +-21); bank long axis along X, bridge screws at "
        "(+-34, +-27.2) about the bank centre, up from under the disc into inserts; lens 48 above the disc (I028); hub csk M4 x 12",
        f"camera: Arducam UC-261 class on fp9_camera_wedge_bridge, optical axis 30 deg to +X, lens front {LENS_ABOVE_FLOOR:g} "
        f"above the floor at Z {LENS[2]:.2f}; M2 hole pitch {CAM_HOLE_PITCH} assumed",
        f"drum stack D1 (S1.9..S1.13): lp_ring_b {RING_B[0]:g} / {RING_B[1]:g} x {RING_T} Z {DRUM_FLOOR:g}..{RING_B_Z1:.3f}; "
        f"fp9_tube_seat_bottom r {SEAT_CH_R[1]:g}..{SEAT_B_OUT_R:g} Z {SEAT_B_Z0:.3f}..{SEAT_B_Z1:.3f}, bore "
        f"{2 * SEAT_CH_R[1]:.1f}, a 6 deep socket round the tube foot (the tube stands on ring B); s_drum_tube "
        f"{TUBE_OD:.1f} OD x {TUBE_WALL:g} wall (bore {2 * TUBE_IR:.1f}) x {TUBE_H:g}, Rebower 145 x 150 x 14 in (I036), "
        f"Z {TUBE_FOOT:.3f}..{TUBE_TOP:.3f}; "
        f"fp9_tube_seat_top r {SEAT_CH_R[1]:g}..{TOP_DISC_D / 2:g} bore {2 * SEAT_CH_R[1]:.1f} Z {SEAT_T_Z0:.3f}..{TUBE_TOP:.3f}; s_top_disc "
        f"{TOP_DISC_D:g} x {TOP_DISC_T:g} Z {TUBE_TOP:.3f}..{TUBE_TOP + TOP_DISC_T:.3f}; lp_ring_top {RING_TOP[0]:g} / "
        f"{RING_TOP[1]:g} to Z {TUBE_TOP + TOP_DISC_T + RING_T:.3f}; screw circle r {STACK_R:g} at "
        f"{'/'.join(f'{a:g}' for a in STACK_ANG)} deg: bottom 6 x {STACK_SCREW_B} BHCS in the seat counterbores through "
        f"ring B into the disc threads (5.6 engaged, tip 0.4 short of the underside), top 6 x {STACK_SCREW_T} button "
        f"through bezel, disc and seat into M4 hex nuts under the seat (outside the tube at r {STACK_R - 4.04:.1f}..{STACK_R + 4.04:.1f}); "
        f"wall band Z {SEAT_B_Z1:.3f}..{SEAT_T_Z0:.3f} ({SEAT_T_Z0 - SEAT_B_Z1:.1f}); rotor top {TUBE_TOP + TOP_DISC_T + RING_T:.2f} "
        f"(+{TUBE_TOP + TOP_DISC_T + RING_T - 157.375:.2f} on the C/E/D stack); drum posts (S1.32): 2 x M5 F-F round "
        f"aluminium standoff {POST_D:g} x {POST_L:g} at r {POST_R:g}, {'/'.join(f'{a:g}' for a in POST_ANG)} deg, Z "
        f"{RING_B_Z1:.3f}..{RING_B_Z1 + POST_L:.3f}, INSIDE the tube (I014, body r {POST_R - POST_D / 2:g}..{POST_R + POST_D / 2:g}, "
        f"{TUBE_IR - POST_R - POST_D / 2:g} clear of the bore r {TUBE_IR:g}), on R {POST_LOBE_R:g} lobes of ring B (M5 x 16 stud into the "
        f"tapped disc, nothing below it, the stud in the 5.5 lobe hole locates the foot; nothing printed touches the "
        f"posts), and under the top disc ({POST_SCREW} button through the bezel lobe and the disc); load path: hub -> carrier disc -> "
        "ring B (aluminium on aluminium, six steel M4 in shear) -> tube foot and the two posts -> top disc and bezel; "
        "the posts carry the top ring, the silicone in both seats is a seal, the PETG seats only locate the tube; "
        "nothing printed carries rotor load",
        f"led_ring (S1.29): WS2812B 16 LED white ring, Adafruit NeoPixel Ring 16 vendor STEP ({LEDR_OD:g} OD / "
        f"{LEDR_ID:g} hole x {LEDR_T:g}, {LEDR_N} x 5050 x {LEDR_PKG_H:g} on r 19), 5 V, coaxial with the lens, board "
        f"underside {LEDR_Z:g} in front of the camera board underside, package tops "
        f"{LEDR_Z + LEDR_T + LEDR_PKG_H - CAM_STACK:+.1f} from the lens front plane, flush in the bridge collar's "
        f"{LEDR_RECESS_D:g} x {LEDR_RECESS_DEPTH:g} recess ({COLLAR_OUT:g} square collar) with 3 silicone dots; 3 wires "
        "(5 V, GND, DIN) to the Pi: Pi 5 V and GND, DIN from GPIO18 through a 74AHCT125 level shifter + 330 ohm series "
        "resistor, 1000 uF across 5 V / GND at the ring, on the Pi side (wiring note, not modelled); no LED driver board "
        "(I034); replaces the Spinel 850 nm infrared ring (I032)",
        "hall_index (S1.30, S1.31): 2 x 6 x 3 N42 at r %g, %s deg (flush in 6.1 through holes, capped by ring B); KY-003 "
        "(A3144) sensing face %g under the disc at r %g, %g deg; sweep clearance (r, Z) to the turning parts: %s; r 76 "
        "instead of r 70: at r 70 the sensor would sit inside the swept switch head (to r 69.5, Z 41.2) and port flange "
        "(to r 72.7, Z 42.5)" % (MAG_R, "/".join(f"{a:g}" for a in MAG_ANG), HALL_GAP, SENSOR_U, HALL_AZ,
                                  "; ".join(f"{k} {v[0]} to {v[1]} (top {v[2]} under the disc)"
                                            for k, v in A.meta_hall_clear.items())),
    ]


def camera_pov():
    ax = np.array(OPTICAL_AXIS, float)
    ax /= np.linalg.norm(ax)
    return dict(eye=tuple(float(x) for x in LENS), axis=tuple(float(x) for x in ax), fov=120.0,
                hide=("v_camera_uc261", "fp9_camera_wedge_bridge"), closeup_focus=(60.0, 0.0, 205.0 + DZF + TUBE_DZ))


def frog_report(A):
    out = {}
    rw, tw = _frog_wall_pose()
    w = _wall_frog_verts() @ rw.T + tw
    rd, td = _frog_disc_pose()
    d = V7._frog_verts() @ rd.T + td
    r = np.hypot(w[:, 0], w[:, 1])
    # I030: the net is a square of flat panels, so the nearest net to a point is its larger |x|, |y|
    out["gap_net"] = round(float(CAGE_FACE - CG.MESH_T - np.maximum(np.abs(w[:, 0]), np.abs(w[:, 1])).max()), 2)
    out["gap_mesh_top"] = round(float(CAGE_TOP - CG.MESH_T - d[:, 2].max()), 2)
    out["gap_tube"] = round(float(r.min() - TUBE_OR), 2)
    out["gap_seat_top"] = round(float(SEAT_T_Z0 - w[:, 2].max()), 2)
    out["gap_seat_bottom"] = round(float(w[:, 2].min() - SEAT_B_Z1), 2)
    out["gap_ring_top"] = round(float(RING_TOP[0] / 2 - np.hypot(d[:, 0], d[:, 1]).max()), 2)
    assert out["gap_net"] >= 8.0, "wall frog too close to the cage net panels"
    assert out["gap_mesh_top"] >= 10.0, "disc frog too close to the net top"
    assert out["gap_seat_top"] >= V7.FROG_RING_GAP, "wall frog too close to the top seat"
    return out


def _hub_screw_envelope():
    """ISO10642 M4x12, supplier maximum envelope (Accu SSK-M4-12-A2).
    The old library used an8mm nominal head. This version uses8.96 theoretical
    diameter and2.48 head depth. Threads are represented at root diameter.
    Actual supplier head tolerance means countersinks must be finished flush to supplied screws.
    """
    from build123d import Cone, RegularPolygon
    head=Cone(2,4.48,2.48,align=(Align.CENTER,Align.CENTER,Align.MIN)).moved(Location((0,0,-2.48)))
    root=Cylinder(1.621113885844308,12,align=(Align.CENTER,Align.CENTER,Align.MAX))
    socket=extrude(RegularPolygon(1.25,6,major_radius=False),amount=1.5).moved(Location((0,0,-1.4)))
    return _solid(head+root-socket)


def _carrier_hub_screws(A):
    for j in A.joints:
        if not j['name'].startswith('j_s1_8_hub_disc_'):continue
        name=j['screw_part'];part=A.parts[name]
        part.solid=_hub_screw_envelope().moved(Location(tuple(j['seat'])))
        part.solid.label=name
        part.meta['note']='ISO10642 maximum8.96mm theoretical head envelope,90deg,2.48mm head depth; Accu SSK-M4-12-A2. Finish countersinks flush to actual supplied heads.'
        A.__dict__.get('_bbox_cache',{}).pop(name,None)
        A.vendor.append(dict(component=name,source_url='https://www.accu.co.uk/countersunk-socket-head-screws/5448-SSK-M4-12-A2',author='Accu dimensions; conservative envelope',file='assemblies/froggy_v12.py:_hub_screw_envelope',licence=None,price_usd=None))


def _drive_locknuts(A):
    """Use 5 mm-high DIN985 envelopes for support/roller mounting nuts.
    Original joint checker uses physical nut extent for engagement and protrusion.
    Nylon insert internal threading is simplified; maximum exterior hex envelope used.
    """
    from build123d import RegularPolygon
    for j in A.joints:
        if not j['name'].startswith(('j_s1_1_spacer_', 'j_s1_27_block_')):
            continue
        oldname=j['nut_part'];old=A.parts.pop(oldname)
        bb=old.solid.bounding_box();x=(bb.min.X+bb.max.X)/2;y=(bb.min.Y+bb.max.Y)/2;z=bb.max.Z
        d=5 if j['thread']=='M5' else 4;af=8 if d==5 else 7
        shape=extrude(RegularPolygon(af/2,6,major_radius=False),amount=5)
        shape-=Cylinder(HW.HEX_NUT[j["thread"]]["bore"]/2,7,align=(Align.CENTER,Align.CENTER,Align.MIN)).moved(Location((0,0,-1)))
        name=oldname.replace('hw_hex_nut_','hw_nyloc_')
        A.add(name,_solid(shape.moved(Location((x,y,z-5)))), 'nut',old.module,**{k:v for k,v in old.meta.items() if k!='module_tint'})
        j['nut_part']=name
        for jj in A.joints:
            if 'bench' in jj:jj['bench']=[name if n==oldname else n for n in jj['bench']]
        A.__dict__.get('_bbox_cache',{}).pop(oldname,None)


def build():
    A = Assembly("froggy_v12")
    R_ = _frame(A)
    _brackets(A, R_)
    _drive(A)
    _hall(A)
    _rotor(A)
    _panels(A, R_)
    _feet(A, R_)
    _clamp_tabs(A, R_)
    _electrics(A)
    _cage(A, R_)
    A.meta_vent_clear = V8._vent_clearance(A)
    _s1_joints(A, R_)
    _drive_locknuts(A)
    _carrier_hub_screws(A)
    BD.retainer_joints(A)
    _plinth_joints(A)
    n_br = len([n for n in A.names("bracket") if A.parts[n].module == "frame"])
    n_rail = len([n for n in A.names("rail") if A.parts[n].module == "frame"])
    A.notes.append(f"frame (DL29, I020 / I021 2026-10-16, I029 2026-10-21): 2020 = {n_rail} rails (deck square 300 x 260 at "
                   f"Z -20..0, the two cross rails, four legs 2020 x {LEG_L:g} at (+-{LEG_XY:g}, +-{LEG_XY:g}) Z "
                   f"{LEG_BOT:g}..{LEG_TOP:g}), {n_br} cast corner brackets (one per leg under the deck X rail), M5 button "
                   f"head + 2020 T-nut; base square removed. Load path: deck rails, brackets, legs, M5 x 16 foot bolts in "
                   f"the leg end taps ({LEG_TAP_DEPTH:g} deep, assumed), rubber feet. Plinth shell (skin): four 3 mm acrylic "
                   f"panels hung from the deck rails (2 x M5 each) and the legs' outer slots (2 x M5 per leg edge at Z "
                   f"{LEG_PANEL_Z[0]:g} / {LEG_PANEL_Z[1]:g}), bottom panel Z {V8.BOTP_Z0:g}..{V8.BOTP_Z1:g} (plinth "
                   f"electrics on it at Z {V8.FLOOR_Z:g}) clamped under the leg ends by the foot bolts; cage posts "
                   f"4 x 6061 tube {CG.TUBE_OD:g} x {CG.TUBE_WALL:g} x {CAGE_POST_L:g} (I022, I030) at (+-{CAGE_P:g}, +-{CAGE_P:g}) "
                   f"in printed foot corners")
    A.notes.append(f"clamp tabs (S2.13, I023): 4 x lp_clamp_tab 6061 {CT_T:g} on the deck sheet over the deck rail corners, "
                   f"Z {CT_Z0:g}..{CT_Z0 + CT_T:g}, 2 x M5 x 14 into the deck rail T-nuts each, pad square x, y "
                   f"{CT_HOLE + 5:g}..{CT_EDGE:g} free of heads; assumed clamp jaw: reach {CLAMP_JAW['reach']:g} mm, "
                   f"thickness {CLAMP_JAW['thick']:g} mm, pad {CLAMP_JAW['pad']:g} mm, opening at least "
                   f"{CLAMP_JAW['opening_min']:g} mm (F clamp or wood quick clamp, bar outside the plinth corner, the "
                   f"lower jaw under a bench top up to 40 thick)")
    A.notes.extend(("UNQUALIFIED ANALYTICAL ESTIMATE: " + n) if ("stop_" in n or "bearing_play_wobble_mm=" in n) else n for n in s1_notes(A))
    A.meta_spindle_fix_a, _fa_notes = spindle_fix_a(A)
    A.notes.extend(_fa_notes)
    A.notes.append("V12: standard POM wheel flat tread supports disc underside. Motor family confirmed 17HS19; encoder suffix and actual dimensions still require incoming check. Deck/plate clamp load bypasses acrylic via twelve stock aluminum spacers. See docs/ICD-V12.md.")
    A.meta_belt_drive = BD.report(A)
    A.notes.append("belt_drive_v11=" + __import__("json").dumps(A.meta_belt_drive, sort_keys=True))
    cr = A["fp9_estop_cradle"]
    A.meta_es_clear = min(cr.distance_to(A[n]) for n, prt in A.parts.items()
                          if prt.module == "drive_base" and A.bbox(n)[0][2] < SHEET_Z1)
    A.notes.append(
        f"estop (S2.10): BACO LBX10510 IP69K one-hole 22 mm control station, 40 mm red push-turn-reset mushroom, 1 NC "
        f"(Nema Supply SKU LBX10510, $66.99; the lab has one), all 26 solids of BACO's own STEP (vendor/electrical/"
        f"baco_lbx10510/lbx10510.stp), recessed (option A): the 74 x 74 x 47.9 box passes through a {ES_OPEN:g} x "
        f"{ES_OPEN:g} R {ES_OPEN_R:g} opening in the front panel centred at X {ES_XC:g}, Z {ES_ZC:g} (Z "
        f"{ES_ZC - ES_W / 2:g}..{ES_ZC + ES_W / 2:g}, 3 clear of the deck rail and the bottom ring), yellow cover face "
        f"{A.meta_es_proud:.2f} outside the panel's outer face (Y {ES_FACE_Y:g}), so the 4 cover screws stay reachable "
        f"and the cover lifts off from outside; only the 40 mm head stands further out. The station is turned 90 deg "
        f"about the panel normal so its M16/20 knockouts face +-X (upright, the bottom knockout would open onto the "
        f"bottom ring 3 below the box): the cable leaves through the -X knockout and the cradle's 26 x 26 window into "
        f"the plinth. fp9_estop_cradle (PETG) holds it: flange on the panel's inner face, 4 x M3 x 10 button head from "
        f"outside through 3.4 panel holes into heat-set inserts, the flange edge 0.8 inside the opening and 0.2 off the "
        f"box (closes the 1 mm gap from behind; the station keeps its own IP rating), back gussets under the box's 2 "
        f"base slots at (X, Z) " + ", ".join(f"({x:g}, {z:g})" for x, z in ES_HOLES)
        + f" with 2 x M4 x 12 button head from inside the base (cover off, on the bench) + M4 nylocs; cradle back "
        f"face Y {ES_BACK_Y + fp9_estop_cradle.BACK_T:.1f}, {A.meta_es_clear:.1f} in front of the nearest drive "
        f"cartridge part, so the station stays centred")
    A.notes.append("bom_s2_estop=BACO LBX10510 control station e-stop | 1 | vendor | $66.99; fp9_estop_cradle PETG | 1 | "
                   "printed | $1.50; M3 x 10 button head + M3 heat-set insert | 4 | vendor | $0.20; M4 x 12 button head + "
                   "M4 nyloc | 2 | vendor | $0.15; total $69.59")
    A.notes.extend(s3_notes(A))
    A.notes.append(f"frog gaps (mm): {frog_report(A)}")
    A.notes.append("vents: clearance through the sheet + 3: " + ", ".join(
        f"{k} {g} ({n})" for k, (g, n) in A.meta_vent_clear.items()))
    A.camera_pov = camera_pov()
    A.cage_lift = 150.0
    A.rotor_top = dict(z_cut=round(TUBE_TOP - 4.175, 3), z_min=DISC_Z0 - 1.0, r_max=135.0, hide=("s_drum_tube",))
    _drive_page(A)
    return A


def _drive_page(A):
    """Render specs for the S1.26 drive-base page (lib.render._drive_renders): drive_exploded, drive_section and
    drive_install."""
    A.drive_exploded_caption = "V12 drive cartridge: offset motor, slotted plate, 20T pulleys, 100 mm belt, retained REX spindle and hub"
    A.motor_section_normal = (1, 0, 0)
    A.motor_section_caption = "section_motor: V12 YZ through both shaft axes, belt, bearings, hub and rotor; cut faces darker"
    A.drive_section_caption = f"V12 YZ section through both shaft axes; belt center Z {BD.BELT_Z:g}; retained pillow-block bearings in magenta"
    A.drive_detail_views = [
        dict(name="drive_belt_detail", parts=[*BD.RETAINER_NAMES, MOTOR, "v_nema17_shaft", "lp_motor_plate", *BD.PART_NAMES,
             "v_gobilda_rex_shaft_2106_72", "v_gobilda_1614_pillow_block", "v_gobilda_1614_bearing_lower",
             "v_gobilda_1614_bearing_upper", "v_gobilda_1313_hub"],
             caption="V12 belt drive: 100 mm loop x 6 mm, 20T/20T, 30 mm centers; 72 mm REX spindle with end retainers",
             camera=[(220,-180,-80),(0,12,-10),(0,0,1)]),
        dict(name="drive_shafts_exposed", parts=[*BD.RETAINER_NAMES, "v_nema17_shaft", "v_gobilda_rex_shaft_2106_72", *BD.PART_NAMES],
             caption="V12 shaft geometry: 5 mm D motor shaft, complete 8 mm REX x 72, toothed pulleys and closed belt; supports hidden",
             camera=[(180,-160,80),(0,10,5),(0,0,1)]),
    ]
    def hw(prefix, kinds=None):
        return [n for n, p in A.parts.items() if str(p.meta.get("joint", "")).startswith(prefix)
                and (kinds is None or p.kind in kinds)]
    for view in A.drive_detail_views:
        view["parts"].extend(hw("j_s1_6_"))
    cart = drive_cartridge(A)
    A.drive_cartridge = cart
    # exploded along Z, bottom to top (dx, dy, dz); the rim rollers also move out radially
    ex = {}
    for n in (MOTOR, "v_nema17_shaft"):
        ex[n] = (0, 0, -170.0)
    for n in ["lp_motor_plate"] + hw("j_s1_1_", ("nut",)):
        ex[n] = (0, 0, -120.0)
    for n in hw("j_s1_2_"):
        ex[n] = (0, 0, -92.0)
    for name in BD.PART_NAMES:
        ex[name] = (0, 0, -70.0)
    for n in SPACER_NAMES:
        ex[n] = (0, 0, -38.0)
    for n in hw("j_s1_5_"):
        ex[n] = (0, 0, -16.0)
    for n in ["lp_plate_a"] + RBLOCK_NAMES + hw("j_s1_27_block"):
        ex[n] = (0, 0, 0.0)
    for n in ("v_gobilda_1614_pillow_block", "v_gobilda_1614_bearing_lower", "v_gobilda_1614_bearing_upper"):
        ex[n] = (0, 0, 45.0)
    ex["v_gobilda_rex_shaft_2106_72"] = (0, 0, 85.0)
    for n in hw("j_s1_1_", ("screw",)):
        ex[n] = (0, 0, 115.0)
    ex["v_gobilda_1313_hub"] = (0, 0, 135.0)
    for k, az in enumerate(ROLLER_ANG):
        c, s_ = math.cos(math.radians(az)), math.sin(math.radians(az))
        for n in [ROLLER_NAMES[k], AXLE_SPACER_NAMES[k]] + hw(f"j_s1_27_axle_{k + 1}"):
            ex[n] = (c * 45.0, s_ * 45.0, 30.0)
    for n in BD.RETAINER_NAMES + tuple(hw("j_s1_6_")):
        ex[n] = (0, 0, 175.0 if ("upper" in n) else -90.0)
    A.drive_explode = ex
    # install montage: one frame per install_order_s1 step (added, removed, context)
    rails = [n for n, r in A.meta_role.items() if r in ("cross_right", "cross_left", "deck_x_back", "deck_x_front",
                                                         "deck_y_right", "deck_y_left")]
    deck = ["s_panel_deck"] + rails
    short = ["Motor on the motor plate (4 x M3 x 10 + washer)", "20T pulley directly on motor D shaft",
             "Pillow block on plate A (4 x M4 x 12 from below)", "Roller blocks (2 x M4 x 16 + nyloc each) + hall bracket",
             "REX shaft, race spacer + shims", "Belt looped over the shaft BEFORE the REX pulley",
             "REX pulley through the belt loop, lower retainer", "Four M5 x 40 down through plate A + four 20 spacers",
             "Plate onto bolts, align pulleys, tension motor, 4 nylocs", "Hub, pulley set-screw stop, upper retainer, bare disc",
             "Feeler-check backup 1522 to pulley gap (>= 0.3 or five 2807 shims)",
             "Rim rollers in their slots, raised to touch the disc, locked", "Carrier disc off again (rollers stay set)",
             "Cartridge down through the deck opening onto the sheet", "4 x M5 x 14 into the cross-rail T-nuts",
             "Populated carrier disc on the hub, paper-drag check"]
    rotor_top = ["lp_carrier_disc", "lp_ring_b"] + hw("j_s1_8_") + hw("j_s1_9_") + \
        ["fp9_tube_seat_bottom", "v_index_magnet_6x3_n42_1", "v_index_magnet_6x3_n42_2", "v_charge_port", "v_power_switch"]
    frames = [
        (["lp_motor_plate", MOTOR, "v_nema17_shaft"] + hw("j_s1_2_"), [], []),
        ([BD.MOTOR_NAME], [], []),
        (["lp_plate_a", "v_gobilda_1614_pillow_block", "v_gobilda_1614_bearing_lower", "v_gobilda_1614_bearing_upper"]
         + hw("j_s1_5_"), [], []),
        (RBLOCK_NAMES + hw("j_s1_27_block") + ["fp9_hall_bracket", "v_index_hall_ky003"] + hw("j_s1_31_"), [], []),
        ([BD.SHAFT_NAME, RACE_SPACER, *RACE_SHIMS], [], []),
        ([BD.BELT_NAME], [], []),
        ([BD.REX_NAME, BD.LOWER_SPACER, BD.LOWER_WASHER] + hw("j_s1_6_retainer_lower"), [], []),
        (SPACER_NAMES + hw("j_s1_1_", ("screw",)), [], []),
        (hw("j_s1_1_", ("nut",)), [], []),
        (["v_gobilda_1313_hub", BD.UPPER_WASHER, "lp_carrier_disc"] + hw("j_s1_6_retainer_upper"), [], []),
        ([], [], []),
        (ROLLER_NAMES + AXLE_SPACER_NAMES + hw("j_s1_27_axle"), [], []),
        ([], ["lp_carrier_disc"], []),
        ([], [], deck),
        (hw("j_s1_4_"), [], deck),
        (rotor_top, [], deck),
    ]
    assert len(frames) == len(short) == len(A.meta_install_s1)
    A.drive_install = [(f"{i}. {t}", a, r, c) for i, (t, (a, r, c)) in enumerate(zip(short, frames), 1)]
    # YZ section through both shaft axes; anchors (Y, Z)
    az = 90.0  # V12 section contains both parallel shaft axes
    u = (math.cos(math.radians(az)), math.sin(math.radians(az)), 0.0)
    A.drive_section = dict(
        u=u, normal=(u[1], -u[0], 0.0), z_range=(-110.0, 60.0),
        pick=cart + ["lp_carrier_disc", "lp_ring_b", "s_panel_deck"] + [n for n in rails if A.meta_role[n].startswith("cross")]
        + hw("j_s1_4_") + hw("j_s1_8_") + hw("j_s1_9_"),
        highlight=["v_gobilda_1614_bearing_lower", "v_gobilda_1614_bearing_upper"],
        labels=[("motor", BD.CENTER, MOTOR_FACE_Z - 22.0), ("encoder", BD.CENTER, MOTOR_FACE_Z - 58.0),
                ("motor shaft", BD.CENTER, MOTOR_FACE_Z + 4.0), ("motor plate", -28.0, MPLATE_Z0 + 1.6),
                ("REX pulley", 5.0, -5.0), ("GT2 belt", 15.0, BD.BELT_Z), ("motor pulley", BD.CENTER, -20.0), ("plate A", 50.0, 6.0),
                ("race spacer + shims", 6.0, 4.0), ("pillow block", 13.0, 17.0), ("bearing (lower)", -5.5, 11.5),
                ("bearing (upper)", -5.5, 22.5), ("REX shaft", -1.0, 32.0), ("hub", 20.0, 36.0),
                ("carrier disc", -70.0, 48.0), ("rim roller", ROLLER_R, ROLLER_Z),
                ("roller block", RB_X1 - 2.0, 18.0), ("deck sheet", -60.0, 1.5)])
