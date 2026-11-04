"""Froggy v6 "shaft under the deck" stack datums (mm). Z 0 = rail top face,
rotor axis = Z, X and Y zero at the frame centre.

Re-derived from the measured vendor parts (goBILDA 1310 hub is 22 tall, the
jaw-coupling envelope 32 long); every change from the proposal's Z stack is
listed in the run's result.txt deviations.
"""
# drive (under the deck)
MOTOR_FACE_Z = -21.0          # NEMA 17 face = motor plate underside
PLATE_Z0, PLATE_T = -21.0, 6.0          # fp6_motor_plate Z -21..-15 (under the cross rails)
PLATE_PAD = 0.4               # 10 mm pads round the four motor screws (top -14.6)
COUPLING_Z0, COUPLING_L = -18.0, 32.0   # jaw coupling envelope, sinks into the plate's 25.5 bore
SHAFT_Z0, SHAFT_Z1, SHAFT_D = 3.5, 75.5, 7.99   # 8 mm shaft (7.99 = h6 running fit in the 8.0 bores)

# bearing housing (fp6_bearing_housing)
FLANGE_T = 6.0
POCKET_R, POCKET_TOP = 12.5, 14.5       # coupling clearance pocket in the flange
BRG_W, BRG_POCKET_R, MID_BORE_R = 7.0, 11.05, 9.0
CONE_K = 1.25                 # cone rise per mm of radius (51 deg: steeper than the 45 deg print limit)
LOWER_BRG_Z0 = POCKET_TOP + CONE_K * (POCKET_R - BRG_POCKET_R)     # 16.31
UPPER_BRG_Z0 = LOWER_BRG_Z0 + 30.0                        # 45.95: 30 mm bearing centre span
COLLAR_W = 8.0
COLLAR_Z0 = UPPER_BRG_Z0 - COLLAR_W                       # collar under the upper inner race
WASHER_Z0, WASHER_T = UPPER_BRG_Z0 + BRG_W, 1.0           # 52.95: spacer on the upper inner race
HUB_Z0, HUB_H = WASHER_Z0 + WASHER_T, 22.0                # 53.95: hub, clamp collar down, lobes up
HUB_TOP_Z = HUB_Z0 + HUB_H                                # 76.31: top of the hub's 14 mm pilot boss
HUB_PILOT_R, HUB_PILOT_H = 7.0, 2.0       # measured on the 1310 STEP: pilot boss on the tapped lobe face
HUB_LOBE_Z = HUB_TOP_Z - HUB_PILOT_H      # 74.31: tapped lobe face (carrier contact face)
BOSS_R, CUP_R_IN = 22.5, 16.5

# rotor
CARRIER_Z0 = HUB_LOBE_Z       # carrier centre underside on the hub's tapped lobe face (boss in a pocket)
DECK_TOP_Z = CARRIER_Z0 + 6.0 # 80.31, print bed face, camera carrier foot
RIM_Z0 = DECK_TOP_Z - 9.0     # rim (r 66..82.5) carries the V groove
V_R, V_HALF, V_DEPTH = 74.6, 4.8, 6.0
DRUM_WALL = 3.2
DRUM_FLOOR_Z = DECK_TOP_Z - V_DEPTH * (1 - (DRUM_WALL / 2) / V_HALF)   # 76.31: tube end in the V
DRUM_H = 152.0
DRUM_TOP_Z = DRUM_FLOOR_Z + DRUM_H

# electrical
COIL_OD, COIL_ID, COIL_T, COIL_GAP = 40.0, 34.0, 2.3, 3.0
RX_Z0 = CARRIER_Z0                       # RX coil flush in the carrier underside recess
TX_Z1 = RX_Z0 - COIL_GAP                 # 72.95
TX_Z0 = TX_Z1 - COIL_T                   # 70.65 = housing cup rim (coil seat)
CUP_TOP_Z = TX_Z0
MAGNET_R, MAGNET_Z0 = 48.0, CARRIER_Z0 - 0.95   # magnet boss 1 mm proud of the underside
HALL_GAP = 1.5
HALL_TOP_Z = MAGNET_Z0 - HALL_GAP
HALL_MODULE_H = 1.6 + 1.52
HALL_POST_TOP_Z = HALL_TOP_Z - HALL_MODULE_H
LED_LEN = 8.6
CARRIER_LED_TOP_Z = DRUM_FLOOR_Z - 0.15
PI_DZ = 8.0                              # Pi Zero PCB underside above the camera-carrier foot (boss tops)
