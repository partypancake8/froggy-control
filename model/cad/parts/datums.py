"""Drum + base stack datums (mm). Z 0 = rail top face, rotor axis = Z.

The brief's numbers (motor face Z 48, carrier plate Z 46..52, drum floor 57)
collide with each other and with the accepted camera cradle, so the stack is
re-derived here from the measured vendor parts; every change is listed in the
run's result.txt deviations.
"""
MOTOR_FACE_Z = 41.0        # NEMA 17 face (17HS19-2004D-E1K: body 68.0 + 0.52 rear screws, shaft 24, D flat from 9)
SHAFT_TOP_Z = MOTOR_FACE_Z + 24.0
PLATE_TOP_Z = 47.5         # fp5_motor_plate Z 41..47.5
HUB_TOP_Z = 57.5           # ServoCity 1308 hub flange-up: flange 53.5..57.5, boss 49.5..53.5, set screw at 53.5
HUB_FLANGE_BOTTOM_Z = HUB_TOP_Z - 4.0
DECK_BOTTOM_Z = HUB_FLANGE_BOTTOM_Z   # carrier central slab underside (3 mm over the plate screw heads)
DECK_TOP_Z = 64.5          # carrier deck top = print bed face = camera carrier foot
DRUM_FLOOR_Z = 60.5        # tube end in the 51 deg V groove
DRUM_H, DRUM_OD, DRUM_WALL = 152.0, 152.4, 3.2
# V12 drum tube (I036, 2026-10-28): Rebower cast acrylic 145 ID x 150 OD x 14 in (Amazon), replaces the 152.4 x 3.2
# Plastic-Craft tube (unavailable). OD tolerance unknown: measure, then print the seats at measured OD + 0.6.
DRUM12_OD, DRUM12_WALL = 150.0, 2.5
DRUM_TOP_Z = DRUM_FLOOR_Z + DRUM_H
DISC_OD, ACRYLIC_T = 160.0, 3.2
CAMERA_DZ = DECK_TOP_Z - 61.0   # camera carrier sits 3.5 above its accepted Z 61 foot
BEARING_Z = (16.0, 26.0)   # lower and upper 6814 faces (stacked)
SET_SCREW_DIR = (-1.0, 0.0, 0.0)  # hub set screw and motor D flat face -X; access channel along -X
