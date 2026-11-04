"""Drum + base stage: the frame (assemblies.frame.add_frame) plus the motor
stack centred on it and the rotor that spins on it.

drive_base: fp5_motor_tower on the cross rails (4 x M3 x 8 slot-nut joints),
two stacked 6814-2RS on the tower, fp_bearing_retainer, the NEMA 17 hanging
face-up inside the tower ring (face Z 41) from fp5_motor_plate (4 x M3 x 12
into lug inserts, 4 x M3 x 10 tapped into the motor face).
rotor: motor shaft, ServoCity 1308 hub flange-up on the D shaft, fp5_drum_carrier
on the outer races keyed and bolted to the hub flange (4 x M3 x 16 + hex nuts),
s_drum_tube, s_top_disc, fp_disc_clamp_ring, fp_camera_carrier (4 x M3 x 8 into
deck inserts) with v_camera_fisheye.

Install steps (driver_path_clear counts bodies installed at or before a joint):
0 frame, 1 tower (+ lug inserts), 2 bearings + retainer, 3 motor + plate,
4 hub + carrier bench subassembly (+ deck inserts), 5 camera carrier + camera,
6 drum tube, top disc, clamp ring.
"""
import os
from functools import lru_cache

import numpy as np
from build123d import Compound, import_step

from assemblies.frame import _location, add_frame
from lib import hardware as HW
from lib import vendor as V
from lib.assembly import Assembly
from lib.joints import bolt_joint, hex_nut_joint, insert_joint, tapped_joint
from parts import datums as D
from parts import (fp5_drum_carrier, fp5_motor_plate, fp5_motor_tower, fp_bearing_retainer, fp_camera_carrier,
                   fp_disc_clamp_ring, s_drum_tube, s_top_disc)

MOTOR_STEP = os.path.join(V.VENDOR, "motion/stepperonline_17hs19_2004d_e1k/17HS19-2004D-E1K.step")
HUB_STEP = os.path.join(V.VENDOR, "coupling/servocity-1308-0016-0005-lightweight-set-screw-hub-5mm.step")
HUB = "v_servocity_1308_hub_5mm"

C = {  # old per-module tints (kept as meta module_tint; display colours come from lib.materials)
    "rail": (0.62, 0.66, 0.72), "bracket": (0.95, 0.45, 0.10),
    "tower": (0.16, 0.42, 0.86), "plate": (0.33, 0.62, 0.96), "retainer": (0.10, 0.30, 0.66),
    "bearing": (0.58, 0.68, 0.84), "motor": (0.10, 0.17, 0.38),
    "carrier": (0.18, 0.66, 0.32), "camera_carrier": (0.50, 0.82, 0.38), "hub": (0.10, 0.45, 0.28),
    "shaft": (0.36, 0.58, 0.40), "camera": (0.08, 0.32, 0.14), "ring": (0.62, 0.88, 0.52),
    "acrylic": (0.72, 0.90, 0.96),
    "screw": (0.14, 0.14, 0.16), "nut": (0.26, 0.26, 0.30), "insert": (0.36, 0.30, 0.14), "set_screw": (0.14, 0.14, 0.16),
}
ACRYLIC_OPACITY = 0.35


@lru_cache(maxsize=None)
def _motor_solids():
    """(mechanical body solids, shaft solid) of the vendor STEP, in source
    coordinates (shaft +Y, face Y 0). The cable, leads and connector bodies that
    leave the 42.3 square are dropped, as in the old manifest's mechanical set."""
    sols = import_step(MOTOR_STEP).solids()
    body, shaft = [], None
    for s in sols:
        b = s.bounding_box()
        if max(abs(b.min.X), abs(b.max.X), abs(b.min.Z), abs(b.max.Z)) <= 2.51 and b.max.Y > 20:
            shaft = s
        elif max(abs(b.min.X), abs(b.max.X), abs(b.min.Z), abs(b.max.Z)) <= 21.16:
            body.append(s)
    return body, shaft


def motor_length():
    body, shaft = _motor_solids()
    ys = [s.bounding_box().min.Y for s in body]
    return dict(body_length_mm=round(-min(ys), 3), shaft_above_face_mm=round(shaft.bounding_box().max.Y, 3),
                pilot_mm=round(max(s.bounding_box().max.Y for s in body), 3), body_solids=len(body))


# motor: source +Y -> world +Z, source +Z -> world +X (D flat, source -Z, faces world -X), source X -> world Y
R_MOTOR = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)
# hub flange-up: source +Y (mounting face -> boss) -> world -Z, set screw (source -Z) -> world -X
R_HUB = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)


def _motor(A):
    body, shaft = _motor_solids()
    loc = _location(R_MOTOR, (0, 0, D.MOTOR_FACE_Z))
    import copy
    comp = Compound(children=[copy.deepcopy(s).moved(loc) for s in body])
    sh = copy.deepcopy(shaft).moved(loc)
    A.add("v_nema17_17hs19_2004d_e1k", comp, "motor", "drive_base", C["motor"], step=3)
    A.add("v_nema17_shaft", sh, "shaft", "rotor", C["shaft"], step=3,
          note="motor shaft (vendor STEP solid), rotates with the hub")
    A.vendor.append(V.manifest_entry("v_nema17_17hs19_2004d_e1k", "v_nema17"))


def _hub(A):
    sols = import_step(HUB_STEP).solids()
    hub = max(sols, key=lambda s: s.volume)
    setscrew = min(sols, key=lambda s: s.volume)
    mount = min(((f, n, c) for f, n, c in V.planar_faces(hub) if n[1] < -0.999), key=lambda t: t[2][1])
    hb = hub.bounding_box()
    p0 = np.array([(hb.min.X + hb.max.X) / 2, mount[2][1], (hb.min.Z + hb.max.Z) / 2])
    t = np.array([0, 0, D.HUB_TOP_Z]) - R_HUB @ p0
    loc = _location(R_HUB, t)
    import copy
    # the vendor product is hub + its set screw (threaded into the hub in the
    # vendor STEP): one component, as ServoCity ships it
    comp = Compound(children=[copy.deepcopy(hub).moved(loc), copy.deepcopy(setscrew).moved(loc)])
    A.add(HUB, comp, "hub", "rotor", C["hub"], step=4, includes="1308-0016-0005 hub + 2806-0005-0004 set screw")
    A.vendor.append(V.manifest_entry(HUB, "v_servocity_1308_0016_0005_lightweight_set_screw_hub_5mm"))


def build():
    A = Assembly("drum_base")
    add_frame(A)
    # drive base
    A.add("fp5_motor_tower", fp5_motor_tower.build(), "printed", "drive_base", C["tower"], step=1)
    for i, z in enumerate(D.BEARING_Z):
        n = f"v_bearing_6814_2rs_{'lower' if i == 0 else 'upper'}"
        from build123d import Location
        A.add(n, HW.bearing_6814_2rs(n).moved(Location((0, 0, z))), "bearing", "drive_base", C["bearing"], step=2)
        A.vendor.append(V.manifest_entry(n, "v_bearing_6814"))
    A.add("fp_bearing_retainer", fp_bearing_retainer.build(), "printed", "drive_base", C["retainer"], step=2)
    _motor(A)
    A.add("fp5_motor_plate", fp5_motor_plate.build(), "printed", "drive_base", C["plate"], step=3)
    # rotor
    _hub(A)
    A.add("fp5_drum_carrier", fp5_drum_carrier.build(), "printed", "rotor", C["carrier"], step=4)
    A.add("fp_camera_carrier", fp_camera_carrier.build(), "printed", "rotor", C["camera_carrier"], step=5)
    cam = HW.camera_fisheye_imx219_160("v_camera_fisheye_imx219_160").moved(fp_camera_carrier.camera_location())
    A.add("v_camera_fisheye_imx219_160", cam, "camera", "rotor", C["camera"], step=5,
          bodies=[("v_camera_fisheye_imx219_160_pcb", "pcb_green"), ("v_camera_fisheye_imx219_160_lens", "glass_lens")])
    A.vendor.append(V.manifest_entry("v_camera_fisheye_imx219_160", "v_camera_fisheye"))
    A.add("s_drum_tube", s_drum_tube.build(), "stock", "rotor", C["acrylic"], step=6, opacity=ACRYLIC_OPACITY)
    A.add("s_top_disc", s_top_disc.build(), "stock", "rotor", C["acrylic"], step=6, opacity=ACRYLIC_OPACITY)
    A.add("fp_disc_clamp_ring", fp_disc_clamp_ring.build(), "printed", "rotor", C["ring"], step=6)

    down = (0, 0, -1)
    rail_at_x = {45: "s_rail_1515_5", -45: "s_rail_1515_6"}
    for x, y in fp5_motor_tower.SCREWS:
        jn = f"j_tower_flange_{'p' if x > 0 else 'n'}x{'p' if y > 0 else 'n'}y"
        bolt_joint(A, ["fp5_motor_tower", rail_at_x[int(x)]], (x, y, 0), down, screw="M3x8", nut="slot_nut",
                   name=jn, module="drive_base", evaluate=False, contact="faces", step=1)
    for i, (x, y) in enumerate(fp5_motor_tower.INSERTS):
        insert_joint(A, ["fp5_motor_plate", "fp5_motor_tower"], (x, y, fp5_motor_tower.TOP_Z), down, screw="M3x12",
                     name=f"j_plate_lug_{fp5_motor_tower.LUG_ANGLES[i]:03d}", module="drive_base", evaluate=False, step=3)
    for x, y in fp5_motor_plate.MOTOR_HOLES:
        tapped_joint(A, ["fp5_motor_plate", "v_nema17_17hs19_2004d_e1k"], (x, y, D.MOTOR_FACE_Z), down, screw="M3x10",
                     name=f"j_motor_face_{'p' if x > 0 else 'n'}x{'p' if y > 0 else 'n'}y", module="drive_base",
                     evaluate=False, step=3)
    for x, y in fp5_drum_carrier.HUB_HOLES:
        hex_nut_joint(A, ["fp5_drum_carrier", HUB], (x, y, D.HUB_TOP_Z), down, screw="M3x16",
                      name=f"j_hub_carrier_{'p' if x > 0 else 'n'}x{'p' if y > 0 else 'n'}y", module="rotor",
                      evaluate=False, step=4)
    for x, y in fp_camera_carrier.MOUNT_HOLES:
        insert_joint(A, ["fp_camera_carrier", "fp5_drum_carrier"], (x, y, D.DECK_TOP_Z), down, screw="M3x8",
                     name=f"j_camera_carrier_{'p' if x > 0 else 'n'}x{'p' if y > 0 else 'n'}y", module="rotor",
                     evaluate=False, step=5)
    A.notes.append(f"motor: {motor_length()}")
    return A
