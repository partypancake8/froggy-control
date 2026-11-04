"""Fixed review cameras for `bin/cad review` and `cad-fusion render`.

These never change between builds, so a before/after pair is the same camera on two commits and
every difference in the picture is a difference in the design. Change a camera only on purpose, in
its own commit, and say so: the next review's montage will then compare unlike views.

Datums (docs/ICD.md): Z 0 = deck rail top face, XY zero on the rotor axis, +X frog side, -Y is the
front panel (the e-stop face), Z up, mm. Plinth Z -123..3, panels at |X|, |Y| 150..153, cage top
ring stack to Z 197, rotor disc Z 45..51, drum top Z ~162.

Each view: eye, focal point and up vector in mm (absolute, never fitted to the geometry), parallel
projection with a fixed `scale` (half the image height in mm), a caption, and `hide` (regex over
component names or `module:<name>`) for parts that would block the subject. `fusion` is the same
camera for Fusion (cm units, see froggy_cad/fusion.py cmd_render).
"""

from lib.naming import MODULE_LABELS

PX = (1400, 1050)          # image size, 4:3

# the drive + drum view keeps the drive cartridge and the rotor modules and hides every other module (by group)
DRIVE_DRUM_KEEP = ("drive_base", "rotor")

VIEWS = [
    dict(name="iso", caption="Isometric from front right",
         eye=(1400.0, -1700.0, 1250.0), focal=(0.0, 0.0, 30.0), up=(0.0, 0.0, 1.0), scale=270.0, hide=[]),
    dict(name="front", caption="Front (operator face, -Y)",
         eye=(0.0, -2000.0, 37.0), focal=(0.0, 0.0, 37.0), up=(0.0, 0.0, 1.0), scale=215.0, hide=[]),
    dict(name="right", caption="Right side (+X)",
         eye=(2000.0, 0.0, 37.0), focal=(0.0, 0.0, 37.0), up=(0.0, 0.0, 1.0), scale=215.0, hide=[]),
    dict(name="back", caption="Back (connector face, +Y)",
         eye=(0.0, 2000.0, 37.0), focal=(0.0, 0.0, 37.0), up=(0.0, 0.0, 1.0), scale=215.0, hide=[]),
    dict(name="top", caption="Top (+X right, front at the bottom)",
         eye=(0.0, 0.0, 2000.0), focal=(0.0, 0.0, 0.0), up=(0.0, 1.0, 0.0), scale=200.0, hide=[]),
    dict(name="close_front_panel", caption="Close-up: front panel and e-stop station",
         eye=(700.0, -1500.0, 350.0), focal=(0.0, -150.0, -60.0), up=(0.0, 0.0, 1.0), scale=105.0, hide=[]),
    dict(name="close_drum_top", caption="Close-up: drum top (cage hidden)",
         eye=(900.0, -1300.0, 1400.0), focal=(0.0, 0.0, 115.0), up=(0.0, 0.0, 1.0), scale=145.0,
         hide=["module:cage"]),
    dict(name="close_hub", caption="Close-up: hub, spindle and rim rollers between deck and disc (cage hidden)",
         eye=(1300.0, -1500.0, 260.0), focal=(0.0, 0.0, 25.0), up=(0.0, 0.0, 1.0), scale=100.0,
         hide=["module:cage"]),
    dict(name="close_drive_underside", caption="Close-up: drive and plinth from below (bottom panel and feet hidden)",
         eye=(900.0, -1100.0, -1500.0), focal=(0.0, 0.0, -55.0), up=(0.0, 0.0, 1.0), scale=170.0,
         hide=["module:feet", r"^s_panel_bottom$"]),
    dict(name="drive_drum", caption="Drive cartridge and rotor only (frame, deck, plinth, electrical, feet, cage hidden)",
         eye=(1300.0, -1490.0, 975.0), focal=(0.0, 10.0, 75.0), up=(0.0, 0.0, 1.0), scale=175.0,
         hide=["module:" + m for m in MODULE_LABELS if m not in DRIVE_DRUM_KEEP]),
]

BY_NAME = {v["name"]: v for v in VIEWS}


def hidden(view, name, module):
    """True when component `name` (module `module`) is hidden in `view`."""
    import re
    for h in view.get("hide", []):
        if h.startswith("module:"):
            if module == h.split(":", 1)[1]:
                return True
        elif re.search(h, name):
            return True
    return False
