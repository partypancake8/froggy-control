"""Froggy design language as importable numbers (docs/DESIGN-LANGUAGE.md, DL1..DL28).

Every part file imports its form numbers from here instead of typing them, so a rule changes in
one place and every part follows on the next build:

    from lib.design_constants import PRINTED, LASER, FASTENERS, fillet_printed, corner_laser

lib/designcheck.py reads the same constants, so the check and the parts can never disagree.
Python stdlib only (designcheck must also run under plain python3). Values are mm unless named.
"""

# ---------------------------------------------------------------- printed PETG form (DL6, DL7)
PRINTED = dict(
    fillet_outside_min=2.0,     # every outside edge of a print
    fillet_outside_large=4.0,   # outside edges on prints with any bbox side over large_part
    large_part=40.0,
    fillet_inside=1.0,          # inside corners
    wall_min=2.4,               # DL7 floor for non-structural walls
    wall_structural=3.0,        # structural walls (lib/printcheck.py MIN_WALL)
    overhang_deg=45.0,          # prints flat without supports (lib/printcheck.py)
    bed_mm=(256.0, 256.0, 256.0),  # Bambu P2S build volume
    insert_only_in_print=True,  # heat-set inserts live only in printed parts (DL10)
)

# ---------------------------------------------------------------- laser aluminium form (DL5, DL8)
SHEET_T = 3.175                 # 1/8 in 6061 off the Fablight
PLATE_T = 6.0                   # 6 mm 6061 where the ICD says PLATE_T
PANEL_T = 3.0                   # plinth panels
LASER = dict(
    corner_outside_min=3.0,     # outside corners of any laser part
    corner_outside_large=6.0,   # outside corners on plates over large_plate in either direction
    large_plate=100.0,
    corner_inside=2.0,          # inside corners (the beam radius is smaller; this is the look)
    edge_margin_d=2.0,          # hole centre at least 2 x hole diameter from any edge
    through_cut_only=True,      # no pockets, grooves, counterbores or countersinks in sheet
    material="aluminium_6061",
)
ACRYLIC_ALLOWED = ("s_drum_tube", "s_top_disc")   # DL5: acrylic only where it must be clear


def fillet_printed(size_mm):
    """Outside edge fillet for a printed part whose largest bbox side is size_mm (DL7)."""
    return PRINTED["fillet_outside_large"] if size_mm > PRINTED["large_part"] else PRINTED["fillet_outside_min"]


def corner_laser(size_mm):
    """Outside corner radius for a laser part whose largest outline side is size_mm (DL8)."""
    return LASER["corner_outside_large"] if size_mm > LASER["large_plate"] else LASER["corner_outside_min"]


def min_edge_margin(hole_d):
    """Smallest hole-centre-to-edge distance for a hole of diameter hole_d (DL8, v9 rule 6)."""
    return LASER["edge_margin_d"] * hole_d


# ---------------------------------------------------------------- fasteners (DL10, DL11)
CLEARANCE_D = {"M2": 2.4, "M2.5": 2.7, "M3": 3.4, "M4": 4.5, "M5": 5.5, "M6": 6.6}
TAP_D = {"M2.5": 2.05, "M3": 2.5, "M4": 3.3, "M5": 4.2}
FASTENERS = {
    # sub-assembly family -> (thread, head, nut side)
    "frame": ("M5", "button", "2020 drop-in T-nut"),
    "gobilda": ("M4", "button", "goBILDA vendor thread"),   # plates to goBILDA parts: the vendor's thread
    "rotor": ("M4", "button", "tapped aluminium or hex nut"),
    "small": ("M3", "button", "hex nut, vendor thread, or heat-set insert in a print"),
}
FUNCTION_SCREWS = ("thumb", "shoulder", "set screw")   # not counted as a family (DL10)
HEAD_MAX_PROUD = 3.0            # DL11: a button head may stand this proud of a show face
NO_COUNTERSINK_BELOW_T = 4.0    # no countersinks in sheet thinner than this (3 mm sheet rule)

# ---------------------------------------------------------------- envelope (DL1)
ENVELOPE = dict(
    tol=0.5,                    # numerical slack on every envelope test
    estop_head_max_proud=35.0,  # only the e-stop head may stand proud of the front panel
    foot_max_below=10.0,        # feet may sit under the bottom panel
)

# ---------------------------------------------------------------- rotor (DL12, DL14, DL26)
RUNNING_BAND = (86.0, 93.0)     # r band under the carrier disc kept empty (ICD S1.28); rim rollers excepted
NET_CLEAR = 8.0                 # rotating parts stay this far inside the net sleeve
HUB_RADIUS = 45.0               # rotor electrics stay inside this radius
DRUM_TOP_POSTS = 2              # DL27 / decision D2: two mirrored aluminium posts carry the drum top

# ---------------------------------------------------------------- part count (DL18, DL26)
CAPS = dict(parts=12, fasteners=40)   # review prompts per sub-assembly, never a target
# sub-assemblies that hold up or guide the rotor and drum: growth here is always allowed (DL26)
SUPPORT_SUBASSEMBLIES = ("Plate A", "Bearing block", "Spindle", "Carrier disc", "Drum", "Rim rollers",
                         "Drum posts", "Motor")
SUPPORT_NAME_RE = r"roller|pillow_block|post|ring_b|ring_top|tube_seat|carrier_disc|plate_a|spindle|hub|collar"

# ---------------------------------------------------------------- colour (DL20)
COLOURS = dict(
    aluminium="#c7ccd4",        # natural 6061 (lib/materials.py aluminium_6061)
    panel="#1b2233",            # plinth panels, near black
    hardware="#202020",         # black oxide fasteners
    acrylic="clear",
    printed="#1f1f21",          # the main PETG filament, black (DL20, 2026-10-12; lib/materials.py PETG_FILAMENT)
    printed_camera="#e0e3e6",   # Camera hub printed parts, light grey (I019, 2026-10-16; lib/materials.py PETG_CAMERA)
    estop=("vendor yellow", "vendor red"),
)
PRINTED_COLOURS_MAX = 2         # black plus the Camera hub light grey (I019, 2026-10-16); a third fails DL20

# ---------------------------------------------------------------- names (DL17)
INTERNAL_PREFIXES = ("v_", "s_", "lp_", "fp9_", "fp7_", "fp6_", "hw_", "j_")
BAD_GROUPS = ("", "other", "misc", "ungrouped", "unsorted", "parts")
RETIRED_TERMS = (r"fp7_pedestal", r"\bpedestal\b", r"608 pair", r"Qi coil", r"acrylic ring", r"lens is 160",
                 r"section_housing", r"TX board")

# ---------------------------------------------------------------- review (DL24)
REVIEW_ROUNDS_MAX = 2           # fresh-context yes/no reviewer, then it goes to Sawyer
