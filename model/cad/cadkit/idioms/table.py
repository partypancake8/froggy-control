"""Fastener dimension tables, M2.5 to M6, all in mm.

Sources: ISO 4762 (socket head cap screw), ISO 7380-1 (button head), ISO 10642
(countersunk, 90 deg), ISO 4032 (hex nut), ISO 10511 / DIN 985 (nyloc),
ISO 273 (clearance holes), ISO 2306 tap drills, and the Ruthex / CNC Kitchen
standard heat-set insert series (check the datasheet of the insert you bought;
pass your own numbers through `insert(size, **override)` if they differ).

Printed holes shrink: `clearance(size, printed=True)` adds PRINT_HOLE_ADD.
"""

SIZES = ("M2.5", "M3", "M4", "M5", "M6")

PRINT_HOLE_ADD = 0.2          # FDM holes print about 0.2 mm under size

# ISO 273 clearance holes: close, medium, coarse
CLEARANCE = {
    "M2.5": dict(close=2.7, medium=2.9, coarse=3.1),
    "M3": dict(close=3.2, medium=3.4, coarse=3.6),
    "M4": dict(close=4.3, medium=4.5, coarse=4.8),
    "M5": dict(close=5.3, medium=5.5, coarse=5.8),
    "M6": dict(close=6.4, medium=6.6, coarse=7.0),
}

TAP_DRILL = {"M2.5": 2.05, "M3": 2.5, "M4": 3.3, "M5": 4.2, "M6": 5.0}

# head diameter dk, head height k, hex key s
SHCS = {      # ISO 4762
    "M2.5": dict(dk=4.5, k=2.5, key=2.0),
    "M3": dict(dk=5.5, k=3.0, key=2.5),
    "M4": dict(dk=7.0, k=4.0, key=3.0),
    "M5": dict(dk=8.5, k=5.0, key=4.0),
    "M6": dict(dk=10.0, k=6.0, key=5.0),
}
BUTTON = {    # ISO 7380-1 (M2.5 is the common non-ISO size)
    "M2.5": dict(dk=4.7, k=1.5, key=1.5),
    "M3": dict(dk=5.7, k=1.65, key=2.0),
    "M4": dict(dk=7.6, k=2.2, key=2.5),
    "M5": dict(dk=9.5, k=2.75, key=3.0),
    "M6": dict(dk=10.5, k=3.3, key=4.0),
}
COUNTERSUNK = {   # ISO 10642, 90 deg included angle (M2.5 per DIN 7991 practice)
    "M2.5": dict(dk=5.0, k=1.5, key=1.5, angle=90.0),
    "M3": dict(dk=6.72, k=1.86, key=2.0, angle=90.0),
    "M4": dict(dk=8.96, k=2.48, key=2.5, angle=90.0),
    "M5": dict(dk=11.2, k=3.1, key=3.0, angle=90.0),
    "M6": dict(dk=13.44, k=3.72, key=4.0, angle=90.0),
}
HEADS = {"shcs": SHCS, "button": BUTTON, "countersunk": COUNTERSUNK}

HEX_NUT = {   # ISO 4032: across flats s, height m
    "M2.5": dict(s=5.0, m=2.0),
    "M3": dict(s=5.5, m=2.4),
    "M4": dict(s=7.0, m=3.2),
    "M5": dict(s=8.0, m=4.7),
    "M6": dict(s=10.0, m=5.2),
}
NYLOC = {     # ISO 10511 / DIN 985: across flats s, total height h
    "M2.5": dict(s=5.0, h=3.8),
    "M3": dict(s=5.5, h=4.0),
    "M4": dict(s=7.0, h=5.0),
    "M5": dict(s=8.0, h=5.0),
    "M6": dict(s=10.0, h=6.0),
}
HEAT_SET = {  # Ruthex / CNC Kitchen standard: hole d, length, outer d, min wall around the hole
    "M2.5": dict(hole_d=3.6, length=5.7, od=4.0, min_wall=1.6),
    "M3": dict(hole_d=4.0, length=5.7, od=4.6, min_wall=1.6),
    "M4": dict(hole_d=5.6, length=8.1, od=6.3, min_wall=2.0),
    "M5": dict(hole_d=6.4, length=9.5, od=7.1, min_wall=2.0),
    "M6": dict(hole_d=8.0, length=12.7, od=8.7, min_wall=2.4),
}

PRINTED_WALL = 2.4            # DL7: printed walls 2.4 minimum


def norm(size):
    """'m3', 'M3x8', 'M2_5', 3 -> 'M3' / 'M2.5'."""
    s = str(size).strip().upper().replace("_", ".")
    if not s.startswith("M"):
        s = "M" + s
    s = s.split("X")[0]
    if s not in CLEARANCE:
        raise KeyError(f"unknown size {size!r}; known {SIZES}")
    return s


def clearance(size, fit="medium", printed=False):
    d = CLEARANCE[norm(size)][fit]
    return round(d + (PRINT_HOLE_ADD if printed else 0.0), 2)


def head(size, style="shcs"):
    return dict(HEADS[style][norm(size)])


def counterbore(size, style="shcs"):
    """(diameter, depth) of a counterbore that sinks the head flush plus 0.3 mm."""
    h = head(size, style)
    return round(h["dk"] + 1.0, 2), round(h["k"] + 0.3, 2)


def countersink(size):
    """(diameter at the surface, included angle) for a flush countersunk head."""
    h = COUNTERSUNK[norm(size)]
    return round(h["dk"] + 0.4, 2), h["angle"]


def insert(size, **override):
    d = dict(HEAT_SET[norm(size)])
    d.update(override)
    return d


def nut(size, nyloc=False):
    return dict((NYLOC if nyloc else HEX_NUT)[norm(size)])


def boss_od(size, wall=PRINTED_WALL):
    """Outer diameter of a printed boss around a heat-set insert, rounded up to 0.5 mm."""
    import math
    raw = insert(size)["hole_d"] + 2 * max(wall, insert(size)["min_wall"])
    return math.ceil(raw * 2) / 2
