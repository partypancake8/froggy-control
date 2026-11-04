"""15x15 T-slot profile (OpenBeam / MakerBeam XL class) and named rails.

Cross-section datums: profile centred on its own axis, four identical slots.
Slot opening 3.20, slot depth 5.15, cavity 5.80, lip 1.40, centre bore 2.50.
The cavity is 5.80 wide under the lip, then its floor tapers to a 2.50 floor at
depth 5.15 so the four cavities leave diagonal webs and the core stays attached
(a literal rectangular 5.80 cavity on every face cuts the core loose).
"""
from functools import lru_cache
import itertools

from build123d import Circle, Plane, Polygon, Rectangle, Rot, Solid, extrude

W = 15.0
SLOT_OPEN, SLOT_DEPTH, CAVITY, LIP, BORE = 3.20, 5.15, 5.80, 1.40, 2.50
H = W / 2
TAPER_DEPTH = LIP + 1.10                     # depth where the cavity starts to taper
FLOOR_HALF = 1.25                            # half width of the cavity floor


def cavity_half_width(depth):
    """Half width of the slot cavity at a depth below the rail face (mm)."""
    if depth < LIP:
        return SLOT_OPEN / 2
    if depth <= TAPER_DEPTH:
        return CAVITY / 2
    k = (CAVITY / 2 - FLOOR_HALF) / (SLOT_DEPTH - TAPER_DEPTH)
    return CAVITY / 2 - k * (depth - TAPER_DEPTH)


def slot_points():
    """One slot polygon, opening on the +Y face, in profile-local XY."""
    return [(-SLOT_OPEN / 2, H + 0.5), (SLOT_OPEN / 2, H + 0.5), (SLOT_OPEN / 2, H - LIP),
            (CAVITY / 2, H - LIP), (CAVITY / 2, H - TAPER_DEPTH), (FLOOR_HALF, H - SLOT_DEPTH),
            (-FLOOR_HALF, H - SLOT_DEPTH), (-CAVITY / 2, H - TAPER_DEPTH), (-CAVITY / 2, H - LIP),
            (-SLOT_OPEN / 2, H - LIP)]


@lru_cache(maxsize=1)
def profile():
    face = Rectangle(W, W)
    for ang in (0, 90, 180, 270):
        face = face - Rot(0, 0, ang) * Polygon(*slot_points(), align=None)
    face = face - Circle(BORE / 2)
    assert len(face.faces()) == 1, "profile split into several faces"
    return face


_counter = itertools.count(1)
_AX = {"x": ((1, 0, 0), (0, 1, 0)), "y": ((0, 1, 0), (-1, 0, 0)), "z": ((0, 0, 1), (1, 0, 0))}


def rail(length, axis, position, n=None):
    """A 15x15 rail `length` long along `axis` ('x', 'y' or 'z'), centred on
    `position` (x, y, z). Returns a Solid labelled s_rail_1515_<n>."""
    z_dir, x_dir = _AX[axis]
    start = tuple(position[i] - z_dir[i] * length / 2 for i in range(3))
    solid = extrude(Plane(origin=start, x_dir=x_dir, z_dir=z_dir) * profile(), amount=length)
    if not isinstance(solid, Solid):
        solid = solid.solids()[0]
    solid.label = f"s_rail_1515_{n if n is not None else next(_counter)}"
    return solid
