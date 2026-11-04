"""Small build123d helpers shared by the part files."""
from build123d import Axis, Polyline, Solid, make_face, revolve


def revolve_rz(points):
    """Solid of revolution about Z from an (r, z) outline (closed automatically)."""
    pts = [(r, 0, z) for r, z in points]
    if pts[0] != pts[-1]:
        pts.append(pts[0])
    s = revolve(make_face(Polyline(*pts)), Axis.Z)
    return s if isinstance(s, Solid) else s.solids()[0]


def one(shape):
    """The single solid of a boolean result (raises if it split)."""
    sols = shape.solids()
    if len(sols) != 1:
        raise ValueError(f"expected one solid, got {len(sols)}")
    return sols[0]


def fillet_safe(solid, radius, edges):
    """Fillet `edges`; returns the input unchanged (and False) if OCC refuses."""
    edges = list(edges)
    if not edges:
        return solid, True
    try:
        out = solid.fillet(radius, edges)
        if out.is_valid:
            return out, True
    except Exception:  # noqa: BLE001
        pass
    return solid, False
