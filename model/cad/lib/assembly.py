"""Assembly container: named solids with module, kind, material (lib.materials),
colour (the material display colour) and metadata,
plus the joint specs that lib.joints evaluates."""
from dataclasses import dataclass, field

from lib import materials as M

COLOURS = {"rail": (0.62, 0.66, 0.72), "bracket": (0.95, 0.45, 0.10),
           "screw": (0.16, 0.16, 0.18), "nut": (0.96, 0.82, 0.10),
           "insert": (0.36, 0.30, 0.14), "set_screw": (0.16, 0.16, 0.18)}


@dataclass
class Part:
    name: str
    solid: object
    kind: str
    module: str
    colour: tuple
    meta: dict = field(default_factory=dict)
    material: str = ""


@dataclass
class Assembly:
    name: str
    parts: dict = field(default_factory=dict)
    joints: list = field(default_factory=list)     # joint specs (plain data)
    vendor: list = field(default_factory=list)     # vendor_parts sheet entries
    notes: list = field(default_factory=list)

    def add(self, name, solid, kind, module, colour=None, **meta):
        if name in self.parts:
            raise ValueError(f"duplicate component {name}")
        solid.label = name
        mat, rgba = M.assign(name, kind)
        meta["module_tint"] = tuple(colour or COLOURS.get(kind, rgba[:3]))   # old per-module tint, kept for reference
        if rgba[3] < 1:
            meta.setdefault("opacity", rgba[3])
        self.parts[name] = Part(name, solid, kind, module, tuple(rgba[:3]), meta, mat)
        return self.parts[name]

    def __getitem__(self, name):
        return self.parts[name].solid

    def names(self, kind=None):
        return [n for n, p in self.parts.items() if kind is None or p.kind == kind]


def bbox_of(solid):
    b = solid.bounding_box()
    return tuple(b.min), tuple(b.max)


def boxes_touch(a, b, tol=1e-3):
    (amn, amx), (bmn, bmx) = a, b
    return all(min(amx[k], bmx[k]) - max(amn[k], bmn[k]) > -tol for k in range(3))


def common_volume(a, b, fuzzy=0.0):
    """Volume of a & b straight through OCP (no result cleaning or casting).
    If OCC fails or returns a null shape (seen on vendor STEP bodies with
    coincident faces), retry with a 1e-4 mm fuzzy tolerance."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    from OCP.collections import List_TopoDS_Shape  # noqa: F401  (OCP 8 list type)
    la, lb = List_TopoDS_Shape(), List_TopoDS_Shape()
    la.Append(a.wrapped)
    lb.Append(b.wrapped)
    op = BRepAlgoAPI_Common()
    op.SetArguments(la)
    op.SetTools(lb)
    if fuzzy:
        op.SetFuzzyValue(fuzzy)
    op.Build()
    if not op.IsDone() or op.Shape().IsNull():
        if not fuzzy:
            return common_volume(a, b, 1e-4)
        raise RuntimeError(f"boolean common failed: {getattr(a, 'label', '')} x {getattr(b, 'label', '')}")
    vol = 0.0
    ex = TopExp_Explorer(op.Shape(), TopAbs_SOLID)
    while ex.More():
        p = GProp_GProps()
        BRepGProp.VolumeProperties_s(ex.Current(), p)
        vol += abs(p.Mass())
        ex.Next()
    return vol


def _bbox(self, name):
    c = self.__dict__.setdefault("_bbox_cache", {})
    if name not in c:
        c[name] = bbox_of(self.parts[name].solid)
    return c[name]


def _ivol(self, a, b, solid_a=None):
    """Intersection volume of components a and b (or of an extra solid keyed
    `a` against component b), cached by name pair; booleans only when the
    bounding boxes overlap or touch. Returns (volume, boolean_was_run)."""
    cache = self.__dict__.setdefault("_ivol_cache", {})
    key = tuple(sorted((a, b)))
    if key in cache:
        return cache[key], False
    sa = solid_a if solid_a is not None else self.parts[a].solid
    if solid_a is not None:                    # extra solids (driver paths): bbox once per key, not per pair
        bc = self.__dict__.setdefault("_extra_bbox", {})
        if a not in bc:
            bc[a] = bbox_of(sa)
        ba = bc[a]
    else:
        ba = self.bbox(a)
    if not boxes_touch(ba, self.bbox(b)):
        cache[key] = 0.0
        return 0.0, False
    if solid_a is None and radial_apart(self, a, b):
        cache[key] = 0.0
        return 0.0, False
    cache[key] = common_volume(sa, self.parts[b].solid)
    return cache[key], True


def radial_env(A, name):
    """(r_min, r_max) about the Z axis: r_max from the bbox corners, r_min from the
    bbox or, for ring-shaped parts, the part's meta `rmin` (its true inner radius)."""
    import math
    (x0, y0, _), (x1, y1, _) = A.bbox(name)
    rmax = max(math.hypot(x, y) for x in (x0, x1) for y in (y0, y1))
    dx = 0.0 if x0 <= 0 <= x1 else min(abs(x0), abs(x1))
    dy = 0.0 if y0 <= 0 <= y1 else min(abs(y0), abs(y1))
    return max(math.hypot(dx, dy), A.parts[name].meta.get("rmin", 0.0)), rmax


def radial_apart(A, a, b):
    """True when the two parts' annular envelopes about Z cannot meet (exact for
    parts tagged with meta rmin; bbox-only parts never reject)."""
    if "rmin" not in A.parts[a].meta and "rmin" not in A.parts[b].meta:
        return False
    ra, rb = radial_env(A, a), radial_env(A, b)
    return ra[1] < rb[0] - 1e-3 or rb[1] < ra[0] - 1e-3


Assembly.bbox = _bbox
Assembly.ivol = _ivol
