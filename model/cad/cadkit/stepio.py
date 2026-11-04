"""Read any STEP into named leaf parts, and fast tessellation for renders.

load(path) -> [Part]: one record per leaf of the STEP label tree (a leaf may hold several solids,
e.g. a vendor motor), each with its readable name, the names of its ancestors, the located shape
and the STEP colour when present. A STEP with no tree gives one Part per solid, named from the
label (often empty, which lint reports).
"""
import os
import re
import struct
import tempfile
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Part:
    name: str
    path: list
    shape: object
    colour: tuple | None = None
    meta: dict = field(default_factory=dict)

    @property
    def display(self):
        return clean(self.name) or "(no name)"


def clean(name):
    """STEP names come back with spaces as underscores; show them with spaces."""
    return re.sub(r"_+", " ", name or "").strip()


def _colour(shape):
    c = getattr(shape, "color", None)
    if c is None:
        return None
    try:
        return tuple(float(v) for v in tuple(c)[:3])
    except Exception:
        return None


def load(path):
    from build123d import Compound, import_step
    root = import_step(str(path))
    parts = []

    def walk(s, anc):
        kids = list(getattr(s, "children", ()) or ())
        if isinstance(s, Compound) and kids:
            for ch in kids:
                walk(ch, anc + [s.label or ""])
            return
        if not s.solids():
            return
        parts.append(Part(name=s.label or "", path=anc, shape=s, colour=_colour(s)))

    walk(root, [])
    if not parts:
        for i, s in enumerate(root.solids()):
            parts.append(Part(name=s.label or "", path=[], shape=s, colour=_colour(s)))
    # the root label is the file's product name; drop it from paths but keep it for reports
    return parts


def bbox(parts):
    mn = np.full(3, np.inf)
    mx = np.full(3, -np.inf)
    for p in parts:
        b = p.shape.bounding_box()
        mn = np.minimum(mn, tuple(b.min))
        mx = np.maximum(mx, tuple(b.max))
    return mn, mx


def tessellate(shape, lin=0.08, ang=0.3):
    """(verts (n,3), faces (m,3)) through BRepMesh and OCC's binary STL writer (much faster than
    the per-face Python loop); coincident vertices merged. Same trick as froggy-cad lib/mesh.py."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.StlAPI import StlAPI_Writer
    w = shape.wrapped
    BRepMesh_IncrementalMesh(w, lin, False, ang, True)
    fd, tmp = tempfile.mkstemp(suffix=".stl")
    os.close(fd)
    try:
        wr = StlAPI_Writer()
        wr.ASCIIMode = False
        if not wr.Write(w, tmp):
            return np.zeros((0, 3)), np.zeros((0, 3), np.int64)
        data = open(tmp, "rb").read()
    finally:
        os.unlink(tmp)
    n = struct.unpack("<I", data[80:84])[0]
    if n == 0:
        return np.zeros((0, 3)), np.zeros((0, 3), np.int64)
    rec = np.frombuffer(data, dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")], count=n, offset=84)
    pts = rec["v"].reshape(-1, 3).astype(float)
    uniq, inv = np.unique(np.round(pts, 5), axis=0, return_inverse=True)
    return uniq, inv.reshape(-1, 3).astype(np.int64)
