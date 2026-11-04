"""Vendor STEP imports: per-instance deepcopy (so labels survive export),
cylindrical hole reading through OCP (Face.radius is None on imported STEP
cylinders), mount-face reading and source.json loading."""
import copy
import json
import os
from functools import lru_cache

import numpy as np
from build123d import Compound, import_step
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VENDOR = os.path.join(ROOT, "vendor")


@lru_cache(maxsize=None)
def _load(path):
    shape = import_step(path)
    if isinstance(shape, Compound):
        shape = shape.solids()[0]
    return shape


def import_step_instance(path, label=None, location=None):
    """Own deep copy of an imported STEP solid (own TShape), optionally moved."""
    part = copy.deepcopy(_load(path))
    if location is not None:
        part = part.moved(location)
    if label:
        part.label = label
    return part


@lru_cache(maxsize=None)
def _load_all(path):
    """Every solid of a multi-solid vendor STEP, in file order (_load keeps only the first). A solid that fails
    BRepCheck is healed with ShapeFix_Shape (kept only when the result is a valid solid of the same volume)."""
    from build123d import Solid
    from OCP.ShapeFix import ShapeFix_Shape
    out = []
    for s in import_step(path).solids():
        if not s.is_valid:
            fx = ShapeFix_Shape(s.wrapped)
            fx.Perform()
            t = Solid(fx.Shape())
            if t.is_valid and abs(t.volume - s.volume) <= 1e-3 * abs(s.volume):
                s = t
        out.append(s)
    return tuple(out)


def import_step_solids(path, location=None, indices=None):
    """Own deep copies of the solids of a multi-solid vendor STEP (all, or the given
    file-order indices), each optionally moved by `location`."""
    sols = _load_all(path)
    out = []
    for i in (range(len(sols)) if indices is None else indices):
        s = copy.deepcopy(sols[i])
        out.append(s.moved(location) if location is not None else s)
    return out


def cyl_holes(solid, rmin=1.2, rmax=2.0):
    """Cylindrical faces with radius in [rmin, rmax]: radius, axis point, unit axis."""
    out = []
    for f in solid.faces():
        s = BRepAdaptor_Surface(f.wrapped)
        if s.GetType() != GeomAbs_Cylinder:
            continue
        c = s.Cylinder()
        r = c.Radius()
        if rmin <= r <= rmax:
            ax = c.Axis()
            d = np.array([ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z()])
            p0 = np.array([ax.Location().X(), ax.Location().Y(), ax.Location().Z()])
            out.append(dict(radius=r, p0=p0, axis=d / np.linalg.norm(d), face=f))
    return out


def planar_faces(solid):
    for f in solid.faces():
        if BRepAdaptor_Surface(f.wrapped).GetType() == GeomAbs_Plane:
            yield f, np.array(tuple(f.normal_at())), np.array(tuple(f.center()))


def mount_face(solid, axis):
    """Largest planar face whose normal is parallel to `axis`; returns
    (face, outward normal, centre)."""
    best = None
    for f, n, c in planar_faces(solid):
        if abs(abs(n @ axis) - 1) < 1e-6 and (best is None or f.area > best[0].area):
            best = (f, n, c)
    return best


def bracket_holes(solid, rmin=1.2, rmax=2.0):
    """Each hole with its mount face: centre where the axis pierces the mount
    face, axis pointing out of the mount face (into whatever it mounts on)."""
    holes = []
    for h in cyl_holes(solid, rmin, rmax):
        f, n, c = mount_face(solid, h["axis"])
        t = ((c - h["p0"]) @ n) / (h["axis"] @ n)
        centre = h["p0"] + t * h["axis"]
        holes.append(dict(radius=h["radius"], centre=centre, axis=n.copy(), mount_face=f, mount_normal=n))
    return holes


def source(rel_dir):
    """source.json of a vendored part directory (relative to vendor/)."""
    d = os.path.join(VENDOR, rel_dir)
    meta = json.load(open(os.path.join(d, "source.json")))
    meta["dir"] = d
    return meta


def vendor_entry(component, rel_dir):
    m = source(rel_dir)
    return dict(component=component, source_url=m.get("url"), author=m.get("author"),
                licence=m.get("licence"), file=os.path.join("vendor", rel_dir, m.get("file", "")),
                price_usd=m.get("price_usd"))


def manifest_row(key):
    """A row of vendor/manifest.json (vendor STEPs copied from fusion-pipeline
    and the catalogue models in lib/hardware.py)."""
    rows = json.load(open(os.path.join(VENDOR, "manifest.json")))
    return rows[key]


def manifest_entry(component, key):
    m = manifest_row(key)
    f = m.get("file", "")
    return dict(component=component, source_url=m.get("source_url"), author=m.get("author"),
                licence=m.get("licence"), file=f if f.startswith("lib/") else os.path.join("vendor", f),
                price_usd=m.get("price_usd"), designation=m.get("designation"))


# ---------------------------------------------------------------- STL vendor parts (froggy v6)

@lru_cache(maxsize=None)
def stl_mesh(path):
    import trimesh
    return trimesh.load(path, force="mesh")


def _face_loops(mesh, level, tol=1e-3):
    """Boundary loops (n x 2, XY) of the horizontal triangles lying at Z = level."""
    from collections import defaultdict
    v, f = mesh.vertices, mesh.faces
    sel = np.all(np.abs(v[f][:, :, 2] - level) < tol, axis=1)
    key = np.round(v / 1e-4).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    F = inv[f[sel]]
    vv = np.zeros((inv.max() + 1, 3))
    vv[inv] = v
    e = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    uq, cnt = np.unique(np.sort(e, axis=1), axis=0, return_counts=True)
    adj = defaultdict(list)
    for a, c in uq[cnt == 1]:
        adj[a].append(c)
        adj[c].append(a)
    seen, out = set(), []
    for s0 in list(adj):
        if s0 in seen:
            continue
        loop, prev, cur = [s0], None, s0
        seen.add(s0)
        while True:
            nxt = [x for x in adj[cur] if x != prev and x not in seen]
            if not nxt:
                break
            prev, cur = cur, nxt[0]
            seen.add(cur)
            loop.append(cur)
        if len(loop) >= 3:
            out.append(vv[loop][:, :2])
    return out


def _area(c):
    x, y = c[:, 0], c[:, 1]
    return 0.5 * (np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def _circle(c):
    """(centre, radius) if the loop is a circle within 2 %, else None."""
    cen = c.mean(axis=0)
    r = np.linalg.norm(c - cen, axis=1)
    return (cen, float(r.mean())) if len(c) >= 12 and r.max() - r.min() < 0.02 * r.mean() else None


@lru_cache(maxsize=None)
def stl_plate_features(path):
    """Read a flat printed plate STL (thin axis Z, rail side at Z 0 or at the
    top, see below): outline, thickness, circular holes with their bottom and
    top radii (countersinks), and raised regions above the main slab (e.g.
    alignment tongues). All numbers come from the mesh."""
    m = stl_mesh(path)
    levels = sorted({round(float(z), 3) for z in m.vertices[:, 2]})
    z0 = levels[0]
    lev_loops = {z: _face_loops(m, z) for z in levels}
    lev_loops = {z: ls for z, ls in lev_loops.items() if ls}
    outline = max(lev_loops[z0], key=lambda c: abs(_area(c)))
    oarea = abs(_area(outline))
    slab_top = max(z for z, ls in lev_loops.items() if max(abs(_area(c)) for c in ls) >= 0.5 * oarea)
    holes = []
    for c in lev_loops[z0]:
        ci = _circle(c)
        if ci and abs(_area(c)) < 50:
            holes.append(dict(centre=ci[0], r=ci[1], r_top=ci[1]))
    for c in lev_loops.get(slab_top, []):
        ci = _circle(c)
        if not ci:
            continue
        for h in holes:
            if np.linalg.norm(h["centre"] - ci[0]) < 0.05 and ci[1] > h["r"] + 0.3:
                h["r_top"] = ci[1]
    raised = []
    for z, ls in lev_loops.items():
        if z <= slab_top + 1e-6:
            continue
        for c in ls:
            if abs(_area(c)) > 50 and not _circle(c):
                raised.append(dict(z=z, loop=c))
    return dict(outline=outline, z0=z0, slab_top=slab_top, holes=holes, raised=raised,
                levels=levels, faces=len(m.faces))


@lru_cache(maxsize=None)
def _stl_plate(path, ream_r, tol=None):
    """B-rep rebuilt from the STL plate's own measured features (planar
    outline extrusion, true cylindrical holes and 90 deg countersinks, raised
    tongues), so holes are real cylinders the joint checks can read. ream_r:
    clearance holes opened to this radius when the printed hole is smaller."""
    from build123d import Align, Cone, Cylinder, Location, Plane, Polyline, extrude, make_face
    F = stl_plate_features(path)
    ol = F["outline"]
    if tol:                                 # representative: outline simplified (Douglas-Peucker) to tol
        from lib.represent import simplify
        ol = simplify(ol, tol)
    pts = [tuple(p) for p in ol]
    if _area(ol) < 0:
        pts = pts[::-1]
    t = F["slab_top"] - F["z0"]
    body = extrude(make_face(Polyline(*pts, pts[0])), amount=t)
    for rz in F["raised"]:
        q = [tuple(p) for p in rz["loop"]]
        if _area(rz["loop"]) < 0:
            q = q[::-1]
        body = body + extrude(Plane.XY.offset(t) * make_face(Polyline(*q, q[0])), amount=rz["z"] - F["slab_top"])
    top = max(rz["z"] for rz in F["raised"]) - F["z0"] if F["raised"] else t
    for h in F["holes"]:
        x, y = h["centre"]
        r = max(h["r"], ream_r or 0.0)
        body = body - Cylinder(r, 3 * top + 2).moved(Location((x, y, 0)))
        if h["r_top"] > h["r"] + 0.3:
            dz = h["r_top"] - h["r"]           # 90 deg countersink (45 deg half angle)
            body = body - Cone(h["r"], h["r_top"] + 0.5, dz + 0.5,
                               align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y, t - dz)))
    sols = body.solids()
    assert len(sols) == 1, f"{path}: plate rebuilt into {len(sols)} solids"
    return sols[0]


def stl_plate(path, label=None, location=None, ream_r=None, tol=None):
    part = copy.deepcopy(_stl_plate(path, ream_r, tol))
    if location is not None:
        part = part.moved(location)
    if label:
        part.label = label
    return part


@lru_cache(maxsize=None)
def _mesh_solid(path):
    """Closed triangle mesh (one or more watertight bodies) as OCC solids:
    one planar face per triangle, sewn into a shell per body."""
    from build123d import Compound, Solid
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeSolid,
                                    BRepBuilderAPI_Sewing)
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_SHELL
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    m = stl_mesh(path)
    sols = []
    for b in m.split(only_watertight=False):
        sew = BRepBuilderAPI_Sewing(1e-5)
        P = [gp_Pnt(*map(float, p)) for p in b.vertices]
        for a, c, d in b.faces:
            poly = BRepBuilderAPI_MakePolygon(P[a], P[c], P[d], True)
            sew.Add(BRepBuilderAPI_MakeFace(poly.Wire(), True).Face())
        sew.Perform()
        ex = TopExp_Explorer(sew.SewedShape(), TopAbs_SHELL)
        s = Solid(BRepBuilderAPI_MakeSolid(getattr(TopoDS, "Shell_s", None) and TopoDS.Shell_s(ex.Current()) or TopoDS.Shell(ex.Current())).Solid())
        if s.volume < 0:
            s = Solid(s.wrapped.Reversed())
        # merge coplanar triangles into single faces (fewer stray edges in Fusion, faster booleans)
        from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
        u = ShapeUpgrade_UnifySameDomain(s.wrapped, True, True, True)
        u.Build()
        merged = Solid(u.Shape())
        if merged.is_valid and abs(merged.volume - s.volume) < 1e-6 * max(1.0, abs(s.volume)):
            s = merged
        sols.append(s)
    return sols[0] if len(sols) == 1 else Compound(sols)


def mesh_solid(path, label=None, location=None):
    part = copy.deepcopy(_mesh_solid(path))
    if location is not None:
        part = part.moved(location)
    if label:
        part.label = label
    return part
