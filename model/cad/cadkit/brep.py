"""Shared B-rep queries (OCP) used by finish() and lint: edge convexity and
dihedral angle, hole faces, oriented bounding boxes, solid conversion.

Convexity rule: for an edge shared by faces F1 and F2, with t the edge tangent
as oriented inside F1 and n1, n2 the outward normals at the edge midpoint,
the inward direction of F1 is n1 x t (F1 lies to the left of its oriented
boundary seen from n1). The edge is convex when n2 . (n1 x t) < 0.
The dihedral angle is the angle between n1 and n2: 0 = tangent (a blend),
90 = a square crease.
"""
import math

import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Curve2d, BRepAdaptor_Surface
from OCP.BRepLProp import BRepLProp_SLProps
from OCP.GCPnts import GCPnts_AbscissaPoint
from OCP.GeomAbs import GeomAbs_Cone, GeomAbs_Cylinder, GeomAbs_Plane, GeomAbs_Sphere, GeomAbs_Torus
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.collections import IndexedDataMap_TopoDS_Shape_List_TopoDS_Shape_TopTools_ShapeMapHasher as _EFMap

SURF = {GeomAbs_Plane: "plane", GeomAbs_Cylinder: "cylinder", GeomAbs_Cone: "cone",
        GeomAbs_Sphere: "sphere", GeomAbs_Torus: "torus"}


def _v(p):
    return np.array([p.X(), p.Y(), p.Z()])


def surf_type(face_w):
    return SURF.get(BRepAdaptor_Surface(face_w).GetType(), "other")


def face_normal_uv(face_w, u, v):
    s = BRepAdaptor_Surface(face_w)
    pr = BRepLProp_SLProps(s, u, v, 1, 1e-7)
    if not pr.IsNormalDefined():
        return None
    n = _v(pr.Normal())
    if face_w.Orientation() == TopAbs_REVERSED:
        n = -n
    return n


def face_normal_mid(face_w):
    s = BRepAdaptor_Surface(face_w)
    u = 0.5 * (s.FirstUParameter() + s.LastUParameter())
    v = 0.5 * (s.FirstVParameter() + s.LastVParameter())
    return face_normal_uv(face_w, u, v), _v(s.Value(u, v))


def edge_length(edge_w):
    try:
        return GCPnts_AbscissaPoint.Length_s(BRepAdaptor_Curve(edge_w))
    except Exception:
        return 0.0


def _oriented_in(face_w, edge_w):
    ex = TopExp_Explorer(face_w, TopAbs_EDGE)
    while ex.More():
        e = ex.Current()
        if e.IsSame(edge_w):
            return TopoDS.Edge(e)
        ex.Next()
    return None


def is_hole_face(face_w, min_span_deg=170.0):
    """A hole wall: a concave cylinder or cone (outward normal toward the axis) that covers at least
    min_span_deg of the circle (a full bore, or one half of a split bore). Concave fillets and the
    corner arcs of a rounded window are not holes."""
    s = BRepAdaptor_Surface(face_w)
    t = s.GetType()
    if t not in (GeomAbs_Cylinder, GeomAbs_Cone):
        return False
    if math.degrees(s.LastUParameter() - s.FirstUParameter()) < min_span_deg:
        return False
    ax = (s.Cylinder() if t == GeomAbs_Cylinder else s.Cone()).Axis()
    n, p = face_normal_mid(face_w)
    if n is None:
        return False
    a0, d = _v(ax.Location()), _v(ax.Direction())
    r = (p - a0) - ((p - a0) @ d) * d
    return float(n @ r) < 0


def cylinder_info(face_w):
    s = BRepAdaptor_Surface(face_w)
    if s.GetType() != GeomAbs_Cylinder:
        return None
    c = s.Cylinder()
    ax = c.Axis()
    d = _v(ax.Direction())
    d = d / np.linalg.norm(d)
    return dict(radius=c.Radius(), p0=_v(ax.Location()), axis=d,
                vspan=(s.FirstVParameter(), s.LastVParameter()),
                uspan=s.LastUParameter() - s.FirstUParameter())


def edge_faces(shape_w):
    """[(edge, face1, face2)] for every manifold edge (two distinct faces)."""
    m = _EFMap()
    TopExp.MapShapesAndAncestors_s(shape_w, TopAbs_EDGE, TopAbs_FACE, m)
    out = []
    for i in range(1, m.Extent() + 1):
        e = TopoDS.Edge(m.FindKey(i))
        fl = list(m.FindFromIndex(i))
        faces = []
        for f in fl:
            if not any(f.IsSame(g) for g in faces):
                faces.append(f)
        if len(faces) != 2:
            continue
        out.append((e, TopoDS.Face(faces[0]), TopoDS.Face(faces[1])))
    return out


def edge_info(edge_w, f1, f2):
    """dict(length, dihedral_deg, convex, mid) or None when undefined."""
    try:
        c = BRepAdaptor_Curve(edge_w)
        t0, t1 = c.FirstParameter(), c.LastParameter()
        tm = 0.5 * (t0 + t1)
        e1 = _oriented_in(f1, edge_w)
        if e1 is None:
            return None
        from OCP.gp import gp_Pnt, gp_Vec
        p, dv = gp_Pnt(), gp_Vec()
        c.D1(tm, p, dv)
        t = _v(dv)
        if np.linalg.norm(t) < 1e-12:
            return None
        t = t / np.linalg.norm(t)
        if e1.Orientation() == TopAbs_REVERSED:
            t = -t
        ns = []
        for f in (f1, f2):
            c2 = BRepAdaptor_Curve2d(_oriented_in(f, edge_w) or edge_w, f)
            uv = c2.Value(tm)
            n = face_normal_uv(f, uv.X(), uv.Y())
            if n is None:
                return None
            ns.append(n / np.linalg.norm(n))
        n1, n2 = ns
        cosang = float(np.clip(n1 @ n2, -1.0, 1.0))
        dih = math.degrees(math.acos(cosang))
        convex = float(n2 @ np.cross(n1, t)) < 0
        return dict(length=edge_length(edge_w), dihedral_deg=dih, convex=convex, mid=_v(p))
    except Exception:
        return None


def classify_edges(shape_w, min_len=0.0, skip_holes=True):
    """[(edge, info, f1, f2)] with info from edge_info; hole rims skipped when asked."""
    out = []
    for e, f1, f2 in edge_faces(shape_w):
        if skip_holes and (is_hole_face(f1) or is_hole_face(f2)):
            continue
        info = edge_info(e, f1, f2)
        if info is None or info["length"] < min_len:
            continue
        out.append((e, info, f1, f2))
    return out


def faces_of(shape_w):
    ex = TopExp_Explorer(shape_w, TopAbs_FACE)
    seen = []
    while ex.More():
        f = TopoDS.Face(ex.Current())
        if not any(f.IsSame(g) for g in seen):
            seen.append(f)
        ex.Next()
    return seen


def obb_dims(shape_w):
    """Sorted (small, mid, large) sides of the oriented bounding box."""
    from OCP.Bnd import Bnd_OBB
    from OCP.BRepBndLib import BRepBndLib
    b = Bnd_OBB()
    BRepBndLib.AddOBB_s(shape_w, b, True, True, False)
    return tuple(sorted((2 * b.XHSize(), 2 * b.YHSize(), 2 * b.ZHSize())))


def as_solid(shape):
    """build123d Part/Compound/Solid -> one Solid when it holds exactly one, else a Compound."""
    from build123d import Compound, Solid
    if isinstance(shape, Solid):
        return shape
    sols = shape.solids()
    if len(sols) == 1:
        return sols[0] if isinstance(sols[0], Solid) else Solid(sols[0].wrapped)
    return Compound(list(sols))
