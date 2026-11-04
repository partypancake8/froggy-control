"""B-rep "ugly" detectors for any STEP. Findings only, never a failure: exit 0 always.

Per leaf part (see stepio.load), with the kind from kinds.kind_of (name prefix fp_/fp9_ printed,
lp_ laser, mp_ machined, v_ vendor; keywords otherwise; --kind overrides):

  sharp_edges       printed: convex edges with a dihedral angle >= SHARP_DEG (a square crease, no
                    fillet or chamfer), hole rims excluded. Count and total length.
  zero_fillet       printed / laser: not one blend face (a curved face tangent to a neighbour).
  raw_block         any non-vendor part: exactly 6 planar faces, every side >= RAW_MIN mm.
  sliver            non-vendor: thinnest OBB side < SLIVER_MIN mm, or longest / middle > NEEDLE.
  outside_envelope  with --envelope: how far the part pokes outside the box.
  mixed_fasteners   per sub-assembly (second level of the tree, else the file): more than one
                    thread size or head style among names carrying M-sizes.
  misaligned_holes  printed / laser / machined: parallel holes whose centres miss a shared row or
                    line by more than ALIGN_MIN and less than ALIGN_MAX mm.
  thin_wall         printed: opposed planar faces, a hole and a parallel planar face, or a hole in a
                    coaxial outer cylinder closer than --min-wall (material between, measured).
  no_name           empty or generic names (Solid, Body, Part, COMPOUND, Open CASCADE ...).
"""
import itertools
import json
import math
import re

import numpy as np

from .brep import (classify_edges, cylinder_info, edge_faces, edge_info, faces_of, face_normal_mid, is_hole_face,
                   obb_dims, surf_type)
from .kinds import kind_of
from .stepio import clean

SHARP_DEG = 70.0          # 90 deg crease counts; 45 deg chamfer faces and 60 deg chamfer corners do not
SHARP_MIN_LEN = 1.0
RAW_MIN = 3.0
SLIVER_MIN = 0.4
NEEDLE = 60.0
ALIGN_MIN, ALIGN_MAX = 0.1, 1.0   # under 0.1 mm nobody can see it; over 1 mm it is a different row
MIN_WALL = 2.0
MAX_FACES_DETAIL = 1500   # skip the per-edge and per-face-pair checks above this (vendor-sized meshes)

FIX = {
    "sharp_edges": "run cadkit.idioms.finish(solid, outer_r=1.0, inner_r=0.5, print_face='-Z') or build it from an idiom",
    "zero_fillet": "round the outline in 2D (RectangleRounded / fillet vertices) before extruding, or finish() it",
    "raw_block": "replace the box with printed_block / filleted_plate, or a shape that follows what it holds",
    "sliver": "remove the sliver (a boolean leftover) or thicken it to a real wall",
    "outside_envelope": "pull the part inside the envelope or house it with a bracket (DL1)",
    "mixed_fasteners": "pick one thread size and head style for the sub-assembly (DL10)",
    "misaligned_holes": "snap the hole centres to one row (use rect_pattern / circle_pattern / mirrored)",
    "thin_wall": "thicken to the minimum wall or move the hole 2 x d from the edge",
    "no_name": "give the part a readable name: what it is plus part number or material",
}

_GENERIC = re.compile(r"(?i)^(|solid|body|part|shape|compound|component|unnamed|untitled|object|item|"
                      r"open[\s_]*cascade.*|product|\=\>.*|\[.*\]|nauo\d*|.*\.st(e)?p)[\s_\-]*\d*$")
_MSIZE = re.compile(r"(?i)(?<![a-z0-9])m(\d+(?:[._]\d)?)(?![0-9])")
_HEADS = (("button", r"(?i)bhcs|button"), ("countersunk", r"(?i)fhcs|countersunk|csk|flat[\s_]head"),
          ("shcs", r"(?i)shcs|socket[\s_]head|cap[\s_]screw"), ("hex", r"(?i)hex[\s_]bolt|hex[\s_]head"),
          ("pan", r"(?i)pan[\s_]head"))
_FUNCTION = re.compile(r"(?i)thumb|set[\s_]screw|grub|shoulder|standoff|spacer")


FIX_LASER = {
    "sharp_edges": "round the outline corners in 2D before extruding: R 3 outside, R 6 on plates over 100 mm, "
                   "R 2 inside (filleted_plate, or fillet the sketch vertices)",
    "zero_fillet": "round the outline corners in 2D before extruding (DL8)",
}


def _f(part, kind, finding, measure, **extra):
    fix = FIX_LASER.get(finding, FIX[finding]) if kind == "laser" else FIX[finding]
    d = dict(part=part.display, path="/".join(clean(p) for p in part.path[1:]), part_kind=kind, finding=finding,
             measure=measure, fix=fix)
    d.update(extra)
    return d


# --------------------------------------------------------------------------------------------- per solid


def _sharp(solid_w):
    n, total = 0, 0.0
    for _, info, _, _ in classify_edges(solid_w, min_len=SHARP_MIN_LEN):
        if info["convex"] and info["dihedral_deg"] >= SHARP_DEG:
            n += 1
            total += info["length"]
    return n, total


def _laser_corners(solid_w):
    """Square outline corners of a flat part: convex edges at >= SHARP_DEG that run through the
    thickness (parallel to the normal of the largest planar face). Count and total length."""
    from build123d import Edge
    best = None
    for f in faces_of(solid_w):
        if surf_type(f) != "plane":
            continue
        from OCP.BRepGProp import BRepGProp
        from OCP.GProp import GProp_GProps
        g = GProp_GProps()
        BRepGProp.SurfaceProperties_s(f, g)
        if best is None or g.Mass() > best[0]:
            best = (g.Mass(),) + tuple(face_normal_mid(f))
    if best is None or best[1] is None:
        return 0, 0.0
    n, c0 = best[1] / np.linalg.norm(best[1]), best[2]
    # sheet thickness: the nearest antiparallel planar face behind the largest face
    t = None
    for f in faces_of(solid_w):
        if surf_type(f) != "plane":
            continue
        fn, fc = face_normal_mid(f)
        if fn is None or fn @ n > -0.999:
            continue
        gap = -((fc - c0) @ n)
        if gap > 0.05 and (t is None or gap < t):
            t = gap
    if t is None:
        return 0, 0.0
    cnt, total = 0, 0.0
    for e, info, _, _ in classify_edges(solid_w, min_len=0.2):
        if not info["convex"] or info["dihedral_deg"] < SHARP_DEG:
            continue
        E = Edge(e)
        d = np.array(tuple(E.position_at(1))) - np.array(tuple(E.position_at(0)))
        if np.linalg.norm(d) > 1e-6 and abs(abs(d @ n) / np.linalg.norm(d) - 1) < 1e-3 \
                and abs(info["length"] - t) < 0.05 * t + 0.01:
            cnt += 1
            total += info["length"]
    return cnt, total


def _blend_faces(solid_w):
    """Curved faces with at least one tangent (dihedral < 5 deg) edge to a neighbour."""
    blends = []
    for e, f1, f2 in edge_faces(solid_w):
        t1, t2 = surf_type(f1), surf_type(f2)
        if t1 == "plane" and t2 == "plane":
            continue
        info = edge_info(e, f1, f2)
        if info is None or info["dihedral_deg"] > 5.0:
            continue
        for f, t in ((f1, t1), (f2, t2)):
            if t != "plane" and not is_hole_face(f) and not any(f.IsSame(g) for g in blends):
                blends.append(f)
    return len(blends)


def _holes(solid_w):
    """Unique holes: concave cylinders, merged by axis line and radius."""
    out = []
    for f in faces_of(solid_w):
        if not is_hole_face(f):
            continue
        ci = cylinder_info(f)
        if ci is None:
            continue
        d, p0 = ci["axis"], ci["p0"]
        foot = p0 - (p0 @ d) * d           # axis line point closest to the origin
        dup = False
        for h in out:
            if abs(abs(h["axis"] @ d) - 1) < 1e-6 and np.linalg.norm(np.cross(h["foot"] - foot, d)) < 1e-4 \
                    and abs(h["radius"] - ci["radius"]) < 1e-4:
                dup = True
                break
        if not dup:
            out.append(dict(axis=d, foot=foot, radius=ci["radius"], face=f))
    return out


def _misaligned(holes):
    """(pairs or triples that nearly share a row, worst offset mm)."""
    bad, worst = 0, 0.0
    groups = []
    for h in holes:   # one row = one fastener: same axis direction and same diameter
        for g in groups:
            if abs(abs(g[0]["axis"] @ h["axis"]) - 1) < 1e-4 and abs(g[0]["radius"] - h["radius"]) < 0.05:
                g.append(h)
                break
        else:
            groups.append([h])
    for g in groups:
        if len(g) < 2:
            continue
        ax = g[0]["axis"]
        # in-plane basis: global axes projected into the plane, then made orthonormal
        cands = [np.eye(3)[i] - (np.eye(3)[i] @ ax) * ax for i in range(3)]
        cands = sorted(cands, key=lambda v: -np.linalg.norm(v))
        u = cands[0] / np.linalg.norm(cands[0])
        v = np.cross(ax, u)
        pts = [np.array([h["foot"] @ u, h["foot"] @ v]) for h in g]
        seen = set()
        for i, j in itertools.combinations(range(len(pts)), 2):
            w = pts[j] - pts[i]
            for a_, b_ in ((0, 1), (1, 0)):
                if abs(w[a_]) >= 3.0 and ALIGN_MIN < abs(w[b_]) < ALIGN_MAX:
                    bad += 1
                    worst = max(worst, abs(w[b_]))
                    seen.add((i, j))
        for i, j, k in itertools.combinations(range(len(pts)), 3):
            a, b, c = pts[i], pts[j], pts[k]
            for p, q, r in ((a, b, c), (a, c, b), (b, c, a)):
                L = np.linalg.norm(q - p)
                if L < 3.0:
                    continue
                along = (r - p) @ (q - p) / L
                if not 3.0 <= along <= L - 3.0:      # r sits between p and q: the middle of a row
                    continue
                dist = abs((q - p)[0] * (r - p)[1] - (q - p)[1] * (r - p)[0]) / L
                if ALIGN_MIN < dist < ALIGN_MAX and (i, j) not in seen and (i, k) not in seen and (j, k) not in seen:
                    bad += 1
                    worst = max(worst, dist)
                    seen.update({(i, j), (i, k), (j, k)})
                    break
    return bad, worst


def _inside(solid_w, p, tol=1e-4):
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_IN
    c = BRepClass3d_SolidClassifier(solid_w, gp_Pnt(*map(float, p)), tol)
    return c.State() == TopAbs_IN


def _dist(f1, f2):
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    d = BRepExtrema_DistShapeShape(f1, f2)
    if not d.IsDone() or d.NbSolution() == 0:
        return None, None, None
    p1, p2 = d.PointOnShape1(1), d.PointOnShape2(1)
    return d.Value(), np.array([p1.X(), p1.Y(), p1.Z()]), np.array([p2.X(), p2.Y(), p2.Z()])


def _thin(solid_w, min_wall):
    """Smallest measured wall below min_wall (mm), with what it was measured between, else None."""
    faces = faces_of(solid_w)
    planes, holes, outers = [], [], []
    for f in faces:
        t = surf_type(f)
        if t == "plane":
            n, c = face_normal_mid(f)
            if n is not None:
                planes.append((f, n / np.linalg.norm(n), c))
        elif t == "cylinder":
            ci = cylinder_info(f)
            if ci is None:
                continue
            (holes if is_hole_face(f) else outers).append((f, ci))
    best = None

    def take(v, what, p1, p2):
        """Material between p1 and p2? Closest points often sit on face boundaries, so probe the
        midpoint and points nudged 0.3 mm sideways (perpendicular to the wall direction)."""
        nonlocal best
        if not (1e-3 < v < min_wall) or (best is not None and v >= best[0]):
            return
        mid = (p1 + p2) / 2
        dvec = p2 - p1
        dvec = dvec / (np.linalg.norm(dvec) or 1.0)
        a = np.cross(dvec, [1.0, 0, 0] if abs(dvec[0]) < 0.9 else [0, 1.0, 0])
        a /= np.linalg.norm(a)
        b = np.cross(dvec, a)
        probes = [mid] + [mid + s * 0.3 * e for e in (a, b) for s in (1, -1)] + \
                 [mid + 0.3 * (s1 * a + s2 * b) for s1 in (1, -1) for s2 in (1, -1)]
        if any(_inside(solid_w, q) for q in probes):
            best = (v, what)
    for (f1, n1, c1), (f2, n2, c2) in itertools.combinations(planes, 2):
        if n1 @ n2 > -0.999:
            continue
        gap = (c2 - c1) @ n1
        if gap >= 0 or -gap >= min_wall:          # wall: f2 lies behind f1 (material between)
            continue
        v, p1, p2 = _dist(f1, f2)
        if v is not None and abs(v + gap) < 1e-3:
            take(v, "opposed faces", p1, p2)
    for (fh, h) in holes:
        for (fp, n, c) in planes:
            if abs(n @ h["axis"]) > 1e-3:
                continue
            off = (c - h["p0"]) @ n
            if off <= 0:
                continue
            wall = off - h["radius"]
            if wall <= 0 or wall >= min_wall:
                continue
            v, p1, p2 = _dist(fh, fp)
            if v is not None and abs(v - wall) < 0.05:
                take(v, "hole to face", p1, p2)
        for (fo, o) in outers:
            if abs(abs(o["axis"] @ h["axis"]) - 1) > 1e-6:
                continue
            if np.linalg.norm(np.cross(o["p0"] - h["p0"], h["axis"])) > 1e-3:
                continue
            wall = o["radius"] - h["radius"]
            if 0 < wall < min_wall:
                v, p1, p2 = _dist(fh, fo)
                if v is not None and abs(v - wall) < 0.05:
                    take(v, "hole in boss", p1, p2)
    return best


# --------------------------------------------------------------------------------------------- per file


def _fastener_family(name):
    m = _MSIZE.search(name or "")
    if not m or _FUNCTION.search(name):
        return None
    size = "M" + m.group(1).replace("_", ".")
    style = None
    for st, rx in _HEADS:
        if re.search(rx, name):
            style = st
            break
    return size, style


def lint_parts(parts, kind=None, envelope=None, min_wall=MIN_WALL, source=""):
    findings = []
    counted = dict(parts=len(parts), solids=0, by_kind={})
    for p in parts:
        k = kind_of(p.name, p.path, kind)
        counted["by_kind"][k] = counted["by_kind"].get(k, 0) + 1
        sols = p.shape.solids()
        counted["solids"] += len(sols)
        if _GENERIC.match(clean(p.name) or ""):
            findings.append(_f(p, k, "no_name", f"name {p.name!r}"))
        if envelope is not None:
            b = p.shape.bounding_box()
            mn, mx = np.array(tuple(b.min)), np.array(tuple(b.max))
            over = max(float(np.max(np.asarray(envelope[:3]) - mn)), float(np.max(mx - np.asarray(envelope[3:]))))
            if over > 0.01:
                findings.append(_f(p, k, "outside_envelope", f"{over:.1f} mm outside", mm=round(over, 2)))
        if k in ("vendor", "fastener"):
            continue
        for s in sols:
            w = s.wrapped
            nf = len(s.faces())
            if len(sols) == 1 and nf == 6 and all(f.geom_type.name == "PLANE" for f in s.faces()):
                dims = obb_dims(w)
                if dims[0] >= RAW_MIN:
                    findings.append(_f(p, k, "raw_block", f"6 planar faces, {dims[2]:.1f} x {dims[1]:.1f} x {dims[0]:.1f} mm"))
            if not re.search(r"(?i)wire|cable|harness|net|film|label|tape|sticker|decal", p.name):
                dims = obb_dims(w)
                if (dims[0] < SLIVER_MIN and dims[2] > 5 * SLIVER_MIN) or (dims[1] > 0 and dims[2] / dims[1] > NEEDLE):
                    findings.append(_f(p, k, "sliver", f"OBB {dims[2]:.2f} x {dims[1]:.2f} x {dims[0]:.2f} mm"))
            if nf > MAX_FACES_DETAIL:
                continue
            if k == "printed":
                n, total = _sharp(w)
                if n:
                    findings.append(_f(p, k, "sharp_edges", f"{n} sharp convex edges, {total:.0f} mm total",
                                       count=n, length_mm=round(total, 1)))
            if k == "laser":
                n, total = _laser_corners(w)
                if n:
                    findings.append(_f(p, k, "sharp_edges", f"{n} square outline corners (DL8: R 3 minimum)",
                                       count=n, length_mm=round(total, 1)))
                if n and _blend_faces(w) == 0:
                    findings.append(_f(p, k, "zero_fillet", f"0 blend faces of {nf}, outline has corners"))
            if k == "printed" and _blend_faces(w) == 0:
                findings.append(_f(p, k, "zero_fillet", f"0 blend faces of {nf}"))
            if k in ("printed", "laser", "machined"):
                hs = _holes(w)
                if len(hs) >= 2:
                    bad, worst = _misaligned(hs)
                    if bad:
                        findings.append(_f(p, k, "misaligned_holes",
                                           f"{bad} near-miss rows of {len(hs)} holes, worst {worst:.2f} mm off line",
                                           count=bad, worst_mm=round(worst, 3)))
            if k == "printed":
                t = _thin(w, min_wall)
                if t:
                    findings.append(_f(p, k, "thin_wall", f"{t[0]:.2f} mm wall ({t[1]}) < {min_wall:g} mm",
                                       mm=round(t[0], 3)))
    # fasteners per sub-assembly
    groups = {}
    for p in parts:
        fam = _fastener_family(p.name)
        if fam is None or kind_of(p.name, p.path) not in ("fastener",):
            continue
        key = "/".join(clean(x) for x in p.path[1:3]) or "(file)"
        groups.setdefault(key, []).append(fam)
    for key, fams in groups.items():
        sizes = sorted({s for s, _ in fams})
        heads = sorted({h for _, h in fams if h})
        if len(sizes) > 1 or len(heads) > 1:
            findings.append(dict(part=key, path=key, part_kind="sub-assembly", finding="mixed_fasteners",
                                 measure=f"sizes {', '.join(sizes)}; heads {', '.join(heads) or '-'}",
                                 fix=FIX["mixed_fasteners"], sizes=sizes, heads=heads))
    counts = {}
    for f in findings:
        counts[f["finding"]] = counts.get(f["finding"], 0) + 1
    return dict(source=source, parts=counted["parts"], solids=counted["solids"], kinds=counted["by_kind"],
                findings=findings, counts=counts)


def lint_step(path, kind=None, envelope=None, min_wall=MIN_WALL):
    from .stepio import load
    return lint_parts(load(path), kind=kind, envelope=envelope, min_wall=min_wall, source=str(path))


def lint_shape(shape, name=None, kind=None, envelope=None, min_wall=MIN_WALL):
    """Lint an in-memory build123d shape (one part), e.g. in a test or inside a build script."""
    from .stepio import Part
    p = Part(name=name if name is not None else (getattr(shape, "label", "") or ""), path=[], shape=shape)
    return lint_parts([p], kind=kind, envelope=envelope, min_wall=min_wall, source=p.name)


def format_text(rep):
    lines = [f"cadkit lint {rep['source']}: {rep['parts']} parts, {rep['solids']} solids; kinds "
             + ", ".join(f"{k} {v}" for k, v in sorted(rep["kinds"].items()))]
    for f in rep["findings"]:
        lines.append(f"  [{f['finding']}] {f['part']} ({f['part_kind']}): {f['measure']} -> {f['fix']}")
    if rep["counts"]:
        lines.append("findings by kind: " + ", ".join(f"{k} {v}" for k, v in sorted(rep["counts"].items())))
    else:
        lines.append("findings: none")
    return "\n".join(lines)


def to_json(rep):
    return json.dumps(rep, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
