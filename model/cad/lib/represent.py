"""Representative solids: clean, low-face build123d stand-ins measured from a
vendor mesh (section loops simplified and extruded slab by slab, revolved
rings), so a printed STL part shows its true outline, thickness and holes
without hundreds of triangle-derived faces."""
import math

import numpy as np


def section_loops(mesh, axis, value):
    """Closed loops (k x 2, the two other axes in order) of the mesh cut by the
    plane axis = value."""
    from trimesh.intersections import mesh_plane
    n = np.zeros(3)
    n[axis] = 1.0
    o = np.zeros(3)
    o[axis] = value
    seg = mesh_plane(mesh, plane_normal=n, plane_origin=o)
    keep = [i for i in range(3) if i != axis]
    return chain([s[:, keep] for s in seg])


def chain(segs, nd=3):
    key = lambda p: (round(float(p[0]), nd), round(float(p[1]), nd))
    adj = {}
    for s in segs:
        p, q = key(s[0]), key(s[1])
        if p == q:
            continue
        adj.setdefault(p, []).append(q)
        adj.setdefault(q, []).append(p)
    seen, loops = set(), []
    for start in adj:
        if start in seen:
            continue
        loop, prev, cur = [start], None, start
        seen.add(start)
        while True:
            nxt = [x for x in adj[cur] if x != prev and x not in seen]
            if not nxt:
                break
            prev, cur = cur, nxt[0]
            loop.append(cur)
            seen.add(cur)
        if len(loop) >= 3:
            loops.append(np.array(loop))
    return loops


def area(c):
    x, y = c[:, 0], c[:, 1]
    return 0.5 * (np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def simplify(c, tol):
    """Douglas-Peucker on a closed loop (k x 2)."""
    c = np.asarray(c, float)
    if len(c) <= 4:
        return c
    i0 = 0
    i1 = int(np.argmax(np.linalg.norm(c - c[0], axis=1)))

    def dp(p):
        if len(p) < 3:
            return p
        a, b = p[0], p[-1]
        ab = b - a
        L = np.linalg.norm(ab)
        if L < 1e-12:
            d = np.linalg.norm(p - a, axis=1)
        else:
            d = np.abs(ab[0] * (p[:, 1] - a[1]) - ab[1] * (p[:, 0] - a[0])) / L
        k = int(np.argmax(d))
        if d[k] <= tol:
            return np.array([a, b])
        return np.vstack([dp(p[:k + 1])[:-1], dp(p[k:])])
    first = dp(np.vstack([c[i0:i1 + 1]]))
    second = dp(np.vstack([c[i1:], c[:1]]))
    out = np.vstack([first[:-1], second[:-1]])
    return out


def circle(c, rel=0.03):
    """(centre, r) when the loop is a circle within rel, else None."""
    cen = c.mean(axis=0)
    r = np.linalg.norm(c - cen, axis=1)
    return (cen, float(r.mean())) if len(c) >= 10 and r.max() - r.min() < rel * r.mean() else None


def _rz_profile(mesh, cx, cy, ang):
    """(r, z) loop of a ring mesh (axis Z through (cx, cy)) cut by the half
    plane at angle ang (deg), collinear points dropped."""
    from trimesh.intersections import mesh_plane
    a = math.radians(ang)
    d = np.array([math.cos(a), math.sin(a), 0.0])
    seg = mesh_plane(mesh, plane_normal=[-math.sin(a), math.cos(a), 0.0], plane_origin=[cx, cy, 0.0])
    seg = seg - np.array([cx, cy, 0.0])
    seg = seg[(seg @ d).mean(axis=1) > 0]
    loops = chain([np.stack([s @ d, s[:, 2]], axis=1) for s in seg], nd=2)
    loop = max(loops, key=lambda c: abs(area(c)))
    return simplify(loop, 0.02)


def ring_from_mesh(mesh, cx, cy, lug_tol=1.0):
    """Revolved ring (axis Z through (cx, cy)) to the mesh's own measured cross
    section, cut opposite any radial lug, plus the lug as one extruded block
    (its outline at mid height, simplified). Returns a Solid in mesh coords."""
    from build123d import Axis, Polyline, Solid, extrude, make_face, revolve
    v = np.asarray(mesh.vertices, float) - np.array([cx, cy, 0.0])
    r = np.hypot(v[:, 0], v[:, 1])
    ang = np.degrees(np.arctan2(v[:, 1], v[:, 0])) % 360
    bins = np.floor(ang / 10).astype(int)
    rmax = np.array([r[bins == b].max() if (bins == b).any() else 0 for b in range(36)])
    r_ring = float(np.median(rmax[rmax > 0]))
    lug = v[r > r_ring + lug_tol]
    lang = math.degrees(math.atan2(lug[:, 1].mean(), lug[:, 0].mean())) if len(lug) else None
    cut = (lang + 180.0) if lang is not None else 45.0
    # cut through a vertex of the facetted ring so the profile sits on the true radius
    near = np.argmin(np.abs(((ang - cut + 180) % 360) - 180) + (r < r_ring - 50) * 1e9)
    cut = float(ang[near]) + 1e-4
    prof = _rz_profile(mesh, cx, cy, cut)
    if area(prof) < 0:
        prof = prof[::-1]
    # geometry built in place (no Location) so a later scale() scales the centre too
    face = make_face(Polyline(*[(cx + float(x), cy, float(z)) for x, z in prof], close=True))
    ring = revolve(face, Axis((cx, cy, 0.0), (0.0, 0.0, 1.0)), 360)
    if lang is not None:
        z0, z1 = float(lug[:, 2].min()), float(lug[:, 2].max())
        best = None
        for zm in np.linspace(z0 + 0.3, z1 - 0.3, 13):      # the lug's fullest outline (a clamp screw hole splits some levels)
            loops = section_loops(mesh, 2, float(zm))
            o = max(loops, key=lambda c: abs(area(c))) - np.array([cx, cy])
            ext = np.hypot(o[:, 0], o[:, 1]).max()
            if best is None or ext > best[0] + 1e-6:
                best = (ext, o)
        outer = best[1]
        rr = np.hypot(outer[:, 0], outer[:, 1])
        idx = np.where(rr > r_ring + 0.3)[0]
        # the lug's contiguous run of outline points (the loop may wrap)
        n = len(outer)
        start = next(i for i in idx if (i - 1) % n not in set(idx))
        run = []
        i = start
        while i % n in set(idx):
            run.append(outer[i % n])
            i += 1
        run = np.array(run)
        a0 = math.atan2(run[0][1], run[0][0])
        a1 = math.atan2(run[-1][1], run[-1][0])
        ri = r_ring - 1.0
        pts = np.vstack([[ri * math.cos(a0), ri * math.sin(a0)], run, [ri * math.cos(a1), ri * math.sin(a1)]])
        pts = simplify(pts, 0.15)
        if area(pts) < 0:
            pts = pts[::-1]
        blk = extrude(make_face(Polyline(*[(cx + float(x), cy + float(y), z0) for x, y in pts], close=True)),
                      amount=z1 - z0)
        ring = ring.fuse(blk).clean()
    sols = ring.solids()
    return Solid(sols[0].wrapped) if len(sols) == 1 else ring


def prism(loop, a0, a1, axis=2):
    """Extrusion of a 2D loop from axis = a0 to a1; the loop's coordinates are
    the two other axes in order ((x, y) for Z, (x, z) for Y, (y, z) for X)."""
    from build123d import Plane, Polyline, extrude, make_face
    loop = np.asarray(loop, float)
    if area(loop) < 0:
        loop = loop[::-1]
    e = np.eye(3)
    u, v = [e[i] for i in range(3) if i != axis]
    n = np.cross(u, v)                      # +axis for Z and X, -Y for axis Y
    pl = Plane(origin=tuple(e[axis] * a0), x_dir=tuple(u), z_dir=tuple(n))
    f = make_face(pl * Polyline(*[(float(x), float(y)) for x, y in loop], close=True))
    return extrude(f, amount=a1 - a0, dir=tuple(e[axis]))


def box_envelope(shape, keep_main_faces=400, vmin=1.0):
    """Representative envelope of a many-solid vendor STEP: the largest solid
    kept as is when it is at most keep_main_faces, every other solid with a
    bounding-box volume >= vmin as its bounding box, boxes inside another box
    dropped. One Compound."""
    from build123d import Box, Compound, Location, Solid
    sols = sorted(shape.solids(), key=lambda q: -q.volume)
    out = []
    main = sols[0]
    rest = sols
    if len(main.faces()) <= keep_main_faces:
        out.append(Solid(main.wrapped))
        rest = sols[1:]
    boxes = []
    for q in rest:
        b = q.bounding_box()
        lo, hi = np.array((b.min.X, b.min.Y, b.min.Z)), np.array((b.max.X, b.max.Y, b.max.Z))
        size = hi - lo
        if np.prod(np.maximum(size, 0.05)) < vmin:
            continue
        boxes.append((lo, hi))
    boxes.sort(key=lambda b: -np.prod(b[1] - b[0]))
    kept = []
    for lo, hi in boxes:
        if any(np.all(lo >= l2 - 0.01) and np.all(hi <= h2 + 0.01) for l2, h2 in kept):
            continue
        kept.append((lo, hi))
    for lo, hi in kept:
        size = np.maximum(hi - lo, 0.05)
        c = (lo + hi) / 2
        out.append(Solid(Box(*map(float, size)).moved(Location(tuple(map(float, c)))).wrapped))
    return Compound(out)
