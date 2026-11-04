"""Printability check for one printed part (`bin/cad printcheck <part>`).

Builds parts/<part>.py alone, rotates it by SPEC['print_rotate'] and drops it
on the bed (Z 0), exports out/parts/<part>/<part>.stl and .3mf (the 3MF is the
Bambu P2S print file), then checks:
- watertight: the tessellated mesh is closed and consistently wound
- min wall: ray thickness (inward along the surface normal to the opposite
  wall) at area-weighted samples on every face; structural walls >= 3.0 mm.
  A short ray is confirmed from the other side (from the hit point, inward
  along that wall's normal) and the larger of the two counts, so a ray that
  only grazes the rim of a hole is not read as a thin wall.
  Samples inside SPEC['nonstructural'] boxes (part coordinates, each with a
  reason) are reported but exempt. The thinnest spot is always reported.
- overhang: faces whose outward normal is within 45 deg of straight down (more
  than 45 deg past vertical), off the bed plane; area sum must be <= 50 mm2
- bed fit: bounding box within 240 x 240 x 240 (Bambu P2S)
- print face planar on the bed: the faces touching Z 0 are flat (normal -Z)
  and give at least 100 mm2 of contact
Every check prints one line; failures start with FAIL and the exit is non-zero.
"""
import importlib
import json
import os
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BED = (240.0, 240.0, 240.0)
MIN_WALL = 3.0
WALL_TOL = 0.02
OVERHANG_DEG = 45.0
OVERHANG_MAX_MM2 = 50.0
BED_MIN_MM2 = 100.0
TESS = (0.05, 0.2)
SAMPLES = 2500


def load_part(name):
    mod = importlib.import_module(f"parts.{name}")
    return mod, mod.build()


def print_pose(solid, spec):
    """Rotate by SPEC['print_rotate'] (deg about X, Y, Z) and put the lowest
    point on Z 0, centred on the bed in X and Y. Returns (solid, transform)
    where transform maps part coordinates to print coordinates."""
    from build123d import Axis, Location
    rx, ry, rz = spec.get("print_rotate", (0, 0, 0))
    s = solid
    for ax, ang in ((Axis.X, rx), (Axis.Y, ry), (Axis.Z, rz)):
        if ang:
            s = s.rotate(ax, ang)
    b = s.bounding_box()
    off = (-(b.min.X + b.max.X) / 2, -(b.min.Y + b.max.Y) / 2, -b.min.Z)
    s = s.moved(Location(off))
    return s, (rx, ry, rz, off)


def _rot(rx, ry, rz):
    def R(axis, a):
        a = np.radians(a)
        c, s = np.cos(a), np.sin(a)
        if axis == 0:
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        if axis == 1:
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return R(2, rz) @ R(1, ry) @ R(0, rx)


def mesh_of(solid, tol=TESS):
    verts, tris = solid.tessellate(*tol)
    v = np.array([tuple(p) for p in verts], float)
    f = np.array(tris, int)
    return v, f


def ray_hits(origins, dirs, tri, eps=1e-9, index=False):
    """Distance along each ray to the nearest triangle (Moller-Trumbore,
    brute force in chunks; inf when nothing is hit), and with index=True the
    hit triangle (-1 when nothing is hit)."""
    v0, e1, e2 = tri[:, 0], tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    out = np.full(len(origins), np.inf)
    idx = np.full(len(origins), -1)
    step = max(1, int(4e6 // max(1, len(tri))))
    for i in range(0, len(origins), step):
        o = origins[i:i + step, None, :]
        d = dirs[i:i + step, None, :]
        p = np.cross(d, e2[None])
        det = np.einsum("rtk,tk->rt", p, e1)
        ok = np.abs(det) > eps
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        s = o - v0[None]
        u = np.einsum("rtk,rtk->rt", s, p) * inv
        q = np.cross(s, e1[None])
        w = np.einsum("rtk,rtk->rt", d, q) * inv
        t = np.einsum("tk,rtk->rt", e2, q) * inv
        hit = ok & (u >= 0) & (w >= 0) & (u + w <= 1) & (t > 1e-4)
        t = np.where(hit, t, np.inf)
        out[i:i + step] = t.min(axis=1)
        idx[i:i + step] = np.where(np.isfinite(out[i:i + step]), t.argmin(axis=1), -1)
    return (out, idx) if index else out


def wall_thickness(v, f, n_samples=SAMPLES, seed=1):
    tri = v[f]
    cr = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area = np.linalg.norm(cr, axis=1) / 2
    nrm = cr / np.maximum(1e-12, 2 * area[:, None])
    rng = np.random.default_rng(seed)
    pick = rng.choice(len(f), size=n_samples, p=area / area.sum())
    a, b = rng.random(n_samples), rng.random(n_samples)
    flip = a + b > 1
    a[flip], b[flip] = 1 - a[flip], 1 - b[flip]
    t = tri[pick]
    pts = t[:, 0] + a[:, None] * (t[:, 1] - t[:, 0]) + b[:, None] * (t[:, 2] - t[:, 0])
    d = -nrm[pick]
    th, hit = ray_hits(pts + d * 1e-3, d, tri, index=True)
    short = np.where(np.isfinite(th) & (th < MIN_WALL) & (hit >= 0))[0]
    if len(short):
        q = pts[short] + d[short] * th[short, None]
        d2 = -nrm[hit[short]]
        back = ray_hits(q + d2 * 1e-3, d2, tri)
        th[short] = np.maximum(th[short], np.where(np.isfinite(back), back, 0.0))
    return pts, th


def watertight(v, f):
    """Closed (every edge shared by exactly two triangles, after welding
    vertices) and consistently wound (each shared edge used once each way)."""
    key = np.round(v / 1e-4).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    ff = inv.reshape(-1)[f]
    e = np.concatenate([ff[:, [0, 1]], ff[:, [1, 2]], ff[:, [2, 0]]])
    e = e[e[:, 0] != e[:, 1]]
    und = np.sort(e, axis=1)
    _, cnt = np.unique(und, axis=0, return_counts=True)
    closed = bool(np.all(cnt == 2))
    _, dcnt = np.unique(e, axis=0, return_counts=True)
    wound = bool(np.all(dcnt == 1))
    return closed and wound, dict(open_edges=int(np.sum(cnt == 1)), nonmanifold_edges=int(np.sum(cnt > 2)),
                                  misoriented=int(np.sum(dcnt > 1)))


def overhang(v, f, z_bed=0.0):
    tri = v[f]
    cr = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area = np.linalg.norm(cr, axis=1) / 2
    nz = cr[:, 2] / np.maximum(1e-12, 2 * area)
    on_bed = np.all(np.abs(tri[:, :, 2] - z_bed) < 1e-3, axis=1)
    lim = np.cos(np.radians(OVERHANG_DEG)) + 1e-4       # exactly 45 deg is allowed
    over = (-nz > lim) & ~on_bed
    worst = None
    if over.any():
        i = np.argmax(np.where(over, area, -1))
        worst = tri[i].mean(axis=0)
    bed = on_bed & (nz < -0.999)
    return float(area[over].sum()), worst, float(area[bed].sum()), int(over.sum())


def export_print_files(solid, name):
    from build123d import Mesher, export_stl
    d = os.path.join(ROOT, "out", "parts", name)
    os.makedirs(d, exist_ok=True)
    stl, tmf = os.path.join(d, f"{name}.stl"), os.path.join(d, f"{name}.3mf")
    export_stl(solid, stl, tolerance=0.02, angular_tolerance=0.15)
    m = Mesher()
    m.add_shape(solid, linear_deflection=0.02, angular_deflection=0.15)
    m.write(tmf)
    return dict(stl=stl, three_mf=tmf)


def _exempt(p_part, boxes):
    if not boxes:
        return np.zeros(len(p_part), bool)
    m = np.zeros(len(p_part), bool)
    for bx in boxes:
        lo, hi = np.array(bx["box"][:3]), np.array(bx["box"][3:])
        m |= np.all((p_part >= lo) & (p_part <= hi), axis=1)
    return m


def run(name):
    t0 = time.time()
    lines, fails = [], []
    mod, solid = load_part(name)
    spec = getattr(mod, "SPEC", {})
    posed, (rx, ry, rz, off) = print_pose(solid, spec)
    files = export_print_files(posed, name)
    v, f = mesh_of(posed)
    # 1. watertight
    wt, wd = watertight(v, f)
    (fails if not wt else lines).append(f"{'FAIL' if not wt else 'ok  '} watertight: {wt} {wd}")
    # 2. min wall
    pts, th = wall_thickness(v, f)
    R = _rot(rx, ry, rz)
    p_part = (pts - np.array(off)) @ R            # back to part (assembly) coordinates
    zref = spec.get("nonstructural_z0", 0.0)
    ex = _exempt(p_part - np.array([0, 0, zref]), spec.get("nonstructural", []))
    finite = np.isfinite(th)
    thin = finite & (th < MIN_WALL - WALL_TOL)
    bad = thin & ~ex
    i_all = int(np.argmin(np.where(finite, th, np.inf)))
    lines_wall = (f"min wall {th[i_all]:.2f} mm at part {np.round(p_part[i_all], 2).tolist()}"
                  f"{' (exempt: non-structural)' if ex[i_all] else ''}; {int(thin.sum())}/{len(th)} samples under "
                  f"{MIN_WALL}, {int(bad.sum())} of them structural")
    if bad.any():
        i = int(np.argmin(np.where(bad, th, np.inf)))
        fails.append(f"FAIL min wall {th[i]:.2f} mm < {MIN_WALL} on a structural wall at part "
                     f"{np.round(p_part[i], 2).tolist()} ({int(bad.sum())} samples); {lines_wall}")
    else:
        lines.append("ok   " + lines_wall)
    # 3. overhang
    oa, worst, bed_area, n_over = overhang(v, f)
    msg = f"overhang > {OVERHANG_DEG:.0f} deg: {oa:.1f} mm2 over {n_over} triangles (limit {OVERHANG_MAX_MM2})"
    if worst is not None:
        wp = (worst - np.array(off)) @ R
        msg += f", largest near part {np.round(wp, 1).tolist()}"
    (fails if oa > OVERHANG_MAX_MM2 else lines).append(("FAIL " if oa > OVERHANG_MAX_MM2 else "ok   ") + msg)
    # 4. bed fit
    b = posed.bounding_box()
    size = (b.max.X - b.min.X, b.max.Y - b.min.Y, b.max.Z - b.min.Z)
    fit = all(s <= m + 1e-6 for s, m in zip(size, BED))
    (fails if not fit else lines).append(f"{'FAIL' if not fit else 'ok  '} bed fit {tuple(round(s, 1) for s in size)} "
                                         f"within {BED}")
    # 5. print face planar on the bed
    planar = _bed_faces_planar(posed)
    okbed = planar and bed_area >= BED_MIN_MM2
    (fails if not okbed else lines).append(f"{'FAIL' if not okbed else 'ok  '} print face {spec.get('print_face')}: "
                                           f"planar={planar}, bed contact {bed_area:.0f} mm2")
    dt = round(time.time() - t0, 2)
    res = dict(command="printcheck", part=name, ok=not fails, seconds=dt, files=files, fails=fails, lines=lines,
               min_wall_mm=round(float(th[i_all]), 3), overhang_mm2=round(oa, 2), bed_mm2=round(bed_area, 1),
               size_mm=[round(s, 2) for s in size], triangles=int(len(f)), samples=int(len(th)))
    return res


def _bed_faces_planar(posed):
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Plane
    faces = [fc for fc in posed.faces() if fc.bounding_box().max.Z < 1e-3]
    if not faces:
        return False
    for fc in faces:
        if BRepAdaptor_Surface(fc.wrapped).GetType() != GeomAbs_Plane or fc.normal_at().Z > -0.999:
            return False
    return True


if __name__ == "__main__":
    import sys
    print(json.dumps(run(sys.argv[1])))
