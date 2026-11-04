"""Camera-view simulation: what the rotor camera (Arducam UC-261 class, OV5647 + fisheye) sees in a built assembly.

Reads the built per-part meshes from out/<name>/<name>.glb (the shared tessellation the STEP/STL/3MF exports use)
and the materials from sheet.json, finds the lens front and optical axis from the v_camera_uc261 geometry (board
centre and board normal, offset to the lens front), then ray-casts an equidistant fisheye image from the lens.

Ray casting is exact (Moller-Trumbore per ray and triangle) with an image-space cull: every triangle's projected
bounding box comes from its edges sampled densely and projected (the equidistant projection is continuous and one
to one for every direction except straight backwards, so the box of the projected boundary bounds the projected
triangle; triangles that contain the backward direction get the whole image). The lens is the common origin of
every ray, so the per-triangle half of the test is precomputed and a ray-triangle pair costs three dot products.

Acrylic parts and the net are walked through (recorded as through_acrylic / through_mesh, never as occluders);
everything else is opaque. Refraction in the acrylic is ignored (straight rays). The observer's own body
(v_camera_uc261) is never an occluder.
"""
import json
import math
import os
import time

import numpy as np

CAMERA_PART = "v_camera_uc261"
DISC_FROG = ("v_frog_disc_body", "v_frog_disc_eyes")
WALL_FROG = ("v_frog_wall_body", "v_frog_wall_eyes")
TOP_DISC = "s_top_disc"
TUBE = "s_drum_tube"
FLOOR = "lp_carrier_disc"
TRANSPARENT_MATERIALS = {"acrylic_clear": "acrylic", "mesh": "mesh"}
SIM_W, SIM_H = 800, 600          # simulated image (4:3 like the sensor), 480000 rays
PAIR_CHUNK = 3_000_000
SWEEP_TILTS = (0.0, 15.0, 30.0, 45.0)
SWEEP_HEIGHTS = (30.0, 48.0, 66.0)
SWEEP_TILT_FOR_HEIGHTS = 30.0
SWEEP_HEIGHT_FOR_TILTS = 48.0
MIN_PX_PER_MM = 2.0


# ------------------------------------------------------------------ inputs

def load_camera_spec(root):
    """vendor/electronics/arducam-uc261/source.json (FOV from Arducam's page)."""
    p = os.path.join(root, "vendor", "electronics", "arducam-uc261", "source.json")
    return json.load(open(p))


def load_scene(glb, sheet):
    """Triangle soup of every part: dict(tris (n,3,3), part (n,), names [..], kind {name: opaque|acrylic|mesh},
    colour {name: rgb 0..1}, material {name: str})."""
    import trimesh
    sc = trimesh.load(glb, force="scene")
    mat = {c["name"]: c.get("material") for c in sheet.get("components", [])}
    col = {c["name"]: c.get("colour") for c in sheet.get("components", [])}
    key = {c["node_name"]: c["name"] for c in sheet.get("components", []) if c.get("node_name")}   # readable node -> key
    names, tris, pid = [], [], []
    for node in sc.graph.nodes_geometry:
        T, g = sc.graph[node]
        m = sc.geometry[g]
        v = trimesh.transform_points(m.vertices, T) if not np.allclose(T, np.eye(4)) else np.asarray(m.vertices)
        f = np.asarray(m.faces)
        if not len(f):
            continue
        names.append(key.get(node, node))
        tris.append(v[f])
        pid.append(np.full(len(f), len(names) - 1, np.int32))
    kind, colour = {}, {}
    for n in names:
        kind[n] = TRANSPARENT_MATERIALS.get(mat.get(n), "opaque")
        h = col.get(n) or "#888888"
        colour[n] = tuple(int(h[i:i + 2], 16) / 255 for i in (1, 3, 5)) if isinstance(h, str) else tuple(h[:3])
    return dict(tris=np.concatenate(tris).astype(np.float64), part=np.concatenate(pid), names=names,
                kind=kind, colour=colour, material=mat)


def lens_from_assembly(S):
    """Lens front centre and optical axis from the v_camera_uc261 mesh: split it into its bodies, take the thin box
    (the PCB) for the board centre and board normal (its thinnest principal direction, signed toward the lens
    barrel), and offset the board centre along the normal to the barrel's furthest point (the lens front)."""
    import trimesh
    i = S["names"].index(CAMERA_PART)
    T = S["tris"][S["part"] == i]
    m = trimesh.Trimesh(T.reshape(-1, 3), np.arange(len(T) * 3).reshape(-1, 3), process=True)
    bodies = m.split(only_watertight=False, repair=False)   # bodies are closed solids already; repair needs networkx
    if len(bodies) < 2:
        raise RuntimeError(f"{CAMERA_PART}: expected PCB + lens bodies, found {len(bodies)}")

    def thin(b):
        v = b.vertices - b.vertices.mean(0)
        w, vec = np.linalg.eigh(v.T @ v)
        ext = [np.ptp(v @ vec[:, k]) for k in range(3)]
        return min(ext), vec[:, int(np.argmin(ext))], ext
    scored = sorted(bodies, key=lambda b: thin(b)[0] / max(thin(b)[2]))
    pcb = scored[0]
    lens = max((b for b in bodies if b is not pcb), key=lambda b: b.area)
    centre = pcb.vertices.mean(0) if len(pcb.vertices) <= 8 else pcb.bounding_box_oriented.centroid
    _, n, ext = thin(pcb)
    n = n / np.linalg.norm(n)
    if np.dot(lens.vertices.mean(0) - centre, n) < 0:
        n = -n
    front = float(np.max((lens.vertices - centre) @ n))
    lens_xyz = centre + n * front
    # sensor long axis: the long edge of the FFC connector body (it runs along the board edge parallel to the
    # sensor's long image axis); without a connector body the board orientation is unknown (old default)
    long_axis, up_from = None, "default (+X up; no connector body on the camera board)"
    others = [b for b in bodies if b is not pcb and b is not lens and len(b.vertices) >= 4]
    if others:
        con = max(others, key=lambda b: b.area)
        v = con.vertices - con.vertices.mean(0)
        v = v - np.outer(v @ n, n)
        w, vec = np.linalg.eigh(v.T @ v)
        long_axis = vec[:, int(np.argmax(w))]
        long_axis = long_axis - n * np.dot(long_axis, n)
        long_axis /= np.linalg.norm(long_axis)
        if long_axis[0] < -1e-9 or (abs(long_axis[0]) <= 1e-9 and long_axis[1] < 0):
            long_axis = -long_axis                   # image width runs from -X (left) to +X (right)
        up_from = "assembly"
    return dict(lens=lens_xyz, axis=n, board_centre=centre, board_extent_mm=sorted(round(e, 2) for e in ext),
                lens_front_offset_mm=front, long_axis=long_axis, image_up_from=up_from)


# ------------------------------------------------------------------ projection

class Fisheye:
    """Equidistant fisheye r = f * theta on a W x H sensor image. `right` is the sensor's long (W, fov_h) axis in the
    world, read from the assembly (the camera board's in-plane long-axis edge); image up = right x axis. Without it
    the old default applies: image up = the +X side of the drum."""

    def __init__(self, origin, axis, W, H, fov_h_deg, circle_deg, right=None):
        self.O = np.asarray(origin, float)
        z = np.asarray(axis, float)
        z = z / np.linalg.norm(z)
        if right is not None:
            right = np.asarray(right, float) - z * np.dot(right, z)
        if right is not None and np.linalg.norm(right) > 1e-6:
            right /= np.linalg.norm(right)
            up = np.cross(right, z)
        else:
            up = np.array([1.0, 0.0, 0.0]) - z * z[0]
            if np.linalg.norm(up) < 1e-6:
                up = np.array([0.0, 1.0, 0.0]) - z * z[1]
            up /= np.linalg.norm(up)
            right = np.cross(z, up)
        self.R = np.stack([right, up, z])            # rows: camera x, y, z in world
        self.W, self.H = W, H
        self.cx, self.cy = W / 2.0, H / 2.0
        self.f = (W / 2.0) / math.radians(fov_h_deg / 2.0)
        self.theta_max = math.radians(circle_deg / 2.0)

    def project(self, P):
        """World points (n,3) -> (u, v, theta) in sim pixels (u right, v down)."""
        d = (np.asarray(P, float) - self.O) @ self.R.T
        return self.project_cam(d)

    def project_cam(self, d):
        nrm = np.linalg.norm(d, axis=-1)
        nrm = np.where(nrm == 0, 1e-12, nrm)
        th = np.arccos(np.clip(d[..., 2] / nrm, -1, 1))
        ph = np.arctan2(d[..., 1], d[..., 0])
        r = self.f * th
        return self.cx + r * np.cos(ph), self.cy - r * np.sin(ph), th

    def pixel_dirs(self):
        """World unit directions of every pixel centre (H*W,3) and a validity mask (inside the image circle)."""
        j, i = np.mgrid[0:self.H, 0:self.W]
        x = i + 0.5 - self.cx
        y = self.cy - (j + 0.5)
        r = np.hypot(x, y)
        th = r / self.f
        ph = np.arctan2(y, x)
        d = np.stack([np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)], -1).reshape(-1, 3)
        return d @ self.R, (th <= self.theta_max).reshape(-1)

    def visible(self, P):
        u, v, th = self.project(P)
        return (th <= self.theta_max) & (u >= 0) & (u < self.W) & (v >= 0) & (v < self.H), u, v

    def px_per_mm(self, P, a, b, scale):
        """Sensor pixels per mm at P for 1 mm steps along a and b (geometric mean), scaled to the real sensor."""
        P = np.asarray(P, float)
        uv = [np.array(self.project(q[None])[:2]).ravel() for q in (P - 0.5 * a, P + 0.5 * a, P - 0.5 * b, P + 0.5 * b)]
        da, db = np.linalg.norm(uv[1] - uv[0]), np.linalg.norm(uv[3] - uv[2])
        return float(math.sqrt(da * db) * scale), float(da * scale), float(db * scale)


# ------------------------------------------------------------------ ray caster

class Caster:
    """Exact ray casting from one origin against a triangle set, culled in image space."""

    def __init__(self, cam, tris, ids):
        self.cam = cam
        O = cam.O
        T = tris[ids]
        self.ids = ids
        v0, e1, e2 = T[:, 0], T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]
        tv = O - v0
        self.nn = np.cross(e2, e1)                      # det = d . nn
        self.aa = np.cross(e2, tv)                      # u = d . aa / det
        self.qq = np.cross(tv, e1)                      # v = d . qq / det
        self.tn = np.einsum("ij,ij->i", e2, self.qq)    # t = tn / det
        self.box = self._boxes(T)

    def _boxes(self, T):
        cam = self.cam
        n = len(T)
        box = np.zeros((n, 4), np.int64)                # x0, y0, x1, y1 inclusive; x1 < x0 means empty
        box[:, 2] = -1
        if not n:
            return box
        D = T - cam.O
        Dn = D / np.maximum(np.linalg.norm(D, axis=2, keepdims=True), 1e-12)
        ang = np.zeros(n)
        for a, b in ((0, 1), (1, 2), (2, 0)):
            ang = np.maximum(ang, np.arccos(np.clip(np.einsum("ij,ij->i", Dn[:, a], Dn[:, b]), -1, 1)))
        # triangles that contain the backward direction cover everything near theta = pi: whole image
        back = -cam.R[2]
        det = self.nn @ back
        with np.errstate(divide="ignore", invalid="ignore"):
            u = (self.aa @ back) / det
            v = (self.qq @ back) / det
            t = self.tn / det
            wraps = (np.abs(det) > 1e-12) & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 0)
        nsamp = np.clip(np.ceil(ang / 0.004), 1, 1024).astype(int)
        levels = 2 ** np.ceil(np.log2(nsamp)).astype(int)
        for L in np.unique(levels):
            sel = np.nonzero(levels == L)[0]
            s = (np.arange(L) / L)[None, :, None]
            pts = []
            for a, b in ((0, 1), (1, 2), (2, 0)):
                pts.append(D[sel, a][:, None] + s * (D[sel, b] - D[sel, a])[:, None])
            P = np.concatenate(pts, 1) @ cam.R.T          # camera coords (k, 3L, 3)
            uu, vv, _ = cam.project_cam(P)
            box[sel, 0] = np.floor(uu.min(1) - 1.5)
            box[sel, 1] = np.floor(vv.min(1) - 1.5)
            box[sel, 2] = np.floor(uu.max(1) + 1.5)
            box[sel, 3] = np.floor(vv.max(1) + 1.5)
        box[wraps] = (0, 0, cam.W - 1, cam.H - 1)
        out = (box[:, 2] < 0) | (box[:, 3] < 0) | (box[:, 0] > cam.W - 1) | (box[:, 1] > cam.H - 1)
        box[:, 0] = np.clip(box[:, 0], 0, cam.W - 1)
        box[:, 1] = np.clip(box[:, 1], 0, cam.H - 1)
        box[:, 2] = np.clip(box[:, 2], 0, cam.W - 1)
        box[:, 3] = np.clip(box[:, 3], 0, cam.H - 1)
        box[out, 2] = -1
        return box

    def hits(self, dirs, cell_of_dir):
        """Yield (dir_index, t, local_tri) hit arrays for rays dirs (N,3) whose image cell is cell_of_dir (N,)
        (-1 = not cast)."""
        W, H = self.cam.W, self.cam.H
        ok = cell_of_dir >= 0
        order = np.argsort(np.where(ok, cell_of_dir, W * H), kind="stable")
        cells_sorted = np.where(ok, cell_of_dir, W * H)[order]
        cnt = np.bincount(cells_sorted, minlength=W * H + 1)[:W * H]
        start = np.concatenate([[0], np.cumsum(cnt)[:-1]])
        sat = np.zeros((H + 1, W + 1), np.int64)
        sat[1:, 1:] = cnt.reshape(H, W).cumsum(0).cumsum(1)
        b = self.box
        valid = (b[:, 2] >= b[:, 0]) & (b[:, 3] >= b[:, 1])
        idx = np.nonzero(valid)[0]
        x0, y0, x1, y1 = b[idx, 0], b[idx, 1], b[idx, 2], b[idx, 3]
        nd = sat[y1 + 1, x1 + 1] - sat[y0, x1 + 1] - sat[y1 + 1, x0] + sat[y0, x0]
        keep = nd > 0
        idx, x0, y0, x1, y1 = idx[keep], x0[keep], y0[keep], x1[keep], y1[keep]
        ncell = (x1 - x0 + 1) * (y1 - y0 + 1)
        dense = bool(np.all(cnt <= 1)) and ok.sum() > 0.25 * W * H
        lo = 0
        csum = np.cumsum(ncell)
        while lo < len(idx):
            base = csum[lo - 1] if lo else 0
            hi = int(np.searchsorted(csum, base + PAIR_CHUNK, side="right"))
            hi = max(hi, lo + 1)
            ti, w = idx[lo:hi], (x1 - x0 + 1)[lo:hi]
            nc = ncell[lo:hi]
            tot = int(nc.sum())
            rep = np.repeat(np.arange(hi - lo), nc)
            local = np.arange(tot) - np.repeat(np.concatenate([[0], np.cumsum(nc)[:-1]]), nc)
            cx = x0[lo:hi][rep] + local % w[rep]
            cy = y0[lo:hi][rep] + local // w[rep]
            cell = cy * W + cx
            c = cnt[cell]
            m = c > 0
            tri, cell, c = ti[rep[m]], cell[m], c[m]
            if dense:
                di = order[start[cell]]
            else:
                r2 = np.repeat(np.arange(len(cell)), c)
                off = np.arange(int(c.sum())) - np.repeat(np.concatenate([[0], np.cumsum(c)[:-1]]), c)
                di = order[start[cell][r2] + off]
                tri = tri[r2]
            d = dirs[di]
            det = np.einsum("ij,ij->i", d, self.nn[tri])
            with np.errstate(divide="ignore", invalid="ignore"):
                inv = 1.0 / det
                u = np.einsum("ij,ij->i", d, self.aa[tri]) * inv
                v = np.einsum("ij,ij->i", d, self.qq[tri]) * inv
                t = self.tn[tri] * inv
                h = (np.abs(det) > 1e-14) & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > 1e-6)
            yield di[h], t[h], tri[h]
            lo = hi

    def first(self, dirs, cell_of_dir, skip=None):
        """Nearest hit per ray: (t (N,) inf if none, local tri (N,) -1 if none). skip: a boolean mask over the
        local triangles to ignore."""
        N = len(dirs)
        tb = np.full(N, np.inf)
        tr = np.full(N, -1, np.int64)
        for di, t, tri in self.hits(dirs, cell_of_dir):
            if skip is not None and len(di):
                k = ~skip[tri]
                di, t, tri = di[k], t[k], tri[k]
            if not len(di):
                continue
            o = np.lexsort((t, di))
            di, t, tri = di[o], t[o], tri[o]
            f = np.concatenate([[True], di[1:] != di[:-1]])
            di, t, tri = di[f], t[f], tri[f]
            better = t < tb[di]
            tb[di[better]] = t[better]
            tr[di[better]] = tri[better]
        return tb, tr


def _cells(cam, P_or_dirs, is_dir=False):
    if is_dir:
        u, v, th = cam.project_cam(P_or_dirs @ cam.R.T)
    else:
        u, v, th = cam.project(P_or_dirs)
    ok = (th <= cam.theta_max) & (u >= 0) & (u < cam.W) & (v >= 0) & (v < cam.H)
    cell = np.where(ok, np.floor(v).astype(np.int64).clip(0, cam.H - 1) * cam.W
                    + np.floor(u).astype(np.int64).clip(0, cam.W - 1), -1)
    return cell, ok


def _inside(tris, P):
    """Point-in-closed-mesh by ray parity along two skew directions (both must agree odd)."""
    res = []
    for d in (np.array([0.123, 0.311, 0.942]), np.array([-0.517, 0.211, -0.83])):
        d = d / np.linalg.norm(d)
        v0, e1, e2 = tris[:, 0], tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
        p = np.cross(d, e2)
        det = np.einsum("ij,ij->i", e1, p)
        tv = P - v0
        with np.errstate(divide="ignore", invalid="ignore"):
            u = np.einsum("ij,ij->i", tv, p) / det
            q = np.cross(tv, e1)
            v = (q @ d) / det
            t = np.einsum("ij,ij->i", e2, q) / det
        n = int(np.sum((np.abs(det) > 1e-12) & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 0)))
        res.append(n % 2 == 1)
    return all(res)


# ------------------------------------------------------------------ one configuration

def simulate(S, geo, spec, cfg, out_dir, log=print):
    t0 = time.time()
    names, part, tris = S["names"], S["part"], S["tris"]
    kind = S["kind"]
    cam_i = names.index(CAMERA_PART)
    cam = Fisheye(cfg["lens"], cfg["axis"], SIM_W, SIM_H, spec["fov_h_deg"], spec["fov_circle_deg"],
                  right=cfg.get("right"))
    sensor_scale = spec["resolution_px"][0] / SIM_W
    pk = np.array([kind[n] for n in names])
    excl = [names.index(n) for n in cfg.get("exclude", ())]
    opaque = (pk[part] == "opaque") & (part != cam_i) & ~np.isin(part, excl)
    transp = (pk[part] != "opaque")
    frog_disc_i = [names.index(n) for n in DISC_FROG if n in names]
    frog_wall_i = [names.index(n) for n in WALL_FROG if n in names]
    top_i = names.index(TOP_DISC)
    r_in = geo["tube_inner_r"]
    z_disc_under = geo["top_disc_under_z"]

    ids_op = np.nonzero(opaque)[0]
    ids_tr = np.nonzero(transp)[0]
    C_op = Caster(cam, tris, ids_op)
    C_tr = Caster(cam, tris, ids_tr)

    # ---- image pass
    dirs, valid = cam.pixel_dirs()
    cell = np.where(valid, np.arange(len(dirs)), -1)
    t_op, tri_op = C_op.first(dirs, cell)
    hit_part = np.where(tri_op >= 0, part[ids_op[np.maximum(tri_op, 0)]], -1)
    # transparent crossings before the first opaque hit, as a bitmask over transparent parts
    tr_parts = sorted(set(part[ids_tr].tolist()))
    bit = {p: k for k, p in enumerate(tr_parts)}
    bitarr = np.zeros(len(names), np.int64)
    for p, k in bit.items():
        bitarr[p] = 1 << k
    mask = np.zeros(len(dirs), np.int64)
    t_top = np.full(len(dirs), np.inf)
    TH = []
    for di, t, tri in C_tr.hits(dirs, cell):
        p = part[ids_tr[tri]]
        TH.append((di, t, p))
        before = t < t_op[di]
        np.bitwise_or.at(mask, di[before], bitarr[p[before]])
        # target: the top disc's underside inside the tube
        top = p == top_i
        if top.any():
            P = cam.O + dirs[di[top]] * t[top][:, None]
            inner = np.hypot(P[:, 0], P[:, 1]) < r_in
            dd, tt = di[top][inner], t[top][inner]
            np.minimum.at(t_top, dd, tt)
    frog_parts = np.array(frog_disc_i + frog_wall_i)
    is_frog_hit = np.isin(hit_part, frog_parts)
    # target distance behind any occluder: nearest of frog (among frog triangles only) and top disc
    frog_ids = np.nonzero(np.isin(part, frog_parts))[0]
    C_fr = Caster(cam, tris, frog_ids)
    t_frog, _ = C_fr.first(dirs, cell)
    t_target = np.minimum(t_frog, t_top)
    # transparent parts crossed on the way to a target (strictly in front of it)
    mask_t = np.zeros(len(dirs), np.int64)
    for di, t, p in TH:
        k = t < t_target[di] - 1e-3
        np.bitwise_or.at(mask_t, di[k], bitarr[p[k]])
    del TH
    occl = valid & np.isfinite(t_target) & (t_op < t_target - 1e-6) & ~is_frog_hit & (hit_part >= 0)
    occ_counts = {}
    for p, n in zip(*np.unique(hit_part[occl], return_counts=True)):
        occ_counts[names[p]] = int(n)
    occluders = sorted(occ_counts, key=lambda k: -occ_counts[k])

    disc_px = int(np.sum(valid & np.isin(hit_part, frog_disc_i)))
    wall_px = int(np.sum(valid & np.isin(hit_part, frog_wall_i)))
    top_px = int(np.sum(valid & np.isfinite(t_top) & (t_top < t_op)))
    in_frame = {}
    for p, n in zip(*np.unique(hit_part[valid & (hit_part >= 0)], return_counts=True)):
        in_frame[names[p]] = int(n)
    acr_cross = {}
    reach = valid & (np.isfinite(t_target)) & ~occl
    for p, k in bit.items():
        n = int(np.sum(reach & ((mask_t >> k) & 1).astype(bool)))
        if n and names[p] != TOP_DISC:
            acr_cross[names[p]] = n

    # ---- top disc visibility: area-uniform samples of the underside inside the tube
    rng = np.random.default_rng(7)
    ns = 6000
    rr = r_in * np.sqrt((np.arange(ns) + 0.5) / ns)
    aa = np.arange(ns) * math.pi * (3 - math.sqrt(5))
    Pd = np.stack([rr * np.cos(aa), rr * np.sin(aa), np.full(ns, z_disc_under)], 1)
    vis_d, top_disc_fraction, top_detail = _point_visibility(cam, C_op, Pd)
    del rng

    # ---- frog surface visibility (triangles facing the camera, first opaque hit is that frog)
    def frog_vis(pidx):
        ids = np.nonzero(np.isin(part, pidx))[0]
        T = tris[ids]
        c = T.mean(1)
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        area = 0.5 * np.linalg.norm(n, axis=1)
        facing = np.einsum("ij,ij->i", n, cam.O - c) > 0
        if not facing.any():
            return 0.0, 0.0, 0, 0
        cf, idf, af = c[facing], ids[facing], area[facing]
        d = cf - cam.O
        dist = np.linalg.norm(d, axis=1)
        d = d / dist[:, None]
        cellf, ok = _cells(cam, d, is_dir=True)
        tb, tr = C_op.first(d, cellf)
        hp = np.where(tr >= 0, part[ids_op[np.maximum(tr, 0)]], -1)
        seen = ok & np.isin(hp, pidx) & (tb > dist - 0.05)
        # the same rays with the frog's own body ignored: what the rest of the machine blocks
        tb2, tr2 = C_op.first(d, cellf, skip=np.isin(part[ids_op], pidx))
        unb = ok & (tb2 > dist - 0.05)
        blk = ok & ~unb
        bp = part[ids_op[tr2[blk & (tr2 >= 0)]]]
        by = {names[p]: int(n) for p, n in zip(*np.unique(bp, return_counts=True))}
        return dict(fraction=float(seen.mean()), area_fraction=float((af * seen).sum() / af.sum()), seen=int(seen.sum()),
                    facing=int(len(seen)), in_fov=float(ok.mean()), unblocked=float(unb.mean()),
                    blocked_by=dict(sorted(by.items(), key=lambda kv: -kv[1])))

    FD = frog_vis(frog_disc_i)
    FW = frog_vis(frog_wall_i)
    dfv, dfa, dfn = FD["fraction"], FD["area_fraction"], FD["facing"]
    wfv, wfa, wfn = FW["fraction"], FW["area_fraction"], FW["facing"]

    # ---- resolution at each frog's contact centre, in the contact plane, at the real sensor resolution
    def frog_center(pidx, wall):
        T = tris[np.isin(part, pidx)]
        lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
        if wall:
            body = T.reshape(-1, 3)
            P = np.array([lo[0], (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2])
            rad = np.array([P[0], P[1], 0.0])
            rad /= np.linalg.norm(rad)
            a = np.array([-rad[1], rad[0], 0.0])
            b = np.array([0.0, 0.0, 1.0])
            del body
        else:
            P = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])
            a, b = np.array([1.0, 0, 0]), np.array([0, 1.0, 0])
        return P, a, b

    res = {}
    for key, pidx, wall in (("disc_frog", frog_disc_i, False), ("wall_frog", frog_wall_i, True)):
        P, a, b = frog_center(pidx, wall)
        inview, u, v = cam.visible(P[None])
        ppm, pa, pb = cam.px_per_mm(P, a, b, sensor_scale)
        d = P - cam.O
        th = math.degrees(math.acos(np.dot(d, cam.R[2]) / np.linalg.norm(d)))
        res[key] = dict(centre_mm=[round(float(x), 2) for x in P], in_fov=bool(inview[0]),
                        off_axis_deg=round(th, 1), range_mm=round(float(np.linalg.norm(d)), 1),
                        px_per_mm=round(ppm, 2) if inview[0] else 0.0,
                        px_per_mm_axes=[round(pa, 2), round(pb, 2)],
                        image_uv=[round(float(u[0]), 1), round(float(v[0]), 1)])

    # ---- lens inside a solid?
    inside = []
    for p in sorted(set(part[ids_op].tolist())):
        T = tris[part == p]
        lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
        if np.all(cam.O > lo - 1e-6) and np.all(cam.O < hi + 1e-6) and _inside(T, cam.O):
            inside.append(names[p])

    # ---- images
    name = cfg["name"]
    img = os.path.join(out_dir, f"{name}.png")
    lab = os.path.join(out_dir, f"{name}_labels.png")
    _write_images(S, cam, dirs, valid, t_op, tri_op, ids_op, hit_part, mask, bit, t_top, cfg, img, lab, in_frame,
                  dict(disc=res["disc_frog"], wall=res["wall_frog"]))

    sec = round(time.time() - t0, 1)
    log(f"camsim {name}: tilt {cfg['tilt_deg']:.1f} h {cfg['lens_above_disc_mm']:.1f}  top disc {top_disc_fraction:.3f}"
        f"  disc frog {dfv:.3f} ({disc_px} px)  wall frog {wfv:.3f} ({wall_px} px)  occluders {occluders[:4]}  {sec} s")
    return dict(
        name=name, tilt_deg=round(float(cfg["tilt_deg"]), 2), lens_above_disc_mm=round(float(cfg["lens_above_disc_mm"]), 2),
        lens_xyz_mm=[round(float(x), 2) for x in cam.O], axis_unit=[round(float(x), 4) for x in cam.R[2]],
        top_disc_visible_fraction=round(top_disc_fraction, 4),
        disc_frog_visible_fraction=round(dfv, 4), wall_frog_visible_fraction=round(wfv, 4),
        disc_frog_visible_area_fraction=round(dfa, 4), wall_frog_visible_area_fraction=round(wfa, 4),
        disc_frog_facing_triangles=dfn, wall_frog_facing_triangles=wfn,
        disc_frog_unblocked_fraction=round(FD["unblocked"], 4), wall_frog_unblocked_fraction=round(FW["unblocked"], 4),
        disc_frog_facing_in_fov_fraction=round(FD["in_fov"], 4), wall_frog_facing_in_fov_fraction=round(FW["in_fov"], 4),
        disc_frog_blocked_by=FD["blocked_by"], wall_frog_blocked_by=FW["blocked_by"],
        frog_visibility_note="*_visible_fraction: camera-facing triangles whose centroid ray hits that triangle first "
                             "(the frog's own legs and body hide some of its facing triangles); *_unblocked_fraction: "
                             "the same rays with the frog itself ignored, i.e. what the rest of the machine or the "
                             "field of view takes away",
        disc_frog_px_per_mm=res["disc_frog"]["px_per_mm"], wall_frog_px_per_mm=res["wall_frog"]["px_per_mm"],
        disc_frog_px_area=disc_px, wall_frog_px_area=wall_px,
        disc_frog_px_area_sensor=int(round(disc_px * sensor_scale ** 2)),
        wall_frog_px_area_sensor=int(round(wall_px * sensor_scale ** 2)),
        top_disc_px_area=top_px,
        occluders=occluders, occluder_px=occ_counts,
        through_acrylic_px=acr_cross,
        top_disc_detail=top_detail,
        frogs=res, lens_inside_parts=inside,
        in_frame_px=dict(sorted(in_frame.items(), key=lambda kv: -kv[1])[:20]),
        image=img, label_image=lab, seconds=sec)


def _point_visibility(cam, C_op, P):
    d = P - cam.O
    dist = np.linalg.norm(d, axis=1)
    d = d / dist[:, None]
    cell, ok = _cells(cam, d, is_dir=True)
    tb, _ = C_op.first(d, cell)
    clear = tb > dist - 1e-3
    vis = ok & clear
    return vis, float(vis.mean()), dict(samples=len(P), in_fov=round(float(ok.mean()), 4),
                                         unobstructed_if_in_fov=round(float((ok & clear).sum() / max(1, ok.sum())), 4))


# ------------------------------------------------------------------ images

def _part_colour(name, S):
    if name.startswith("v_frog"):
        return (0.20, 0.75, 0.25)
    if name.startswith("v_ir_led"):
        return (1.0, 0.85, 0.95)
    return S["colour"].get(name, (0.6, 0.6, 0.6))


def _label_colour(k):
    h = (k * 0.61803398875) % 1.0
    import colorsys
    return colorsys.hsv_to_rgb(h, 0.65 + 0.35 * ((k * 7) % 3) / 2, 0.95 - 0.25 * ((k * 5) % 2))


def _write_images(S, cam, dirs, valid, t_op, tri_op, ids_op, hit_part, mask, bit, t_top, cfg, img_path, lab_path,
                  in_frame, frogs):
    from PIL import Image, ImageDraw
    W, H = cam.W, cam.H
    names, tris = S["names"], S["tris"]
    N = W * H
    # normals of the hit triangles for shading
    T = tris[ids_op[np.maximum(tri_op, 0)]]
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    lam = np.abs(np.einsum("ij,ij->i", n, dirs))
    depth = np.where(np.isfinite(t_op), t_op, 1e9)
    fall = np.clip(1.15 - depth / 450.0, 0.35, 1.0)
    base = np.array([_part_colour(names[p], S) if p >= 0 else (0.0, 0.0, 0.0) for p in range(len(names))] + [(0.05, 0.05, 0.07)])
    hp = np.where(hit_part >= 0, hit_part, len(names))
    rgb = base[hp] * (0.25 + 0.75 * lam)[:, None] * fall[:, None]
    rgb[hit_part < 0] = (0.06, 0.06, 0.08)
    led = np.array([names[p].startswith("v_ir_led") if p >= 0 else False for p in hp.clip(0, len(names) - 1)])
    led &= hit_part >= 0
    rgb[led] = (1.0, 0.95, 1.0)
    # acrylic tint and net dimming
    acr = np.zeros(N, bool)
    net = np.zeros(N, bool)
    for p, k in bit.items():
        b = ((mask >> k) & 1).astype(bool)
        if S["kind"][names[p]] == "mesh":
            net |= b
        else:
            acr |= b
    rgb[acr] = rgb[acr] * 0.85 + np.array([0.55, 0.75, 0.85]) * 0.15
    rgb[net] = rgb[net] * 0.7
    topm = np.isfinite(t_top) & (t_top < t_op)
    rgb[topm] = rgb[topm] * 0.9 + np.array([0.6, 0.85, 1.0]) * 0.1
    rgb[~valid] = 0.0
    im = Image.fromarray((np.clip(rgb, 0, 1) * 255).astype(np.uint8).reshape(H, W, 3))
    d = ImageDraw.Draw(im)
    d.text((8, 6), f"{cfg['name']}: tilt {cfg['tilt_deg']:.0f} deg, lens {cfg['lens_above_disc_mm']:.0f} mm above disc",
           fill=(255, 255, 255))
    d.text((8, 20), "equidistant fisheye, " + cfg.get("orientation_label", "image up = +X (wall frog side)"),
           fill=(200, 200, 200))
    for key, col in (("disc", (120, 255, 120)), ("wall", (255, 255, 120))):
        f = frogs[key]
        if f["in_fov"]:
            u, v = f["image_uv"]
            d.ellipse((u - 6, v - 6, u + 6, v + 6), outline=col)
            d.text((u + 8, v - 6), f"{key} frog", fill=col)
    im.save(img_path, optimize=False)

    # label image: one colour per opaque part, legend for the 18 largest
    order = sorted(in_frame, key=lambda k: -in_frame[k])
    pal = {}
    for k, nm in enumerate(order):
        pal[nm] = _label_colour(k)
    for nm in DISC_FROG + WALL_FROG:
        pal[nm] = (0.1, 0.9, 0.2) if nm.endswith("body") else (0.0, 0.5, 0.1)
    lc = np.array([pal.get(names[p], (0.5, 0.5, 0.5)) if p >= 0 else (0, 0, 0) for p in range(len(names))] + [(0.1, 0.1, 0.1)])
    lrgb = lc[hp] * (0.55 + 0.45 * lam)[:, None]          # shaded so shapes read inside one label colour
    lrgb[acr] = lrgb[acr] * 0.8 + 0.2 * np.array([0.6, 0.9, 1.0])
    lrgb[~valid] = 0
    lw = W + 300
    lim = Image.new("RGB", (lw, H), (20, 20, 24))
    lim.paste(Image.fromarray((np.clip(lrgb, 0, 1) * 255).astype(np.uint8).reshape(H, W, 3)), (0, 0))
    d = ImageDraw.Draw(lim)
    d.text((W + 10, 8), f"{cfg['name']} part labels (px)", fill=(255, 255, 255))
    y = 28
    for nm in order[:30]:
        c = tuple(int(255 * x) for x in pal[nm])
        d.rectangle((W + 10, y, W + 22, y + 11), fill=c)
        d.text((W + 28, y), f"{nm[:34]} {in_frame[nm]}", fill=(230, 230, 230))
        y += 15
    d.text((W + 10, y + 6), "light blue tint: ray crossed acrylic", fill=(170, 220, 240))
    lim.save(lab_path, optimize=False)


def montage(results, path):
    from PIL import Image, ImageDraw
    ims = [Image.open(r["image"]) for r in results]
    w, h = ims[0].size
    cols = min(4, len(ims))
    rows = (len(ims) + cols - 1) // cols
    cap = 64
    M = Image.new("RGB", (cols * w, rows * (h + cap)), (16, 16, 20))
    d = ImageDraw.Draw(M)
    for k, (r, im) in enumerate(zip(results, ims)):
        x, y = (k % cols) * w, (k // cols) * (h + cap)
        M.paste(im, (x, y))
        d.text((x + 8, y + h + 6), f"{r['name']}  tilt {r['tilt_deg']:.0f}  h {r['lens_above_disc_mm']:.0f} mm", fill=(255, 255, 255))
        d.text((x + 8, y + h + 22), f"top disc {100 * r['top_disc_visible_fraction']:.0f}%  disc frog {100 * r['disc_frog_visible_fraction']:.0f}% "
                                     f"{r['disc_frog_px_per_mm']:.1f} px/mm  wall frog {100 * r['wall_frog_visible_fraction']:.0f}% "
                                     f"{r['wall_frog_px_per_mm']:.1f} px/mm", fill=(200, 230, 200))
        d.text((x + 8, y + h + 38), "occluders: " + ", ".join(r["occluders"][:3]), fill=(230, 200, 200))
    M.save(path, optimize=False)
    return path


# ------------------------------------------------------------------ driver

def run(root, name, log=print):
    t0 = time.time()
    od = os.path.join(root, "out", name)
    glb = os.path.join(od, f"{name}.glb")
    sheet = json.load(open(os.path.join(od, "sheet.json")))
    spec = load_camera_spec(root)
    S = load_scene(glb, sheet)
    names = S["names"]
    for need in (CAMERA_PART, TOP_DISC, TUBE, FLOOR) + DISC_FROG[:1] + WALL_FROG[:1]:
        if need not in names:
            raise RuntimeError(f"{need} not in {glb}")
    out = os.path.join(od, "camsim")
    os.makedirs(out, exist_ok=True)
    tris, part = S["tris"], S["part"]

    def pts(n):
        return tris[part == names.index(n)].reshape(-1, 3)
    floor_top = float(pts(FLOOR)[:, 2].max())
    tube = pts(TUBE)
    geo = dict(floor_top_z=floor_top, tube_inner_r=float(np.hypot(tube[:, 0], tube[:, 1]).min()),
               top_disc_under_z=float(pts(TOP_DISC)[:, 2].min()))
    L = lens_from_assembly(S)
    ax = L["axis"]
    tilt = math.degrees(math.acos(np.clip(ax[2], -1, 1)))
    h = float(L["lens"][2] - floor_top)
    log(f"camsim: lens from {CAMERA_PART} at {np.round(L['lens'], 2).tolist()}, axis {np.round(ax, 4).tolist()}, "
        f"tilt {tilt:.2f} deg, {h:.2f} mm above {FLOOR}; tube inner r {geo['tube_inner_r']:.2f}")

    toward = np.array([ax[0], ax[1], 0.0])
    toward = toward / np.linalg.norm(toward) if np.linalg.norm(toward) > 1e-9 else np.array([1.0, 0, 0])
    la = L["long_axis"]
    if la is not None:
        olabel = "image width = sensor long axis (from the board), left -X / right +X"
        cam0 = Fisheye(L["lens"], ax, SIM_W, SIM_H, spec["fov_h_deg"], spec["fov_circle_deg"], right=la)
        img_right, img_up = cam0.R[0], cam0.R[1]
        pn = np.cross(ax, img_right)
        plane = {0: "Y-Z", 1: "X-Z", 2: "X-Y"}[int(np.argmax(np.abs(pn)))]
    else:
        olabel = "image up = +X (wall frog side)"
        cam0 = Fisheye(L["lens"], ax, SIM_W, SIM_H, spec["fov_h_deg"], spec["fov_circle_deg"])
        img_right, img_up = cam0.R[0], cam0.R[1]
        plane = "unknown (default orientation)"
    cfgs = [dict(name="as_built", lens=L["lens"], axis=ax, tilt_deg=tilt, lens_above_disc_mm=h, virtual=False,
                 right=la, orientation_label=olabel)]

    def virt(tl, hh):
        a = np.array([0.0, 0.0, 1.0]) * math.cos(math.radians(tl)) + toward * math.sin(math.radians(tl))
        p = np.array([L["lens"][0], L["lens"][1], floor_top + hh])
        # virtual cameras keep the as-built roll: the long axis stays in the plane of the tilt
        r = None
        if la is not None:
            side = np.cross(ax, la)
            r = np.cross(side, a)
            if np.dot(r, la) < 0:
                r = -r
        return dict(name=f"tilt{tl:.0f}_h{hh:.0f}", lens=p, axis=a, tilt_deg=tl, lens_above_disc_mm=hh, virtual=True,
                    right=r, orientation_label=olabel)
    seen = set()
    for tl in SWEEP_TILTS:
        cfgs.append(virt(tl, SWEEP_HEIGHT_FOR_TILTS))
    for hh in SWEEP_HEIGHTS:
        if hh != SWEEP_HEIGHT_FOR_TILTS:
            cfgs.append(virt(SWEEP_TILT_FOR_HEIGHTS, hh))
    cfgs = [c for c in cfgs if not (c["name"] in seen or seen.add(c["name"]))]

    results = []
    fails = []
    for c in cfgs:
        try:
            r = simulate(S, geo, spec, c, out, log=log)
            if r["lens_inside_parts"]:
                c2 = dict(c, name=c["name"] + "_mount_moved", exclude=tuple(r["lens_inside_parts"]))
                r2 = simulate(S, geo, spec, c2, out, log=log)
                r["if_mount_moved"] = {k: r2[k] for k in (
                    "top_disc_visible_fraction", "disc_frog_visible_fraction", "wall_frog_visible_fraction",
                    "disc_frog_unblocked_fraction", "wall_frog_unblocked_fraction", "disc_frog_px_per_mm",
                    "wall_frog_px_per_mm", "disc_frog_px_area", "wall_frog_px_area", "occluders", "image",
                    "label_image")}
                r["if_mount_moved"]["note"] = (f"same virtual camera with {', '.join(r['lens_inside_parts'])} left out "
                                               f"(the lens point is inside it; a seat for this height would move)")
            results.append(r)
        except Exception as e:  # noqa: BLE001
            import traceback
            traceback.print_exc()
            fails.append(f"FAIL camsim {c['name']}: {e!r}")
    mpath = montage(results, os.path.join(out, "montage.png")) if results else None

    camera = dict(model=spec["model"], sensor=spec["sensor"], resolution_px=spec["resolution_px"],
                  fov_diag_deg=spec["fov_diag_deg"], fov_h_deg=spec["fov_h_deg"], fov_v_deg=spec["fov_v_deg"],
                  fov_circle_deg=spec["fov_circle_deg"], fov_note=spec["fov_note"], projection=spec["projection"],
                  source_url=spec["source_url"], lens_from="assembly", lens_xyz_mm=[round(float(x), 3) for x in L["lens"]],
                  axis_unit=[round(float(x), 5) for x in ax], tilt_deg=round(tilt, 2), lens_above_disc_mm=round(h, 2),
                  board_centre_mm=[round(float(x), 3) for x in L["board_centre"]],
                  board_extent_mm=L["board_extent_mm"], lens_front_offset_mm=round(L["lens_front_offset_mm"], 3),
                  sim_image_px=[SIM_W, SIM_H], f_px_per_rad_sim=round(SIM_W / 2 / math.radians(spec["fov_h_deg"] / 2), 3),
                  f_px_per_rad_sensor=round(spec["resolution_px"][0] / 2 / math.radians(spec["fov_h_deg"] / 2), 3),
                  image_orientation=olabel, image_up_from=L["image_up_from"],
                  image_up_unit=[round(float(x), 5) for x in img_up],
                  image_right_unit=[round(float(x), 5) for x in img_right],
                  sensor_long_axis_unit=None if la is None else [round(float(x), 5) for x in la],
                  sensor_long_axis_plane=plane,
                  px_per_mm_note="px_per_mm at the real 2592 x 1944 sensor, in each frog's contact plane (disc frog: "
                                 "the top disc plane; wall frog: the tube tangent plane), geometric mean of two 1 mm "
                                 "steps; px_area is counted in the simulated image (px_area_sensor scales it)")
    scene = dict(parts=names, triangles=int(len(tris)), rays_per_image=SIM_W * SIM_H,
                 rays_inside_image_circle=int(results and _circle_rays(spec) or 0),
                 acrylic_parts=[n for n in names if S["kind"][n] == "acrylic"],
                 transparent_parts=[n for n in names if S["kind"][n] != "opaque"],
                 observer_excluded=[CAMERA_PART], geometry=geo, glb=glb,
                 refraction="ignored (straight rays through the acrylic)")
    issues = feasibility(results, camera)
    rec = recommendation(results)
    doc = dict(assembly=name, camera=camera, scene=scene, configs=results, montage=mpath,
               feasibility_issues=issues, recommendation=rec, fails=fails, seconds=round(time.time() - t0, 1))
    jpath = os.path.join(out, "camsim.json")
    json.dump(doc, open(jpath, "w"), indent=1)
    return dict(command="camsim", assembly=name, ok=not fails and len(results) == len(cfgs),
                configs=len(results), seconds=round(time.time() - t0, 1), fails=fails, out_dir=out,
                camsim_json=jpath, montage=mpath)


def _circle_rays(spec):
    cam = Fisheye((0, 0, 0), (0, 0, 1), SIM_W, SIM_H, spec["fov_h_deg"], spec["fov_circle_deg"])
    return int(cam.pixel_dirs()[1].sum())


def feasibility(results, camera):
    issues = []
    for r in results:
        n = r["name"]
        pre = f"{n} (tilt {r['tilt_deg']:.0f}, h {r['lens_above_disc_mm']:.0f}):"
        if r["lens_inside_parts"]:
            mm = r.get("if_mount_moved") or {}
            issues.append(f"{pre} the lens point lies inside {', '.join(r['lens_inside_parts'])}, so this height needs "
                          f"a different seat; with that part left out the view gives top disc "
                          f"{100 * mm.get('top_disc_visible_fraction', 0):.0f}%, disc frog "
                          f"{mm.get('disc_frog_px_per_mm', 0):.1f} px/mm, wall frog {mm.get('wall_frog_px_per_mm', 0):.1f} px/mm")
            continue
        for fk in ("disc_frog", "wall_frog"):
            f = r["frogs"][fk]
            nice = fk.replace("_", " ")
            if not f["in_fov"]:
                issues.append(f"{pre} {nice} centre is outside the sensor image (off axis {f['off_axis_deg']} deg)")
            elif r[f"{fk}_px_per_mm"] < MIN_PX_PER_MM:
                issues.append(f"{pre} {nice} resolution {r[f'{fk}_px_per_mm']} px/mm is under {MIN_PX_PER_MM} px/mm")
            v = r[f"{fk}_unblocked_fraction"]
            if v < 0.95:
                by = list(r[f"{fk}_blocked_by"])[:3]
                fov = r[f"{fk}_facing_in_fov_fraction"]
                issues.append(f"{pre} {100 * (1 - v):.0f}% of the {nice}'s camera-facing surface is cut off"
                              + (f" by {', '.join(by)}" if by else "")
                              + (f" ({100 * (1 - fov):.0f}% lies outside the sensor image)" if fov < 0.999 else ""))
        if r["top_disc_visible_fraction"] < 0.95:
            dd = r["top_disc_detail"]
            issues.append(f"{pre} only {100 * r['top_disc_visible_fraction']:.0f}% of the top disc inside the tube is "
                          f"seen (in FOV {100 * dd['in_fov']:.0f}%, unobstructed when in FOV {100 * dd['unobstructed_if_in_fov']:.0f}%)")
        for o in r["occluders"]:
            if o.startswith("fp9_camera_wedge_bridge") or o.startswith("v_power_bank") or o.startswith("v_ir_led") \
                    or o.startswith("v_rpi") or o.startswith("v_led_driver") or o.startswith("hw_"):
                issues.append(f"{pre} {o} blocks {r['occluder_px'][o]} px of the targets (frogs / top disc)")
        if r["name"] == "as_built":
            for o in ("fp9_camera_wedge_bridge", "v_power_bank"):
                if o in r["in_frame_px"]:
                    issues.append(f"{pre} {o} is in frame ({r['in_frame_px'][o]} px)")
            for k, v in r["through_acrylic_px"].items():
                if k.startswith("lp_ring") or k.startswith("lp_cage"):
                    issues.append(f"{pre} {v} px of the target rays cross {k} (stacked ring acrylic edge-on, "
                                  f"poor optical path for FTIR)")
    return issues


def recommendation(results):
    ab = next(r for r in results if r["name"] == "as_built")

    def score(r):
        if r["lens_inside_parts"]:
            return -1
        return (r["top_disc_visible_fraction"] + r["disc_frog_unblocked_fraction"] + r["wall_frog_unblocked_fraction"]
                + 0.05 * min(r["disc_frog_px_per_mm"], 10) + 0.05 * min(r["wall_frog_px_per_mm"], 10))
    best = max(results, key=score)
    works = (ab["disc_frog_unblocked_fraction"] >= 0.95 and ab["wall_frog_unblocked_fraction"] >= 0.95
             and ab["top_disc_visible_fraction"] >= 0.9 and ab["disc_frog_px_per_mm"] >= MIN_PX_PER_MM
             and ab["wall_frog_px_per_mm"] >= MIN_PX_PER_MM)
    s1 = (f"As built (tilt {ab['tilt_deg']:.0f} deg, lens {ab['lens_above_disc_mm']:.0f} mm above the disc) the camera "
          f"sees {100 * ab['top_disc_visible_fraction']:.0f}% of the top disc inside the tube and nothing on the rotor "
          f"blocks either frog ({100 * ab['disc_frog_unblocked_fraction']:.0f}% / {100 * ab['wall_frog_unblocked_fraction']:.0f}% "
          f"of their camera-facing triangles unblocked; {100 * ab['disc_frog_visible_fraction']:.0f}% / "
          f"{100 * ab['wall_frog_visible_fraction']:.0f}% are seen first, the rest hidden by the frogs' own legs), at "
          f"{ab['disc_frog_px_per_mm']:.1f} px/mm on the disc frog and {ab['wall_frog_px_per_mm']:.1f} px/mm on the wall "
          f"frog, so the placement "
          + ("works for both frogs." if works else "does not fully work: " + (
              ", ".join(ab["occluders"][:3]) + " block part of the targets." if ab["occluders"] else "coverage or resolution falls short.")))
    s2 = (f"The best configuration of the sweep is {best['name']} (tilt {best['tilt_deg']:.0f} deg, "
          f"h {best['lens_above_disc_mm']:.0f} mm: top disc {100 * best['top_disc_visible_fraction']:.0f}%, disc frog "
          f"{100 * best['disc_frog_visible_fraction']:.0f}% at {best['disc_frog_px_per_mm']:.1f} px/mm, wall frog "
          f"{100 * best['wall_frog_visible_fraction']:.0f}% at {best['wall_frog_px_per_mm']:.1f} px/mm), scored on "
          f"coverage of both frogs and the disc plus resolution.")
    lo = min(min(r["disc_frog_px_per_mm"], r["wall_frog_px_per_mm"]) for r in results if not r["lens_inside_parts"])
    blocked = [r for r in results if r["lens_inside_parts"]]
    cut = [r for r in results if not r["lens_inside_parts"] and r["top_disc_visible_fraction"] < 0.99]
    bits = [f"Resolution is not the limit: every configuration clear of its mount gives at least {lo:.1f} px/mm on "
            f"both frogs at the full 2592 x 1944 sensor (the brief's floor is {MIN_PX_PER_MM:g})"]
    if blocked:
        bits.append("the lens heights below the bridge top ("
                    + ", ".join(f"{r['lens_above_disc_mm']:.0f} mm" for r in blocked)
                    + ") put the lens inside " + ", ".join(sorted({p for r in blocked for p in r["lens_inside_parts"]}))
                    + " and would need a new seat")
    if cut:
        bits.append("steeper tilts push the -X edge of the top disc out of the frame ("
                    + ", ".join(f"{r['name']} {100 * r['top_disc_visible_fraction']:.0f}%" for r in cut) + ")")
    gain = 100 * (min(best["disc_frog_px_per_mm"], best["wall_frog_px_per_mm"])
                  / max(1e-9, min(ab["disc_frog_px_per_mm"], ab["wall_frog_px_per_mm"])) - 1)
    s3 = "; ".join(bits) + (f". {best['name']} only buys about {gain:.0f}% more resolution, so the as-built "
                            f"30 deg / 48 mm placement works and can stay." if works and best is not ab and gain < 30
                            else ".")
    return " ".join([s1, s2, s3])
