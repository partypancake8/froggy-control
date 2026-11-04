"""Off-screen renders. PyVista (default): iso, top, front, right, section (XZ
plane at Y 0, red caps) and section_joint (plane through one screw axis), 1200
px; assemblies with a rotor module also get section_motor, section_corner,
section_flange, section_plate and exploded. Component meshes come from
lib.mesh (tessellated once, shared with the exports). Blender -b Workbench is
an optional fallback (`--engine blender`, iso from the GLB). Never `import vtk`
directly; pyvista pulls it in."""
import math
import os
import subprocess

import numpy as np
from build123d import Align, Box, Location

BLENDER = "/Applications/Blender.app/Contents/MacOS/Blender"
HERE = os.path.dirname(os.path.abspath(__file__))
RED = (0.85, 0.08, 0.08)
PX = 1200
TESS = (0.08, 0.3)


def _pv():
    os.environ.setdefault("PYVISTA_OFF_SCREEN", "true")
    import pyvista as pv
    pv.OFF_SCREEN = True
    return pv


def to_pv(pv, shape, tol=TESS):
    from lib.mesh import fast_tessellate
    v, tris = fast_tessellate(shape, tol)
    if not len(tris):
        return None
    f = np.hstack([np.full((len(tris), 1), 3), tris]).ravel()
    return pv.PolyData(v, f)


def _plotter(pv):
    pl = pv.Plotter(off_screen=True, window_size=(PX, PX))
    pl.set_background("white")
    return pl


def _shoot(pv, path, items, view, label, camera=None, parallel=True, zoom=1.0, fit=None):
    """items: (mesh, colour) or (mesh, colour, outline) or (mesh, colour,
    outline, opacity). `fit` = view direction vector: camera looks along -fit
    at the scene and is reset to fit it."""
    pl = _plotter(pv)
    for it in items:
        mesh, col = it[0], it[1]
        op = it[3] if len(it) > 3 else 1.0
        _add(pl, mesh, col, smooth_shading=False, opacity=op)
        if len(it) > 2 and it[2]:
            edges = mesh.extract_feature_edges(boundary_edges=True, feature_edges=False,
                                               manifold_edges=False, non_manifold_edges=False)
            if edges.n_points:
                pl.add_mesh(edges, color="black", line_width=2)
    if fit is not None:
        pl.view_vector(fit, viewup=(0, 0, 1))
        pl.reset_camera()
    elif camera is not None:
        pl.camera_position = camera
    else:
        getattr(pl, view)()
    if parallel:
        pl.enable_parallel_projection()
    if camera is None and fit is None:
        pl.reset_camera()
    pl.camera.zoom(zoom)
    pl.add_text(label, font_size=10, color="black")
    pl.screenshot(path)
    pl.close()
    return path


def _cut(pv, A, keep, plane_axis, plane_value, pick=None):
    """B-rep section: each part intersected with the keep box; caps = cut faces
    lying on the plane, lifted 0.05 mm toward the viewer to avoid z-fighting."""
    k = "xyz".index(plane_axis)
    kb = keep.bounding_box()
    kmn, kmx = tuple(kb.min), tuple(kb.max)
    items, caps = [], []
    for n, p in A.parts.items():
        if pick and not pick(n):
            continue
        mn, mx = A.bbox(n)
        if any(mx[i] < kmn[i] or mn[i] > kmx[i] for i in range(3)):
            continue
        inside = all(mn[i] >= kmn[i] and mx[i] <= kmx[i] for i in range(3))
        piece = p.solid if inside else (p.solid & keep)
        if piece is None or not piece.solids():
            continue
        m = to_pv(pv, piece)
        if m is not None:
            items.append((m, p.colour))
        for f in piece.faces():
            c = tuple(f.center())
            nrm = tuple(f.normal_at())
            if abs(c[k] - plane_value) < 1e-5 and abs(abs(nrm[k]) - 1) < 1e-6:
                cm = to_pv(pv, f)
                if cm is not None:
                    off = [0, 0, 0]
                    off[k] = 0.05 if tuple(keep.center())[k] < plane_value else -0.05
                    caps.append((cm.translate(off), RED, True))
    return items, caps


def _t(msg, t0):
    import sys
    import time
    print(f"  render {msg}: {time.time() - t0:.2f} s", file=sys.stderr, flush=True)
    return time.time()


def _pv_mesh(pv, v, f, rgb=None):
    m = pv.PolyData(v, np.hstack([np.full((len(f), 1), 3), f]).ravel())
    if rgb is not None and len(rgb) == len(f):
        m.cell_data["body_rgb"] = np.round(np.asarray(rgb) * 255).astype(np.uint8)
    return m


def _add(pl, mesh, col, **kw):
    """add_mesh with the part colour, or the per-triangle body colours when the mesh carries them."""
    if "body_rgb" in mesh.cell_data:
        return pl.add_mesh(mesh, scalars="body_rgb", rgb=True, **kw)
    return pl.add_mesh(mesh, color=col, **kw)


def render_pyvista(A, outdir, joint=None):
    import time
    from lib import mesh as MS
    t0 = time.time()
    pv = _pv()
    t0 = _t("import pyvista", t0)
    os.makedirs(outdir, exist_ok=True)
    M = MS.meshes(A)
    FR = MS.face_rgb(A)
    pvm = {n: _pv_mesh(pv, v, f, FR.get(n)) for n, (v, f) in M.items()}
    meshes = [(pvm[n], A.parts[n].colour, False, A.parts[n].meta.get("opacity", 1.0)) for n in pvm]
    t0 = _t("meshes (shared tessellation)", t0)
    out = {}
    for fn, view, zoom in (("iso", "view_isometric", 1.0), ("top", "view_xy", 1.15),
                           ("front", "view_xz", 1.15), ("right", "view_yz", 1.15)):
        out[fn] = _shoot(pv, os.path.join(outdir, f"{fn}.png"), meshes, view, f"{A.name} {fn}",
                         parallel=(fn != "iso"), zoom=zoom)

    t0 = _t("4 views", t0)
    # section: XZ plane at Y 0, keep Y <= 0, look from +Y at the cut
    keep = Box(2000, 1000, 1000, align=(Align.CENTER, Align.MAX, Align.CENTER))
    items, caps = _cut(pv, A, keep, "y", 0.0)
    out["section"] = _shoot(pv, os.path.join(outdir, "section.png"), items + caps, None,
                            f"{A.name} section XZ at Y 0 (caps red)",
                            fit=(0.55, 1.0, 0.75), parallel=False, zoom=1.05)

    t0 = _t("section", t0)
    # section_joint: plane through one screw axis, perpendicular to its rail
    spec = joint or next(s for s in A.joints if abs(s["axis"][1]) > 0.99)
    at = np.array(spec["at"])
    axis = np.array(spec["axis"])
    rail = A.parts[spec["parts"][1]]
    rb = np.array(A.bbox(spec["parts"][1])[1]) - np.array(A.bbox(spec["parts"][1])[0])
    k = int(np.argmax(rb))                      # rail axis = section plane normal
    pa = "xyz"[k]
    half = 22.0
    size = [2 * half] * 3
    size[k] = half
    centre = at.copy()
    centre[k] -= half / 2                       # keep the half below the plane
    keep = Box(*size).moved(Location(tuple(centre)))
    names = {spec["screw_part"], spec["nut_part"], spec["parts"][0], spec["parts"][1]}
    items, caps = _cut(pv, A, keep, pa, at[k], pick=lambda n: n in names)
    focus = at + axis * 1.5
    side = np.cross(axis, np.eye(3)[k])
    eye = focus + np.eye(3)[k] * 75 - axis * 25 + side * 0 + np.array([0, 0, 28])
    up = (0, 0, 1)
    out["section_joint"] = _shoot(pv, os.path.join(outdir, "section_joint.png"), items + caps, None,
                                  f"{spec['name']}: {spec['screw']} into slot nut, plane through screw axis (caps red)",
                                  camera=[tuple(eye), tuple(focus), up], parallel=False)
    _t("section_joint", t0)
    out["section_joint_name"] = spec["name"]
    out["section_caps"] = len(caps)
    _ = rail
    if A.name.startswith(("froggy_v7", "froggy_v8", "froggy_v9", "froggy_v10", "froggy_v11", "froggy_v12")):
        out.update(_v7_renders(pv, A, outdir, pvm))
    elif any(s["name"].startswith("j_tower_flange_") for s in A.joints):
        out.update(_rotor_renders(pv, A, outdir, pvm))
    elif any(p.module == "rotor" for p in A.parts.values()):
        out.update(_v6_renders(pv, A, outdir, pvm))
    return out


def _cap_colour(c):
    return tuple(0.55 * v for v in c[:3])


def _cut_plane(pv, A, origin, normal, pick, half=400.0):
    """B-rep section by an arbitrary plane: keep the half-space on the -normal
    side (the viewer looks from +normal); caps = cut faces on the plane, in a
    darkened copy of each part's colour with a black outline."""
    from build123d import Location, Plane
    o, n = np.asarray(origin, float), np.asarray(normal, float)
    n = n / np.linalg.norm(n)
    up = np.array([0, 0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0, 0])
    keep = Plane(origin=tuple(o - n * half), x_dir=tuple(n), z_dir=tuple(np.cross(n, np.cross(up, n)))) * \
        Box(2 * half, 2 * half, 2 * half)
    kb = keep.bounding_box()
    kmn, kmx = tuple(kb.min), tuple(kb.max)
    items, caps = [], []
    for name, p in A.parts.items():
        if not pick(name):
            continue
        mn, mx = A.bbox(name)
        corners = np.array([[x, y, z] for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])])
        side = (corners - o) @ n
        if side.min() > 0:
            continue
        piece = p.solid if side.max() <= 0 else (p.solid & keep)
        if piece is None or not piece.solids():
            continue
        m = to_pv(pv, piece)
        if m is not None:
            items.append((m, p.colour, False, p.meta.get("opacity", 1.0)))
        for f in piece.faces():
            c, fn = np.array(tuple(f.center())), np.array(tuple(f.normal_at()))
            if abs((c - o) @ n) < 1e-4 and abs(abs(fn @ n) - 1) < 1e-6:
                cm = to_pv(pv, f)
                if cm is not None:
                    caps.append((cm.translate(tuple(n * 0.05)), _cap_colour(p.colour), True))
        _ = kmn, kmx
    return items, caps


def _joint(A, prefix):
    return next(s for s in A.joints if s["name"].startswith(prefix))


def _rotor_renders(pv, A, outdir, pvm):
    import time
    t0 = time.time()
    out = {}
    stack = {n for n, p in A.parts.items() if p.module in ("drive_base", "rotor")}
    # section_motor: XZ through the motor axis, drive + rotor only, looking from +Y
    items, caps = _cut_plane(pv, A, (0, 0, 0), (0, 1, 0), lambda n: n in stack or n.startswith("s_rail_1515_5")
                             or n.startswith("s_rail_1515_6"))
    out["section_motor"] = _shoot(pv, os.path.join(outdir, "section_motor.png"), items + caps, None,
                                  "section_motor: XZ through the motor axis (motor, tower, 6814 pair, hub, carrier, "
                                  "drum; cut faces darker)", fit=(0, 1, 0), parallel=True, zoom=1.0)
    t0 = _t("section_motor", t0)

    def screw_section(fname, spec, label, extra=(), span=24.0, normal=None):
        at, axis = np.array(spec["at"]), np.array(spec["axis"])
        nrm = np.array(normal, float) if normal is not None else np.cross(axis, [0, 0, 1.0] if abs(axis[2]) < .9 else [1.0, 0, 0])
        nrm /= np.linalg.norm(nrm)
        names = {spec["screw_part"], spec.get("nut_part"), *spec["parts"], *extra}
        names.discard(None)
        items, caps = _cut_plane(pv, A, at, nrm, lambda n: n in names)
        lo = np.array(A.bbox(spec["screw_part"])[0])
        hi = np.array(A.bbox(spec["screw_part"])[1])
        focus = (lo + hi) / 2
        side = np.cross(nrm, [0, 0, 1.0])
        eye = focus + nrm * 120 + np.array([0, 0, 40]) + side * 30
        out[fname] = _shoot(pv, os.path.join(outdir, f"{fname}.png"), items + caps, None, label,
                            camera=[tuple(eye), tuple(focus), (0, 0, 1)], parallel=True, zoom=1.0)
        # tighten the parallel scale around the joint
        return out[fname]

    corner = _joint(A, "j_corner_")
    screw_section("section_corner", corner, f"section_corner: {corner['name']} M3 x 8 into a slot nut "
                  "(plane through the screw axis, cut faces darker)")
    t0 = _t("section_corner", t0)
    fl = _joint(A, "j_tower_flange_")
    screw_section("section_flange", fl, f"section_flange: {fl['name']} M3 x 8 counterbored into a cross-rail slot nut",
                  normal=(1, 0, 0))
    t0 = _t("section_flange", t0)
    lug = _joint(A, "j_plate_lug_000")
    mot = _joint(A, "j_motor_face_pxpy")
    a, b = np.array(lug["at"]), np.array(mot["at"])
    d = b - a
    nrm = np.cross(d, [0, 0, 1.0])
    nrm /= np.linalg.norm(nrm)
    names = {lug["screw_part"], lug["nut_part"], mot["screw_part"], "fp5_motor_plate", "fp5_motor_tower",
             "v_nema17_17hs19_2004d_e1k", "fp_bearing_retainer", "v_bearing_6814_2rs_upper"}
    if nrm @ np.array([0.5, 1, 0]) < 0:
        nrm = -nrm
    items, caps = _cut_plane(pv, A, a, nrm, lambda n: n in names)
    focus = (a + b) / 2 + np.array([0, 0, 0])
    side = np.cross(nrm, [0, 0, 1.0])
    out["section_plate"] = _shoot(pv, os.path.join(outdir, "section_plate.png"), items + caps, None,
                                  f"section_plate: {lug['name']} (M3 x 12 into a lug insert) and {mot['name']} "
                                  "(M3 x 10 tapped into the motor face)",
                                  camera=[tuple(focus + nrm * 120 + side * 10 + np.array([0, 0, 25])), tuple(focus),
                                          (0, 0, 1)], parallel=True, zoom=1.0)
    t0 = _t("section_plate", t0)
    # exploded: drive stack +40, rotor +160 (120 above the lifted drive stack)
    lift = {"frame": 0.0, "drive_base": 40.0, "rotor": 160.0}
    items = [(pvm[n].translate((0, 0, lift[A.parts[n].module]), inplace=False), A.parts[n].colour, False,
              A.parts[n].meta.get("opacity", 1.0)) for n in pvm]
    out["exploded"] = _shoot(pv, os.path.join(outdir, "exploded.png"), items, "view_isometric",
                             "exploded: drive stack +40 mm, rotor +160 mm", parallel=False, zoom=1.0)
    _t("exploded", t0)
    return out


def _box_pick(A, lo, hi):
    def pick(n):
        mn, mx = A.bbox(n)
        return all(mn[k] >= lo[k] and mx[k] <= hi[k] for k in range(3))
    return pick


def _v6_renders(pv, A, outdir, pvm):
    """froggy v6: section_motor (XZ through the axis, drive + rotor + frogs),
    section_post (through a post corner), section_bearing (close-up of the
    bearing stack) and exploded (lifted by install step)."""
    import time
    t0 = time.time()
    out = {}
    stack = {n for n, p in A.parts.items() if p.module in ("drive_base", "rotor", "electrical")}
    stack |= {"s_rail_1515_5", "s_rail_1515_6"}
    # keep Y >= 0 (look from -Y): the wall frog on the +Y wall stays in view
    items, caps = _cut_plane(pv, A, (0, 0, 0), (0, -1, 0), lambda n: n in stack)
    out["section_motor"] = _shoot(pv, os.path.join(outdir, "section_motor.png"), items + caps, None,
                                  "section_motor: XZ through the axis (motor, coupling, shaft, bearings, "
                                  "hub, carrier, drum, frogs; cut faces darker)", fit=(0, -1, 0), parallel=True, zoom=1.0)
    t0 = _t("section_motor", t0)
    post = next(n for n, p in A.parts.items() if p.meta.get("role") == "post" and A.bbox(n)[0][0] > 0
                and A.bbox(n)[0][1] > 0)
    (pmn, pmx) = A.bbox(post)
    cy = (pmn[1] + pmx[1]) / 2
    pick = _box_pick(A, (pmn[0] - 60, pmn[1] - 40, -130), (pmx[0] + 10, pmx[1] + 10, 60))
    items, caps = _cut_plane(pv, A, (0, cy, 0), (0, 1, 0), pick)
    out["section_post"] = _shoot(pv, os.path.join(outdir, "section_post.png"), items + caps, None,
                                 f"section_post: plane through {post} (post, long rail end, Tovi ties, foot screw; "
                                 "cut faces darker)", fit=(0.3, 1, 0.25), parallel=True, zoom=1.0)
    t0 = _t("section_post", t0)
    pick = _box_pick(A, (-62, -62, -30), (62, 62, 90))
    items, caps = _cut_plane(pv, A, (0, 0, 0), (0, -1, 0),
                             lambda n: pick(n) and A.parts[n].module in ("drive_base", "rotor", "electrical"))
    out["section_bearing"] = _shoot(pv, os.path.join(outdir, "section_bearing.png"), items + caps, None,
                                    "section_bearing: drive and rotor parts within r 62 of the axis "
                                    "(cut faces darker)", fit=(0, -1, 0), parallel=True, zoom=1.0)
    t0 = _t("section_bearing", t0)
    lift_step = {1: -70.0, 2: 40.0, 3: 110.0, 4: 170.0, 5: 230.0, 6: 300.0}
    fixed = {"frame": 0.0, "cage": 0.0, "feet": -60.0}

    def lift(n):
        p = A.parts[n]
        return fixed.get(p.module, lift_step.get(p.meta.get("step", 0), 0.0))
    items = [(pvm[n].translate((0, 0, lift(n)), inplace=False), A.parts[n].colour, False,
              A.parts[n].meta.get("opacity", 1.0)) for n in pvm]
    out["exploded"] = _shoot(pv, os.path.join(outdir, "exploded.png"), items, "view_isometric",
                             "exploded by install step: motor stack -70, bearing +40, hub +110, carrier +170, "
                             "camera +230, drum + frogs +300, feet -60", parallel=False, zoom=1.0)
    _t("exploded", t0)
    return out


def _v7_renders(pv, A, outdir, pvm):
    """froggy v7: section_motor, section_hub (close XZ of the drive and hub stack),
    section_cage (YZ through the +Y strut: hoops, mesh, strut, seat), exploded (by
    install step), cage_off (cage module hidden) and plinth_open (side panels hidden)."""
    import time
    t0 = time.time()
    out = {}
    mods = ("drive_base", "rotor", "electrical")
    cross = {n for n, p in A.parts.items() if p.meta.get("role", "").startswith("cross")}
    pick = _box_pick(A, (-100, -200, -100), (100, 200, 400))
    normal = getattr(A, "motor_section_normal", (0, -1, 0))
    items, caps = _cut_plane(pv, A, (0, 0, 0), normal,
                             lambda n: (A.parts[n].module in mods and pick(n)) or n in cross)
    out["section_motor"] = _shoot(pv, os.path.join(outdir, "section_motor.png"), items + caps, None,
                                  getattr(A, "motor_section_caption", "section_motor: XZ through the axis (motor, coupling, shaft, bearings, "
                                  "hub, carrier, drum, frogs; cut faces darker)"), fit=normal, parallel=True, zoom=1.0)
    t0 = _t("section_motor", t0)
    pick = _box_pick(A, (-100, -100, -30), (100, 100, 100))
    items, caps = _cut_plane(pv, A, (0, 0, 0), (0, -1, 0), lambda n: (pick(n) and A.parts[n].module in mods)
                             or n in cross)
    out["section_hub"] = _shoot(pv, os.path.join(outdir, "section_hub.png"), items + caps, None,
                                     "section_hub: drive, hub and carrier stack within r 100 of the axis "
                                     "(cut faces darker)", fit=(0, -1, 0), parallel=True,
                                     zoom=1.0)
    t0 = _t("section_hub", t0)
    if getattr(A, "cage_explode", None):         # froggy v9 S3 (I030): XZ through the axis, drum, frogs, side panels
        items, caps = _cut_plane(pv, A, (0, 0.0, 0), (0, -1, 0),
                                 lambda n: A.parts[n].module == "cage" or n in ("s_drum_tube", "s_top_disc")
                                 or n.startswith(("v_frog", "lp_ring_c", "lp_ring_top", "fp9_tube_seat", "lp_carrier_disc", "s_panel_deck")))
        out["section_cage"] = _shoot(pv, os.path.join(outdir, "section_cage.png"), items + caps, None,
                                     "section_cage: XZ through the axis: side mesh panels on the corner nodes, top "
                                     "panel, drum and frogs inside the cube; cut faces darker",
                                     fit=(0, -1, 0), parallel=True, zoom=1.0)
    elif getattr(A, "cage_lift", None):          # froggy v9 S3 before I030: XZ through the two side struts (y -70)
        items, caps = _cut_plane(pv, A, (0, -70.0, 0), (0, -1, 0),
                                 lambda n: A.parts[n].module == "cage" or n in ("s_drum_tube", "s_top_disc")
                                 or n.startswith(("v_frog", "lp_ring_c", "lp_ring_top", "fp9_tube_seat", "lp_carrier_disc", "s_panel_deck"))
                                 or n.startswith("hw_slot_nut_m5_j_s3"))
        out["section_cage"] = _shoot(pv, os.path.join(outdir, "section_cage.png"), items + caps, None,
                                     "section_cage: XZ through the side struts (y -70): struts on the deck sheet, "
                                     "ring pairs, net sleeve and top, drum and frogs; cut faces darker",
                                     fit=(0, -1, 0), parallel=True, zoom=1.0)
    else:
        items, caps = _cut_plane(pv, A, (0, 0, 0), (1, 0, 0),
                                 lambda n: A.parts[n].module == "cage" or n in ("s_drum_tube", "s_top_disc",
                                                                                 "fp_disc_clamp_ring", "fp6_drum_carrier")
                                 or n.startswith("v_frog"))
        out["section_cage"] = _shoot(pv, os.path.join(outdir, "section_cage.png"), items + caps, None,
                                     "section_cage: YZ through the +Y strut (seat, Keiran hoop pairs, mesh sleeve and top, "
                                     "strut, drum and frogs; cut faces darker)", fit=(1, 0, 0), parallel=True, zoom=1.0)
    t0 = _t("section_cage", t0)
    lift_step = {1: -60.0, 2: 40.0, 3: 100.0, 4: 160.0, 5: 220.0, 6: 290.0, 7: 0.0, 8: 420.0}
    fixed = {"frame": 0.0, "feet": -50.0}
    cage_lift = getattr(A, "cage_lift", None)
    if cage_lift:
        fixed["cage"] = 420.0                     # froggy v9: the cage module explodes as one body group

    def lift(n):
        p = A.parts[n]
        return fixed.get(p.module, lift_step.get(p.meta.get("step", 0), 0.0))
    items = [(pvm[n].translate((0, 0, lift(n)), inplace=False), A.parts[n].colour, False,
              A.parts[n].meta.get("opacity", 1.0)) for n in pvm]
    out["exploded"] = _shoot(pv, os.path.join(outdir, "exploded.png"), items, "view_isometric",
                             "exploded by install step: motor -60, drive +40, hub +100, carrier +160, camera +220, "
                             "drum + frogs +290, cage +420, feet -50", parallel=False, zoom=1.0)
    t0 = _t("exploded", t0)
    if cage_lift:
        items = [(pvm[n].translate((0, 0, cage_lift if A.parts[n].module == "cage" else 0.0), inplace=False),
                  A.parts[n].colour, False, A.parts[n].meta.get("opacity", 1.0)) for n in pvm]
        label = (f"{A.name}: cage module lifted {cage_lift:g} mm as one body (tube posts and corners, mesh panels, "
                 "magnets, thumb screws); T-nuts stay in the deck rails") if getattr(A, "cage_explode", None) else \
            (f"{A.name}: cage module lifted {cage_lift:g} mm as one body (tube struts in printed sockets, rings, "
             "net, thumb screws); T-nuts stay in the deck rails")
    else:
        items = [(pvm[n], A.parts[n].colour, False, A.parts[n].meta.get("opacity", 1.0)) for n in pvm
                 if A.parts[n].module != "cage"]
        label = f"{A.name} with the net cage hidden"
    out["cage_off"] = _shoot(pv, os.path.join(outdir, "cage_off.png"), items, "view_isometric", label,
                             parallel=False, zoom=1.0)
    t0 = _t("cage_off", t0)
    items = [(pvm[n], A.parts[n].colour, False, A.parts[n].meta.get("opacity", 1.0)) for n in pvm
             if A.parts[n].module in ("cage", "rotor") or n.startswith("v_ir_led")]
    out["cage_detail"] = _shoot(pv, os.path.join(outdir, "cage_detail.png"), items, None,
                                ("cage_detail: 12 x 1 aluminium tube posts in printed corners, five magnet-on mesh "
                                 "panels, see-through netting (rotor inside)") if getattr(A, "cage_explode", None) else
                                ("cage_detail: 12 x 1 aluminium tube struts in printed sockets, aluminium ring pairs, "
                                 "see-through net sleeve and top (rotor inside)") if cage_lift else
                                "cage_detail: seat ring, Keiran 220 hoop pairs, see-through mesh sleeve and top, struts "
                                "(rotor inside)", camera=[(470, -540, 470), (0, 0, 135), (0, 0, 1)], parallel=False)
    t0 = _t("cage_detail", t0)
    if getattr(A, "cage_explode", None):         # froggy v9 S3 (I030): panels pulled off along their normals
        ex = A.cage_explode
        items = [(pvm[n].translate(ex[n], inplace=False), A.parts[n].colour, False, A.parts[n].meta.get("opacity", 1.0))
                 for n in ex if n in pvm]
        out["cage_exploded"] = _shoot(pv, os.path.join(outdir, "cage_exploded.png"), items, "view_isometric",
                                      "cage_exploded (S3 parts only): the five mesh panels pulled off their magnets "
                                      "110 out (top 120 up), tube frame and corners in place, foot screws and T-nuts -50",
                                      parallel=False, zoom=1.0)
        t0 = _t("cage_exploded", t0)
    elif cage_lift:                              # froggy v9 S3 page: cage parts only, exploded by install step
        items = []
        for n, p in A.parts.items():
            if n not in pvm or (p.module != "cage" and not n.startswith("hw_slot_nut_m5_j_s3")):
                continue
            mn, mx = A.bbox(n)
            cx, cy, zc = ((mn[i] + mx[i]) / 2 for i in range(3))
            r = max(math.hypot(cx, cy), 1.0)
            if "thumb" in n or n.startswith("hw_slot_nut_m5_j_s3"):
                dx, dy, dz = cx / r * 60, cy / r * 60, -60.0                 # thumb screws and their T-nuts
            elif n.startswith(("s_rail_2020_strut", "v_cast_bracket_2020_s3", "s_strut_tube", "fp9_strut_")) \
                    or "_leg_strut_" in n:
                dx, dy, dz = cx / r * 60, cy / r * 60, 0.0                    # struts with their 3 sockets
            elif n.startswith("s_mesh_sleeve"):
                dx, dy, dz = 0.0, 0.0, 0.0
            else:
                dx, dy, dz = 0.0, 0.0, (-70.0 if zc < 100 else 90.0)          # ring pairs, mesh top, bolts
            items.append((pvm[n].translate((dx, dy, dz), inplace=False), p.colour, False, p.meta.get("opacity", 1.0)))
        out["cage_exploded"] = _shoot(pv, os.path.join(outdir, "cage_exploded.png"), items, "view_isometric",
                                      "cage_exploded (S3 parts only): struts with their brackets out 60 radially, "
                                      "bottom ring pair -70, top ring pair +90, thumb screws -60", parallel=False,
                                      zoom=1.0)
        t0 = _t("cage_exploded", t0)
    hide = {"s_panel_front", "s_panel_back", "s_panel_left", "s_panel_right"}
    items = [(pvm[n], A.parts[n].colour, False, A.parts[n].meta.get("opacity", 1.0)) for n in pvm
             if n not in hide and A.parts[n].module != "cage"]
    out["plinth_open"] = _shoot(pv, os.path.join(outdir, "plinth_open.png"), items, None,
                                f"{A.name} plinth with the side panels and cage hidden (electronics on the bottom panel)",
                                fit=(0.8, -1.0, 0.35), parallel=False, zoom=1.25)
    _t("plinth_open", t0)
    out.update(_panel_renders(pv, A, outdir, pvm))
    out.update(_pov_renders(pv, A, outdir, pvm))
    out.update(_rotor_top_render(pv, A, outdir, pvm))
    out.update(_drive_renders(pv, A, outdir, pvm))
    return out


def _rotor_top_render(pv, A, outdir, pvm):
    """rotor_top: parallel view straight down the rotor axis at the carrier disc, tube hidden and
    everything above A.rotor_top['z_cut'] left out (top disc, rings, frogs, cage). Only for assemblies
    that set A.rotor_top = dict(z_cut, z_min, r_max, hide)."""
    import math
    import time
    cfg = getattr(A, "rotor_top", None)
    if not cfg:
        return {}
    t0 = time.time()
    hide = set(cfg.get("hide", ()))

    def keep(n):
        if n in hide:
            return False
        (x0, y0, z0), (x1, y1, z1) = A.bbox(n)
        rmax = max(math.hypot(x, y) for x in (x0, x1) for y in (y0, y1))
        return z0 >= cfg["z_min"] and z1 <= cfg["z_cut"] and rmax <= cfg["r_max"]
    items = [(pvm[n], A.parts[n].colour, False, A.parts[n].meta.get("opacity", 1.0)) for n in pvm if keep(n)]
    z = cfg["z_min"]
    path = os.path.join(outdir, "rotor_top.png")
    pl = _plotter(pv)
    for mesh, col, _, op in items:
        _add(pl, mesh, col, smooth_shading=False, opacity=op)
    pl.camera_position = [(0, 0, z + 800.0), (0, 0, z), (0, 1, 0)]
    pl.enable_parallel_projection()
    pl.reset_camera()
    pl.camera.zoom(1.05)
    pl.add_text(f"rotor_top: straight down the axis at the carrier disc (tube hidden, cut at Z {cfg['z_cut']:g}; "
                "+X right, +Y up)", font_size=10, color="black")
    pl.screenshot(path)
    pl.close()
    _t("rotor_top", t0)
    return {"rotor_top": path}


def _panel_renders(pv, A, outdir, pvm):
    """plinth_front (front panel and e-stop), plinth_back (IEC inlet, USB-C) and plinth_vent (right panel
    vents): from outside, perspective, slightly above eye level, cage hidden. Only for assemblies that
    set A.meta_panels."""
    import time
    if not getattr(A, "meta_panels", None):
        return {}
    t0 = time.time()
    out = {}
    items = [(pvm[n], A.parts[n].colour, False, A.parts[n].meta.get("opacity", 1.0)) for n in pvm
             if A.parts[n].module != "cage"]
    for key, eye, focus, label in (
            ("plinth_front", (70.0, -720.0, 40.0), (0.0, -140.0, -60.0),
             "plinth_front: front panel from outside (BACO e-stop station recessed in the panel on its printed cradle, yellow cover just proud, cage hidden)"),
            ("plinth_back", (-70.0, 720.0, 40.0), (0.0, 140.0, -60.0),
             "plinth_back: back panel from outside (IEC C14 inlet with its 2 x M3, USB-C; cage hidden)"),
            ("plinth_vent", (720.0, 70.0, 40.0), (140.0, 0.0, -60.0),
             "plinth_vent: right panel from outside (vent slots beside the PSU; cage hidden)")):
        out[key] = _shoot(pv, os.path.join(outdir, f"{key}.png"), items, None, label,
                          camera=[eye, focus, (0, 0, 1)], parallel=False)
        t0 = _t(key, t0)
    return out


def _pov_renders(pv, A, outdir, pvm):
    """camera_view (perspective from the assembly's camera_pov lens datum along its
    optical axis, wide view angle, camera body and carrier hidden) and frogs_closeup
    (close iso of the drum top half). Only for assemblies that set A.camera_pov."""
    import time
    pov = getattr(A, "camera_pov", None)
    if not pov:
        return {}
    t0 = time.time()
    out = {}
    eye = np.array(pov["eye"], float)
    ax = np.array(pov["axis"], float)
    ax /= np.linalg.norm(ax)
    hide = set(pov.get("hide", ()))
    items = [(pvm[n], A.parts[n].colour, False, A.parts[n].meta.get("opacity", 1.0)) for n in pvm
             if n not in hide]
    up = np.cross(np.cross(ax, (0.0, 0.0, 1.0)), ax)
    up = up / np.linalg.norm(up) if np.linalg.norm(up) > 1e-9 else np.array((1.0, 0.0, 0.0))
    pl = _plotter(pv)
    for mesh, col, _, op in items:
        _add(pl, mesh, col, smooth_shading=False, opacity=op)
    pl.camera_position = [tuple(eye), tuple(eye + ax * 100.0), tuple(up)]
    pl.camera.view_angle = float(pov.get("fov", 120.0))
    pl.camera.clipping_range = (0.5, 2000.0)
    pl.add_text(f"camera_view: rotor fisheye POV from the lens datum along the optical axis, "
                f"{pov.get('fov', 120.0):.0f} deg view angle", font_size=10, color="black")
    out["camera_view"] = os.path.join(outdir, "camera_view.png")
    pl.screenshot(out["camera_view"])
    pl.close()
    t0 = _t("camera_view", t0)
    f = np.array(pov.get("closeup_focus", (0.0, 0.0, 200.0)), float)
    items = [(pvm[n], A.parts[n].colour, False, A.parts[n].meta.get("opacity", 1.0)) for n in pvm
             if A.parts[n].module != "cage"]
    out["frogs_closeup"] = _shoot(pv, os.path.join(outdir, "frogs_closeup.png"), items, None,
                                  "frogs_closeup: drum top half from +X, wall frog on the tube and disc frog on the "
                                  "top disc (cage hidden)",
                                  camera=[tuple(f + np.array((210.0, -150.0, 120.0))), tuple(f), (0, 0, 1)],
                                  parallel=False)
    _t("frogs_closeup", t0)
    return out


def render_blender(glb, outdir):
    dst = os.path.join(outdir, "iso_blender.png")
    log = os.path.join(outdir, "blender.log")
    with open(log, "w") as fh:
        rc = subprocess.run([BLENDER, "-b", "--factory-startup", "--python",
                             os.path.join(HERE, "render_blender.py"), "--", glb, dst],
                            stdout=fh, stderr=subprocess.STDOUT, timeout=90).returncode
    if rc != 0 or not os.path.exists(dst):
        raise RuntimeError(f"blender rc={rc}, see {log}")
    return {"iso_blender": dst}


MAGENTA = (0.88, 0.10, 0.78)


def _drive_renders(pv, A, outdir, pvm):
    """froggy v9 S1.26 drive-base page, for assemblies that set A.drive_cartridge / drive_explode / drive_section /
    drive_install: drive_exploded (cartridge parts only, exploded along Z), drive_section (section through the axis
    and one rim roller at spindle scale, Z -110..60, both pillow-block bearings in magenta, every part labelled) and
    drive_install (a montage, one frame per install step)."""
    import time
    if not getattr(A, "drive_cartridge", None):
        return {}
    t0 = time.time()
    out = {}

    def item(n, col=None, shift=(0, 0, 0)):
        p = A.parts[n]
        m = pvm[n] if shift == (0, 0, 0) else pvm[n].translate(shift, inplace=False)
        return (m, col or p.colour, False, p.meta.get("opacity", 1.0))

    # drive_exploded
    ex = A.drive_explode
    items = [item(n, shift=tuple(ex.get(n, (0, 0, 0)))) for n in A.drive_cartridge if n in pvm]
    out["drive_exploded"] = _shoot(pv, os.path.join(outdir, "drive_exploded.png"), items, None,
                                   getattr(A, "drive_exploded_caption", "drive_exploded (S1.26 cartridge only, along Z): motor, motor plate + nylocs, motor "
                                   "screws, coupling, spacers, plate A + roller blocks, collar, pillow block, REX shaft, "
                                   "M5 x 50 bolts, hub; rim rollers out 45 radially"), fit=(0.9, -1.0, 0.55),
                                   parallel=False, zoom=1.05)
    t0 = _t("drive_exploded", t0)

    # drive_section
    sec = A.drive_section
    u, n = np.array(sec["u"], float), np.array(sec["normal"], float)
    pick = set(sec["pick"])
    hl = set(sec["highlight"])
    zlo, zhi = sec["z_range"]
    items, caps = _cut_plane(pv, A, (0, 0, 0), n, lambda q: q in pick and q not in hl and q in A.parts)
    # recolour the two bearings (and their caps) magenta: redo them on their own
    keep_items = []
    hl_items, hl_caps = _cut_plane(pv, A, (0, 0, 0), n, lambda q: q in hl)
    for it in items:
        keep_items.append(it)
    for m, *_ in hl_items:
        keep_items.append((m, MAGENTA, False, 1.0))
    for m, *_ in hl_caps:
        caps.append((m.translate(tuple(n * 0.02), inplace=False), (0.55, 0.0, 0.48), True))
    W, H = 2000, 1300
    scale = (zhi - zlo) / 2 + 12.0
    zc = (zlo + zhi) / 2
    focus = np.array([0.0, 0.0, zc])
    pl = pv.Plotter(off_screen=True, window_size=(W, H))
    pl.set_background("white")
    for it in keep_items + caps:
        mesh, col = it[0], it[1]
        op = it[3] if len(it) > 3 else 1.0
        _add(pl, mesh, col, smooth_shading=False, opacity=op)
        if len(it) > 2 and it[2]:
            e = mesh.extract_feature_edges(boundary_edges=True, feature_edges=False, manifold_edges=False,
                                           non_manifold_edges=False)
            if e.n_points:
                pl.add_mesh(e, color="black", line_width=2)
    up = np.array([0, 0, 1.0])
    pl.camera_position = [tuple(focus + n * 1500.0), tuple(focus), tuple(up)]
    pl.enable_parallel_projection()
    pl.camera.parallel_scale = scale
    pl.camera.clipping_range = (1.0, 4000.0)
    k = (H / 2) / scale                                   # px per mm
    right = np.cross(-n, up)

    def px(p):
        d = np.asarray(p, float) - focus
        return W / 2 + (d @ right) * k, H / 2 + (d @ up) * k

    labs = []
    for text, s_, z in sec["labels"]:
        p = u * s_ + np.array([0, 0, z]) + n * 2.0
        x, y = px(p)
        labs.append([text, p, x, y])
    fs = 15
    for side in (-1, 1):
        col = sorted([l for l in labs if (l[2] >= W / 2) == (side > 0)], key=lambda l: l[3])
        ys = []
        for l in col:
            y = max(l[3], (ys[-1] + 34) if ys else 20)
            ys.append(y)
        over = (ys[-1] - (H - 60)) if ys else 0
        if over > 0:
            ys = [y - over for y in ys]
        for l, y in zip(col, ys):
            tx = W - 250 if side > 0 else 20
            wx = (tx - 8) if side > 0 else (20 + int(fs * 0.62 * len(l[0])) + 8)
            q = focus + right * ((wx - W / 2) / k) + up * ((y - H / 2) / k) + n * 2.0
            pl.add_mesh(pv.Line(tuple(l[1]), tuple(q)), color="black", line_width=1)
            pl.add_mesh(pv.Sphere(radius=0.9, center=tuple(l[1])), color="black")
            pl.add_text(l[0], position=(tx, int(y - fs * 0.5)), font_size=fs - 4, color="black")
    pl.add_text(getattr(A, "drive_section_caption", "drive_section: through the rotor axis and the 330 deg rim roller (Z -110..60), cut faces darker, the "
                "two goBILDA 1614 bearings in magenta"), position="upper_edge", font_size=11, color="black")
    out["drive_section"] = os.path.join(outdir, "drive_section.png")
    pl.screenshot(out["drive_section"])
    pl.close()
    t0 = _t("drive_section", t0)

    # drive_install montage
    frames = A.drive_install
    present, ctx = [], []
    tiles = []
    allc = [q for q in A.drive_cartridge if q in pvm]
    b = np.array([A.bbox(q)[0] for q in allc]).min(0), np.array([A.bbox(q)[1] for q in allc]).max(0)
    cen = (b[0] + b[1]) / 2
    rad = float(np.linalg.norm(b[1] - b[0]) / 2)
    from PIL import Image
    for i, (title, add, rem, context) in enumerate(frames, 1):
        prev = list(present)
        present = [q for q in present if q not in rem] + [q for q in add if q in pvm and q not in present]
        pl = pv.Plotter(off_screen=True, window_size=(700, 640))
        pl.set_background("white")
        for q in context:
            if q in pvm:
                pl.add_mesh(pvm[q], color=(0.72, 0.72, 0.72), opacity=0.45, smooth_shading=False)
        for q in present:
            new = q not in prev
            p = A.parts[q]
            pl.add_mesh(pvm[q], color=p.colour if new else (0.80, 0.80, 0.82), smooth_shading=False,
                        opacity=p.meta.get("opacity", 1.0) if new else 1.0)
        d = np.array([0.9, -1.0, 0.6])
        d /= np.linalg.norm(d)
        pl.camera_position = [tuple(cen + d * 900.0), tuple(cen), (0, 0, 1)]
        pl.enable_parallel_projection()
        pl.camera.parallel_scale = rad * 0.82
        pl.camera.clipping_range = (1.0, 4000.0)
        pl.add_text(title, position="upper_left", font_size=10, color="black")
        path = os.path.join(outdir, f"_drive_install_{i:02d}.png")
        pl.screenshot(path)
        pl.close()
        tiles.append(path)
    cols = 4
    rows = (len(tiles) + cols - 1) // cols
    ims = [Image.open(t).convert("RGB") for t in tiles]
    tw, th = ims[0].size
    sheet = Image.new("RGB", (cols * tw, rows * th + 40), "white")
    for j, im in enumerate(ims):
        sheet.paste(im, ((j % cols) * tw, 40 + (j // cols) * th))
    from PIL import ImageDraw
    ImageDraw.Draw(sheet).text((12, 12), "drive_install: the S1.26 cartridge, one frame per install_order_s1 step "
                               "(new parts in colour, earlier parts grey, frame and deck sheet translucent)", fill=(0, 0, 0))
    out["drive_install"] = os.path.join(outdir, "drive_install.png")
    sheet.save(out["drive_install"])
    for t in tiles:
        os.remove(t)
    _t("drive_install", t0)
    for spec in getattr(A, "drive_detail_views", []):
        items = [item(n) for n in spec["parts"] if n in pvm]
        out[spec["name"]] = _shoot(pv, os.path.join(outdir, spec["name"] + ".png"), items, None,
                                   spec["caption"], fit=tuple(np.subtract(spec["camera"][0], spec["camera"][1])),
                                   parallel=True, zoom=0.9)
    return out
