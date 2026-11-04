"""Fixed, comparable, headless views of any STEP (PyVista off-screen, same renderer as froggy-cad).

Every camera is written out explicitly (eye direction, focal point, up, parallel scale) to
views.json, so the same model gives the same pixels on every run, and `--frame-from <dir>` reuses
another run's cameras so a before/after pair is pixel-comparable even when the bounding box moved.
Crease edges (feature angle 35 deg) are drawn in black: a sharp edge shows a line, a fillet shows
none, which makes unfinished edges easy to spot.

Views: iso (from front-right-top), front (eye at -Y), right (+X), top (+Z), back (+Y), bottom (-Z),
plus close-ups `--close name=x,y,z,r` (iso direction, focal x,y,z, half-height r mm).
"""
import json
import math
import os

import numpy as np

from .kinds import COLOURS, kind_of

PX = 1000
VIEWS = {   # name: (eye direction from the focal point, view up)
    "iso": ((1.0, -1.0, 0.8), (0, 0, 1)),
    "front": ((0.0, -1.0, 0.0), (0, 0, 1)),
    "right": ((1.0, 0.0, 0.0), (0, 0, 1)),
    "top": ((0.0, 0.0, 1.0), (0, 1, 0)),
    "back": ((0.0, 1.0, 0.0), (0, 0, 1)),
    "bottom": ((0.0, 0.0, -1.0), (0, 1, 0)),
}
ORDER = list(VIEWS)


def _pv():
    os.environ.setdefault("PYVISTA_OFF_SCREEN", "true")
    import pyvista as pv
    pv.OFF_SCREEN = True
    return pv


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def camera_for(direction, up, mn, mx, margin=1.08, focal=None, half=None):
    """Explicit camera dict for a view of the box mn..mx (parallel projection, square image)."""
    d = _unit(direction)
    up = np.asarray(up, float)
    up = _unit(up - (up @ d) * d)
    right = np.cross(up, d)  # screen x
    c = (np.asarray(mn) + np.asarray(mx)) / 2 if focal is None else np.asarray(focal, float)
    if half is None:
        corners = np.array([[x, y, z] for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])]) - c
        half = max(np.abs(corners @ right).max(), np.abs(corners @ up).max()) * margin
    diag = float(np.linalg.norm(np.asarray(mx) - np.asarray(mn))) or 1.0
    return dict(position=(c + d * 3 * diag).tolist(), focal=c.tolist(), up=up.tolist(), parallel_scale=float(half),
                direction=d.tolist())


def _meshes(pv, parts, lin):
    from .stepio import tessellate
    out = []
    for p in parts:
        v, f = tessellate(p.shape, lin, 0.12)   # fine angular deflection: no zipper ticks on long arcs
        if not len(f):
            continue
        m = pv.PolyData(v, np.hstack([np.full((len(f), 1), 3), f]).ravel())
        col = p.colour or COLOURS[kind_of(p.name, p.path)]
        out.append((m, col))
    return out


def _shoot(pv, meshes, cam, path, label, edges=True, px=PX):
    pl = pv.Plotter(off_screen=True, window_size=(px, px))
    pl.set_background("white")
    for m, col in meshes:
        pl.add_mesh(m, color=col, smooth_shading=True, split_sharp_edges=True, feature_angle=35)
        if edges:
            fe = m.extract_feature_edges(feature_angle=35, boundary_edges=False, feature_edges=True,
                                         manifold_edges=False, non_manifold_edges=False)
            if fe.n_points:
                pl.add_mesh(fe, color="black", line_width=1.5)
    pl.camera_position = [tuple(cam["position"]), tuple(cam["focal"]), tuple(cam["up"])]
    pl.enable_parallel_projection()
    pl.camera.parallel_scale = cam["parallel_scale"]
    pl.camera.clipping_range = (1e-3, 1e7)
    if label:
        pl.add_text(label, font_size=10, color="black")
    pl.screenshot(path)
    pl.close()
    return path


def render_parts(parts, outdir, title="", views=None, closes=None, frame=None, px=PX, edges=True):
    """Render parts (stepio.Part list) to outdir/<view>.png plus views.json and sheet.png.
    frame: a cameras dict from an earlier views.json to reuse. Returns {view: path}."""
    from .stepio import bbox
    pv = _pv()
    os.makedirs(outdir, exist_ok=True)
    mn, mx = bbox(parts)
    diag = float(np.linalg.norm(mx - mn)) or 1.0
    lin = min(0.1, max(0.005, diag / 4000))
    meshes = _meshes(pv, parts, lin)
    cams = {}
    for v in views or ORDER:
        d, up = VIEWS[v]
        cams[v] = (frame or {}).get(v) or camera_for(d, up, mn, mx)
    for name, (x, y, z, r) in (closes or {}).items():
        key = f"close_{name}"
        cams[key] = (frame or {}).get(key) or camera_for(VIEWS["iso"][0], VIEWS["iso"][1], mn, mx,
                                                           focal=(x, y, z), half=r)
    out = {}
    for v, cam in cams.items():
        out[v] = _shoot(pv, meshes, cam, os.path.join(outdir, f"{v}.png"), f"{title} {v}".strip(), edges, px)
    json.dump(dict(title=title, bbox=[mn.tolist(), mx.tolist()], cameras=cams, px=px),
              open(os.path.join(outdir, "views.json"), "w"), indent=1)
    out["sheet"] = contact_sheet([out[v] for v in cams], os.path.join(outdir, "sheet.png"), list(cams), title)
    return out


def _font(size):
    from PIL import ImageFont
    for f in ("/System/Library/Fonts/Helvetica.ttc", "/System/Library/Fonts/Supplemental/Arial.ttf",
              "/Library/Fonts/Arial.ttf"):
        if os.path.exists(f):
            try:
                return ImageFont.truetype(f, size)
            except Exception:
                pass
    return ImageFont.load_default()


def contact_sheet(paths, out, labels=None, title="", cols=3, tile=500):
    from PIL import Image, ImageDraw
    labels = labels or [os.path.splitext(os.path.basename(p))[0] for p in paths]
    rows = math.ceil(len(paths) / cols)
    head = 50 if title else 0
    sheet = Image.new("RGB", (cols * tile, rows * (tile + 30) + head), "white")
    dr = ImageDraw.Draw(sheet)
    if title:
        dr.text((12, 10), title, fill="black", font=_font(26))
    for i, (p, lab) in enumerate(zip(paths, labels)):
        im = Image.open(p).convert("RGB").resize((tile, tile))
        x, y = (i % cols) * tile, head + (i // cols) * (tile + 30)
        sheet.paste(im, (x, y))
        dr.text((x + 10, y + tile + 4), lab, fill="black", font=_font(20))
    sheet.save(out)
    return out


def montage(dir_a, dir_b, out=None, labels=("before", "after"), tile=600):
    """Before/after sheet: one row per view present in both dirs, A left, B right."""
    from PIL import Image, ImageDraw
    names = sorted({os.path.splitext(f)[0] for f in os.listdir(dir_a) if f.endswith(".png")}
                   & {os.path.splitext(f)[0] for f in os.listdir(dir_b) if f.endswith(".png")} - {"sheet", "montage"},
                   key=lambda n: (ORDER.index(n) if n in ORDER else len(ORDER), n))
    if not names:
        raise SystemExit(f"no common view PNGs in {dir_a} and {dir_b}")
    out = out or os.path.join(dir_b, "montage.png")
    head = 46
    sheet = Image.new("RGB", (2 * tile + 140, head + len(names) * tile), "white")
    dr = ImageDraw.Draw(sheet)
    f = _font(24)
    dr.text((140 + 10, 10), f"{labels[0]}: {dir_a}", fill="black", font=_font(16))
    dr.text((140 + tile + 10, 10), f"{labels[1]}: {dir_b}", fill="black", font=_font(16))
    for i, n in enumerate(names):
        y = head + i * tile
        dr.text((10, y + tile // 2 - 12), n, fill="black", font=f)
        for j, d in enumerate((dir_a, dir_b)):
            im = Image.open(os.path.join(d, f"{n}.png")).convert("RGB").resize((tile, tile))
            sheet.paste(im, (140 + j * tile, y))
        dr.line([(140 + tile, y), (140 + tile, y + tile)], fill=(200, 0, 0), width=2)
    sheet.save(out)
    return out


def views_step(step, outdir=None, closes=None, frame_from=None, views=None):
    from .stepio import load
    parts = load(step)
    outdir = outdir or os.path.join(os.path.dirname(os.path.abspath(step)),
                                    os.path.splitext(os.path.basename(step))[0] + "_views")
    frame = None
    if frame_from:
        frame = json.load(open(os.path.join(frame_from, "views.json")))["cameras"]
    return render_parts(parts, outdir, os.path.basename(step), views=views, closes=closes, frame=frame)
