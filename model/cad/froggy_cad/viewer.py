"""`bin/cad view <assembly>`: out/<assembly>/<assembly>.glb in the standard cadkit viewer (`cadkit view`,
cadkit/viewer, vendored next to this package in froggy-control). This layer only passes froggy's pieces: the module tree from
out/<name>/sheet.json, the review cameras of lib/review_views.py, and the build result for status.json
(build_ok, interference_pairs, design_violations, sheet_mtime). Default port 7891; head-view owns 7890.
bin/cad view resolves the GLB first (froggy_cad/outcache.py): fresh out/, else restore ~/cad/froggy-cad-cache/<name>/, else build.
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
try:
    from cadkit.viewer import server as CV
except ImportError:                       # cadkit not importable: use the copy vendored at ROOT/cadkit
    sys.path.insert(0, ROOT)
    from cadkit.viewer import server as CV

SHORT = {"close_front_panel": "panel", "close_drum_top": "drum top", "close_hub": "hub",
         "close_drive_underside": "underside", "drive_drum": "drive + drum"}


def _read_json(p):
    try:
        with open(p) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def build_status(out):
    """Newest build result: out/<name>/build.json if a build ever writes one, else sheet.json (bin/cad build)."""
    s = dict(build_ok=None, interference_pairs=None, design_violations=None, build_source=None, build_mtime=None,
             sheet_mtime=None)
    for fn in ("build.json", "sheet.json"):
        p = os.path.join(out, fn)
        d = _read_json(p) if os.path.exists(p) else None
        if not isinstance(d, dict):
            continue
        ok = d.get("ok", d.get("valid"))
        s["build_ok"] = bool(ok) if ok is not None else None
        ip = d.get("interference_pairs")
        if ip is None and isinstance(d.get("interference"), dict):
            ip = d["interference"].get("pairs")
        s["interference_pairs"] = ip
        s["design_violations"] = d.get("design_violations")
        s["build_source"] = p
        s["build_mtime"] = os.path.getmtime(p)
        break
    if s["design_violations"] is None:
        d = _read_json(os.path.join(out, "design.json"))
        if isinstance(d, dict):
            s["design_violations"] = d.get("violation_count")
    sp = os.path.join(out, "sheet.json")
    if os.path.exists(sp):
        s["sheet_mtime"] = os.path.getmtime(sp)
    return s


def views(name=None):
    """The fixed review cameras of lib/review_views.py (eye, focal, up, scale = half image height in mm, hide)."""
    from lib import review_views as RV
    keep = ("name", "caption", "eye", "focal", "up", "scale", "hide")
    vs = []
    for v in RV.VIEWS:
        d = {k: v[k] for k in keep if k in v}
        d["label"] = SHORT.get(v["name"], v["name"])
        vs.append(d)
    if name in ("froggy_v10", "froggy_v11", "froggy_v12"):
        for v in vs:
            if v["name"] == "iso":
                v.update(focal=[0,0,90], scale=300)
        vs.extend([
            dict(name="belt_drive", label="belt drive", caption="Compact belt drive and retained spindle",
                 eye=[220,-180,120], focal=[0,12,-10], up=[0,0,1], scale=70,
                 hide=[r"^(?!(?:v_gt2_|v_nema17_|v_gobilda_rex_shaft_|v_shaft_collar_|v_gobilda_1614_|v_gobilda_1313_|lp_motor_plate$|v_spacer_raf_|v_retainer_|hw_retainer_|hw_.*j_s1_6_)).*"]),
            dict(name="shaft_detail", label="shafts + belt", caption="REX and D shafts exposed; bearings and hub hidden for inspection",
                 eye=[180,-160,80], focal=[0,10,5], up=[0,0,1], scale=45,
                 hide=[r"^(?!(?:v_gt2_|v_nema17_shaft$|v_gobilda_rex_shaft_|v_retainer_|hw_retainer_|hw_.*j_s1_6_)).*"]),
        ])
    if name == "froggy_v12":
        # Drive-focused assembly review: retain all scoped fasteners and kits.
        import re
        manifest = _read_json(os.path.join(ROOT, "out", name, "assembly-package.json"))
        if manifest:
            by_id = {b["item"]: b["cad_components"] for b in manifest["bom"]}
            core_ids = ["D01", "D02", "D03", "D04", "D05", "D06", "D07", "D08", "D10", "D11", "D13", "H08"]
            cart_ids = core_ids + ["P01", "P02", "P05", "D09", "R01", "R02", "R03", "S01", "S02", "S03", "H01", "H02", "H03", "H04", "H05", "H06", "H07", "H10", "H11", "H13", "H14", "H16"]
            for key, label, codes, eye, focal, scale in [
                ("complete_drive", "complete drive", cart_ids, [220,-250,-75], [0,0,-18], 115),
                ("powertrain", "drive exposed", core_ids, [180,-150,55], [0,10,-20], 82),
            ]:
                names = sorted({n for code in codes for n in by_id[code]})
                vs.append(dict(name=key, label=label, caption="Complete V12 drive assembly" if key == "complete_drive" else "Motor, belt, pulleys and retained spindle; plates hidden", eye=eye, focal=focal, up=[0,0,1], scale=scale, hide=[r"^(?!(?:" + "|".join(re.escape(n) for n in names) + r")$).*"]))
        vs.extend([
            dict(name="frame_drive",label="frame + drive",caption="V12 frame, metal hardpoints and flat-tread Ender supports",
                 eye=[400,-500,260],focal=[0,0,-25],up=[0,0,1],scale=205,
                 hide=[r"^(?!(?:s_rail_|v_cast_bracket_|lp_clamp_tab_|lp_plate_a$|lp_motor_plate$|v_deck_hardpoint_|fp12_roller_|v_rim_roller_|v_spacer_|v_nema17_|v_gt2_|v_gobilda_1614_|v_gobilda_1313_|v_gobilda_rex_|v_shaft_collar_|v_retainer_|hw_retainer_|fp9_hall_|v_index_hall_|hw_.*j_(?:deck_corner|cross_end|upright_top|clamp_tab|s1_1_|s1_2_|s1_4_|s1_5_|s1_6_|s1_27_|s1_31_))).*"]),
            dict(name="rim_support",label="flat tread support",caption="Horizontal M5 axle; flat POM tread contacts disc underside",
                 eye=[110,-80,38],focal=[0,89,32],up=[0,0,1],scale=32,
                 hide=[r"^(?!(?:fp12_roller_block_1$|v_rim_roller_creality_4001070002_1$|v_spacer_axle_10x5x1_1$|hw_.*j_s1_27_(?:axle_1$|block_1_)|lp_plate_a$|lp_carrier_disc$)).*"]),
        ])
    # top-bar order: "drive + drum" sits right after "complete drive" when that view exists
    names = [v["name"] for v in vs]
    if "drive_drum" in names and "complete_drive" in names:
        dd = vs.pop(names.index("drive_drum"))
        vs.insert([v["name"] for v in vs].index("complete_drive") + 1, dd)
    cv = camera_view(name)
    if cv:
        vs.append(cv)                     # last: after the review views
    return dict(px=list(RV.PX), views=vs)


_LENS_CACHE = {}


def _lens(name):
    """Lens front, optical axis and sensor long axis of v_camera_uc261 in the built GLB, by the same code path
    as `bin/cad camsim` (lib.camsim.lens_from_assembly), cached per GLB mtime; camsim.json as the fallback."""
    out = os.path.join(ROOT, "out", name)
    glb, sheet = os.path.join(out, f"{name}.glb"), os.path.join(out, "sheet.json")
    try:
        key = (os.path.getmtime(glb), os.path.getmtime(sheet))
    except OSError:
        key = None
    if key and _LENS_CACHE.get(name, (None,))[0] == key:
        return _LENS_CACHE[name][1]
    res = None
    try:
        from lib import camsim
        L = camsim.lens_from_assembly(camsim.load_scene(glb, _read_json(sheet) or {}))
        res = dict(eye=[float(v) for v in L["lens"]], axis=[float(v) for v in L["axis"]],
                   right=[float(v) for v in L["long_axis"]] if L.get("long_axis") is not None else None,
                   source=f"lib.camsim.lens_from_assembly on {glb}")
    except Exception as e:  # noqa: BLE001  (no camera part, no trimesh): use the last simulation's numbers
        cj = _read_json(os.path.join(out, "camsim", "camsim.json"))
        c = (cj or {}).get("camera") or {}
        if c.get("lens_xyz_mm") and c.get("axis_unit"):
            res = dict(eye=c["lens_xyz_mm"], axis=c["axis_unit"], right=c.get("image_right_unit"),
                       source=f"out/{name}/camsim/camsim.json ({e})")
    if key and res:
        _LENS_CACHE[name] = (key, res)
    return res


def camera_view(name):
    """View entry of kind "camera": the UC-261 fisheye exactly as lib/camsim.py projects it. The page renders a cube
    map at the lens point and maps every sensor pixel through the same equidistant model (r = f * theta, f from the
    194 deg horizontal field over the 2592 px sensor width, dark beyond the 206 deg image circle) with the same image
    frame (lib.camsim.Fisheye: right = sensor long axis from the board, up = right x axis). Nothing is hidden; only
    the observer's own body (v_camera_uc261) is left out of the lens render, as camsim leaves it out of the rays."""
    import math
    L = _lens(name)
    if not L:
        return None
    from lib import camsim
    spec = camsim.load_camera_spec(ROOT)
    W, H = spec["resolution_px"]
    fe = camsim.Fisheye(L["eye"], L["axis"], W, H, spec["fov_h_deg"], spec["fov_circle_deg"], right=L.get("right"))
    right, up, ax = (fe.R[k].tolist() for k in range(3))
    eye = [float(x) for x in L["eye"]]
    r3 = lambda v, n=6: [round(float(x), n) for x in v]
    return dict(
        name="camera", label="camera", kind="camera",
        caption="UC-261 rotor camera: the true LS-40180 fisheye from the lens point (nothing hidden)",
        note="UC-261 LS-40180, equidistant 194 deg H, sensor 4:3 crop",
        eye=r3(eye, 3), axis=r3(ax), up=r3(up), focal=[round(eye[i] + 100.0 * ax[i], 3) for i in range(3)],
        scale=100.0, hide=[],
        fisheye=dict(projection="equidistant", right=r3(right), up=r3(up), axis=r3(ax),
                     sensor_px=[int(W), int(H)], f_px_per_rad=round(fe.f, 6), theta_max_rad=round(fe.theta_max, 8),
                     fov_h_deg=spec["fov_h_deg"], circle_deg=spec["fov_circle_deg"],
                     observer=[camsim.CAMERA_PART], near_mm=0.5),
        source=L["source"])


def serve(name, port=7891, open_browser=True, glb=None):
    """Foreground cadkit viewer. Returns (exit code, summary dict)."""
    glb = glb or os.path.join(ROOT, "out", name, f"{name}.glb")
    if not os.path.exists(glb):
        print(f"no {glb}; run bin/cad build {name} first", file=sys.stderr, flush=True)
        return 1, dict(ok=False, error=f"no {glb}")
    out = os.path.join(ROOT, "out", name)
    profile = _read_json(os.path.join(ROOT, "viewer.json")) or {}
    title = f"Froggy | {name}"
    if profile.get("assembly") == name:
        title = f"{profile['environment']} | Froggy {profile['revision']}"

    def status():
        result = build_status(out)
        if profile.get("assembly") == name:
            result.update(environment=profile["environment"], revision=profile["revision"])
        return result

    return CV.serve([glb], port=port, open_browser=open_browser, tree=os.path.join(out, "sheet.json"),
                    views=lambda: views(name), status_fn=status, name=name, title=title,
                    explode_origin="origin")
