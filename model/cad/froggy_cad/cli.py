"""froggy-cad CLI: build, inspect, render, check, export, printcheck, animate, camsim, tree, design, review, intent,
view.

Every subcommand prints one JSON object on stdout, human lines on stderr, and
exits non-zero on failure. Artifacts go to out/<assembly>/.
"""
import argparse
import importlib
import json
import os
import pickle
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def log(*a):
    print(*a, file=sys.stderr, flush=True)


def json_stdout():
    """Keep stdout for the one JSON object: OCC and VTK print warnings straight to the C-level fd 1
    (e.g. "2 faces have been skipped due to null triangulation"), so fd 1 is pointed at stderr and
    Python's sys.stdout writes to a private copy of the real stdout."""
    try:
        sys.stdout.flush()
        real = os.dup(1)
        os.dup2(2, 1)
        sys.stdout = os.fdopen(real, "w", buffering=1)
    except OSError:
        pass


def outdir(name):
    return os.path.join(ROOT, "out", name)


def build_assembly(name):
    mod = importlib.import_module(f"assemblies.{name}")
    return mod.build()


def save(A, name):
    """Pickle the assembly; solids are re-wrapped bare (OCP colour objects do
    not pickle) and get their labels back on load."""
    import copy
    from build123d import Compound, Solid
    B = copy.copy(A)
    B.parts = {}
    for n, p in A.parts.items():
        q = copy.copy(p)
        q.solid = (Compound if isinstance(p.solid, Compound) else Solid)(p.solid.wrapped)
        B.parts[n] = q
    for k in ("_ivol_cache", "_meshes", "_dist_cache", "_extra_bbox"):
        B.__dict__.pop(k, None)
    with open(os.path.join(outdir(name), "assembly.pkl"), "wb") as fh:
        pickle.dump(B, fh, protocol=pickle.HIGHEST_PROTOCOL)


def load(name):
    """The built assembly from out/<name>/assembly.pkl (built if missing)."""
    p = os.path.join(outdir(name), "assembly.pkl")
    if os.path.exists(p):
        with open(p, "rb") as fh:
            A = pickle.load(fh)
        for n, part in A.parts.items():
            part.solid.label = n
        return A, "loaded"
    os.makedirs(outdir(name), exist_ok=True)
    A = build_assembly(name)
    save(A, name)
    return A, "built"


def read_sheet(name):
    p = os.path.join(outdir(name), "sheet.json")
    return json.load(open(p)) if os.path.exists(p) else {}


def export(A, name):
    """STEP from the B-rep; STL, 3MF and GLB from the one shared tessellation."""
    from lib import mesh
    return mesh.export_all(A, outdir(name), name)


def render(A, name, engine="pyvista", glb=None):
    from lib import render as RN
    if engine == "blender":
        glb = glb or os.path.join(outdir(name), f"{name}.glb")
        if not os.path.exists(glb):
            export(A, name)
        return RN.render_blender(glb, outdir(name))
    return RN.render_pyvista(A, outdir(name))


def run_checks(A):
    from lib import checks
    return checks.run_all(A)


def report_fails(res):
    for line in res["fails"]:
        log(line)


def cmd_build(a):
    from lib import sheet
    t0 = time.time()
    os.makedirs(outdir(a.name), exist_ok=True)
    phases = {}
    t = time.time()
    A = build_assembly(a.name)
    phases["build"] = round(time.time() - t, 2)
    log(f"built {a.name}: {len(A.parts)} components, {len(A.joints)} joints in {phases['build']} s")
    t = time.time()
    res = run_checks(A)
    phases["check"] = round(time.time() - t, 2)
    log(f"checks: {res['interference']['pairs']} interference hits over {res['interference']['checked_pairs']} pairs "
        f"({res['interference']['booleans_run']} booleans), {sum(r['ok'] for r in res['joints'])}/{len(res['joints'])} "
        f"joints ok in {phases['check']} s")
    report_fails(res)
    t = time.time()
    files = export(A, a.name)
    phases["export"] = round(time.time() - t, 2)
    log(f"exported step/stl/3mf/glb in {phases['export']} s")
    t = time.time()
    pngs = render(A, a.name, a.engine)
    phases["render"] = round(time.time() - t, 2)
    log(f"rendered {', '.join(k for k in pngs if not k.startswith('section_') or k == 'section_joint')} in {phases['render']} s")
    from lib.hardware import DEVIATIONS, screw_source
    A.notes = [f"screw source: {screw_source()}"] + DEVIATIONS + [n for n in A.notes if not n.startswith("screw source")]
    save(A, a.name)
    prev_rows = [dict(name=c["name"], display_name=c.get("display_name"), group=c.get("group"), kind=c.get("kind"))
                 for c in read_sheet(a.name).get("components", [])] or None
    sh = sheet.write(A, outdir(a.name), res, phases, extra=dict(renders=pngs, exports=files))
    t = time.time()
    design = run_design(sh, a.name, prev_rows)
    phases["design"] = round(time.time() - t, 2)
    log(f"design: {design.get('violation_count')} violations {design.get('summary', {})} in {phases['design']} s "
        f"(reported, never fails the build; bin/cad design {a.name})")
    total = round(time.time() - t0, 2)
    ok = res["valid"]
    from froggy_cad import outcache
    meta = outcache.save(a.name, build_ok=ok)
    if meta:
        log(f"cached {a.name}.glb + sheet.json ({meta['glb_bytes']} bytes) in {outcache.cache_dir(a.name)}")
    print(json.dumps(dict(command="build", assembly=a.name, ok=ok, components=len(A.parts), joints=len(A.joints),
                          interference_pairs=res["interference"]["pairs"],
                          checked_pairs=res["interference"]["checked_pairs"], phase_seconds=phases,
                          total_seconds=total, sheet=os.path.join(outdir(a.name), "sheet.json"),
                          design_violations=design.get("violation_count"),
                          design_by_severity=design.get("by_severity"), design_summary=design.get("summary"),
                          design_json=os.path.join(outdir(a.name), "design.json"),
                          renders=pngs, fails=res["fails"])))
    log(("OK" if ok else "FAIL") + f" build {a.name} in {total} s")
    return 0 if ok else 1


def run_design(sh, name, previous=None):
    """lib/designcheck on a sheet dict; writes out/<name>/design.json; never raises."""
    from lib import designcheck
    p = os.path.join(outdir(name), "sheet.json")
    try:
        d = designcheck.check(sh, p, previous=previous)
    except Exception as e:  # noqa: BLE001
        d = dict(command="design", ok=False, error=f"{type(e).__name__}: {e}", violations=[], violation_count=None)
    with open(os.path.join(outdir(name), "design.json"), "w") as fh:
        json.dump(d, fh, indent=1)
    return d


def cmd_design(a):
    """Design-language check of out/<name>/sheet.json. Previous parts (for DL26 support removal): --previous,
    else the newest archived review packet before HEAD. One JSON object, exit 0 even with violations."""
    from lib import review as RVW
    sh = read_sheet(a.name)
    if not sh:
        print(json.dumps(dict(command="design", assembly=a.name, ok=False, error=f"no out/{a.name}/sheet.json; "
                              f"run bin/cad build {a.name}", violations=[])))
        return 1
    prev, prev_from = a.previous, a.previous
    if not prev:
        key, sha, _ = RVW.packet_key(ROOT)
        pd = RVW.find_previous(ROOT, key, sha, a.name)
        if pd and os.path.exists(os.path.join(pd, "parts.json")):
            prev, prev_from = os.path.join(pd, "parts.json"), pd
    d = run_design(sh, a.name, prev)
    d.update(assembly=a.name, previous_from=prev_from, design_json=os.path.join(outdir(a.name), "design.json"))
    for v in d.get("violations", []):
        log(f"{v['rule']:5s} [{v['severity']}] {v['component']}: {v['detail'][:160]}")
    print(json.dumps(d))
    log(f"design {a.name}: {d.get('violation_count')} violations {d.get('by_severity', {})}")
    return 0 if d.get("ok", True) else 1


def cmd_review(a):
    """Review packet in out/<name>/review/ (lib/review.py), archived to out/reviews/<key>/<name>/."""
    from lib import review as RVW
    if a.pkl:
        with open(a.pkl, "rb") as fh:
            A = pickle.load(fh)
        for n, part in A.parts.items():
            part.solid.label = n
    else:
        A, _ = load(a.name)
    sh = json.load(open(a.sheet)) if a.sheet else read_sheet(a.name)
    if not sh:
        print(json.dumps(dict(command="review", assembly=a.name, ok=False, error="no sheet.json; build first")))
        return 1
    step = a.step or os.path.join(outdir(a.name), f"{a.name}.step")
    out_root = os.path.dirname(a.sheet) if a.sheet else outdir(a.name)
    cam = None
    if not a.pkl and not a.no_camsim:
        cj = os.path.join(outdir(a.name), "camsim", "camsim.json")
        pk = os.path.join(outdir(a.name), "assembly.pkl")
        if os.path.exists(cj) and os.path.getmtime(cj) >= os.path.getmtime(pk):
            cam = json.load(open(cj))
            log(f"camsim: reusing {cj} (newer than the build)")
        else:
            from lib import camsim
            log("camsim: running (the build is newer than camsim.json)")
            cam = camsim.run(ROOT, a.name, log=log)
    res = RVW.run(ROOT, a.name, A, sh, step, out_root, key=a.key, against=a.against, camsim_res=cam,
                  archive_only=a.archive_only, log=log)
    views_ok = all(os.path.exists(os.path.join(res["packet"], f"{v}.png")) for v in res["views"])
    ok = views_ok and "REVIEW.md" in res["files"] and len(res["views"]) >= 7
    res.update(command="review", ok=ok)
    print(json.dumps(res))
    log(("OK" if ok else "FAIL") + f" review {a.name} {res['key']} vs {res['previous'] or '(none)'}: "
        f"{len(res['views'])} views, design {res['design_violations']} (was {res['design_previous']}), "
        f"{res['assumed_values']} assumed values, packet {res['packet']}")
    return 0 if ok else 1


def cmd_intent(argv):
    """bin/cad intent add|list|resolve|md [--md]: Sawyer's verbatim verdicts (lib/intent.py)."""
    from lib import intent as IN
    ap = argparse.ArgumentParser(prog="cad intent", description="Sawyer's verdicts: docs/intent/intent.json, "
                                 "gallery docs/intent/REJECTED.md")
    ap.add_argument("action", nargs="?", choices=("add", "list", "resolve", "md"), default=None)
    ap.add_argument("id", nargs="?", help="resolve: intent id, e.g. I004")
    ap.add_argument("--verbatim", help="add: his exact words")
    ap.add_argument("--subject", default="", help="add: what it is about")
    ap.add_argument("--commit", help="add: commit he was looking at; resolve: commit that addressed it")
    ap.add_argument("--image", help="add: screenshot path (copied into docs/intent/images/)")
    ap.add_argument("--status", choices=IN.STATUSES, help="add (default open) / resolve (default addressed); "
                                                          "list: filter")
    ap.add_argument("--date", help="add: YYYY-MM-DD (default today)")
    ap.add_argument("--rule", default="", help="add: comma-separated DL ids")
    ap.add_argument("--note", help="add / resolve: one line of context")
    ap.add_argument("--addressed-in", help="add: commit that already addressed it")
    ap.add_argument("--md", action="store_true", help="regenerate docs/intent/REJECTED.md")
    a = ap.parse_args(argv)
    out = dict(command="intent", action=a.action or "md", ok=True)
    if a.action == "add":
        if not a.verbatim:
            ap.error("add needs --verbatim")
        e = IN.add(ROOT, a.verbatim, a.subject, a.commit, a.image, a.status or "open", a.date,
                   [r for r in a.rule.split(",") if r], a.note or "", a.addressed_in)
        out["intent"] = e
        log(f"added {e['id']} ({e['status']}): {e['verbatim']}")
    elif a.action == "resolve":
        if not a.id:
            ap.error("resolve needs an id")
        e = IN.resolve(ROOT, a.id, a.commit, a.status or "addressed", a.note)
        out["intent"] = e
        log(f"{e['id']} -> {e['status']} ({e.get('addressed_in')})")
    elif a.action == "list":
        es = [e for e in IN.load(ROOT)["intents"] if not a.status or e["status"] == a.status]
        for e in es:
            log(f"{e['id']} {e['date']} {e['status']:13s} {e.get('commit') or '-':8s} {e['verbatim']}")
        out.update(count=len(es), intents=es)
    if a.action in ("add", "resolve", "md") or a.md or a.action is None:
        out["markdown"] = IN.markdown(ROOT)
        log(f"wrote {out['markdown']}")
    print(json.dumps(out))
    return 0


def cmd_check(a):
    """Rebuild from source (a few seconds) and rerun every check: OCC booleans
    on unpickled shapes ran 3 to 4 x slower than on freshly built ones."""
    from lib import sheet
    t = time.time()
    if a.pickle:
        A, how = load(a.name)
    else:
        A, how = build_assembly(a.name), "rebuilt"
    log(f"{how} {a.name} in {time.time() - t:.2f} s")
    t = time.time()
    res = run_checks(A)
    dt = round(time.time() - t, 2)
    report_fails(res)
    prev = read_sheet(a.name)
    phases = dict(prev.get("phase_seconds", {}), check=dt)
    sheet.write(A, outdir(a.name), res, phases,
                extra={k: prev[k] for k in ("renders", "exports") if k in prev})
    ok = res["valid"]
    print(json.dumps(dict(command="check", assembly=a.name, ok=ok, source=how,
                          interference=dict((k, res["interference"][k]) for k in ("pairs", "checked_pairs", "booleans_run")),
                          joints_ok=sum(r["ok"] for r in res["joints"]), joints=len(res["joints"]),
                          fails=res["fails"], seconds=dt)))
    log(("OK" if ok else "FAIL") + f" check {a.name}: {len(res['fails'])} failures")
    return 0 if ok else 1


def cmd_render(a):
    A, how = load(a.name)
    t = time.time()
    pngs = render(A, a.name, a.engine)
    dt = round(time.time() - t, 2)
    print(json.dumps(dict(command="render", assembly=a.name, ok=True, engine=a.engine, renders=pngs, seconds=dt)))
    log(f"OK render {a.name} ({a.engine}) in {dt} s")
    return 0


def cmd_export(a):
    A, how = load(a.name)
    t = time.time()
    files = export(A, a.name)
    dt = round(time.time() - t, 2)
    ok = all(f["bytes"] > 1000 for f in files.values())
    print(json.dumps(dict(command="export", assembly=a.name, ok=ok, files=files, seconds=dt)))
    log(("OK" if ok else "FAIL") + f" export {a.name} in {dt} s")
    return 0 if ok else 1


def cmd_printcheck(a):
    from lib import printcheck
    res = printcheck.run(a.name)
    for line in res["lines"] + res["fails"]:
        log(line)
    log(("OK" if res["ok"] else "FAIL") + f" printcheck {a.name} in {res['seconds']} s")
    print(json.dumps(res))
    return 0 if res["ok"] else 1


def cmd_animate(a):
    from lib import animate as AN
    if a.pkl:
        with open(a.pkl, "rb") as fh:
            A = pickle.load(fh)
        for n, part in A.parts.items():
            part.solid.label = n
        how = a.pkl
    else:
        A, how = load(a.name)
    dst = a.out or outdir(a.name)
    res = AN.animate(A, dst, frames=a.frames, fps=a.fps, loops=a.loops, px=a.px, gif_w=a.gif_width, bg=a.bg,
                     shadows=a.shadows, elev=a.elev, vis_px=a.vis_px, log=log)
    ok = res["gif_bytes"] > 1000 and res.get("video_bytes", 0) > 1000 and os.path.exists(res["sheet"])
    print(json.dumps(dict(command="animate", assembly=a.name, ok=ok, source=how, **res)))
    log(("OK" if ok else "FAIL") + f" animate {a.name}: {res['gif']} ({res['gif_bytes']} bytes), {res.get('video')} "
        f"in {res['total_seconds']} s")
    return 0 if ok else 1


def cmd_camsim(a):
    from lib import camsim
    res = camsim.run(ROOT, a.name, log=log)
    for line in res["fails"]:
        log(line)
    log(("OK" if res["ok"] else "FAIL") + f" camsim {a.name}: {res['configs']} configs in {res['seconds']} s, "
        f"{res['camsim_json']}, montage {res['montage']}")
    print(json.dumps(res))
    return 0 if res["ok"] else 1


def cmd_inspect(a):
    sh = read_sheet(a.name)
    if not sh:
        from lib import sheet
        A, how = load(a.name)
        comps = sheet.components(A)
        sh = dict(components=comps)
    comps = sh.get("components", [])
    if a.component:
        comps = [c for c in comps if a.component in c["name"]]
    kinds = {}
    for c in comps:
        kinds[c.get("kind", "?")] = kinds.get(c.get("kind", "?"), 0) + 1
    xs = [c["bbox_min_mm"][0] for c in comps] + [c["bbox_max_mm"][0] for c in comps]
    ys = [c["bbox_min_mm"][1] for c in comps] + [c["bbox_max_mm"][1] for c in comps]
    zs = [c["bbox_min_mm"][2] for c in comps] + [c["bbox_max_mm"][2] for c in comps]
    ext = [round(max(v) - min(v), 3) for v in (xs, ys, zs)] if comps else None
    for c in comps:
        log(f"{c['name']:44s} {c.get('kind', ''):8s} min {c['bbox_min_mm']} max {c['bbox_max_mm']} vol {c['volume_mm3']}")
    out = dict(command="inspect", assembly=a.name, ok=bool(comps), count=len(comps), kinds=kinds, extent_mm=ext,
               valid=sh.get("valid"), interference=sh.get("interference", {}).get("pairs"),
               joints_ok=sum(1 for r in sh.get("joints", []) if r.get("ok")), joints=len(sh.get("joints", [])),
               components=comps if a.component else [c["name"] for c in comps])
    print(json.dumps(out))
    return 0 if comps else 1


def cmd_tree(a):
    """Label tree of out/<name>/<name>.step read back with OCP: human tree on stderr, rule check as
    JSON on stdout (ok, components, unnamed, misplaced). Exit 1 on any violation."""
    from lib import steptree
    t0 = time.time()
    step = os.path.join(outdir(a.name), f"{a.name}.step")
    if not os.path.exists(step):
        print(json.dumps(dict(command="tree", assembly=a.name, ok=False, error=f"no {step}; run bin/cad build {a.name}")))
        return 1
    tree = steptree.read_tree(step)
    sh = read_sheet(a.name)
    for line in steptree.render(tree, []):
        log(line)
    res = steptree.check(tree, sh)
    dry = steptree.fusion_dry_run(tree, sh)
    for u in res["unnamed"]:
        log(f"UNNAMED {u['name']!r} at {u['at']}: {', '.join(u['why'])}")
    for m in res["misplaced"]:
        log(f"MISPLACED {m['name']}: {m['why']}")
    summary = [dict(module=m, subassemblies=[dict(name=s_, parts=p_, joint_groups=j_) for s_, p_, j_ in subs])
               for m, subs in steptree.module_summary(tree)]
    out = dict(command="tree", assembly=a.name, root=tree["name"], **res, summary=summary,
               fusion_dry_run=dict(ok=dry["ok"], leaves=dry["leaves"], unmapped=dry["unmapped"][:20],
                                   parts_missing=dry["parts_missing"][:20],
                                   material_mismatch=dry["material_mismatch"][:20]),
               seconds=round(time.time() - t0, 2))
    out["ok"] = bool(res["ok"])
    print(json.dumps(out))
    log(("OK" if res["ok"] else "FAIL") + f" tree {a.name}: {res['components']} components, {res['joint_groups']} joint "
        f"groups, {len(res['unnamed'])} unnamed, {len(res['misplaced'])} misplaced; fusion dry run "
        f"{'ok' if dry['ok'] else 'FAIL'} ({dry['leaves']} leaves, {len(dry['unmapped'])} unmapped, "
        f"{len(dry['parts_missing'])} missing)")
    return 0 if res["ok"] else 1


def cmd_view(a):
    """Foreground live viewer: the standard cadkit viewer (`cadkit view`) with froggy's tree, review cameras and
    build status (froggy_cad/viewer.py). Human lines on stderr while it runs, the one JSON object on
    stdout at exit. Exit 2 when the port is taken.

    The GLB is resolved first (froggy_cad/outcache.py): a fresh out/<name>/<name>.glb, else a fresh cached copy
    from ~/cad/froggy-cad-cache/<name>/ restored into out/, else a full build in-process. --no-build serves the
    newest stale GLB instead; --resolve-only prints the resolution as JSON and exits."""
    from froggy_cad import outcache, viewer
    if a.glb:
        code, res = viewer.serve(a.name, port=a.port, open_browser=not a.no_open, glb=a.glb)
        print(json.dumps(dict(command="view", assembly=a.name, port=a.port, **res)))
        return code
    r = outcache.resolve(a.name)
    first = r["candidates"][0] if r["candidates"] else None
    if a.resolve_only:
        fresh = bool(first and first[2] >= r["newest"])
        print(json.dumps(dict(command="view", assembly=a.name, source=first[0] if first else "none",
                              glb=first[1] if first else None, fresh=fresh, would_build=not fresh,
                              source_newest_mtime=r["newest"], glb_mtime=first[2] if first else None,
                              cache_meta=r["meta"])))
        return 0
    if r["fresh"]:
        source, path, _ = r["fresh"][0]
        if source == "cache":
            path = outcache.restore(a.name)
            log(f"restored {a.name}.glb + sheet.json from cache {outcache.cache_dir(a.name)} "
                f"(built {(r['meta'] or {}).get('built_at')}, sha {(r['meta'] or {}).get('sha')})")
    elif a.no_build:
        if not first:
            return _view_serve(a, None, "none")
        source, path, _ = first
        if source == "cache":
            path = outcache.restore(a.name)
            log(f"restored {a.name}.glb from cache {outcache.cache_dir(a.name)}")
        log(f"WARNING: serving a stale {source} GLB ({path}): sources changed since it was built; "
            f"run bin/cad build {a.name} to refresh")
    else:
        log(f"no fresh GLB for {a.name} in out/ or the cache; building it now (bin/cad build {a.name})")
        a.engine = getattr(a, "engine", None) or "pyvista"
        real = sys.stdout
        sys.stdout = sys.stderr           # the build's JSON goes to stderr; stdout keeps view's one object
        try:
            bcode = cmd_build(a)
        finally:
            sys.stdout = real
        if bcode != 0:
            log(f"build {a.name} exited {bcode}; serving what it exported")
        source, path = "built", os.path.join(outdir(a.name), f"{a.name}.glb")
    return _view_serve(a, path, source)


def _view_serve(a, path, source):
    from froggy_cad import viewer
    code, res = viewer.serve(a.name, port=a.port, open_browser=not a.no_open, glb=path)
    print(json.dumps(dict(command="view", assembly=a.name, port=a.port, source=source, **res)))
    return code


def main(argv=None):
    ap = argparse.ArgumentParser(prog="cad", description="Headless build123d CAD for the Froggy Turntable")
    sub = ap.add_subparsers(dest="cmd", required=True)
    specs = {
        "build": (cmd_build, "build an assembly, run checks, export and render everything into out/<name>/"),
        "inspect": (cmd_inspect, "print components with bbox, volume, faces and edges"),
        "render": (cmd_render, "render iso, top, front, right, section and section_joint PNGs off-screen"),
        "check": (cmd_check, "rerun interference, joint and validity checks on the built assembly"),
        "export": (cmd_export, "export STEP, STL, 3MF and GLB"),
        "printcheck": (cmd_printcheck, "build one printed part alone, export its STL + 3MF print files and check "
                                       "watertight, 3 mm walls, 45 deg overhangs, P2S bed fit and a planar print face"),
        "animate": (cmd_animate, "render the rotor spinning about Z (rest fixed): <out>/drum_spin.gif (looping), "
                                 "drum_spin.mp4 (H.264) and a 0/90/180/270 deg contact sheet drum_spin_sheet.png"),
        "tree": (cmd_tree, "print the exported STEP's label tree (module > sub-assembly > parts, Fasteners > joint "
                           "groups) read back with OCP, check every name is readable and every part is placed; "
                           "also dry-runs cad-fusion's occurrence-to-sheet matching"),
        "design": (cmd_design, "check out/<name>/sheet.json against docs/DESIGN-LANGUAGE.md (lib/designcheck.py): "
                               "JSON violations list with rule and severity, out/<name>/design.json; exit 0 with "
                               "violations"),
        "review": (cmd_review, "write the review packet out/<name>/review/ (fixed views, before/after montage, design "
                               "JSON, tree, ASSUMPTIONS.md, REVIEW.md) and archive it to out/reviews/<key>/"),
        "camsim": (cmd_camsim, "ray-cast what the rotor camera (UC-261 fisheye) sees in the built assembly, as built "
                               "and over a tilt / height sweep: out/<name>/camsim/camsim.json, per-config view and "
                               "part-label PNGs, montage.png"),
        "view": (cmd_view, "serve out/<name>/<name>.glb in the cadkit viewer on 127.0.0.1:7891 (tree, review "
                           "cameras, clip, explode, undo/redo) that reloads itself when the GLB changes; "
                           "foreground, Ctrl-C stops it; a missing or stale GLB is restored from "
                           "~/cad/froggy-cad-cache/<name>/ or rebuilt first (--no-build, --resolve-only)"),
    }
    for k, (fn, h) in specs.items():
        p = sub.add_parser(k, help=h, description=h)
        if k == "printcheck":
            p.add_argument("name", help="part name (a module in parts/), e.g. fp5_motor_tower")
        else:
            p.add_argument("name", help="assembly name (a module in assemblies/), e.g. frame")
        if k == "check":
            p.add_argument("--pickle", action="store_true", help="check out/<name>/assembly.pkl instead of rebuilding")
        if k in ("build", "render"):
            p.add_argument("--engine", choices=("pyvista", "blender"), default="pyvista",
                           help="renderer: pyvista off-screen (default) or blender -b workbench (iso only)")
        if k == "design":
            p.add_argument("--previous", help="older sheet.json or review parts.json to compare support parts against")
        if k == "review":
            p.add_argument("--no-camsim", action="store_true", help="do not run camsim when camsim.json is stale")
            p.add_argument("--against", help="archived packet key to compare with (default: newest before HEAD)")
            p.add_argument("--key", help="archive key (default: short sha, -wip when geometry is uncommitted)")
            p.add_argument("--pkl", help="pickled assembly to review instead of out/<name>/assembly.pkl")
            p.add_argument("--sheet", help="sheet.json to use instead of out/<name>/sheet.json (packet goes "
                                           "next to it)")
            p.add_argument("--step", help="STEP to read the tree from (default out/<name>/<name>.step)")
            p.add_argument("--archive-only", action="store_true", help="write and archive the packet, no montage")
        if k == "view":
            p.add_argument("--port", type=int, default=7891, help="port on 127.0.0.1 (default 7891; head-view owns 7890)")
            p.add_argument("--no-open", action="store_true", help="do not open the browser")
            p.add_argument("--glb", help=argparse.SUPPRESS)
            p.add_argument("--no-build", action="store_true", help="never build: serve the newest stale GLB (out/, "
                                                                 "then the cache) with a warning if nothing is fresh")
            p.add_argument("--resolve-only", action="store_true", help="print which GLB view would serve (source, "
                                                                     "fresh, would_build) as JSON and exit; no "
                                                                     "build, restore or serve")
        if k == "inspect":
            p.add_argument("--component", help="only components whose name contains this text")
        if k == "animate":
            p.add_argument("--frames", type=int, default=72, help="frames per revolution (default 72, 5 deg steps)")
            p.add_argument("--fps", type=int, default=24, help="frames per second (default 24)")
            p.add_argument("--loops", type=int, default=2, help="revolutions in the MP4 (default 2; the GIF is one looping revolution)")
            p.add_argument("--px", type=int, default=1600, help="rendered frame and MP4 size in pixels (default 1600)")
            p.add_argument("--gif-width", type=int, default=1200, help="GIF width in pixels (default 1200)")
            p.add_argument("--elev", type=float, default=24.0, help="camera elevation above horizontal, deg (default 24)")
            p.add_argument("--vis-px", type=int, default=1000, help="frame size for the disc-frog visibility count (default 1000)")
            p.add_argument("--bg", choices=("warm_grey", "gradient"), default="gradient",
                           help="background: slate-to-light-grey vertical gradient (default) or plain light warm grey")
            p.add_argument("--no-shadows", dest="shadows", action="store_false", help="turn shadow mapping off")
            p.add_argument("--pkl", help="pickled assembly to read instead of out/<name>/assembly.pkl")
            p.add_argument("--out", help="output directory instead of out/<name>/")
        p.set_defaults(fn=fn)
    sub.add_parser("intent", help="Sawyer's verbatim verdicts: intent add|list|resolve|md [--md] "
                                  "(docs/intent/intent.json, gallery docs/intent/REJECTED.md)", add_help=False)
    argv = list(sys.argv[1:] if argv is None else argv)
    json_stdout()
    if argv and argv[0] == "intent":
        try:
            return cmd_intent(argv[1:])
        except Exception as e:  # noqa: BLE001
            import traceback
            traceback.print_exc(file=sys.stderr)
            print(json.dumps(dict(command="intent", ok=False, error=repr(e))))
            return 2
    a = ap.parse_args(argv)
    try:
        return a.fn(a)
    except Exception as e:  # noqa: BLE001
        import traceback
        traceback.print_exc(file=sys.stderr)
        print(json.dumps(dict(command=a.cmd, ok=False, error=repr(e))))
        return 2


if __name__ == "__main__":
    sys.exit(main())
