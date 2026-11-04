"""out/<name>/sheet.json and topology.txt."""
import json
import os


def hexcol(c):
    return "#%02x%02x%02x" % tuple(int(round(v * 255)) for v in c)


def components(A):
    """sheet.json rows. `name` stays the internal key; `display_name`, `group` [module label,
    sub-assembly], `tree_path`, `node_name` (GLB/3MF node) and, for fasteners, `joint` (the joint
    group's display name) come from lib.naming, the same names the STEP tree uses."""
    from lib.naming import name_assembly
    rows = name_assembly(A)["parts"]
    out = []
    for n, p in A.parts.items():
        r = rows[n]
        b = A.bbox(n)
        c = dict(name=n, kind=p.kind, module=p.module, material=getattr(p, "material", ""), colour=hexcol(p.colour),
                 bbox_min_mm=[round(v, 4) for v in b[0]], bbox_max_mm=[round(v, 4) for v in b[1]],
                 volume_mm3=round(p.solid.volume, 3), faces=len(p.solid.faces()),
                 edges=len(p.solid.edges()))
        c.update(display_name=r["display_name"], group=list(r["group"]), tree_path=list(r["tree_path"]),
                 node_name=r["node_name"], naming_rule=r["rule"])
        if r.get("joint"):
            c.update(joint=r["joint"], joint_key=r["joint_key"])
        if p.meta.get("bodies"):
            c["sub_materials"] = dict(p.meta["bodies"])
            c["body_names"] = dict(r["bodies"])
        out.append(c)
    return out


def export_dxf(A, outdir):
    """Laser parts (components named lp_*): the largest planar face (the sheet outline with every
    hole, slot and notch) flattened to Z 0 and written to <outdir>/dxf/<part>.dxf for the cutter.
    Returns {part: path}."""
    from build123d import ExportDXF, Location, Unit
    from lib import vendor as V
    names = [n for n in A.parts if n.startswith("lp_")]
    if not names:
        return {}
    d = os.path.join(outdir, "dxf")
    os.makedirs(d, exist_ok=True)
    out = {}
    for n in names:
        faces = [(f, nrm, c) for f, nrm, c in V.planar_faces(A[n]) if abs(abs(nrm[2]) - 1) < 1e-6]
        if not faces:
            continue
        f, _, c = max(faces, key=lambda q: q[0].area)
        flat = f.moved(Location((0, 0, -float(c[2]))))
        ex = ExportDXF(unit=Unit.MM)
        ex.add_layer("cut")
        ex.add_shape(flat, layer="cut")
        path = os.path.join(d, f"{n}.dxf")
        ex.write(path)
        out[n] = path
    return out


def write(A, outdir, checks, phase_seconds, extra=None):
    os.makedirs(outdir, exist_ok=True)
    dxf = export_dxf(A, outdir)
    if dxf:
        extra = dict(extra or {}, dxf=dxf)
    comps = components(A)
    from lib.naming import name_assembly
    jn = name_assembly(A)["joints"]
    joints = [dict(r, display_name=jn[r["name"]]) if r.get("name") in jn else r for r in checks["joints"]]
    sheet = dict(name=A.name, components=comps, interference=checks["interference"], joints=joints,
                 clearances=checks["clearances"], vendor_parts=A.vendor, phase_seconds=phase_seconds,
                 valid=checks["valid"], fails=checks["fails"], notes=A.notes)
    for k in ("spin_test", "appearance_coverage", "modules"):
        if k in checks:
            sheet[k] = checks[k]
    if extra:
        sheet.update(extra)
    with open(os.path.join(outdir, "sheet.json"), "w") as fh:
        json.dump(sheet, fh, indent=1)
    __import__("lib.mass", fromlist=["report_stderr"]).report_stderr(sheet)   # human mass line (lib/mass.py)
    with open(os.path.join(outdir, "topology.txt"), "w") as fh:
        fh.write(f"assembly {A.name}: {len(comps)} components, {len(A.joints)} joints, valid={checks['valid']}\n\n")
        fh.write(f"{'name':44s} {'kind':8s} {'bbox_min':>26s} {'bbox_max':>26s} {'volume_mm3':>11s} {'faces':>5s} {'edges':>5s}\n")
        for c in comps:
            fh.write(f"{c['name']:44s} {c['kind']:8s} {str(tuple(round(v, 2) for v in c['bbox_min_mm'])):>26s} "
                     f"{str(tuple(round(v, 2) for v in c['bbox_max_mm'])):>26s} {c['volume_mm3']:11.2f} "
                     f"{c['faces']:5d} {c['edges']:5d}\n")
        i = checks["interference"]
        fh.write(f"\ninterference: {i['pairs']} hits over {i['checked_pairs']} pairs ({i['booleans_run']} booleans run)"
                 f"{' by class ' + str(i['by_class']) if 'by_class' in i else ''}\n")
        if "spin_test" in checks:
            st = checks["spin_test"]
            fh.write(f"spin test: ok={st['ok']} angles {st['angles']} candidate pairs {st['candidate_pairs']} "
                     f"booleans {st['booleans_run']} hits {len(st['hits'])}\n")
        if "modules" in checks:
            fh.write(f"modules: {checks['modules']['counts']}, appearance coverage {checks.get('appearance_coverage')}\n")
        fh.write("\njoints:\n")
        for r in checks["joints"]:
            flags = " ".join(k for k in ("fit_ok", "head_seated", "thread_engagement_ok", "nut_aligned",
                                         "no_interference", "contact_ok", "driver_path_clear") if r[k])
            fh.write(f"  {r['name']:26s} {r['screw']:6s} {r['nut']:15s} at {r['at']} axis {r['axis']} "
                     f"hole_d {r['hole_d_mm']} engage {r['engagement_mm']} ok={r['ok']} [{flags}]\n")
        fh.write("\nbracket clearances (mm):\n")
        for n, d in checks["clearances"].items():
            fh.write(f"  {n}: {d}\n")
    return sheet
