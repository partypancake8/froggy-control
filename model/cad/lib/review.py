"""`bin/cad review <name>`: the review packet a fresh-context reviewer (and Sawyer) reads.

out/<name>/review/
  <view>.png            fixed cameras from lib/review_views.py (never fitted to the geometry)
  montage.png           before | after | changed pixels for every view, against the previous packet
  compare/<view>.png    the same pair at full size
  design.json           bin/cad design output (with DL26 support removal against the previous parts)
  tree.json, tree.txt   the exported STEP's label tree and its rule check (bin/cad tree)
  parts.json            name, display_name, group, kind, module per component (next review's "before")
  ASSUMPTIONS.md        every value the build assumed that no vendor source or ICD row states
  REVIEW.md             yes/no checklist of every DL rule and of Sawyer's open verdicts, max two rounds
  packet.json           key, commit, previous packet, per-view change, counts

Each packet is archived to out/reviews/<key>/ (key = short sha, "-wip" when assemblies/, parts/,
vendor/ or geometry lib files have uncommitted changes). The "before" is the newest archived packet on
HEAD's first-parent history: HEAD's clean packet (when this one is wip), then for each older commit its
"-wip" packet (the state that became the next commit), then its clean one.
"""
import json
import os
import re
import shutil
import subprocess
import time
from collections import Counter

from lib import review_views as RV

REVIEW_ONLY = ("lib/designcheck.py", "lib/review.py", "lib/review_views.py", "lib/intent.py", "lib/render.py",
               "lib/steptree.py", "lib/camsim.py", "lib/animate.py", "lib/printcheck.py")
GIT_SCOPE = ("assemblies", "parts", "vendor", "lib")

# which fixed views a reviewer should look at for each rule
RULE_VIEWS = {
    "DL1": "front, right, back, top, close_front_panel", "DL2": "close_front_panel, front, back",
    "DL3": "close_front_panel, close_drive_underside, iso", "DL4": "iso, close_drive_underside",
    "DL5": "iso, close_hub, close_drum_top", "DL6": "close_hub, close_drum_top, close_drive_underside",
    "DL7": "close_hub, close_drum_top, close_front_panel", "DL8": "top, close_hub, close_drum_top",
    "DL9": "top, close_hub, close_drum_top", "DL10": "close_front_panel, close_hub, close_drive_underside",
    "DL11": "front, back, close_drum_top, top", "DL12": "close_hub", "DL13": "close_drive_underside, close_hub",
    "DL14": "close_drum_top, close_hub", "DL15": "close_drive_underside", "DL16": "iso, close_drive_underside",
    "DL17": "tree.txt", "DL18": "design.json, parts added/removed below", "DL19": "iso, front, right, back",
    "DL20": "iso, close_hub, close_drum_top", "DL21": "front, back, right", "DL22": "front, right, close_hub",
    "DL23": "the brief", "DL24": "this file", "DL25": "docs/intent/intent.json",
    "DL26": "close_hub, close_drum_top, parts removed below", "DL27": "close_drum_top, iso",
    "DL28": "close_hub, close_drive_underside",
}


# ------------------------------------------------------------------ git and archive
def _git(root, *args):
    try:
        r = subprocess.run(["git", "-C", root, *args], capture_output=True, text=True, timeout=20)
        return r.stdout.strip() if r.returncode == 0 else ""
    except Exception:
        return ""


def packet_key(root):
    """(key, sha, dirty files): short sha of HEAD, '-wip' when geometry sources are uncommitted."""
    sha = _git(root, "rev-parse", "--short", "HEAD") or "nogit"
    lines = _git(root, "status", "--porcelain", "--untracked-files=all", "--", *GIT_SCOPE).splitlines()
    dirty = [l[3:] for l in lines if l[3:] not in REVIEW_ONLY and "__pycache__" not in l]
    return (sha + "-wip" if dirty else sha), sha, dirty


def reviews_dir(root):
    return os.path.join(root, "out", "reviews")


def find_previous(root, key, sha, name):
    """The newest archived packet before this one on HEAD's first-parent history, for this assembly."""
    base = reviews_dir(root)
    if not os.path.isdir(base):
        return None
    have = [d for d in os.listdir(base) if d != key and os.path.exists(os.path.join(base, d, name, "packet.json"))]

    def match(full, wip):
        for h in sorted(have):
            hb = h[:-4] if h.endswith("-wip") else h
            if h.endswith("-wip") == wip and full.startswith(hb):
                return h
        return None
    shas = _git(root, "rev-list", "--first-parent", "--max-count=400", "HEAD").splitlines()
    for i, full in enumerate(shas):
        order = ([False] if key.endswith("-wip") else []) if i == 0 else [True, False]
        for wip in order:
            h = match(full, wip)
            if h:
                return os.path.join(base, h, name)
    return None


def archive(packet_dir, root, key, name):
    dst = os.path.join(reviews_dir(root), key, name)
    if os.path.isdir(dst):
        shutil.rmtree(dst)
    os.makedirs(dst, exist_ok=True)
    for f in os.listdir(packet_dir):
        p = os.path.join(packet_dir, f)
        if os.path.isfile(p) and f != "montage.png":
            shutil.copy2(p, os.path.join(dst, f))
    return dst


# ------------------------------------------------------------------ rendering
def render_views(A, outdir, log=print):
    """One PNG per fixed view: per-part colour and opacity, dark feature edges, parallel projection."""
    from lib import mesh as MS
    from lib.render import _add, _pv, _pv_mesh
    pv = _pv()
    t0 = time.time()
    M = MS.meshes(A)
    FR = MS.face_rgb(A)
    pvm = {n: _pv_mesh(pv, v, f, FR.get(n)) for n, (v, f) in M.items()}
    edges = {}
    for n, m in pvm.items():
        e = m.extract_feature_edges(feature_angle=35, boundary_edges=True, feature_edges=True,
                                    manifold_edges=False, non_manifold_edges=False)
        if e.n_points:
            edges[n] = e
    log(f"  review meshes + edges: {time.time() - t0:.2f} s")
    out = {}
    for view in RV.VIEWS:
        t = time.time()
        pl = pv.Plotter(off_screen=True, window_size=RV.PX)
        pl.set_background("white")
        shown = []
        for n, m in pvm.items():
            p = A.parts[n]
            if RV.hidden(view, n, p.module):
                continue
            shown.append(n)
            _add(pl, m, p.colour[:3], opacity=p.meta.get("opacity", 1.0), smooth_shading=False,
                 specular=0.15, ambient=0.18)
        e = [edges[n] for n in shown if n in edges]
        if e:
            pl.add_mesh(pv.merge(e), color="#26292e", line_width=1.2)
        pl.camera_position = [view["eye"], view["focal"], view["up"]]
        pl.enable_parallel_projection()
        pl.camera.parallel_scale = view["scale"]
        pl.camera.clipping_range = (1.0, 8000.0)
        pl.add_text(view["caption"], position="upper_left", font_size=11, color="black")
        pl.add_text(f"{A.name}  view {view['name']}  scale {2 * view['scale']:.0f} mm tall", position="lower_left",
                    font_size=8, color="#555555")
        try:
            pl.enable_anti_aliasing("ssaa")
        except Exception:
            pass
        path = os.path.join(outdir, f"{view['name']}.png")
        pl.screenshot(path)
        pl.close()
        out[view["name"]] = path
        log(f"  review view {view['name']}: {time.time() - t:.2f} s ({len(shown)} parts)")
    return out


def _font(size):
    from PIL import ImageFont
    for f in ("/System/Library/Fonts/Helvetica.ttc", "/System/Library/Fonts/Supplemental/Arial.ttf"):
        try:
            return ImageFont.truetype(f, size)
        except Exception:
            pass
    return ImageFont.load_default()


def compare(prev_dir, cur_dir, out_png, compare_dir, prev_key, cur_key):
    """Before | after | changed (red on a faded after) per view; returns {view: changed percent or None}."""
    import numpy as np
    from PIL import Image, ImageDraw
    os.makedirs(compare_dir, exist_ok=True)
    tw, th = 460, 345
    head = 70
    rows = []
    changed = {}
    for view in RV.VIEWS:
        n = view["name"]
        cur = Image.open(os.path.join(cur_dir, f"{n}.png")).convert("RGB")
        pp = os.path.join(prev_dir, f"{n}.png") if prev_dir else None
        prev = Image.open(pp).convert("RGB") if pp and os.path.exists(pp) else None
        if prev is not None and prev.size == cur.size:
            a, b = np.asarray(prev, dtype=np.int16), np.asarray(cur, dtype=np.int16)
            mask = np.abs(a - b).max(axis=2) > 40
            changed[n] = round(100.0 * float(mask.mean()), 2)
            faded = (np.asarray(cur, dtype=np.float32) * 0.35 + 255 * 0.65).astype(np.uint8)
            faded[mask] = (220, 30, 30)
            diff = Image.fromarray(faded)
        else:
            changed[n] = None
            diff = Image.new("RGB", cur.size, "white")
            ImageDraw.Draw(diff).text((40, 40), "no previous packet for this view", fill="black", font=_font(40))
            if prev is None:
                prev = diff
        big = Image.new("RGB", (cur.size[0] * 2, cur.size[1]), "white")
        big.paste(prev.resize(cur.size), (0, 0))
        big.paste(cur, (cur.size[0], 0))
        d = ImageDraw.Draw(big)
        d.text((20, cur.size[1] - 50), f"BEFORE {prev_key or '(none)'}", fill="black", font=_font(30))
        d.text((cur.size[0] + 20, cur.size[1] - 50), f"AFTER {cur_key}", fill="black", font=_font(30))
        big.save(os.path.join(compare_dir, f"{n}.png"))
        rows.append((n, view["caption"], prev.resize((tw, th)), cur.resize((tw, th)), diff.resize((tw, th))))
    W = tw * 3 + 40
    H = head + len(rows) * (th + 40)
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)
    d.text((10, 10), f"before {prev_key or '(no previous packet)'}   after {cur_key}   (red = changed pixels)",
           fill="black", font=_font(22))
    for i, lab in enumerate(("BEFORE", "AFTER", "CHANGED")):
        d.text((10 + i * (tw + 10), 40), lab, fill="#333333", font=_font(20))
    y = head
    for n, cap, a, b, c in rows:
        pct = changed[n]
        d.text((10, y + 4), f"{n}: {cap}" + (f"   ({pct}% of pixels changed)" if pct is not None else ""),
               fill="black", font=_font(18))
        for i, im in enumerate((a, b, c)):
            img.paste(im, (10 + i * (tw + 10), y + 30))
        y += th + 40
    img.save(out_png)
    return changed


# ------------------------------------------------------------------ packet contents
def strip_n(dn):
    return re.sub(r"\s+\d+ of \d+", "", dn or "").strip()


def parts_rows(sheet):
    return [dict(name=c["name"], display_name=c.get("display_name", c["name"]), group=c.get("group", []),
                 kind=c.get("kind"), module=c.get("module")) for c in sheet.get("components", [])]


def parts_diff(prev_rows, cur_rows):
    """Added / removed by display name ('n of N' folded), fasteners counted separately."""
    fk = {"screw", "nut", "washer", "insert", "set_screw"}

    def count(rows, fast):
        return Counter(strip_n(r["display_name"]) for r in rows if (r.get("kind") in fk) == fast)
    out = {}
    for label, fast in (("parts", False), ("fasteners", True)):
        a, b = count(prev_rows or [], fast), count(cur_rows, fast)
        out[label] = dict(added=sorted(((k, b[k] - a[k]) for k in b if b[k] > a[k]), key=lambda kv: kv[0]),
                          removed=sorted(((k, a[k] - b[k]) for k in a if a[k] > b[k]), key=lambda kv: kv[0]),
                          before=sum(a.values()), after=sum(b.values()))
    return out


def assumptions(root, sheet):
    """Every value the build assumed: source.json 'assum' notes, sheet notes, ICD (ds) rows, no-source vendor parts."""
    lines = ["# Assumed values", "",
             "Every value the build used that no fetched vendor source or settled ICD row states. Each line is",
             "either confirmed (a source is fetched, or Sawyer says so) or stays listed. Brief rule: the worker",
             "lists any new assumption here before building, and the orchestrator confirms it with Sawyer.", ""]
    n = 0
    lines += ["## Vendor source.json notes", ""]
    for d, _, files in sorted(os.walk(os.path.join(root, "vendor"))):
        if "source.json" not in files:
            continue
        p = os.path.join(d, "source.json")
        try:
            js = json.load(open(p))
        except Exception:
            continue
        rel = os.path.relpath(p, root)

        def walk(o, path=""):
            nonlocal n
            if isinstance(o, dict):
                for k, v in o.items():
                    if "assum" in k.lower() and not isinstance(v, (dict, list)):
                        lines.append(f"- `{rel}` {path}{k}: {v}")
                        n += 1
                    else:
                        walk(v, f"{path}{k}.")
            elif isinstance(o, list):
                for i, v in enumerate(o):
                    walk(v, f"{path}{i}.")
            elif isinstance(o, str) and re.search(r"\bassum", o, re.I):
                key = path.rstrip(".")
                lines.append(f"- `{rel}` {key}: {o[:300]}")
                n += 1
        walk(js)
    lines += ["", "## Build notes (sheet.json notes that say assumed)", ""]
    for note in sheet.get("notes", []):
        head = re.split(r"[=:]", note, 1)[0][:40]
        for seg in re.split(r";\s+|\.\s+(?=[A-Z0-9])", note):
            if re.search(r"\bassum", seg, re.I):
                lines.append(f"- [{head.strip()}] {seg.strip()[:320]}")
                n += 1
    lines += ["", "## ICD rows waiting on a vendor sheet (ds)", ""]
    icd = os.path.join(root, "docs", "ICD.md")
    if os.path.exists(icd):
        for row in open(icd).read().splitlines():
            if row.startswith("| S") and re.search(r"\(ds\)|\bds\)|, ds\b", row):
                cells = [c.strip() for c in row.strip("|").split("|")]
                lines.append(f"- {cells[0]}: {cells[1][:160]} | {cells[3][:200] if len(cells) > 3 else ''}")
                n += 1
    lines += ["", "## Bought parts with no source_url (modelled from a class envelope)", ""]
    nosrc = Counter()
    for vp in sheet.get("vendor_parts", []):
        if not vp.get("source_url") and not vp.get("file"):
            nosrc[(vp.get("note") or vp.get("author") or "no note")] += 1
    for note, k in sorted(nosrc.items(), key=lambda kv: -kv[1]):
        lines.append(f"- {k} part(s): {note}")
        n += 1
    return "\n".join(lines) + "\n", n


def dl_rules(root):
    """[(id, title, fail text, auto)] in document order from docs/DESIGN-LANGUAGE.md."""
    p = os.path.join(root, "docs", "DESIGN-LANGUAGE.md")
    txt = open(p).read() if os.path.exists(p) else ""
    out = []
    for m in re.finditer(r"\*\*(DL\d+) ([^*]+?)\*\*(.*?)(?=\n\n|\Z)", txt, re.S):
        body = " ".join(m.group(3).split())
        f = re.search(r"Fail: (.*?)(?: \[auto[^\]]*\]| Source:|$)", body)
        out.append((m.group(1), m.group(2).strip().rstrip("."), f.group(1).strip() if f else body[:200],
                    "[auto" in body))
    return out


def intents(root):
    p = os.path.join(root, "docs", "intent", "intent.json")
    try:
        return json.load(open(p)).get("intents", [])
    except Exception:
        return []


def review_md(name, key, prev_key, root, design, prev_design, diff, changed, camsim, tree, sheet, n_assumed):
    L = [f"# Review: {name} at {key}", "",
         f"Before: {prev_key or '(no previous packet)'}. Packet written {time.strftime('%Y-%m-%d %H:%M')}.", "",
         "## Reviewer instructions", "",
         "You are a fresh-context reviewer. You did not build this. Read `docs/DESIGN-LANGUAGE.md` and",
         "`docs/intent/REJECTED.md` (Sawyer's verbatim verdicts with his screenshots) first, then answer every item",
         "below with yes or no and one line of evidence from the images or JSON in this folder. Judge from the fixed",
         "views (`<view>.png`), `montage.png` (before | after | changed) and `design.json`; never from the author's",
         "description. Any no goes back to the owner. At most two rounds: after round 2 the open items go to Sawyer",
         "as they stand, with their images. Do not suggest removing support around the rotor or drum (DL26) or",
         "Sawyer's rim rollers and spindle (DL28).", ""]
    # build health
    inter = sheet.get("interference", {})
    joints = sheet.get("joints", [])
    dcount = design.get("violation_count", len(design.get("violations", [])))
    pcount = prev_design.get("violation_count") if prev_design else None
    L += ["## Build health", "",
          f"- build valid: {sheet.get('valid')}; interference pairs {inter.get('pairs')} over {inter.get('checked_pairs')}; "
          f"joints ok {sum(1 for j in joints if j.get('ok'))}/{len(joints)}",
          f"- design violations: {dcount} ({design.get('by_severity', {})})"
          + (f", previous {pcount} (delta {dcount - pcount:+d})" if pcount is not None else ", no previous packet"),
          f"- tree: ok {tree.get('ok')}, {tree.get('components')} components, {len(tree.get('unnamed', []))} unnamed, "
          f"{len(tree.get('misplaced', []))} misplaced" if tree else "- tree: not read (no STEP)",
          f"- assumed values listed: {n_assumed} (ASSUMPTIONS.md)", ""]
    if camsim:
        rec = (camsim.get("recommendation") or "").split(". ")[0]
        L += ["## Camera (camsim)", "", f"- ok: {not camsim.get('fails')}; fails: {camsim.get('fails') or 'none'}",
              f"- {rec}.", f"- montage: {camsim.get('montage')}", ""]
    else:
        L += ["## Camera (camsim)", "", "- not run for this packet", ""]
    # changes
    L += ["## What changed", ""]
    for v, pct in changed.items():
        L.append(f"- {v}: " + ("no previous image" if pct is None else f"{pct}% of pixels changed"))
    L.append("")
    for label in ("parts", "fasteners"):
        d = diff[label]
        L.append(f"**{label.capitalize()}**: {d['before']} before, {d['after']} after.")
        L.append("- added: " + ("; ".join(f"{k} x{n}" for k, n in d["added"]) or "none"))
        L.append("- removed: " + ("; ".join(f"{k} x{n}" for k, n in d["removed"]) or "none"))
        L.append("")
    # DL checklist
    by_rule = {}
    for v in design.get("violations", []):
        by_rule.setdefault(v["rule"], []).append(v)
    L += ["## Design language checklist (yes = passes)", ""]
    for rid, title, fail, auto in dl_rules(root):
        vs = by_rule.get(rid, [])
        L.append(f"### {rid} {title}")
        L.append(f"Fails if: {fail}")
        if auto:
            L.append(f"Auto check: {len(vs)} violation(s)" + (":" if vs else "."))
            for x in vs[:6]:
                L.append(f"  - [{x['severity']}] {x['component']}: {x['detail'][:220]}")
            if len(vs) > 6:
                L.append(f"  - ... {len(vs) - 6} more in design.json")
        L.append(f"Look at: {RULE_VIEWS.get(rid, 'iso')}")
        L.append("- Round 1: [ ] yes  [ ] no   evidence:")
        L.append("- Round 2: [ ] yes  [ ] no   evidence:")
        L.append("")
    # intents
    its = intents(root)
    L += ["## Sawyer's verdicts (docs/intent/intent.json)", "",
          "For each open verdict and standing rule: yes = this build no longer draws the complaint / keeps the rule.", ""]
    for it in its:
        if it.get("status") not in ("open", "standing-rule"):
            continue
        img = f" image `{it['image']}`" if it.get("image") else ""
        L.append(f"### {it['id']} ({it['status']}) {it.get('subject', '')}")
        L.append(f"> \"{it['verbatim']}\" ({it.get('date')}, commit {it.get('commit') or 'n/a'}){img}")
        L.append("- Round 1: [ ] yes  [ ] no   evidence:")
        L.append("- Round 2: [ ] yes  [ ] no   evidence:")
        L.append("")
    done = [it for it in its if it.get("status") == "addressed"]
    if done:
        L += ["### Addressed verdicts: confirm none came back", ""]
        for it in done:
            L.append(f"- [ ] {it['id']} \"{it['verbatim']}\" (fixed in {it.get('addressed_in') or '?'})")
        L.append("")
    L += ["## Verdict", "", "- Round 1 result: [ ] all yes  [ ] back to owner (list the no items)",
          "- Round 2 result: [ ] all yes  [ ] to Sawyer with the open items", ""]
    return "\n".join(L)


def run(root, name, A, sheet, step, out_root, key=None, against=None, camsim_res=None, archive_only=False,
        log=print):
    """Write the packet for assembly A (sheet dict, STEP path) into <out_root>/review and archive it."""
    from lib import designcheck
    t0 = time.time()
    k, sha, dirty = packet_key(root)
    if key:  # an explicit key (e.g. a packet made from another checkout's build) names its own commit
        sha, dirty = key.split("-wip")[0], []
    key = key or k
    pdir = os.path.join(out_root, "review")
    if os.path.isdir(pdir):
        shutil.rmtree(pdir)
    os.makedirs(pdir)
    prev_dir = against and os.path.join(reviews_dir(root), against, name)
    if against and not os.path.isdir(prev_dir):
        raise RuntimeError(f"no archived packet {prev_dir}")
    if not against and not archive_only:
        prev_dir = find_previous(root, key, sha, name)
    prev_key = os.path.basename(os.path.dirname(prev_dir)) if prev_dir else None
    pngs = render_views(A, pdir, log=log)
    rows = parts_rows(sheet)
    json.dump(rows, open(os.path.join(pdir, "parts.json"), "w"), indent=0)
    prev_rows = json.load(open(os.path.join(prev_dir, "parts.json"))) if prev_dir else None
    sheet_path = os.path.join(out_root, "sheet.json")
    design = designcheck.check(sheet, sheet_path if os.path.exists(sheet_path) else None, previous=prev_rows)
    json.dump(design, open(os.path.join(pdir, "design.json"), "w"), indent=1)
    prev_design = None
    if prev_dir and os.path.exists(os.path.join(prev_dir, "design.json")):
        prev_design = json.load(open(os.path.join(prev_dir, "design.json")))
    tree = {}
    if step and os.path.exists(step):
        from lib import steptree
        tr = steptree.read_tree(step)
        tree = steptree.check(tr, sheet)
        open(os.path.join(pdir, "tree.txt"), "w").write("\n".join(steptree.render(tr, [])) + "\n")
        json.dump(dict(tree, summary=[dict(module=m, subassemblies=[dict(name=s, parts=p, joint_groups=j)
                                                                     for s, p, j in subs])
                                      for m, subs in steptree.module_summary(tr)]),
                  open(os.path.join(pdir, "tree.json"), "w"), indent=1, default=str)
    text, n_assumed = assumptions(root, sheet)
    open(os.path.join(pdir, "ASSUMPTIONS.md"), "w").write(text)
    if archive_only:
        changed = {v["name"]: None for v in RV.VIEWS}
    else:
        changed = compare(prev_dir, pdir, os.path.join(pdir, "montage.png"), os.path.join(pdir, "compare"),
                          prev_key, key)
    diff = parts_diff(prev_rows, rows)
    open(os.path.join(pdir, "REVIEW.md"), "w").write(
        review_md(name, key, prev_key, root, design, prev_design, diff, changed, camsim_res, tree, sheet, n_assumed))
    meta = dict(assembly=name, key=key, commit=sha, dirty_files=dirty[:50], written=time.strftime("%Y-%m-%dT%H:%M:%S"),
                previous=prev_key, previous_dir=prev_dir, views=list(pngs), changed_percent=changed,
                design_violations=design.get("violation_count"),
                design_previous=prev_design.get("violation_count") if prev_design else None,
                parts_added=diff["parts"]["added"], parts_removed=diff["parts"]["removed"],
                fasteners_added=diff["fasteners"]["added"], fasteners_removed=diff["fasteners"]["removed"],
                assumed_values=n_assumed, tree_ok=tree.get("ok") if tree else None,
                camsim_ok=(not camsim_res.get("fails")) if camsim_res else None)
    json.dump(meta, open(os.path.join(pdir, "packet.json"), "w"), indent=1)
    arch = archive(pdir, root, key, name)
    files = sorted(os.listdir(pdir))
    return dict(meta, packet=pdir, archive=arch, files=files, montage=os.path.join(pdir, "montage.png")
                if not archive_only else None, seconds=round(time.time() - t0, 2))
