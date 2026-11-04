#!/usr/bin/env python3
"""designcheck: check the measurable rules of docs/DESIGN-LANGUAGE.md against a froggy-cad sheet.json.

  bin/cad design <name>                    (reads out/<name>/sheet.json; also a phase of bin/cad build)
  python3 lib/designcheck.py <sheet.json> [--previous <older sheet.json or parts.json>]

Python 3 stdlib only. Limits come from lib/design_constants.py. Reads the sheet (and, when the sheet
sits at <repo>/out/<name>/sheet.json, reads <repo>/lib/render.py for stale render captions and asks git
for the commit). With a previous sheet it also reports support parts removed (DL26). Never writes.
Prints ONE JSON object:

  {"command": "design", "ok": true, "commit", "sheet", "checked": {...}, "summary": {rule: count},
   "by_severity": {"fail": n, "review": n}, "violation_count": n,
   "violations": [{rule, severity, component, detail}]}

severity "fail" breaks a rule outright; "review" is a prompt the fresh-context reviewer must answer
(part-count caps, printed parts on the rotor). Exit code is always 0: this reports, the orchestrator
decides, and `bin/cad build` never fails on it.
"""
import json
import math
import os
import re
import subprocess
import sys
from collections import Counter, defaultdict

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
from lib import design_constants as DC  # noqa: E402

# ---- stated limits (lib/design_constants.py, quoted from DESIGN-LANGUAGE.md) ----
TOL = DC.ENVELOPE["tol"]
ESTOP_HEAD_MAX_PROUD = DC.ENVELOPE["estop_head_max_proud"]
FOOT_MAX_BELOW = DC.ENVELOPE["foot_max_below"]
HEAD_MAX_PROUD = DC.HEAD_MAX_PROUD
RUN_BAND = DC.RUNNING_BAND
NET_CLEAR = DC.NET_CLEAR
CAP_PARTS = DC.CAPS["parts"]
CAP_FASTENERS = DC.CAPS["fasteners"]
HUB_RADIUS = DC.HUB_RADIUS
FASTENER_KINDS = {"screw", "nut", "washer", "insert", "set_screw"}
WIRE_KINDS = {"wire", "cable", "harness", "cable_tie", "clip", "gland", "duct", "strain_relief"}
INTERNAL_PREFIXES = DC.INTERNAL_PREFIXES
BAD_GROUPS = set(DC.BAD_GROUPS)
ACRYLIC_ALLOWED = set(DC.ACRYLIC_ALLOWED)
SUPPORT_SUBS = set(DC.SUPPORT_SUBASSEMBLIES)
SUPPORT_RE = re.compile(DC.SUPPORT_NAME_RE, re.I)
REVIEW_SEVERITY = {"DL18", "DL26"}  # caps are review prompts (DL18); a support removal needs his intent id (DL26)
PRINTED_FORBIDDEN_WORDS = re.compile(r"(ring|plate|disc|disk|hub|carrier_disc)", re.I)
REQUIRED_SUBSYSTEMS = {          # DL16: the whole machine, every build
    "motor": r"nema17|motor_?\d|stepper",
    "bearing": r"pillow_block|bearing",
    "carrier disc": r"carrier_disc",
    "drum tube": r"drum_tube",
    "top disc": r"top_disc",
    "camera": r"camera",
    "rotor computer": r"rpi|pi_zero",
    "IR illumination": r"ir_led|ring_light|led_ring",
    "hall sensor": r"hall",
    "hall magnets": r"magnet",
    "e-stop": r"estop",
    "motor driver": r"cl42t|driver",
    "power supply": r"psu|lrs_",
    "net cage": r"mesh|net",
    "feet": r"foot|feet",
    "wiring": None,              # checked by DL13 instead
}
RETIRED_TERMS = list(DC.RETIRED_TERMS)   # DL17: captions must describe the current build


def bbox(c):
    return c["bbox_min_mm"], c["bbox_max_mm"]


def xy_radius_range(c):
    """Min and max distance from the Z axis over the component's XY bbox rectangle."""
    (x0, y0, _), (x1, y1, _) = bbox(c)
    cx = 0.0 if x0 <= 0 <= x1 else min(abs(x0), abs(x1))
    cy = 0.0 if y0 <= 0 <= y1 else min(abs(y0), abs(y1))
    rmin = math.hypot(cx, cy)
    rmax = math.hypot(max(abs(x0), abs(x1)), max(abs(y0), abs(y1)))
    return rmin, rmax


def sub(c):
    g = c.get("group") or []
    return " / ".join(g[:2]) if g else "(no group)"


def is_fastener(c):
    return c.get("kind") in FASTENER_KINDS or c.get("naming_rule") == "fastener" and c.get("kind") != "standoff"


def rotating(c):
    g = c.get("group") or []
    if c.get("module") == "rotor":
        return True
    return len(g) > 1 and g[0] == "Electrical" and g[1] in ("Rotor electronics", "IR LEDs")


def union_box(cs):
    lo = [min(c["bbox_min_mm"][i] for c in cs) for i in range(3)]
    hi = [max(c["bbox_max_mm"][i] for c in cs) for i in range(3)]
    return lo, hi


def repo_of(sheet_path):
    p = os.path.abspath(sheet_path)
    parts = p.split(os.sep)
    if len(parts) >= 4 and parts[-3] == "out":
        return os.sep.join(parts[:-3])
    return None


def git_commit(repo):
    if not repo:
        return None
    try:
        out = subprocess.run(["git", "-C", repo, "log", "-1", "--format=%h %s"], capture_output=True,
                             text=True, timeout=10)
        dirty = subprocess.run(["git", "-C", repo, "status", "--porcelain", "--", "assemblies", "parts", "lib"],
                               capture_output=True, text=True, timeout=10)
        c = out.stdout.strip() or None
        if c and dirty.stdout.strip():
            c += " (working tree has uncommitted changes; the sheet may be newer or older than this commit)"
        return c
    except Exception:
        return None


def is_support(c):
    """A part that holds up or guides the rotor or drum (DL26): never a removal or count target."""
    if is_fastener(c) or c.get("module") == "cage":
        return False          # the cage guards the rotor, it does not hold it up (I030: its tube posts are not drum posts)
    g = c.get("group") or []
    return (len(g) > 1 and g[1] in SUPPORT_SUBS) or bool(SUPPORT_RE.search(c["name"]))


def previous_components(prev):
    """Components of an older sheet.json or of a review packet's parts.json (list of rows)."""
    if prev is None:
        return None
    if isinstance(prev, str):
        try:
            prev = json.load(open(prev))
        except Exception:
            return None
    if isinstance(prev, dict):
        prev = prev.get("components", prev.get("parts", []))
    return prev


def check(sheet, path=None, previous=None):
    """Run every [auto] rule on a sheet dict. `previous`: older sheet / parts.json path or rows."""
    comps = sheet.get("components", [])
    by_name = {c["name"]: c for c in comps}
    joints = sheet.get("joints", [])
    V = []
    path = path or "(sheet dict)"

    def v(rule, comp, detail, severity=None):
        V.append({"rule": rule, "severity": severity or ("review" if rule in REVIEW_SEVERITY else "fail"),
                  "component": comp, "detail": detail})

    checked = Counter()

    # ---------- DL1 envelope ----------
    panels = [c for c in comps if c["name"].startswith("s_panel_")]
    cage_rings = [c for c in comps if c["name"].startswith("lp_cage_ring")]
    # I030 (2026-10-21): the cage top is the top of the cage's own structure (rings in v9 before I030, the top mesh
    # panel after), never a fastener
    cage_struct = cage_rings or [c for c in comps if c.get("module") == "cage" and not is_fastener(c)]
    if panels:
        (px0, py0, pz0), (px1, py1, pz1) = union_box(panels)
        cage_top = union_box(cage_struct)[1][2] if cage_struct else pz1
        # one envelope: the plinth footprint in XY (panel outer faces), from the bottom panel underside
        # up to the top of the cage
        env = {"footprint_xy": [[px0, py0], [px1, py1]], "z": [pz0, cage_top], "deck_top_z": pz1}
        for c in comps:
            checked["DL1 envelope"] += 1
            (x0, y0, z0), (x1, y1, z1) = bbox(c)
            name = c["name"]
            over = {}
            ax0, ay0, ax1, ay1 = px0, py0, px1, py1
            if x0 < ax0 - TOL: over["-X"] = ax0 - x0
            if x1 > ax1 + TOL: over["+X"] = x1 - ax1
            if y0 < ay0 - TOL: over["-Y"] = ay0 - y0
            if y1 > ay1 + TOL: over["+Y"] = y1 - ay1
            if z0 < pz0 - TOL: over["-Z"] = pz0 - z0
            if z1 > cage_top + TOL: over["+Z"] = z1 - cage_top
            if not over:
                continue
            if "estop" in name and set(over) <= {"-Y"} and over["-Y"] <= ESTOP_HEAD_MAX_PROUD:
                continue
            if c.get("kind") == "foot" and set(over) <= {"-Z"} and over["-Z"] <= FOOT_MAX_BELOW:
                continue
            if is_fastener(c) and max(over.values()) <= HEAD_MAX_PROUD:
                continue  # a button head on a face (DL11 allows it)
            if is_fastener(c) and set(over) <= {"-Z"} and any(
                    f.get("kind") == "foot" and z0 >= bbox(f)[0][2] - TOL and
                    all(bbox(f)[0][k] - TOL <= (x0, y0)[k] and (x1, y1)[k] <= bbox(f)[1][k] + TOL for k in range(2))
                    for f in comps):
                continue  # a foot bolt head recessed in its bumper foot's counterbore (I029): only rubber below it
            if is_fastener(c) and set(over) == {"+Z"}:
                continue  # tails and nuts above the top ring are reported once, under DL11
            v("DL1", name, "outside the plinth/cage envelope by " +
              ", ".join(f"{k} {d:.1f} mm" for k, d in sorted(over.items())) + f" ({c.get('display_name', '')})")
    else:
        env = None
        v("DL1", "(sheet)", "no s_panel_* components: the plinth envelope cannot be derived")

    # ---------- DL3 bought parts: real models, sourced ----------
    vend = {p["component"]: p for p in sheet.get("vendor_parts", [])}
    for c in comps:
        if not c["name"].startswith("v_") or is_fastener(c):
            continue
        checked["DL3 vendor"] += 1
        (x0, y0, z0), (x1, y1, z1) = bbox(c)
        bvol = max((x1 - x0) * (y1 - y0) * (z1 - z0), 1e-9)
        if c.get("faces", 99) <= 16 and c.get("volume_mm3", 0) >= 0.95 * bvol and c.get("kind") not in ("specimen",):
            v("DL3", c["name"], f"plain box stand-in ({c.get('faces')} faces, fills {c['volume_mm3'] / bvol:.0%} of its bbox) "
              "instead of the vendor's model")
        p = vend.get(c["name"])
        if p is not None and not p.get("source_url") and not p.get("file"):
            v("DL3", c["name"], f"vendor part with no source_url and no file: {p.get('note') or p.get('author') or ''}".strip())

    # ---------- DL5 / DL6 materials ----------
    for c in comps:
        n, m = c["name"], c.get("material", "")
        checked["DL5/DL6 material"] += 1
        if n.startswith("lp_") and m != "aluminium_6061":
            v("DL5", n, f"laser part in {m}, not 6061 aluminium")
        if m.startswith("acrylic") and n not in ACRYLIC_ALLOWED and not (m == "acrylic_navy" and n.startswith("s_panel_")):
            # I016 (2026-10-13): the plinth panels and deck sheet are opaque cast acrylic 3 mm, non-structural
            v("DL5", n, "acrylic outside the drum tube and top disc")
        if n.startswith(("fp9_", "fp7_", "fp6_")) and m != "petg":
            v("DL6", n, f"printed part in {m}")
        if m == "petg" or c.get("kind") == "printed":
            if c.get("kind") in ("ring", "plate", "disc"):
                v("DL6", n, f"printed part of kind {c.get('kind')}: rings, plates and discs are laser aluminium")
            if PRINTED_FORBIDDEN_WORDS.search(n) or PRINTED_FORBIDDEN_WORDS.search(c.get("display_name", "")):
                v("DL6", n, "printed part named as a ring/plate/disc/hub")
            if rotating(c):
                v("DL6", n, "printed part on the rotor: allowed only as a cradle that carries no rotor load; "
                  "the reviewer must confirm the load path in the part header", severity="review")
            if n.startswith(("fp7_", "fp6_")):
                v("DL17", n, "legacy fp6/fp7 part name in a v9 build")

    # ---------- DL10 one fastener family per sub-assembly ----------
    tapped_vendor = set()
    for j in joints:
        if j.get("nut") == "tapped" and any(p.startswith("v_") for p in j.get("parts", [])):
            if j.get("screw_part"):
                tapped_vendor.add(j["screw_part"])
    fam = defaultdict(lambda: defaultdict(list))
    for c in comps:
        if c.get("kind") != "screw":
            continue
        checked["DL10 screws"] += 1
        if c["name"] in tapped_vendor:
            continue  # the vendor part dictates this thread
        dn = c.get("display_name", "")
        m = re.match(r"(M\d+(?:\.\d+)?)\s*x\s*[\d.]+\s*([A-Za-z ]+?)$", dn) or re.match(r"(M\d+(?:\.\d+)?)\D.*?([A-Z]{3,5}|thumb screw|shoulder screw)", dn)
        size, head = (m.group(1), m.group(2).strip()) if m else (dn.split(" ")[0], "?")
        if re.search(r"thumb|shoulder|set screw", dn, re.I):
            continue  # function screws (tool-free service, axles, set screws) are not the family
        fam[sub(c)][(size, head)].append(c["name"])
    for sa, fams in sorted(fam.items()):
        sizes = sorted({k[0] for k in fams})
        heads = sorted({k[1] for k in fams})
        if len(sizes) > 1 or len(heads) > 1:
            v("DL10", sa, f"{len(fams)} screw families (not dictated by a vendor thread): " +
              ", ".join(f"{s} {h} x{len(fams[(s, h)])}" for s, h in sorted(fams)))

    # ---------- DL11 proud tails / nuts on the top show face ----------
    if cage_struct:
        top_ring_top = union_box(cage_struct)[1][2]
        for c in comps:
            if is_fastener(c) and c["bbox_max_mm"][2] > top_ring_top + HEAD_MAX_PROUD + TOL:
                v("DL11", c["name"], f"fastener stands {c['bbox_max_mm'][2] - top_ring_top:.1f} mm above the top cage ring "
                  f"(the top show face) ({c.get('display_name', '')})")
    drum_top = [c for c in comps if c["name"] in ("lp_ring_c", "lp_bezel", "lp_ring_top")]
    if drum_top:
        zt = max(c["bbox_max_mm"][2] for c in drum_top)
        for c in comps:
            if is_fastener(c) and c.get("kind") == "nut" and rotating(c) and c["bbox_max_mm"][2] > zt - 0.1 \
                    and c["bbox_min_mm"][2] >= zt - 1.0:
                v("DL11", c["name"], "nut on the drum top face (show face)")

    # ---------- DL12 running band and net clearance ----------
    disc = by_name.get("lp_carrier_disc")
    mesh = by_name.get("s_mesh_sleeve")
    # I030: flat side mesh panels (s_cage_mesh_<side>): the net is a square, its inner half-width the nearest panel
    side_mesh = [c for c in comps if re.match(r"^s_cage_mesh_(back|front|right|left)$", c["name"])]
    if disc:
        dz = disc["bbox_min_mm"][2]
        for c in comps:
            if c["name"] == "lp_carrier_disc" or "roller" in c["name"]:
                continue
            (x0, y0, z0), (x1, y1, z1) = bbox(c)
            if rotating(c):
                checked["DL12 band"] += 1
            if z0 < dz - 0.05 and z1 > dz - 8.0:          # hangs below the disc underside within 8 mm
                rmin, rmax = xy_radius_range(c)
                if rmax > RUN_BAND[0] and rmin < RUN_BAND[1] and rotating(c):
                    v("DL12", c["name"], f"rotating part below the disc in the running band r {RUN_BAND[0]:.0f}..{RUN_BAND[1]:.0f} "
                      f"(r {rmin:.1f}..{rmax:.1f}, Z {z0:.1f})")
    if mesh or side_mesh:
        if mesh:
            r_net = (mesh["bbox_max_mm"][0] - mesh["bbox_min_mm"][0]) / 2.0 - 1.0
        else:
            r_net = min(min(abs(c["bbox_min_mm"][k]), abs(c["bbox_max_mm"][k]))
                        for c in side_mesh for k in (0, 1)
                        if c["bbox_max_mm"][k] - c["bbox_min_mm"][k] < 5.0)
        for c in comps:
            if not rotating(c):
                continue
            checked["DL12 net"] += 1
            (x0, y0, _), (x1, y1, _) = bbox(c)
            reach = max(abs(x0), abs(x1), abs(y0), abs(y1))
            if reach > r_net - NET_CLEAR + TOL:
                v("DL12", c["name"], f"rotating part reaches {reach:.1f} mm, net inner half-width {r_net:.1f} (needs {NET_CLEAR:.0f} clear)")

    # ---------- DL13 / DL14 wiring ----------
    wires = [c for c in comps if c.get("kind") in WIRE_KINDS or re.search(r"wire|harness|cable|gland", c["name"])]
    electrical = [c for c in comps if c.get("module") == "electrical"]
    checked["DL13 wiring"] = len(wires)
    if electrical and not wires:
        v("DL13", "(whole machine)", f"{len(electrical)} electrical components and 0 modelled wires, harnesses, clips, "
          "glands or strain reliefs: no wire has a route")
    for c in comps:
        if rotating(c) and c.get("module") == "electrical" and c.get("kind") in ("led",):
            rmin, rmax = xy_radius_range(c)
            if rmin > HUB_RADIUS:
                v("DL14", c["name"], f"rotor LED at r {rmin:.0f}..{rmax:.0f}: off the hub, its wire must cross the disc or climb the drum")

    # ---------- DL16 completeness ----------
    names = " ".join(c["name"] for c in comps).lower()
    for label, pat in REQUIRED_SUBSYSTEMS.items():
        if pat is None:
            continue
        checked["DL16 subsystems"] += 1
        if not re.search(pat, names):
            v("DL16", label, f"no component matches /{pat}/: the build is missing the {label}")

    # ---------- DL17 naming and tree ----------
    seen_paths = Counter()
    for c in comps:
        checked["DL17 naming"] += 1
        dn = (c.get("display_name") or "").strip()
        g = c.get("group") or []
        if not dn:
            v("DL17", c["name"], "display_name missing")
        elif dn == c["name"] or dn.startswith(INTERNAL_PREFIXES) or re.search(r"\b[a-z]+_[a-z0-9_]+\b", dn):
            v("DL17", c["name"], f"display_name reads like an internal key: '{dn}'")
        if c.get("naming_rule") == "fallback":
            v("DL17", c["name"], f"display_name from the fallback humaniser: '{dn}'")
        if len(g) < 2 or g[0].strip().lower() in BAD_GROUPS or g[1].strip().lower() in BAD_GROUPS:
            v("DL17", c["name"], f"not grouped into module > sub-assembly (group {g})")
        seen_paths[tuple(c.get("tree_path") or [c["name"]])] += 1
    for p, n in seen_paths.items():
        if n > 1:
            v("DL17", " > ".join(p), f"{n} components share one browser path")
    repo = repo_of(path)
    if repo and os.path.exists(os.path.join(repo, "lib", "render.py")):
        src = open(os.path.join(repo, "lib", "render.py"), errors="replace").read().splitlines()
        for i, line in enumerate(src, 1):
            if '"' not in line and "'" not in line:
                continue
            for t in RETIRED_TERMS:
                if re.search(t, line, re.I) and not line.lstrip().startswith("#"):
                    v("DL17", f"lib/render.py:{i}", f"render caption names a retired part ('{re.search(t, line, re.I).group(0)}'): {line.strip()[:110]}")
                    break

    # ---------- DL18 part count ----------
    pc, fc = Counter(), Counter()
    for c in comps:
        (fc if is_fastener(c) else pc)[sub(c)] += 1
    for sa, n in sorted(pc.items()):
        checked["DL18 sub-assemblies"] += 1
        if sa.split(" / ")[-1] in SUPPORT_SUBS:
            continue  # DL26: growth for support around the rotor and drum is always allowed
        if n > CAP_PARTS:
            v("DL18", sa, f"{n} non-fastener components (cap {CAP_PARTS}; a review prompt: state the reason)")
    for sa, n in sorted(fc.items()):
        if sa.split(" / ")[-1] in SUPPORT_SUBS:
            continue
        if n > CAP_FASTENERS:
            v("DL18", sa, f"{n} fastener pieces (cap {CAP_FASTENERS}; a review prompt: state the reason)")

    # ---------- DL26 support around the rotor and drum is never removed ----------
    prev = previous_components(previous)
    if prev is not None:
        cur = {c["name"] for c in comps}
        gone = [p for p in prev if p.get("name") not in cur and is_support(p)]
        checked["DL26 support compared"] = len([p for p in prev if is_support(p)])
        for p in gone:
            v("DL26", p["name"], f"support part removed since the previous build: {p.get('display_name', p['name'])} "
              f"({' / '.join(p.get('group') or [])}); support is kept unless Sawyer asked for it to go: cite his intent id or put it back")

    # ---------- DL27 two mirrored aluminium posts carry the drum top ----------
    posts = [c for c in comps if re.search(r"^(lp_|s_|v_drum_)\w*post", c["name"]) and not is_fastener(c)
             and c.get("module") != "cage"]      # I030: the cage's tube posts are not drum posts   # v_drum_post_*: bought 6061 standoffs (S1.32)
    checked["DL27 drum posts"] = len(posts)
    if len(posts) != DC.DRUM_TOP_POSTS:
        v("DL27", "(drum top)", f"{len(posts)} drum posts found, decision D2 is {DC.DRUM_TOP_POSTS} mirrored "
          "aluminium posts carrying the drum top (tube silicone is a seal only)")
    else:
        a, b = posts
        for p in posts:
            if p.get("material") != DC.LASER["material"]:
                v("DL27", p["name"], f"drum post in {p.get('material')}, not 6061 aluminium")
        ca = [(a["bbox_min_mm"][i] + a["bbox_max_mm"][i]) / 2 for i in range(3)]
        cb = [(b["bbox_min_mm"][i] + b["bbox_max_mm"][i]) / 2 for i in range(3)]
        sa_ = sorted(a["bbox_max_mm"][i] - a["bbox_min_mm"][i] for i in range(3))
        sb_ = sorted(b["bbox_max_mm"][i] - b["bbox_min_mm"][i] for i in range(3))
        mirrored = (abs(ca[2] - cb[2]) < 0.5 and all(abs(x - y) < 0.5 for x, y in zip(sa_, sb_)) and
                    (abs(ca[0] + cb[0]) < 0.5 and (abs(ca[1] - cb[1]) < 0.5 or abs(ca[1] + cb[1]) < 0.5) or
                     abs(ca[1] + cb[1]) < 0.5 and abs(ca[0] - cb[0]) < 0.5))
        if not mirrored:
            v("DL27", f"{a['name']}, {b['name']}", f"posts are not mirror images about the rotor axis "
              f"(centres {[round(x, 1) for x in ca]} / {[round(x, 1) for x in cb]})")

    # ---------- DL28 Sawyer's rim rollers and spindle stay ----------
    rollers = [c for c in comps if re.search(r"roller_umbb|rim_roller", c["name"]) and not is_fastener(c)
               and c.get("kind") != "printed"]
    checked["DL28 rim rollers"] = len(rollers)
    if len(rollers) != 3:
        v("DL28", "(rim rollers)", f"{len(rollers)} PU rim rollers, Sawyer's design has 3 (S1.27); they stay")
    if not any("pillow_block" in c["name"] for c in comps):
        v("DL28", "(spindle)", "goBILDA 1614 pillow block missing; Sawyer's spindle stays")

    # ---------- DL20 printed filament colours, at most DC.PRINTED_COLOURS_MAX ----------
    pcol = defaultdict(list)
    for c in comps:
        if c.get("material") == "petg":
            pcol[c.get("colour", "?").lower()].append(c["name"])
    checked["DL20 printed colours"] = len(pcol)
    if len(pcol) > DC.PRINTED_COLOURS_MAX:
        v("DL20", "(printed parts)", f"{len(pcol)} filament colours on printed parts: " +
          "; ".join(f"{k}: {', '.join(sorted(n))}" for k, n in sorted(pcol.items())))

    # ---------- DL21 connectors on one panel share a centreline ----------
    conn = [c for c in comps if c.get("group") and len(c["group"]) > 1 and c["group"][1] == "Panel connectors"]
    by_face = defaultdict(list)
    for c in conn:
        (x0, y0, z0), (x1, y1, z1) = bbox(c)
        face = "back" if y1 > 140 else "front" if y0 < -140 else "right" if x1 > 140 else "left" if x0 < -140 else "inside"
        by_face[face].append((c["name"], (z0 + z1) / 2.0))
    for face, lst in by_face.items():
        checked["DL21 connector faces"] += 1
        zs = [z for _, z in lst]
        if len(lst) > 1 and max(zs) - min(zs) > 1.0:
            v("DL21", ", ".join(n for n, _ in lst), f"connectors on the {face} panel sit on {len(lst)} centrelines "
              f"(Z {min(zs):.1f}..{max(zs):.1f}); one row, one datum")

    # ---------- DL22 open spinning mechanism between the deck and the disc ----------
    if disc and panels:
        gap = disc["bbox_min_mm"][2] - pz1
        guards = [c for c in comps if re.search(r"skirt|guard|shroud|cowl", c["name"])]
        checked["DL22 guard"] = 1
        if gap > 10.0 and not guards:
            v("DL22", "lp_carrier_disc", f"{gap:.0f} mm open band between the deck top (Z {pz1:.0f}) and the spinning "
              "disc underside: hub and collar are on show and reachable; no skirt or guard part (add one around the support, DL26)")

    # ---------- build health carried into the report ----------
    inter = sheet.get("interference", {})
    if inter.get("pairs"):
        v("DL24", "(build)", f"{inter['pairs']} interference pairs")
    bad = [j["name"] for j in joints if not j.get("ok", True)]
    for b in bad:
        v("DL24", b, "joint check not ok")

    return {
        "command": "design",
        "ok": True,
        "commit": git_commit(repo),
        "sheet": os.path.abspath(path) if os.path.exists(path) else path,
        "checked": {"components": len(comps), "joints": len(joints), **dict(checked),
                    "envelope": env, "caps": {"parts": CAP_PARTS, "fasteners": CAP_FASTENERS},
                    "previous": prev is not None},
        "summary": dict(sorted(Counter(x["rule"] for x in V).items(), key=lambda kv: int(kv[0][2:]))),
        "by_severity": dict(Counter(x["severity"] for x in V)),
        "violation_count": len(V),
        "violations": V,
    }


def run(path, previous=None):
    """check() on a sheet.json path; never raises (an error is reported in the JSON)."""
    try:
        sheet = json.load(open(path))
    except Exception as e:  # report, never crash
        return {"command": "design", "ok": False, "error": f"cannot read {path}: {e}", "violations": [],
                "violation_count": 0}
    try:
        return check(sheet, path, previous)
    except Exception as e:
        return {"command": "design", "ok": False, "error": f"{type(e).__name__}: {e}", "violations": [],
                "violation_count": 0}


def main(argv):
    args = argv[1:]
    prev = None
    if "--previous" in args:
        i = args.index("--previous")
        prev = args[i + 1] if i + 1 < len(args) else None
        del args[i:i + 2]
    if not args:
        print(json.dumps({"error": "usage: designcheck.py <sheet.json> [--previous <sheet.json>]", "violations": []}))
        return 0
    print(json.dumps(run(args[0], prev), indent=1))
    return 0


if __name__ == "__main__":
    try:
        main(sys.argv)
    except Exception as e:  # always one JSON object, always exit 0
        print(json.dumps({"error": f"{type(e).__name__}: {e}", "violations": []}))
    sys.exit(0)
