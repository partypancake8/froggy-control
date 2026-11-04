"""Assembly checks: pairwise interference (bbox prefilter, real boolean only on
overlapping or touching boxes, every N choose 2 pair counted), clearance
distances for touching pairs, B-rep validity, and joint record evaluation.
Drum + base additions: interference by class (crash / hardware_in_host / joint),
a spin test of the rotor module about Z, appearance coverage and module tags."""
import itertools
import math
import time

from lib.assembly import boxes_touch
from lib.joints import FLAGS, evaluate_joint

TOL_VOL = 0.01


def interference(A):
    names = list(A.parts)
    hits, booleans, checked = [], 0, 0
    for a, b in itertools.combinations(names, 2):
        checked += 1
        v, _ = A.ivol(a, b)          # cached when a joint check already ran it
        booleans += boxes_touch(A.bbox(a), A.bbox(b))
        if v >= TOL_VOL:
            hits.append(dict(a=a, b=b, volume_mm3=round(v, 4)))
    return dict(pairs=len(hits), checked_pairs=checked, booleans_run=booleans,
                note="booleans_run = pairs whose boxes overlap or touch; all others are disjoint by bbox", hits=hits,
                tolerance_mm3=TOL_VOL)


HARDWARE = ("screw", "nut", "insert", "set_screw")
MODULES = ("frame", "drive_base", "rotor", "cage", "feet", "electrical", "plinth")


def _joint_of(A, name):
    return A.parts[name].meta.get("joint")


def classify(A, a, b):
    """crash: two non-hardware bodies; joint: two bodies of the same joint;
    hardware_in_host: a screw, nut or insert in anything else."""
    ha, hb = A.parts[a].kind in HARDWARE, A.parts[b].kind in HARDWARE
    if not ha and not hb:
        return "crash"
    ja, jb = _joint_of(A, a), _joint_of(A, b)
    if ha and hb and ja and ja == jb:
        return "joint"
    return "hardware_in_host"


def by_class(A, hits):
    out = {"crash": 0, "hardware_in_host": 0, "joint": 0}
    for h in hits:
        h["class"] = classify(A, h["a"], h["b"])
        out[h["class"]] += 1
    return out


def _cyl_env(bb):
    """Rotation-invariant envelope of a bbox about Z: (r_min, r_max, z_min, z_max)."""
    (x0, y0, z0), (x1, y1, z1) = bb
    rmax = max(math.hypot(x, y) for x in (x0, x1) for y in (y0, y1))
    dx = 0.0 if x0 <= 0 <= x1 else min(abs(x0), abs(x1))
    dy = 0.0 if y0 <= 0 <= y1 else min(abs(y0), abs(y1))
    return math.hypot(dx, dy), rmax, z0, z1


def spin_test(A, angles=(0, 90, 180, 270)):
    """Rotate every rotor component about Z and rerun interference against the
    frame and drive_base modules. Angle 0 is the assembled pose (cached)."""
    from build123d import Axis
    from lib.assembly import bbox_of, common_volume
    def spins(p):   # rotor module, or a part of another module that turns with it (meta spins)
        return p.module == "rotor" or bool(p.meta.get("spins"))
    rotor = [n for n, p in A.parts.items() if spins(p)]
    base = [n for n, p in A.parts.items() if not spins(p)]
    env = {}
    for n in A.parts:
        e = _cyl_env(A.bbox(n))
        origin = A.parts[n].meta.get("spin_origin", (0, 0, 0))
        if origin[:2] != (0, 0) and any(origin[:2]):
            # Off-axis motor shaft/pulley rotates about its own spindle, not Z0.
            bb = A.bbox(n)
            radius = max(math.hypot(x-origin[0], y-origin[1])
                         for x in (bb[0][0], bb[1][0]) for y in (bb[0][1], bb[1][1]))
            offset = math.hypot(*origin[:2])
            e = (max(0, offset-radius), offset+radius, e[2], e[3])
        env[n] = (max(e[0], A.parts[n].meta.get("rmin", 0.0)),) + e[1:]
    cand = []
    for r in rotor:
        er = env[r]
        for b in base:
            eb = env[b]
            if er[1] < eb[0] - 1e-3 or eb[1] < er[0] - 1e-3 or er[3] < eb[2] - 1e-3 or eb[3] < er[2] - 1e-3:
                continue
            cand.append((r, b))
    hits, booleans = [], 0
    for ang in angles:
        moved = {}
        for r, b in cand:
            if ang % 360 == 0:
                v, ran = A.ivol(r, b)
                booleans += ran
            else:
                if r not in moved:
                    origin = A.parts[r].meta.get("spin_origin", (0, 0, 0))
                    s = A.parts[r].solid.rotate(Axis(origin, (0, 0, 1)), ang)
                    moved[r] = (s, bbox_of(s))
                s, bb = moved[r]
                if not boxes_touch(bb, A.bbox(b)):
                    continue
                v = common_volume(s, A.parts[b].solid)
                booleans += 1
            if v >= TOL_VOL:
                hits.append(dict(angle=ang, rotor=r, base=b, volume_mm3=round(v, 4)))
    return dict(ok=not hits, angles_checked=len(angles), angles=list(angles), hits=hits,
                candidate_pairs=len(cand), booleans_run=booleans)


def appearance(A):
    have = [n for n, p in A.parts.items() if p.colour is not None and len(p.colour) >= 3]
    return round(len(have) / max(1, len(A.parts)), 6)


def modules(A):
    counts = {}
    for p in A.parts.values():
        counts[p.module] = counts.get(p.module, 0) + 1
    untagged = [n for n, p in A.parts.items() if p.module not in MODULES]
    return dict(counts=counts, untagged=untagged)


def clearances(A):
    """Distance between each bracket and its mate rails (0 = face contact)."""
    out = {}
    for n in A.names("bracket"):
        out[n] = {m: round(A[n].distance_to(A[m]), 4) for m in A.parts[n].meta.get("mates", [])}
    return out


def validity(A):
    bad = [n for n, p in A.parts.items() if not p.solid.is_valid]
    return dict(valid=not bad, invalid=bad)


def run_all(A, spin=None):
    t = time.time()
    joints = [evaluate_joint(A, s) for s in A.joints]
    inter = interference(A)
    inter["by_class"] = by_class(A, inter["hits"])
    clear = clearances(A)
    val = validity(A)
    fails = []
    for h in inter["hits"]:
        fails.append(f"FAIL interference ({h['class']}) {h['a']} x {h['b']} {h['volume_mm3']} mm3")
    extra = {}
    if spin is None:
        spin = any(p.module == "rotor" for p in A.parts.values())
    if spin:
        st = spin_test(A)
        extra["spin_test"] = st
        for h in st["hits"]:
            fails.append(f"FAIL spin {h['angle']} deg {h['rotor']} x {h['base']} {h['volume_mm3']} mm3")
        cov = appearance(A)
        extra["appearance_coverage"] = cov
        if cov != 1.0:
            fails.append(f"FAIL appearance_coverage {cov}")
        mods = modules(A)
        extra["modules"] = mods
        for n in mods["untagged"]:
            fails.append(f"FAIL component {n} not tagged with a module in {MODULES}")
    for r in joints:
        for k in FLAGS:
            if r.get(k) is not True:
                fails.append(f"FAIL joint {r['name']} {k}: {r['why']}")
    for n, d in clear.items():
        for m, v in d.items():
            if v > 0.05:
                fails.append(f"FAIL clearance {n} to {m} {v} mm (expected contact)")
    for n in val["invalid"]:
        fails.append(f"FAIL invalid solid {n}")
    return dict(interference=inter, joints=joints, clearances=clear, valid=val["valid"] and not fails,
                fails=fails, seconds=round(time.time() - t, 2), **extra)
