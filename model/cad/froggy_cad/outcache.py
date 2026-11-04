"""Latest-build cache outside the repo: <repo parent>/froggy-cad-cache/<name>/ holds the GLB, sheet.json
(and build.json) of the newest `bin/cad build <name>` plus meta.json, so `bin/cad view` can serve or restore it
when out/ is gone. One copy per assembly, no history; the STEP is not cached."""
import json
import os
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CACHE_ROOT = os.path.join(os.path.dirname(ROOT), "froggy-cad-cache")
SKIP_DIRS = {"__pycache__", ".venv", "out", ".git"}
SOURCE_DIRS = ("parts", "lib", "assemblies", "froggy_cad")


def out_dir(name):
    return os.path.join(ROOT, "out", name)


def cache_dir(name):
    return os.path.join(CACHE_ROOT, name)


def _files(name):
    return [f"{name}.glb", "sheet.json", "build.json"]


def source_newest_mtime(name):
    """Newest mtime among assemblies/<name>.py, every .py under parts/ lib/ assemblies/ froggy_cad/ and every file
    under vendor/."""
    newest = 0.0
    p = os.path.join(ROOT, "assemblies", f"{name}.py")
    if os.path.exists(p):
        newest = os.path.getmtime(p)
    for d, py_only in [(s, True) for s in SOURCE_DIRS] + [("vendor", False)]:
        top = os.path.join(ROOT, d)
        for dp, dns, fns in os.walk(top):
            dns[:] = [x for x in dns if x not in SKIP_DIRS]
            for f in fns:
                if py_only and not f.endswith(".py"):
                    continue
                try:
                    m = os.path.getmtime(os.path.join(dp, f))
                except OSError:
                    continue
                if m > newest:
                    newest = m
    return newest


def _git(*args):
    try:
        return subprocess.run(["git", "-C", ROOT, *args], capture_output=True, text=True, timeout=20).stdout.strip()
    except Exception:  # noqa: BLE001
        return ""


def _atomic_copy(src, dst):
    tmp = f"{dst}.tmp-{os.getpid()}"
    shutil.copy2(src, tmp)
    os.replace(tmp, dst)


def read_meta(name):
    try:
        with open(os.path.join(cache_dir(name), "meta.json")) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def save(name, build_ok=None):
    """Copy out/<name>/{<name>.glb, sheet.json, build.json} into the cache (replacing it) and write meta.json."""
    src, dst = out_dir(name), cache_dir(name)
    glb = os.path.join(src, f"{name}.glb")
    if not os.path.exists(glb):
        return None
    os.makedirs(dst, exist_ok=True)
    for f in _files(name):
        s, d = os.path.join(src, f), os.path.join(dst, f)
        if os.path.exists(s):
            _atomic_copy(s, d)
        elif os.path.exists(d):
            os.remove(d)
    dirty = bool(_git("status", "--porcelain", "--untracked-files=no", "--", *SOURCE_DIRS, "vendor"))
    meta = dict(sha=_git("rev-parse", "--short", "HEAD") or "unknown", dirty=dirty,
                built_at=time.strftime("%Y-%m-%dT%H:%M:%S%z"), source_newest_mtime=source_newest_mtime(name),
                glb_bytes=os.path.getsize(os.path.join(dst, f"{name}.glb")), build_ok=build_ok)
    tmp = os.path.join(dst, f"meta.json.tmp-{os.getpid()}")
    with open(tmp, "w") as fh:
        json.dump(meta, fh, indent=1)
    os.replace(tmp, os.path.join(dst, "meta.json"))
    return meta


def restore(name):
    """Copy the cached GLB + sheet.json (+ build.json) back into out/<name>/."""
    src, dst = cache_dir(name), out_dir(name)
    os.makedirs(dst, exist_ok=True)
    for f in _files(name):
        s = os.path.join(src, f)
        if os.path.exists(s):
            _atomic_copy(s, os.path.join(dst, f))
    return os.path.join(dst, f"{name}.glb")


def resolve(name):
    """What `view` would serve without building: out GLB if it exists, else cache GLB, else none."""
    newest = source_newest_mtime(name)
    out_glb = os.path.join(out_dir(name), f"{name}.glb")
    cache_glb = os.path.join(cache_dir(name), f"{name}.glb")
    cands = [(s, p, os.path.getmtime(p)) for s, p in (("out", out_glb), ("cache", cache_glb)) if os.path.exists(p)]
    fresh_c = [c for c in cands if c[2] >= newest]
    return dict(newest=newest, candidates=cands, fresh=fresh_c, meta=read_meta(name))


def log(*a):
    print(*a, file=sys.stderr, flush=True)
