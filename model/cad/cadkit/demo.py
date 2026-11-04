"""`cadkit demo`: build every idiom with defaults to out/idioms/<name>.step, render one iso per idiom
into out/idioms/montage.png, build tests/ugly_part.py and tests/nice_part.py to out/test_parts/
and print `cadkit lint` on both."""
import importlib.util
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _export(shape, path, extra=None):
    from build123d import Compound, export_step
    if extra:
        c = Compound(children=[shape] + list(extra))
        c.label = os.path.splitext(os.path.basename(path))[0]
        export_step(c, path)
    else:
        export_step(shape, path)
    return path


def build_idioms(outdir):
    from .idioms import IDIOMS
    os.makedirs(outdir, exist_ok=True)
    rows = []
    for name, fn in IDIOMS.items():
        t = time.time()
        shape, feats = fn()
        extra = [feats["lip"]] if "lip" in feats else None
        path = _export(shape, os.path.join(outdir, f"{name}.step"), extra)
        rows.append(dict(name=name, step=path, label=shape.label, valid=shape.is_valid,
                         volume=round(shape.volume, 1), faces=len(shape.faces()), seconds=round(time.time() - t, 2)))
    return rows


def render_montage(rows, outdir):
    from .stepio import load
    from .views import contact_sheet, render_parts
    tiles, labels = [], []
    for r in rows:
        parts = load(r["step"])
        d = os.path.join(outdir, "renders", r["name"])
        out = render_parts(parts, d, "", views=["iso"], px=700)
        tiles.append(out["iso"])
        labels.append(r["name"])
    return contact_sheet(tiles, os.path.join(outdir, "montage.png"), labels, "cadkit idioms (defaults), iso", cols=4,
                         tile=420)


def _load_test(name):
    path = os.path.join(REPO, "tests", f"{name}.py")
    spec = importlib.util.spec_from_file_location(f"cadkit_test_{name}", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def build_test_parts(outdir):
    os.makedirs(outdir, exist_ok=True)
    out = {}
    for n in ("ugly_part", "nice_part"):
        mod = _load_test(n)
        out[n] = mod.write(os.path.join(outdir, f"{n}.step"))
    return out


def run(out_root=None):
    from .lint import format_text, lint_step
    out_root = out_root or os.path.join(REPO, "out")
    idir = os.path.join(out_root, "idioms")
    t0 = time.time()
    rows = build_idioms(idir)
    for r in rows:
        print(f"  {r['name']:<24} {r['label']:<34} valid={r['valid']} vol={r['volume']} mm3 "
              f"faces={r['faces']} {r['seconds']} s -> {r['step']}")
    m = render_montage(rows, idir)
    print(f"montage: {m}")
    tdir = os.path.join(out_root, "test_parts")
    tp = build_test_parts(tdir)
    for n, path in tp.items():
        print(f"\n== cadkit lint {path}")
        print(format_text(lint_step(path)))
    from .views import montage, views_step
    ua = views_step(tp["ugly_part"], os.path.join(tdir, "ugly_views"))
    views_step(tp["nice_part"], os.path.join(tdir, "nice_views"), frame_from=os.path.dirname(ua["iso"]))
    um = montage(os.path.join(tdir, "ugly_views"), os.path.join(tdir, "nice_views"),
                 os.path.join(tdir, "ugly_vs_nice.png"), ("ugly (raw primitives)", "nice (cadkit idioms)"))
    print(f"\nbefore/after montage: {um}")
    print(f"\ndemo done in {time.time() - t0:.1f} s")
    return dict(idioms=rows, montage=m, test_parts=tp)


if __name__ == "__main__":
    run(sys.argv[1] if len(sys.argv) > 1 else None)
