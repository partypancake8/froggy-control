"""Write a lighter STEP of a built assembly: same solids, names and tree as `bin/cad build`, without the 2D
p-curves (the optional parameter-space copies of every edge, which STEP readers rebuild on import).

  .venv/bin/python scripts/export_step_light.py froggy_v12_5 [out.step]

Reads out/<name>/assembly.pkl (written by `bin/cad build <name>`) and writes out/<name>/<name>_light.step by
default. The froggy_v12_5 full STEP is about 125 MB, over GitHub's 100 MB file limit; this one is what
model/exports/froggy_v12_5.step in froggy-control is.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)


def main(argv):
    if not argv:
        print(__doc__, file=sys.stderr)
        return 2
    name = argv[0]
    out = os.path.join(ROOT, "out", name)
    dest = argv[1] if len(argv) > 1 else os.path.join(out, f"{name}_light.step")
    from build123d import export_step
    from froggy_cad.cli import load
    from lib.mesh import step_tree
    if not os.path.exists(os.path.join(out, "assembly.pkl")):
        print(f"no out/{name}/assembly.pkl; run bin/cad build {name} first", file=sys.stderr)
        return 1
    A, _ = load(name)
    export_step(step_tree(A, name), dest, write_pcurves=False)
    print(f"{dest} {os.path.getsize(dest) / 1e6:.1f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
