"""cadkit command line: lint, views, montage, demo, idioms, view."""
import argparse
import json
import sys
import time


def _floats(s, n):
    v = [float(x) for x in s.split(",")]
    if len(v) != n:
        raise argparse.ArgumentTypeError(f"expected {n} comma-separated numbers, got {s!r}")
    return v


def cmd_lint(a):
    from .lint import format_text, lint_step, to_json
    t = time.time()
    rep = lint_step(a.step, kind=a.kind, envelope=_floats(a.envelope, 6) if a.envelope else None, min_wall=a.min_wall)
    rep["seconds"] = round(time.time() - t, 1)
    print(to_json(rep) if a.json else format_text(rep) + f"\n({rep['seconds']} s)")
    return 0


def cmd_views(a):
    from .views import views_step
    closes = {}
    for c in a.close or []:
        name, _, xyz = c.partition("=")
        closes[name] = _floats(xyz, 4)
    t = time.time()
    out = views_step(a.step, a.out, closes=closes, frame_from=a.frame_from)
    print(json.dumps(dict(renders=out, seconds=round(time.time() - t, 1)), indent=1))
    return 0


def cmd_montage(a):
    from .views import montage
    print(montage(a.dir_a, a.dir_b, a.out, tuple(a.labels.split(","))))
    return 0


def cmd_demo(a):
    from .demo import run
    run(a.out)
    return 0


def cmd_idioms(a):
    from .idioms import parts
    for name in parts.IDIOMS:
        print(name)
    return 0


def cmd_view(a):
    from .viewer import server
    return server.run(a)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="cadkit", description="Finished CAD idioms, B-rep lint and fixed views.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("lint", help="B-rep ugly detectors on a STEP (exit 0 always)")
    p.add_argument("step")
    p.add_argument("--kind", choices=["printed", "laser", "machined", "vendor"], help="treat every part as this kind")
    p.add_argument("--envelope", help="xmin,ymin,zmin,xmax,ymax,zmax in mm")
    p.add_argument("--min-wall", type=float, default=2.0, help="thin wall threshold for printed parts (mm)")
    p.add_argument("--json", action="store_true")
    p.set_defaults(fn=cmd_lint)
    p = sub.add_parser("views", help="fixed camera views of a STEP: iso front right top back bottom + close-ups")
    p.add_argument("step")
    p.add_argument("--out", help="output dir (default <step dir>/<stem>_views)")
    p.add_argument("--close", action="append", help="name=x,y,z,r close-up (repeatable)")
    p.add_argument("--frame-from", help="reuse the cameras in <dir>/views.json (before/after framing)")
    p.set_defaults(fn=cmd_views)
    p = sub.add_parser("montage", help="before/after sheet of two views dirs")
    p.add_argument("dir_a")
    p.add_argument("dir_b")
    p.add_argument("--out")
    p.add_argument("--labels", default="before,after")
    p.set_defaults(fn=cmd_montage)
    p = sub.add_parser("demo", help="build every idiom, montage, lint the ugly and nice test parts")
    p.add_argument("--out", help="output root (default <cadkit repo>/out)")
    p.set_defaults(fn=cmd_demo)
    p = sub.add_parser("idioms", help="list the idioms")
    p.set_defaults(fn=cmd_idioms)
    from .viewer.server import add_args as view_args
    p = sub.add_parser("view", help="the standard local three.js viewer for .glb/.stl/.ply files or directories "
                                    "(tree, cameras, section, explode, undo/redo, movable group + pose save)")
    view_args(p)
    p.set_defaults(fn=cmd_view)
    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
