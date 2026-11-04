"""Serve this checkout's existing PROD or DEV build, with explicit identity."""
import argparse
import json
from pathlib import Path
from . import viewer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-open", action="store_true")
    args = parser.parse_args()
    profile = json.loads((Path(viewer.ROOT) / "viewer.json").read_text())
    code, result = viewer.serve(profile["assembly"], port=profile["port"],
                                open_browser=not args.no_open)
    print(json.dumps(result))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
