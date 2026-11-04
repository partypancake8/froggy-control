# froggy-cad (V12.5 subset)

> **This folder in froggy-control is a trimmed copy of Sawyer's froggy-cad repo at commit 357616d ("Promote V12.5 to PROD", 2026-11-04).** It holds only what `bin/cad build froggy_v12_5` and `bin/view` need: the V12.5 assembly and the older assemblies it imports (v7, v8, v12, frame, drum_base), the parts and lib modules they use, a vendored copy of the `cadkit` helper package (commit 4036170), and the vendor STEP/STL files the V12.5 build actually loads. Other assemblies named below (frame, drum_base, froggy_v9) may not build here because their vendor files were left out. Paths like `~/cad/froggy-cad` and the PROD/DEV worktrees below describe Sawyer's machine, not this copy. Setup, build and viewer commands for this copy are in [../../docs/SIMULATION.md](../../docs/SIMULATION.md).

Headless build123d CAD for the Froggy Turntable (ROB 450 Team 03). Nothing drives Fusion; open `out/<name>/<name>.step` there to view.

Current production revision: **V12.5**. The main checkout (`~/cad/froggy-cad`, branch `master`) is **PROD**. Start its live viewer with `bin/view --no-open` at http://127.0.0.1:7894/.

Active development lives in `~/cad/froggy-cad-wt/dev`, branch `dev`, with a separate **DEV** viewer at http://127.0.0.1:7897/. Run `bin/view --no-open` there. Both viewers show the environment and revision in the page header and browser tab. `viewer.json` records each checkout's environment, assembly and port. The launcher serves the existing build; it does not rebuild geometry.

Make design changes in DEV and rebuild there with `bin/cad build froggy_v12_5`; the DEV viewer reloads its own output automatically. PROD keeps its independent output until an explicitly requested promotion. Start with [V12.5 assembly instructions](docs/V12.5-ASSEMBLY.md) and [interfaces](docs/ICD-V12.5.md). Earlier V12 manufacturing packets are historical and do not describe every V12.5 change. PROD denotes the selected baseline, not physical qualification.

The commands and original framework dimensions below describe the repository's early examples; V12 dimensions are in its ICD and assembly packet.

Setup: `uv venv --python 3.12 .venv`, then `uv pip install --python .venv/bin/python -r requirements.txt` (or `python3.12 -m venv .venv && .venv/bin/pip install -r requirements.txt`).
- `bin/cad build frame`: build, check, export STEP/STL/3MF/GLB, render 6 PNGs, write out/frame/sheet.json + topology.txt
- `bin/cad check frame`: rerun interference + joint checks on the built assembly, FAIL lines and non-zero exit on failure
- `bin/cad inspect|render|export frame` (`render --engine blender` for a Workbench iso); every command prints one JSON object
- `bin/cad build drum_base`: frame + motor stack (drive_base) + rotor; adds spin test, interference by class, 10 PNGs
- `bin/cad printcheck <part>`: one printed part alone, writes out/parts/<part>/<part>.stl + .3mf (P2S), checks walls/overhang/bed
- `bin/cad check <name>` rebuilds from source (fast); `--pickle` checks out/<name>/assembly.pkl instead
- `bin/cad tree <name>`: the exported STEP's label tree read back with OCP (module > sub-assembly > parts, one
  "Fasteners" folder of joint groups per sub-assembly), checks every label is readable and placed, and dry-runs
  cad-fusion's occurrence matching; names come from `lib/naming.py` (sheet.json `display_name`, `group`, `tree_path`)
- `bin/cad design froggy_v9`: design-language check (docs/DESIGN-LANGUAGE.md, numbers in `lib/design_constants.py`), JSON violations with rule and severity, exit 0; also a phase of `build` (`design_violations`)
- `bin/cad review froggy_v9`: review packet in out/froggy_v9/review/ (9 fixed views from `lib/review_views.py`, before/after montage, design.json, tree, ASSUMPTIONS.md, REVIEW.md), archived to out/reviews/<sha>/
- `bin/cad intent add|list|resolve|md`: Sawyer's verbatim verdicts in docs/intent/intent.json, gallery docs/intent/REJECTED.md
- `bin/cad-fusion render froggy_v9 [--dry-run]`: Fusion viewport captures from the same fixed cameras into out/froggy_v9/review/fusion_<view>.png
- Briefs start from docs/BRIEF-TEMPLATE.md; open punch items in docs/PUNCH-LIST.md

Layout:
- `assemblies/<name>.py`: `build()` returns an Assembly (frame = 6 rails, 8 Tovi brackets, 16 M3 slot-nut joints)
- `lib/`: profile_1515 (rail), vendor (STEP import, hole reading), hardware (SHCS, slot nut), joints (bolt_joint records), checks, render, sheet, assembly
- `vendor/opensource/<part>/`: downloaded STEP + source.json (url, author, licence, price)
- `vendor/motion`, `vendor/coupling`: NEMA 17 and ServoCity hub STEPs; rows in `vendor/manifest.json`
- `parts/<name>.py`: one printed or stock part each, `build()` + `SPEC`; stack datums in `parts/datums.py`
- Drum + base stack: motor face Z 41, plate 41..47.5, 6814 pair Z 16..36, hub flange-up 53.5..57.5, deck top 64.5, drum floor 60.5
- `froggy_cad/cli.py` behind `bin/cad`; `out/` is build output (gitignored); protected checks live outside the repo

Datums: Z 0 = rail top face, rails occupy Z -15..0, X and Y zero at the frame centre, Z up.
15x15 profile: slot opening 3.20, slot depth 5.15, cavity 5.80, lip 1.40, centre bore 2.50.
Frame: 340 (X) x 250 (Y) outer, long X rails full length, Y rails between them, cross rails at X -45 and +45.
