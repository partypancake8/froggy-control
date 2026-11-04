# model/

Froggy Turntable V12.5 CAD, exports and simulation numbers. Start with [../docs/SIMULATION.md](../docs/SIMULATION.md).

| Path | What it is |
| --- | --- |
| `sim-params.json` | Every number a simulation needs, with units: `rotor` (spin axis, which GLB nodes turn, mass, inertia, disc and wall frog positions), `drum`, `camera`, `motor` (24 V torque curve), `drive`, `frog` (literature adhesion values), `targets` (400/600 rpm, 60 ms slip-to-stop, rpm vs g table, ramp profiles) and `machine` |
| `exports/froggy_v12_5.glb` | Whole machine as a GLB, 477 named meshes (one per part, named as in the CAD tree), coloured, millimetres, Z up |
| `exports/froggy_v12_5.step` | Whole machine as a STEP AP214 assembly tree (module > sub-assembly > part). Written without p-curves (see below) |
| `exports/froggy_v12_5.stl` | Whole machine as one STL, millimetres, Z up |
| `exports/design.json` | Design-language check result for this build |
| `exports/camera_view.png` | Render from the rotor camera |
| `exports/camsim/` | Camera simulation: `camsim.json` (lens pose, FOV, projection, per-frog pixel density) and fisheye renders for the as-built mount and six alternative tilts and heights (`*_labels.png` colour each part and list its pixel count) |
| `cad/` | The build123d source that makes all of the above: `bin/cad build froggy_v12_5`, `bin/view` (web viewer). A trimmed copy of the froggy-cad repo at commit 357616d, see `cad/README.md` |

About the STEP: the normal `bin/cad build` STEP is about 125 MB, over GitHub's 100 MB file limit. The committed one comes from `cad/scripts/export_step_light.py`, which writes the same 668 solids, names and tree but leaves out the 2D parameter-space curves (p-curves) that STEP readers rebuild on import. It is 52 MB, reads back with all 668 solids valid, and the total solid volume matches the build to within 2 mm3 out of 3.7 million mm3. Nothing was left out of `exports/`; the 3MF (16.6 MB) is skipped because the GLB carries the same meshes and colours.

The drive in these files is the as-built 20T:20T GT2 pulleys with a 100 mm belt. The decided change to 30T:30T with a 120 mm belt is in `sim-params.json` under `drive.decided_not_in_cad` but not in the geometry yet.
