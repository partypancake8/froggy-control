# Simulating the Froggy Turntable

This is the guide for building a simulation of the V12.5 turntable. The two jobs it is written for are:

1. testing the slip-detection software against synthetic camera video, and
2. simulating a tree frog on the spinning drum until it slips.

Everything you need is in [`model/`](../model/README.md). You do not need Sawyer's machine or the private froggy-cad repo.

## What is in model/

| Path | Use it for |
| --- | --- |
| `model/sim-params.json` | All the numbers: rotor mass and inertia, which meshes spin, drum and disc sizes, camera pose and lens, motor torque curve, drive ratio, frog adhesion values, speed and latency targets. Every number has its unit in the key name or a `unit` field. |
| `model/exports/froggy_v12_5.glb` | The whole machine for a simulator or renderer. 477 meshes, one per part, named like the CAD tree. |
| `model/exports/froggy_v12_5.step` | Exact CAD geometry (solids) if your tool wants B-rep, for example Fusion, Onshape or FreeCAD. |
| `model/exports/froggy_v12_5.stl` | Single triangle mesh of the whole machine. |
| `model/exports/camsim/` | What the rotor camera sees, from the CAD: `camsim.json` plus fisheye renders. |
| `model/exports/camera_view.png`, `design.json` | A render from the camera position and the design check result. |
| `model/cad/` | The source that makes the exports, if you want to change the geometry and rebuild. |

## Coordinate frame and units

The GLB, STEP, STL and `sim-params.json` all share one frame:

- units are **millimetres** (the GLB too, it is not converted to metres)
- **Z is up**, Z 0 is the top face of the 2020 deck rails
- X = Y = 0 is the rotor spin axis, so the rotor turns about the world Z axis
- +X is the side the CAD frogs sit on, -Y is the front panel (where the e-stop is)

glTF normally means Y-up metres, so most loaders (three.js, Blender, Unity, Isaac Sim) will show the model lying on its side and 1000 times too big. Fix that on import with a scale of 0.001 and a rotation of -90 degrees about X. In MuJoCo or PyBullet, keep Z up and just scale by 0.001.

The GLB is a flat list of 477 nodes with no transforms; every vertex is already in the world frame. A node's name is the part's display name, for example `Top disc, 186 x 3.2 acrylic` or `Disc frog body, Agalychnis callidryas`.

### Which nodes spin

`sim-params.json` lists them:

- `rotor.rotates_glb_nodes`: the 78 meshes that turn with the drum about the Z axis (spindle, hub, carrier disc, tube, top disc, bezel, camera, LED ring, Pi, battery, both frogs and their screws)
- `rotor.rotates_about_motor_axis`: the motor shaft and motor pulley, which spin about their own axis at (0, 30)
- `rotor.belt_glb_nodes`: the GT2 belt
- `rotor.fixed_glb_nodes`: everything else (frame, plinth, cage, electronics box)

This splits the GLB into a rotor mesh and a stator mesh, in mm, with trimesh (it is installed by the CAD requirements, or `pip install trimesh`):

```python
import json
import trimesh

P = json.load(open("model/sim-params.json"))
scene = trimesh.load("model/exports/froggy_v12_5.glb", force="scene")
turns = set(P["rotor"]["rotates_glb_nodes"])
parts = {"rotor": [], "stator": []}
for node in scene.graph.nodes_geometry:
    T, geom = scene.graph[node]
    mesh = scene.geometry[geom].copy()
    mesh.apply_transform(T)
    parts["rotor" if node in turns else "stator"].append(mesh)
for name, meshes in parts.items():
    trimesh.util.concatenate(meshes).export(f"{name}.stl")
```

It prints nothing; it writes `rotor.stl` (78 meshes) and `stator.stl` (399 meshes, the belt and motor shaft included). To spin the rotor in a simulator, attach `rotor.stl` to one body with a hinge about +Z through the origin and give it `rotor.mass_kg` and `rotor.izz_kg_m2` from the params file, not the mesh's own mass properties (the meshes are not watertight solids of the right density).

## Numbers for a slip model

All from `model/sim-params.json`:

| Quantity | Key | V12.5 value |
| --- | --- | --- |
| Rotor mass | `rotor.mass_kg` | 1.486 kg (includes about 10 g of placeholder frog meshes) |
| Rotor inertia about Z | `rotor.izz_kg_m2` | 0.00635 kg m2 |
| Total inertia at the spindle | `rotor.reflected_inertia_kg_m2.total_at_spindle` | 0.00666 kg m2 (adds the motor rotor and the three rim wheels) |
| Rotor CG offset from the axis | `rotor.cg_offset_from_axis_mm` | 0.11 mm |
| Top disc | `rotor.top_disc` | 186 dia x 3.2 acrylic, top face Z 197.375, bare acrylic inside r 73.5 |
| Frog on the disc (shear test) | `rotor.frog_on_disc` | stands anywhere from r 0 to r 70.5; CAD frog centred at r 42.95 |
| Frog on the outside wall (normal test) | `rotor.frog_on_outside_wall` | belly at r 75.3 on the tube, centre Z 124.175, azimuth 0 deg (+X) |
| Drum tube | `drum` | 150 OD x 145 ID x 140 tall, smooth clear acrylic, Z 54.175 to 194.175 |
| Motor torque vs speed | `motor.torque_curve_24v.points` | 35.8 N cm at 50 rpm down to 16.0 N cm at 1500 rpm |
| Drive ratio | `drive.as_built_v12_5.ratio` | 1:1, so rotor rpm = motor rpm |
| Frog mass | `frog.working_case_mass_g` | 35 g (5 g as the small case) |
| Toe pad area | `frog.total_pad_area_mm2` | about 95 mm2 for a 35 g frog |
| Pad shear strength | `frog.pad_shear_strength_kpa` | 1 to 8 kPa smooth, 30 kPa on 6 um roughness |
| Pad normal (pull-off) strength | `frog.pad_normal_strength_kpa` | 1.2 to 1.7 kPa |
| Targets | `targets` | 400 and 600 rpm, 60 ms slip-to-stop |

The frog numbers are literature values from other species (mostly Litoria caerulea), not measurements of a grey tree frog. Treat them as a range to sweep, not a single answer.

The basic model is a point mass on a spinning disc:

- angular speed: omega = 2 pi rpm / 60 (rad/s)
- outward (centripetal) load: F = m omega^2 r, with r in metres
- in g: a / 9.80665; `targets.centripetal_acceleration.rpm_vs_g` has the table for r = 42.95, 70.5 and 75.3 mm
- disc frog: F is a shear load in the contact plane, gravity adds m g of normal load into the disc. It slips when F / pad area is above the shear strength.
- wall frog: F pulls straight off the tube wall (normal load) and gravity is a shear load along the wall. It lets go when F / pad area is above the normal strength.

Worked example, disc frog: 35 g at r 42.95 mm with 95 mm2 of pad. At 1 kPa the pads hold 0.095 N, so it slips at omega^2 = 0.095 / (0.035 x 0.04295) = 63 rad2/s2, about 76 rpm. At 8 kPa it holds 0.76 N and slips at about 215 rpm. Both are well under the 400 rpm target, which is why the literature range matters for any test plan.

For the stop after a slip: the decel the motor can give is T(rpm) / J, with T from the torque curve and J = `rotor.reflected_inertia_kg_m2.total_at_spindle`. `targets.ramp_profiles_used` has the CAD's own estimate (from 400 rpm: 0.2625 N m usable, 39.4 rad/s2, 1.06 s and 3.5 revolutions to stop), the bench ramp profiles and the controller limits from `motor/motor.toml`. In 60 ms of detection latency at 400 rpm the drum turns 0.4 revolutions (144 deg).

The drive in the exports is 20T:20T with a 100 mm belt. A change to 30T:30T with a 120 mm belt is decided but not in the CAD yet (`drive.decided_not_in_cad`); it keeps the 1:1 ratio, so the speed and torque numbers do not change.

## The camera and synthetic video

The camera is an Arducam UC-261 (OmniVision OV5647 sensor, 2592 x 1944) with an LS-40180 M12 fisheye lens. It sits on the carrier disc **inside the drum and spins with it**, looking up and out through the acrylic top disc. From `camera` in the params file:

- lens front at (0, 0, 99) mm, 48 mm above the carrier disc
- optical axis (0.5, 0, 0.866): tilted 30 deg from vertical toward +X, the frog side
- image right = (0.866, 0, -0.5), image up = (0, -1, 0)
- projection: equidistant fisheye, r = f theta, with f = 765.5 px/rad on the full 2592 x 1944 sensor (236.3 px/rad on the 800 x 600 camsim image)
- FOV 194 deg horizontal, 145.5 deg vertical, image circle 206 deg (corners outside it are dark)
- planned frame rate 30 fps (not yet measured on the rotor)
- lighting: a 16-LED white WS2812B ring, coaxial with the lens, on the same printed bridge

Look at `model/exports/camsim/as_built.png` for the view (and `as_built_labels.png` for which part is which). The other renders in that folder are the alternative tilt and height positions that were compared; `camsim.json` has the numbers for each, and `montage.png` puts them side by side. As built, the camera sees 99 percent of the top disc, nothing on the rotor blocks either frog, and a frog is resolved at about 6.8 px/mm on the disc and 9.8 px/mm on the wall at full sensor resolution. The two drum posts are the only rotor parts in the way of the view of the disc, and they are always in the same place in the image.

What the synthetic video should look like, because the camera co-rotates:

- everything on the rotor (top disc, bezel ring, tube, posts, LED ring reflection, a frog that is holding on) is **still** in the image at any speed
- everything fixed (the cage, its mesh panels, the frame, the room) is seen through the clear acrylic and **sweeps around the image** once per revolution, so at 400 rpm the background turns 6.7 times a second
- a slip is the frog moving relative to the still disc and bezel: radially outward on the disc, or away from the wall
- 1 mm of slip on the disc is about 7 px at full resolution; at 30 fps and 400 rpm the background moves 80 deg between frames, so a frame-difference detector has to mask or ignore the background seen through the acrylic

Refraction through the acrylic is ignored in camsim (straight rays), so the real image will be a little distorted where it looks through the 2.5 mm tube wall.

To render your own frames from the GLB, put a fisheye camera at `camera.lens_position_mm` looking along `camera.optical_axis_unit` with `camera.image_up_unit` as up, rotate every node in `rotor.rotates_glb_nodes` and the camera together about Z, and leave the rest still. `model/cad/lib/camsim.py` is the ray caster that made the camsim images and can be used as a reference.

## Rebuilding the CAD from model/cad

You only need this to change the geometry. The exports are already built.

Requirements: **Python 3.12** (exactly; the CAD kernel wheels pin it), and about 2 GB of disk for the venv and build output. Tested on macOS on Apple silicon. Linux should work with the same pinned wheels, but rendering the PNGs needs an OpenGL context, so a headless Linux box may need `xvfb-run`.

```sh
cd model/cad
python3.12 -m venv .venv                     # or: uv venv --python 3.12 .venv
.venv/bin/pip install -r requirements.txt    # or: uv pip install --python .venv/bin/python -r requirements.txt
bin/cad build froggy_v12_5                   # build, check, export, render, design check
.venv/bin/python scripts/export_step_light.py froggy_v12_5   # the under-100 MB STEP
bin/cad camsim froggy_v12_5                  # camera simulation renders
```

`bin/cad` uses `model/cad/.venv/bin/python`; set `FROGGY_PYTHON` to use a different interpreter. Output goes to `model/cad/out/froggy_v12_5/` (ignored by Git): the GLB, a full 125 MB STEP, STL, 3MF, sheet.json (every part with its bounding box, volume, material and the build notes), about 30 PNG renders and `assembly.pkl`. `bin/cad build` prints one JSON object on stdout and progress on stderr. `.venv/bin/python lib/mass.py out/froggy_v12_5/sheet.json` gives the mass breakdown.

Measured on 2026-11-04 from a fresh copy of `model/cad` (this repo's files only) on an Apple M4 Pro:

| Step | Command | Result |
| --- | --- | --- |
| Install | `uv pip install -r requirements.txt` into a new Python 3.12.14 venv | exit 0 in under 1 s from uv's local cache (a cold install downloads several hundred MB of wheels and takes a few minutes) |
| Build | `bin/cad build froggy_v12_5` | exit 0 in 581 s: geometry 135 s, interference and joint checks 80 s, export 11 s, renders 342 s, design check 1 s; 477 parts, 153 joints, 0 interference pairs, GLB 21.5 MB. That run shared the CPU with a camsim job; an earlier run of the same commit on its own took 305 s |
| Light STEP | `scripts/export_step_light.py froggy_v12_5` | exit 0 in 4 s, 51.6 MB, reads back as 668 valid solids |
| Viewer | `bin/view --no-open` | HTTP 200 within 1 s; `/status.json` reports `froggy_v12_5`, PROD V12.5, build_ok true (tested with the port set to 7900 because 7894 was busy on that machine) |

Expect about 5 to 10 minutes for a full build on a fast laptop and longer on an older one. Most of it is the interference and spin checks and the renders, not the geometry.

## The web viewer

```sh
cd model/cad
bin/view --no-open
```

Then open http://127.0.0.1:7894/ in a browser (the port is set in `model/cad/viewer.json`; drop `--no-open` to have it open a tab for you). It serves the last build in `out/froggy_v12_5/`, so run `bin/cad build froggy_v12_5` once first; without a build it exits with "no ... glb; run bin/cad build froggy_v12_5 first". The page has the part tree, the fixed review cameras, a camera-view button that shows the fisheye view from the rotor camera, Z and X section cuts and an exploded view. Stop it with Ctrl-C. `bin/cad view froggy_v12_5 --no-open --port 7891` does the same on another port.

## Where the numbers come from

- CAD values (`rotor`, `drum`, camera pose, drive geometry, mass) were read from the V12.5 build of froggy-cad commit 357616d, the same build as the exports.
- Camera optics come from `camsim.json` and the Arducam product page.
- The motor torque curve is the STEPPERONLINE product page data table (CL42T driver, 24 V, 2 A, typical values). The production driver is a CL57T-V41.
- Frog values are literature: Hanna and Barnes 1991, Federle et al. 2006, Endlein et al. 2013, Crawford et al. 2016 (full references in `frog.sources`). The 2 to 10 cm body-length range and the grey tree frog come from the sponsors.
- The 400 and 600 rpm targets and the 60 ms slip-to-stop latency are team requirements.
