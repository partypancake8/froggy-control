# Froggy Control

Control and bench-test software for the ROB450 Froggy Turntable. This repository collects the October work on the main Raspberry Pi 3 B+, the camera Pi Zero W, Hall index sensor, camera streaming and Pi provisioning, plus the November 9 move from the open-loop DM320T to a STEPPERONLINE CL57T-V41 closed-loop driver.

The motor runs closed loop on the CL57T-V41 with a 17HS19-2004D-E1K motor on a 24 V supply. The Pi reads the driver's ALM fault output on GPIO 22 and taps the motor encoder's EA+ line on GPIO 5, so measured RPM now comes from the encoder. The first bare-motor closed-loop runs were made on November 9. **Loaded runs and direct Pi-to-Pi communication are still to be implemented and tested.**

| Location | Contents |
| --- | --- |
| `motor/froggy_motor.py` | PWM step generation, ramps, direction, stop/e-stop handling, CL57T ALM and encoder tap, closed-loop test, per-trial store, CSV logging and the HTTP API for the control page |
| `motor/motor.toml` | November 9 CL57T-V41 bench configuration (pin map, 1/16 microstep, encoder 1000 PPR, ALM, trial limits) |
| `motor/ui/` | Control (HMI) page served by `froggy-motor ui`: start/stop, live RPM chart, closed-loop tile, trial list with CSV and PDF export; vendored uPlot, jsPDF and Tabler icons |
| `motor/WIRING-CL57T.md` | CL57T-V41 wiring, switch settings, ALM divider, encoder tap and manual page references |
| `motor/TEST-PLAN-CL57T.md` | Closed-loop bench procedure and dated observations |
| `motor/schematic-cl57t.html`, `.png` | CL57T wiring drawing and its render; image sources in `schematic-img/sources.txt` |
| `motor/test_motor.py`, `motor/ui/test_ui.py` | Hardware-free controller and API tests using fake pigpio, and headless browser tests of the page (skipped without Playwright) |
| `motor/WIRING.md`, `motor/TEST-PLAN.md`, `motor/schematic.png` | October DM320T open-loop setup, kept as history |
| `motor/debs/` | SHA256-checked offline pigpio packages for the main Pi's Bookworm arm64 OS |
| `motor/logs/` | Four original October 21 ramp CSVs |
| `bin/froggy-motor` | Mac SSH/deployment wrapper, UI tunnel, closed-loop test, trial list/export and log collection |
| `bin/froggy-pi-flash` | Mac SD-card provisioning for Pi 3 B+ and original Pi Zero W |
| `bin/froggy-pi-net` | Campus network profile, phone-home discovery and Tailscale setup |
| `bin/froggy-cam-live` | Pi camera H.264 live stream to Mac over UDP or TCP |
| `bin/froggy-cam-latency` | QR-clock camera latency tool and synthetic self-test |
| `bin/froggy-motor-song` | Unloaded motor tone experiment using generic scales |
| `docs/STATUS.md` | What worked, what remains, and team assignments |
| `docs/import-manifest.json` | Sanitized source labels and historical import hashes |

## Local simulation

Python 3.11 or later is sufficient for controller simulation. No Pi or GPIO dependency is needed for these commands:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-dev.txt
python -m pytest -q motor
python motor/froggy_motor.py --dry-run --sim-fast ramp --rpm 60 --accel-s 3 --hold-s 2 --decel-s 3
python motor/froggy_motor.py --dry-run ui --no-open
```

The simulated UI is at http://127.0.0.1:7895. Simulation validates software behavior, not the wiring or loaded machine. The browser tests in `motor/ui/test_ui.py` need `pip install playwright` and `playwright install chromium`; without them they are skipped.

## Main Pi and motor

Set `FROGGY_PI_USER` to your Pi login; the default for new installations is `froggy`. Existing cards keep their existing accounts. The shell wrappers target macOS with Bash, Zsh, SSH and `nc`. Add this checkout's `bin/` to PATH:

```sh
export PATH="$PWD/bin:$PATH"
export FROGGY_PI=froggy-pi
export FROGGY_PI_USER=your_pi_login
froggy-motor --dry-run status
```

`FROGGY_PI` selects the main Pi. Without it, the motor wrapper tries `froggy-pi.local`, the address saved by `froggy-pi-net --listen`. Live `froggy-motor` commands deploy the checkout's `motor/` directory before running, unless `FROGGY_SKIP_DEPLOY` is set. They can change the Pi and drive hardware. Read [WIRING-CL57T.md](motor/WIRING-CL57T.md) and [TEST-PLAN-CL57T.md](motor/TEST-PLAN-CL57T.md) before bench use.

The committed `motor.toml` is the November 9 **CL57T-V41 bare-motor** configuration: 24 V supply, GPIO 18 PUL, GPIO 17 DIR, GPIO 27 ENA (low enables the driver), GPIO 25 e-stop, ALM on GPIO 22 through a 10 k / 15 k divider, encoder EA+ tap on GPIO 5, 1/16 microstep, `enc_ppr = 1000`, no Hall sensor and no camera on the bench Pi. An active ALM refuses starts and stops a run (`alm_blocks_start`). For the old open-loop bench set `[driver] model = "DM320T"`, see [WIRING.md](motor/WIRING.md). It is not yet a commissioned configuration for the loaded belt-driven assembly. The bench still uses an e-stop jumper; a passed simulation does not establish a functioning physical stop circuit.

`froggy-motor ui` starts the control page on the Pi and tunnels it to the laptop. `froggy-motor cl-test --rpm R` ramps, holds and stops while logging ALM and encoder RPM every 100 ms, and prints a verdict. Every actuation, rest to rest, is saved on the Pi as one `trials/<id>.json.gz` record; `froggy-motor trials list`, `trials export <id> --csv PATH` and the page's CSV and PDF buttons get it back out. `trials_keep` and `trials_max_mb` cap the store.

## Camera Pi and networking

The original Zero W uses the flash script's `--zero` armhf path and hostname `froggy-zero`. The main Pi uses the arm64 path. Do not install `motor/debs/` on the Zero: the bundled binaries are arm64. Network provisioning is implemented, but the last recorded Zero boot could not see Wi-Fi networks. No end-to-end Pi 3 to Zero command/video protocol has been completed.

The Mac camera tools need FFmpeg (`ffplay` included); the latency tool also needs `requirements-camera.txt`. Select the camera Pi with `FROGGY_PI` or the latency tool's `--host`. Existing streaming goes between a Pi and the Mac. Connecting it to the main Pi is Jun's remaining task.

Provisioning scripts write or erase SD cards and may install services. They are retained for deliberate setup, not executed by tests. `froggy-pi-flash` now requires an explicit target disk. It reads your public SSH key from `~/.ssh/id_ed25519.pub` or `FROGGY_SSH_PUBLIC_KEY`; new accounts have password login locked unless `FROGGY_PASSWORD_HASH` is supplied. Wi-Fi credentials are prompted for or supplied at runtime. Generated profiles, keys, images and staging folders do not belong in Git.

Run/cache files use `~/.cache/froggy` (`FROGGY_RUN_DIR` override). Camera test results use this checkout's ignored `testing/` directory (`FROGGY_TESTING_DIR` override). Provisioning is Mac-specific. Campus credentials must be supplied at runtime. The main-Pi phone-home target is optional and uses `FROGGY_MAC_HOST`; without it the phone-home script sends nothing.

## Import scope

Imported from `ROB450/Frog/software/motor` and the six `~/bin/froggy-*` tools on October 26, 2026; the CL57T closed-loop work was ported on November 10, 2026. Original files remain in place. Motor control logic, wiring documents and bench evidence were preserved. Import edits make Mac paths relative/configurable, prefer the explicit motor-host override, remove an embedded historical login hash, require an explicit flash target and avoid printing generated credential-bearing setup content. See [STATUS.md](docs/STATUS.md) for limitations.

## Model and simulation assets

The V12.5 turntable CAD lives in [`model/`](model/README.md): the headless build123d source and viewer (`model/cad/`), ready-made GLB, STEP and STL exports with camera-simulation renders (`model/exports/`), and one machine-readable parameter file for rotor, drum, camera, motor, drive, frog and target numbers (`model/sim-params.json`). [docs/SIMULATION.md](docs/SIMULATION.md) explains how to rebuild the model, run the viewer, load the GLB into a simulator, and which numbers feed a slip model and synthetic camera video.

## Public distribution

See [PUBLIC-REVIEW.md](docs/PUBLIC-REVIEW.md) for the publication review and [THIRD-PARTY.md](THIRD-PARTY.md) for bundled package notices and links to the vendor manuals the wiring notes cite. This repository is publicly readable; no additional license for the project-authored code is granted by publication alone.
