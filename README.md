# Froggy Control

Control and bench-test software for the ROB450 Froggy Turntable. This repository collects the existing October 7 and October 21 work on the main Raspberry Pi 3 B+, the camera Pi Zero W, DM320T motor driver, Hall index sensor, camera streaming, and Pi provisioning.

The motor controller and Mac tools already exist. **Motor onboard encoder capture and direct Pi-to-Pi communication are still to be implemented and tested.** The controller's measured RPM currently comes from the Hall index, not the motor encoder.

| Location | Contents |
| --- | --- |
| `motor/froggy_motor.py` | PWM step generation, ramps, direction, stop/e-stop handling, Hall RPM, CSV logging, web UI and local camera preview |
| `motor/motor.toml` | October 21 bare-motor bench configuration |
| `motor/WIRING.md` | Driver, transistor interface, GPIO, DIP switches and wiring notes |
| `motor/TEST-PLAN.md` | Existing bench procedure and dated observations |
| `motor/test_motor.py` | Hardware-free controller and HTTP API tests using fake pigpio |
| `motor/schematic.png` | Existing motor wiring schematic |
| `motor/debs/` | SHA256-checked offline pigpio packages for the main Pi's Bookworm arm64 OS |
| `motor/logs/` | Four original October 21 ramp CSVs |
| `bin/froggy-motor` | Mac SSH/deployment wrapper, UI tunnel and log collection |
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
python -m pytest -q motor/test_motor.py
python motor/froggy_motor.py --dry-run --sim-fast ramp --rpm 60 --accel-s 3 --hold-s 2 --decel-s 3
python motor/froggy_motor.py --dry-run ui --no-open
```

The simulated UI is at http://127.0.0.1:7895. Simulation validates software behavior, not the wiring or loaded machine.

## Main Pi and motor

Set `FROGGY_PI_USER` to your Pi login; the default for new installations is `froggy`. Existing cards keep their existing accounts. The shell wrappers target macOS with Bash, Zsh, SSH and `nc`. Add this checkout's `bin/` to PATH:

```sh
export PATH="$PWD/bin:$PATH"
export FROGGY_PI=froggy-pi
export FROGGY_PI_USER=your_pi_login
froggy-motor --dry-run status
```

`FROGGY_PI` selects the main Pi. Without it, the motor wrapper tries `froggy-pi.local`, the address saved by `froggy-pi-net --listen`. Live `froggy-motor` commands deploy the checkout's `motor/` directory before running, unless `FROGGY_SKIP_DEPLOY` is set. They can change the Pi and drive hardware. Read [WIRING.md](motor/WIRING.md) and [TEST-PLAN.md](motor/TEST-PLAN.md) before bench use.

The committed `motor.toml` is the original **17HS08 bare-motor** configuration: GPIO 18 PWM, GPIO 27 direction, ENA unwired, GPIO 25 e-stop, GPIO 17 Hall index. It is not a commissioned configuration for the V10 belt-driven assembly or its encoder-equipped motor. The original test setup used an e-stop jumper; a passed simulation does not establish a functioning physical stop circuit.

## Camera Pi and networking

The original Zero W uses the flash script's `--zero` armhf path and hostname `froggy-zero`. The main Pi uses the arm64 path. Do not install `motor/debs/` on the Zero: the bundled binaries are arm64. Network provisioning is implemented, but the last recorded Zero boot could not see Wi-Fi networks. No end-to-end Pi 3 to Zero command/video protocol has been completed.

The Mac camera tools need FFmpeg (`ffplay` included); the latency tool also needs `requirements-camera.txt`. Select the camera Pi with `FROGGY_PI` or the latency tool's `--host`. Existing streaming goes between a Pi and the Mac. Connecting it to the main Pi is Jun's remaining task.

Provisioning scripts write or erase SD cards and may install services. They are retained for deliberate setup, not executed by tests. `froggy-pi-flash` now requires an explicit target disk. It reads your public SSH key from `~/.ssh/id_ed25519.pub` or `FROGGY_SSH_PUBLIC_KEY`; new accounts have password login locked unless `FROGGY_PASSWORD_HASH` is supplied. Wi-Fi credentials are prompted for or supplied at runtime. Generated profiles, keys, images and staging folders do not belong in Git.

Run/cache files use `~/.cache/froggy` (`FROGGY_RUN_DIR` override). Camera test results use this checkout's ignored `testing/` directory (`FROGGY_TESTING_DIR` override). Provisioning is Mac-specific. Campus credentials must be supplied at runtime. The main-Pi phone-home target is optional and uses `FROGGY_MAC_HOST`; without it the phone-home script sends nothing.

## Import scope

Imported from `ROB450/Frog/software/motor` and the six `~/bin/froggy-*` tools on October 26, 2026. Original files remain in place. Motor control logic, wiring documents and bench evidence were preserved. Import edits make Mac paths relative/configurable, prefer the explicit motor-host override, remove an embedded historical login hash, require an explicit flash target and avoid printing generated credential-bearing setup content. See [STATUS.md](docs/STATUS.md) for limitations.

## Public distribution

See [PUBLIC-REVIEW.md](docs/PUBLIC-REVIEW.md) for the publication review and [THIRD-PARTY.md](THIRD-PARTY.md) for bundled package notices. This repository is publicly readable; no additional license for the project-authored code is granted by publication alone.
