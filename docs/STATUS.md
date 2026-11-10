# Status and next work

## Recovered work

The existing controller implements pigpio hardware PWM, acceleration/deceleration profiles, direction, software stop and e-stop input handling, one Hall index input, CSV logging, a camera preview, and an HTTP control page. Tests use a fake GPIO backend and loopback HTTP servers.

Historical bench report, October 21: DM320T and 17HS08 ran smoothly unloaded on 12 V through the reported 300 RPM test. Earlier vibration was traced to a loose motor wire; the documented black/green and red/blue coil pairs were restored. The 24 V runs occurred during the loose-wire period and require a new test. The CSVs preserve commanded and available measured fields; they are not proof of encoder operation or loaded torque.

The Pi 3 had joined MWireless and Tailscale. The original Pi Zero W used the 32-bit image path but did not join Wi-Fi; the recorded scan saw no networks. A later one-off Wi-Fi diagnostic hook was described in the historical session but is not present among the recovered live files. Do not describe Zero connectivity as working based on card provisioning alone. Camera latency tooling exists; live Zero latency remains unverified.

## CL57T-V41 closed loop, November 9, 2026

The bench moved from the open-loop DM320T to a STEPPERONLINE CL57T-V41 closed-loop driver with the 17HS19-2004D-E1K motor (1000-line encoder) on a 24 V supply. The driver closes the loop itself. The Pi reads two things back: the ALM fault output on GPIO 22 through a 10 k / 15 k divider, and a passive tap on the encoder's EA+ line on GPIO 5, which gives real shaft RPM and following error. ENA on GPIO 27 enables the driver only during a run, and an active ALM refuses starts and stops a run. Wiring, switch settings and manual page references are in `motor/WIRING-CL57T.md`; the bench procedure is `motor/TEST-PLAN-CL57T.md`.

Software added with it: `froggy-motor cl-test` (ramp, hold, stop, 100 ms log of ALM and encoder RPM, one verdict line), a per-trial store on the Pi (one gzip record per actuation, rest to rest, capped by `trials_keep` and `trials_max_mb`) with `froggy-motor trials list/export/prune`, and a new HMI page in `motor/ui/` served by `froggy-motor ui` with live RPM chart, closed-loop tile and CSV and PDF export of any trial. The page and API are covered by headless browser tests.

Still open: the ALM active level has only been seen in its normal state, since no real fault has happened yet; motor.toml records the encoder tap as a 10 k series resistor into GPIO 5, while WIRING-CL57T.md recommends adding a 74LVC1G17 buffer; no loaded drum run, stopping test or real e-stop circuit has been done on the CL57T. The old DM320T notes (`motor/WIRING.md`, `motor/TEST-PLAN.md`, `motor/schematic.png`) are kept as history.

## Due Monday, November 2, 2026

| Owner | Task |
| --- | --- |
| Sawyer | Order required parts. Continue CAD work. |
| Daniel | Cut and build the frame. |
| Li and Jun | Get motor wiring fully working, capture the onboard encoder, and document the wiring and capture setup. |
| Li | Wire and verify the Hall effect sensor. |
| Jun | Establish main Raspberry Pi to Pi W communication over Wi-Fi. |

For the encoder task, confirm the exact motor/encoder output interface and voltage before assigning pins or choosing an input circuit. Keep encoder counts separate from Hall index pulses in logs and the UI. For the Pi link, first establish Zero Wi-Fi connectivity, then define and test the command/video transport and behavior when the link drops.

V10 CAD retains the 54 mm REX spindle, pillow block and Hyper Hub, with equal 20-tooth pulleys and a nominal 100 mm belt. The imported controller settings predate this transmission. Loaded acceleration, stopping, belt capacity and the real stop circuit still need commissioning.

## Validation on import

Local checks cover Python controller tests, shell syntax, offline package checksums, path portability and a credential-pattern scan. No motor movement, Pi deployment, network reconfiguration, card flashing or live camera session is performed as part of repository creation. See the repository's initial commit and CI for the reproducible checks.
