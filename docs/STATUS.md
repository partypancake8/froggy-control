# Status and next work

## Recovered work

The existing controller implements pigpio hardware PWM, acceleration/deceleration profiles, direction, software stop and e-stop input handling, one Hall index input, CSV logging, a camera preview, and an HTTP control page. Tests use a fake GPIO backend and loopback HTTP servers. The RPM feedback is a once-per-revolution Hall measurement. There is no A/B quadrature or onboard motor encoder capture in the recovered code.

Historical bench report, October 21: DM320T and 17HS08 ran smoothly unloaded on 12 V through the reported 300 RPM test. Earlier vibration was traced to a loose motor wire; the documented black/green and red/blue coil pairs were restored. The 24 V runs occurred during the loose-wire period and require a new test. The CSVs preserve commanded and available measured fields; they are not proof of encoder operation or loaded torque.

The Pi 3 had joined MWireless and Tailscale. The original Pi Zero W used the 32-bit image path but did not join Wi-Fi; the recorded scan saw no networks. A later one-off Wi-Fi diagnostic hook was described in the historical session but is not present among the recovered live files. Do not describe Zero connectivity as working based on card provisioning alone. Camera latency tooling exists; live Zero latency remains unverified.

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
