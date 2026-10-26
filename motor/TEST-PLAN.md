# First powered motor test (bare motor on the bench)

Everything runs from the Mac. Plug the Pi into the Mac with the Ethernet cable (no hotspot or internet needed), then use one of:

- `froggy-motor ui`: opens the control page (camera, target RPM, Ramp, Stop, big red STOP, live commanded vs measured RPM chart). Every ramp started there is logged to its own CSV.
  Opening or reloading the page never moves the motor. A ramp only starts from a click on Ramp, and the first one after each page load asks for a second click ("Yes, spin the motor"). Every request the page makes is logged with its time in `~/froggy-motor/ui.log` on the Pi.
- or the terminal: `froggy-motor ramp --rpm 60 --accel-s 3 --hold-s 5 --decel-s 3`

The camera can only feed one viewer program at a time, so close any `froggy-cam-live` window first (`froggy-cam-live --stop`), otherwise the page shows "camera offline". Closing the terminal running `froggy-motor ui` (or Ctrl-C) stops the server on the Pi, and the motor ramps to rest first. Only one thing can drive the motor at a time: while the page is open, a terminal `froggy-motor ramp` refuses with "the motor is in use", and the other way round.

The first command deploys the code and installs pigpio on the Pi by itself. The Pi's address is found automatically: froggy-pi.local over the cable, then `FROGGY_PI`, then the saved host.

Offline install path: the pigpio .debs for Raspberry Pi OS bookworm arm64 are vendored in `debs/` and installed with `dpkg -i` when apt cannot reach the internet. I picked this over the sysfs PWM route because it keeps the exact pigpio hardware-PWM code the Mac tests run against, and it needs no config.txt edit or reboot. apt is still tried first when the Pi happens to have internet.

## 0. Before 24 V is on

Pins as wired 2026-10-21 (two 2N7000s, see WIRING.md): PUL on GPIO 18 (physical pin 12), DIR on GPIO 27 (pin 13), ENA not connected, e-stop GPIO 25 (pin 22) to GND (pin 20), hall on GPIO 17 (pin 11). `froggy-motor status` should print `ENA: not wired (driver always enabled)`.

1. DIP switches on the DM320T (see WIRING.md):
   - 17HS08-1004 (1.0 A): SW1 ON, SW2 ON, SW3 OFF (1.3 A peak)
   - 17HS19-2004 (2.0 A): SW1 OFF, SW2 OFF, SW3 OFF (2.2 A peak)
   - both: SW4 OFF, SW5 OFF, SW6 ON (1/16 step, 3200 pulses/rev)
2. Motor wires: black A+, green A-, red B+, blue B- (the sheet colours, verified on the bench 17HS08). If it whines without turning, reseat the wires first, since a loose wire looks like a crossed pair, then meter: about 3.7 ohm across a pair, open across coils. Never plug or unplug the motor or power connector with 24 V on.
3. Put a strip of tape on the shaft as a flag so the rotation is easy to see. Put the hall magnet on whatever spins, facing the sensor at 2 to 5 mm.
4. E-stop released (twisted out).

## 1. Pi only (no 24 V yet)

Open `froggy-motor ui`. The e-stop tile should read "closed". Press the mushroom: it turns red "OPEN" within a fraction of a second. Twist to release. (Terminal version: `froggy-motor estop-test`.)

## 2. 24 V on

The DM320T green LED comes on and the red one stays off. The shaft is now held, and stays held whenever 24 V is on, because ENA is not wired.

## 3. No-load spin at 60 RPM

Ramp to 60 RPM, accel 3 s, hold 5 s, decel 3 s (the page defaults, or the command above). Expect a smooth, quiet turn at 1 rev/s, then a stop. Check:
- the measured RPM tile reads about 60 once the magnet has passed twice
- the driver and motor are only warm, and the red LED never blinks

If it turns the wrong way, set `dir_cw_level = 1` in motor.toml.

## 4. E-stop during a run

Ramp 60 RPM with hold 20 s. Halfway through, press the e-stop. The motor should stop within 0.5 s and the phase goes to "estop". With ENA not wired the shaft stays held after the stop (it would only go free with ENA wired). Release the button, and the next Ramp works again.

## 5. Index count

Terminal: `froggy-motor index-test --s 10 --rpm 60`. Expect about 10 pulses and "measured 60.0 rpm vs 60.0 commanded" within 1 percent. If it reports 0 pulses, flip the magnet (the A3144 only sees a south pole).

## 6. 200 and 400 RPM ramps

- 200 RPM: accel 2 s, hold 10 s, decel 2 s
- 400 RPM: accel 3 s, hold 10 s, decel 3 s

Ramps faster than 200 RPM/s get stretched automatically to protect the torque budget. Listen for any resonance speeds on the way up and note them.

## Bench log 2026-10-21

11:50 to 12:00, bare 17HS08-1004 on the 12 V bench supply, wired as in WIRING.md (two 2N7000s, PUL pin 12, DIR pin 13, OPTO 5 V pin 2, ENA open, e-stop jumpered pin 22 to pin 20, DIP SW1 ON SW2 ON SW3 OFF SW4 OFF SW5 OFF SW6 ON).
- First try used the sheet colours (black A+, green A-, red B+, blue B-). It only vibrated and whined at 10, 60 and 120 RPM.
- Swapped green and blue and it ran. (False lead, see 12:30: the swap just reseated a bad contact.)
- Smooth at 60 and 200 RPM in the hold.
- Some jiggle only while accelerating and decelerating below about 100 RPM, clean in the middle. That is the bare motor's unloaded mid-band resonance and should go away once the coupling and drum inertia are on.
- 12:30 correction: swapped green and blue back to the sheet colours (A+ black, A- green, B+ red, B- blue). Perfectly smooth the whole way, best run yet.
- So the sheet was right and the first whine was a loose or badly seated wire. Next time it whines, reseat and meter the pairs before swapping anything.
- Still to do: 400 RPM and the direction check (clockwise seen from the shaft end). Nothing has been loaded yet.

## What gets logged

Every run writes a CSV on the Pi with t_s, rpm_cmd, step_hz, rpm_measured, estop and phase every 10 ms, and prints a one-line summary (peak RPM, total steps, e-stop events, measured vs commanded RPM at hold). `froggy-motor --pull-logs` copies them to `logs/` here. Also jot down which motor and current setting were used, how warm the motor and driver got, and any noisy speeds.

## Before the drum goes on

Set `emergency_decel_s = 2.0` in motor.toml (3.0 if you plan to go to 600 RPM). The 0.5 s default is only for the bare motor; with the drum on, a 0.5 s stop from 400 RPM needs about three times the torque the 17HS08 has (see WIRING.md).
