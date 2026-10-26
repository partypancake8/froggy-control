# Froggy motor wiring: Pi 3 B+ to DM320T, NEMA 17, 24 V, e-stop, hall index

## Sources

- DM320T datasheet: https://www.stepperonline.co.uk/download/DM320T.pdf (STEPPERONLINE "DM320T Full Datasheet", 13 pages, image-only PDF made 2022-03-31, sha256 774833fcd27ee9c5327d4284d3f479fd6f1be5fc357441aaeedff3456870ff8c). The same file sits at www.omc-stepperonline.com/download/DM320T.pdf, but that host refused scripted downloads (403). Every number below was read off the sheet's own pages.
- 17HS08-1004S drawing: https://www.stepperonline.co.uk/download/17HS08-1004S.pdf
- 17HS19-2004S1 drawing: https://www.stepperonline.co.uk/download/17HS19-2004S1.pdf

## DM320T facts (from the datasheet)

| Item | Value | Where on the sheet |
|---|---|---|
| Supply voltage | 10 to 30 VDC, 24 V typical and suggested | 2.1 table, 6.3 |
| Over-voltage protection | trips above 34 VDC (red LED 2 blinks, repower to reset) | 11 |
| Output current | 0.3 to 2.2 A peak (1.6 A RMS max), 8 steps on SW1 to SW3 | 2.1, 7.2 |
| Standstill current | drops to 50 percent 0.4 s after the last pulse | 7.2.2 |
| Logic signal current | 7 min, 10 typical, 16 max mA | 2.1 |
| Pulse input frequency | 0 to 60 kHz | 2.1 |
| Pulse width | high and low each at least 7.5 us | 2.1, 10 |
| DIR setup | DIR at least 5 us ahead of the PUL edge (the 2.1 table says 7.5 us) | 3.1, 10 |
| ENA setup | ENA at least 5 us ahead of DIR | 10 |
| Signal levels | PUL high 4 to 5 V, low 0 to 0.5 V; timing chart: high above 3.5 V, low below 0.5 V | 3.1, 10 |
| Input stage | common anode: OPTO is the +5 V opto supply, each input has a 270 ohm resistor inside; add 1 k for 12 V or 2 k for 24 V logic, R = 0 at 5 V | 3.1, 4 |
| ENA polarity | ENA terminal high (+5 V, NPN signal off) or unconnected = ENABLED; ENA pulled low = DISABLED | 3.1 |

Note: the PDF title says "18-30VDC" but its spec table and text both say 10 to 30 VDC. Our LRS-50-24 is 24 V, which is inside both.

### DIP switches (exact from the sheet)

SW1 to SW3 set the dynamic (peak) current, SW4 to SW6 set the microstep.

| Peak | RMS | SW1 | SW2 | SW3 |
|---|---|---|---|---|
| 0.3 A | 0.21 A | ON | ON | ON |
| 0.5 A | 0.35 A | OFF | ON | ON |
| 0.7 A | 0.49 A | ON | OFF | ON |
| 1.0 A | 0.71 A | OFF | OFF | ON |
| 1.3 A | 0.92 A | ON | ON | OFF |
| 1.6 A | 1.13 A | OFF | ON | OFF |
| 1.9 A | 1.34 A | ON | OFF | OFF |
| 2.2 A | 1.56 A | OFF | OFF | OFF |

| Microstep | Pulses/rev (1.8 deg) | SW4 | SW5 | SW6 |
|---|---|---|---|---|
| 2 | 400 | ON | ON | ON |
| 4 | 800 | OFF | ON | ON |
| 8 | 1600 | ON | OFF | ON |
| 16 | 3200 | OFF | OFF | ON |
| 32 | 6400 | ON | ON | OFF |
| 64 | 12800 | OFF | ON | OFF |
| 20 | 4000 | ON | OFF | OFF |
| 40 | 8000 | OFF | OFF | OFF |

## Microstep choice: 1/16 (3200 pulses/rev)

step_hz = rpm / 60 x 200 x microstep. At 1/8 the 400 RPM case is 10.7 kHz. We want the 600 RPM top speed under 50 kHz, which leaves margin under the sheet's 60 kHz limit.

| Microstep | 400 RPM | 600 RPM | fits under 50 kHz? |
|---|---|---|---|
| 8 | 10.7 kHz | 16.0 kHz | yes |
| 16 | 21.3 kHz | **32.0 kHz** | yes |
| 20 | 26.7 kHz | 40.0 kHz | yes, less margin |
| 32 | 42.7 kHz | 64.0 kHz | no, over the 60 kHz spec |

1/16 is the finest standard setting that fits. At 32 kHz and 50 percent duty each high and low half is 15.6 us, twice the 7.5 us minimum. Finer steps also run smoother through the low speed resonance band. **Set SW4 OFF, SW5 OFF, SW6 ON** for either motor.

## Current setting for each motor

| Motor | Rated | Coil | Holding torque | Rotor inertia | DM320T setting | SW1 | SW2 | SW3 |
|---|---|---|---|---|---|---|---|---|
| 17HS08-1004S | 1.0 A/phase | 3.7 ohm, 4.5 mH | 0.16 Nm | 22 g cm^2 | 1.3 A peak / 0.92 A RMS | ON | ON | OFF |
| 17HS19-2004S1 | 2.0 A/phase | 1.4 ohm, 3.0 mH | 0.59 Nm | 82 g cm^2 | 2.2 A peak / 1.56 A RMS | OFF | OFF | OFF |

For the 1.0 A motor, 1.3 A peak is the closest setting whose RMS (0.92 A) stays under the rating. For the 2.0 A motor the driver tops out at 2.2 A peak, so it runs a little under its rating, which is fine. Set `motor = ` in motor.toml to match; the code only uses it for the status line.

## The 3.3 V question

The sheet only talks about 5, 12 and 24 V logic. It never mentions 3.3 V. What it does say: the inputs are common anode with a 270 ohm resistor inside, they want 7 to 16 mA, a "high" is above 3.5 V and a "low" is below 0.5 V.

- Pi GPIO straight in with OPTO on the Pi's 3.3 V: the GPIO sinks (3.3 - 1.2 LED - 0.2) / 270 = about 7 mA. That is exactly the 7 mA minimum, no margin. Rejected.
- Pi GPIO straight in with OPTO on 5 V: a GPIO "high" is only 3.3 V, below the sheet's 3.5 V high level, so about (5 - 3.3 - 1.2) / 270 = 1.9 mA still flows and the opto never fully turns off. It also pushes 5 V current back into a 3.3 V pin. Rejected.
- **Chosen: the sheet's Figure 2 open-collector wiring with one NPN per signal.** OPTO goes to the Pi's 5 V pin. Each of PUL, DIR and ENA goes to the collector of a small NPN (2N3904, PN2222 or BC547), emitter to Pi ground, base through 1 k from the GPIO. Opto current is (5 - 1.2 - 0.1) / 270 = about 13.7 mA, inside 7 to 16 mA. The low level is the transistor's saturation voltage (about 0.1 V), well under 0.5 V, and the GPIO only supplies about 2.6 mA of base current. R = 0 because the logic supply is 5 V, as the sheet says.

A 74AHCT125 buffer powered from 5 V would also work (non-inverting), but three NPNs and three resistors are in every kit.

The NPN stage inverts each signal. GPIO high means transistor on, terminal low, opto on.

- PUL: does not matter, it is a 50 percent square wave.
- DIR: just picks the sense. `dir_cw_level` in motor.toml flips it if "cw" turns the drum the wrong way.
- ENA: GPIO high pulls ENA low, which DISABLES the driver. GPIO low (and the Pi's boot default, GPIO 24 pulled down) leaves ENA high, which ENABLES it. So `ena_enable_level = 0`. The driver will hold the rotor at 50 percent current whenever 24 V is on and nothing drives GPIO 24 high. The software disables it at the end of every run, on Stop, on e-stop and on Ctrl-C.

The code waits 200 ms after enabling before the first pulse and only changes DIR at rest, far more than the 5 us setup times.

## As wired 2026-10-21 (Sawyer's bench): two 2N7000 MOSFETs, ENA not wired

Schematic of this wiring: schematic.png in this folder, live page https://claude.ai/artifact/1qiwGVE5m61tSuMWfVLkjb

This is what is on the bench and what motor.toml ships with. It is the same low-side idea as the NPN version below: each 2N7000 drain pulls a DM320T input low, which turns that opto on. So GPIO high = opto on, the same sense as the NPN stage, and `dir_cw_level` keeps its meaning.

| Signal | Pi | Through | To | Notes |
|---|---|---|---|---|
| PUL | GPIO 18 (pin 12) | 220 ohm to 2N7000 gate, 10 k gate to GND, drain to terminal | DM320T PUL | moved from pin 11 (GPIO 17): hardware PWM only exists on GPIO 12, 13, 18, 19 |
| DIR | GPIO 27 (pin 13) | 220 ohm to 2N7000 gate, 10 k gate to GND, drain to terminal | DM320T DIR | |
| ENA | not wired | | DM320T ENA open | open ENA = always enabled (sheet 3.1); `ena = "none"` |
| OPTO | 5 V | direct | DM320T OPTO | |
| MOSFET sources | Pi GND | direct | both 2N7000 sources | |
| E-stop | GPIO 25 (pin 22), pull-up | NC contact (jumper for now) | GND (pin 20) | closed reads 0 (run) |
| Hall S | GPIO 17 (pin 11), pull-up | 15 k S to GND | KY-003 S | free once PUL moved |
| Motor A+ | | | DM320T A+ | black (BLK) |
| Motor A- | | | DM320T A- | green (GRN), pair with black (sheet colour, verified) |
| Motor B+ | | | DM320T B+ | red (RED) |
| Motor B- | | | DM320T B- | blue (BLU), pair with red (sheet colour, verified) |

### As wired and running, 2026-10-21 12:00

- Coil pairs on this 17HS08-1004 match the STEPPERONLINE sheet: black with green (coil A) and red with blue (coil B). As wired: A+ black, A- green, B+ red, B- blue. Verified 12:30: perfectly smooth the whole ramp.
- Lesson: a loose or badly seated motor wire and a crossed pair give the same symptom, whine and jitter with no rotation. The earlier green/blue swap only reseated a bad contact. Reseat the wires and measure the pairs with a meter before changing anything: about 3.7 ohm across a pair, open across coils.
- Rest of the bench as running: two 2N7000s, PUL GPIO 18 (pin 12), DIR GPIO 27 (pin 13), OPTO on 5 V (pin 2), sources on GND (pin 6), 220 ohm gates with 10 k pull-downs, ENA unconnected, e-stop GPIO 25 jumpered pin 22 to pin 20, 12 V bench supply, DIP SW1 ON SW2 ON SW3 OFF SW4 OFF SW5 OFF SW6 ON.

What changes with ENA not wired:
- The driver holds the rotor (50 percent current after 0.4 s idle) whenever 24 V is on. Stop, e-stop and Ctrl-C still bring the step rate to zero along the decel ramp, but the shaft never goes free; `status` prints "ENA: not wired (driver always enabled)".
- The 2N7000 is specified with Vgs(th) up to 3 V, so a 3.3 V gate is close to its limit on paper. At 14 mA of opto current a typical part is fully on. If a part ever misses steps, swap in a logic-level FET (for example a BSS138 or AO3400) or the NPN circuit below.
- The 10 k pull-downs hold PUL and DIR off while the Pi boots, so the motor cannot step before the software runs.

## Wiring table (NPN variant, the original design)

Pi pin numbers are physical header pins. GPIO numbers are BCM.

| Signal | Pi | Through | To | Notes |
|---|---|---|---|---|
| PUL | GPIO 18 (pin 12), hardware PWM0 | 1 k to NPN base, collector to terminal | DM320T PUL | step pulses, pigpio hardware_PWM |
| DIR | GPIO 23 (pin 16) in this variant (set `dir = 23`) | 1 k to NPN base, collector to terminal | DM320T DIR | |
| ENA | GPIO 24 (pin 18) in this variant (set `ena = 24`) | 1 k to NPN base, collector to terminal | DM320T ENA | GPIO high = disabled |
| OPTO | 5 V (pin 2) | direct | DM320T OPTO | opto anode supply, about 41 mA total |
| NPN emitters | GND (pin 14) | direct | 3 NPN emitters | keep the logic ground separate from the 24 V ground |
| Motor A+ | | | DM320T A+ | black (BLK) |
| Motor A- | | | DM320T A- | green (GRN): black with green, as on the sheet (meter it if it whines; a loose wire looks like a crossed pair) |
| Motor B+ | | | DM320T B+ | red (RED) |
| Motor B- | | | DM320T B- | blue (BLU): red with blue, as on the sheet |
| 24 V + | | LRS-50-24 +V | DM320T +Vdc | |
| 24 V - | | LRS-50-24 -V | DM320T GND | |
| Mains | | LRS-50-24 L, N, earth | wall cord | earth to the frame |
| E-stop | GPIO 25 (pin 22), internal pull-up | BACO LBX10510 NC contact | GND (pin 20) | contact closed reads 0 (run), open reads 1 (stop) |
| Hall VCC | 5 V (pin 4) | | KY-003 + (middle pin) | the A3144 needs 4.5 V minimum, so not 3.3 V |
| Hall GND | GND (pin 9) | | KY-003 - | |
| Hall S | GPIO 17 (pin 11), internal pull-up | 15 k from S to GND at the Pi end | KY-003 S | see below |
| Camera | CSI ribbon | | Arducam UC-261 | unchanged from froggy-cam-live |

Hall sensor level shift: most KY-003 boards carry a 10 k pull-up from S to VCC, so at 5 V the output idles at 5 V, which is too high for a Pi pin. A 15 k resistor from S to ground makes a divider with that 10 k: about 5 x 15 / 25 = 3.0 V high, and about 0.2 V when the sensor pulls low. If your board measures under 3.3 V on S with no magnet nearby, it has no pull-up and the 15 k can stay anyway (the Pi's pull-up then sets the high level). The A3144 is a unipolar south-pole switch, so face the magnet's south pole at the printed face of the sensor, 2 to 5 mm away. One magnet = one pulse per revolution (`index_pulses_per_rev = 1`).

E-stop behavior: the contact is normally closed to ground, so pressing the button or breaking a wire both read as "open" and stop the motor (fail safe). The code watches it with an edge callback (1 ms glitch filter) and a 10 ms poll, then ramps to rest in `emergency_decel_s` and drops ENA. This is a software-monitored control stop, not a safety-rated one: if the Pi crashes, the button does nothing. A second contact block breaking the 24 V line would make it hardware, at the cost of the drum coasting free. That is a DR2 decision.

## Two notes for when the drum is on

- Torque budget: with the full rotor at about 0.0038 kg m^2 and the 17HS08 giving about 0.1 Nm at speed, the safe angular acceleration is about 0.08 / 0.0038 = 21 rad/s^2, which is 200 RPM/s. That is the default `max_accel_rpm_per_s`. The 0.5 s e-stop decel from 400 RPM needs 0.32 Nm, so with the drum on the motor would lose sync and the drum would coast down instead. That is safe but slower, so set `emergency_decel_s = 2.0` (400 RPM) or `3.0` (600 RPM) in motor.toml before loaded runs. 0.5 s is right for the bare motor bench test.
- Regen: hard decels push energy back into the 24 V rail. The driver trips at 34 V. If the red LED blinks twice after an e-stop, lengthen `emergency_decel_s`.

## Pi software notes

- pigpiod runs with its default PCM clock, so the PWM block is free for GPIO 18. The Pi's analog audio also uses PWM; nothing plays audio on Lite, so leave it alone.
- Offline install: deploy.sh uses apt only if the Pi can reach archive.raspberrypi.com, otherwise it installs these vendored files from `debs/` with `dpkg -i` (checked against `debs/SHA256SUMS` first). All are Raspberry Pi OS bookworm, from http://archive.raspberrypi.com/debian/ and their sha256 matches the bookworm arm64 Packages index:

| File | URL path under archive.raspberrypi.com/debian/ | sha256 |
|---|---|---|
| libpigpio1_1.79-1+rpt1_arm64.deb | pool/main/p/pigpio/ | eb71bb889f79926c0d2c5078f2b0fe087a6e8f0c6e0c47659a2a5a072e8e831e |
| pigpiod_1.79-1+rpt1_arm64.deb | pool/main/p/pigpio/ | fcd0f97f497b58b8beb9780d0241561b443466f6416fabc00dc67466da9eb4e3 |
| pigpio-tools_1.79-1+rpt1_arm64.deb | pool/main/p/pigpio/ | 27381f19eca3bfed85945f7da616220f4a5c27c0c05e1a11cc12576d9db8a0a0 |
| python3-pigpio_1.79-1+rpt1_all.deb | pool/main/p/pigpio/ | a32d85d6ec327cf2ab075ab38ddf9b54cf193cbeb356ee23a1baeb161df46af9 |

Dependencies are only libc6 and python3, both on the stock image. pigpiod ships its own systemd unit, which deploy.sh enables.
