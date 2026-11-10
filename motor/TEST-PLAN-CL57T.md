# CL57T-V41 closed loop bench test (bare motor)

Replaces TEST-PLAN.md from 2026-11-09. Hardware: bench Pi 3 B+ (main Pi), STEPPERONLINE CL57T-V41, 17HS19-2004D-E1K, Mean Well LRS-50-24. Wiring, switch settings and every manual page reference: WIRING-CL57T.md and schematic-cl57t.png. Everything below runs from the laptop with `froggy-motor`; the first command deploys the code and installs pigpio on the Pi by itself. There is no camera on this Pi; the UI hides the camera preview.

## Driver settings for this motor

| Switch | Setting | Reason (details in WIRING-CL57T.md) |
|---|---|---|
| S1 | 0 = 2.8 A peak / 2.0 A RMS, factory gains | the motor is rated 2.0 A per phase; codes 4 and up are 4 A or 5 A RMS (manual p. 6) |
| SW1..SW4 | OFF OFF ON ON = 3200 pulses/rev | matches `microstep = 16`; 600 RPM = 32 kHz, far under 500 kHz (p. 1, 7) |
| SW5 | OFF | direction is set in software (`dir_cw_level`) |
| SW6 | OFF = closed loop | (p. 7) |
| SW7 | OFF = PUL/DIR | (p. 7) |
| SW8 | OFF = 1.5 ms filter | the Pi already ramps smoothly; keeps the following error honest |
| S3 | 5V | factory is 24V and the motor will not move with 5 V signals until this is changed (p. 7) |

Current, justified with numbers: the CL57T's lowest current group is 2.8 A peak, which is 2.8 / 1.414 = 2.0 A RMS, the motor's exact rating. Copper loss at that current is 2 phases x (2.0 A)^2 x 1.4 ohm = 11.2 W in the motor, the same as the vendor's own torque-curve condition (24 V, 2.0 A). The next group (5.6 A peak) would be 4 times the copper loss, about 45 W in a 48 mm NEMA 17, which overheats it. Compared with the DM320T, which topped out at 2.2 A peak (1.56 A RMS), the CL57T gives 27 percent more current, so about 27 percent more low-speed torque.

## Why the open-loop motor fell short, and what the CL57T changes

What was seen (TEST-PLAN.md, 2026-10-26): DM320T at 2.2 A peak (if SW1..SW3 were really all OFF for the 17HS19; nobody recorded checking), 1/16 step, bare motor. On the 12 V bench supply it held 240 RPM and stalled at 250 RPM and up: an abrupt stop, a whine while stopped, green LED only, no fault. On 24 V it held 250 RPM; nothing higher was tried. The hall index never gave a pulse, so every RPM in those notes is a commanded number and "held" was judged by eye and ear.

Numbers used below: coil R = 1.4 ohm, L = 3.0 mH (motor sheet), 50 rotor pole pairs (1.8 deg), so the coil current frequency is f = 50 x RPM / 60. There is no back-EMF constant on the sheet, so I estimate it from the holding torque: k = 0.59 Nm / (1.414 x 2.0 A) = 0.21 V s/rad per phase (peak). A cruder estimate (0.59 / 2.0) gives 0.30, so the answers below are ranges, not exact.

| RPM | coil frequency | coil reactance 2 pi f L | back-EMF peak (k = 0.21) | step rate at 1/16 |
|---|---|---|---|---|
| 240 | 200 Hz | 3.8 ohm | 5.2 V | 12.8 kHz |
| 250 | 208 Hz | 3.9 ohm | 5.4 V | 13.3 kHz |
| 300 | 250 Hz | 4.7 ohm | 6.5 V | 16.0 kHz |
| 400 | 333 Hz | 6.3 ohm | 8.7 V | 21.3 kHz |
| 600 | 500 Hz | 9.4 ohm | 13.1 V | 32.0 kHz |

### Candidate causes, worked through

1. **Not enough supply voltage to push full current at speed (back-EMF plus inductance).** A chopper driver can only hold its set current while the supply exceeds the coil's resistive drop plus back-EMF plus the inductive drop. Voltage needed is roughly sqrt((I R + E)^2 + (I 2 pi f L)^2). At 250 RPM and 2.2 A peak: I R = 3.1 V, E = 5.4 V, I X = 8.6 V, so about 12.2 V. A 12 V supply minus about 1 V lost in the driver's bridge gives about 11 V. Solving for the speed where the need reaches 11 V: **about 220 RPM** (190 RPM with k = 0.30). That lands right next to the 240 / 250 RPM line found on the bench, and the 24 V run getting through 250 RPM fits too: at 23 V available the same sum puts the full-current corner at **about 520 RPM** at 2.2 A (450 with k = 0.30). Above the corner the current, and with it the torque, falls off roughly as 1 / speed.
   But a bare motor needs almost no torque to keep turning (accelerating the 82 g cm^2 rotor at 200 RPM/s takes 0.0002 Nm), so running out of current alone should make it weak, not stop it dead. That points at cause 2 as the trigger.
2. **Mid-band resonance.** Hybrid steppers have an instability band, usually a few rev/s (roughly 2 to 8 rev/s for NEMA 17), where the rotor's oscillation about its step position grows each step until it slips out of sync and stops, still buzzing. 250 RPM is 4.2 rev/s, squarely in that band, and it is worst on an unloaded motor because there is no load inertia or friction to damp it. It is also worst exactly when the driver loses current control: the current loop is what damps the oscillation, and above the voltage corner in cause 1 the driver is no longer regulating current. A sudden stop with a whine and no fault is the textbook look of this. I cannot compute the band from the data we have; it is a known behaviour, not a number from our sheets.
3. **Missed steps that open loop cannot see.** The DM320T has no feedback and no fault output for a stall (WIRING.md), and the hall gave no pulses, so a slip or a full stall left every log line saying "commanded 250 RPM". This is not a cause of the stall, but it is why the stall could not be measured, and why we do not know whether the "held 240" run was really clean. It also means the 24 V "held 250" result is the only upper bound we have.
4. **Current setting and supply limit (unverified).** The 10-21 bench ran the 17HS08 at SW1 ON SW2 ON SW3 OFF (1.3 A peak). motor.toml says the 17HS19 got SW1..SW3 OFF (2.2 A peak) on 10-26, but the DIP was not recorded on the day. If it was still at 1.3 A, the voltage corner moves up but torque everywhere drops by 40 percent. The 12 V bench supply's current limit was also not recorded; a supply that sagged into current limit lowers the voltage in cause 1. Both are cheap to rule out and both are gone with the new setup (S1 is a single visible dial, and the LRS-50-24 is a fixed 24 V supply).
5. **Supply power (LRS-50-24).** The LRS-50-24 is rated 24 V at 2.2 A, 52.8 W (Mean Well's LRS-50 spec, from memory). The motor's 2 A per phase is coil current, not supply current: a chopper drive draws from the supply only the power it uses. Bare motor at 600 RPM: about 11 W copper + about 3 W driver + a few W of friction is roughly 16 W, 0.7 A. Even at the vendor curve's full 31.6 N cm at 600 RPM (19.9 W of shaft power) the total is about 34 W, 1.4 A. So supply power was not the bare-motor problem and is fine for the drum too (the 200 RPM/s ramp on the 0.0038 kg m^2 drum needs 0.08 Nm, 5 W at 600 RPM). Supply **voltage** is the real limit, see cause 1.
6. **Pulse timing on the Pi.** The step train is pigpio hardware PWM on GPIO 18, generated by the PWM block, so there is no software jitter. The frequency changes only every 10 ms during a ramp, and the stall happened during a constant-speed hold, when the PWM frequency was not changing at all. 13.3 kHz at 50 percent duty is 37.5 us high and low, five times the DM320T's 7.5 us minimum and far over the CL57T's 1 us. Least likely.

Ranking (most to least likely): a combined voltage and resonance stall (causes 1 and 2) at 12 V, with the corner computed at about 220 RPM against 240 to 250 seen; then the unknowns (3 and 4) that kept us from measuring it; supply power and Pi timing (5 and 6) are ruled out by the numbers.

### What the CL57T fixes

- **Missed steps are corrected, and a real stall raises a fault.** The driver reads the 1000 line encoder, adds current when the rotor lags, and if the following error grows anyway it trips fault 7 (red LED, 7 blinks) and pulls ALM (manual p. 10 to 11). The Pi now reads ALM on GPIO 22 and stops the ramp the same tick, with the cause in `status`, the UI and the cl-test verdict. No more silent stall.
- **Resonance.** Closed loop drives commutate from the measured rotor angle, which damps the mid-band oscillation that tips an open-loop motor out of sync; the manual lists anti-resonance as a feature (p. 1). This should remove cause 2 as a stall mechanism.
- **More current and torque at low speed**: 2.8 A peak against 2.2 A.
- **Measurement**: with the optional encoder tap the Pi gets real shaft RPM and a following error, so the bench numbers stop being commanded-only.

### What it does not fix

- **Supply voltage headroom.** Back-EMF and coil inductance are physics, not driver features. At 24 V and the full 2.8 A peak the corner where current starts to fall is about 420 RPM (380 with k = 0.30). Above it torque falls off. The vendor curve says that is still enough: 33.4 N cm at 300 RPM and 31.6 N cm at 600 RPM at 24 V. Check: 31.6 N cm needs about 1.5 A peak, which takes about 21 V at 600 RPM by the same formula, just under the 23 V available. So 600 RPM at 24 V should work but with little margin, and it will not go much past it. The only fix is more voltage: the CL57T takes up to 50 V (p. 1), so a 36 V or 48 V supply (an LRS-100-36 or LRS-150-48 class part) would roughly double or triple the corner speed if the loaded drum needs it.
- **Speed measurement.** There is no hall sensor on this bench and there will not be one, so the Pi still cannot see shaft speed. A clean closed loop run proves the driver followed every pulse it received; it cannot prove that every pulse the Pi sent got through the 3.3 V inputs (WIRING-CL57T.md, Variant A). The optional encoder tap is the way to a real number.
- **Regen on hard stops with the drum.** The drum stores 0.5 x 0.0038 x (62.8 rad/s)^2 = 7.5 J at 600 RPM, against 0.016 J for the bare rotor. A 0.5 s e-stop pushes most of that back into the 24 V rail. Keep `emergency_decel_s` at 2 to 3 s with the drum on (as before), and watch for fault 2 (over-voltage).

## Software (what is new)

- motor.toml (Variant A, ships ready): `[pins]` `pul = 18` (pin 12), `dir = 17` (pin 11), `ena = 27` (pin 13), `estop = 25`, `index = "none"`, `alm = 22` (pin 15), `pend = "none"`, `brk = "none"`, `enc_a = "none"`. `ena_enable_level = 0` (GPIO 27 high = disabled). `[driver]` `model = "CL57T-V41"`, `s1 = "0"`, `loop = "closed"`, `alm_active_level = 0`, `enc_ppr = 1000`. `camera = false`.
- ENA: disabled at start, at idle, at the end of every run, on Stop, e-stop, ALM and exit; a run enables it, waits 200 ms, sets DIR, then pulses (manual p. 10). From power-up until the software first runs the driver is enabled and holds the shaft (GPIO 27 pulled low at boot); it cannot move then because PUL is low too.
- `froggy-motor status` prints the driver model and switch settings, the ENA state, and the live ALM, PEND, BRK and encoder tap states. `index-test` refuses with "no hall index sensor configured".
- ALM handling: an ALM edge (or two 10 ms polls in a row) stops the PWM at once, drops ENA if wired, logs the time and commanded RPM, and refuses new runs until ALM clears (power-cycle the driver). No decel ramp, because the CL57T has already freed the shaft.
- `froggy-motor cl-test --rpm R [--hold-s 10] [--accel-s A] [--decel-s D] [--dir cw|ccw]`: ramps at the normal limit (default accel time rpm / 200 RPM/s, at least 2 s), holds, stops, writes `logs/cl-test-<R>rpm-<time>.csv` on the Pi with one row per 100 ms (t_s, rpm_cmd, step_hz, rpm_hall, rpm_encoder, cmd_rev, enc_rev, follow_err_deg, alm, pend, brk, estop, phase) and ends with one line. On this bench (no hall, no tap) it is honest about what it knows: `verdict: reached 300 RPM commanded (shaft speed unmeasured: no hall sensor, no encoder tap), tracking error unknown, alarms none (ALM ok in all 72 samples); closed loop with no fault 7: the driver followed every pulse it received, but pulses missed at its 3.3 V inputs would not show; log ...`. With the encoder tap fitted (`enc_a = 5`) it becomes `verdict: reached 299.8 RPM (encoder tap, 300 commanded), tracking error mean x % max y % ..., following error max w deg, alarms ...`. Exit code 5 if ALM stopped it. The UI's run-mode menu has the same test ("Closed-loop test") and a "Closed loop driver" tile with ALM, PEND, encoder RPM, following error and the last verdict.
- Dry runs on the Mac: `froggy-motor --dry-run --sim-fast cl-test --rpm 300` (healthy), add `--sim-stall-rpm 250` to see a closed-loop alarm, or `--driver DM320T --sim-stall-rpm 250` to replay the October silent stall (verdict: reached 0.0 RPM, alarms none).

## Bench procedure (run from the Mac)

Before each step note the time. After each cl-test run `froggy-motor --pull-logs` to copy the CSVs into `logs/`. Keep the e-stop in reach; Ctrl-C or `froggy-motor stop` also stops any run.

1. **Wiring and switches, power off.** Set the switches as in the table above (S3 to 5V is the easy one to miss). Wire Variant A per WIRING-CL57T.md and schematic-cl57t.png: PUL+ to GPIO 18 (pin 12), DIR+ to GPIO 17 (pin 11), ENA+ to GPIO 27 (pin 13), PUL-, DIR-, ENA- to Pi GND (pin 6); ALM wire to GPIO 22 (pin 15) with 10 k from 5 V (pin 4) and 15 k to GND (pin 14) at the Pi end; COM- to GND (pin 14). Coil first try A+ BLK, A- GRN, B+ RED, B- BLU (vendor article); encoder EB+ YEL, EB- GRN, EA+ BLK, EA- BLU, VCC RED, EGND WHT (check these colours against the E1K cable label first). Meter: about 1.4 ohm across BLK-GRN and across RED-BLU, open between the pairs. Put a tape flag on the shaft. Record: S1 code, all eight SW positions, S3, measured coil resistances.
2. **Pi only, then ALM polarity proof (no motion).** With the driver unpowered run `froggy-motor status` (first run deploys and installs pigpio: watch for errors on trixie). Expect `driver: CL57T-V41, closed loop (SW6 OFF), S1 code 0 ...`, `ENA: GPIO 27, driver disabled (idle)`, `ALM: ok`, `PEND: not wired (the CL57T-V41 P1 connector has no PEND terminal)`. With 24 V on and the software idle, the shaft should turn freely by hand (ENA high = disabled); if it is held, the ENA polarity reading in WIRING-CL57T.md is wrong, so set `ena_enable_level = 1` and check again. Then, power still off, unplug the motor connector P3, power the driver: the manual says it should flash 4 (fail to lock motor shaft). Run `froggy-motor status`: it must say `ALM: ALARM`. If it says ok while the red LED flashes, set `alm_active_level = 1` in motor.toml and check again. Power off, plug P3 back in, power on: green on, red off, `status` says `ALM: ok`. Record: ALM raw level when ok and when in alarm, red LED blink count, `alm_active_level` used.
3. **60 RPM check.** `froggy-motor cl-test --rpm 60 --hold-s 30`. Expect a smooth, quiet turn at 1 rev/s: count the tape flag, 30 turns in the 30 s hold (a phone video makes this easy). The verdict should end `alarms none (ALM ok in all ... samples)` and say the speed is unmeasured. Fewer turns than commanded with no alarm means pulses are being missed at the 3.3 V inputs: go to Variant B. If the red LED flashes 7, or the shaft runs away or hunts at the first move: power off, swap A+ and A- at P3 (coil A polarity against the encoder), nothing else, and retry. (The manual's own check for fault 7 is SW6 ON for open loop: if it then turns, the encoder or coil phase was wrong.) Record: verdict line, direction seen from the shaft end (flip `dir_cw_level` if needed), noise, driver and motor temperature by hand, any red LED.
4. **250 RPM, the old stall speed.** `froggy-motor cl-test --rpm 250 --hold-s 15`. Record: verdict, any whine or buzz and at what commanded RPM, ALM. The flag is too fast to count here, so the evidence is no fault 7 and no stall sound.
5. **300 RPM.** `froggy-motor cl-test --rpm 300 --hold-s 15`. Past the old DM320T limit on both supplies. Record the same as step 4.
6. **400 RPM.** `froggy-motor cl-test --rpm 400 --hold-s 15`. Near the computed full-current corner at 24 V (about 420 RPM). Record the same, plus motor temperature after the hold.
7. **600 RPM, design top speed.** `froggy-motor cl-test --rpm 600 --hold-s 15` (3 s ramp). Record the same, plus driver temperature, supply current if a clamp meter is handy, and whether the motor sounds rough near the top. If ALM trips (fault 7), note the commanded RPM at the trip: that is the 24 V torque limit for the bare motor, and the case for a 36 or 48 V supply.
8. **Optional open-loop comparison.** SW6 ON, `loop = "open"`, `froggy-motor cl-test --rpm 300 --hold-s 10`, then back to SW6 OFF. Shows how much of the gain is the closed loop and how much is just 24 V. Record the verdict.

Record sheet (one row per step):

| Step | Time | Command | Verdict line | Flag count (step 3) | ALM / LED | Noise | Motor / driver temp | Notes |
|---|---|---|---|---|---|---|---|---|
| 3 | | cl-test 60 | | | | | | |
| 4 | | cl-test 250 | | | | | | |
| 5 | | cl-test 300 | | | | | | |
| 6 | | cl-test 400 | | | | | | |
| 7 | | cl-test 600 | | | | | | |

Pass for the bare motor: steps 3 to 7 all end `alarms none (ALM ok in all ... samples)`, step 3's flag count matches the command, and no step stalls or buzzes. That shows the closed loop followed every pulse it received up to 600 RPM; it does not measure speed. Then the drum can go on, with `emergency_decel_s` at 2 to 3 s. Fitting the encoder tap later turns these into measured numbers with no other change.
