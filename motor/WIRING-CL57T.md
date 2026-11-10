# Froggy motor wiring: Pi 3 B+ to CL57T-V41 closed loop driver, 17HS19-2004D-E1K, 24 V, e-stop

This replaces WIRING.md (DM320T, open loop) for the bench from 2026-11-09 on. Drawing: `schematic-cl57t.html` (open in a browser) and its render `schematic-cl57t.png`, built from the real vendor and Raspberry Pi pictures in `schematic-img/` (sources and licences in `schematic-img/sources.txt`), with only the wires, values and marks drawn on top.

Main wiring is **Variant A: GPIO straight to the driver's optos, no FETs**, which is the vendor's own closed loop wiring diagram. Variant B (two 2N7000s) is the fallback if Variant A misses pulses. The only added parts in Variant A are the two ALM divider resistors.

## Sources

Page numbers below are the printed footer numbers on each manual. In the PDF viewer the CL57T(V4.0) and CL42T(V4.1) pages sit 2 later (printed p. 3 is PDF page 5).

| Reference PDF (not in this repository, download it from the link) | What it is | Where it came from | sha256 |
|---|---|---|---|
| `CL57T-V4.0-manual.pdf` | STEPPERONLINE "User Manual CL57T(V4.0) Closed Loop Stepper Driver", Revision 4.0, 14 pages, PDF made 2022-05-17 | https://www.stepperonline.co.uk/download/CL57T_V4.0.pdf (plain download). Byte for byte the same as the Wayback Machine copy of https://www.omc-stepperonline.com/download/CL57T_V4.0.pdf from 2025-01-09 | e1b4a0028185f268fc3efecdc84ff0617630d8c2e0daad17bb7f32cc8c1f82be |
| `CL42T-V4.1-manual.pdf` | STEPPERONLINE "User Manual CL42T(V4.1)", Revision 4.1, 14 pages. The 3 A sibling of our driver, same revision step | LinuxCNC forum attachment https://forum.linuxcnc.org/media/kunena/attachments/36700/CL42T-V41Manual2.pdf | e7175044d1ec441d064138f1307427aa06d2e179dc9ff97cea2487834c7734e2 |
| `17HS19-2004D-E1000-datasheet.pdf` | STEPPERONLINE drawing and spec sheet for the 17HS19-2004D-E1000 (same 2.0 A, 48 mm motor with the 1000 line encoder, sold before the E1K name), 1 page, dated 10.16.2021 | https://www.stepperonline.co.uk/download/17HS19-2004D-E1000.pdf | 08bee5391be8222c26007b0ee163ff8c92e33169d506696e25742fdf23c82857 |

| `2N7000-onsemi-datasheet.pdf` | onsemi "2N7000, 2N7002, NDS7002A" data sheet, Rev. 11, March 2025, 10 pages | https://www.onsemi.com/download/data-sheet/pdf/nds7002a-d.pdf | e1ded9dc51aabdc74fa1f157e19f16fe96f0af4c7672d6d847102cbf934b9cdf |

Also used:
- STEPPERONLINE help article "Wiring diagram for closed loop stepper motor", https://help.omc-stepperonline.com/hc/s/articles/wiring-diagram-for-closed-loop-stepper-motor (images saved in `schematic-img/`, see sources.txt). Its CL57T-V41 picture shows a generic "Controller" with four solid wires to PUL+, PUL-, DIR+, DIR-, dashed optional wires to ENA+, ENA-, ALM, BRK, COM-, encoder EB+ yellow, EB- green, EA+ black, EA- blue, VCC red, EGND white, and the supply to +Vdc and GND. It states no controller voltage.
- `model/cad/vendor/motion/stepperonline_17hs19_2004d_e1k/17HS19-2004D-E1K_Dimension.svg` in this repository (vendor drawing of our exact motor): flying leads, a 6-core shielded encoder cable and a 4-core motor cable, 500 mm each, no DB15 plug.
- `.../17HS19-2004D-E1K_Torque_Curve.svg`: pull-out torque at 24 V, 2.0 A, 2000 pulses/rev.

### The CL57T(V4.1) manual itself is NOT in this folder

The real file is https://files.omc-stepperonline.com/CL57T-V41_Manual.pdf ("User Manual CL57T(V4.1) Closed Loop Stepper Driver, Revision 4.1", found by web search). What I tried on 2026-11-09:
- Claude in Chrome: the extension was not connected in this session, so the product page could not be opened in a real browser.
- Plain downloads: files.omc-stepperonline.com/CL57T-V41_Manual.pdf, www.omc-stepperonline.com/download/CL57T-V41.pdf and /download/CL57T.pdf all answer 403 (Cloudflare). No header spoofing was tried.
- stepperonline.co.uk mirror: CL57T-V41.pdf, CL57T-V4.1.pdf, CL57T_V4.1.pdf, CL57T-V41_Manual.pdf, CL57T-V41-Manual.pdf all 404. (CL57T.pdf there is the old V3.0 manual, CL57T_V4.0.pdf is the file above.)
- manuals.plus kit manuals (403), oyostepper.com (only the 2017 CL57T), Wayback Machine (the V4.1 file was never archived).
- yumpu.com document 70653589 "CL57T(V4.1)" is the real V4.1 manual, but only its text layer was readable (through a web fetch summary, not a file). That text gives the revision record: 1.00 Aug 2019 initial, 4.00 Oct 2020 "add S1 rotating switch, 5/24V selector switch, brake output", 4.10 May 2023 "add connector P5, tuning port and description". Its S1 table, SW1 to SW8 tables, P1 and P2 pin lists, electrical table and fault list match the V4.0 manual line for line.

So: every CL57T value below comes from the V4.0 manual, which the V4.1 only extends with the P5 tuning port. The P5 pinout comes from the CL42T(V4.1) manual (same revision note, "4.10 May,2023 Add connector P5 -tuning port and description", the revision table on the CL42T cover page). To close this gap, open the files.omc link above in Chrome once and drop the PDF here; I expect no changes, but check the P5 figure.

## CL57T facts (from the CL57T(V4.0) manual)

| Item | Value | Where |
|---|---|---|
| Supply voltage | 18 to 50 VDC, 24, 36 or 48 typical, 24 to 48 recommended | p. 1 (1, 2.1), p. 5 (4.2) |
| Peak current | 2.8 A (2 A RMS), 5.6 A (4 A RMS), 7.0 A (5 A RMS), picked on the S1 rotary switch | p. 1 (1), p. 6 (5.1). The 2.1 table prints "2.4 (RMS 2A)"; 2.8 / 1.414 = 2.0, so 2.8 is right and 2.4 is a typo |
| Logic input current | 7 min, 10 typical, 16 max mA | p. 1 (2.1) |
| Pulse input | 0 to 500 kHz; pulse width at least 1.0 us; 50 percent duty recommended | p. 1 (2.1), p. 3 (3.1), p. 10 (7) |
| DIR setup | DIR at least 2 us ahead of the active PUL edge | p. 1 (2.1), p. 3, p. 10 (7, t2) |
| ENA setup | ENA at least 200 ms ahead of DIR | p. 3, p. 10 (7, t1) |
| Input levels | high 3.5 to 5 V or 24 V, low 0 to 0.5 V; optically isolated; differential or single ended | p. 3 (3.1), p. 8 (6.1) |
| Signal voltage selector S3 | factory setting 24 V; with 5 V signals "the S3 must be set to 5V, otherwise, the motor won't work". 24 V signals with S3 on 5V damage the optos | p. 7 (5.3), p. 8 (6, note 1) |
| 12 V signals | set S3 to 5V and add 1 k in series | p. 8 (6.1, note 2) |
| ENA | optional, high 3.5 to 24 V, not connected by default. The troubleshooting table's fix for a disabled drive is to leave ENA+ and ENA- unconnected, so open = enabled | p. 3 (3.1), p. 8 (6.1 note 1), p. 12 |
| ALM output | "sinking or sourcing 100mA current at 5-24V. Max 30V"; spec table: alarm output voltage max 24 VDC | p. 3 (3.1), p. 1 (2.1) |
| ALM behaviour | on an over-voltage or over-current fault the impedance between ALM and COM- flips, low to high or high to low "depending on configuration". The normal state is not given | p. 9 (6.2, Fig. 7 sinking, Fig. 8 sourcing) |
| BRK output | max 24 V / 100 mA, drives a brake relay with a diode | p. 3 (3.1), p. 9 (6.3, Fig. 9) |
| COM- | "Common connection of single-end output signals (common-cathode)", printed "COMO" in the table, "COM-" on the label | p. 3 (3.1, Fig. 2) |
| PEND (in-position) | **no such terminal.** The P1 table lists only PUL, DIR, ENA, ALM, BRK, COM-, and so does the label photo. Fig. 4 on p. 8 shows an "In-position / PEND" wire, but it is a generic drawing (it also shows EZ and a 6-pin motor connector this driver does not have) | p. 3 (3.1, Fig. 2), p. 8 (Fig. 4) |
| Encoder input P2 | EB+, EB-, EA+, EA-, VCC (encoder +5 V out), EGND | p. 4 (3.2) |
| Motor P3 | A+, A-, B+, B- | p. 4 (3.3) |
| Power P4 | GND, +VDC (24 to 48 VDC suggested). Never plug or unplug P3 or P4 with power on | p. 4 (3.4) |
| LEDs | green = power, always on. Red = alarm, off when fine, flashes a count in a 3 s period on a fault | p. 4 (3.5) |
| After a fault | the shaft goes free or the red LED blinks; the only reset is to power-cycle the drive | p. 11 (8) |
| Fault codes | 1 over-current, 2 over-voltage, 3 chip error, 4 fail to lock motor shaft (no motor connected, or bad motor cable), 5 EEPROM error, 6 fail to auto tune, 7 position following error (check by setting SW6 ON: if the alarm goes away the encoder wiring is wrong; otherwise not enough torque or too much speed), always on = PCB burned out | p. 10 to 11 (8) |
| Supply voltage and speed | higher supply voltage raises torque above about 300 RPM and helps avoid lost steps, at the cost of more low-speed vibration and over-voltage risk | p. 5 (4.2) |
| Motor stalls during acceleration | listed causes: current setting too small, motor undersized, acceleration too high, "Power supply voltage too low" | p. 12 |

P1 to P4 terminal order on the driver label, top to bottom (p. 3, Fig. 2): S1 window, PUL+, PUL-, DIR+, DIR-, ENA+, ENA-, ALM, BRK, COM-, EB+, EB-, EA+, EA-, VCC, EGND, A+, A-, B+, B-, +Vdc, GND.

### P5 tuning port (V4.1 only)

From the CL42T(V4.1) manual p. 4 (section 3.5, P5 Debug Port): P5 is an RS232 port for a PC, "just used to modify parameter, not for equipment control", because the manual says its precision and stability are not good enough. The figure shows a 4-pin white connector, not an RJ11:

| P5 pin | Signal | Female DB9 pin (PC side cable) |
|---|---|---|
| 1 | NC | |
| 2 | TXD | 2 |
| 3 | GND | 5 |
| 4 | RXD | 3 |

The manual gives only this pinout. It does not publish the protocol, baud rate, register map, or any readable position error, actual speed or current value. So the software does not read P5 and I did not invent a protocol. To use the vendor's PC tuning tool: use a true RS232 USB adapter with a male DB9 (FTDI chip plus RS232 level shifter), not a 3.3 V TTL USB serial cable, since RS232 swings past +-5 V. Run the tool on a Windows PC, not the Pi. The vendor tool's name and download could not be checked because the product page was unreadable today.

### DIP and rotary switches (exact from the manual)

S1, rotary, peak current and gains (p. 6, 5.1). "Usually keep factory settings."

| Code | Peak current | RMS current | Velocity loop Ki | Position loop Kp | Velocity loop Kp |
|---|---|---|---|---|---|
| 0 (factory) | 2.8A | 2A | 0 | 25 | 25 |
| 1 | 2.8A | 2A | 0 | 50 | 15 |
| 2 | 2.8A | 2A | 16 | 25 | 25 |
| 3 | 2.8A | 2A | 16 | 50 | 15 |
| 4 | 5.6A | 4A | 0 | 25 | 25 |
| 5 | 5.6A | 4A | 0 | 50 | 15 |
| 6 | 5.6A | 4A | 0 | 100 | 5 |
| 7 | 5.6A | 4A | 16 | 25 | 25 |
| 8 | 5.6A | 4A | 16 | 50 | 15 |
| 9 | 5.6A | 4A | 16 | 100 | 5 |
| A | 7A | 5A | 0 | 25 | 25 |
| B | 7A | 5A | 0 | 50 | 15 |
| C | 7A | 5A | 0 | 100 | 5 |
| D | 7A | 5A | 16 | 25 | 25 |
| E | 7A | 5A | 16 | 50 | 15 |
| F | 7A | 5A | 16 | 100 | 5 |

Manual notes on the gains (p. 6): Ki "0" means a longer stop time but smaller position error, "16" a shorter stop time with slightly larger error. Position Kp and velocity Kp are a pair that sets rigidity: 25 / 25 weak, 100 / 5 strong. If the motor creeps after stopping, raise position Kp, but too high makes noise.

S2 SW1 to SW4, microstep (p. 7, 5.2.1):

| Micro step | Pulses/Rev (1.8 deg) | SW1 | SW2 | SW3 | SW4 |
|---|---|---|---|---|---|
| 1 | 200 | on | on | on | on |
| 4 | 800 | off | on | on | on |
| 8 | 1600 | on | off | on | on |
| 16 | 3200 | off | off | on | on |
| 32 | 6400 | on | on | off | on |
| 64 | 12800 | off | on | off | on |
| 128 | 25600 | on | off | off | on |
| 256 | 51200 | off | off | off | on |
| 5 | 1000 | on | on | on | off |
| 10 | 2000 | off | on | on | off |
| 20 | 4000 | on | off | on | off |
| 25 | 5000 | off | off | on | off |
| 40 | 8000 | on | on | off | off |
| 50 | 10000 | off | on | off | off |
| 100 | 20000 | on | off | off | off |
| 200 | 40000 | off | off | off | off |

S2 SW5 to SW8, mode (p. 7, 5.2.2):

| Switch | Function | ON | OFF |
|---|---|---|---|
| SW5 | Rotation Direction | CW (clockwise) | CCW (counterclockwise) |
| SW6 | Control Mode | Open loop control | Closed loop control |
| SW7 | Pulse Mode | CW/CCW (double pulse) | PUL/DIR (single pulse) |
| SW8 | Pulse Filter Time | Yes (10ms) | No (1.5ms) |

S3 (p. 7, 5.3): 5V or 24V control signal level, factory 24V.

## Settings for our motor (17HS19-2004D-E1K, 2.0 A per phase)

| Switch | Set to | Why |
|---|---|---|
| S1 | **0** | Codes 0 to 3 are the 2.8 A peak / 2.0 A RMS group (p. 6), which is exactly the motor's 2.0 A rating (motor sheet: AMPS/PHASE 2.00). Codes 4 and up give 4 A or 5 A RMS, twice the rating or more: the motor would cook. 0 is the factory gain set the manual says to keep; try 1 (stiffer) only if the loaded drum hunts or creeps. |
| SW1..SW4 | **OFF OFF ON ON** (3200 pulses/rev) | Matches `microstep = 16` in motor.toml, so no software change. 600 RPM is 32 kHz, 6.4 percent of the 500 kHz limit (p. 1), and each half period is 15.6 us against the 1.0 us minimum. The closed loop drive does its own fine control, so finer steps buy nothing here. |
| SW5 | **OFF** (factory) | Direction sense only. Use `dir_cw_level` in motor.toml if cw is backwards. |
| SW6 | **OFF** = closed loop | The point of the new driver. ON is only for the encoder wiring check the manual describes for fault 7 (p. 11). |
| SW7 | **OFF** = PUL/DIR | The Pi sends step and direction. |
| SW8 | **OFF** = 1.5 ms filter | The Pi already ramps at up to 200 RPM/s in 10 ms steps (about 107 Hz per step at 1/16), so the 10 ms S-curve filter is not needed, and OFF keeps the commanded position close to what the Pi counts, which keeps the cl-test following error honest. Try ON if speed changes sound rough. |
| S3 | **5V** | Variant A drives the inputs from 3.3 V GPIOs and Variant B from the Pi's 5 V; both need the 5V setting. Factory 24V would leave the optos dark and the motor dead (p. 7). |

The 2026-10-28 interface review said "CL57T-V41 works with 17HS19 at S1 2.8 A, S3 5 V, closed loop". Confirmed, with one detail: "2.8 A" is a group of four S1 codes (0 to 3), not one position, so the setting is S1 = 0. (The CL42T(V4.1) also has a 2.8 A group, codes C to F on its p. 6, so the review may have read that manual; for the CL57T the code is 0.)

## Motor data (17HS19-2004D-E1000 sheet, same motor body as our E1K)

| Item | Value |
|---|---|
| Amps/phase | 2.00 |
| Resistance/phase | 1.40 ohm +-10 percent at 25 C |
| Inductance/phase | 3.00 mH +-20 percent at 1 kHz |
| Holding torque | 0.59 Nm |
| Step angle | 1.8 deg |
| Encoder | 1000 PPR (4000 CPR) |
| Coil leads | A = GRN, A\ = BLK, B = RED, B\ = BLU |
| Encoder leads (15-pin DB on the E1000) | pin 1 EA+ BLK, pin 2 VCC RED, pin 3 GND WHT, pin 11 EB+ YEL, pin 12 EB- GRN, pin 13 EA- BLU |

Encoder colours: three sources agree. The CL57T manual's Fig. 4 (p. 8) prints EA+ BLK, EA- BLU, EB+ YEL, EB- GRN, +5V RED, GND WHT; the E1000 sheet has the same six; the vendor wiring article's CL57T-V41 picture draws the same colours. (The same article also has a GRN/BLK/RED/BLU motor table with an 8-lead encoder, RED VCC, BLK EGND, BRN EA+, ORG EA-, BLU EB+, GRN EB-, YEL EZ+, WHT EZ-. That does not match the 6-core cable on our E1K, so it is for a different encoder.) Our E1K has flying leads instead of the DB15 and I could not get its own sheet, so **check the six encoder colours against the label on the E1K cable before power**.

### Coil colours: the two sources disagree, and how to settle it on the bench

| Source | Coil A | Coil B |
|---|---|---|
| Vendor wiring article (CL57T-V41 picture, and its "Option B" for black/green/red/blue motors as relayed to me) | A+ BLK, A- GRN | B+ RED, B- BLU |
| 17HS19-2004D-E1000 sheet lead table | A (A+) GRN, A\ (A-) BLK | B (B+) RED, B\ (B-) BLU |
| DM320T bench, 2026-10-21 | A+ BLK, A- GRN | B+ RED, B- BLU |

The pairs agree everywhere (black with green, red with blue). Only coil A's polarity differs. Open loop that just flips the direction, but closed loop the coil phase must match the encoder phase, or the driver sees the shaft move against the command.

**First try: the vendor article, A+ BLK, A- GRN, B+ RED, B- BLU.** It is the closed loop specific vendor diagram and it is also how the bench is already wired.

**Bench rule:** if the driver throws fault 7 (7 red blinks, position following error), or the shaft runs away or hunts back and forth on the first move, power off and swap A+ and A- (coil A polarity relative to the encoder). Never change anything else for this symptom: not the encoder leads, not coil B, not the pairs.

## Wiring table, Variant A (main): GPIO straight to the optos

Pi pin numbers are physical header pins, GPIO numbers are BCM. Same 40-pin header on the Pi 3 B+ as before. This is the vendor article's picture with the Pi as the "Controller", and the manual's single-ended hookup (p. 3, 3.1; p. 8, 6.1).

| Signal | Pi 3 B+ pin | Through | Driver terminal | Manual page |
|---|---|---|---|---|
| PUL step | GPIO 18 (pin 12), hardware PWM0 | direct | PUL+ | p. 3 (3.1), p. 8 (6.1) |
| PUL return | GND (pin 6) | direct | PUL- | p. 3 (3.1), p. 8 (6.1) |
| DIR | GPIO 17 (pin 11) | direct | DIR+ | p. 3 (3.1), p. 8 (6.1) |
| DIR return | GND (pin 6) | direct | DIR- | p. 3 (3.1), p. 8 (6.1) |
| ENA | GPIO 27 (pin 13) | direct; GPIO high = driver disabled | ENA+ | p. 3 (3.1), p. 8 (6.1 note 1), p. 10 (t1), p. 12 |
| ENA return | GND (pin 6) | direct | ENA- | p. 3 (3.1), p. 8 (6.1) |
| Signal level | | S3 set to 5V (factory 24V) | | p. 7 (5.3) |
| ALM pull-up | 5 V (pin 4) | 10 k from pin 4 to the ALM wire, at the Pi end | ALM | p. 3, p. 9 (6.2, Fig. 7 sinking) |
| ALM read | GPIO 22 (pin 15) | the ALM wire; 15 k from GPIO 22 to GND (pin 14) at the Pi end | ALM | p. 3, p. 9 (Fig. 7) |
| ALM common | GND (pin 14) | direct | COM- | p. 3, p. 9 (Fig. 7) |
| BRK | not wired | (optional: same 10 k / 15 k divider to GPIO 23, pin 16, logged raw) | BRK | p. 3, p. 9 (6.3) |
| PEND | none on this driver | | | p. 3 (3.1) |
| Motor coil A (first try) | | | A+ BLK, A- GRN (swap if fault 7, see above) | p. 4 (3.3); vendor article |
| Motor coil B | | | B+ RED, B- BLU | p. 4 (3.3); vendor article and motor sheet |
| Encoder | | motor encoder cable | EB+ YEL, EB- GRN, EA+ BLK, EA- BLU, VCC RED, EGND WHT | p. 4 (3.2), p. 8 (Fig. 4); motor sheet; vendor article |
| 24 V + | | LRS-50-24 +V | +Vdc | p. 4 (3.4) |
| 24 V - | | LRS-50-24 -V | GND | p. 4 (3.4) |
| Mains | | LRS-50-24 L, N, earth | wall cord, earth to the frame | supply side of P4 (p. 4, 3.4) |
| E-stop | GPIO 25 (pin 22), internal pull-up | NC contact (jumper for now) | GND (pin 20) | Pi only, no driver terminal, so no CL57T page; unchanged from WIRING.md |
| Hall index | none | there is no hall sensor on this bench and there will not be one; `index = "none"` | | no driver terminal, so no CL57T page |
| Encoder tap (optional) | GPIO 5 (pin 29); 3.3 V (pin 17); GND (pin 25) | EA+ through 10 k into a 74LVC1G17 (or one gate of a 74LVC125) powered at 3.3 V, output to GPIO 5; EGND to Pi GND (pin 25) | EA+ (in parallel with the motor lead), EGND | p. 4 (3.2) |

Logic sense: GPIO high drives current from PUL+ to PUL-, so the opto is on, the same sense as the DM320T's FET stage. `dir_cw_level` keeps its meaning; motor.toml ships for Variant A (`pul = 18`, `dir = 17`, `ena = 27`, `index = "none"`, `alm = 22`, `estop = 25`).

### ENA on GPIO 27: polarity, sequence and boot state

- Polarity, from the manual: ENA is optional and "default no connection" (p. 3), and the troubleshooting table's fix for "The drive is disabled" is to keep ENA+ and ENA- unconnected (p. 12). So ENA opto dark = enabled, ENA opto lit = disabled. Wired direct, GPIO 27 high lights the opto: **GPIO 27 high = disabled, low = enabled, `ena_enable_level = 0`**. The manual never states the lit state in one sentence; this is the reading of p. 3 and p. 12 together, and TEST-PLAN-CL57T.md step 2 checks it (shaft free with the software idle).
- What the software does: GPIO 27 is driven high (disabled, shaft free) the moment `froggy_motor.py` starts, at idle, at the end of every run, on Stop, on e-stop, on a driver ALM and on exit (pigpiod keeps the level after the program ends). A run drives it low, waits 200 ms, then sets DIR, then starts pulses, as the sequence chart asks (p. 10: ENA at least 200 ms ahead of DIR, DIR at least 2 us ahead of PUL).
- Boot state: GPIO 27 comes out of reset as an input with its default pull-down, so the ENA opto is dark and **the driver is enabled from power-up until the software first runs**. It holds the shaft at standstill current; it cannot move, because PUL (GPIO 18) is also an input pulled low, so there are no pulses. That is the same as the original NPN design's boot behaviour. If the shaft must be free while the Pi boots, ENA needs an inverting stage (for example ENA+ to 5 V, ENA- to GND, and a 2N7000 whose drain shorts ENA+ to GND when GPIO 27 is high, so the opto is lit, and the driver disabled, while the GPIO sits low at boot; then `ena_enable_level = 1`). That is a hardware change, not a software one, and it is not planned.

Grounds: the Pi ground meets the driver only at PUL-, DIR-, ENA- and COM-, all on the isolated opto side, so the 24 V minus stays separate. The optional encoder tap is the one thing that ties Pi GND to EGND (the driver's logic ground, probably common with the 24 V minus; the manual does not say). Fine on a one-supply bench; meter EGND to P4 GND with power off first so you know.

## Is 3.3 V enough? (Variant A numbers)

What the manual asks for (p. 1, 3, 7): 7 to 16 mA through each opto (10 typical), high level 3.5 to 5 V with S3 on 5V, low under 0.5 V, pulses at least 1 us wide.

- **High level:** a Pi GPIO high is 3.3 V nominal and a little lower under load. That is under the manual's 3.5 V minimum, so on paper Variant A is outside the spec by about 0.2 to 0.4 V.
- **Opto current at 3.3 V:** the manual does not give the internal resistor. If it is a plain resistor plus an LED (about 1.2 V), 10 mA at 5 V means roughly 380 ohm, and then a 3.3 V GPIO (about 3.0 V at the pin under load) gives (3.0 - 1.2) / 380 = **about 5 mA**. Across the manual's 7 to 16 mA range at 5 V the same estimate gives 3 to 8 mA at 3.3 V. So most likely just under the 7 mA minimum. This is my estimate from the spec table, not a measured or published number.
- **The vendor's own note:** the CL42T(V4.1) manual, same input stage revision, says on p. 3 that 3.3 V "can also work, but it will make the motor take more steps and reduce the accuracy". So the vendor expects it to run, with some risk of miscounted pulses.
- **Pi GPIO current:** about 5 mA per pin is inside a Pi GPIO's default 8 mA drive strength (settable 2 to 16 mA), and three pins (PUL, DIR, ENA) at about 5 mA are well inside the roughly 50 mA total the Raspberry Pi documentation suggests for the 3.3 V GPIO bank (Pi numbers from memory, not re-checked today). If the pin sags under load, `pigs pads 0 16` raises the drive strength of GPIO 0 to 27 to 16 mA, which keeps the high level closer to 3.3 V.
- **Speed:** 600 RPM at 3200 pulses/rev is 32 kHz, 15.6 us high and low, far above the 1 us minimum (p. 1).

**What a missed pulse looks like:** the CL57T closes the loop on the pulses it actually sees. A pulse the opto misses is simply never commanded, so the driver does not alarm: the shaft just turns slower (or goes less far) than the Pi commanded. There is no hall sensor on this bench, so the Pi cannot see that. cl-test says so in its verdict ("shaft speed unmeasured ... the driver followed every pulse it received, but pulses missed at its 3.3 V inputs would not show"). What the bench can still check: a tape flag on the shaft at 60 RPM should make exactly 1 rev/s (count 30 turns in 30 s by eye or phone video), and over a timed hold the turns counted should match the commanded revolutions. A shortfall with no alarm means go to Variant B. For a real number, fit the optional encoder tap (below); cl-test then reports measured RPM and tracking error.

## Variant B, only if Variant A misses pulses: two 2N7000 low-side switches

This is the wiring the DM320T bench used, moved to the CL57T's Fig. 5 open-collector hookup (p. 8). GPIO high turns the FET on, the drain pulls PUL- low and the opto lights: the same sense as Variant A, so nothing in the software changes.

| Signal | Pi 3 B+ pin | Through | Driver terminal | Manual page |
|---|---|---|---|---|
| PUL step | GPIO 18 (pin 12) | 220 ohm to 2N7000 Q1 gate, 10 k gate to GND, Q1 drain to PUL-, source to Pi GND | PUL- | p. 3 (3.1), p. 8 (Fig. 5) |
| PUL opto supply | 5 V (pin 2) | direct | PUL+ | p. 3, p. 8 (Fig. 5) |
| DIR | GPIO 17 (pin 11) | 220 ohm to 2N7000 Q2 gate, 10 k gate to GND, Q2 drain to DIR-, source to Pi GND | DIR- | p. 3, p. 8 (Fig. 5) |
| ENA (optional third FET) | GPIO 27 (pin 13) | same stage, Q3 drain to ENA-, ENA+ to Pi 5 V. Note the FET does not invert the sense: GPIO high still lights the opto (disabled) | ENA- | p. 3, p. 8 (Fig. 5) |
| DIR opto supply | 5 V (pin 2) | direct | DIR+ | p. 3, p. 8 (Fig. 5) |
| FET sources | GND (pin 6) | direct | | |

Everything else (S3 on 5V, ENA, ALM divider, motor, encoder, supply) is as in Variant A. 2N7000 legs, flat face toward you, legs down: Source, Gate, Drain left to right (pins 1, 2, 3; onsemi data sheet p. 1).

- Current: the opto now sees the full 5 V, so about 10 mA, inside 7 to 16 mA.
- Gate margin: onsemi lists the 2N7000's gate threshold at 0.8 V min, 2.1 V typical, 3 V max (VDS = VGS, ID = 1 mA; `2N7000-onsemi-datasheet.pdf`, electrical characteristics). A typical part at 3.3 V of gate drive conducts far more than 10 mA; a worst-case 3.0 V part has only 0.3 V of overdrive and could leave the opto dim. The two bench parts are known good from the DM320T.
- Check before driver power: force DIR high (`pigs w 27 1`), measure DIR- to Pi GND; under 0.3 V means the FET is fully on. Then `pigs w 27 0`. Same for PUL with `pigs w 18 1` before any PWM.

**Permanent harness note:** for the build, the 74AHCT125 already on the parts list is the clean answer. Powered from 5 V it has TTL inputs (a 3.3 V GPIO is a solid high) and full 0 to 5 V outputs. Wire it sourcing, Fig. 6 style (p. 8): PUL+ to 1Y, PUL- to Pi GND, DIR+ to 2Y, DIR- to Pi GND; 1OE and 2OE (pins 1 and 4) to GND, VCC pin 14 to 5 V, GND pin 7 to Pi GND, 10 k pull-downs on 1A and 2A (pins 2 and 5) so nothing steps while the Pi boots, 0.1 uF across VCC and GND. Same logic sense again.

## The ALM divider (the only parts added in Variant A)

Why it is needed: ALM is an opto transistor output rated for 5 to 24 V (p. 3), the Pi's inputs are 3.3 V only, and reading the alarm is the whole point of moving to closed loop. The vendor diagram marks ALM as optional; for us it is wired.

The output is a transistor between ALM and COM- (common cathode), rated 5 to 24 V and 100 mA (p. 3). The pull-up goes to 5 V and a divider brings it down: 10 k from 5 V (pin 4) to the ALM wire and 15 k from the ALM wire to GND (pin 14), both at the Pi end, and the same wire to GPIO 22. Transistor off: 5 x 15 / 25 = 3.0 V (a clean high, under 3.3 V). Transistor on: about 0.2 V. It sinks only 0.5 mA. This is the manual's sinking hookup (Fig. 7, p. 9) with the divider as the load. The software also turns on the Pi's weak internal pull-down, so a loose ALM wire reads 0, which the software treats as an alarm and refuses to start: fail safe.

Polarity: the manual says only that the impedance flips "depending on configuration" (p. 9). A LinuxCNC forum user reports the CL57 ALM switch is closed in alarm, which with this wiring reads 0, so the software default is `alm_active_level = 0`. That is a forum report, not the manual. TEST-PLAN-CL57T.md step 2 proves it on the bench with no motion (power up with the motor unplugged, which the manual says gives fault 4). If `status` says ALARM while the red LED is off, set `alm_active_level = 1`.

### Encoder tap (optional)

The driver keeps the encoder to itself. To see real shaft speed on the Pi, tap EA+ in parallel: a 10 k series resistor into a 74LVC1G17 (or 74LVC125) powered from the Pi's 3.3 V, whose inputs tolerate 5 V, then to GPIO 5. A plain resistor divider is not good enough here because an encoder line driver's high level can sit as low as about 3.4 V, which a divider would bring under the Pi's input threshold. The 10 k keeps the load on the encoder line negligible. The software counts EA+ rising edges, 1000 per rev, so 600 RPM is 10 kHz of callbacks, which pigpio handles on a Pi 3 B+ but not with much to spare.

## What the Pi can and cannot read from the CL57T-V41

| Data | Path | Status |
|---|---|---|
| Fault (ALM) | GPIO 22 through the divider | read every 10 ms plus an edge callback; the ramp stops at once and the cause is reported |
| In position (PEND) | none | the CL57T-V41 has no PEND terminal (p. 3); `status` and cl-test print "not wired" |
| Brake (BRK) | optional GPIO 23 | logged raw if wired; the manual does not say when it switches |
| Shaft speed, following error | optional encoder tap on GPIO 5 | speed over 100 ms windows; following error = Pi pulse count minus encoder count |
| Position error, actual speed, current inside the driver | P5 RS232 | protocol not published; not implemented |
| Hall index speed | none | no hall sensor on this bench; `index = "none"`; nothing depends on it |

## Pi software notes

- Bench Pi since 2026-11-09: Raspberry Pi 3 B+ on Raspberry Pi OS Lite 64-bit (Debian 13 trixie), reached as `$FROGGY_PI_USER@<host>` over SSH. `froggy-motor` and `deploy.sh` try `$FROGGY_PI`, then froggy-pi.local, then `~/.config/froggy/pi-host`. No camera on this Pi: motor.toml sets `camera = false`, the UI hides the preview and starts no rpicam-vid.
- pigpio is not installed on the new card yet. deploy.sh tries apt first; if trixie has no pigpio package it installs the vendored bookworm arm64 debs (WIRING.md lists them), which only depend on libc6 and python3. Not tried on trixie yet, so watch the first deploy's output.
- GPIO 18 hardware PWM is the same block on the Pi 3 B+ as before.
