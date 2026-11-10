# Third-party software

The four original Raspberry Pi pigpio packages in `motor/debs/` are version `1.79-1+rpt1`. Their embedded `/usr/share/doc/<package>/copyright` files identify the upstream work as Unlicense/public domain and permit source and binary redistribution. Their unchanged copyright notice is also included as [motor/debs/COPYRIGHT](motor/debs/COPYRIGHT). Package contents and SHA256 checksums are unchanged.

Python packages installed through the requirements files, Raspberry Pi OS images downloaded during provisioning, and tools installed separately (including FFmpeg, Tailscale and OpenSSH) remain under their respective licenses. OS images, credentials and generated boot files are not committed here.

The optional motor-tone demo includes only generic scales and arpeggios. No commercial melody transcription is included in the published branch.

## Motor control page libraries in `motor/ui/vendor/`

| Folder | Library | Licence |
| --- | --- | --- |
| `uplot/` | uPlot by Leon Sorokin (live RPM chart) | MIT, full text in `LICENSE` |
| `jspdf/` | jsPDF 2.5.2 by James Hall and yWorks GmbH (trial PDF report) | MIT, full text in `LICENSE` |
| `tabler-icons/` | @tabler/icons 3.51.0 subset and sprite | MIT, full text in `LICENSE` |

Each folder has a `VERSION.txt` with the source URL. They are served locally by the Pi; the page makes no network requests at runtime.

## Vendor manuals cited by `motor/WIRING-CL57T.md`

These PDFs are not redistributed here. Download them from their owners:

| Document | Owner | Source |
| --- | --- | --- |
| CL57T(V4.0) closed loop stepper driver user manual | STEPPERONLINE | https://www.stepperonline.co.uk/download/CL57T_V4.0.pdf |
| CL57T(V4.1) user manual (not downloadable at the time of writing, see WIRING-CL57T.md) | STEPPERONLINE | https://files.omc-stepperonline.com/CL57T-V41_Manual.pdf |
| CL42T(V4.1) user manual (source of the P5 tuning port pinout) | STEPPERONLINE | https://forum.linuxcnc.org/media/kunena/attachments/36700/CL42T-V41Manual2.pdf |
| 17HS19-2004D-E1000 motor drawing and datasheet | STEPPERONLINE | https://www.stepperonline.co.uk/download/17HS19-2004D-E1000.pdf |
| 2N7000, 2N7002, NDS7002A data sheet | onsemi | https://www.onsemi.com/download/data-sheet/pdf/nds7002a-d.pdf |

The crops and pictures in `motor/schematic-img/` come from these manuals, a STEPPERONLINE help article, the Raspberry Pi documentation (CC BY-SA 4.0) and Wikimedia Commons (CC BY-SA 4.0). `motor/schematic-img/sources.txt` lists each source, licence and checksum. Vendor images stay under their owners' terms and are included only as wiring references for this student project.

Publication does not assign a new license to the project-authored code or claim ownership of third-party dependencies.

## Vendor CAD models in `model/cad/vendor/`

The V12.5 CAD build in `model/cad/` imports the manufacturer and community models below. Each folder keeps the `source.json` it came with in froggy-cad (URL, author, licence notes), and `model/cad/vendor/manifest.json` holds the rows for the two flat files. They are redistributed here only so the ROB 450 capstone team (Team 03) can rebuild and simulate the turntable; they remain under their owners' terms and are not covered by anything else in this repository.

| File(s) | Part | Owner | Source | Terms as recorded |
| --- | --- | --- | --- | --- |
| `motion/stepperonline_17hs19_2004d_e1k/17HS19-2004D-E1K.step` (+ dimension and torque-curve SVG) | 17HS19-2004D-E1K closed-loop NEMA 17 motor | STEPPERONLINE | https://www.omc-stepperonline.com/nema-17-closed-loop-stepper-motor-52ncm-73-64oz-in-with-encoder-1000ppr-4000cpr-17hs19-2004d-e1k | vendor CAD download |
| `drive/stepperonline-cl42t-v41-driver.step` | CL42T-V41 closed-loop driver | STEPPERONLINE | https://www.omc-stepperonline.com/closed-loop-stepper-driver-v4-1-0-3-0a-24-48vdc-for-nema-11-14-17-stepper-motor-cl42t-v41 | vendor model |
| `motion/gobilda_1614_pillow_block/1614-0016-4008.step` | 1614 dual-bearing pillow block | goBILDA | https://www.gobilda.com/dual-bearing-pillow-block-8mm-rex-bore/ | vendor CAD download |
| `motion/gobilda_1313_hyper_hub/1313-1632-4008.step` | 1313 Hyper Hub, 8 mm REX | goBILDA | https://www.gobilda.com/1313-series-hyper-hub-8mm-rex-bore/ | vendor CAD download |
| `motion/gobilda_2106_rex_shaft_72/2106-4008-0720.step` | REX shaft 8 x 72 | goBILDA | https://www.gobilda.com/8mm-rex-shaft-with-e-clip-stainless-steel-72mm-length/ | vendor CAD |
| `motion/gobilda_3422_rex_pulley_20/3422-4008-0020 assembly.STEP` | 20T GT2 REX pulley | goBILDA | https://www.gobilda.com/2mm-pitch-gt2-pinion-timing-pulley-8mm-rex-bore-20-tooth/ | vendor CAD download |
| `hardware/gobilda_1522_8x6/1522-0010-0060.step` | 1522 spacer 8 ID x 6 | goBILDA | https://www.gobilda.com/1522-series-8mm-id-spacer-10mm-od-6mm-length-4-pack/ | vendor CAD |
| `hardware/gobilda_2807_8x1/2807-0811-1000.step` | 2807 shim 8 x 11 x 1 | goBILDA | https://www.gobilda.com/2807-series-stainless-steel-shim-8mm-id-x-11mm-od-1mm-thickness-12-pack/ | vendor CAD |
| `motion/creality_ender_vwheel/` (Rollers, 625pillow, M5 washer1 STEPs, LICENSE) | Ender-3 POM V wheel and 625 bearings | Creality | https://github.com/Creality3DPrinting/Ender-3 | GPL-3.0, licence file included in the folder |
| `electronics/ws2812_ring_16/neopixel_ring_16.step` | NeoPixel Ring 16 | Adafruit Industries | https://github.com/adafruit/Adafruit_CAD_Parts | MIT, Copyright (c) 2016 Adafruit Industries |
| `electronics/rpi-zero-2w.step` | Raspberry Pi Zero 2 W board | AndrewK0val (community model) | https://github.com/AndrewK0val/CAD-Models/blob/master/Raspberry%20Pi%20Zero%202%20W.STEP | community CAD model |
| `electrical/baco_lbx10510/lbx10510.stp` (+ TraceParts attribute file) | LBX10510 22 mm e-stop control station | BACO Controls, via TraceParts | https://nemasupply.com/products/22-mm-control-stations-non-illuminated-push-turn-40mm-mushroom-head-emegency-stop-e-stop | manufacturer CAD download for design use (TraceParts terms) |
| `opensource/3square-tree-frog/agalychnis-*-decimated-*.stl` | Red-eyed tree frog (Agalychnis callidryas) mesh, decimated | 3square | https://www.printables.com/model/1484159-red-eyed-tree-frog-agalychnis-callidryas | CC BY-SA; the decimated meshes are derived from 3square's model and shared under the same licence |

The other `source.json` folders (Olimex belt and pulley, RAF spacer, magnets, standoff, panel parts, stock tube, netting) carry metadata only; those parts are modelled in code from published dimensions and no vendor file is included. The rebuilt exports in `model/exports/` contain geometry from all of the models above and carry the same terms.
