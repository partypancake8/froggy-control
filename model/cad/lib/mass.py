#!/usr/bin/env python3
"""mass: the machine's mass from a froggy-cad sheet.json.

  python3 lib/mass.py out/<name>/sheet.json

Python 3 stdlib only (like lib/designcheck.py). Mass per component = volume_mm3 x density (g/cm3) from
lib/materials.py MATERIALS (stdlib-only module, imported directly), except the bought parts in
KNOWN_MASS_G, whose modelled solid is a filled envelope of a hollow housing (or a sleeve standing in for
netting): those take the catalogue mass. Other bought (v_) parts that are box stand-ins keep the volume
estimate and are flagged `estimated: true`.

Prints ONE JSON object:
  {"command": "mass", "sheet", "total_g", "by_module" {module: g}, "by_material" {material: g},
   "by_kind" {kind: g}, "heaviest" [20 x {name, display_name, module, material, g, source, estimated}],
   "rail_2020_mm", "rail_2020_g", "rail_2020_count", "bracket_count", "known_mass_parts", "estimated_parts"}

`report_stderr(sheet)` prints the same totals as one human line (the bin/cad build / check hook in
lib/sheet.py write()).
"""
import json
import os
import re
import sys
from collections import defaultdict

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
from lib.materials import MATERIALS  # noqa: E402  (stdlib only: no build123d, no vtk)

# Catalogue masses (g) for bought parts whose model volume is not their mass, matched as a name prefix,
# first match wins. Source per row.
KNOWN_MASS_G = [
    ("v_gt2_rex_pulley_20", 10.0, "vendor/motion/gobilda_3422_rex_pulley_20/source.json"),
    # vendor/motion/stepperonline_17hs19_2004d_e1k/source.json mass_g 430 (0.43 kg physical spec, UK page);
    # the modelled motor is a filled solid (976 g at steel density)
    ("v_nema17_17hs19", 430.0, "vendor/motion/stepperonline_17hs19_2004d_e1k/source.json"),
    # vendor/motion/stepperonline_17hs08_1004d_e1k/source.json mass_g 270 (the brief's 'about 220 g' is the bare
    # 17HS08 without the encoder)
    ("v_nema17_17hs08", 270.0, "vendor/motion/stepperonline_17hs08_1004d_e1k/source.json"),
    # vendor/motion/gobilda_1614_pillow_block/source.json mass_g 24 for the whole block; its two bearings are
    # separate components (about 4 g each by volume), so the housing row is 24 - 8 = 16
    ("v_gobilda_1614_pillow_block", 16.0, "vendor/motion/gobilda_1614_pillow_block/source.json (24 g less bearings)"),
    # MEAN WELL LRS-50-24 datasheet: weight 0.18 kg; modelled as a filled 99 x 82 x 30 steel box (1.9 kg)
    ("v_psu_lrs_50_24", 180.0, "MEAN WELL LRS-50 datasheet, weight 0.18 kg"),
    # vendor/hardware/cast_bracket_2020_28/source.json mass_g 12 (cast zinc alloy, modelled as steel envelope)
    ("v_cast_bracket_2020", 12.0, "vendor/hardware/cast_bracket_2020_28/source.json"),
    # vendor/electrical/baco_lbx10510/source.json mass_g 160 (assumed there); the cover screws ride in it
    ("v_estop_baco_lbx10510_cover_screws", 0.0, "included in v_estop_baco_lbx10510"),
    ("v_estop_baco_lbx10510", 160.0, "vendor/electrical/baco_lbx10510/source.json (assumed)"),
    # vendor/electronics/anker-321-power-bank-a1112/source.json mass_g 118
    ("v_power_bank", 118.0, "vendor/electronics/anker-321-power-bank-a1112/source.json"),
    # vendor/motion/gobilda_1313_hyper_hub/source.json mass_g 30
    ("v_gobilda_1313_hub", 30.0, "vendor/motion/gobilda_1313_hyper_hub/source.json"),
    # vendor/motion/gobilda_2106_rex_shaft_54/source.json mass_g 16
    ("v_gobilda_rex_shaft_2106_72", 22.0, "vendor/motion/gobilda_2106_rex_shaft_72/source.json"),
    ("v_gobilda_rex_shaft_2106_54", 16.0, "vendor/motion/gobilda_2106_rex_shaft_54/source.json"),
    # vendor/motion/gobilda_2920_collar/source.json mass_g 8.3
    ("v_shaft_collar_gobilda_2920", 8.3, "vendor/motion/gobilda_2920_collar/source.json"),
    # vendor/coupling/ruland_mjc19_5_8/source.json mass_g 14
    ("v_coupling_ruland_mjc19", 14.0, "vendor/coupling/ruland_mjc19_5_8/source.json"),
    # vendor/motion/umbb_pu_8x22x5/source.json mass_g 5.5 (bearing + PU tread, assumed there)
    ("v_roller_umbb_pu", 5.5, "vendor/motion/umbb_pu_8x22x5/source.json"),
    # vendor/hardware/din653_m5x12_thumb/source.json mass_g 7
    ("hw_m5x12_thumb", 7.0, "vendor/hardware/din653_m5x12_thumb/source.json"),
    # vendor/textile/sailrite_no_see_um_netting/source.json mass_g 25 (the sleeve solid is a thick stand-in)
    ("s_mesh_sleeve", 25.0, "vendor/textile/sailrite_no_see_um_netting/source.json"),
    # I030 cage panels: the netting is modelled 0.3 thick; real fabric 0.81 oz/sq yd = 27.5 g/m2 (source.json) x the
    # modelled outline (side 60157 mm2, top 42137 mm2)
    ("s_cage_mesh_top", 1.16, "vendor/textile/sailrite_no_see_um_netting/source.json (27.5 g/m2 x 0.0421 m2)"),
    ("s_cage_mesh_", 1.65, "vendor/textile/sailrite_no_see_um_netting/source.json (27.5 g/m2 x 0.0602 m2)"),
]

# bought parts whose model is a filled box or plain stand-in and that have no catalogue mass here
_STANDIN = re.compile(r"^v_(cl42t_driver|buck_module|relay_delay|iec_inlet|usb_panel|rpi_zero|camera|led_driver|pico)")
_RAIL = re.compile(r"\b2020 x (\d+(?:\.\d+)?)")
FASTENER_KINDS = {"screw", "nut", "washer", "set_screw", "insert"}


def _known(name):
    for pre, g, src in KNOWN_MASS_G:
        if name.startswith(pre):
            return g, src
    return None


def component_mass(c):
    """(g, source, estimated) for one sheet.json component row."""
    k = _known(c["name"])
    if k:
        return k[0], k[1], False
    rho = MATERIALS.get(c.get("material"), {}).get("density", 1.0)
    g = c.get("volume_mm3", 0.0) / 1000.0 * rho
    est = bool(_STANDIN.match(c["name"])) or c.get("material") not in MATERIALS
    return g, f"volume x {c.get('material')} {rho} g/cm3", est


def rail_length(c):
    """Length of a 2020 rail component (mm) or None: length_mm meta if the sheet carries it, else the display
    name '2020 x <length>'."""
    if c.get("length_mm") and "2020" in (c.get("display_name") or c["name"]):
        return float(c["length_mm"])
    m = _RAIL.search(c.get("display_name") or "")
    return float(m.group(1)) if m else None


def summarise(sheet):
    comps = sheet["components"]
    by_module, by_material, by_kind = defaultdict(float), defaultdict(float), defaultdict(float)
    rows, known, estimated = [], [], []
    rail_mm = rail_g = 0.0
    rail_n = 0
    brackets = 0
    for c in comps:
        g, src, est = component_mass(c)
        by_module[c.get("module", "?")] += g
        by_material[c.get("material", "?")] += g
        by_kind[c.get("kind", "?")] += g
        if _known(c["name"]):
            known.append(c["name"])
        if est:
            estimated.append(c["name"])
        L = rail_length(c)
        if L is not None:
            rail_mm += L
            rail_g += g
            rail_n += 1
        if c.get("kind") == "bracket":
            brackets += 1
        rows.append(dict(name=c["name"], display_name=c.get("display_name"), module=c.get("module"),
                         material=c.get("material"), g=round(g, 1), source=src, estimated=est))
    rows.sort(key=lambda r: -r["g"])
    r1 = lambda d: {k: round(v, 1) for k, v in sorted(d.items(), key=lambda kv: -kv[1])}  # noqa: E731
    return dict(command="mass", sheet=sheet.get("name"), total_g=round(sum(by_module.values()), 1),
                by_module=r1(by_module), by_material=r1(by_material), by_kind=r1(by_kind), heaviest=rows[:20],
                rail_2020_mm=round(rail_mm, 2), rail_2020_g=round(rail_g, 1), rail_2020_count=rail_n,
                bracket_count=brackets, known_mass_parts=len(known), estimated_parts=estimated)


def report_stderr(sheet):
    """One human line for bin/cad build / check (never raises)."""
    try:
        m = summarise(dict(sheet, components=sheet["components"]))
        mods = ", ".join(f"{k} {v / 1000:.2f}" for k, v in m["by_module"].items())
        print(f"mass: {m['total_g'] / 1000:.2f} kg ({mods}); 2020 {m['rail_2020_mm']:.0f} mm "
              f"in {m['rail_2020_count']} rails, {m['rail_2020_g']:.0f} g; {m['bracket_count']} brackets "
              f"(python3 lib/mass.py <sheet.json>)", file=sys.stderr, flush=True)
    except Exception as e:  # the mass line must never break a build
        print(f"mass: not computed ({e!r})", file=sys.stderr, flush=True)


def main(argv):
    if len(argv) != 1:
        print(__doc__, file=sys.stderr)
        return 2
    with open(argv[0]) as fh:
        sheet = json.load(fh)
    out = summarise(sheet)
    out["sheet"] = os.path.abspath(argv[0])
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
