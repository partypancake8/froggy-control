"""Readable names for the exported tree (STEP labels, GLB nodes, 3MF objects, sheet.json).

Internal keys (A.parts names, joint names) stay the identity for checks, joints, ICD rows and
sheet.json `name`. This module adds a naming layer on top:

- every component gets a `display_name` ("Pillow block, goBILDA 1614-0016-4008", "Plate A, 6 mm
  aluminium", "M5 x 25 SHCS") and a `group` [module label, sub-assembly label];
- every joint group (the screw, nut, washer or insert sharing a meta `joint`) gets a display name
  ("M5 x 14 BHCS + T-nut, plate A to cross rail right 1 of 2") and sits in the "Fasteners" folder of
  the sub-assembly that owns it (the first joined part that is not a rail or a panel);
- multi-body parts name their bodies ("Inner race", "Cover screw 1 of 4").

Rules, in order: OVERRIDES (exact key) > PART_RULES (key regex, per module) > fallback (humanised
key, flagged `rule: "fallback"` so `bin/cad tree` reports it). Repeated names get "n of N" after the
first comma segment ("Roller block 1 of 3, PETG"); joint groups get it at the end. No em dashes:
commas, "x" and "+" only.

`tree_path` (module, sub-assembly[, "Fasteners", joint group], part) is the address used by
cad-fusion to match occurrences to sheet rows (`match_entry`, also run inside Fusion).
"""
import re
from collections import Counter, defaultdict

PATH_SEP = " > "
FASTENERS = "Fasteners"
FASTENER_KINDS = ("screw", "nut", "insert", "washer", "set_screw")
INTERNAL_PREFIXES = ("v_", "s_", "lp_", "fp9_", "hw_", "j_")

MODULE_LABELS = {
    "frame": "Frame", "feet": "Feet", "drive_base": "Drive cartridge", "rotor": "Rotor",
    "electrical": "Electrical", "plinth": "Plinth", "cage": "Cage",
}

CORNERS = {"pxpy": "back right", "pxny": "front right", "nxpy": "back left", "nxny": "front left"}
SIDES = {"back": "back", "front": "front", "right": "right", "left": "left"}
AL = "aluminium"
FAB = "3.175 mm aluminium"

# exact key -> (sub-assembly, display name). Display names may omit "n of N"; it is added.
OVERRIDES = {
    # drive cartridge
    "v_nema17_17hs19_2004d_e1k": ("Motor", "Stepper motor, STEPPERONLINE 17HS19-2004D-E1K"),
    "v_nema17_shaft": ("Motor", "Motor shaft, 5 mm"),
    "lp_motor_plate": ("Motor", f"Motor plate, {FAB}"),
    "fp9_motor_cradle": ("Motor", "Motor cradle, PETG, four tabs to the cross rails"),
    "fp9_motor_plate": ("Motor", "Motor plate, PETG 12 mm slab with four tabs to the cross rails"),
    "v_coupling_ruland_mjc19_5_8": ("Motor", "Jaw coupling, Ruland MJC19-5-8"),
    "v_gt2_motor_pulley_20_5": ("Belt drive", "Motor pulley, 20T GT2, 5 mm bore on D shaft"),
    "v_gt2_rex_pulley_20": ("Belt drive", "REX pulley, goBILDA 3422-4008-0020, 20T GT2"),
    "v_gt2_belt_100_6": ("Belt drive", "Timing belt, 100 mm loop x 6 mm, 50 teeth GT2"),
    "lp_plate_a": ("Plate A", "Plate A, 6 mm aluminium"),
    "v_gobilda_1614_pillow_block": ("Bearing block", "Pillow block, goBILDA 1614-0016-4008"),
    "v_gobilda_1614_bearing_lower": ("Bearing block", "Pillow block bearing lower, goBILDA 1614"),
    "v_gobilda_1614_bearing_upper": ("Bearing block", "Pillow block bearing upper, goBILDA 1614"),
    # rotor
    "v_shaft_collar_gobilda_2920": ("Spindle", "Shaft collar, goBILDA 2920-0001-4008"),
    "v_gobilda_rex_shaft_2106_54": ("Spindle", "REX shaft 8 x 54, goBILDA 2106-4008-0540"),
    "v_gobilda_rex_shaft_2106_72": ("Spindle", "REX shaft 8 x 72, goBILDA 2106-4008-0720"),
    "v_retainer_spacer_8x6": ("Spindle", "Backup spacer, goBILDA 1522-0010-0060"),
    "v_retainer_shim_8x1": ("Spindle", "Backup shim, goBILDA 2807-0811-1000"),
    "v_retainer_race_spacer_8x6": ("Spindle", "Race spacer, goBILDA 1522-0010-0060"),
    **{f"v_retainer_race_shim_8x1_{k}": ("Spindle", "Race shim, goBILDA 2807-0811-1000") for k in range(1, 7)},
    "hw_retainer_washer_upper_m4": ("Spindle", "Upper end washer, M4 ISO 7093"),
    "hw_retainer_washer_lower_m4": ("Spindle", "Lower end washer, M4 ISO 7093"),
    "v_gobilda_1313_hub": ("Spindle", "Hyper hub, goBILDA 1313-1632-4008"),
    "lp_carrier_disc": ("Carrier disc", "Carrier disc, 6 mm aluminium"),
    "s_drum_tube": ("Drum", "Acrylic tube, 140 tall"),
    "s_top_disc": ("Drum", "Top disc, 186 x 3.2 acrylic"),
    "lp_ring_b": ("Drum", f"Ring B tube foot, {FAB}"),
    "lp_ring_top": ("Drum", f"Top bezel ring, {FAB}"),
    "fp9_tube_seat_bottom": ("Drum", "Tube seat bottom, PETG"),
    "fp9_tube_seat_top": ("Drum", "Tube seat top, PETG"),
    "v_ir_ring_light": ("Camera hub", "IR ring light, flat SMD 10 LED 850 nm"),
    "v_led_ring_ws2812": ("Camera hub", "LED ring, WS2812 16 (Adafruit NeoPixel 16 footprint)"),
    "fp9_camera_wedge_bridge": ("Camera hub", "Camera mount, PETG wedge saddle over the power bank"),
    "v_camera_uc261": ("Camera hub", "Camera, Arducam UC-261"),
    "v_frog_wall_body": ("Test frogs", "Wall frog body, Agalychnis callidryas"),
    "v_frog_wall_eyes": ("Test frogs", "Wall frog eyes"),
    "v_frog_disc_body": ("Test frogs", "Disc frog body, Agalychnis callidryas"),
    "v_frog_disc_eyes": ("Test frogs", "Disc frog eyes"),
    # electrical
    "v_power_bank": ("Rotor electronics", "Power bank, Anker 321 A1112"),
    "v_charge_port": ("Rotor electronics", "USB-C charge port, Adafruit 6069"),
    "v_power_switch": ("Rotor electronics", "Power switch, TRU LAS4GQH-11"),
    "v_rpi_zero_2w": ("Rotor electronics", "Pi Zero 2 W"),
    "v_led_driver": ("Rotor electronics", "IR LED driver board"),
    "fp9_hall_bracket": ("Plate A", "Hall sensor bracket, PETG"),
    "v_index_hall_ky003": ("Plate A", "Hall sensor, Joy-it KY-003 A3144"),
    "v_cl42t_driver": ("Motor driver and PSU", "Closed-loop driver, STEPPERONLINE CL42T V4.1"),
    "v_psu_lrs_50_24": ("Motor driver and PSU", "Power supply, MEAN WELL LRS-50-24"),
    "fp9_controller_carrier": ("Controller", "Controller carrier, PETG"),
    "v_pico": ("Controller", "Raspberry Pi Pico"),
    "v_buck_module": ("Controller", "Buck converter module"),
    "v_relay_delay": ("Controller", "Delay-off relay module"),
    "fp9_estop_cradle": ("E-stop station", "E-stop cradle, PETG"),
    "v_estop_baco_lbx10510": ("E-stop station", "E-stop station, BACO LBX10510"),
    "v_estop_baco_lbx10510_cover_screws": ("E-stop station", "E-stop cover screws, BACO LBX10510"),
    "v_iec_inlet": ("Panel connectors", "IEC C14 fused switched inlet"),
    "v_usb_panel": ("Panel connectors", "USB-C panel mount"),
    # plinth
    "s_panel_front": ("Front panel", "Front panel, navy acrylic"),
    "s_panel_back": ("Side and back panels", "Back panel, navy acrylic"),
    "s_panel_right": ("Side and back panels", "Right panel, navy acrylic"),
    "s_panel_left": ("Side and back panels", "Left panel, navy acrylic"),
    "s_panel_deck": ("Deck panel", "Deck panel, navy acrylic"),
    "v_grommet_g2": ("Deck panel", "Cable grommet, 8 mm hole"),
    "s_panel_bottom": ("Bottom panel", "Bottom panel, navy acrylic"),
    # cage (I030, 2026-10-21): the top panel is one piece
    "fp9_cage_panel_top": ("Panel top", "Mesh panel top frame, PETG"),
    "s_cage_mesh_top": ("Panel top", "Mesh panel top, no-see-um netting"),
}

RAIL_ROLES = {"deck": ("Deck rails", "Deck rail"), "cross": ("Deck rails", "Cross rail"),
              "bot": ("Base rails", "Base rail"), "leg": ("Legs", "Leg")}
BRACKETS = {"deck_corner": "Deck corner bracket", "bot_corner": "Base corner bracket", "leg_top": "Leg top bracket",
            "upright_top": "Leg top bracket", "leg_bottom": "Leg foot bracket", "cross_end": "Cross rail end bracket",
            "s3_foot": "Strut foot bracket", "s3_tab_bot": "Ring tab bracket bottom",
            "s3_tab_top": "Ring tab bracket top"}


def _rail(key, p):
    role = str(p.meta.get("role", ""))
    head = role.split("_")[0]
    sub, noun = RAIL_ROLES.get(head, ("Rails", "Rail"))
    prof = "2020" if "2020" in key else "1515" if "1515" in key else ""
    where = ""
    if head in ("deck", "bot"):
        where = " " + role.split("_")[-1]
    elif head == "cross":
        where = " " + role.split("_")[-1]
    elif head == "leg":
        where = " " + CORNERS.get(role.split("_")[-1], "")
    L = p.meta.get("length_mm")
    size = f"{prof} x {L:g}" if L else prof
    return sub, f"{noun}{where.rstrip()}, {size}".strip(", ")


def _bracket(key, p, m):
    kind, corner = m.group(1), m.group(2)
    noun = BRACKETS.get(kind, kind.replace("_", " ").capitalize() + " bracket")
    where = " " + CORNERS[corner] if corner in CORNERS else ""
    sub = "Struts" if kind.startswith("s3_") else "Corner brackets"
    return sub, f"{noun}{where}, cast 2020"


# (module or None, key regex, fn(key, part, match) -> (sub, display))
PART_RULES = [
    (None, r"^v_motor_plate_spacer_", lambda k,p,m: ("Motor", "Metal spacer under the motor plate, 5 ID x 10 OD x 3 aluminium")),
    (None, r"^v_deck_hardpoint_", lambda k,p,m: ("Plate A" if p.module=="drive_base" else "Clamp tabs", "Metal deck spacer, 5 ID x 10 OD x 3 aluminium")),
    (None, r"^v_spacer_axle_10x5x1_", lambda k,p,m: ("Rim rollers", "Axle precision shim, 5 ID x 10 OD x 1 steel")),
    (None, r"^s_rail_(?:2020|1515)_", lambda k, p, m: _rail(k, p)),
    (None, r"^v_cast_bracket_2020_(s3_foot|s3_tab_bot|s3_tab_top|deck_corner|bot_corner|leg_top|upright_top|leg_bottom|cross_end)"
           r"_?(pxpy|pxny|nxpy|nxny|\d+)?$", _bracket),
    (None, r"^v_foot_bumper_(\w+)$", lambda k, p, m: ("Bumper feet", f"Bumper foot {CORNERS.get(m.group(1), '')}, "
                                                                       "rubber 20 x 8")),  # I029: on the leg axes
    # DL29 / I023 (2026-10-16): deck clamp tabs (the I022 tube struts were replaced by the I030 cube, 2026-10-21)
    (None, r"^lp_clamp_tab_(pxpy|pxny|nxpy|nxny)$",
     lambda k, p, m: ("Clamp tabs", f"Clamp tab {CORNERS[m.group(1)]}, 6 mm aluminium")),
    (None, r"^v_spacer_raf_m0690_5_al_\d+$", lambda k, p, m: ("Motor", "Spacer, RAF M0690-5-AL")),
    (None, r"^v_spacer_raf_m0669_5_al_\d+$", lambda k, p, m: ("Motor", "Spacer, RAF M0669-5-AL, 20 mm")),
    (None, r"^fp(?:9|12)_roller_block_\d+$", lambda k, p, m: ("Rim rollers", "Roller block, PETG")),
    (None, r"^v_roller_umbb_pu_8x22x5_\d+$", lambda k, p, m: ("Rim rollers", "Rim roller, UMBB PU 8 x 22 x 5")),
    (None, r"^v_spacer_axle_12x8x3_\d+$", lambda k, p, m: ("Rim rollers", "Axle spacer, 12 x 8 x 3 aluminium")),
    (None, r"^v_drum_post_\d+$", lambda k, p, m: ("Drum", "Drum post, aluminium")),
    (None, r"^v_index_magnet_6x3_n42_\d+$", lambda k, p, m: ("Carrier disc", "Index magnet, 6 x 3 N42")),
    (None, r"^v_pi_standoff_m2p5_\d+$", lambda k, p, m: ("Rotor electronics", "Pi standoff, M2.5 x 6 hex")),
    (None, r"^v_drv_standoff_m2p5_\d+$", lambda k, p, m: ("Rotor electronics", "LED driver standoff, M2.5 x 6 hex")),
    # I030 (2026-10-21): tube-post cube with magnet-on mesh panels. Sub-assemblies stay at or under the DL18 cap of 12:
    # Tube frame (posts, top rails, top corners), Feet (foot corners + their magnets), Top corner magnets, one per panel
    (None, r"^s_cage_post_(pxpy|pxny|nxpy|nxny)$",
     lambda k, p, m: ("Tube frame", f"Cage post {CORNERS[m.group(1)]}, 6061 tube 12 x 1 x {p.meta.get('length_mm', 0):g}")),
    (None, r"^s_cage_top_(back|front|right|left)$",
     lambda k, p, m: ("Tube frame", f"Cage top rail {m.group(1)}, 6061 tube 12 x 1 x {p.meta.get('length_mm', 0):g}")),
    (None, r"^fp9_cage_corner_top_(pxpy|pxny|nxpy|nxny)$",
     lambda k, p, m: ("Tube frame", f"Cage top corner {CORNERS[m.group(1)]}, PETG")),
    (None, r"^fp9_cage_corner_foot_(pxpy|pxny|nxpy|nxny)$",
     lambda k, p, m: ("Feet", f"Cage foot corner {CORNERS[m.group(1)]}, PETG")),
    (None, r"^hw_magnet_8x3_corner_foot_(pxpy|pxny|nxpy|nxny)_([xy])$",
     lambda k, p, m: ("Feet", f"Corner magnet foot {CORNERS[m.group(1)]} {m.group(2).upper()}, 8 x 3 N42")),
    (None, r"^hw_magnet_8x3_corner_top_(pxpy|pxny|nxpy|nxny)_([xyz])$",
     lambda k, p, m: ("Top corner magnets", f"Corner magnet top {CORNERS[m.group(1)]} {m.group(2).upper()}, 8 x 3 N42")),
    (None, r"^fp9_cage_panel_(back|front|right|left)_(lower|upper)$",
     lambda k, p, m: (f"Panel {m.group(1)}", f"Mesh panel {m.group(1)} {m.group(2)} half frame, PETG")),
    (None, r"^s_cage_mesh_(back|front|right|left)$",
     lambda k, p, m: (f"Panel {m.group(1)}", f"Mesh panel {m.group(1)}, no-see-um netting")),
    (None, r"^hw_magnet_8x3_panel_(back|front|right|left|top)_(lower|upper|top)_(\d)$",
     lambda k, p, m: (f"Panel {m.group(1)}", f"Panel magnet {m.group(1)} {m.group(2) if m.group(2) != 'top' else ''} "
                                             f"{m.group(3)}, 8 x 3 N42".replace("  ", " "))),
]

# multi-body part bodies: (part key regex, body suffix regex) -> name. Numbered suffixes get "n of N".
BODY_NAMES = {
    "hub_5": "Hub, 5 mm bore", "hub_8": "Hub, 8 mm bore", "spider": "Spider, elastomer",
    "inner": "Inner race", "outer": "Outer race", "tyre": "Polyurethane tyre",
    "pcb": "Camera PCB", "lens": "M12 lens", "fpc": "FFC connector",
    "board": "Ring board", "leds": "IR LEDs", "smd_led": "SMD IR LED, 3535", "emitter": "Emitter window",
    "pad": "Solder pad", "glass": "Lens front glass", "pi_pcb": "Pi PCB", "pi_part": "Component", "a3144": "A3144 hall switch", "breakout": "Breakout board",
    "head_40mm": "Mushroom head, 40 mm", "base": "Base", "cover": "Cover",
}

# fastener leaves: label stem (before _j_) -> spec name
HEADS = {"button": "BHCS", "shcs": "SHCS", "csk": "FHCS", "thumb": "thumb screw", "stud": "threaded stud"}
NUT_NAMES = {"slot_nut": ("T-nut", "T-nut"), "hex_nut": ("hex nut", "hex nut"), "nyloc": ("nyloc nut", "nyloc nut"),
             "insert": ("heat-set insert", "insert"), "washer": ("washer", "washer"),
             "nut_washer": ("washer", "washer")}

# joint groups without a joint record: key regex -> "what it joins" text
JOINT_TEXT = [
    (r"^j_s1_27_axle_\d+$", "rim roller axle"),
    (r"^j_s1_32_stud_\d+$", "drum post to ring B and carrier disc"),
]


def humanise(key):
    s = key
    for pre in INTERNAL_PREFIXES:
        if s.startswith(pre):
            s = s[len(pre):]
            break
    s = s.replace("_", " ").strip()
    return s[:1].upper() + s[1:]


def fastener_name(key):
    """'hw_m5x25_button_j_...' -> 'M5 x 25 BHCS'; 'hw_slot_nut_m5_j_...' -> 'M5 T-nut'."""
    if key.startswith("hw_retainer_washer_") and key in OVERRIDES:
        return OVERRIDES[key][1]
    stem = key.split("_j_")[0]
    s = stem[3:] if stem.startswith("hw_") else stem
    m = re.match(r"^m(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)_(\w+)$", s)
    if m:
        return f"M{m.group(1)} x {m.group(2)} {HEADS.get(m.group(3), m.group(3).replace('_', ' '))}"
    m = re.match(r"^shoulder_(\d+)x(\d+)_m(\d+)$", s)
    if m:
        return f"M{m.group(3)} shoulder screw {m.group(1)} x {m.group(2)}"
    m = re.match(r"^(slot_nut|hex_nut|nyloc|insert|nut_washer|washer)_m(\d+(?:\.\d+)?)$", s)
    if m:
        return f"M{m.group(2)} {NUT_NAMES[m.group(1)][0]}"
    return None


def _short(display):
    """'Plate A, 6 mm aluminium' -> 'plate A' (the part as named inside a joint group name)."""
    head = display.split(",")[0].strip()
    w = head.split(" ")[0]
    if len(w) > 1 and w[1:].islower() and w not in ("Pi",):
        head = head[:1].lower() + head[1:]
    return head


def _number(names, at_end=False):
    """{key: name} -> names with ' n of N' for every name used more than once (key order)."""
    cnt, seen, out = Counter(names.values()), Counter(), {}
    for k, n in names.items():
        if cnt[n] < 2:
            out[k] = n
            continue
        seen[n] += 1
        tag = f" {seen[n]} of {cnt[n]}"
        if at_end or "," not in n:
            out[k] = n + tag
        else:
            a, b = n.split(",", 1)
            out[k] = a + tag + "," + b
    return out


def _body_names(part_key, labels):
    raw = {}
    for lbl in labels:
        if lbl.startswith(part_key + "_"):
            suf = lbl[len(part_key) + 1:]
        else:                  # part "<stem>_cover_screws", bodies "<stem>_cover_screw_1": drop the shared stem
            stem = part_key[:part_key.rfind("_") + 1]
            while stem and not lbl.startswith(stem):
                stem = stem[:stem[:-1].rfind("_") + 1]
            suf = lbl[len(stem):] if stem else lbl
        if suf in BODY_NAMES:
            raw[lbl] = BODY_NAMES[suf]
            continue
        m = re.match(r"^(.*?)_?(\d+)$", suf)
        base = m.group(1) if m else suf
        raw[lbl] = BODY_NAMES.get(base) or (humanise(base) if base else humanise(suf))
    return _number(raw)


def name_assembly(A):
    """{'parts': {key: row}, 'joints': {joint key: display}, 'root': label, 'modules': {module: label}}.
    row = display_name, group [module label, sub], tree_path [...], node_name (unique), fastener, joint
    (display or None), bodies {body label: display}, rule (override | rule | fastener | fallback)."""
    modules = {p.module: MODULE_LABELS.get(p.module, humanise(p.module)) for p in A.parts.values()}
    rows, raw = {}, {}
    for k, p in A.parts.items():
        if p.kind in FASTENER_KINDS and p.meta.get("joint"):
            continue
        sub = disp = None
        rule = "override"
        if k in OVERRIDES:
            sub, disp = OVERRIDES[k]
        else:
            for mod, rx, fn in PART_RULES:
                if mod not in (None, p.module):
                    continue
                m = re.match(rx, k)
                if m:
                    sub, disp = fn(k, p, m)
                    rule = "rule"
                    break
        if k == "lp_motor_plate":
            sub, disp = "Motor", f"Motor plate, {p.solid.bounding_box().size.Z:g} mm aluminium"
        if k.startswith("v_rim_roller_creality_"):
            sub, disp = "Rim rollers", "Ender POM wheel kit, 2 x 625 bearings"
        if disp is None:
            sub, disp, rule = "Parts", humanise(k), "fallback"
        raw[k] = disp
        rows[k] = dict(group=[modules[p.module], sub], rule=rule)
    for k, d in _number(raw).items():
        rows[k]["display_name"] = d

    # joint groups
    groups = defaultdict(list)
    for k, p in A.parts.items():
        if p.kind in FASTENER_KINDS and p.meta.get("joint"):
            groups[p.meta["joint"]].append(k)
    rec = {j["name"]: j for j in A.joints if isinstance(j, dict) and "name" in j}
    jraw, jowner, jrule = {}, {}, {}
    order = [j for j in rec if j in groups] + [j for j in groups if j not in rec]
    for jk in order:
        hw = groups[jk]
        screws = [h for h in hw if A.parts[h].kind in ("screw", "set_screw")]
        others = [h for h in hw if h not in screws]
        spec = fastener_name(screws[0]) if screws else None
        extra = []
        for h in others:
            m = re.match(r"^hw_(slot_nut|hex_nut|nyloc|insert|nut_washer|washer)_m", h)
            nm = NUT_NAMES[m.group(1)][1] if m else humanise(h.split("_j_")[0])
            if nm not in extra:
                extra.append(nm)
        extra.sort(key=lambda s: s == "washer")       # nut or insert first, washer last
        head = " + ".join([spec or humanise(jk)] + extra)
        r = rec.get(jk)
        owner, what = None, None
        if r:
            ps = [q for q in r.get("parts", []) if q in A.parts]
            main = [q for q in ps if A.parts[q].kind not in ("rail", "panel")]
            owner = r.get("owner") if r.get("owner") in ps else (main or ps or [None])[0]   # a joint may name its owner
            names = [_short(rows[q]["display_name"]) for q in ps if q in rows]
            if owner in ps and len(names) == len(ps) and len(ps) >= 2:
                i = ps.index(owner)
                names = [names[i]] + names[:i] + names[i + 1:]
            what = " to ".join(names) if names else None
            jrule[jk] = "rule" if what else "fallback"
        else:
            for rx, txt in JOINT_TEXT:
                if re.match(rx, jk):
                    what = txt
                    break
            jrule[jk] = "override" if what else "fallback"
            n = re.search(r"_(\d+)$", jk)
            if jk.startswith("j_s1_27_axle_") and n:
                owner = next((q for q in (f"v_rim_roller_creality_4001070002_{n.group(1)}", f"v_roller_umbb_pu_8x22x5_{n.group(1)}") if q in A.parts), None)
            if jk.startswith("j_s1_32_stud_") and n:
                owner = f"v_drum_post_{n.group(1)}"
        if owner not in rows:
            owner = None
        if owner is None:      # fall back to the module of the screw
            mod = modules[A.parts[hw[0]].module]
            jowner[jk] = [mod, "Parts"]
            jrule[jk] = "fallback"
        else:
            jowner[jk] = list(rows[owner]["group"])
        jraw[jk] = head + (", " + what if what else "")
    jnames = _number(jraw, at_end=True)

    for jk, hw in groups.items():
        for h in hw:
            disp = fastener_name(h) or humanise(h.split("_j_")[0])
            rows[h] = dict(display_name=disp, group=list(jowner[jk]), joint=jnames[jk], joint_key=jk,
                           rule="fastener" if fastener_name(h) else "fallback")
    # loose fasteners without a joint (none in v9): their own sub-assembly
    for k, p in A.parts.items():
        if k not in rows:
            rows[k] = dict(display_name=fastener_name(k) or humanise(k), group=[modules[p.module], "Parts"],
                           rule="fallback")

    used = Counter()
    for k, p in A.parts.items():
        r = rows[k]
        r["fastener"] = "joint" in r
        r["tree_path"] = r["group"] + ([FASTENERS, r["joint"]] if r["fastener"] else []) + [r["display_name"]]
        node = r["display_name"] if not r["fastener"] else f"{r['display_name']}, {r['joint']}"
        used[node] += 1
        r["node_name"] = node if used[node] == 1 else f"{node} {used[node]}"
        bodies = p.meta.get("bodies")
        if bodies:
            r["bodies"] = _body_names(k, [b[0] for b in bodies])
    root = humanise(A.name) if A.name else "Assembly"
    return dict(parts=rows, joints=jnames, joint_rule=jrule, root=root, modules=modules)


def label_problems(name, keys=()):
    """Why `name` is not a readable label ([] when it is)."""
    out = []
    if not name or not name.strip():
        return ["empty"]
    if name.startswith(INTERNAL_PREFIXES):
        out.append("internal prefix")
    if "_" in name:
        out.append("underscore")
    if "—" in name or "–" in name:
        out.append("dash")
    for k in keys:
        if k and k in name:
            out.append(f"contains key {k}")
            break
    return out


def clean_occurrence_name(n):
    """Fusion component name -> the STEP label it came from (drops ' (2)' and ':1' suffixes)."""
    n = re.sub(r":\d+$", "", n or "")
    return re.sub(r"\s*\(\d+\)$", "", n)


def match_entry(chain, table, sep=" > "):
    """Material entry for an occurrence, from its chain of component names (top-level module first).
    Tries the longest path suffix keyed in `table` (tree paths joined by `sep`), then any single
    chain element (legacy STEPs whose labels were the internal keys). Runs in Fusion too, so it
    uses only builtins and re."""
    import re as _re
    ch = [_re.sub(r"\s*\(\d+\)$", "", _re.sub(r":\d+$", "", c or "")) for c in chain]
    for i in range(len(ch)):
        key = sep.join(ch[i:])
        if key in table:
            return table[key]
    for c in reversed(ch):
        if c in table:
            return table[c]
    return None
