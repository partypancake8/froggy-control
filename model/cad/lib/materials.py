"""Physical materials for every froggy-cad component.

MATERIALS[id]: display colour (RGBA 0..1; acrylic and glass translucent) used
by the PNG renders, GLB/3MF and STEP colours; density (g/cm3); the Fusion
Appearance Library search string (`appearance`) and Fusion Material Library
search string (`fusion_material`) that cad-fusion resolves (exact name first,
then first name containing the string). `appearance_name` renames the copy in
the design where the library name would hide the base metal (black oxide).
Every clear part (acrylic_clear, mesh, ir_led, glass_lens) uses the Fusion library
appearance "Acrylic (Clear)" unchanged (Sawyer's rule: no custom translucent looks).

assign(name, kind) -> (material id, RGBA). Printed PETG parts all take the one
filament colour in PETG_COLOURS (black, DL20), everything else uses
the material's colour."""

MATERIALS = {
    "pom_black": dict(colour=(.08,.08,.09,1),density=1.41,appearance="Plastic - Matte (Black)",fusion_material="Acetal"),
    "aluminium_6061": dict(colour=(0.78, 0.80, 0.83, 1.0), density=2.70,
                           appearance="Aluminum - Satin", fusion_material="Aluminum 6061"),
    "petg": dict(colour=(0.12, 0.12, 0.13, 1.0), density=1.27,
                 appearance="Plastic - Matte (Gray)", fusion_material="PET Plastic"),
    "acrylic_clear": dict(colour=(0.80, 0.92, 0.97, 0.35), density=1.18,
                          appearance="Acrylic (Clear)", fusion_material="Acrylic, Clear"),
    "steel_black_oxide": dict(colour=(0.25, 0.26, 0.28, 1.0), density=7.85, recolour=True,
                              appearance="Coating - Black Oxide", appearance_name="Steel - Black Oxide (Readable)",
                              fusion_material="Steel, Alloy"),
    "steel_zinc": dict(colour=(0.76, 0.78, 0.80, 1.0), density=7.85,
                       appearance="Steel - Galvanized", fusion_material="Steel, Galvanized"),
    "steel_bearing": dict(colour=(0.70, 0.72, 0.75, 1.0), density=7.81,
                          appearance="Stainless Steel - Polished", fusion_material="Steel AISI 5210"),
    "brass": dict(colour=(0.80, 0.64, 0.30, 1.0), density=8.50,
                  appearance="Brass - Polished", fusion_material="Brass"),
    "motor_steel_black": dict(colour=(0.22, 0.23, 0.26, 1.0), density=7.85, recolour=True,
                              appearance="Paint - Metallic (Black)", appearance_name="Paint - Metallic (Gunmetal)",
                              fusion_material="Steel"),
    "aluminium_hub": dict(colour=(0.84, 0.85, 0.87, 1.0), density=2.70,
                          appearance="Aluminum - Polished", fusion_material="Aluminum 6061"),
    "pcb_green": dict(colour=(0.05, 0.38, 0.17, 1.0), density=1.85,
                      appearance="Paint - Enamel Glossy (Green)", fusion_material="FR4"),
    "glass_lens": dict(colour=(0.12, 0.15, 0.20, 0.6), density=2.50, housing="lens_housing",
                       appearance="Acrylic (Clear)", fusion_material="Glass"),
    # dark barrel of a lens body inside a multi-body vendor part (cad-fusion only: the body gets this,
    # its dome faces keep "Acrylic (Clear)")
    "lens_housing": dict(colour=(0.08, 0.08, 0.09, 1.0), density=1.20, recolour=True,
                         appearance="Plastic - Matte (Gray)", appearance_name="Plastic - Matte (Lens Black)",
                         fusion_material="ABS Plastic"),
    # froggy v6 additions
    "steel_shaft": dict(colour=(0.66, 0.68, 0.71, 1.0), density=7.85,
                        appearance="Steel - Satin", fusion_material="Steel"),
    "rubber": dict(colour=(0.10, 0.10, 0.10, 1.0), density=1.10,
                   appearance="Rubber", fusion_material="Rubber"),
    "copper_coil": dict(colour=(0.80, 0.45, 0.22, 1.0), density=8.96,
                        appearance="Copper", fusion_material="Copper"),
    "neodymium": dict(colour=(0.72, 0.73, 0.76, 1.0), density=7.50,
                      appearance="Chrome", fusion_material="Iron"),
    # camera stack colours (I015, 2026-10-13): dark green soldermask, matte black ring board, warm white SMD
    # packages with a dark red emitter window, dark glossy blue-grey lens front glass, tinned pads / metal cans
    "pcb_soldermask": dict(colour=(0.05, 0.30, 0.15, 1.0), density=1.85, recolour=True,
                           appearance="Paint - Enamel Glossy (Green)",
                           appearance_name="Paint - Enamel Glossy (Soldermask Green)", fusion_material="FR4"),
    "pcb_black": dict(colour=(0.08, 0.08, 0.09, 1.0), density=1.85, recolour=True,
                      appearance="Plastic - Matte (Gray)", appearance_name="Plastic - Matte (Soldermask Black)",
                      fusion_material="FR4"),
    "smd_led_white": dict(colour=(0.93, 0.90, 0.80, 1.0), density=1.20, recolour=True,
                          appearance="Plastic - Matte (White)", appearance_name="Plastic - Matte (LED Package White)",
                          fusion_material="ABS Plastic"),
    "ir_emitter_red": dict(colour=(0.38, 0.05, 0.05, 1.0), density=1.20, recolour=True,
                           appearance="Plastic - Matte (Gray)", appearance_name="Plastic - Matte (Emitter Dark Red)",
                           fusion_material="ABS Plastic"),
    "lens_glass_dark": dict(colour=(0.16, 0.20, 0.27, 1.0), density=2.50, recolour=True,
                            appearance="Paint - Enamel Glossy (Green)",
                            appearance_name="Paint - Enamel Glossy (Lens Blue Grey)", fusion_material="Glass"),
    "solder_tin": dict(colour=(0.78, 0.79, 0.81, 1.0), density=7.30,
                       appearance="Steel - Satin", fusion_material="Steel"),
    "ir_led": dict(colour=(0.35, 0.25, 0.45, 0.8), density=1.20,
                   appearance="Acrylic (Clear)", fusion_material="Acrylic, Clear"),
    "black_plastic": dict(colour=(0.35, 0.36, 0.40, 1.0), density=1.20, recolour=True,
                          appearance="Plastic - Matte (Gray)", appearance_name="Plastic - Matte (Mid Grey)",
                          fusion_material="ABS Plastic"),
    # froggy v7 additions
    "mesh": dict(colour=(0.75, 0.80, 0.90, 0.25), density=1.14,
                 appearance="Acrylic (Clear)", fusion_material="Nylon 6/6"),
    # plinth panels and deck sheet: opaque navy cast acrylic, 3 mm, laser cut (I016, 2026-10-13; was ABS)
    "acrylic_navy": dict(colour=(0.20, 0.25, 0.36, 1.0), density=1.18, recolour=True,
                         appearance="Plastic - Matte (Gray)", appearance_name="Plastic - Matte (Panel Navy Acrylic)",
                         fusion_material="Acrylic, Clear"),
    "steel_enclosure": dict(colour=(0.68, 0.70, 0.72, 1.0), density=7.85,
                            appearance="Steel - Satin", fusion_material="Steel"),
    "nylon": dict(colour=(0.93, 0.92, 0.88, 1.0), density=1.14,
                  appearance="Plastic - Matte (White)", fusion_material="Nylon 6/6"),
    # froggy v8 stage C: the rotor power bank, density set so the modelled Anker 321 outline weighs 118 g
    "power_bank": dict(colour=(0.14, 0.15, 0.17, 1.0), density=1.214, recolour=True,
                       appearance="Plastic - Matte (Gray)", appearance_name="Plastic - Matte (Power Bank Black)",
                       fusion_material="ABS Plastic"),
    # froggy v9 S2.10: BACO LBX10510 STEP bodies (sub_materials): red mushroom head, yellow station cover
    "estop_red": dict(colour=(0.80, 0.08, 0.08, 1.0), density=1.20, recolour=True,
                      appearance="Plastic - Matte (Gray)", appearance_name="Plastic - Matte (E-stop Red)",
                      fusion_material="ABS Plastic"),
    "estop_yellow": dict(colour=(0.95, 0.78, 0.10, 1.0), density=1.20, recolour=True,
                         appearance="Plastic - Matte (Gray)", appearance_name="Plastic - Matte (E-stop Yellow)",
                         fusion_material="ABS Plastic"),
}

# per-component display colours that override the material colour (RGBA)
NAME_COLOURS = {"v_frog_wall_eyes": (0.85, 0.10, 0.10, 1.0), "v_frog_disc_eyes": (0.85, 0.10, 0.10, 1.0),
                "v_frog": (0.35, 0.65, 0.25, 1.0),       # red-eyed tree frog: green body, red eyes
                "v_estop_baco_lbx10510_head": (0.80, 0.08, 0.08, 1.0),      # froggy v9 S2.10: red mushroom head
                "v_estop_baco_lbx10510_contact": (0.20, 0.20, 0.22, 1.0),   # contact block
                "v_estop_baco": (0.95, 0.78, 0.10, 1.0),                    # yellow BACO station box
                "v_estop": (0.80, 0.08, 0.08, 1.0)}

# Fusion appearance label per NAME_COLOURS prefix: cad-fusion copies the material's library appearance
# as "<Base> (<label>)" recoloured to the component colour
COLOUR_LABELS = {"v_frog_wall_eyes": "Frog Red", "v_frog_disc_eyes": "Frog Red", "v_frog": "Frog Green",
                 "v_estop_baco_lbx10510_head": "E-stop Red", "v_estop_baco_lbx10510_contact": "Contact Black",
                 "v_estop_baco": "E-stop Yellow", "v_estop": "Red"}

# per-face colour zones of single-solid vendor parts (cad-fusion only): {prefix: {style, zone: (RGBA, label)}}.
# "mushroom": the e-stop STEP is one solid (box + bezel collar + mushroom button) plus a lid solid; the
# bodies keep the material appearance (dark box), faces outside the box front along the actuator axis
# become the collar, and the narrowest neck plus everything beyond it toward the tip becomes the head.
FACE_ZONES = {"v_estop_22mm": dict(style="mushroom", head=((0.80, 0.08, 0.08, 1.0), "E-stop Red"),
                              collar=((0.95, 0.78, 0.10, 1.0), "E-stop Yellow"))}

# PETG filament colour (DL20, 2026-10-12): black for every printed part except the I019 entries below. Set it here only, never per
# part: PETG_COLOURS maps every prefix ("" matches all) to the one filament, so a new printed part is black by default.
PETG_FILAMENT = ((0.12, 0.12, 0.13, 1.0), "Black")
# Second filament (I019, 2026-10-16, Sawyer): the Camera hub printed parts are light grey so they read against the
# black panels. Specific prefixes come first; petg_colour() takes the first match, so "" stays the fallback.
PETG_CAMERA = ((0.88, 0.89, 0.90, 1.0), "Light grey")
PETG_COLOURS = {"fp9_camera_wedge_bridge": PETG_CAMERA, "": PETG_FILAMENT}

BY_KIND = {"rail": "aluminium_6061", "bracket": "petg", "printed": "petg", "stock": "acrylic_clear",
           "screw": "steel_black_oxide", "set_screw": "steel_black_oxide", "nut": "steel_zinc",
           "insert": "brass", "bearing": "steel_bearing", "motor": "motor_steel_black",
           "shaft": "steel_zinc", "hub": "aluminium_hub", "camera": "pcb_green",
           "coupling": "aluminium_hub", "pulley": "aluminium_hub", "belt": "rubber", "collar": "steel_black_oxide", "washer": "steel_zinc",
           "pcb": "pcb_green", "coil": "copper_coil", "magnet": "neodymium", "led": "ir_led",
           "sensor": "pcb_green", "specimen": "rubber", "grommet": "black_plastic", "panel": "acrylic_navy",
           "enclosure": "steel_enclosure", "switch": "black_plastic", "driver": "black_plastic", "mesh": "mesh",
           "foot": "rubber", "clip": "petg",
           "standoff": "steel_zinc", "power_bank": "power_bank", "weight": "steel_zinc"}


def petg_colour(name):
    for pre, (rgba, label) in sorted(PETG_COLOURS.items(), key=lambda kv: -len(kv[0])):
        if name.startswith(pre):
            return rgba, label
    return PETG_FILAMENT


def assign(name, kind):
    """(material id, RGBA) for component `name` of `kind`."""
    n = name.lower()
    if n.startswith("fp") or any(k in n for k in ("tovi", "emimart", "leon", "chocolate", "idig", "3way", "c47",
                                                     "keiran", "v_foot_placeholder")):
        mat = "petg"
    elif n.startswith("lp_"):              # laser / Fablight cut aluminium plates and rings (v9 rule 1, 2026-10-12:
        #                                    no acrylic structure; the drum rings and cage rings are 6061 too)
        mat = "aluminium_6061"
    elif n.startswith("v_spacer_axle_10x5x1"):
        mat = "steel_zinc"
    elif n.startswith("v_deck_hardpoint_"):
        mat = "aluminium_6061"
    elif n.startswith(("v_spacer", "v_drum_post")):   # froggy v9: RAF aluminium round spacers (S1.1, S1.27), the
        mat = "aluminium_6061"                       # S1.32 aluminium drum posts
    elif "cast_bracket" in n:            # froggy v8: bought cast 2020 corner brackets
        mat = "steel_zinc"
    elif n.startswith("s_panel"):
        mat = "acrylic_navy"
    elif n.startswith("s_mesh"):
        mat = "mesh"
    elif n.startswith("s_shaft"):
        mat = "steel_shaft"
    elif n.startswith("s_drum_tube") or n.startswith("s_top_disc"):
        mat = "acrylic_clear"
    elif n.startswith(("v_camera_uc261", "v_rpi_zero_2w")):   # I015: dark green soldermask boards
        mat = "pcb_soldermask"
    elif n.startswith(("v_ir_ring_light", "v_led_ring_ws2812")):   # I015 / I032: matte black ring boards
        mat = "pcb_black"
    else:
        mat = BY_KIND.get(kind)
    if mat is None:
        raise ValueError(f"no material for component {name} (kind {kind})")
    return mat, colour(name, mat)


def colour(name, mat):
    for pre, rgba in NAME_COLOURS.items():
        if name.startswith(pre):
            return rgba
    return petg_colour(name)[0] if mat == "petg" else MATERIALS[mat]["colour"]


def _hex_rgb(h):
    h = (h or "").lstrip("#")
    return [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)] if len(h) == 6 else None


def colour_label(name, rgb):
    for pre, label in COLOUR_LABELS.items():
        if name.startswith(pre):
            return label
    return "#%02x%02x%02x" % tuple(int(round(v * 255)) for v in rgb[:3])


def _entry(name, mat, sheet_hex=None):
    """cad-fusion row: the component colour comes from the sheet's `colour` field when present.
    `label` is set when that colour differs from the material default (then cad-fusion makes a
    recoloured copy "<Base> (<label>)"); petg always carries its filament label."""
    rgb = _hex_rgb(sheet_hex) or list(colour(name, mat)[:3])
    default = MATERIALS[mat]["colour"][:3]
    zones = next((z for pre, z in FACE_ZONES.items() if name.startswith(pre)), None)
    if zones is not None:
        return dict(material=mat, rgb=list(default), label=None, petg_label=None,
                    zones=dict(style=zones["style"], **{k: dict(rgb=list(v[0][:3]), label=v[1])
                                                        for k, v in zones.items() if k != "style"}))
    if mat == "petg":
        label = petg_colour(name)[1]
    elif max(abs(a - b) for a, b in zip(rgb, default)) > 2 / 255:
        label = colour_label(name, rgb)
    else:
        label = None
    return dict(material=mat, rgb=rgb, label=label, petg_label=label if mat == "petg" else None)


def fusion_table(parts):
    """{component or body label: {material, rgb, label, petg_label}} for cad-fusion.
    `parts` = sheet.json components (with optional sub_materials and per-component `colour` hex).
    Keys: the internal name (legacy STEPs labelled by key) and, when the sheet has one, the readable
    tree path joined by ' > ' (module > sub-assembly [> Fasteners > joint group] > part [> body]),
    which is what cad-fusion matches occurrences against (lib.naming.match_entry)."""
    out = {}
    for c in parts:
        mat = c["material"]
        e = _entry(c["name"], mat, c.get("colour"))
        out[c["name"]] = e
        tp = " > ".join(c["tree_path"]) if c.get("tree_path") else None
        if tp:
            out[tp] = e
        bn = c.get("body_names") or {}
        for sub, smat in (c.get("sub_materials") or {}).items():
            se = _entry(sub, smat)
            out[sub] = se
            if tp and sub in bn:
                out[tp + " > " + bn[sub]] = se
    return out
