"""Stock hardware as real solids.

Local frames (both): +Z points from the joint toward the screw head side, the
screw axis is the Z axis.
- screw: head underside at z = 0, head up to z = +k, shank down to z = -L.
- slot nut: z = 0 is the rail face, the nut sits below it inside the slot
  cavity, its long side along local X (the rail axis).
"""
import copy
from functools import lru_cache

from build123d import Align, Cylinder, Location, Plane, Polygon, Solid, extrude

from lib import profile_1515 as P
from lib import profile_2020 as P20

DEVIATIONS = []
SCREW_DIMS = {"M3": dict(d=3.0, head_d=5.5, head_k=3.0, tap_d=2.5, pitch=0.5),   # tap_d: tap drill (nut bore)
              "M4": dict(d=4.0, head_d=7.0, head_k=4.0, tap_d=3.3, pitch=0.7),
              "M5": dict(d=5.0, head_d=8.5, head_k=5.0, tap_d=4.2, pitch=0.8),
              "M2.5": dict(d=2.5, head_d=4.5, head_k=2.5, tap_d=2.05, pitch=0.45)}   # froggy v9: Pi / LED driver standoffs

# Slot nut cross-section (half width at depth below the rail face). The tongue
# rides in the 3.20 slot opening, the body sits under the 1.40 lip in the 5.80
# cavity, the bottom follows the tapered cavity floor with 0.12 clearance.
NUT_LEN = 6.0
NUT_TONGUE_TOP = 0.30
NUT_BOTTOM = 4.20
NUT_SIDE_CLEAR = 0.12
# 2020 M5 drop-in T-nut (froggy v8): 10 long, tongue 5.8 wide in the 6.0 slot opening from 0.3 below the
# rail face to the 1.8 lip, body 10.76 wide under the lip following the tapered cavity to 4.5 deep
# (4.2 tall overall), M5 tapped bore. Datasheet class: generic zinc-plated "2020 M5 drop-in T-nut"
# (Misumi HNTT6 / OpenBuilds drop-in tee nut envelope, 10 x 10.8 x 4.2).
TNUT20_LEN = 10.0
TNUT20_BOTTOM = 4.50
SLOT_NUT_PROFILE = {"M3": P, "M4": P, "M5": P20}     # which rail family a thread's slot nut fits


def parse(size):
    """'M3x8' -> ('M3', 8.0)."""
    s, l = size.upper().split("X")
    return s, float(l)


@lru_cache(maxsize=None)
def _screw_template(size, head="shcs"):
    thread, length = parse(size)
    pitch = SCREW_DIMS[thread]["pitch"]
    if head == "button":
        # ISO 7380-1 button head: head underside at z = 0, dome up to z = +k, shank down to z = -L
        from bd_warehouse.fastener import ButtonHeadScrew
        s = ButtonHeadScrew(f"{thread}-{pitch}", length=length, fastener_type="iso7380_1", simple=True)
        return Solid(s.wrapped), "bd_warehouse ButtonHeadScrew iso7380_1 simple"
    if head == "thumb":
        # DIN 653 knurled thumb screw, low type (froggy v9 S3 lift-off joints): knurled head dk x k, underside at
        # z = 0, 24 shallow knurl flutes, plain shank (tap-drill diameter, as the fallback SHCS) down to z = -L
        import math as _m
        from build123d import Box
        dk, k = {"M5": (20.0, 4.0)}[thread]
        s = Cylinder(dk / 2, k, align=(Align.CENTER, Align.CENTER, Align.MIN))
        for i in range(24):
            a = 360.0 * i / 24
            s = s - Box(1.0, 1.0, k + 1, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
                Location((dk / 2 * _m.cos(_m.radians(a)), dk / 2 * _m.sin(_m.radians(a)), -0.5), (0, 0, a + 45)))
        s = s + Cylinder(SCREW_DIMS[thread]["tap_d"] / 2 - 0.02, length, align=(Align.CENTER, Align.CENTER, Align.MAX))
        return Solid(s.solids()[0].wrapped), "DIN 653 knurled thumb screw, low type (modelled from the standard table)"
    if head == "csk":
        # ISO 10642 countersunk (flat) head: local z = 0 is the head top (flush
        # face), the 90 deg head cone and the shank run down to z = -L
        from bd_warehouse.fastener import CounterSunkScrew
        s = CounterSunkScrew(f"{thread}-{pitch}", length=length, fastener_type="iso10642", simple=True)
        return Solid(s.wrapped), "bd_warehouse CounterSunkScrew iso10642 simple"
    try:
        from bd_warehouse.fastener import SocketHeadCapScrew
        s = SocketHeadCapScrew(f"{thread}-{pitch}", length=length, fastener_type="iso4762", simple=True)
        return Solid(s.wrapped), "bd_warehouse SocketHeadCapScrew iso4762 simple"
    except Exception as e:  # noqa: BLE001
        DEVIATIONS.append(f"bd_warehouse screw failed ({e!r}); plain modelled SHCS used")
        d = SCREW_DIMS[thread]
        head = Cylinder(d["head_d"] / 2, d["head_k"], align=(Align.CENTER, Align.CENTER, Align.MIN))
        shank = Cylinder(d["tap_d"] / 2 - 0.02, length, align=(Align.CENTER, Align.CENTER, Align.MAX))
        return Solid((head + shank).wrapped), "modelled SHCS"


def screw(size="M3x8", label=None, head="shcs"):
    """Own copy of an ISO 4762 socket head cap screw in the local frame above
    (head="csk": ISO 10642 countersunk screw, z = 0 at the head top)."""
    tpl, src = _screw_template(size, head)
    s = copy.deepcopy(tpl)
    s.label = label or f"hw_{head}_{size.lower()}"
    return s


def screw_source(size="M3x8"):
    return _screw_template(size)[1]


def _nut_half(depth, prof=P):
    if depth <= prof.LIP:
        return prof.SLOT_OPEN / 2 - 0.10
    return min(prof.CAVITY / 2 - NUT_SIDE_CLEAR - 0.0, prof.cavity_half_width(depth) - NUT_SIDE_CLEAR)


@lru_cache(maxsize=None)
def _nut_template(thread="M3", tongue_top=NUT_TONGUE_TOP):
    d = SCREW_DIMS[thread]["tap_d"]
    prof = SLOT_NUT_PROFILE[thread]
    bottom, length = (TNUT20_BOTTOM, TNUT20_LEN) if prof is P20 else (NUT_BOTTOM, NUT_LEN)
    t, body = _nut_half(0.5, prof), _nut_half(prof.LIP + 0.01, prof)
    bot = _nut_half(bottom, prof)
    pts = [(-t, -tongue_top), (t, -tongue_top), (t, -prof.LIP), (body, -prof.LIP),
           (body, -prof.TAPER_DEPTH), (bot, -bottom), (-bot, -bottom),
           (-body, -prof.TAPER_DEPTH), (-body, -prof.LIP), (-t, -prof.LIP)]
    # cross-section in the local YZ plane, extruded along local X
    pl = Plane(origin=(0, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    blank = extrude(pl * Polygon(*pts, align=None), amount=length / 2, both=True)
    bore = Cylinder(d / 2, 20)
    nut = blank - bore
    return Solid(nut.solids()[0].wrapped)


def slot_nut(thread="M3", label=None, tongue_top=NUT_TONGUE_TOP):
    """Slot nut for `thread`, own copy, local frame above: 15-series M3 slot nut
    (drop-in, with tongue), or for M5 the 2020 drop-in T-nut (TNUT20_*).
    tongue_top: depth of the tongue top below the rail face (1.0 under brackets
    whose own 1 mm alignment tongue already fills the slot opening)."""
    n = copy.deepcopy(_nut_template(thread, tongue_top))
    n.label = label or f"hw_slot_nut_{thread.lower()}"
    return n


# ---------------------------------------------------------------- drum + base stage
# Local frames follow the convention above: z = 0 is the joint face the part
# seats against, the part extends to -z (into the host) unless noted.

HEX_NUT = {"M3": dict(s=5.5, m=2.4, bore=2.5), "M4": dict(s=7.0, m=3.2, bore=3.3),
           "M5": dict(s=8.0, m=4.7, bore=4.2)}            # ISO 4032, bore = tap drill
# Heat-set insert (brass, knurled). Pocket 4.1 mm; the insert is modelled as
# installed: a plain sleeve that conforms to the pocket wall (OD 4.0, leaving a
# 0.05 mm radial gap so the boolean checks see two bodies, not one).
INSERT = {"M3": dict(pocket_d=4.1, od=4.0, bore=2.5, length=4.0, pocket_depth=6.0)}


@lru_cache(maxsize=None)
def _hex_nut_template(thread="M3"):
    from build123d import RegularPolygon
    d = HEX_NUT[thread]
    blank = extrude(RegularPolygon(d["s"] / 2, 6, major_radius=False), amount=-d["m"])
    return Solid((blank - Cylinder(d["bore"] / 2, 20)).solids()[0].wrapped)


def hex_nut(thread="M3", label=None):
    """ISO 4032 hex nut, top face (the face that bears on the host) at z = 0,
    body to z = -m, flats parallel to local X."""
    n = copy.deepcopy(_hex_nut_template(thread))
    n.label = label or f"hw_hex_nut_{thread.lower()}"
    return n


@lru_cache(maxsize=None)
def _insert_template(thread="M3"):
    d = INSERT[thread]
    sleeve = Cylinder(d["od"] / 2, d["length"], align=(Align.CENTER, Align.CENTER, Align.MAX)) - \
        Cylinder(d["bore"] / 2, 3 * d["length"])
    return Solid(sleeve.solids()[0].wrapped)


# ISO 7089 plain washer (froggy v9: under the motor-plate M3 screw heads), d1 x d2 x h
WASHER = {"M3": dict(d1=3.2, d2=7.0, h=0.5), "M4": dict(d1=4.3, d2=9.0, h=0.8), "M5": dict(d1=5.3, d2=10.0, h=1.0)}


@lru_cache(maxsize=None)
def _washer_template(thread="M3"):
    d = WASHER[thread]
    w = Cylinder(d["d2"] / 2, d["h"], align=(Align.CENTER, Align.CENTER, Align.MIN)) - \
        Cylinder(d["d1"] / 2, 3 * d["h"] + 2)
    return Solid(w.solids()[0].wrapped)


def washer(thread="M3", label=None):
    """ISO 7089 plain washer: underside (the face on the clamped part) at z = 0, top at z = +h."""
    w = copy.deepcopy(_washer_template(thread))
    w.label = label or f"hw_washer_{thread.lower()}"
    return w


def heat_set_insert(thread="M3", label=None):
    """Installed M3 heat-set insert: top flush with the host face at z = 0,
    body down to z = -length."""
    s = copy.deepcopy(_insert_template(thread))
    s.label = label or f"hw_insert_{thread.lower()}"
    return s


def bearing_6814_2rs(label=None):
    """Catalogue 6814-2RS deep groove ball bearing, 70 x 90 x 10, axis Z, faces
    at z = 0 and z = 10. Inner race 70..75, outer race 85..90, the seals and
    rolling elements as one envelope 75..85 recessed 1 mm from each face."""
    from build123d import Axis, Polyline, make_face, revolve
    pts = [(35, 0, 0), (37.5, 0, 0), (37.5, 0, 1), (42.5, 0, 1), (42.5, 0, 0), (45, 0, 0), (45, 0, 10),
           (42.5, 0, 10), (42.5, 0, 9), (37.5, 0, 9), (37.5, 0, 10), (35, 0, 10), (35, 0, 0)]
    prof = make_face(Polyline(*pts))
    s = revolve(prof, Axis.Z)
    s = s if isinstance(s, Solid) else s.solids()[0]
    s.label = label or "v_bearing_6814_2rs"
    return s


def camera_fisheye_imx219_160(label=None):
    """Waveshare IMX219-160 fisheye camera envelope from the old manifest: PCB
    25 x 24 x 1.6 with a D12 x 10 lens barrel; PCB underside at z = 0, lens
    axis = local +Z through the PCB centre. Returned as a compound of two
    solids, [PCB, lens], so each carries its own material (pcb_green,
    glass_lens); the envelope is the same as the old fused solid."""
    from build123d import Box, Compound
    pcb = Box(25, 24, 1.6, align=(Align.CENTER, Align.CENTER, Align.MIN))
    lens = Cylinder(6, 10, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, 1.6)))
    s = Compound([Solid(pcb.solids()[0].wrapped), Solid(lens.solids()[0].wrapped)])
    s.label = label or "v_camera_fisheye_imx219_160"
    return s


# ---------------------------------------------------------------- froggy v6 stage

def bearing_608_2rs(label=None):
    """Catalogue 608-2RS deep groove ball bearing, 8 x 22 x 7, axis Z, faces at
    z = 0 and z = 7. Inner race 8..12.1, outer race 19..22, seals and balls as
    one envelope 12.1..19 recessed 0.3 mm from each face."""
    from build123d import Axis, Polyline, make_face, revolve
    pts = [(4, 0, 0), (6.05, 0, 0), (6.05, 0, 0.3), (9.5, 0, 0.3), (9.5, 0, 0), (11, 0, 0), (11, 0, 7),
           (9.5, 0, 7), (9.5, 0, 6.7), (6.05, 0, 6.7), (6.05, 0, 7), (4, 0, 7), (4, 0, 0)]
    s = revolve(make_face(Polyline(*pts)), Axis.Z)
    s = s if isinstance(s, Solid) else s.solids()[0]
    s.label = label or "v_bearing_608_2rs"
    return s


def ring(od, id_, h, label=None):
    """Plain ring (washer, shaft collar, coil envelope) with faces at z = 0 and z = h."""
    s = Cylinder(od / 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN)) - \
        Cylinder(id_ / 2, 3 * h + 2, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    s = Solid(s.solids()[0].wrapped)
    s.label = label or "ring"
    return s


def shaft_collar_8mm(label=None):
    """8 mm bore set-screw shaft collar, 16 OD x 8 wide, faces z = 0 and z = 8,
    with its M4 set-screw tapped hole on -X at mid width."""
    s = ring(16.0, 8.0, 8.0)
    pl = Plane(origin=(-8.5, 0, 4.0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    s = s - pl * Cylinder(1.65, 5.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    s = Solid(s.solids()[0].wrapped)
    s.label = label or "v_shaft_collar_8mm"
    return s


def ir_ring_smd(od, hole, t=1.6, n=10, led_r=16.0, pkg=(3.5, 3.5, 1.9), phase=18.0, dot_d=1.2, pad_r=None,
                label=None):
    """Flat SMD IR ring illuminator board (representative solid, no domes): an FR4 disc `od` x `t` with a `hole`
    centre hole for the lens, `n` SMD packages pkg = (radial, tangential, height) on the top face on radius `led_r`
    (first at `phase` deg, equal pitch), a small round emitter window `dot_d` x 0.1 on each package top, and two
    2.0 x 1.5 x 0.1 solder pads on the top face at radius `pad_r` (default midway between the LED ring and the rim),
    at -6 / +6 deg about the local +X axis. Local: board underside z 0, LEDs face +Z.
    Compound in this solid order: board, package 1..n, emitter 1..n, pad 1, pad 2."""
    import math
    from build123d import Box, Compound
    board = Cylinder(od / 2, t, align=(Align.CENTER, Align.CENTER, Align.MIN)) - Cylinder(hole / 2, 3 * t + 2)
    kids = [Solid(board.solids()[0].wrapped)]
    pk, dots = [], []
    for i in range(n):
        ang = phase + 360.0 * i / n
        c, s_ = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        rot = Location((0, 0, 0), (0, 0, 1), ang)
        b = Box(*pkg, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(rot).moved(
            Location((led_r * c, led_r * s_, t)))
        pk.append(Solid(b.solids()[0].wrapped))
        d = Cylinder(dot_d / 2, 0.1, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((led_r * c, led_r * s_, t + pkg[2])))
        dots.append(Solid(d.solids()[0].wrapped))
    pr = pad_r if pad_r is not None else (led_r + pkg[0] / 2 + od / 2) / 2
    pads = []
    for ang in (-6.0, 6.0):
        c, s_ = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        p = Box(2.0, 1.5, 0.1, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((0, 0, 0), (0, 0, 1), ang)).moved(Location((pr * c, pr * s_, t)))
        pads.append(Solid(p.solids()[0].wrapped))
    s = Compound(kids + pk + dots + pads)
    s.label = label or "v_ir_ring_smd"
    return s


def ir_led_5mm(label=None):
    """5 mm IR LED (940 nm, T-1 3/4): 5.8 flange 1.0 thick at z 0..1, 5.0 body
    to z 6.1, hemispherical dome to z 8.6; local +Z = emission axis."""
    from build123d import Sphere
    f = Cylinder(2.9, 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    b = Cylinder(2.5, 5.1, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, 1.0)))
    d = Sphere(2.5).moved(Location((0, 0, 6.1)))
    s = f + b + d
    s = Solid(s.solids()[0].wrapped)
    s.label = label or "v_ir_led"
    return s


def magnet_6x3(label=None):
    """N52 disc magnet 6 x 3, faces z = 0 and z = 3."""
    s = Solid(Cylinder(3.0, 3.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).wrapped)
    s.label = label or "v_magnet_6x3"
    return s


def pcb_box(lx, ly, t=1.6, label=None):
    from build123d import Box
    s = Solid(Box(lx, ly, t, align=(Align.CENTER, Align.CENTER, Align.MIN)).wrapped)
    s.label = label or "pcb"
    return s


def hall_a3144_module(label=None):
    """A3144 hall switch on its breakout: PCB 16 x 12 x 1.6 (z 0..1.6) with the
    TO-92 body lying flat, branded (sensing) face up: 4.0 x 3.0 x 1.52 at
    z 1.6..3.12, centred over the board. Compound [PCB, sensor]."""
    from build123d import Box, Compound
    pcb = Box(16, 12, 1.6, align=(Align.CENTER, Align.CENTER, Align.MIN))
    body = Box(4.0, 3.0, 1.52, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, 1.6)))
    c = Compound([Solid(pcb.solids()[0].wrapped), Solid(body.solids()[0].wrapped)])
    c.label = label or "v_hall_a3144"
    return c


# ---------------------------------------------------------------- froggy v8 stage B

KFL08 = dict(L=48.0, W=27.0, J=37.0, hole=4.8, flange_t=4.5, boss_d=22.0, boss_h=7.0, sphere_top=7.8,
             ring_d=13.0, H=11.5, bore=8.0, set_screw_d=3.0, set_screw_z=9.65)


def kfl08(label=None):
    """KFL08 2-bolt flange bearing unit (zinc die-cast housing, KP08 8 mm insert), from the catalogue
    drawing (A 48, J 37, W 27, N 4.8, overall 11.5): oval flange 48 x 27 with rounded ends round the
    two 4.8 holes at 37 centres, 4.5 thick with 0.8 rounded top edges; cylindrical housing boss 22 OD
    to 7.0 with a 0.8 chamfer; the spherical-outer insert ring (sphere R 10.2 centred 4.0 above the
    mounting face) showing as a domed band to 7.8; the extended inner ring collar 13 OD to 11.5 with
    two M3 set-screw bosses 120 deg apart (radial pads round the tapped holes); 8.0 bore. Mounting
    face at z = 0, boss toward +Z, holes on local X. Inner-ring top stays at H = 11.5 (stage B stack)."""
    import math
    from build123d import Axis, Box, Circle, Pos, Sphere, chamfer, fillet, make_hull
    d = KFL08
    r_end = (d["L"] - d["J"]) / 2
    f = make_hull((Circle(d["W"] / 2) + Pos(d["J"] / 2, 0) * Circle(r_end) + Pos(-d["J"] / 2, 0) * Circle(r_end)).edges())
    fl = extrude(f, amount=d["flange_t"])
    top = fl.edges().filter_by_position(Axis.Z, d["flange_t"] - 1e-3, d["flange_t"] + 1e-3)
    fl = fillet(top, 0.8)
    boss = Cylinder(d["boss_d"] / 2, d["boss_h"], align=(Align.CENTER, Align.CENTER, Align.MIN))
    boss = chamfer(boss.edges().sort_by(Axis.Z)[-1], 0.8)
    s = fl + boss
    band = Sphere(10.2).moved(Location((0, 0, 4.0))) & \
        Cylinder(10.0, d["sphere_top"] - d["boss_h"] + 0.5, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((0, 0, d["boss_h"] - 0.5)))
    s = s + band
    s = s + Cylinder(d["ring_d"] / 2, d["H"], align=(Align.CENTER, Align.CENTER, Align.MIN))
    for a in (90.0, 210.0):
        c, sn = math.cos(math.radians(a)), math.sin(math.radians(a))
        pad = Plane(origin=(c * 5.0, sn * 5.0, d["set_screw_z"]), x_dir=(-sn, c, 0), z_dir=(c, sn, 0)) * \
            Cylinder(1.8, 2.1, align=(Align.CENTER, Align.CENTER, Align.MIN))
        s = s + pad
    s = s - Cylinder(d["bore"] / 2, 3 * d["H"])
    for a in (90.0, 210.0):
        c, sn = math.cos(math.radians(a)), math.sin(math.radians(a))
        s = s - Plane(origin=(c * 3.0, sn * 3.0, d["set_screw_z"]), x_dir=(-sn, c, 0), z_dir=(c, sn, 0)) * \
            Cylinder(d["set_screw_d"] / 2 * 0.83, 6.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for sx in (-1, 1):
        s = s - Cylinder(d["hole"] / 2, 3 * d["H"]).moved(Location((sx * d["J"] / 2, 0, 0)))
    s = Solid(s.solids()[0].wrapped)
    s.label = label or "v_kfl08"
    return s


def hex_standoff(af, length, tap_d, label=None, chamfer_mm=0.0):
    """Female-female hex standoff: `af` across flats, faces z = 0 and z = length, tapped through
    (tap drill bore); chamfer_mm breaks the six corner edges at both ends (turned bar stock)."""
    from build123d import Axis, RegularPolygon, chamfer
    s = extrude(RegularPolygon(af / 2, 6, major_radius=False), amount=length)
    if chamfer_mm:
        ends = s.edges().filter_by_position(Axis.Z, -1e-3, 1e-3) + \
            s.edges().filter_by_position(Axis.Z, length - 1e-3, length + 1e-3)
        s = chamfer(ends, chamfer_mm)
    s = s - Cylinder(tap_d / 2, 3 * length)
    s = Solid(s.solids()[0].wrapped)
    s.label = label or "standoff"
    return s


def camera_uc261(label=None, connector=False, glass=False):
    """Arducam UC-261 class camera envelope: PCB 36 x 36 x 1.6 with a 14 dia x 12 lens barrel;
    PCB underside at z = 0, lens axis local +Z through the PCB centre, lens front at z = 13.6.
    Sensor orientation: the sensor's LONG image axis (2592 px, 194 deg) runs along local X, its short axis along
    local Y. With connector=True a 22 x 5.5 x 2.5 ribbon (FFC) connector body sits on the PCB top face along the
    local +Y edge (1 mm inset), so the ribbon leaves the board toward local +Y, parallel to the short image axis.
    Assumption: no vendor drawing of the UC-261 was found; the connector edge and sensor alignment follow the
    usual Pi camera board layout (FFC on the edge parallel to the sensor's long axis).
    With glass=True the barrel's front carries a 9.9 dia x 0.98 front glass flush in a 10 x 1.0 recess (front stays at 13.6),
    appended last so a colour map can tell it from the barrel.
    Compound [PCB, lens] or [PCB, lens, connector] (+ [glass])."""
    from build123d import Box, Compound
    pcb = Box(36, 36, 1.6, align=(Align.CENTER, Align.CENTER, Align.MIN))
    lens = Cylinder(7.0, 12.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, 1.6)))
    gl = None
    if glass:
        lens = lens - Cylinder(5.0, 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, 12.6)))
        # 0.05 radial and 0.02 floor gap: the glass shares no vertex with the barrel, so mesh splits keep two
        # watertight bodies (lib/camsim.py splits the camera mesh into its bodies); 1.0 thick so its thickness to
        # width ratio (0.1) stays above the PCB's (0.044), which camsim picks as the thinnest body
        gl = Cylinder(4.95, 0.98, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, 12.62)))
    kids = [Solid(pcb.solids()[0].wrapped), Solid(lens.solids()[0].wrapped)]
    if connector:
        con = Box(22.0, 5.5, 2.5, align=(Align.CENTER, Align.MAX, Align.MIN)).moved(Location((0, 17.0, 1.6)))
        kids.append(Solid(con.solids()[0].wrapped))
    if gl is not None:
        kids.append(Solid(gl.solids()[0].wrapped))
    s = Compound(kids)
    s.label = label or "v_camera_uc261"
    return s
