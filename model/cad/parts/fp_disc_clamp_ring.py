"""fp_disc_clamp_ring: flat-printing top collar (ported from the accepted
fusion-pipeline fp_disc_clamp_ring): a 4 mm collar around the tube top
(ID 152.7) and the disc edge-light bezel (ID 160.3), three M3 insert pockets
at r 86.5 for washer-head clamp screws.

v6 change: the twelve IR emitters now edge-light the 3.2 acrylic disc from
its rim: twelve radial 5.2 holes through the ring's edge, axis at the disc's
mid-thickness, each with a 45 deg teardrop roof so it prints without
supports. The bezel wall rises to 12.4 (the old vertical wells are gone) so
the teardrop has a 3 mm roof. build(drum_top_z) places it on any drum."""
import math

from build123d import Align, Box, Cylinder, Location, Plane, Polygon, extrude

from lib import hardware as HW
from parts import datums as D
from parts._util import one

OD, COLLAR_ID, BEZEL_ID, COLLAR_H = 186.0, 152.7, 160.3, 4.0
H = 12.4
INSERT_R, LED_D = 86.5, 5.2
LED_Z = COLLAR_H + D.ACRYLIC_T / 2          # 5.6 above the ring underside: disc mid-thickness
INSERTS = [(INSERT_R * math.cos(i * 2 * math.pi / 3), INSERT_R * math.sin(i * 2 * math.pi / 3)) for i in range(3)]
LED_ANGLES = [15 + 30 * i for i in range(12)]
# v7: a 12.2 wide notch at -Y from r 83.9 to the OD takes the fp7_wire_spine key tongue
SPINE_NOTCH = (12.2, 83.9)

SPEC = dict(
    name="fp_disc_clamp_ring",
    function="Locates the tube top and edge-lights the 3.2 acrylic top disc with twelve radial IR emitters.",
    mates=[dict(part="s_top_disc", feature="160.3 bezel, disc at Z 4..7.2", kind="clamp"),
           dict(part="s_drum_tube", feature="152.7 collar over the tube top", kind="seat"),
           dict(part="v_ir_led x12", feature="radial 5.2 teardrop holes at Z 5.6, flange outside the OD", kind="press")],
    loads=[dict(where=[80, 0, 0], direction=[0, 0, -1], newton=10, note="disc clamp screws")],
    print_face="-z (collar underside)",
    print_rotate=(0, 0, 0),
    material="PETG",
    inserts=dict(thread="M3", pocket_d=HW.INSERT["M3"]["pocket_d"], positions=INSERTS),
    nonstructural=[],
)


def _teardrop(pl, r, length):
    """Solid teardrop hole along pl.z_dir from pl.origin: circle r plus a 45 deg
    roof toward pl.y_dir (up)."""
    k = r / math.sqrt(2)
    cyl = pl * Cylinder(r, length, align=(Align.CENTER, Align.CENTER, Align.MIN))
    roof = extrude(pl * Polygon((-k, k), (0, r * math.sqrt(2)), (k, k), (0, 0), align=None), amount=length, dir=pl.z_dir)
    return cyl + roof


def build(drum_top_z=None):
    z0 = (D.DRUM_TOP_Z if drum_top_z is None else drum_top_z) - COLLAR_H
    b = Cylinder(OD / 2, H, align=(Align.CENTER, Align.CENTER, Align.MIN)) - Cylinder(COLLAR_ID / 2, 40)
    b = b - Cylinder(BEZEL_ID / 2, 20, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, COLLAR_H)))
    for x, y in INSERTS:
        b = b - Cylinder(HW.INSERT["M3"]["pocket_d"] / 2, 40).moved(Location((x, y, 0)))
    for a in LED_ANGLES:
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        r0 = BEZEL_ID / 2 - 0.15
        pl = Plane(origin=(c * r0, s * r0, LED_Z), x_dir=(-s, c, 0), z_dir=(c, s, 0))
        b = b - _teardrop(pl, LED_D / 2, OD / 2 - r0 + 3)
    nw, nr = SPINE_NOTCH
    b = b - Box(nw, OD / 2 - nr + 5, 3 * H, align=(Align.CENTER, Align.MAX, Align.CENTER)).moved(Location((0, -nr, 0)))
    b = one(b).moved(Location((0, 0, z0)))
    b.label = "fp_disc_clamp_ring"
    return b


def led_locations(drum_top_z):
    """build123d Locations of the 12 edge LEDs: local +Z (emission) pointing at
    the axis, flange 0.05 outside the ring OD."""
    from build123d import Location as L
    z = drum_top_z - COLLAR_H + LED_Z
    out = []
    for a in LED_ANGLES:
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        r = OD / 2 + 1.0 + 0.05
        out.append(L(Plane(origin=(c * r, s * r, z), x_dir=(0, 0, 1), z_dir=(-c, -s, 0))))
    return out
