"""fp_camera_carrier: low camera cradle with the Pi tray and balancing cassette,
ported 1:1 from the accepted fusion-pipeline fp_camera_carrier (Sawyer likes
this mockup: tilted on-axis fisheye cradle).

Geometry is the accepted part's, built relative to its foot (accepted foot Z
61, lens datum (0, 0, 80.3), optical axis (0.5, 0, 0.866), 30 deg toward +X);
the assembly places the foot on the carrier deck at Z 64.5 (+3.5).
"""
import math

from build123d import Align, Box, Cylinder, Location, Plane, Polygon, extrude

from parts import datums as D
from parts._util import fillet_safe, one

Z_ACCEPTED = 61.0
TILT = 30.0
LENS_DATUM = (0.0, 0.0, 80.3)            # accepted (foot at Z 61)
SEAT = (-5.8, 0.0, 70.254105)            # camera PCB underside centre on the landing (accepted)
OPTICAL_AXIS = (0.5, 0.0, math.cos(math.radians(TILT)))
MOUNT_HOLES = [(sx * 24.0, sy * 24.0) for sx in (1, -1) for sy in (1, -1)]
PI_BOSSES = [(x, y) for x in (-29, 29) for y in (33.5, 56.5)]

SPEC = dict(
    name="fp_camera_carrier",
    function="Tilted on-axis fisheye cradle, Pi Zero standoffs, Qi board and counterweight cassette.",
    mates=[dict(part="fp5_drum_carrier", feature="four M3 through holes on the deck inserts (+-24, +-24)", kind="heat_set_insert x4"),
           dict(part="v_camera_fisheye", feature="30 deg camera landing on the two cheeks", kind="seat"),
           dict(part="v_pi_zero2w (later stage)", feature="58 x 23 board on four M2.5 standoffs", kind="seat")],
    loads=[dict(where=[0, 45, 6], direction=[0, 0, -1], newton=3, note="Pi Zero at 600 rpm"),
           dict(where=[0, 0, 19.3], direction=[1, 0, 0], newton=2, note="camera centrifugal")],
    print_face="-z (foot underside)",
    print_rotate=(0, 0, 0),
    material="PETG",
    # Accepted-part features kept as they are, below 3 mm by design (not structural);
    # boxes are relative to the foot underside (nonstructural_z0 = deck top)
    nonstructural_z0=D.DECK_TOP_Z,
    nonstructural=[
        dict(box=[-17, -53, 0, 17, -40, 2.05], reason="washer balance pocket floor (2 mm), carries M6 washers only"),
        dict(box=[-15, -36, 0, 15, -29, 2.05], reason="receiver PCB cable trough floor (2 mm)"),
        dict(box=[-13, 36, 0, 13, 54, 2.05], reason="Pi underside ventilation / CSI route floor (2 mm)"),
        dict(box=[-1, -17.5, -0.1, 6.5, 17.5, 3.5], reason="cheek toe: the 30 deg wedge runs out to 2.44 mm at x 6, "
             "the landing load goes down the 6 mm cheek behind it"),
    ],
)


def build(z0=None):
    """Foot underside at z0 (default: on the carrier deck, Z 64.5)."""
    z0 = D.DECK_TOP_Z if z0 is None else z0
    b = Cylinder(40, 4, align=(Align.CENTER, Align.CENTER, Align.MIN)) - Cylinder(20, 10)
    b = b + Box(70, 34, 4, align=(Align.CENTER, Align.MIN, Align.MIN)).moved(Location((0, 28, 0)))
    b = b + Box(50, 29, 4, align=(Align.CENTER, Align.MIN, Align.MIN)).moved(Location((0, -57, 0)))
    # two 30 deg wedge cheeks, 6 thick, at y -17..-11 and 11..17
    prof = Polygon((-20, 0), (6, 0), (6, 2.442), (-20, 17.453), align=None)
    for y in (-17.0, 11.0):
        pl = Plane(origin=(0, y + 3, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
        b = b + extrude(pl * prof, amount=3, both=True)
    b = one(b)
    hi = [e for e in b.edges() if abs(e.center().X + 20) < 1e-6 and abs(e.center().Z - 17.453) < 1e-6]
    b, _ = fillet_safe(b, 2.0, hi)
    b = b + Box(6, 28, 12, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(Location((-20, 0, 0)))
    for x, y in PI_BOSSES:
        b = b + Cylinder(4.5, 4, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y, 4)))
    b = b - Box(4, 7, 4.2, align=(Align.MIN, Align.MIN, Align.MIN)).moved(Location((-2, -25, -0.1)))
    for x, y in MOUNT_HOLES:
        b = b - Cylinder(1.7, 10).moved(Location((x, y, 0)))
    for x, y in PI_BOSSES:
        b = b - Cylinder(1.1, 4.01, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y, 4)))
    b = b - Box(34, 13, 3, align=(Align.MIN, Align.MIN, Align.MIN)).moved(Location((-17, -53, 2)))
    b = b - Box(30, 7, 3, align=(Align.MIN, Align.MIN, Align.MIN)).moved(Location((-15, -36, 2)))
    b = b - Box(26, 18, 3, align=(Align.MIN, Align.MIN, Align.MIN)).moved(Location((-13, 36, 2)))
    b = one(b)
    b = b.moved(Location((0, 0, z0)))
    b.label = "fp_camera_carrier"
    return b


def camera_location(z0=None):
    """build123d Location of v_camera_fisheye: PCB underside on the landing,
    local Z = optical axis, local X down the slope toward +X."""
    from build123d import Location as L
    z0 = D.DECK_TOP_Z if z0 is None else z0
    n = OPTICAL_AXIS
    lift = 0.01          # PCB resting on the landing; 0.01 mm so the boolean check sees contact, not overlap
    o = (SEAT[0] + n[0] * lift, SEAT[1], SEAT[2] - Z_ACCEPTED + z0 + n[2] * lift)
    xd = (n[2], 0.0, -n[0])
    return L(Plane(origin=o, x_dir=xd, z_dir=n))


def lens_datum(z0=None):
    z0 = D.DECK_TOP_Z if z0 is None else z0
    return (LENS_DATUM[0], LENS_DATUM[1], LENS_DATUM[2] - Z_ACCEPTED + z0)
