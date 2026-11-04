"""V11 belt transmission, ICD-V11.md S1.2 / S1.3.

Bought parts: vendor goBILDA pulley STEP, Olimex motor pulley / belt specification.
Motor pulley groove/flange dimensions and the belt tooth fillets are explicit
modelling assumptions in source.json, not manufacturing drawings for those parts.
All 50 belt teeth and both 20T pulleys are represented as solid geometry.
The belt is a fixed routed shape, not a rigid body rotating about the rotor axis.
"""
import copy
import math
from functools import lru_cache
from pathlib import Path

import numpy as np
from build123d import (Align, Axis, Box, Compound, Cylinder, Face, Location,
                      Plane, RegularPolygon, Solid, extrude, import_step, section)
from assemblies.frame import _location

ROOT = Path(__file__).resolve().parents[1]
PITCH, TEETH, WIDTH, LENGTH = 2.0, 20, 6.0, 100.0
CENTER = (LENGTH - PITCH * TEETH) / 2
MOTOR_XY, MOTOR_ANGLE = (0.0, CENTER), 0.0
TRAVEL = (-1.5, 0.5)
R_PITCH = PITCH * TEETH / (2 * math.pi)
REX_TOP = 0.25
BELT_Z = REX_TOP - 13.0
REX_NAME = 'v_gt2_rex_pulley_20'
MOTOR_NAME = 'v_gt2_motor_pulley_20_5'
BELT_NAME = 'v_gt2_belt_100_6'
PART_NAMES = (MOTOR_NAME, REX_NAME, BELT_NAME)
SHAFT_NAME = 'v_gobilda_rex_shaft_2106_72'
UPPER_WASHER = 'hw_retainer_washer_upper_m4'
LOWER_WASHER = 'hw_retainer_washer_lower_m4'
LOWER_SPACER = 'v_retainer_spacer_8x6'
LOWER_SHIM = 'v_retainer_shim_8x1'
RETAINER_NAMES = (UPPER_WASHER, LOWER_WASHER, LOWER_SPACER, LOWER_SHIM)
# Retainer datums (V11 values). A later assembly may replace these module values before it builds (V12 fix A, I033).
RETAINER_WASHERS = ((UPPER_WASHER, 47.), (LOWER_WASHER, -26.))          # washer bottom face Z
RETAINER_STOCK = (                                                       # (name, vendor dir, STEP, bottom Z, note)
    (LOWER_SPACER, 'hardware/gobilda_1522_8x6', '1522-0010-0060.step', -25.,
     'ICD-V11 S1.6c; captured backup stack, 0.25 mm nominal gap to pulley'),
    (LOWER_SHIM, 'hardware/gobilda_2807_8x1', '2807-0811-1000.step', -19.,
     'ICD-V11 S1.6c; captured backup stack, 0.25 mm nominal gap to pulley'))
RETAINER_ENDS = (('upper', UPPER_WASHER, 47., (0, 0, -1), 48.),        # (tag, washer, shaft end Z, axis, head seat Z)
                 ('lower', LOWER_WASHER, -25., (0, 0, 1), -26.))
COLLAR_NAME = 'v_shaft_collar_gobilda_2920'


def rz(angle):
    a = math.radians(angle)
    return np.array([[math.cos(a), -math.sin(a), 0],
                     [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def solid(s):
    ss = s.solids()
    assert len(ss) == 1, f'Expected one connected solid, got {len(ss)}'
    return ss[0]


def box(x0, x1, y0, y1, z0, z1):
    return Box(x1-x0, y1-y0, z1-z0).moved(Location(((x0+x1)/2, (y0+y1)/2, (z0+z1)/2)))


def cylinder(r, h, z=0):
    return Cylinder(r, h, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, z)))


@lru_cache(None)
def rex_local():
    """Own vendor solids at origin; local Z 0..18, hub 0.5..9.5, tooth band 9.5..16.5.
    Original STEP axis is Y, center (-37.004537459, *, 14.108290643).
    REX flats horizontal in XY, matching V9's shaft and bearing bores.
    """
    src = import_step(str(ROOT/'vendor/motion/gobilda_3422_rex_pulley_20/3422-4008-0020 assembly.STEP')).solids()
    b = src[0].bounding_box()
    origin = np.array([(b.min.X+b.max.X)/2, b.min.Y, (b.min.Z+b.max.Z)/2])
    rot = np.array([[1,0,0], [0,0,-1], [0,1,0]], float)
    loc = _location(rot, -rot @ origin)
    return [copy.deepcopy(s).moved(loc) for s in src]


@lru_cache(None)
def tooth_face():
    """Exact 20T GT2 outer profile from vendor STEP; no bore in this face."""
    f = section(rex_local()[0], Plane(origin=(0,0,13))).faces()[0]
    return Face(f.outer_wire()).moved(Location((0,0,-13)))


def rex_pulley():
    # Flip hub upward. X is unchanged so REX flats remain aligned with the shaft.
    loc = _location(np.diag([1.,-1.,-1.]), (0,0,REX_TOP))
    return Compound(children=[copy.deepcopy(s).moved(loc) for s in rex_local()])


def _motor_pulley_hub_down():
    """Olimex 20T, 5 bore, 16 OD x 16 long. Round bore directly on motor's D shaft.
    Hub down, 7 mm hub + 0.5 flange + 8 tooth band + 0.5 flange. Two M3 set screws.
    Grub-screw axis -90 degrees matches the motor's actual D-flat normal.
    """
    z0 = BELT_Z - 11.5
    teeth = extrude(tooth_face(), amount=8).moved(Location((0,0,7.5)))
    body = cylinder(6,7) + cylinder(8,.5,7) + teeth + cylinder(8,.5,15.5)
    body = body - cylinder(2.51,18,-1)
    screws = []
    for ang, inner in ((MOTOR_ANGLE-90,2.01),(MOTOR_ANGLE,2.51)):
        a = math.radians(ang)
        normal = (math.cos(a),math.sin(a),0)
        pl = Plane(origin=(0,0,3.5), z_dir=normal)
        body = body - pl * Cylinder(1.51,12,align=(Align.CENTER,Align.CENTER,Align.MIN))
        # Model each M3 grub with its hex socket; simplified thread envelope.
        sc = cylinder(1.49,3)
        socket = extrude(RegularPolygon(1.5/math.sqrt(3),6), amount=1.25).moved(Location((0,0,1.8)))
        sc = sc - socket
        sc = sc.moved(Location((0,0,inner)))
        screws.append(solid(pl*sc))
    loc = Location((0,CENTER,z0))
    return Compound(children=[solid(body).moved(loc)] + [s.moved(loc) for s in screws])


def motor_pulley():
    """Flip V10 pulley about its D-flat normal, preserving the screw/flat contact.
    Hub upward, groove at the original belt Z. Minor dimensions remain assumed.
    """
    u = np.array([math.cos(math.radians(MOTOR_ANGLE-90)), math.sin(math.radians(MOTOR_ANGLE-90)), 0.])
    R = 2 * np.outer(u,u) - np.eye(3)
    origin = np.array([0., CENTER, BELT_Z])
    return _motor_pulley_hub_down().moved(_location(R, origin-R@origin))


def stadium(r, z0, height):
    return (cylinder(r,height,z0) + cylinder(r,height,z0).moved(Location((0,CENTER,0)))
            + box(-r,r,0,CENTER,z0,z0+height))


def belt():
    """Closed 100 mm pitch loop, 6 wide, 50 teeth. Wrapped teeth follow the
    vendor pulley grooves; straight tooth flanks are rounded visual geometry.
    Nominal geometry only, not an elastomer contact/deformation simulation.
    """
    z0 = BELT_Z - WIDTH/2
    outer, root = R_PITCH + .43, R_PITCH - .20  # .63 backing; tooth fillets approximate
    band = stadium(outer,z0,WIDTH) - stadium(root,z0-.1,WIDTH+.2)
    # Wrapped teeth fill the vendor grooves with a small radial clearance.
    for cy, side in ((0,-1),(CENTER,1)):
        annulus = cylinder(root+.02,WIDTH,z0) - cylinder(4.9,WIDTH+.2,z0-.1)
        # Shape follows the same cross-section as the mating 20T pulley.
        cutter = extrude(tooth_face(),amount=WIDTH+.2).moved(Location((0,0,z0-.1)))
        annulus = annulus - cutter
        half = box(-10,10,-10,0,z0-.1,z0+WIDTH+.1) if side<0 else box(-10,10,0,10,z0-.1,z0+WIDTH+.1)
        band = band + (annulus & half).moved(Location((0,cy,0)))
    # 15 teeth along each 30 mm tangent plus 10 on each semicircle = 50.
    for side in (-1,1):
        for i in range(15):
            tooth = cylinder(.55,WIDTH,z0).moved(Location((side*(root-.20),1+2*i,0)))
            band = band + tooth
    # At the two tangent transitions use the actual mating tooth profile too.
    # This trims only the rounded straight-tooth approximation at engagement.
    return solid(band - motor_pulley() - rex_pulley())


def motor_holes():
    rot = rz(MOTOR_ANGLE)
    return [tuple((rot @ np.array([x,y,0]) + np.array([0,CENTER,0]))[:2])
            for x in (-15.5,15.5) for y in (-15.5,15.5)]


def slot(x,y,diameter,z0,z1):
    a,b = TRAVEL
    r = diameter/2
    return (cylinder(r,z1-z0,z0).moved(Location((x,y+a,0)))
            + cylinder(r,z1-z0,z0).moved(Location((x,y+b,0)))
            + box(x-r,x+r,y+a,y+b,z0,z1))


def add(A):
    from lib import vendor as V
    for name,shape,kind,source in (
        (MOTOR_NAME,motor_pulley(),'pulley','motion/olimex_gt2_pulley_20_5'),
        (REX_NAME,rex_pulley(),'pulley','motion/gobilda_3422_rex_pulley_20'),
        (BELT_NAME,belt(),'belt','motion/olimex_gt2_belt_100')):
        meta = dict(step=1 if name==MOTOR_NAME else 2,
                    note='ICD-V11.md S1.3; source.json records geometry and qualification assumptions')
        if kind=='pulley':
            meta.update(spins=True,spin_origin=(0,CENTER,0) if name==MOTOR_NAME else (0,0,0))
        if name==MOTOR_NAME:
            meta['bodies']=[('motor_pulley_body','aluminium_hub'),('motor_pulley_grub_flat','steel_black_oxide'),('motor_pulley_grub_side','steel_black_oxide')]
        elif name==REX_NAME:
            meta['bodies']=[('rex_pulley_teeth','aluminium_hub'),('rex_pulley_hub','steel_black_oxide'),('rex_pulley_flange','steel_black_oxide'),('rex_pulley_grub_1','steel_black_oxide'),('rex_pulley_grub_2','steel_black_oxide')]
        A.add(name,shape,kind,'drive_base',**meta)
        A.vendor.append(V.vendor_entry(name,source))


def report(A):
    """Analytical dimensions and fit checks, separate from collision checks."""
    shaft = A['v_gobilda_rex_shaft_2106_72'].bounding_box()
    motor_shaft = A['v_nema17_shaft'].bounding_box()
    p = A[MOTOR_NAME].bounding_box()
    return dict(pitch_mm=PITCH,belt_pitch_length_mm=LENGTH,belt_teeth=50,
                belt_width_mm=WIDTH,pulley_teeth=[20,20],ratio=1.0,
                center_distance_mm=CENTER,motor_travel_mm=[CENTER+x for x in TRAVEL],
                pitch_radius_mm=R_PITCH,belt_center_z_mm=BELT_Z,
                wrap_degrees=180,teeth_engaged_per_pulley=10,
                rex_shaft_length_mm=shaft.size.Z,rex_hub_engagement_mm=min(9.5, REX_TOP-shaft.min.Z),
                motor_engagement_mm=min(motor_shaft.max.Z,p.max.Z)-max(motor_shaft.min.Z,p.min.Z),
                collar_clearance_mm=(A[COLLAR_NAME].distance_to(A[REX_NAME]) if COLLAR_NAME in A.parts else None),
                qualification='geometry only; belt working tension, screw torque and bearing/motor radial loads pending')


def _stock_axial(rel, bottom):
    """Position a vendor Y-axis spacer on world Z, retaining original geometry."""
    src = import_step(str(ROOT/'vendor'/rel)).solids()[0]
    bb = src.bounding_box()
    R = np.array([[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]])
    c = np.array([(bb.min.X+bb.max.X)/2, bb.min.Y, (bb.min.Z+bb.max.Z)/2])
    return src.moved(_location(R, np.array([0.,0.,bottom])-R@c))


def add_retainers(A):
    """S1.6b/c-V11: stock end washers, shaft spacer and shim; no bearing preload."""
    from lib import hardware as HW, vendor as V
    for name,z in RETAINER_WASHERS:
        A.add(name, HW.ring(12,4.3,1,name).moved(Location((0,0,z))),
              'washer','rotor',step=2,spins=True,
              note='ISO 7093 M4 washer, ICD-V11 S1.6; shaft end retention, not a bearing preload washer')
        A.vendor.append(V.vendor_entry(name,'hardware/iso7093_m4'))
    for name,rel,file,z,note in RETAINER_STOCK:
        A.add(name,_stock_axial(rel+'/'+file,z),'standoff','rotor',step=2,spins=True,note=note)
        A.vendor.append(V.vendor_entry(name,rel))


def retainer_joints(A):
    from lib.joints import tapped_joint
    for tag,washer,z,axis,seat in RETAINER_ENDS:
        tapped_joint(A,[washer,SHAFT_NAME],(0,0,z),axis,screw='M4x8',head='button',
                     seat=(0,0,seat),name='j_s1_6_retainer_'+tag,module='rotor',
                     evaluate=False,step=2,min_engagement=6.,
                     bench=[washer,SHAFT_NAME])
        A.parts[washer].meta["joint"] = "j_s1_6_retainer_" + tag
        A.joints[-1]["owner"] = SHAFT_NAME
