"""fp9_motor_plate: printed PETG motor plate, one uniform 12 mm slab with the cradle's four tabs (staged, not in froggy_v12).

Sawyer (2026-11-01): "i was thikning that the new piece you made basicallyt is added onto the og prate, combine them to one
solid no cutouts does that make sense?", then "no supports on the motor plate, just make it thicker". Earlier (I031):
"also thinking of 3d printing some parts namely, the motor bracket, and the plate that attaches to the hub ... and 3d
print plate A". So the 6061 motor plate (froggy_v12._motor_plate) and the PETG cradle (parts/fp9_motor_cradle.py, kept
for the record) become ONE flat printed slab: no lightening cutouts, no step, no ramp, no supports. Every FUNCTIONAL
opening of the aluminium plate stays, because the drive stack is frozen (shaft, 1614 block, spacers, REX pulley, belt
plane Z -13.0, motor pulley Z -17.5..-1.5, motor slots, 30 mm centre distance). Then (same day): "go ahead and do
the 2.5 nominal change on only the motorplate rail tab art": the motor alone drops 2.5 (face Z -23.0 -> -25.5,
MOTOR_DROP) so the unmoved motor pulley's top (Z -1.5) is flush with the nominal shaft tip (-25.5 + 24 = -1.5).

Justification (DESIGN-RULES 1, printed; DL6): a locating plate that hangs the motor from plate A on four spacers, plus
(DL29) a second load path to the 2020 core through the four tabs bolted into the cross rails' bottom slots. Reference
(refboard): the printed one-piece motor mounts of open 3D printer frames (Voron 2.4 A/B motor mounts, Prusa MK4 Y-motor
holder): one thick flat print, the motor in a pocket from one side, frame screws through slotted tabs into T-nuts.

World frame of froggy_v12 (Z 0 = deck rail top, X 0 = rotor axis, +Y away from the front panel, mm). Every datum is read
from assemblies/froggy_v12 (motor face, spacer holes, motor, pilot, belt slot, cross rails) or the cradle module.

Geometry (one solid, 114 x 120 x 12, Z -32..-20):
  slab     BODY_T 12, Z -32..-20. Plan = the aluminium plate's 64 x 120 footprint plus the four tabs out to X +-57 at
           Y +38..+60 / -60..-38 (the cradle's tab footprint, run on to the plate ends because a 2 mm step at Y 58..60
           cannot carry R 4 plan rounds; slot centres unchanged). The whole top face is one plane at Z -20.0 = the rail
           underside, so the tabs bear flush on the rails; the plate top moves from Z -17 to Z -20 and four 3 mm stock
           aluminium spacers (5 ID / 10 OD / 3, the ICD-V12 deck hardpoint part) under the four 20 mm spacers take up
           the 3 mm (assemblies/motor_cradle_scene.py)
  pocket   the motor comes in from the BOTTOM: Z -32..-25.5 (6.5 deep), the motor face seats at MOTOR_FACE_Z -25.5 and
           the roof over the motor flange is Z -25.5..-20 = 5.5. Outline = the 17HS19 STEP footprint (42.3 square, X +-21.15,
           corners chamfered 4.0: STEP max |x| + |y - 30| = 38.3) swept over the tension travel C 28.5..30.5 (body
           Y 7.35..51.65), offset 1.35 (ARC): X +-22.5, -Y wall Y 6.0, chamfered -Y corners with R 1.35 arcs, open past
           the +Y end (the motor slides +-Y for belt tension, its lead glands leave at +Y). The spacer hole at (+-21, +1)
           keeps 3.1 of wall to the chamfer
  keyhole  the pocket joins the 24 spindle hole: X +-12 from Y 0 to the pocket wall, so no sliver is left between the
           hole (r 12) and the pocket's -Y wall (Y 6.0); corners at (+-12, 6.0) R 2. The lower spindle stack (12 OD end
           washer, M4 head, r <= 6) clears the hole by 6
  openings (all through the slab, the aluminium plate's own): 24 spindle hole at (0, 0); I035 belt channel 20 wide
           (X +-10, Y 0..30) joining it to the 24 motor pilot slot at (0, 30) (+-Y travel -1.5..+0.5), four cusps R 2;
           four 3.4 motor screw slots on the 31 square (same travel); four 5.5 spacer holes at (+-21, +1), (+-21, -45);
           four 5.5 x 7 tab slots along X at (+-45, +-48). No lightening openings.
  finish   plan corners R 4 and tab roots R 4 (fillet_printed, part > 40, in 2D so every vertical outer edge is round),
           pocket -Y corners R 1.35 (the offset arcs), keyhole corners R 2, pocket mouth vertical edges R 2, bottom edges
           (Z -32: the outer outline, pocket and keyhole rims) R 1.5. Nothing on the bed face (Z -20, the print face;
           it is also the rail bearing face). Left sharp on purpose: the screw hole and slot rims (screw heads, nylocs,
           washers, T-nut faces) and the aluminium plate's pilot / I035 opening edges. PETG black.

Print: Z -20 top face DOWN on the bed (print_rotate 180 about X), pocket opening upward: every face is vertical or
horizontal and faces up, so no supports and no bridges (the roof is printed first, from the bed). Prototype at 100 %
infill in PETG.

Assumed values: motor body 42.3 square with 4.0 corner chamfers (vendor STEP), pocket clearance 1.35 (>= 1.0 asked),
tension travel BD.TRAVEL -1.5..+0.5, PETG 1.27 g/cm3 at 100 % infill, tab top line to line with the rail underside
(see fp9_motor_cradle TAB_AIR), keyhole corner R 2, pocket mouth R 2, bottom-edge round R 1.5.

Hole table (world X, Y; size; joint):
  (+-21, +1), (+-21, -45)   5.5 through: S1.1 spacer screw M5 x 50 SHCS from plate A + DIN985 nyloc under the slab,
                            j_s1_1_spacer_* (grip plate A 6 + spacer 20 + extra 3 spacer + slab 12 = 41)
  (+-15.5, 30 +-15.5)       3.4 slots in the 5.5 roof: S1.2 M3 x 10 SHCS + 0.5 washer from the top face (Z -20) into
                            the motor, 4.0 engaged (x 12 would be 6.0, past the 4.5 deep motor hole)
  (+-45, +-48)              5.5 x 7 slot along X: M5 x 18 BHCS up through 12 into a 2020 T-nut, j_cradle_tab_*
"""
import math

from build123d import Circle, Face, Kind, Location, Polygon, Pos, Rectangle, SlotOverall, extrude, fillet

from assemblies import froggy_v12 as V12
from lib.design_constants import CLEARANCE_D, PRINTED, fillet_printed
from parts import fp9_motor_cradle as CR
from parts._util import one

BD = V12.BD
# ------------------------------------------------------------------ datums read from froggy_v12 / the cradle (never retyped)
X1 = V12.MPLATE_X / 2                               # 32: plate side
Y1 = V12.MPLATE_Y / 2                               # 60: plate ends
Z_TOP = V12.DECK_Z0                                 # -20: rail underside = the slab's top (bed) face
MOTOR_DROP = 2.5                                    # Sawyer 2026-11-01, "the 2.5 nominal change": motor 2.5 lower so the
#                                                     motor pulley's top (Z -1.5, not moved) is flush with the shaft tip
MOTOR_FACE_Z = V12.MOTOR_FACE_Z - MOTOR_DROP        # -25.5 (froggy_v12 keeps -23.0; this branch's scene only)
FACE_Z = MOTOR_FACE_Z                               # -25.5: motor face = pocket roof underside
BODY_T = 12.0
Z_BOT = Z_TOP - BODY_T                              # -32
ROOF_T = Z_TOP - FACE_Z                             # 5.5: roof over the motor flange
TAB_X1 = CR.TAB_X1                                  # 57
TAB_Y0 = CR.TAB_YC - CR.TAB_W / 2                   # 38: tab inner edge (cradle number); the tab runs to the plate end
TAB_SCREWS = CR.TAB_SCREWS                          # (+-45, +-48)
SLOT_W, SLOT_L = CR.SLOT_W, CR.SLOT_L               # 5.5 x 7 along X
SPACER_XY = V12.SPACER_XY
MOTOR_XY = BD.MOTOR_XY                              # (0, 30)
TRAVEL = BD.TRAVEL                                  # (-1.5, +0.5)
HOLE_D = CLEARANCE_D["M5"]                          # 5.5
PILOT_D = V12.MOTOR_PILOT_D                         # 24
SPINDLE_R = 12.0                                    # froggy_v12 _motor_plate: BD.cylinder(12, ...)
BELT_W = V12.MPLATE_BELT_SLOT_W                     # 20 (I035)
CUSP_R = V12.MPLATE_BELT_SLOT_R                     # 2

# ------------------------------------------------------------------ pocket (from the vendor STEP footprint)
MOTOR_HALF = 21.15                                  # 17HS19 STEP body X +-21.15 (42.3 square)
MOTOR_CHAMFER = 4.0                                 # STEP: max |x| + |y - 30| = 38.3 = 2 x 21.15 - 4.0
POCKET_CLEAR = 1.35                                 # motor body to pocket wall (>= 1.0 asked, 1.35 as the cradle strip)
BODY_Y0 = MOTOR_XY[1] + TRAVEL[0] - MOTOR_HALF      # 7.35: motor body -Y face at the -1.5 travel end
POCKET_X = MOTOR_HALF + POCKET_CLEAR                # 22.5
POCKET_Y0 = BODY_Y0 - POCKET_CLEAR                  # 6.0
KEY_X = SPINDLE_R                                   # 12: keyhole half width (tangent to the spindle hole)
R_KEY = PRINTED["fillet_outside_min"]               # 2: keyhole corners (convex in the material)
R_MOUTH = PRINTED["fillet_outside_min"]             # 2: pocket mouth vertical edges at Y 60
R_OUT = fillet_printed(2 * TAB_X1)                  # 4: plan corners (part > 40 mm)
R_ROOT = fillet_printed(2 * TAB_X1)                 # 4: tab roots in plan
R_BOT = 1.5                                         # bottom-edge round (Z -32), "small rounds on the bottom edges"

SPEC = dict(
    name="fp9_motor_plate",
    print_rotate=(180.0, 0.0, 0.0),
    print_face="+Z (top face, world Z -20, on the bed; pocket opening up)",
    material="PETG, 100 % infill for the prototype",
)


def outline():
    """Plan: the 64 x 120 plate plus the four tabs to X +-57 (Y 38..60), convex corners and tab roots R 4 in 2D."""
    p = [(TAB_X1, TAB_Y0), (TAB_X1, Y1), (-TAB_X1, Y1), (-TAB_X1, TAB_Y0), (-X1, TAB_Y0), (-X1, -TAB_Y0),
         (-TAB_X1, -TAB_Y0), (-TAB_X1, -Y1), (TAB_X1, -Y1), (TAB_X1, -TAB_Y0), (X1, -TAB_Y0), (X1, TAB_Y0)]
    f = Polygon(*p, align=None).faces()[0]
    f = fillet([v for v in f.vertices() if abs(abs(v.X) - TAB_X1) < 1e-3], R_OUT)
    f = fillet([v for v in f.vertices() if abs(abs(v.X) - X1) < 1e-3 and abs(abs(v.Y) - TAB_Y0) < 1e-3], R_ROOT)
    return f


def pocket_outline():
    """Motor pocket (Z -32..-25.5), one face: the motor footprint swept over its tension travel and offset 1.35 (open past
    the +Y end), joined to the spindle hole by the X +-12 keyhole, its two corners R 2."""
    h, c, y0 = MOTOR_HALF, MOTOR_CHAMFER, BODY_Y0
    sweep = Polygon((-h, Y1 + 5), (-h, y0 + c), (-h + c, y0), (h - c, y0), (h, y0 + c), (h, Y1 + 5), align=None).faces()[0]
    pocket = Face(sweep.outer_wire().offset_2d(POCKET_CLEAR, kind=Kind.ARC))
    key = Pos(0, (POCKET_Y0 + 1.0) / 2) * Rectangle(2 * KEY_X, POCKET_Y0 + 1.0)
    f = (pocket + key + Circle(SPINDLE_R)).faces()[0]
    f = fillet([v for v in f.vertices() if abs(abs(v.X) - KEY_X) < 1e-3 and abs(v.Y - POCKET_Y0) < 1e-3], R_KEY)
    return f


def _at(z0, z1, face):
    return extrude(face.moved(Location((0, 0, z0))), z1 - z0)


def build():
    p = _at(Z_BOT, Z_TOP, outline())
    p = one((p - _at(Z_BOT - 1, FACE_Z, pocket_outline())).clean())
    # the aluminium plate's openings, through (froggy_v12._motor_plate, same calls)
    p = p - BD.slot(*MOTOR_XY, PILOT_D, Z_BOT - 1, Z_TOP + 1)
    p = p - BD.cylinder(SPINDLE_R, BODY_T + 2, Z_BOT - 1)
    w = BELT_W / 2
    p = p - V12._box(-w, w, 0.0, MOTOR_XY[1], Z_BOT - 1, Z_TOP + 1)
    p = one(p.clean())
    yj = math.sqrt((PILOT_D / 2) ** 2 - w ** 2)
    cusp = [(sx * w, y) for sx in (1, -1) for y in (yj, MOTOR_XY[1] + TRAVEL[0] - yj)]
    edges = [e for e in p.edges() if abs(e.length - ROOF_T) < 1e-3
             and any(math.hypot(e.center().X - x, e.center().Y - y) < 0.05 for x, y in cusp)]
    assert len(edges) == 4, f"belt channel: expected 4 cusp edges, got {len(edges)}"
    p = one(p.fillet(CUSP_R, edges))
    mouth = [e for e in p.edges() if abs(e.length - (FACE_Z - Z_BOT)) < 1e-3
             and abs(abs(e.center().X) - POCKET_X) < 1e-3 and abs(e.center().Y - Y1) < 1e-3]
    assert len(mouth) == 2, f"pocket mouth: expected 2 edges, got {len(mouth)}"
    p = one(p.fillet(R_MOUTH, mouth))
    # bottom edges: the Z -32 outer wire (outline, pocket, keyhole, spindle hole rim), before the screw holes are cut
    bot = [e for e in p.edges() if abs(e.start_point().Z - Z_BOT) < 1e-3 and abs(e.end_point().Z - Z_BOT) < 1e-3
           and abs(e.center().Z - Z_BOT) < 1e-3]
    p = one(p.fillet(R_BOT, bot))
    for x, y in SPACER_XY:
        p = p - extrude(Pos(x, y) * Circle(HOLE_D / 2), 40).moved(Location((0, 0, Z_BOT - 10)))
    for x, y in V12.MOTOR_HOLES:
        p = p - BD.slot(x, y, 3.4, Z_BOT - 1, Z_TOP + 1)
    for x, y in TAB_SCREWS:
        p = p - extrude(Pos(x, y) * SlotOverall(SLOT_L, SLOT_W), 40).moved(Location((0, 0, Z_BOT - 10)))
    p = one(p.clean())
    p.label = "fp9_motor_plate"
    return p


def features():
    """Datums the staging scene places hardware from."""
    return dict(z_bot=Z_BOT, z_top=Z_TOP, face_z=FACE_Z, body_t=BODY_T, roof_t=ROOF_T, spacer_xy=list(SPACER_XY),
                tab_screws=TAB_SCREWS, pocket=dict(x=POCKET_X, y0=POCKET_Y0, open_to="+Y end (Y 60)",
                                                   chamfer=MOTOR_CHAMFER, r_corner=POCKET_CLEAR, depth=FACE_Z - Z_BOT,
                                                   key_x=KEY_X),
                print_face=SPEC["print_face"])
