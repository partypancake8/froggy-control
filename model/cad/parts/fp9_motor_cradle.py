"""fp9_motor_cradle: PETG U cradle under the motor plate that ties the S1.26 drive cartridge to the two cross rails.

Sawyer (2026-11-01): "i was thiknkng of making the motor plate also connect to the underside of the center supports but
his cannot change any shaft diamter or anhting like that, i think we could make it just 3d printed geometry that allows it
to connect on four tabs?" ... "just keep that al lthe same only modify the bracket inplace, will be 3d pritned, the petg
seems like it will be fine given the test parts printed at 100% at least for tihs prototype". The drive stack (shaft,
1614 block, spacers, shims, REX pulley, hub, belt plane, motor pulley, motor plate, motor slots, 30 mm centre distance,
the four 20 mm spacers) is unchanged; the cradle only adds a second load path from the motor plate to the rails.

Justification (DESIGN-RULES 1, printed; DL6): a locating bracket, not a plate in the rotor load path. The motor plate
already hangs from plate A on its four spacers; the cradle is clamped under the plate by the same four M5 spacer screws
(M5 x 45 SHCS, was x 40, + DIN985 nyloc on the cradle underside) and reaches out on four tabs to the cross rails' bottom
slots, so the motor plate is also held from below by the 2020 core (DL29). A flat laser plate cannot make the 3 mm step
from the plate underside to the rail underside; no stock bracket spans 2 x 2020 at X +-45 round a NEMA 17.
Reference (refboard): the printed motor cradles of open 3D printer frames (Voron Z-drive mounts, Prusa MK4 motor
brackets): one flat print, filleted plan, bosses round the clamp screws, tabs with slots into extrusion T-nuts.

World frame of froggy_v12 (Z 0 = deck rail top, X 0 = rotor axis, +Y away from the front panel, mm); every datum is read
from assemblies/froggy_v12 (motor plate, spacer holes, cross rails, motor) so the cradle follows the machine.

Print orientation: flat on the web underside (world Z -29, part print face -Z) on the bed, everything rises from it, no
overhangs, no supports. Prototype at 100 % infill in PETG (Sawyer, 2026-11-01).

Geometry:
  web      6 thick, Z -29..-23, directly under the motor plate (64 x 120 at Z -23..-17): a U open at +Y, where the
           motor leads leave (the motor turns 180 so its glands face +Y)
  strips   right X 15..32 for Y -58..+7 (round the spacer bosses), narrowing to X 22.5..32 for Y +7..+58: the 42.3
           motor body (X +-21.15, Y 8.85..51.15) keeps 1.35 to the strip; its 1 mm tension travel is along Y. Left mirrored
  bar      X -15..15, Y -58..-52, joins the strips at the -Y end; the gap X +-15 between the strips clears the lower
           spindle stack (12 OD end washer and M4 head down to Z -28 at r <= 6) by 9 radially and the 20 wide belt slot
  bosses   12 OD (1.2 d inner wall, DL8 bend accepted) round the four 5.5 spacer holes at (+-21, +1) and (+-21, -45):
           the strip's inner edge (X 15) is 6 from the hole centre, tangent to the boss. The 24 spindle hole and the
           20 wide belt slot of the motor plate lie 2 mm further in, so a wider strip would only cover air the plate
           already gives up; the nyloc (8 AF) seats on the boss underside
  tabs     X 32..57 (rail X 35..55 plus 2 outboard) and Y +38..+58 / -58..-38, 9 thick from the web underside to the
           rail underside, Z -29..-20; each with a 5.5 x 7 slot along X at (+-45, +-48) for an M5 x 14 BHCS up into a
           2020 drop-in T-nut in the rail's bottom slot. The web climbs to the tab top on a 45 deg ramp, X 34..37 (foot
           R 3, crest R 4): the ramp foot stays 2 off the motor plate edge (X 32) and the rail bears on X 38.7..55 round
           the screw. The tab top is line to line with the rail (see TAB_AIR)
  finish   vertical outer edges R 4 (fillet_printed, part > 40), tab and bar roots R 4, the boss-to-strip corner R 1
           (PRINTED fillet_inside, keeps the motor clearance), all rounded in plan so the outline is tangent-continuous;
           every top edge R 2 (PRINTED fillet_outside_min) in one pass, over the ramp too; 0.5 elephant-foot chamfer on
           the print face (cadkit finish). The slot rims stay sharp (button head and T-nut faces). PETG black
           (COLOURS printed, lib/materials petg).

Hole table (world X, Y; size; joint):
  (+-21, +1), (+-21, -45)   5.5 through: spacer screw M5 x 45 SHCS from plate A + DIN985 nyloc under the cradle,
                            j_s1_1_spacer_* (S1.1, grip plate A 6 + spacer 20 + motor plate 6 + cradle 6 = 38)
  (+-45, +-48)              5.5 x 7 slot along X: M5 x 14 BHCS up into a 2020 T-nut, j_cradle_tab_* (+-0.75 in X for
                            the rail position; x 12 reaches only 2.7 into the T-nut through the 9 tab, x 14 gives 3.9)

Install: bench, with the cartridge built (motor plate on its spacers): cradle under the motor plate, the four spacer
screws through it, nylocs on the cradle underside; drop the cartridge through the deck opening, S1.4 screws into the
rail top T-nuts, then the four tab screws from below into the bottom-slot T-nuts (slid in from the rail ends before the
frame is closed).
"""
from build123d import Circle, Location, Plane, Polygon, Pos, Rectangle, SlotOverall, extrude, fillet

from assemblies import froggy_v12 as V12
from lib.design_constants import CLEARANCE_D, PRINTED, fillet_printed
from lib import profile_2020 as P
from parts._util import one

# ------------------------------------------------------------------ datums read from froggy_v12 (never retyped)
PLATE_X1 = V12.MPLATE_X / 2                         # 32: motor plate side edge, the strip's outer edge
PLATE_Y1 = V12.MPLATE_Y / 2                         # 60: motor plate end
WEB_Z1 = V12.MPLATE_Z0                              # -23: motor plate underside
RAIL_Z0 = V12.DECK_Z0                               # -20: cross rail underside
RAIL_X = V12.CROSS_X                                # 45: cross rail axis (bottom slot centre line)
SPACER_XY = V12.SPACER_XY                           # (+-21, +1), (+-21, -45)
MOTOR_XY = V12.BD.MOTOR_XY                          # (0, 30)
HOLE_D = CLEARANCE_D["M5"]                          # 5.5 (= V12.M5_HOLE)

# ------------------------------------------------------------------ cradle numbers
WEB_T = 6.0
WEB_Z0 = WEB_Z1 - WEB_T                             # -29: print face
BOSS_D = 12.0                                       # 1.2 d wall round the 5.5 hole: inner edge 6 from the centre
END_INSET = 2.0
Y1 = PLATE_Y1 - END_INSET                           # 58: cradle ends, 2 inside the plate ends
MOTOR_W = 42.3                                      # 17HS19 drawing body square (STEP: X +-21.15)
MOTOR_SIDE_CLEAR = 1.35                             # strip to motor body (>= 1 asked)
STRIP_X0 = SPACER_XY[0][0] - BOSS_D / 2             # 15: strip inner edge tangent to the bosses
NARROW_X0 = MOTOR_W / 2 + MOTOR_SIDE_CLEAR          # 22.5: strip inner edge beside the motor
BOSS_Y = max(y for _, y in SPACER_XY)               # +1: the +Y bosses, where the strip narrows
BAR_W = WEB_T                                       # 6: -Y bar, Y -58..-52
TAB_W = 20.0                                        # along Y
TAB_YC = Y1 - TAB_W / 2                             # 48: tab and slot centre
TAB_REACH = 2.0                                     # tab past the rail faces in X
TAB_X1 = RAIL_X + P.W / 2 + TAB_REACH               # 57
STEP_GAP = 2.0                                      # ramp foot to the motor plate edge (its R 3 cove starts 0.8 out)
TAB_X0 = PLATE_X1 + STEP_GAP                        # 34: foot of the 45 deg ramp up to the tab top (top at X 37)
TAB_AIR = 0.0                                       # tab top to rail underside. The concept had 0.3 air (the screw pulls
#                                                     the tab up); bin/cad's joint check needs face contact at a joint
#                                                     (contact gap <= 0.05), so the model is line to line: the spacer
#                                                     stack and the rail agree on paper and PETG takes the tolerance
TAB_Z1 = RAIL_Z0 - TAB_AIR                          # -20
SLOT_W, SLOT_L = HOLE_D, 7.0                        # 5.5 x 7 along X
TAB_SCREWS = [(sx * RAIL_X, sy * TAB_YC) for sx in (1, -1) for sy in (1, -1)]

R_OUT = fillet_printed(2 * TAB_X1)                  # 4: vertical outer edges (part > 40 mm)
R_ROOT = fillet_printed(2 * TAB_X1)                 # 4: tab and bar roots (tangent blends)
R_IN = PRINTED["fillet_inside"]                     # 1: boss-to-strip corner beside the motor
R_TOP = PRINTED["fillet_outside_min"]               # 2: top edges
R_RAMP_TOP, R_RAMP_FOOT = 2 * R_TOP, 1.5 * R_TOP    # 4 / 3: the ramp's profile rounds, larger than R_TOP so the top-edge
#                                                     round can roll over the ramp

SPEC = dict(
    name="fp9_motor_cradle",
    print_rotate=(0.0, 0.0, 0.0),
    print_face="-Z (web underside, world Z -29)",
    material="PETG, 100 % infill for the prototype",
)


def _outline():
    """Plan outline (one face): two strips with the +Y bosses, the -Y bar and four tabs, corners rounded in 2D."""
    import math
    f = None
    for sx in (1, -1):
        def r(x0, x1, y0, y1):
            return Pos(sx * (x0 + x1) / 2, (y0 + y1) / 2) * Rectangle(x1 - x0, y1 - y0)
        parts = [r(STRIP_X0, PLATE_X1, -Y1, BOSS_Y), r(NARROW_X0, PLATE_X1, BOSS_Y, Y1),
                 Pos(sx * SPACER_XY[0][0], BOSS_Y) * Circle(BOSS_D / 2),
                 r(PLATE_X1 - 1.0, TAB_X1, TAB_YC - TAB_W / 2, Y1), r(PLATE_X1 - 1.0, TAB_X1, -Y1, -TAB_YC + TAB_W / 2)]
        for q in parts:
            f = q if f is None else f + q
    f = f + Pos(0, -Y1 + BAR_W / 2) * Rectangle(2 * STRIP_X0 + 2.0, BAR_W)
    yj = BOSS_Y + math.sqrt((BOSS_D / 2) ** 2 - (NARROW_X0 - SPACER_XY[0][0]) ** 2)   # boss arc meets X 22.5

    def at(pts, tol=0.05):
        return [v for v in f.vertices() if any(abs(abs(v.X) - x) < tol and abs(v.Y - y) < tol for x, y in pts)]
    convex = at([(TAB_X1, Y1), (TAB_X1, TAB_YC - TAB_W / 2), (TAB_X1, -Y1), (TAB_X1, -TAB_YC + TAB_W / 2), (NARROW_X0, Y1)])
    roots = at([(PLATE_X1, TAB_YC - TAB_W / 2), (PLATE_X1, -TAB_YC + TAB_W / 2), (STRIP_X0, -Y1 + BAR_W)])
    junction = at([(NARROW_X0, yj)])
    assert (len(convex), len(roots), len(junction)) == (10, 6, 2), (len(convex), len(roots), len(junction))
    f = fillet(convex, R_OUT)
    f = fillet([v for v in f.vertices() if any(abs(v.X - q.X) < 0.05 and abs(v.Y - q.Y) < 0.05 for q in roots)], R_ROOT)
    f = fillet([v for v in f.vertices() if any(abs(v.X - q.X) < 0.05 and abs(v.Y - q.Y) < 0.05 for q in junction)], R_IN)
    return f


def build():
    from assemblies.froggy_v12 import _finish_print
    face = _outline()
    web = extrude(face.moved(Location((0, 0, WEB_Z0))), WEB_T)
    tall = extrude(face.moved(Location((0, 0, WEB_Z0))), TAB_Z1 - WEB_Z0)
    keep = None
    rise = TAB_Z1 - WEB_Z1                          # 3: tab top over the web top, climbed by a 45 deg ramp
    prof = Polygon((PLATE_X1 - 1.0, WEB_Z0), (TAB_X1 + 5, WEB_Z0), (TAB_X1 + 5, TAB_Z1), (TAB_X0 + rise, TAB_Z1),
                   (TAB_X0, WEB_Z1), (PLATE_X1 - 1.0, WEB_Z1), align=None)
    for (x, z), rr in (((TAB_X0 + rise, TAB_Z1), R_RAMP_TOP), ((TAB_X0, WEB_Z1), R_RAMP_FOOT)):
        prof = fillet([v for v in prof.vertices() if abs(v.X - x) < 1e-3 and abs(v.Y - z) < 1e-3], rr)
    for sx in (1, -1):                              # the raised tabs: ramp from X 34, over the root blends too
        for sy in (1, -1):
            y_lo, y_hi = TAB_YC - TAB_W / 2 - R_ROOT - 1.0, Y1 + 5.0      # 33..63: over the root blend too
            b = extrude(Plane.XZ * prof, (y_hi - y_lo) / 2, both=True)
            b = b.moved(Location((0, sy * (y_lo + y_hi) / 2, 0)))
            if sx < 0:
                b = b.mirror(Plane.YZ)
            keep = b if keep is None else keep + b
    p = one((web + (tall & keep)).clean())
    for x, y in SPACER_XY:
        p = p - extrude(Pos(x, y) * Circle(HOLE_D / 2), 40).moved(Location((0, 0, WEB_Z0 - 10)))
    for x, y in TAB_SCREWS:
        p = p - extrude(Pos(x, y) * SlotOverall(SLOT_L, SLOT_W), 40).moved(Location((0, 0, WEB_Z0 - 10)))
    p = one(p.clean())
    # top edges R 2 in one pass (the outline is tangent-continuous after the 2D rounds, so it is one chain per loop);
    # the screw slots' rims and the print-face edges stay for the foot chamfer
    import math
    from build123d import Edge
    from cadkit.brep import classify_edges
    top = [Edge(e) for e, i, _, _ in classify_edges(p.wrapped, min_len=0.3)
           if i["convex"] and i["dihedral_deg"] >= 20 and Edge(e).center().Z > WEB_Z0 + 0.1
           and min(math.hypot(Edge(e).center().X - x, Edge(e).center().Y - y) for x, y in TAB_SCREWS) > SLOT_L]
    p = one(p.fillet(R_TOP, top))
    p = _finish_print(p, "-Z", outer=0.0, inner=0.0, foot=0.5)      # elephant-foot chamfer on the bed face only
    p.label = "fp9_motor_cradle"
    return p


def features():
    """Datums the staging scene places hardware from."""
    return dict(web_z0=WEB_Z0, web_z1=WEB_Z1, tab_z1=TAB_Z1, spacer_xy=list(SPACER_XY), tab_screws=TAB_SCREWS,
                print_face=SPEC["print_face"])
