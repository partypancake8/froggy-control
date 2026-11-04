"""fp9_estop_cradle: PETG cradle that holds the BACO LBX10510 e-stop station recessed in the front panel (ICD S2.10).

Justification (DESIGN-RULES 1, printed): one open frame has to clamp to the 3 mm ABS panel round a 76 x 76 opening,
carry the station's two diagonal Ø4.5 base slots 40 mm behind the panel, hug the box on all four sides and leave a
window for the cable gland; a flat laser plate cannot reach that depth and no stock bracket matches the LBX pattern.
It carries only the station's own weight and the push on the mushroom head.

Local frame (as built here): X = world X, Y = world -Z, Z = depth into the plinth (world +Y), origin on the panel's
inner face at the station centre. The assembly places it with Plane(origin=(xc, -150, zc), x_dir=+X, z_dir=+Y).

Print orientation: flat on the flange (local Z 0, the face against the panel) on the bed, walls up, no supports. The
walls are vertical, the cable window's top is a 26 mm bridge, the four corner gussets of the back plate are bridges
between two walls (legs 40, hypotenuse 57) and nothing else overhangs.

Geometry:
  flange   108 x 79.4 x 8 (Z 0..8), plan corners R 6; it sits on the panel's inner face round the opening
  lip      the flange's inner edge is the box outline + 0.2 (74.4 square, R 3.2): it stands 0.8 inside the 76 x 76
           panel opening on every side, so the opening's 1 mm clearance gap is closed from behind, 0.2 off the box
  walls    from the flange to the back plate (Z 0..43.9) hugging the box at 0.2: sides 4 thick (X +-37.2..41.2), top and
           bottom 2.5 thick (to +-39.7, 0.3 inside the deck rail and the bottom ring), outer corners R 4
  back     Z 39.9..43.9 (the station's back pads at Z 39.9): the wall rim plus four corner gussets, legs 40, two of
           them with the station's Ø4.5 slots
  window   26 (Y) x 26 (Z 9.5..35.5) in the -X side wall, centred on the station's M16/20 knockout (Z 22.4)
  edges    the back rim's outer edges and the flange's back outer edges R 2

Hole table (local X, Y; diameter; ICD row):
  (+-46, +-25)  4.1 x 6 deep heat-set pocket + 3.4 relief through: M3 x 10 button head from the panel's outside, S2.10
  (29, -20.5)   4.5 through the back gusset: station slot, M4 x 12 button head + M4 nyloc, S2.10 (j_s2_10_estop_1)
  (-30.5, 22)   4.5 through the back gusset: station slot, M4 x 12 button head + M4 nyloc, S2.10 (j_s2_10_estop_2)

Install: bench, station cover off: base on the cradle's back plate, 2 x M4 x 12 from inside the base + nylocs; cradle
and station from behind through the panel opening with the panel off the rails, 4 x M3 x 10 from the panel's outside
into the inserts; cover on; panel on the rails (step 7).
"""
from build123d import Align, Box, Cylinder, Location, Polygon, RectangleRounded, extrude

from parts._util import fillet_safe, one

BOX = 74.0                    # BACO LBX single-position box (STEP 74 x 74), corners about R 3
BOX_R = 3.0
CLR = 0.2                     # cradle to box, every side
W_IN = BOX / 2 + CLR          # 37.2
WALL_SIDE = 4.0
OUT_V = 39.7                  # top / bottom walls' outer face: 0.3 inside the rails (rel +-40)
FL_U, FL_T, FL_R = 54.0, 8.0, 6.0
DEPTH = 39.9                  # panel inner face to the station's back pads
BACK_T = 4.0
GUSSET = 40.0
INSERTS = [(sx * 46.0, sy * 25.0) for sx in (1, -1) for sy in (1, -1)]
INSERT_D, INSERT_DEPTH, RELIEF_D = 4.1, 6.0, 3.4
SLOTS = [(29.0, -20.5), (-30.5, 22.0)]          # local (X, Y) = world rel (X, -Z)
SLOT_D = 4.5
KNOCKOUT_Z = 22.4                                # station's M16/20 knockout centre depth (world Y -127.6)
WIN_W, WIN_Z0, WIN_Z1 = 26.0, 9.5, 35.5
Z_TOP = DEPTH + BACK_T                           # 43.9


def build():
    hole = RectangleRounded(2 * W_IN, 2 * W_IN, BOX_R + CLR)
    flange = extrude(RectangleRounded(2 * FL_U, 2 * OUT_V, FL_R) - hole, FL_T)
    u_out = W_IN + WALL_SIDE
    walls = extrude(RectangleRounded(2 * u_out, 2 * OUT_V, 4.0) - hole, Z_TOP)
    p = flange + walls
    c = W_IN
    for su in (1, -1):                           # corner gussets of the back plate
        for sv in (1, -1):
            tri = Polygon((su * c, sv * c), (su * (c - GUSSET), sv * c), (su * c, sv * (c - GUSSET)), align=None)
            p = p + extrude(tri, BACK_T, dir=(0, 0, 1)).moved(Location((0, 0, DEPTH)))
    p = one(p.clean())
    # R 2 on the back rim's outer edges and the flange's back outer edges
    top = [e for e in p.edges() if abs(e.center().Z - Z_TOP) < 1e-3
           and (abs(abs(e.center().Y) - OUT_V) < 1e-3 or abs(abs(e.center().X) - u_out) < 1e-3)]
    p, _ = fillet_safe(p, 2.0, top)
    fl = [e for e in p.edges() if abs(e.center().Z - FL_T) < 1e-3
          and (abs(abs(e.center().X) - FL_U) < 1e-3 or (abs(e.center().X) > u_out + 1 and abs(abs(e.center().Y) - OUT_V) < 1e-3))]
    p, _ = fillet_safe(p, 2.0, fl)
    for x, y in INSERTS:
        p = p - Cylinder(INSERT_D / 2, INSERT_DEPTH, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((x, y, 0)))
        p = p - Cylinder(RELIEF_D / 2, FL_T + 2, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((x, y, -1)))
    for x, y in SLOTS:
        p = p - Cylinder(SLOT_D / 2, 20).moved(Location((x, y, DEPTH)))
    p = p - Box(WALL_SIDE + 2, WIN_W, WIN_Z1 - WIN_Z0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((-(W_IN + WALL_SIDE / 2), 0, WIN_Z0)))
    return one(p)
