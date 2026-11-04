"""fp9_cage: the S3 cage printed parts (I030, 2026-10-21): a tube-post cube on the deck with magnet-on mesh panels.

Sawyer (2026-10-21): "really the cage should be a structure that is put over some alu posts or soemthing like that,
that forms a cube ideally the panels have magnets on them so you can pull any one off like a mesh door thing".

Custom parts (DESIGN-RULES 1, printed): no open design is a three-way 12 mm tube corner with magnet seats on its outer
faces that also bolts to a 2020 deck rail, and no stock frame holds a no-see-um mesh panel to the 217.4 mm cube face
(the BugDorm-style pole corners and spline frames are the idea, not files to import). The corners only locate the
tubes and carry the panels' magnets; the panels only hold netting. Nothing here carries the machine (DL29) or the
rotor (DL6).

Parts (local frames; the assembly places them):
  corner_top()        3-way top corner: an L-tripod of 19.4 square bars, node centred on the origin with its outer faces
                      +X, +Y, +Z (the panel planes); two horizontal arms along -X and -Y (19.4 square, z -11.3..9.7,
                      flush with the node top) and a round post socket boss OD 18.2 down from the node (0.6 inside the
                      panel planes), each with a 12.1 x 15 push-fit tube socket (the horizontal ones teardrop, roof
                      down in this frame); magnet seats: teardrop pockets 8.2 x 3.3 on +X and +Y, a through hole on
                      +Z down to the post socket. Prints upside down on its top face, no supports
                      (parts/fp9_cage_corner_top.py SPEC).
  corner_foot(hand)   foot corner: square base 19.4 x 19.4 x 20 on the deck sheet (local Z 0 = deck top), outer faces
                      +X, +Y, magnet teardrop pockets on both at Z 11, a round socket boss OD 18.2 on top with the
                      12.1 x 15 tube socket (Z 20..35), and a 4 thick tongue along +X to the 5.5 hole at (41, -2) for
                      one M5 x 12 button head into a T-nut in the deck Y rail. hand "b" is the mirror image about local
                      Y 0 (front right and back left). Prints on its deck face.
  panel(part, notch)  mesh panel frame: bars BAR_W x BAR_T (8 x 4), local X across the panel (centred), local Y up from
                      the bottom edge, local Z 0 = outer face, Z BAR_T = inner face (on the node faces); four corner
                      gussets (opening corners R 36) with 8.2 magnet through holes at 9.7 from both edges (the node
                      face centres; the bottom pair at 10.9 from the bottom edge, under the foot socket); a 1.5 deep
                      45 deg V groove 3 wide on the inner face round each opening for the spline that holds the netting;
                      side panels in two halves joined by a 10 tall half-lap (lower half: outer 2 of the lap, upper
                      half: inner 2) with 2 x M3 x 8 button head + hex nut, nut inside; the top panel in one piece
                      (217.4 square fits the bed). part: "lower" | "upper" | "top". notch: the X panels' lower halves
                      clear the foot tongues. Print: the lower half and the top panel on the outer face, the upper half
                      on its inner face (its lap on the bed; the V groove's 45 deg walls need no supports).
  mesh_outline()      the netting outline per panel (the openings grown to the groove's outer edge, less the corner
                      faces in the panel plane and the lap screws).
"""
import math

import numpy as np

from build123d import (Align, Axis, Box, Cylinder, Face, Kind, Location, Plane, Polygon, Pos, Rectangle, RectangleRounded,
                       Solid, extrude, mirror)

from lib.design_constants import PRINTED

# ---------------------------------------------------------------- shared numbers (world placement in froggy_v9)
TUBE_OD, TUBE_WALL = 12.0, 1.0      # 6061 tube 12 x 1 (vendor/stock/aluminium_tube_12x1)
BORE = 12.1                         # push-fit socket bore (0.05 radial)
SOCK = 15.0                         # socket depth
NODE = 19.4                         # node cube: its outer faces are the panel plane
NH = NODE / 2                       # 9.7
ARM = 18.2                          # arm section: wall (18.2 - 12.1) / 2 = 3.05 (structural >= 3.0)
ARM_DZ = NH - ARM / 2               # 0.6: the horizontal sockets (and the top rails) sit 0.6 over the node centre
NODE_BOT = 11.3                     # the top node and its horizontal arms reach 11.3 under the node centre: 3.3 of wall
                                    # under the teardrop roof of the horizontal sockets (printed upside down)
MAG_D, MAG_T = 7.94, 3.175          # K&J D52 5/16 x 1/8 N42 (vendor/hardware/magnet_8x3_n42)
POCKET_D, POCKET_T = 8.2, 3.3       # magnet pocket (node) / through hole (panel); magnet face 0.05 under the face
MAG_RECESS = 0.05
FOOT_H = 35.0                       # foot node height on the deck sheet
FOOT_SOCK_Z = FOOT_H - SOCK         # 20: socket floor (the tube foot)
FOOT_MAG_Z = 11.0                   # foot magnets: centre 11 above the deck top (teardrop pocket roof at 16.8, 3.2 under
                                    # the socket floor)
TONGUE_T = 4.0
TONGUE_HOLE = (41.0, -2.0)          # foot screw hole from the post axis: world (+-140, +-97), on the deck Y rail
TONGUE_END_R = 9.5
M5_HOLE = 5.5
# panels
BAR_W, BAR_T = 8.0, 4.0              # 8 wide (the proposal's about 10 x 4, narrowed for the 6600 g cap)
GUSSET_R = 36.0                     # opening corner radius at the node corners (holds the magnet hole and the groove)
OPEN_R = 4.0                        # opening corner radius at the lap
OUTER_R = 4.0                       # panel outline corners (above the R 2 edge fillet: no degenerate blend)
GROOVE_IN, GROOVE_W, GROOVE_D = 3.0, 3.0, 1.5   # V groove 3.0..6.0 outside the opening edge, 1.5 deep, 45 deg walls
GROOVE_STOP = 3.0                   # the halves' grooves stop 3 short of the lap
LAP_H = 10.0
LAP_LAND = 1.0                      # full-thickness land between each opening and the lap step (mid bar 12 tall)
LAP_SCREW_X = 70.0                  # +-70 from the panel centre, on the lap centre line
M3_HOLE = 3.4
MESH_T = 0.3                        # modelled netting (real fabric about 0.3)
MESH_OUT = GROOVE_IN + GROOVE_W     # the netting reaches the groove's outer edge
FOOT_CUT = NODE                     # node footprint cut out of the netting (+0.3 margin, see mesh_outline)
EDGE_GAP = 0.1                      # panel bottom edge over the deck sheet / side panel top under the top panel


def _solid(s):
    if isinstance(s, Solid):
        return s
    sols = s.solids()
    if len(sols) != 1:
        raise ValueError(f"expected one solid, got {len(sols)}")
    return sols[0]


def _box(x0, x1, y0, y1, z0, z1):
    return Box(x1 - x0, y1 - y0, z1 - z0, align=(Align.MIN, Align.MIN, Align.MIN)).moved(Location((x0, y0, z0)))


def _finish(s, print_face, outer=None, inner=None):
    from cadkit.idioms import finish
    from lib.design_constants import fillet_printed
    bb = s.bounding_box()
    r = fillet_printed(max(bb.size.X, bb.size.Y, bb.size.Z)) if outer is None else outer
    return _solid(finish(_solid(s), outer_r=r, inner_r=PRINTED["fillet_inside"] if inner is None else inner,
                         print_face=print_face, foot=0.4, log=[]))


def _plan_wire(sketch):
    """Outer wire of the fused outline of a multi-face Sketch (algebra-mode sketch unions are not fused)."""
    sols = [extrude(f, 1.0) for f in sketch.faces()]
    sol = sols[0]
    for x in sols[1:]:
        sol = sol + x
    sol = _solid(sol.clean())
    bot = min((f for f in sol.faces() if f.normal_at(f.center()).Z < -0.99), key=lambda f: f.center().Z)
    return bot.outer_wire()


def _fillet_try(s, r, edges):
    edges = list(edges)
    if not edges:
        return s
    try:
        out = _solid(s.fillet(r, edges))
        return out if out.is_valid else s
    except Exception:  # noqa: BLE001
        return s


# ---------------------------------------------------------------- corners

def _teardrop(start, axis, up, r, length):
    """Teardrop prism (a circle of radius r plus a 45 deg roof pointing along `up`) from `start` along `axis`: a
    horizontal bore or pocket that prints without supports (the roof faces are 45 deg)."""
    ax = np.asarray(axis, float)
    u = np.asarray(up, float)
    w = np.cross(u, ax)
    pl = Plane(origin=tuple(float(v) for v in start), x_dir=tuple(float(v) for v in w),
               z_dir=tuple(float(v) for v in ax))
    c = pl * Cylinder(r, length, align=(Align.CENTER, Align.CENTER, Align.MIN))
    a = 0.97 * r                          # side corners just inside the circle (no tangent seam), apex at r * sqrt 2
    sq = pl * Box(a, a, length, align=(Align.CENTER, Align.CENTER, Align.MIN)).rotate(Axis.Z, 45).moved(
        Location((0, r * math.sqrt(2) - a / math.sqrt(2), 0)))
    return _solid(c + sq)


def corner_top():
    """3-way top corner, local frame (see the module docstring). The two horizontal arms are flush with the node's top
    face (it prints on that face): their bores sit ARM_DZ over the node centre."""
    # one L-tripod of 19.4 square bars sharing their outer faces. Each bar's four long edges are rounded R 2 before the
    # union and the socket mouths R 1 after it (a fillet pass over the merged corner makes a B-spline vertex blend with
    # a loose bounding box); the top face (the print face) keeps sharp edges plus the elephant-foot chamfer.
    r_out = PRINTED["fillet_outside_min"]
    bars = [(_box(-NH - SOCK, NH, -NH, NH, -NODE_BOT, NH), "X"), (_box(-NH, NH, -NH - SOCK, NH, -NODE_BOT, NH), "Y")]
    s = None
    for b_, ax in bars:
        long_ = [e for e in b_.edges() if abs(getattr(e.tangent_at(0), ax)) > 0.99
                 and not (ax != "Z" and abs(e.center().Z - NH) < 1e-6)]
        b_ = _fillet_try(_solid(b_), r_out, long_)
        s = b_ if s is None else s + b_
    # the post socket points up on the bed, so it is a round boss (OD ARM, 0.6 inside the panel planes)
    s = s + Cylinder(ARM / 2, SOCK + 0.5, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(
        Location((0, 0, -NODE_BOT + 0.5)))
    s = _solid(s).clean()
    mouths = [e for e in s.edges() if abs(e.center().X + NH + SOCK) < 1e-6 or abs(e.center().Y + NH + SOCK) < 1e-6
              or abs(e.center().Z + NODE_BOT + SOCK) < 1e-6]
    mouths = [e for e in mouths if abs(e.center().Z - NH) > 1e-6]
    s = _fillet_try(s, 1.0, mouths)
    up = (0.0, 0.0, -1.0)                                      # printed upside down: local -Z is up on the bed
    zc = ARM_DZ
    s = s - Cylinder(BORE / 2, SOCK + 1, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(
        Location((0, 0, -NODE_BOT)))
    s = s - _teardrop((-NH - SOCK - 1, 0, zc), (1, 0, 0), up, BORE / 2, SOCK + 1)
    s = s - _teardrop((0, -NH - SOCK - 1, zc), (0, 1, 0), up, BORE / 2, SOCK + 1)
    # magnet pockets: +X and +Y teardrop pockets, +Z a through hole to the post socket (no ceiling on the bed face)
    s = s - _teardrop((NH + 1, 0, 0), (-1, 0, 0), up, POCKET_D / 2, POCKET_T + 1)
    s = s - _teardrop((0, NH + 1, 0), (0, -1, 0), up, POCKET_D / 2, POCKET_T + 1)
    s = s - Cylinder(POCKET_D / 2, NH + NODE_BOT + 2).moved(Location((0, 0, (NH - NODE_BOT) / 2)))
    s = _solid(s)
    s.label = "fp9_cage_corner_top"
    return s


def corner_foot(hand="a"):
    """Foot corner, local frame (Z 0 = deck sheet top, post axis on Z). The plan outline (node square plus the tongue
    to its round end) is rounded in 2D (outside R 2, the tongue-to-node inside corner R 1, DL7); the node's top edges
    and the tongue's top edges get R 2 / R 1.5 fillets (a 3D pass over the tangent tongue edges crashes OCC)."""
    from build123d import CenterArc, Line, Wire
    hx, hy = TONGUE_HOLE
    r_out, r_in = PRINTED["fillet_outside_min"], PRINTED["fillet_inside"]
    R = TONGUE_END_R
    w = Wire([Line((-NH, -NH), (NH, -NH)), Line((NH, -NH), (hx, hy - R)), CenterArc((hx, hy), R, -90.0, 180.0),
              Line((hx, hy + R), (NH, NH)), Line((NH, NH), (-NH, NH)), Line((-NH, NH), (-NH, -NH))])
    for d in (r_in, -r_in, -r_out, r_out):
        w = w.offset_2d(d, kind=Kind.ARC)
    tongue = _solid(extrude(Face(w), TONGUE_T))
    node = _solid(extrude(RectangleRounded(NODE, NODE, r_out + 1.0), FOOT_SOCK_Z))   # R 3 plan corners under the R 2 top fillet
    s = _solid(tongue + node)
    top = [e for e in s.edges() if abs(e.center().Z - FOOT_SOCK_Z) < 1e-6]
    s = _fillet_try(s, r_out, top)
    boss = Cylinder(ARM / 2, SOCK + 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, FOOT_SOCK_Z - 1.0)))
    s = _solid(s + boss)
    s = _fillet_try(s, 1.0, [e for e in s.edges() if abs(e.center().Z - FOOT_H) < 1e-6])
    tt = [e for e in s.edges() if abs(e.center().Z - TONGUE_T) < 1e-6 and e.center().X > NH + 0.5]
    s = _fillet_try(s, 1.5, tt)
    s = s - Cylinder(BORE / 2, SOCK + 1, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, FOOT_SOCK_Z)))
    s = s - Cylinder(M5_HOLE / 2, 20).moved(Location((hx, hy, 0)))
    s = s - _teardrop((NH + 1, 0, FOOT_MAG_Z), (-1, 0, 0), (0, 0, 1), POCKET_D / 2, POCKET_T + 1)
    s = s - _teardrop((0, NH + 1, FOOT_MAG_Z), (0, -1, 0), (0, 0, 1), POCKET_D / 2, POCKET_T + 1)
    s = _solid(s)
    if hand == "b":
        s = _solid(mirror(s, about=Plane.XZ))
    s.label = f"fp9_cage_corner_foot_{hand}"
    return s


# ---------------------------------------------------------------- panels

def panel_size(post_l, kind):
    """(width, height) of a side or top panel for posts `post_l` long."""
    w = 2 * (99.0 + NH)                               # edges on the node outer faces (post axis 99, see froggy_v9)
    if kind == "top":
        return w, w
    top_z = FOOT_SOCK_Z + post_l + NODE_BOT + NH     # node top above the deck top
    return w, top_z - 2 * EDGE_GAP


def _rr_wire(x0, x1, y0, y1, radii):
    """Rounded rectangle face with per-corner radii (bl, br, tr, tl)."""
    s = Rectangle(x1 - x0, y1 - y0, align=(Align.MIN, Align.MIN)).moved(Location((x0, y0)))
    f = s.faces()[0]
    rb = max(radii)
    face = f
    # build by subtracting corner squares and adding quarter circles
    out = face
    corners = [((x0, y0), (1, 1)), ((x1, y0), (-1, 1)), ((x1, y1), (-1, -1)), ((x0, y1), (1, -1))]
    for ((cx, cy), (dx, dy)), r in zip(corners, radii):
        if r <= 0:
            continue
        sq = Rectangle(r, r, align=(Align.MIN, Align.MIN)).moved(Location((cx if dx > 0 else cx - r,
                                                                            cy if dy > 0 else cy - r)))
        out = out - sq
        from build123d import Circle
        out = out + Circle(r).moved(Location((cx + dx * r, cy + dy * r)))
    del rb
    return out


def _opening(x0, x1, y0, y1, radii):
    return _rr_wire(x0, x1, y0, y1, radii)


def panel_openings(part, w, h):
    """[(x0, x1, y0, y1, radii)] of the openings of one panel part (local)."""
    i = w / 2 - BAR_W
    ys = h / 2
    if part == "top":
        return [(-i, i, BAR_W, h - BAR_W, (GUSSET_R,) * 4)]
    if part == "lower":
        return [(-i, i, BAR_W, ys - LAP_H / 2 - LAP_LAND, (GUSSET_R, GUSSET_R, OPEN_R, OPEN_R))]
    return [(-i, i, ys + LAP_H / 2 + LAP_LAND, h - BAR_W, (OPEN_R, OPEN_R, GUSSET_R, GUSSET_R))]


def magnet_points(part, w, h, side=True):
    """Magnet hole centres (local x, y) of one panel part."""
    x = w / 2 - NH
    ybot = FOOT_MAG_Z - EDGE_GAP if side else NH
    ytop = h - NH + (EDGE_GAP if side else 0.0)
    if part == "top":
        return [(sx * x, y) for y in (NH, h - NH) for sx in (-1, 1)]
    if part == "lower":
        return [(sx * x, ybot) for sx in (-1, 1)]
    return [(sx * x, ytop) for sx in (-1, 1)]


def lap_screws(w, h):
    return [(sx * LAP_SCREW_X, h / 2) for sx in (-1, 1)]


def panel(part="lower", post_l=300.0, notch=False):
    """One printed mesh panel part (local frame, module docstring)."""
    w, h = panel_size(post_l, "top" if part == "top" else "side")
    ys = h / 2
    if part == "top":
        y0, y1 = 0.0, h
    elif part == "lower":
        y0, y1 = 0.0, ys + LAP_H / 2
    else:
        y0, y1 = ys - LAP_H / 2, h
    rad = {"top": (OUTER_R,) * 4, "lower": (OUTER_R, OUTER_R, 0, 0), "upper": (0, 0, OUTER_R, OUTER_R)}[part]
    outline = _rr_wire(-w / 2, w / 2, y0, y1, rad)
    ops = panel_openings(part, w, h)
    face = outline
    for o in ops:
        face = face - _opening(*o)
    s = _solid(extrude(face, BAR_T))
    pf = "+Z" if part == "upper" else "-Z"
    s = _finish(s, pf, outer=PRINTED["fillet_outside_min"], inner=0.0)
    # half-lap
    if part == "lower":
        s = s - _box(-w, w, ys - LAP_H / 2, ys + LAP_H, BAR_T / 2, BAR_T + 1)
    elif part == "upper":
        s = s - _box(-w, w, ys - LAP_H, ys + LAP_H / 2, -1, BAR_T / 2)
    # V groove on the inner face round each opening (3 wide at the face, 1.5 deep, 45 deg walls): a triangle swept
    # along the groove centre line; on the halves it stops GROOVE_STOP short of the lap
    for o in ops:
        g = _v_groove(o)
        if part != "top":
            if part == "lower":
                g = g - _box(-w, w, ys - LAP_H / 2 - GROOVE_STOP, h + 10, -5, 10)
            else:
                g = g - _box(-w, w, -10, ys + LAP_H / 2 + GROOVE_STOP, -5, 10)
        s = s - g
    # magnet through holes
    for x, y in magnet_points(part, w, h, side=part != "top"):
        s = s - Cylinder(POCKET_D / 2, 20).moved(Location((x, y, 0)))
    if part in ("lower", "upper"):
        for x, y in lap_screws(w, h):
            s = s - Cylinder(M3_HOLE / 2, 20).moved(Location((x, y, 0)))
    if notch and part == "lower":
        for sx in (-1, 1):
            nx0, nx1 = sorted((sx * (w / 2 + 1), sx * (w / 2 - NODE - 2.4)))
            s = s - _box(nx0, nx1, -1, TONGUE_T + 0.5 - EDGE_GAP, -1, BAR_T + 1)
    s = _solid(s)
    s.label = f"fp9_cage_panel_{part}"
    return s


def _v_groove(o):
    """Swept 45 deg V groove round opening `o` (x0, x1, y0, y1, radii): centre line GROOVE_IN + GROOVE_W / 2 outside
    the opening edge, apex GROOVE_D under the inner face (Z BAR_T), the triangle 0.5 proud of the face."""
    from build123d import Transition, sweep
    x0, x1, y0, y1, rr = o
    d = GROOVE_IN + GROOVE_W / 2
    path = _plan_wire(_rr_wire(x0 - d, x1 + d, y0 - d, y1 + d, tuple(r + d for r in rr)))
    p0 = path.position_at(0)
    t0 = path.tangent_at(0)
    xd = np.cross((0, 0, 1), (t0.X, t0.Y, t0.Z))
    hw = GROOVE_W / 2 + 0.5
    pl = Plane(origin=(p0.X, p0.Y, 0), x_dir=tuple(float(v) for v in xd), z_dir=(t0.X, t0.Y, t0.Z))
    tri = pl * Polygon((-hw, BAR_T + 0.5), (hw, BAR_T + 0.5), (0.0, BAR_T - GROOVE_D), align=None)
    g = sweep(tri, path=path, transition=Transition.RIGHT)
    return _solid(g)


def mesh_cuts(part_kind, w, h):
    """Rectangles (x0, x1, y0, y1) in panel-local coordinates where a corner part's face lies in the panel plane:
    the netting is cut there (the corners' faces carry the magnets). Side panels: the top corner's node and its horizontal
    arm along the panel (the round post socket sits 0.6 inside the plane); the foot corner's square base (its round socket boss sits 0.6 inside the
    plane). Top panel: the top corners' node and both horizontal arms."""
    m, big = 0.3, 50.0
    e = w / 2
    out = []
    for sx in (-1, 1):
        def xs(inner):
            a, b = sx * (e - inner), sx * (e + big)
            return min(a, b), max(a, b)
        if part_kind == "top":
            for sy in (-1, 1):
                for ix, iy in ((NODE + SOCK + m, NODE + m), (NODE + m, NODE + SOCK + m)):
                    y0, y1 = sorted((sy * (e - iy), sy * (e + big)))
                    out.append(xs(ix) + (y0 + e, y1 + e))
        else:
            zc = h - NH + EDGE_GAP                      # top node centre, panel-local
            out.append(xs(NODE + SOCK + m) + (zc - NODE_BOT - m, h + big))
            out.append(xs(NODE + m) + (-big, FOOT_SOCK_Z - EDGE_GAP + m))
    return out


def mesh_outline(part_kind, w, h, post_l=300.0):
    """Netting outline (a Face, local panel coords): the openings grown by MESH_OUT, less mesh_cuts() and the lap
    screws (their nuts sit on the inner face)."""
    from build123d import Circle
    i = w / 2 - BAR_W
    m = MESH_OUT
    f = _rr_wire(-i - m, i + m, BAR_W - m, h - BAR_W + m, (GUSSET_R + m,) * 4)
    for x0, x1, y0, y1 in mesh_cuts(part_kind, w, h):
        f = f - Rectangle(x1 - x0, y1 - y0, align=(Align.MIN, Align.MIN)).moved(Location((x0, y0)))
    if part_kind != "top":
        for x, y in lap_screws(w, h):
            f = f - Circle(3.75).moved(Location((x, y)))
    return f
