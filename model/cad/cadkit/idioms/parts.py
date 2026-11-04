"""Finished building blocks. Every idiom returns (shape, features):

- shape: a build123d Solid (or a Compound when the idiom makes two parts), labelled with a
  readable name whose prefix tags its kind for `cadkit lint` (fp_ printed, lp_ laser, mp_ machined).
- features: a small dict of named features (hole centres, insert positions, mount face, print face)
  so callers place mating parts from it and never recompute.

All dimensions in mm. Z is up; printed parts are modelled in their print orientation unless the
docstring says otherwise, and `features['print_face']` names the bed face.
"""
import math

from build123d import (Align, Axis, Box, Compound, Cone, Cylinder, Location, Plane, Polyline, Pos, Rectangle,
                       RectangleRounded, Rot, Solid, Torus, extrude, make_face, revolve)

from ..brep import as_solid
from . import table as T
from .finish import finish

ZMIN = (Align.CENTER, Align.CENTER, Align.MIN)
ZMAX = (Align.CENTER, Align.CENTER, Align.MAX)


def _named(shape, name):
    s = as_solid(shape)
    s.label = name
    return s


# ---------------------------------------------------------------------------------------------
# hole patterns


def rect_pattern(nx=2, ny=2, px=40.0, py=30.0, centre=(0.0, 0.0)):
    """Points of an nx by ny grid at pitch px, py, centred on `centre`. Rule: DL9 equal pitch."""
    cx, cy = centre
    return [(cx + (i - (nx - 1) / 2) * px, cy + (j - (ny - 1) / 2) * py) for j in range(ny) for i in range(nx)]


def circle_pattern(n=6, r=30.0, start_deg=0.0, centre=(0.0, 0.0)):
    """n points equally spaced on a circle of radius r. Rule: DL9 3- or 4-fold symmetry on an axis."""
    cx, cy = centre
    return [(cx + r * math.cos(math.radians(start_deg + 360.0 * k / n)),
             cy + r * math.sin(math.radians(start_deg + 360.0 * k / n))) for k in range(n)]


def mirrored(points, axis="y"):
    """points plus their mirror images. axis 'y' mirrors x -> -x (about the Y axis), 'x' mirrors
    y -> -y, 'xy' both. Duplicates on the axis are dropped. Rule: DL9 symmetry."""
    out = list(points)
    def add(p):
        if not any(abs(p[0] - q[0]) < 1e-6 and abs(p[1] - q[1]) < 1e-6 for q in out):
            out.append(p)
    for x, y in list(points):
        if "y" in axis:
            add((-x, y))
        if "x" in axis:
            add((x, -y))
        if axis == "xy":
            add((-x, -y))
    return out


def with_d(points, d):
    """[(x, y)] -> [(x, y, d)] for the `holes=` argument."""
    return [(x, y, d) for x, y in points]


# ---------------------------------------------------------------------------------------------
# plates and blocks


def filleted_plate(w=80.0, h=50.0, t=3.175, r=None, holes=None, screw="M3", counterbore=None,
                   countersink=None, kind="laser", edge_r=None, name=None):
    """Flat plate with rounded corners and a clean hole layout.

    Rule source: DL8 (laser: outside corners R 3, R 6 over 100 mm, hole centres 2 x d from any edge,
    through-cut only) and DL7 (printed: R 2 / R 4 corners, filleted top edges, foot chamfer).
    holes: [(x, y, d)]; default is 4 corner holes for `screw`, inset 2 x d (and at least r).
    counterbore / countersink: a screw size ('M3'); printed plates only, since a laser cut cannot
    make them (the request is dropped and noted in features['notes'] for kind='laser').
    Print face (printed): -Z. Plate spans Z 0..t, centred on X/Y.
    """
    notes = []
    if r is None:
        r = 6.0 if max(w, h) > 100 else (4.0 if (kind == "printed" and max(w, h) > 40) else 3.0)
    d = T.clearance(screw, printed=(kind == "printed"))
    if holes is None:
        inset = math.ceil(max(2 * d, r + d / 2) * 2) / 2
        holes = with_d(rect_pattern(2, 2, w - 2 * inset, h - 2 * inset), d)
    p = extrude(RectangleRounded(w, h, r), t)
    for x, y, dd in holes:
        p = p - Pos(x, y, 0) * Cylinder(dd / 2, t, align=ZMIN)
    if kind == "laser" and (counterbore or countersink):
        notes.append("counterbore/countersink dropped: laser parts are through-cut only (DL5)")
        counterbore = countersink = None
    if counterbore:
        cd, cdep = T.counterbore(counterbore)
        cdep = min(cdep, t - T.PRINTED_WALL)   # keep a 2.4 mm floor under the head (DL7)
        for x, y, _ in holes:
            p = p - Pos(x, y, t) * Cylinder(cd / 2, cdep, align=ZMAX)
    if countersink:
        sd, _ = T.countersink(countersink)
        for x, y, dd in holes:
            hgt = (sd - dd) / 2
            p = p - Pos(x, y, t - hgt) * Cone(dd / 2, sd / 2, hgt, align=ZMIN)
    s = as_solid(p)
    if kind == "printed":
        s = finish(s, outer_r=edge_r or min(1.0, t / 3), inner_r=0.5, print_face="-Z", foot=0.4)
    s = _named(s, name or f"{'fp' if kind == 'printed' else 'lp'}_plate_{w:g}x{h:g}x{t:g}")
    return s, dict(kind=kind, holes=[(x, y, dd) for x, y, dd in holes], top_z=t, mount_face="-Z",
                   print_face="-Z" if kind == "printed" else None, outline=(w, h, r), t=t, notes=notes)


def printed_block(w=40.0, d=25.0, h=15.0, r=None, edge_r=None, foot=0.5, name=None):
    """Printed block, rounded all round: vertical corners R (R 4 over 40 mm, else R 3), top edges
    filleted, elephant-foot chamfer on the bed face. Never a bare box.

    Rule source: DL7 (printed form). Print face -Z; block spans Z 0..h, centred on X/Y.
    """
    r = r if r is not None else (4.0 if max(w, d) > 40 else 3.0)
    edge_r = edge_r if edge_r is not None else min(r - 0.5, 2.0, h / 4)
    s = as_solid(extrude(RectangleRounded(w, d, r), h))
    s = finish(s, outer_r=edge_r, inner_r=1.0, print_face="-Z", foot=foot)
    s = _named(s, name or f"fp_block_{w:g}x{d:g}x{h:g}")
    return s, dict(kind="printed", top_z=h, print_face="-Z", footprint=(w, d, r), centre=(0.0, 0.0, h / 2))


# ---------------------------------------------------------------------------------------------
# bosses and standoffs


def _boss_body(size, height, top_r=0.6):
    ins = T.insert(size)
    R = T.boss_od(size) / 2
    body = Cylinder(R, height, align=ZMIN)
    depth = min(ins["length"] + 0.8, height - 0.8)
    body = body - Pos(0, 0, height) * Cylinder(ins["hole_d"] / 2, depth, align=ZMAX)
    lead = 0.5
    body = body - Pos(0, 0, height - lead) * Cone(ins["hole_d"] / 2, ins["hole_d"] / 2 + lead, lead, align=ZMIN)
    s = as_solid(body)
    top = [e for e in s.edges() if abs(e.center().Z - height) < 1e-6 and abs(e.radius - R) < 1e-6] \
        if hasattr(s.edges()[0], "radius") else []
    try:
        if top:
            s = as_solid(s.fillet(top_r, top))
    except Exception:
        pass
    return s, R, depth, ins


def _root_fillet(host, R, at, root_r):
    x0, y0, z0 = at
    cand = []
    for e in host.edges():
        c = e.center()
        if abs(c.Z - z0) < 1e-3:
            try:
                if abs(e.radius - R) < 1e-3 and math.hypot(e.arc_center.X - x0, e.arc_center.Y - y0) < 1e-3:
                    cand.append(e)
            except Exception:
                continue
    if not cand:
        return host
    for rr in (root_r, root_r / 2):
        try:
            new = as_solid(host.fillet(rr, cand))
            if new.is_valid:
                return new
        except Exception:
            continue
    return host


def boss_with_insert(size="M3", height=8.0, host=None, at=(0.0, 0.0, 0.0), root_r=1.5, name=None):
    """Printed boss for a heat-set insert (sizes from table.HEAT_SET): wall >= 2.4 around the pocket,
    0.5 mm lead-in, top edge rounded, root blended into the host with root_r.

    Rule source: DL7 (walls 2.4 minimum, inside corners rounded); insert pocket per the Ruthex /
    CNC Kitchen table. With host=None the boss stands on a 2 mm round footing so it prints alone
    (print face -Z). With a host, the boss is unioned at `at` (its base, axis +Z) and the root is
    filleted; the host's own finish is kept.
    """
    boss, R, depth, ins = _boss_body(size, height)
    x0, y0, z0 = at
    if host is None:
        foot_r = R + root_r + 3.0
        pad = Cylinder(foot_r, 2.0, align=ZMIN)
        s = as_solid(pad + Pos(0, 0, 2.0) * boss)
        s = _root_fillet(s, R, (0, 0, 2.0), root_r)
        s = finish(s, outer_r=0.8, inner_r=0, print_face="-Z", foot=0.4)
        top = (0.0, 0.0, 2.0 + height)
        s = _named(s, name or f"fp_boss_insert_{T.norm(size).lower().replace('.', '_')}")
        return s, dict(kind="printed", size=T.norm(size), insert=ins, insert_top=top, insert_axis=(0, 0, -1),
                       pocket_d=ins["hole_d"], pocket_depth=depth, boss_od=2 * R, print_face="-Z",
                       screw_clearance=T.clearance(size))
    s = as_solid(as_solid(host) + Pos(x0, y0, z0) * boss)
    s = _root_fillet(s, R, at, root_r)
    s.label = getattr(host, "label", None) or name or "fp_part_with_boss"
    return s, dict(kind="printed", size=T.norm(size), insert=ins, insert_top=(x0, y0, z0 + height),
                   insert_axis=(0, 0, -1), pocket_d=ins["hole_d"], pocket_depth=depth, boss_od=2 * R)


BOARDS = {   # mounting hole pattern (x pitch, y pitch), board outline, thread
    "pi_zero": dict(pitch=(58.0, 23.0), board=(65.0, 30.0), size="M2.5"),
    "pi4": dict(pitch=(58.0, 49.0), board=(85.0, 56.0), size="M2.5"),
    "pi5": dict(pitch=(58.0, 49.0), board=(85.0, 56.0), size="M2.5"),
}


def standoff_mount(board="pi_zero", size=None, height=6.0, plate_t=3.0, margin=6.0, screw="M3", name=None):
    """Printed carrier plate with four insert bosses for a board, a lightening window under the board
    and two plate mounting holes on the long axis.

    Rule source: DL7 (printed form, walls 2.4), DL9 (equal pitch, symmetric), DL6 (printed parts
    locate, bolted to structure). board: a BOARDS key or a (pitch_x, pitch_y) tuple.
    Print face -Z. Insert tops are at Z plate_t + height.
    """
    spec = BOARDS.get(board) if isinstance(board, str) else dict(pitch=tuple(board), board=None, size=size or "M2.5")
    size = size or spec["size"]
    px, py = spec["pitch"]
    od = T.boss_od(size)
    d = T.clearance(screw, printed=True)
    w = px + od + 2 * margin + 4 * d
    h = py + od + 2 * margin
    mount = [(-(w / 2 - 2 * d), 0.0), (w / 2 - 2 * d, 0.0)]
    plate = as_solid(extrude(RectangleRounded(w, h, 4.0), plate_t))
    for x, y in mount:
        plate = as_solid(plate - Pos(x, y, 0) * Cylinder(d / 2, plate_t, align=ZMIN))
    win_w, win_h = px - od - 2 * 2.4, py - od - 2 * 2.4
    if win_w > 8 and win_h > 8:
        plate = as_solid(plate - extrude(RectangleRounded(win_w, win_h, min(4.0, win_h / 2 - 0.5)), plate_t))
    plate = finish(plate, outer_r=1.0, inner_r=0.5, print_face="-Z", foot=0.4)
    pts = rect_pattern(2, 2, px, py)
    s = plate
    feats = None
    for x, y in pts:
        s, feats = boss_with_insert(size, height, host=s, at=(x, y, plate_t), root_r=1.5)
    s = _named(s, name or f"fp_standoff_mount_{board if isinstance(board, str) else 'custom'}")
    return s, dict(kind="printed", board=board, size=size, inserts=[(x, y, plate_t + height) for x, y in pts],
                   insert_axis=(0, 0, -1), plate_holes=[(x, y, d) for x, y in mount], plate_t=plate_t,
                   print_face="-Z", pocket_depth=feats["pocket_depth"], outline=(w, h))


def round_rod_post(d=10.0, length=60.0, thread="M4", depth=10.0, end_chamfer=0.8, name=None):
    """Round rod (turned or cut from bar) with tapped ends: both outer ends chamfered, a 90 deg
    countersunk lead-in and a 118 deg drill point at the bottom of each tap-drill bore.
    The thread itself is cosmetic (bore at tap-drill size). Axis Z, spans Z 0..length.

    Rule source: DL3/DL8 practice: no sharp outside edge, tapped depth >= 1.5 x d of the thread.
    """
    tap = T.TAP_DRILL[T.norm(thread)]
    major = float(T.norm(thread)[1:])
    rod = as_solid(Cylinder(d / 2, length, align=ZMIN))
    rod = as_solid(rod.chamfer(end_chamfer, None, rod.edges()))
    for z0, sgn in ((0.0, 1), (length, -1)):
        loc = Location((0, 0, z0), (0, 0, 0)) if sgn == 1 else Location((0, 0, z0), (180, 0, 0))
        bore = Cylinder(tap / 2, depth, align=ZMIN)
        point = Pos(0, 0, depth) * Cone(tap / 2, 0.0, (tap / 2) / math.tan(math.radians(59)), align=ZMIN)
        lead_h = (major + 0.4 - tap) / 2
        lead = Cone(major / 2 + 0.2, tap / 2, lead_h, align=ZMIN)
        rod = as_solid(rod - loc * (bore + point + lead))
    rod = _named(rod, name or f"mp_rod_post_d{d:g}x{length:g}_{T.norm(thread)}_tapped")
    return rod, dict(kind="machined", ends=[((0, 0, 0), (0, 0, -1)), ((0, 0, length), (0, 0, 1))],
                     thread=T.norm(thread), tap_drill=tap, depth=depth, d=d, length=length)


# ---------------------------------------------------------------------------------------------
# panels, seats, cradles, brackets, grommets


def panel_cutout_with_lip(cut_w=40.0, cut_h=24.0, corner_r=3.0, panel=None, at=(0.0, 0.0), panel_w=120.0,
                          panel_h=80.0, panel_t=3.0, lip_wall=2.4, flange=4.0, flange_t=2.0, depth=None,
                          clearance=0.2, name=None):
    """Cut a rounded window in a panel and make the printed lip ring (bezel) that trims it: a
    flange on the front face and a sleeve through the panel.

    Rule source: DL8 (rounded inside corners on laser cuts), DL11 (show faces: no raw cut edge on
    show), DL21 (controls grouped). The panel lies in XY, Z 0..panel_t, front face +Z.
    Returns (cut panel Solid, features) with features['lip'] the bezel Solid (print face +Z: the
    show face prints on the bed). Retained by friction fit or a glue bead.
    """
    ax, ay = at
    depth = depth if depth is not None else panel_t + 3.0
    if panel is None:
        panel, _ = filleted_plate(panel_w, panel_h, panel_t, holes=[], kind="laser", name="lp_panel_with_cutout")
    hole = Pos(ax, ay, -1) * extrude(RectangleRounded(cut_w, cut_h, corner_r), panel_t + 2)
    cut = _named(as_solid(panel) - hole, getattr(panel, "label", None) or "lp_panel_with_cutout")
    so_w, so_h, so_r = cut_w - 2 * clearance, cut_h - 2 * clearance, corner_r - clearance
    op_w, op_h = so_w - 2 * lip_wall, so_h - 2 * lip_wall
    op_r = max(so_r - lip_wall, 1.0)
    fl = Pos(0, 0, panel_t) * extrude(RectangleRounded(cut_w + 2 * flange, cut_h + 2 * flange, corner_r + flange),
                                      flange_t)
    sleeve = Pos(0, 0, panel_t - depth) * extrude(RectangleRounded(so_w, so_h, so_r), depth)
    window = Pos(0, 0, panel_t - depth - 1) * extrude(RectangleRounded(op_w, op_h, op_r), depth + flange_t + 2)
    lip = as_solid(fl + sleeve - window)
    lip = finish(lip, outer_r=0.8, inner_r=0.6, print_face="+Z", foot=0.4)
    lip = _named(Pos(ax, ay, 0) * lip, name or "fp_cutout_lip")
    return cut, dict(kind="laser", lip=lip, lip_kind="printed", opening=(op_w, op_h, op_r), cut=(cut_w, cut_h, corner_r),
                     at=(ax, ay), front_z=panel_t, lip_print_face="+Z")


def _profile_xz(points):
    """Closed polygon in the XZ plane (x = radius or width, second coordinate = Z)."""
    return make_face(Plane.XZ * Polyline(*points, close=True))


def _fillet2d(face, picks, default=None):
    """Fillet 2D vertices of `face`: picks = [((x, z), r)], matched to the nearest vertex."""
    from build123d import fillet as fillet2d
    for (px, pz), r in picks:
        vs = [v for v in face.vertices() if abs(v.X - px) < 1e-4 and abs(v.Z - pz) < 1e-4]
        if not vs:
            continue
        try:
            face = fillet2d(vs, r)
        except Exception:
            pass
    return face


def printed_seat_ring(tube_od=100.0, tube_wall=3.0, depth=6.0, wall=2.4, base_t=3.0, clearance=0.3, bolts=3,
                      screw="M3", name=None):
    """Annular seat with a channel the end of a tube drops into (like the froggy drum seats):
    inner and outer lips with round tops, a bolt flange outside, filleted channel and root,
    elephant-foot chamfers. Revolved from one 2D profile so every edge is designed, not patched.

    Rule source: DL7 (printed form, wall 2.4), DL9 (bolts equally spaced, 3- or 4-fold), DL6
    (printed seat locates, bolted to the aluminium under it). Print face -Z; axis Z.
    """
    d = T.clearance(screw, printed=True)
    ci = tube_od / 2 - tube_wall - clearance
    co = tube_od / 2 + clearance
    ri = ci - wall
    ro_lip = co + wall
    rb = ro_lip + 1.5 + T.head(screw, "button")["dk"] / 2
    ro = rb + max(2 * d, T.head(screw, "button")["dk"] / 2 + 2.0)
    top = base_t + depth
    pts = [(ri, 0), (ro, 0), (ro, base_t), (ro_lip, base_t), (ro_lip, top), (co, top), (co, base_t),
           (ci, base_t), (ci, top), (ri, top)]
    f = _profile_xz(pts)
    f = _fillet2d(f, [((ri, top), wall / 2 - 0.05), ((ci, top), wall / 2 - 0.05), ((co, top), wall / 2 - 0.05),
                      ((ro_lip, top), wall / 2 - 0.05), ((ro_lip, base_t), 1.5), ((co, base_t), 0.5),
                      ((ci, base_t), 0.5), ((ro, base_t), 1.0)])
    s = as_solid(revolve(f, Axis.Z))
    holes = circle_pattern(bolts, rb, 90.0)
    for x, y in holes:
        s = as_solid(s - Pos(x, y, 0) * Cylinder(d / 2, base_t, align=ZMIN))
    s = finish(s, outer_r=0.8, inner_r=0.5, print_face="-Z", foot=0.5)
    s = _named(s, name or f"fp_seat_ring_tube_{tube_od:g}")
    return s, dict(kind="printed", bolt_holes=[(x, y, d) for x, y in holes], bolt_circle_r=rb, seat_z=base_t,
                   channel=(ci, co), outer_r=ro, print_face="-Z")


def cradle(item=(40.0, 60.0, 25.0), wall=3.0, clearance=0.4, wall_h=None, flange=10.0, base_t=3.0, screw="M3",
           gussets=True, window=True, name=None):
    """Open U cradle for a bought box: two side walls with round tops, a base with a window under the
    item, an outside flange with an aligned hole row each side, and gussets that tie walls to flange.

    Rule source: DL2 (bought boxes are housed by a bracket that keeps them in the frame line), DL7
    (printed form), DL9 (aligned hole rows). item = (width X, length Y, height Z) of the held part;
    item bottom centre sits at features['item_origin']. Print face -Z.
    """
    iw, il, ih = item
    wi = iw + 2 * clearance
    hw = wall_h if wall_h is not None else max(8.0, 0.6 * ih)
    L = il + 2 * clearance
    xo = wi / 2 + wall
    xf = xo + flange
    top = base_t + hw
    pts = [(-xf, 0), (xf, 0), (xf, base_t), (xo, base_t), (xo, top), (wi / 2, top), (wi / 2, base_t),
           (-wi / 2, base_t), (-wi / 2, top), (-xo, top), (-xo, base_t), (-xf, base_t)]
    f = _profile_xz(pts)
    f = _fillet2d(f, [((xo, top), wall / 2 - 0.05), ((wi / 2, top), wall / 2 - 0.05), ((-wi / 2, top), wall / 2 - 0.05),
                      ((-xo, top), wall / 2 - 0.05), ((xo, base_t), 2.0), ((-xo, base_t), 2.0),
                      ((wi / 2, base_t), 1.0), ((-wi / 2, base_t), 1.0), ((xf, base_t), 1.0), ((-xf, base_t), 1.0)])
    body = as_solid(extrude(f, L / 2, both=True))
    d = T.clearance(screw, printed=True)
    xh = (xo + xf) / 2
    ys = [-(L / 2 - 2 * d), L / 2 - 2 * d]
    holes = [(sx * xh, y) for sx in (-1, 1) for y in ys]
    if gussets:
        g_h, g_run = 0.7 * hw, flange - 2.0
        tri = _profile_xz([(xo - 0.5, base_t - 0.5), (xo + g_run, base_t - 0.5), (xo - 0.5, base_t + g_h)])
        for y in (0.0,) if L < 60 else (-L / 4, L / 4):
            for sx in (1, -1):
                g = Pos(0, y, 0) * extrude(tri, wall / 2, both=True)
                if sx < 0:
                    g = g.mirror(Plane.YZ)
                body = as_solid(body + g)
    for x, y in holes:
        body = as_solid(body - Pos(x, y, 0) * Cylinder(d / 2, base_t, align=ZMIN))
    if window and wi > 20 and L > 24:
        body = as_solid(body - extrude(RectangleRounded(wi - 12, L - 16, 4.0), base_t))
    body = finish(body, outer_r=0.8, inner_r=1.0, print_face="-Z", foot=0.5)
    body = _named(body, name or f"fp_cradle_{iw:g}x{il:g}x{ih:g}")
    return body, dict(kind="printed", item_origin=(0.0, 0.0, base_t), holes=[(x, y, d) for x, y in holes],
                      inner=(wi, L), wall_top_z=top, print_face="-Z")


def bracket_l(a=30.0, b=30.0, w=20.0, t=4.0, inside_r=None, kind="printed", screw="M3", holes_per_leg=1,
              gussets=True, name=None):
    """L bracket. Leg A lies on Z 0..t along +X (length a), leg B stands at X 0..t along +Z (height b),
    width w along Y (centred).

    printed: concentric bend (inside R = inside_r, default t; outside R = inside_r + t) so the wall is
    uniform, side gussets that leave the screw heads clear, every edge finished. Print face -Y
    (on its side, so layers wrap the bend).
    laser: a bent sheet (inside bend R = t, 6061 rule of thumb), leg-end corners R 3 (DL8).
    Holes: centred in each leg's free length, 2 x d from the end; equal pitch when 2 per leg.
    Rule source: DL7 / DL8 (form), DL9 (aligned rows), DL10 (one screw family per bracket).
    """
    ir = inside_r if inside_r is not None else t
    orr = ir + t
    printed = kind == "printed"
    d = T.clearance(screw, printed=printed)
    pts = [(0, 0), (a, 0), (a, t), (t, t), (t, b), (0, b)]
    f = _profile_xz(pts)
    f = _fillet2d(f, [((t, t), ir), ((0, 0), orr)])
    body = as_solid(extrude(f, w / 2, both=True))
    free_a0, free_b0 = t + ir, t + ir
    xa = max(free_a0 + (a - free_a0) / 2, free_a0 + d)
    zb = max(free_b0 + (b - free_b0) / 2, free_b0 + d)
    ys = [0.0] if holes_per_leg == 1 else [-w / 4, w / 4]
    head_r = T.head(screw, "button")["dk"] / 2 + 0.5
    if printed and gussets:
        run_a, run_b = 0.75 * (a - t), 0.75 * (b - t)
        tri = _profile_xz([(t - 0.5, t - 0.5), (t + run_a, t - 0.5), (t - 0.5, t + run_b)])
        if holes_per_leg == 1:      # two side gussets, screw head clear in the middle
            g = min(t * 0.75, (w - 2 * head_r - 2) / 2)
            places = [(sy * (w / 2 - g / 2)) for sy in (1, -1)] if g >= 2.0 else []
        else:                       # one centre gusset between the two screw rows
            g = min(t * 0.75, 2 * (w / 4 - head_r - 1))
            places = [0.0] if g >= 2.0 else []
        for y0 in places:
            body = as_solid(body + Pos(0, y0, 0) * extrude(tri, g / 2, both=True))
    holes = []
    for y in ys:
        body = as_solid(body - Pos(xa, y, 0) * Cylinder(d / 2, t, align=ZMIN))
        body = as_solid(body - Pos(0, y, zb) * Rot(0, 90, 0) * Cylinder(d / 2, t, align=ZMIN))
        holes += [dict(leg="a", centre=(xa, y, 0.0), axis=(0, 0, -1)), dict(leg="b", centre=(0.0, y, zb), axis=(-1, 0, 0))]
    if printed:
        body = finish(body, outer_r=1.0, inner_r=1.0, print_face="-Y", foot=0.4)
    else:
        rr = min(3.0, w / 4)
        ends = [e for e in body.edges()
                if (abs(e.center().X - a) < 1e-4 and abs(e.center().Y) > w / 2 - 1e-4)
                or (abs(e.center().Z - b) < 1e-4 and abs(e.center().Y) > w / 2 - 1e-4)]
        ends = [e for e in ends if e.length <= t + 1e-4]
        try:
            body = as_solid(body.fillet(rr, ends))
        except Exception:
            pass
    body = _named(body, name or f"{'fp' if printed else 'lp'}_bracket_l_{a:g}x{b:g}x{w:g}")
    return body, dict(kind=kind, holes=holes, mount_a="-Z", mount_b="-X", print_face="-Y" if printed else None,
                      inside_r=ir, t=t)


def cable_grommet_hole(cable_d=6.0, panel_t=3.0, wall=2.2, flange=3.0, flange_t=2.0, slit=1.0, name=None):
    """Printed snap grommet for a cable through a panel: flange on the front, sleeve through the
    panel, a chamfered snap lip behind it, rounded bore ends (strain relief) and a slit so the
    cable drops in from the side.

    Rule source: DL13/DL15 (every pass-through has a grommet), DL7 (printed form, sleeve wall >= 2).
    The panel lies Z -panel_t..0 with its front face at Z 0; features['cutter'] is the panel hole
    solid. Print face +Z (the flange face prints on the bed).
    """
    bore = cable_d + 1.0
    hole_d = math.ceil((bore + 2 * wall) * 2) / 2
    so = hole_d - 0.2
    lip_h = 1.4
    fl = Cylinder(hole_d / 2 + flange, flange_t, align=ZMIN)
    sleeve = Pos(0, 0, -panel_t - 0.1) * Cylinder(so / 2, panel_t + 0.1, align=ZMIN)
    lip = Pos(0, 0, -panel_t - 0.1 - lip_h) * Cone(so / 2, hole_d / 2 + 0.5, lip_h * 0.5, align=ZMIN) \
        + Pos(0, 0, -panel_t - 0.1 - lip_h * 0.5) * Cylinder(hole_d / 2 + 0.5, lip_h * 0.5, align=ZMIN)
    g = as_solid(fl + sleeve + lip)
    z0 = -panel_t - 0.1 - lip_h
    g = as_solid(g - Pos(0, 0, z0 - 1) * Cylinder(bore / 2, flange_t - z0 + 2, align=ZMIN))
    # round the bore ends (strain relief) first: finish() leaves hole rims alone
    rims = [e for e in g.edges() if abs(e.center().X) < 1e-3 and abs(e.center().Y) < 1e-3
            and abs(getattr(e, "radius", 0) - bore / 2) < 1e-3]
    for rr in (0.8, 0.4):
        try:
            g = as_solid(g.fillet(rr, rims))
            break
        except Exception:
            continue
    if slit:
        g = as_solid(g - Pos(hole_d, 0, -20) * Box(2 * hole_d, slit, 40, align=ZMIN))
    g = finish(g, outer_r=0.3, inner_r=0.3, print_face="+Z", foot=0.3, min_len=0.2)
    g = _named(g, name or f"fp_grommet_cable_{cable_d:g}")
    cutter = Pos(0, 0, -panel_t - 1) * Cylinder(hole_d / 2, panel_t + 2, align=ZMIN)
    return g, dict(kind="printed", cutter=as_solid(cutter), hole_d=hole_d, bore_d=bore, panel_t=panel_t,
                   print_face="+Z")


IDIOMS = {
    "filleted_plate": lambda: filleted_plate(),
    "filleted_plate_printed": lambda: filleted_plate(60, 40, 6, kind="printed", counterbore="M3"),
    "printed_block": lambda: printed_block(),
    "boss_with_insert": lambda: boss_with_insert(),
    "standoff_mount": lambda: standoff_mount(),
    "panel_cutout_with_lip": lambda: panel_cutout_with_lip(),
    "printed_seat_ring": lambda: printed_seat_ring(),
    "cradle": lambda: cradle(),
    "bracket_l": lambda: bracket_l(),
    "bracket_l_laser": lambda: bracket_l(kind="laser", t=3.175),
    "cable_grommet_hole": lambda: cable_grommet_hole(),
    "round_rod_post": lambda: round_rod_post(),
}
