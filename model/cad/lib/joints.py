"""Bolt joints as checked records (ported idea from fusion-pipeline fk_joints,
no Fusion code). A joint places a real screw and nut solid in the assembly and
returns a record whose flags are computed from the geometry:

fit_ok               clearance hole diameter in [3.2, 3.6] for M3, axis on the joint axis
head_seated          head underside face touches the bracket (distance 0 within 0.05)
thread_engagement_ok at least 3 mm of screw length inside the nut along the axis
nut_aligned          nut inside the rail cavity (no rail overlap, inside the rail
                     envelope) and its bore axis on the screw axis
no_interference      screw and nut overlap nothing (each other included) by >= 0.01 mm3
contact_ok           the bracket's mount faces touch their rail faces (0 within 0.05)
driver_path_clear    a 7 mm cylinder from the head outward 40 mm along the axis hits
                     no body except the screw (bodies are counted when they are
                     installed at or before the joint's install step; every frame
                     body and joint is step 0, so the frame check is unchanged)

Joint kinds (record field `kind`, the nut field names the same thing):
slot_nut         M3 into a 15-series slot nut in a rail (frame stage, tower flange)
hex_nut          screw through both parts into an ISO 4032 hex nut bearing on the
                 host; engagement = the whole nut height with >= 0.5 mm protrusion
heat_set_insert  screw into a heat-set insert (real solid) in a printed part;
                 engagement = the whole insert length
tapped           screw into a tapped hole of a vendor body (no nut, nut_aligned is
                 True by definition); engagement >= 1.5 x d into the tapped body,
                 or, where the vendor's modelled hole is shallower than that, the
                 full modelled thread depth less 0.25 mm and never under 1.0 x d;
                 the tip may not pass the thread unless the hole is tapped through
"""
import numpy as np
from build123d import Align, Cylinder, Location, Plane

from lib import hardware as HW
from lib import vendor as V

FLAGS = ("fit_ok", "head_seated", "thread_engagement_ok", "nut_aligned", "no_interference",
         "contact_ok", "driver_path_clear")
FIT = {"M3": (3.2, 3.6), "M4": (4.2, 5.05), "M5": (5.3, 5.8), "M2.5": (2.6, 3.0)}   # M4 up to 5.0: the CL42T flange slots
HOLE_R = {"M3": (1.2, 2.0), "M4": (1.9, 2.6), "M5": (2.4, 3.2), "M2.5": (1.26, 1.55)}  # clearance-hole radius search range per thread
TAP_R = {"M3": (1.0, 1.6), "M4": (1.3, 1.7), "M5": (1.9, 2.3), "M2.5": (0.95, 1.1)}   # tapped-hole (tap drill / thread crest) radius range
# how far from the joint axis the head seat annulus / the rail face may start (the clearance hole radius,
# or for the contact faces the slot opening half width): 2.5 / 3.0 for M3 and M4 (15-series), wider for
# M5 in the 2020 slot (5.5 hole, 6.0 slot opening)
SEAT_REACH = {"M3": 2.5, "M4": 2.5, "M5": 3.5, "M2.5": 2.0}
CONTACT_REACH = {"M3": 3.0, "M4": 3.0, "M5": 4.0, "M2.5": 2.5}
TOL_CONTACT = 0.05
TOL_VOL = 0.01
MIN_ENGAGEMENT = 3.0
DRIVER_D, DRIVER_L = 7.0, 40.0


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _perp(axis):
    a = np.array([1.0, 0, 0]) if abs(axis[0]) < 0.9 else np.array([0, 1.0, 0])
    return _unit(np.cross(axis, a))


def _proj(solid, origin, axis):
    vals = [(np.array(tuple(v)) - origin) @ axis for v in solid.vertices()]
    return min(vals), max(vals)


def _rail_dir(solid):
    b = solid.bounding_box()
    size = np.array(tuple(b.max)) - np.array(tuple(b.min))
    d = np.zeros(3)
    d[int(np.argmax(size))] = 1.0
    return d


def _seat(bracket, at, axis, max_back=20.0, reach=2.5):
    """Head seat: the nearest planar face of the bracket facing back up the
    axis (normal = -axis) that the axis passes through, behind `at`."""
    best = None
    for f, n, c in V.planar_faces(bracket):
        if n @ -axis < 1 - 1e-6:
            continue
        s = (c - at) @ -axis        # distance behind the mount face
        if not 0.1 < s < max_back:
            continue
        p = at - axis * s
        # the seat is an annulus around the clearance hole: the axis point lies
        # in the hole, within the hole radius plus its 0.4 edge fillet
        if f.distance_to(tuple(p)) > reach:
            continue
        if best is None or s < best:
            best = s
    if best is None:
        raise ValueError("no head seat face on the bracket along the joint axis")
    return at - axis * best


def bolt_joint(assembly, parts, at, axis, screw="M3x8", nut="slot_nut", name=None, module=None,
               evaluate=True, contact="mount_faces", step=0, head="shcs", seat=None, tongue_top=None, hw_tag=None,
               stack=None):
    """Place `screw` through parts[0] (the clamped part, head side) into a `nut`
    in parts[1] (the rail). `at` is where the joint axis crosses the interface
    between the two parts; `axis` points from the head toward the nut.
    head="csk": countersunk screw, `seat` = its head-top point (flush face).
    tongue_top: slot nut tongue depth (see lib.hardware.slot_nut).
    stack: [(name, thickness), ...] sheets clamped between parts[0] and the rail, listed from the
    rail outward (e.g. the deck sheet under the pedestal flange); contact is then checked at every
    interface of the stack (contact "stack")."""
    if nut != "slot_nut":
        return fastened_joint(assembly, parts, at, axis, screw=screw, kind=nut, name=name, module=module,
                              evaluate=evaluate, step=step, head=head, seat=seat, hw_tag=hw_tag)
    thread, length = HW.parse(screw)
    at, axis = np.asarray(at, float), _unit(axis)
    name = name or f"j_{len(assembly.joints) + 1:02d}"
    bracket, rail = assembly[parts[0]], assembly[parts[1]]
    module = module or assembly.parts[parts[0]].module
    seat = _seat(bracket, at, axis, reach=SEAT_REACH.get(thread, 2.5)) if seat is None else np.asarray(seat, float)
    x_dir = _perp(axis)
    tag = hw_tag or name
    s_name, n_name = f"hw_{screw.lower()}_{head}_{tag}", f"hw_slot_nut_{thread.lower()}_{tag}"
    s = HW.screw(screw, head=head).moved(Location(Plane(origin=tuple(seat), x_dir=tuple(x_dir), z_dir=tuple(-axis))))
    rdir = _rail_dir(rail)
    nkw = {} if tongue_top is None else dict(tongue_top=tongue_top)
    n = HW.slot_nut(thread, **nkw).moved(Location(Plane(origin=tuple(at), x_dir=tuple(rdir), z_dir=tuple(-axis))))
    assembly.add(s_name, s, "screw", module, joint=name, step=step)
    assembly.add(n_name, n, "nut", module, joint=name, step=step)
    spec = dict(name=name, kind="slot_nut", screw=screw, nut=nut, thread=thread, length=length, parts=list(parts),
                at=[round(float(v), 4) for v in at], axis=[round(float(v), 6) for v in axis],
                seat=[round(float(v), 4) for v in seat], screw_part=s_name, nut_part=n_name,
                contact="stack" if stack else contact, step=step, head=head)
    if stack:
        spec["stack"] = [[n, float(t)] for n, t in stack]
    assembly.joints.append(spec)
    return evaluate_joint(assembly, spec) if evaluate else dict(spec)


def fastened_joint(assembly, parts, at, axis, screw="M3x10", kind="tapped", name=None, module=None,
                   evaluate=True, step=0, head="shcs", seat=None, max_back=20.0, hw_tag=None, stack=None,
                   bench=None, washer=False, min_engagement=None, nut_washer=False, far=None, tap_depth=None):
    """Screw through parts[0] (clamped part, head side) into parts[1] (host):
    kind 'tapped' (vendor tapped hole), 'heat_set_insert' (insert solid placed
    flush with the host face at `at`) or 'hex_nut' (nut placed on the far face
    of the host, found along the axis). `at` = where the axis crosses the
    interface between the two parts; `axis` points from the head toward the host.
    hw_tag: names the screw / nut / insert components instead of the joint name
    (the joint record keeps `name`).
    stack: [(name, thickness), ...] spacers or sheets clamped between parts[0] and the host,
    listed from the host outward; `at` is then the host face and contact is checked at every
    interface (as bolt_joint).
    bench: names of the only bodies present when the joint is made (a bench sub-assembly put
    in the machine afterwards); the driver path is checked against those alone.
    nut_washer: hex_nut only, an ISO 7089 plain washer between the host's far face and the nut (nuts on acrylic);
    nut_aligned then checks washer-on-host and nut-on-washer.
    washer: True puts an ISO 7089 plain washer (lib.hardware.WASHER) on the clamped part's seat and
    the screw head on the washer; head_seated then checks head-on-washer and washer-on-part.
    far: hex_nut only, the nut's seat on the host's far face when the vendor STEP's slot has no planar face the
    axis search can find (as `seat` does for the head side).
    min_engagement: tapped joints only, a stated thread engagement (mm) that replaces the 1.5 x d rule where the
    vendor drawing's hole depth and the ICD fastener fix it (recorded in the joint record).
    tap_depth: tapped joints only, how deep the thread is cut from `at` where the modelled tap-drill bore runs
    further than the thread (a 2020 centre bore tapped at its end); caps the measured thread depth."""
    thread, length = HW.parse(screw)
    at, axis = np.asarray(at, float), _unit(axis)
    name = name or f"j_{len(assembly.joints) + 1:02d}"
    clamped, host = assembly[parts[0]], assembly[parts[1]]
    module = module or assembly.parts[parts[0]].module
    seat = _seat(clamped, at, axis, max_back, reach=SEAT_REACH.get(thread, 2.5)) if seat is None else \
        np.asarray(seat, float)
    x_dir = _perp(axis)
    w_name = None
    if washer:
        w_name = f"hw_washer_{thread.lower()}_{hw_tag or name}"
        w = HW.washer(thread).moved(Location(Plane(origin=tuple(seat), x_dir=tuple(x_dir), z_dir=tuple(-axis))))
        assembly.add(w_name, w, "washer", module, joint=name, step=step)
        seat = seat - axis * HW.WASHER[thread]["h"]
    s_name = f"hw_{screw.lower()}_{head}_{hw_tag or name}"
    s = HW.screw(screw, head=head).moved(Location(Plane(origin=tuple(seat), x_dir=tuple(x_dir), z_dir=tuple(-axis))))
    assembly.add(s_name, s, "screw", module, joint=name, step=step)
    n_name = None
    if kind == "heat_set_insert":
        n_name = f"hw_insert_{thread.lower()}_{hw_tag or name}"
        n = HW.heat_set_insert(thread).moved(Location(Plane(origin=tuple(at), x_dir=tuple(x_dir), z_dir=tuple(-axis))))
        assembly.add(n_name, n, "insert", module, joint=name, step=assembly.parts[parts[1]].meta.get("step", step))
    elif kind == "hex_nut":
        far = _far_face(host, at, axis, reach=SEAT_REACH.get(thread, 2.5)) if far is None else np.asarray(far, float)
        if nut_washer:
            nw_name = f"hw_nut_washer_{thread.lower()}_{hw_tag or name}"
            nw = HW.washer(thread).moved(Location(Plane(origin=tuple(far), x_dir=tuple(x_dir), z_dir=tuple(axis))))
            assembly.add(nw_name, nw, "washer", module, joint=name, step=step)
            far = far + axis * HW.WASHER[thread]["h"]
        n_name = f"hw_hex_nut_{thread.lower()}_{hw_tag or name}"
        n = HW.hex_nut(thread).moved(Location(Plane(origin=tuple(far), x_dir=tuple(x_dir), z_dir=tuple(-axis))))
        assembly.add(n_name, n, "nut", module, joint=name, step=step)
    elif kind != "tapped":
        raise ValueError(f"unknown joint kind {kind}")
    spec = dict(name=name, kind=kind, screw=screw, nut=kind, thread=thread, length=length, parts=list(parts),
                at=[round(float(v), 4) for v in at], axis=[round(float(v), 6) for v in axis],
                seat=[round(float(v), 4) for v in seat], screw_part=s_name, nut_part=n_name,
                contact="stack" if stack else "faces", step=step, head=head)
    if stack:
        spec["stack"] = [[n, float(t)] for n, t in stack]
    if bench is not None:
        spec["bench"] = list(bench)
    if w_name:
        spec["washer_part"] = w_name
    if kind == "hex_nut" and nut_washer:
        spec["nut_washer_part"] = nw_name
    if min_engagement is not None:
        spec["min_engagement"] = float(min_engagement)
    if tap_depth is not None:
        spec["tap_depth"] = float(tap_depth)
    assembly.joints.append(spec)
    return evaluate_joint(assembly, spec) if evaluate else dict(spec)


def tapped_joint(assembly, parts, at, axis, screw="M3x10", **kw):
    return fastened_joint(assembly, parts, at, axis, screw=screw, kind="tapped", **kw)


def insert_joint(assembly, parts, at, axis, screw="M3x8", **kw):
    return fastened_joint(assembly, parts, at, axis, screw=screw, kind="heat_set_insert", **kw)


def hex_nut_joint(assembly, parts, at, axis, screw="M3x16", **kw):
    return fastened_joint(assembly, parts, at, axis, screw=screw, kind="hex_nut", **kw)


def _far_face(host, at, axis, max_depth=30.0, reach=2.5):
    """Where the axis leaves the host on the far side: the nearest planar face
    of the host facing along +axis, crossed by the axis within `reach` (2.5 mm; the M5 5.5 hole needs 3.5)."""
    best = None
    for f, n, c in V.planar_faces(host):
        if n @ axis < 1 - 1e-6:
            continue
        s = (c - at) @ axis
        if not 0.1 < s < max_depth:
            continue
        p = at + axis * s
        if f.distance_to(tuple(p)) > reach:
            continue
        if best is None or s < best:
            best = s
    if best is None:
        raise ValueError("no far face on the host along the joint axis")
    return at + axis * best


def _face_contact_gap(clamped, host, at, axis, reach=3.0):
    """Contact at the joint: the clamped part's planar face facing the host
    (normal +axis) and the host's planar face facing back (normal -axis), both
    within `reach` of the axis point at the interface. Returns the gap between
    the two planes (99 when either face is missing)."""
    def nearest(solid, sign):
        best = None
        for f, n, c in V.planar_faces(solid):
            if n @ axis * sign < 1 - 1e-6:
                continue
            off = (c - at) @ axis
            if abs(off) > 0.5:
                continue
            p = at + axis * off
            if f.distance_to(tuple(p)) > reach:
                continue
            if best is None or abs(off) < abs(best):
                best = off
        return best
    a, b = nearest(clamped, 1), nearest(host, -1)
    if a is None or b is None:
        return 99.0
    return abs(a - b)


def _tapped_depth(host, at, axis, thread="M3"):
    """Full-thread depth of the tapped hole in `host` on this axis, measured
    from `at` along the axis (cylindrical faces of tap-drill radius, TAP_R)."""
    depth = 0.0
    for h in V.cyl_holes(host, *TAP_R[thread]):
        if abs(abs(h["axis"] @ axis) - 1) > 1e-4 or np.linalg.norm(np.cross(h["p0"] - at, axis)) > 0.05:
            continue
        lo, hi = _proj(h["face"], at, axis)
        depth = max(depth, hi)
    return depth


def _bore_d(clamped, seat, at):
    """Clear diameter of a non-cylindrical hole (a vendor STEP's drafted slot): twice the smallest distance from
    the joint axis segment seat -> interface to the clamped part. None when the axis runs through material."""
    from build123d import Edge
    seat, at = np.asarray(seat, float), np.asarray(at, float)
    if np.linalg.norm(at - seat) < 1e-6:
        return None
    r = clamped.distance_to(Edge.make_line(tuple(seat), tuple(at)))
    return 2 * r if r > 1e-3 else None


def evaluate_joint(assembly, spec):
    kind = spec.get("kind", "slot_nut")
    at, axis, seat = (np.array(spec[k], float) for k in ("at", "axis", "seat"))
    bname, rname = spec["parts"]
    bracket = assembly[bname]
    sc = assembly[spec["screw_part"]]
    nu = assembly[spec["nut_part"]] if spec.get("nut_part") else None
    rec = {k: spec[k] for k in ("name", "screw", "nut", "at", "axis", "parts")}
    rec["kind"] = kind
    rec["screw_part"], rec["nut_part"] = spec["screw_part"], spec.get("nut_part")
    rec["step"] = spec.get("step", 0)
    if spec.get("bench") is not None:
        rec["bench"] = spec["bench"]
    why = []

    # fit_ok: the clamped part's own cylindrical hole on this axis
    holes = [h for h in V.cyl_holes(bracket, *HOLE_R[spec["thread"]])
             if abs(abs(h["axis"] @ axis) - 1) < 1e-4
             and np.linalg.norm(np.cross(h["p0"] - at, axis)) < 0.05]
    d = 2 * holes[0]["radius"] if holes else None
    if d is None:
        d = _bore_d(bracket, seat, at)       # drafted or slotted vendor hole (cone / B-spline walls, no cylinder)
        if d is not None:
            rec["hole_measured"] = "min clearance from the axis between the seat and the interface"
    lo, hi = FIT[spec["thread"]]
    rec["hole_d_mm"] = round(d, 3) if d else None
    rec["fit_ok"] = bool(d is not None and lo <= d <= hi)
    if not rec["fit_ok"]:
        why.append(f"hole d {d}")

    # head_seated: head underside face of the screw = planar face facing +axis nearest the seat
    under = None
    for f, n, c in V.planar_faces(sc):
        if n @ axis > 1 - 1e-6:
            s = abs((c - seat) @ axis)
            if under is None or s < under[1]:
                under = (f, s)
    if spec.get("head") == "csk":
        # countersunk: the head cone bears on the countersink cone
        dist = sc.distance_to(bracket)
        rec["head_gap_mm"] = round(dist, 4)
        rec["head_seated"] = bool(dist <= TOL_CONTACT)
    elif spec.get("washer_part"):
        wa = assembly[spec["washer_part"]]
        dist = max(under[0].distance_to(wa) if under else 99.0, wa.distance_to(bracket))
        rec["washer_part"] = spec["washer_part"]
        rec["head_gap_mm"] = round(dist, 4)
        rec["head_seated"] = bool(under is not None and under[1] < TOL_CONTACT and dist <= TOL_CONTACT)
    else:
        dist = under[0].distance_to(bracket) if under else 99.0
        rec["head_gap_mm"] = round(dist, 4)
        rec["head_seated"] = bool(under is not None and under[1] < TOL_CONTACT and dist <= TOL_CONTACT)
    if not rec["head_seated"]:
        why.append(f"head gap {dist:.3f}")

    # thread_engagement_ok, per kind
    s0, s1 = _proj(sc, seat, axis)          # s1 = tip
    dia = HW.SCREW_DIMS[spec["thread"]]["d"]
    if kind == "tapped":
        face = (at - seat) @ axis            # host face along the axis from the seat
        tdepth = _tapped_depth(assembly[rname], at, axis, spec["thread"])
        if spec.get("tap_depth") is not None:
            tdepth = min(tdepth, spec["tap_depth"])
            rec["tap_depth_stated"] = spec["tap_depth"]
        eng = max(0.0, min(s1 - face, tdepth))
        need = 1.5 * dia if tdepth >= 1.5 * dia else max(1.0 * dia, tdepth - 0.25)
        if spec.get("min_engagement") is not None:
            need = spec["min_engagement"]
            rec["min_engagement_stated"] = need
        rec["tapped_depth_mm"] = round(tdepth, 3)
        # a hole tapped right through the host (its far face within 1 mm past the thread) lets the tip run out
        # into the air beyond; anything the tip would hit there is caught by no_interference
        try:
            far = float((_far_face(assembly[rname], at, axis) - at) @ axis)
        except ValueError:
            far = None
        through = bool(far is not None and far <= tdepth + 1.0)
        rec["tapped_through"] = through
        ok = eng >= need - 1e-3 and (s1 - face <= tdepth + 1e-3 or through)
    else:
        n0, n1 = _proj(nu, seat, axis)
        eng = max(0.0, min(s1, n1) - max(0.0, n0))
        if kind == "slot_nut":
            need = MIN_ENGAGEMENT
            ok = eng >= need
        elif kind == "hex_nut":
            need = n1 - n0
            rec["protrusion_mm"] = round(s1 - n1, 3)
            ok = eng >= need - 1e-3 and s1 - n1 >= 0.5
        else:                                 # heat_set_insert: the whole insert engaged
            need = n1 - n0
            ok = eng >= need - 1e-3
    rec["engagement_mm"] = round(eng, 3)
    rec["engagement_needed_mm"] = round(need, 3)
    rec["thread_engagement_ok"] = bool(ok)
    if not rec["thread_engagement_ok"]:
        why.append(f"engagement {eng:.2f} < {need:.2f}")

    # nut_aligned, per kind
    if kind == "tapped":
        rec["nut_aligned"] = True
    else:
        (rmn, rmx), (nmn, nmx) = assembly.bbox(rname), assembly.bbox(spec["nut_part"])
        v_host, _ = assembly.ivol(spec["nut_part"], rname)
        bores = V.cyl_holes(nu, *TAP_R[spec["thread"]])   # tapped bore (tap drill)
        on_axis = bool(bores) and all(abs(abs(b["axis"] @ axis) - 1) < 1e-4
                                      and np.linalg.norm(np.cross(b["p0"] - at, axis)) < 0.05 for b in bores)
        if kind == "slot_nut":
            inside = all(nmn[k] >= rmn[k] - 1e-3 and nmx[k] <= rmx[k] + 1e-3 for k in range(3))
            rec["nut_aligned"] = bool(inside and v_host < TOL_VOL and on_axis)
            if not rec["nut_aligned"]:
                why.append(f"nut inside={inside} rail_overlap={v_host:.4f} on_axis={on_axis}")
        elif kind == "heat_set_insert":
            inside = all(nmn[k] >= rmn[k] - 1e-3 and nmx[k] <= rmx[k] + 1e-3 for k in range(3))
            seated = nu.distance_to(assembly[rname]) <= 0.1      # sits in its pocket
            rec["nut_aligned"] = bool(inside and seated and v_host < TOL_VOL and on_axis)
            if not rec["nut_aligned"]:
                why.append(f"insert inside={inside} seated={seated} host_overlap={v_host:.4f} on_axis={on_axis}")
        else:                                 # hex_nut bears on the host's far face
            if spec.get("nut_washer_part"):
                nw = assembly[spec["nut_washer_part"]]
                gap = max(nu.distance_to(nw), nw.distance_to(assembly[rname]))
                rec["nut_washer_part"] = spec["nut_washer_part"]
            else:
                gap = nu.distance_to(assembly[rname])
            rec["nut_gap_mm"] = round(gap, 4)
            rec["nut_aligned"] = bool(gap <= TOL_CONTACT and v_host < TOL_VOL and on_axis)
            if not rec["nut_aligned"]:
                why.append(f"nut gap={gap:.3f} host_overlap={v_host:.4f} on_axis={on_axis}")

    # no_interference: screw and nut/insert against every other component
    worst = []
    own_parts = (spec["screw_part"], spec.get("nut_part"), spec.get("washer_part"), spec.get("nut_washer_part"))
    for own in own_parts:
        if not own:
            continue
        for other in assembly.parts:
            if other == own or (other in own_parts and spec.get("washer_part") in (own, other)):
                continue
            v, _ = assembly.ivol(own, other)
            if v >= TOL_VOL:
                worst.append(f"{own}x{other}={v:.3f}")
    rec["no_interference"] = not worst
    if worst:
        why.append("interference " + ",".join(worst[:3]))

    # contact_ok: mount faces on rails (frame brackets) or the two parts touch
    if spec.get("contact", "mount_faces") == "mount_faces":
        gaps = []
        for h in V.bracket_holes(bracket):
            f = h["mount_face"]
            gaps.append(min(f.distance_to(assembly[r]) for r in assembly.names("rail")
                            if _near(assembly, r, f)))
    elif spec["contact"] == "faces":
        gaps = [_face_contact_gap(bracket, assembly[rname], at, axis, CONTACT_REACH.get(spec["thread"], 3.0))]
    elif spec["contact"] == "stack":
        # parts[0] -> stack sheets (outermost first) -> rail, each interface checked on the axis
        reach = CONTACT_REACH.get(spec["thread"], 3.0)
        chain = [rname] + [n for n, _ in spec["stack"]]
        offs = [0.0]
        for _, t in spec["stack"]:
            offs.append(offs[-1] + t)
        chain.append(bname)
        gaps = [_face_contact_gap(assembly[chain[i + 1]], assembly[chain[i]], at - axis * offs[i], axis, reach)
                for i in range(len(chain) - 1)]
    else:
        gaps = [bracket.distance_to(assembly[rname])]
    rec["contact_gaps_mm"] = [round(g, 4) for g in gaps]
    rec["contact_ok"] = bool(gaps and max(gaps) <= TOL_CONTACT)
    if not rec["contact_ok"]:
        why.append(f"contact gaps {gaps}")

    # driver_path_clear: 7 mm x 40 mm cylinder from the head top outward,
    # against every body installed at or before this joint's step
    head_top = seat + axis * s0
    drv = Plane(origin=tuple(head_top), x_dir=tuple(_perp(axis)), z_dir=tuple(-axis)) * \
        Cylinder(DRIVER_D / 2, DRIVER_L, align=(Align.CENTER, Align.CENTER, Align.MIN))
    hits = []
    key = f"driver_{spec['name']}"
    step = spec.get("step", 0)
    bench = spec.get("bench")
    for other, part in assembly.parts.items():
        if other in (spec["screw_part"], spec.get("washer_part")) or part.meta.get("step", 0) > step:
            continue
        if bench is not None and other not in bench:
            continue
        v, _ = assembly.ivol(key, other, solid_a=drv)
        if v >= TOL_VOL:
            hits.append(f"{other}={v:.3f}")
    rec["driver_hits"] = hits
    rec["driver_path_clear"] = not hits
    if hits:
        why.append("driver path " + ",".join(hits[:3]))
    rec["ok"] = all(rec[k] for k in FLAGS)
    rec["why"] = "; ".join(why)
    return rec


def _near(assembly, rname, face, tol=0.5):
    b = face.bounding_box()
    from lib.assembly import boxes_touch
    return boxes_touch((tuple(b.min), tuple(b.max)), assembly.bbox(rname), tol)
