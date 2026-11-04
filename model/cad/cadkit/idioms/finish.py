"""finish(): turn a raw solid into a finished one without ever raising.

Rule source: froggy DESIGN-LANGUAGE DL7 (printed outside edges R 2, inside
corners R 1, no bare boxes) and DL8 (laser corners), plus the elephant-foot
chamfer every FDM part needs on its bed face.

Order: (1) fillet every convex edge (dihedral >= min_dihedral, length >=
min_len, not a hole rim, not lying in the print plane) with outer_r,
(2) fillet every concave edge the same way with inner_r, (3) chamfer the
edges of the print face (outer wire and non-round inner wires) with `foot`.
Each step tries all edges at once, then each connected chain, then each edge,
halving the radius once before giving up on an edge. A failed step keeps the
previous solid, so the result is always valid.
"""
import numpy as np
from build123d import Edge, Face, Solid

from ..brep import as_solid, classify_edges, edge_faces, faces_of, face_normal_mid, is_hole_face, surf_type

_DIRS = {"-Z": (0, 0, -1), "+Z": (0, 0, 1), "-X": (-1, 0, 0), "+X": (1, 0, 0), "-Y": (0, -1, 0), "+Y": (0, 1, 0)}


def _ok(new, old):
    try:
        s = as_solid(new)
        return isinstance(s, Solid) and s.is_valid and s.volume > 0.5 * old.volume
    except Exception:
        return False


def _print_plane(s, print_face):
    """(unit normal, offset) of the print plane: the planar face(s) with that outward normal at the
    extreme of the part along it."""
    if print_face is None:
        return None
    if isinstance(print_face, Face):
        n = np.array(tuple(print_face.normal_at()))
        return n, float(n @ np.array(tuple(print_face.center())))
    n = np.array(_DIRS[print_face], float)
    bb = s.bounding_box()
    pts = np.array([tuple(bb.min), tuple(bb.max)])
    return n, float(max(pts @ n))


def _in_plane(edge_w, plane, tol=1e-3):
    if plane is None:
        return False
    n, off = plane
    e = Edge(edge_w)
    for p in (e.position_at(0), e.position_at(0.5), e.position_at(1)):
        if abs(np.array(tuple(p)) @ n - off) > tol:
            return False
    return True


def _chains(edges):
    """Group edges into connected chains via shared vertices."""
    n = len(edges)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    verts = [[v for v in Edge(e).vertices()] for e in edges]
    for i in range(n):
        for j in range(i + 1, n):
            if any(a.wrapped.IsSame(b.wrapped) for a in verts[i] for b in verts[j]):
                parent[find(i)] = find(j)
    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(edges[i])
    return list(groups.values())


def _key(edge_w):
    e = Edge(edge_w)
    return tuple(np.round(np.array(tuple(e.position_at(0.5))), 2)) + (round(e.length, 2),)


def _fillet_pass(s, r, convex, plane, min_len, min_dihedral, log, max_ops=60):
    failed = set()
    for _ in range(max_ops):
        cands = [e for e, info, _, _ in classify_edges(s.wrapped, min_len=min_len)
                 if info["convex"] == convex and info["dihedral_deg"] >= min_dihedral
                 and not _in_plane(e, plane) and _key(e) not in failed]
        if not cands:
            return s
        try:
            new = s.fillet(r, [Edge(e) for e in cands])
            if _ok(new, s):
                log.append(f"fillet {'convex' if convex else 'concave'} r{r}: {len(cands)} edges")
                return as_solid(new)
        except Exception:
            pass
        progressed = False
        for chain in _chains(cands):
            for rr in (r, r / 2):
                try:
                    new = s.fillet(rr, [Edge(e) for e in chain])
                    if _ok(new, s):
                        s = as_solid(new)
                        log.append(f"fillet chain r{rr}: {len(chain)} edges")
                        progressed = True
                        break
                except Exception:
                    continue
            if progressed:
                break
            if len(chain) == 1:
                failed.add(_key(chain[0]))
                log.append(f"skip edge at {_key(chain[0])[:3]}")
            else:
                # split the chain: try its first edge alone next round, mark it failed if that fails too
                e0 = chain[0]
                for rr in (r, r / 2):
                    try:
                        new = s.fillet(rr, [Edge(e0)])
                        if _ok(new, s):
                            s = as_solid(new)
                            progressed = True
                            break
                    except Exception:
                        continue
                if progressed:
                    break
                failed.add(_key(e0))
                log.append(f"skip edge at {_key(e0)[:3]}")
        if not progressed and all(_key(e) in failed for e in cands):
            return s
    return s


def _print_face_edges(s, plane):
    n, off = plane
    out = []
    for f in faces_of(s.wrapped):
        if surf_type(f) != "plane":
            continue
        fn, c = face_normal_mid(f)
        if fn is None or fn @ n < 0.999 or abs(c @ n - off) > 1e-3:
            continue
        for e, f1, f2 in edge_faces(s.wrapped):
            if not (f1.IsSame(f) or f2.IsSame(f)):
                continue
            other = f2 if f1.IsSame(f) else f1
            if is_hole_face(other):
                continue
            out.append((e, f))
    return out


def _chamfer_foot(s, plane, foot, log):
    pe = _print_face_edges(s, plane)
    if not pe:
        return s
    for c in (foot, foot / 2):
        try:
            new = s.chamfer(c, None, [Edge(e) for e, _ in pe])
            if _ok(new, s):
                log.append(f"foot chamfer {c}: {len(pe)} edges")
                return as_solid(new)
        except Exception:
            pass
    done = 0
    for chain in _chains([e for e, _ in pe]):
        for c in (foot, foot / 2):
            try:
                new = s.chamfer(c, None, [Edge(e) for e in chain])
                if _ok(new, s):
                    s = as_solid(new)
                    done += len(chain)
                    break
            except Exception:
                continue
    log.append(f"foot chamfer by chain: {done}/{len(pe)} edges")
    return s


def finish(solid, outer_r=1.0, inner_r=0.5, print_face=None, foot=0.5, min_len=0.3, min_dihedral=20.0,
           log=None):
    """Fillet convex edges (outer_r), fillet concave edges (inner_r), chamfer the print face (foot).

    print_face: None, '-Z' (default for prints), '+Z', '-X', '+X', '-Y', '+Y' or a planar Face.
    Edges lying in the print plane are never filleted (a fillet there is an overhang on the bed).
    log: optional list that receives one line per operation. Never raises: on any failure the
    previous valid solid is kept and the edge is skipped.
    """
    log = log if log is not None else []
    try:
        s = as_solid(solid)
        if not isinstance(s, Solid):
            log.append("finish: input is not a single solid, returned unchanged")
            return solid
    except Exception as e:  # noqa: BLE001
        log.append(f"finish: cannot read input ({e})")
        return solid
    plane = None
    try:
        plane = _print_plane(s, print_face)
    except Exception as e:  # noqa: BLE001
        log.append(f"finish: bad print_face {print_face!r} ({e})")
    for r, convex in ((outer_r, True), (inner_r, False)):
        if r and r > 0:
            try:
                s = _fillet_pass(s, r, convex, plane, min_len, min_dihedral, log)
            except Exception as e:  # noqa: BLE001
                log.append(f"finish: fillet pass failed ({e})")
    if plane is not None and foot and foot > 0:
        try:
            s = _chamfer_foot(s, plane, foot, log)
        except Exception as e:  # noqa: BLE001
            log.append(f"finish: foot chamfer failed ({e})")
    if getattr(solid, "label", None):
        s.label = solid.label
    return s
