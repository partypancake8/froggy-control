"""Tessellate every component once and reuse the meshes for the PNG renders
and the STL / 3MF / GLB exports (the STEP export stays B-rep)."""
import os
import struct

import numpy as np

TESS = (0.08, 0.3)


def fast_tessellate(shape, tol=TESS):
    """(verts (n,3), faces (m,3)) of a B-rep: BRepMesh, then the triangulation
    is written by OCC's own binary STL writer and read back with numpy (the
    per-face Python loop in Shape.tessellate costs about 2 ms per face, 15 s on
    the Pi Zero STEP alone). Coincident vertices are merged."""
    import tempfile
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.StlAPI import StlAPI_Writer
    w = shape.wrapped
    BRepMesh_IncrementalMesh(w, tol[0], False, tol[1], True)
    fd, path = tempfile.mkstemp(suffix=".stl")
    os.close(fd)
    try:
        wr = StlAPI_Writer()
        wr.ASCIIMode = False
        if not wr.Write(w, path):
            return np.zeros((0, 3)), np.zeros((0, 3), np.int64)
        data = open(path, "rb").read()
    finally:
        os.unlink(path)
    n = struct.unpack("<I", data[80:84])[0]
    if n == 0:
        return np.zeros((0, 3)), np.zeros((0, 3), np.int64)
    rec = np.frombuffer(data, dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")], count=n, offset=84)
    pts = rec["v"].reshape(-1, 3).astype(float)
    uniq, inv = np.unique(np.round(pts, 5), axis=0, return_inverse=True)
    return uniq, inv.reshape(-1, 3).astype(np.int64)


def tessellate(A, tol=TESS):
    """{name: (verts (n,3) float, faces (m,3) int)} for every component."""
    out, rgb = {}, {}
    for n, p in A.parts.items():
        bodies = p.meta.get("bodies")
        if bodies and len(bodies) == len(p.solid.solids()):
            # multi-body part (meta['bodies'], one (label, material) per solid): tessellate solid by solid and keep
            # one RGB per triangle from each body's material, so the renders and the GLB show the body colours
            from lib.materials import MATERIALS
            vs, fs, cs, off = [], [], [], 0
            for (lbl, mat), s in zip(bodies, p.solid.solids()):
                v, f = fast_tessellate(s, tol)
                if not len(f):
                    continue
                vs.append(v)
                fs.append(f + off)
                cs.append(np.tile(np.array(MATERIALS[mat]["colour"][:3], float), (len(f), 1)))
                off += len(v)
            if not fs:
                continue
            out[n] = (np.concatenate(vs), np.concatenate(fs))
            rgb[n] = np.concatenate(cs)
            continue
        v, f = fast_tessellate(p.solid, tol)
        if not len(f):
            continue
        out[n] = (v, f)
    A.__dict__["_meshes"] = out
    A.__dict__["_face_rgb"] = rgb
    return out


def face_rgb(A):
    """{name: (m, 3) float RGB per triangle} for the multi-body parts (meta['bodies']) of the shared tessellation."""
    meshes(A)
    return A.__dict__.get("_face_rgb", {})


def meshes(A):
    m = A.__dict__.get("_meshes")
    return m if m is not None else tessellate(A)


def write_stl(path, M):
    """One binary STL of every component."""
    tris = np.concatenate([v[f] for v, f in M.values()]).astype(np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(1e-12, np.linalg.norm(n, axis=1))[:, None]
    rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tris
    with open(path, "wb") as fh:
        fh.write(b"froggy-cad".ljust(80, b" "))
        fh.write(struct.pack("<I", len(tris)))
        fh.write(rec.tobytes())


def _scene(A, M):
    """One node per component, named by its unique readable node name (lib.naming); sheet.json
    `node_name` maps it back to the internal key."""
    import trimesh
    from lib.naming import name_assembly
    rows = name_assembly(A)["parts"]
    sc = trimesh.Scene()
    FR = face_rgb(A)
    for n, (v, f) in M.items():
        p = A.parts[n]
        nn = rows[n]["node_name"]
        a = p.meta.get("opacity", 1.0)
        if n in FR and len(FR[n]) == len(f):          # body colours per triangle (multi-body vendor parts)
            fc = np.hstack([np.round(FR[n] * 255), np.full((len(f), 1), round(a * 255))]).astype(np.uint8)
            tm = trimesh.Trimesh(v, f, process=False, face_colors=fc)
            sc.add_geometry(tm, node_name=nn, geom_name=nn)
            continue
        rgba = [int(round(c * 255)) for c in p.colour[:3]] + [int(round(a * 255))]
        mat = trimesh.visual.material.PBRMaterial(name=nn, baseColorFactor=rgba, metallicFactor=0.0,
                                                  roughnessFactor=0.8, alphaMode="BLEND" if a < 1 else "OPAQUE")
        tm = trimesh.Trimesh(v, f, process=False, visual=trimesh.visual.TextureVisuals(material=mat))
        sc.add_geometry(tm, node_name=nn, geom_name=nn)
    return sc


def write_glb(path, A, M):
    with open(path, "wb") as fh:
        fh.write(_scene(A, M).export(file_type="glb"))


def write_3mf(path, A, M):
    """3MF through lib3mf (build123d's Mesher model), one named, coloured mesh
    object per component, from the shared tessellation."""
    from build123d import Mesher
    from lib.naming import name_assembly
    rows = name_assembly(A)["parts"]
    me = Mesher()
    group = me.model.AddBaseMaterialGroup()
    for n, (v, f) in M.items():
        p = A.parts[n]
        nn = rows[n]["node_name"]
        obj = me.model.AddMeshObject()
        verts, tris = Mesher._create_3mf_mesh([tuple(q) for q in v.tolist()], f.tolist())
        obj.SetGeometry(verts, tris)
        obj.SetName(nn)
        col = me.wrapper.FloatRGBAToColor(*p.colour[:3], p.meta.get("opacity", 1.0))
        obj.SetObjectLevelProperty(group.GetResourceID(), group.AddMaterial(Name=nn, DisplayColor=col))
        me.model.AddBuildItem(obj, me.wrapper.GetIdentityTransform())
    me.write(path)


def step_tree(A, name):
    """Nested, labelled Compound for the STEP, all labels readable (lib.naming):
    root -> module -> sub-assembly -> parts, plus one "Fasteners" folder per sub-assembly holding
    one group per joint (its screw and nut, washer or insert). Leaves are fresh compounds/solids
    that carry the display name and material colour; a part with meta['bodies'] becomes a node
    with one named child per solid."""
    from build123d import Color, Compound, Solid
    from lib.naming import FASTENERS, name_assembly
    N = name_assembly(A)
    rows = N["parts"]

    def col(rgb, a):
        return Color(*rgb[:3], a)

    def leaf(p):
        r = rows[p.name]
        a = p.meta.get("opacity", 1.0)
        bodies = p.meta.get("bodies")
        if bodies:
            from lib.materials import MATERIALS
            kids = []
            for (lbl, mat), s in zip(bodies, p.solid.solids()):
                rgba = MATERIALS[mat]["colour"]
                kids.append(Solid(s.wrapped, label=r["bodies"][lbl], color=col(rgba, rgba[3])))
            return Compound(label=r["display_name"], children=kids)
        sols = p.solid.solids()
        shape = Solid(sols[0].wrapped) if len(sols) == 1 else Compound([Solid(s.wrapped) for s in sols])
        shape.label, shape.color = r["display_name"], col(p.colour, a)
        return shape

    tree = {}           # module -> sub -> {"parts": [...], "joints": {joint display: [...]}}
    for p in A.parts.values():
        r = rows[p.name]
        m, sub = r["group"]
        d = tree.setdefault(m, {}).setdefault(sub, {"parts": [], "joints": {}})
        if r["fastener"]:
            d["joints"].setdefault(r["joint"], []).append(leaf(p))
        else:
            d["parts"].append(leaf(p))
    mods = []
    for m, subs in tree.items():
        kids = []
        for sub, d in subs.items():
            sk = list(d["parts"])
            if d["joints"]:
                sk.append(Compound(label=FASTENERS,
                                   children=[Compound(label=j, children=v) for j, v in d["joints"].items()]))
            kids.append(Compound(label=sub, children=sk))
        mods.append(Compound(label=m, children=kids))
    root = N["root"]
    if root in tree:
        root = f"{root} assembly"
    return Compound(label=root, children=mods)


def export_all(A, outdir, name):
    from build123d import export_step
    M = meshes(A)
    files = {k: os.path.join(outdir, f"{name}.{k}") for k in ("step", "stl", "3mf", "glb")}
    export_step(step_tree(A, name), files["step"])
    write_stl(files["stl"], M)
    write_3mf(files["3mf"], A, M)
    write_glb(files["glb"], A, M)
    return {k: dict(path=v, bytes=os.path.getsize(v)) for k, v in files.items()}
