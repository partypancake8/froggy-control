"""Read the label tree of an exported STEP back with OCP (XCAF) and check it is organised and readable.

read_tree(path) -> {"name", "children": [...], "assembly": bool} for the root (free shape).
check(tree, sheet) -> dict(ok, components, unnamed, misplaced, modules, ...): the rules of
`bin/cad tree` (see froggy_cad/cli.py cmd_tree).
"""
import re

from lib.naming import FASTENERS, label_problems


def read_tree(path):
    from OCP.collections import Sequence_TDF_Label
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    r = STEPCAFControl_Reader()
    r.SetNameMode(True)
    r.SetColorMode(False)
    r.SetLayerMode(False)
    if r.ReadFile(str(path)) != IFSelect_RetDone:
        raise RuntimeError(f"cannot read {path}")
    r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def name(lab):
        a = TDataStd_Name()
        return a.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), a) else ""

    def node(lab):
        ref = lab
        if st.IsReference_s(lab):
            ref = TDF_Label()
            st.GetReferredShape_s(lab, ref)
        kids = Sequence_TDF_Label()
        st.GetComponents_s(ref, kids)
        nm = name(ref) or name(lab)
        return dict(name=nm, assembly=bool(st.IsAssembly_s(ref)),
                    children=[node(kids.Value(i)) for i in range(1, kids.Length() + 1)])

    roots = Sequence_TDF_Label()
    st.GetFreeShapes(roots)
    tops = [node(roots.Value(i)) for i in range(1, roots.Length() + 1)]
    return tops[0] if len(tops) == 1 else dict(name="", assembly=True, children=tops)


def render(tree, out, depth=0, max_fastener_leaves=None):
    """Indented lines: modules, sub-assemblies, Fasteners, joint groups, parts (bodies under parts)."""
    tag = ""
    if tree["children"] and depth > 0:
        tag = f"  [{len(tree['children'])}]"
    out.append("  " * depth + tree["name"] + tag)
    for c in tree["children"]:
        render(c, out, depth + 1)
    return out


def check(tree, sheet):
    """Rules (froggy-cad naming layer):
    - root > module > sub-assembly; a sub-assembly holds parts and at most one "Fasteners" folder;
    - "Fasteners" holds only joint groups (assemblies whose children are screw/nut/washer/insert leaves);
    - a part is a leaf or a node of named body leaves; nothing else sits at any other depth;
    - every name is readable (lib.naming.label_problems: no v_/s_/lp_/fp9_/hw_/j_ prefix, no
      underscore, no internal key inside, no dashes);
    - the tree matches sheet.json: every component's tree_path is present (fasteners under Fasteners,
      parts not), with nothing extra, and no component was named by the fallback rule."""
    comps = sheet.get("components", [])
    keys = sorted({c["name"] for c in comps} | {j["name"] for j in sheet.get("joints", []) if j.get("name")} |
                  {c.get("joint_key") for c in comps if c.get("joint_key")} - {None}, key=len, reverse=True)
    keys = [k for k in keys if "_" in k]          # only snake keys can leak; short words are not keys
    unnamed, misplaced, seen_paths = [], [], []
    stats = dict(modules=0, subassemblies=0, fastener_folders=0, joint_groups=0, parts=0, bodies=0)

    def bad(name, where):
        p = label_problems(name, keys)
        if p:
            unnamed.append(dict(name=name, at=where, why=p))

    root = tree
    bad(root["name"], "root")
    for mod in root["children"]:
        stats["modules"] += 1
        mpath = [mod["name"]]
        bad(mod["name"], "module")
        if not mod["children"]:
            misplaced.append(dict(name=mod["name"], why="module is not an assembly"))
        for sub in mod["children"]:
            spath = mpath + [sub["name"]]
            bad(sub["name"], " > ".join(mpath))
            if not sub["assembly"] or not sub["children"]:
                misplaced.append(dict(name=" > ".join(spath), why="part directly under a module (no sub-assembly)"))
                continue
            if sub["name"] == FASTENERS:
                misplaced.append(dict(name=" > ".join(spath), why="Fasteners folder directly under a module"))
            stats["subassemblies"] += 1
            nf = 0
            for item in sub["children"]:
                ipath = spath + [item["name"]]
                bad(item["name"], " > ".join(spath))
                if item["name"] == FASTENERS:
                    nf += 1
                    stats["fastener_folders"] += 1
                    for grp in item["children"]:
                        gpath = ipath + [grp["name"]]
                        bad(grp["name"], " > ".join(ipath))
                        if not grp["children"]:
                            misplaced.append(dict(name=" > ".join(gpath), why="loose part in Fasteners (not in a joint group)"))
                            continue
                        stats["joint_groups"] += 1
                        if not re.match(r"^(M\d|Shoulder screw)", grp["name"]):
                            misplaced.append(dict(name=" > ".join(gpath), why="joint group name does not start with the screw spec"))
                        for hw in grp["children"]:
                            bad(hw["name"], " > ".join(gpath))
                            if hw["children"]:
                                misplaced.append(dict(name=" > ".join(gpath + [hw["name"]]), why="nested group inside a joint group"))
                            stats["parts"] += 1
                            seen_paths.append(tuple(gpath + [hw["name"]]))
                    continue
                if item["children"]:
                    for b in item["children"]:
                        bad(b["name"], " > ".join(ipath))
                        stats["bodies"] += 1
                        if b["children"]:
                            misplaced.append(dict(name=" > ".join(ipath + [b["name"]]), why="part nested deeper than its bodies"))
                stats["parts"] += 1
                seen_paths.append(tuple(ipath))
            if nf > 1:
                misplaced.append(dict(name=" > ".join(spath), why=f"{nf} Fasteners folders"))

    # agreement with sheet.json
    from collections import Counter
    want = Counter(tuple(c["tree_path"]) for c in comps if c.get("tree_path"))
    have = Counter(seen_paths)
    for c in comps:
        if not c.get("tree_path"):
            misplaced.append(dict(name=c["name"], why="sheet row has no tree_path (rebuild)"))
        if c.get("naming_rule") == "fallback":
            unnamed.append(dict(name=c.get("display_name"), at=" > ".join(c.get("group") or []),
                                why=["named by the fallback rule (add a rule in lib/naming.py)"]))
        fast = c.get("kind") in ("screw", "nut", "insert", "washer", "set_screw")
        tp = c.get("tree_path") or []
        if fast and FASTENERS not in tp:
            misplaced.append(dict(name=c.get("display_name"), why="fastener outside a Fasteners folder"))
        if not fast and FASTENERS in tp:
            misplaced.append(dict(name=c.get("display_name"), why="part inside a Fasteners folder"))
    for p, n in (want - have).items():
        misplaced.append(dict(name=" > ".join(p), why=f"in sheet.json x{n} but not in the STEP tree"))
    for p, n in (have - want).items():
        misplaced.append(dict(name=" > ".join(p), why=f"in the STEP tree x{n} but not in sheet.json"))
    return dict(ok=not unnamed and not misplaced, components=sum(have.values()), sheet_components=len(comps),
                unnamed=unnamed, misplaced=misplaced, **stats)


def module_summary(tree):
    """[(module, [(sub, parts, joint groups)])]."""
    out = []
    for mod in tree["children"]:
        subs = []
        for sub in mod["children"]:
            parts = sum(1 for i in sub["children"] if i["name"] != FASTENERS)
            jg = sum(len(i["children"]) for i in sub["children"] if i["name"] == FASTENERS)
            subs.append((sub["name"], parts, jg))
        out.append((mod["name"], subs))
    return out


def fusion_dry_run(tree, sheet):
    """What cad-fusion's appearance pass and readback would do with this STEP, without Fusion:
    lift the modules (drop the single root wrapper), walk every shape leaf as an occurrence chain,
    match it to sheet.json through lib.materials.fusion_table + lib.naming.match_entry (the same
    function the load script runs), and look every component up by display name the way the
    readback's parts_missing does."""
    from lib import materials as M
    from lib.naming import match_entry
    comps = sheet["components"]
    table = M.fusion_table(comps)
    tops = tree["children"] if tree.get("children") else [tree]
    leaves, unmapped, wrong = 0, [], []
    names = set()
    by_path = {" > ".join(c["tree_path"]): c for c in comps if c.get("tree_path")}

    def walk(n, chain):
        nonlocal leaves
        ch = chain + [n["name"]]
        names.add(n["name"])
        if n["children"]:
            for k in n["children"]:
                walk(k, ch)
            return
        leaves += 1
        e = match_entry(ch, table)
        if e is None:
            unmapped.append(" > ".join(ch))
            return
        c = by_path.get(" > ".join(ch)) or by_path.get(" > ".join(ch[:-1]))
        if c is not None:
            mats = {c["material"]} | set((c.get("sub_materials") or {}).values())
            if e["material"] not in mats:
                wrong.append(dict(path=" > ".join(ch), got=e["material"], want=sorted(mats)))

    for t in tops:
        walk(t, [])
    missing = [c["name"] for c in comps if (c.get("display_name") or c["name"]) not in names]
    return dict(ok=not unmapped and not missing and not wrong, leaves=leaves, unmapped=unmapped,
                parts_missing=missing, material_mismatch=wrong, top=[t["name"] for t in tops])
