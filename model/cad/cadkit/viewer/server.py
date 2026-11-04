"""`cadkit view`: the one standard local three.js CAD viewer for every project on this Mac.

Stdlib only (http.server on 127.0.0.1, foreground, Ctrl-C stops it). Page files are index.html, view.js and
style.css next to this module. Routes:
  /                    the page
  /models.json         the loaded files and their groups (each directory is one group)
  /models/<n>          bytes of file n (.glb, .stl, .ply); /model.glb is an alias for the first GLB
  /status.json         name, mtime, size, build_ok (null without a build.json), per-file mtimes; polled every 2 s
                       by the page, which reloads in place (camera, visibility and history kept) on any change
  /tree.json           module tree: from a sheet.json (tree_path per component) when one is given, else one node
                       per file or directory group with the GLB's own node names underneath
  /views.json          camera buttons: a views.json in the froggy format when given, else fitted iso, front,
                       right, back, top, bottom computed by the page. A view of kind "camera" (eye, axis, fisheye:
                       right, up, axis, sensor_px, f_px_per_rad, theta_max_rad, observer) is the lens image: a
                       cube map at the eye resampled per sensor pixel (equidistant); an optional overlay image
                       of any view is served as /views/<name>/overlay
  POST /log, /log.json page errors (echoed on stderr)
  POST /pose, GET /pose, GET /pose-info   pose of the movable group (--movable NAME, --save-to pose.json),
                       every save also appended to pose_history.jsonl next to the pose file
"""
import argparse
import datetime
import errno
import http.server
import json
import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE_FILES = {"index.html": "text/html; charset=utf-8", "view.js": "text/javascript; charset=utf-8",
              "style.css": "text/css; charset=utf-8"}
EXTS = (".glb", ".stl", ".ply")
CTYPES = {".glb": "model/gltf-binary", ".stl": "model/stl", ".ply": "application/octet-stream", ".png": "image/png",
          ".jpg": "image/jpeg", ".jpeg": "image/jpeg"}


def log(*a):
    print(*a, file=sys.stderr, flush=True)


def _read_json(p):
    try:
        with open(p) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def glb_complete(path, size):
    """True when the GLB header's total length equals the file size (a file being rewritten is short)."""
    try:
        with open(path, "rb") as fh:
            head = fh.read(12)
        if len(head) < 12:
            return False
        magic, _ver, length = struct.unpack("<4sII", head)
        return magic == b"glTF" and length == size
    except OSError:
        return False


def glb_mesh_nodes(path):
    """Names of the GLB nodes that carry a mesh, in file order."""
    try:
        with open(path, "rb") as fh:
            b = fh.read(20)
            cl = struct.unpack("<I", b[12:16])[0]
            j = json.loads(fh.read(cl))
        return [n.get("name") or f"node{i}" for i, n in enumerate(j.get("nodes", [])) if "mesh" in n]
    except (OSError, ValueError, struct.error):
        return []


def tree_from_sheet(sheet, name):
    """froggy-cad's module tree from sheet.json components: tree_path gives module > sub-assembly > part, leaves
    carry the GLB node name. None when the sheet has no components."""
    comps = (sheet or {}).get("components") or []
    if not comps:
        return None
    root = dict(name=name, children=[], _idx={})

    def child(node, nm):
        if nm not in node["_idx"]:
            c = dict(name=nm, children=[], _idx={})
            node["_idx"][nm] = c
            node["children"].append(c)
        return node["_idx"][nm]

    for c in comps:
        path = c.get("tree_path") or [c.get("module") or "Other", c.get("display_name") or c["name"]]
        node = root
        for seg in path[:-1]:
            node = child(node, seg)
        node["children"].append(dict(name=path[-1], node=c.get("node_name") or c["name"], component=c["name"],
                                     kind=c.get("kind"), module=c.get("module"), path=list(path)))

    def strip(n):
        n.pop("_idx", None)
        for k in n.get("children", []):
            strip(k)
        return n

    return dict(name=name, source="sheet.json", modules=strip(root)["children"])


class Sources:
    """The files a viewer shows. Each argument is a file (its own group) or a directory (one group of its
    .glb/.stl/.ply files, sorted). Directories are rescanned on every manifest or status request."""

    def __init__(self, paths, movable=None, second=None):
        self.args = []
        for p in paths:
            p = os.path.abspath(os.path.expanduser(p))
            if os.path.isdir(p):
                self.args.append(dict(kind="dir", path=p, name=os.path.basename(p.rstrip("/")) or p))
            elif os.path.isfile(p):
                if not p.lower().endswith(EXTS):
                    raise ValueError(f"not a .glb, .stl or .ply file: {p}")
                self.args.append(dict(kind="file", path=p, name=os.path.basename(p)))
            else:
                raise FileNotFoundError(f"no such file or directory: {p}")
        if second:
            p = os.path.abspath(os.path.expanduser(second))
            if not os.path.isdir(p):
                raise FileNotFoundError(f"--second needs a directory: {p}")
            self.args.append(dict(kind="dir", path=p, name=os.path.basename(p.rstrip("/")), second=True))
        names = set()
        for a in self.args:                    # group names are unique (they are tree keys)
            n, i = a["name"], 2
            while n in names:
                n, i = f"{a['name']} #{i}", i + 1
            a["name"] = n
            names.add(n)
        self.movable = movable
        if movable and not any(a["name"] == movable for a in self.args):
            raise ValueError(f"--movable {movable}: no group of that name (groups: {sorted(names)})")
        self.lock = threading.Lock()
        self._files = None
        self.scan()

    def scan(self):
        files, groups = [], []
        for a in self.args:
            if a["kind"] == "dir":
                fs = sorted(f for f in os.listdir(a["path"]) if f.lower().endswith(EXTS)
                            and os.path.isfile(os.path.join(a["path"], f)))
                paths = [os.path.join(a["path"], f) for f in fs]
            else:
                paths = [a["path"]]
            idx = []
            for p in paths:
                idx.append(len(files))
                files.append(dict(i=len(files), path=p, real=os.path.realpath(p), name=os.path.basename(p),
                                  ext=os.path.splitext(p)[1].lower(), group=a["name"]))
            groups.append(dict(name=a["name"], dir=a["kind"] == "dir", path=a["path"], files=idx,
                               movable=a["name"] == self.movable, second=bool(a.get("second"))))
        with self.lock:
            self._files, self._groups = files, groups
        return files, groups

    def files(self):
        return self.scan()[0]

    def first_glb(self):
        for f in self.files():
            if f["ext"] == ".glb":
                return f
        return None


class Viewer:
    def __init__(self, sources, name=None, title=None, tree=None, views=None, status_fn=None, save_to=None,
                 explode_origin="center", check=False):
        self.src = sources
        first = sources.args[0]
        self.name = name or os.path.splitext(first["name"])[0]
        self.title = title or self.name
        self.tree_arg, self.views_arg, self.status_fn = tree, views, status_fn
        self.explode_origin = explode_origin
        self.save_to = None
        if sources.movable:
            if not save_to:
                f0 = sources.files()[0]["path"] if sources.files() else first["path"]
                save_to = os.path.join(os.path.dirname(f0), "pose.json")
            self.save_to = os.path.abspath(os.path.expanduser(save_to))
        self.history = os.path.join(os.path.dirname(self.save_to), "pose_history.jsonl") if self.save_to else None
        self.logs = []
        self.lock = threading.Lock()
        self._tree = (None, None)

    # ---- manifest / status ----
    def manifest(self):
        files, groups = self.src.scan()
        fg = self.src.first_glb()
        # STL has no material data. Keep optional display colours beside the original meshes,
        # following symlinks used by head-view rather than baking project names into the viewer.
        palettes = {os.path.dirname(f["real"]) for f in files}
        palettes = {d: _read_json(os.path.join(d, "viewer-materials.json")) or {} for d in palettes}
        models = []
        for f in files:
            model = {k: f[k] for k in ("i", "name", "ext", "group")}
            model["url"] = f"/models/{f['i']}"
            palette = palettes[os.path.dirname(f["real"])]
            color = palette.get(os.path.basename(f["real"])) if isinstance(palette, dict) else None
            if (isinstance(color, str) and len(color) == 7 and color[0] == "#"
                    and all(c in "0123456789abcdefABCDEF" for c in color[1:])):
                model["color"] = color
            models.append(model)
        return dict(name=self.name, title=self.title, movable=self.src.movable, save_to=self.save_to,
                    explode_origin=self.explode_origin, first_glb=fg["i"] if fg else None,
                    has_tree=self.tree_arg is not None, has_views=self.views_arg is not None,
                    files=models,
                    groups=[{k: g[k] for k in ("name", "dir", "files", "movable", "second")} for g in groups])

    def status(self):
        files = self.src.files()
        per, mt, size, complete = [], None, 0, True
        for f in files:
            try:
                st = os.stat(f["path"])
                m, s = st.st_mtime, st.st_size
            except OSError:
                m, s = None, None
            ok = bool(s) and (f["ext"] != ".glb" or glb_complete(f["path"], s))
            complete = complete and ok
            per.append(dict(i=f["i"], name=f["name"], mtime=m, size=s, complete=ok))
            if m is not None:
                mt = m if mt is None else max(mt, m)
            size += s or 0
        fg = self.src.first_glb()
        st = dict(name=self.name, glb=(fg or (files[0] if files else {})).get("path"), mtime=mt, size=size,
                  complete=complete and bool(files), build_ok=None, files=per,
                  sig="|".join(f"{p['name']}:{p['mtime']}:{p['size']}" for p in per), tree_mtime=None)
        if fg:                                      # the froggy contract: mtime and size are the first GLB's
            g = per[fg["i"]]
            st.update(mtime=g["mtime"], size=g["size"])
        if self.status_fn is None and files:
            b = _read_json(os.path.join(os.path.dirname(files[0]["path"]), "build.json"))
            if isinstance(b, dict):
                ok = b.get("ok", b.get("valid"))
                st["build_ok"] = bool(ok) if ok is not None else None
        tp = self.tree_path()
        if tp and os.path.exists(tp):
            st["tree_mtime"] = os.path.getmtime(tp)
        if self.status_fn:
            st.update(self.status_fn() or {})
        return st

    # ---- tree ----
    def tree_path(self):
        return self.tree_arg if isinstance(self.tree_arg, str) else None

    def tree(self):
        if callable(self.tree_arg):
            t = self.tree_arg()
            if t:
                return t
        tp = self.tree_path()
        sig = (os.path.getmtime(tp) if tp and os.path.exists(tp) else None, self.status()["sig"])
        with self.lock:
            if self._tree[0] == sig:
                return self._tree[1]
        t = tree_from_sheet(_read_json(tp), self.name) if tp and sig[0] else None
        if t is None:
            t = self.generic_tree()
        with self.lock:
            self._tree = (sig, t)
        return t

    def generic_tree(self):
        files, groups = self.src.scan()

        def file_node(f, path):
            if f["ext"] == ".glb":
                kids = [dict(name=n, node=n, file=f["i"], component=n, kind=None, path=path + [f["name"], n])
                        for n in glb_mesh_nodes(f["path"])]
                return dict(name=f["name"], file=f["i"], children=kids)
            return dict(name=f["name"], node=f["name"], file=f["i"], component=f["name"], kind=f["ext"][1:],
                        path=path + [f["name"]])

        mods = []
        for g in groups:
            if g["dir"]:
                mods.append(dict(name=g["name"], group=g["name"],
                                 children=[file_node(files[i], [g["name"]]) for i in g["files"]]))
            elif g["files"]:
                n = file_node(files[g["files"][0]], [])
                n["group"] = g["name"]
                mods.append(n)
        return dict(name=self.name, source="files", modules=mods)

    # ---- views ----
    def views(self):
        v = self.views_arg
        if v is None:
            return dict(views=[], fitted=True)
        if callable(v):
            return v()
        if isinstance(v, dict):
            return v
        d = _read_json(v)
        if isinstance(d, list):
            d = dict(views=d)
        return d or dict(views=[], fitted=True)

    def views_json(self):
        """views() for the page: a view's overlay image file (`overlay.file`, a local path) is served as
        /views/<name>/overlay and only that URL goes out."""
        d = self.views()
        vs = []
        for v in d.get("views", []):
            ov = v.get("overlay")
            if isinstance(ov, dict) and ov.get("file"):
                v = dict(v, overlay={k: x for k, x in ov.items() if k != "file"})
                v["overlay"]["url"] = f"/views/{v['name']}/overlay"
            vs.append(v)
        return dict(d, views=vs)

    def overlay_file(self, name):
        for v in self.views().get("views", []):
            ov = v.get("overlay")
            if v.get("name") == name and isinstance(ov, dict) and ov.get("file"):
                return ov["file"]
        return None

    # ---- pose ----
    def save_pose(self, pose, path=None):
        path = path or self.save_to
        pose.setdefault("saved_at", datetime.datetime.now().astimezone().isoformat(timespec="seconds"))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(pose, f, indent=2)
            f.write("\n")
        os.replace(tmp, path)
        hist = os.path.join(os.path.dirname(path), "pose_history.jsonl")
        with open(hist, "a") as f:
            f.write(json.dumps(pose) + "\n")
        return path, hist


def make_handler(V):
    class H(http.server.BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)

        def _json(self, code, obj):
            self._send(code, json.dumps(obj).encode(), "application/json")

        def _file(self, f):
            try:
                with open(f["path"], "rb") as fh:
                    body = fh.read()
            except OSError as e:
                return self._json(404, dict(error=str(e)))
            return self._send(200, body, CTYPES.get(f["ext"], "application/octet-stream"))

        def do_HEAD(self):
            self.do_GET()

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                path = "/index.html"
            if path == "/favicon.ico":
                return self._send(204, b"", "image/x-icon")
            if path == "/status.json":
                return self._json(200, V.status())
            if path == "/models.json":
                return self._json(200, V.manifest())
            if path == "/tree.json":
                return self._json(200, V.tree())
            if path == "/views.json":
                return self._json(200, V.views_json())
            if path.startswith("/views/") and path.endswith("/overlay"):
                fp = V.overlay_file(path[len("/views/"):-len("/overlay")])
                ext = os.path.splitext(fp or "")[1].lower()
                if not fp or ext not in (".png", ".jpg", ".jpeg"):
                    return self._json(404, dict(error=f"no {path}"))
                return self._file(dict(path=fp, ext=ext))
            if path == "/log.json":
                return self._json(200, dict(entries=V.logs[-200:]))
            if path == "/pose-info":
                return self._json(200, dict(save_to=V.save_to, history=V.history))
            if path == "/pose":
                d = _read_json(V.save_to) if V.save_to else None
                return self._json(200, d if isinstance(d, dict) else {})
            if path == "/model.glb":
                f = V.src.first_glb()
                return self._file(f) if f else self._json(404, dict(error="no GLB loaded"))
            if path.startswith("/models/"):
                try:
                    n = int(path.split("/")[2])
                    f = V.src.files()[n]
                except (ValueError, IndexError):
                    return self._json(404, dict(error=f"no {path}"))
                return self._file(f)
            fn = path.lstrip("/")
            if fn in PAGE_FILES:
                with open(os.path.join(HERE, fn), "rb") as fh:
                    return self._send(200, fh.read(), PAGE_FILES[fn])
            return self._json(404, dict(error=f"no {path}"))

        def _body(self):
            n = int(self.headers.get("Content-Length", 0))
            return json.loads(self.rfile.read(n) or b"{}")

        def do_POST(self):
            path = self.path.split("?")[0]
            if path == "/log":
                try:
                    e = self._body()
                except ValueError:
                    e = dict(level="error", msg="unparseable /log body")
                e["t"] = time.strftime("%H:%M:%S")
                V.logs.append(e)
                del V.logs[:-500]
                if e.get("level") in ("error", "warn"):
                    log(f"page {e.get('level')}: {e.get('msg')}")
                return self._json(200, dict(ok=True))
            if path == "/pose":
                if not V.save_to:
                    return self._json(404, dict(error="no movable group (--movable) in this viewer"))
                try:
                    pose = self._body()
                    if not isinstance(pose, dict):
                        raise ValueError("pose must be a JSON object")
                    saved, hist = V.save_pose(pose)
                except Exception as e:  # noqa: BLE001
                    return self._json(400, dict(error=str(e)))
                log(f"saved pose -> {saved}")
                return self._json(200, dict(saved=saved, history=hist))
            return self._json(404, dict(error="only POST /log and /pose"))

    return H


def free_port(start):
    p = start
    while True:
        s = socket.socket()
        try:
            s.bind(("127.0.0.1", p))
            return p
        except OSError:
            p += 1
        finally:
            s.close()


def _bind(V, port):
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), make_handler(V))
    srv.daemon_threads = True
    srv.block_on_close = False             # never wait on a browser's idle keep-alive connection at exit
    return srv


def serve(sources, port=7891, open_browser=True, tree=None, views=None, status_fn=None, save_to=None, *,
          name=None, title=None, movable=None, second=None, explode_origin="center"):
    """Foreground viewer. `sources`: paths (files or directories) or a Sources. `tree`: a sheet.json path or a
    callable returning a tree dict. `views`: a views.json path, a dict or a callable. `status_fn()`: extra
    status.json fields. Returns (exit code, summary dict)."""
    try:
        src = sources if isinstance(sources, Sources) else Sources(sources, movable=movable, second=second)
    except (OSError, ValueError) as e:
        log(str(e))
        return 1, dict(ok=False, error=str(e))
    V = Viewer(src, name=name, title=title, tree=tree, views=views, status_fn=status_fn, save_to=save_to,
               explode_origin=explode_origin)
    try:
        srv = _bind(V, port)
    except OSError as e:
        if e.errno == errno.EADDRINUSE:
            log(f"port {port} on 127.0.0.1 is already in use (another viewer?); stop it or pass --port N")
            return 2, dict(ok=False, error=f"port {port} in use")
        raise

    def _term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    signal.signal(signal.SIGINT, _term)    # also when started from a script, where SIGINT is inherited ignored
    url = f"http://127.0.0.1:{port}/"
    log(f"viewer {url} serving {len(src.files())} file(s): {', '.join(a['path'] for a in src.args)} (Ctrl-C stops)")
    if V.save_to:
        log(f"pose of group '{src.movable}' saves to: {V.save_to}")
    if open_browser:
        subprocess.Popen(["open", url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        log("viewer stopped")
    finally:
        srv.server_close()
    fg = src.first_glb()
    return 0, dict(ok=True, url=url, glb=fg["path"] if fg else None, files=[f["path"] for f in src.files()])


# ---- --check ----
def _sample(dirpath):
    """A tiny head-mode sample: head.ply (colored tetrahedron) and mask/ with two STL pieces."""
    import random
    os.makedirs(os.path.join(dirpath, "mask"), exist_ok=True)
    v = [(0, 0, 0), (60, 0, 0), (30, 50, 0), (30, 20, 70)]
    f = [(0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3)]
    with open(os.path.join(dirpath, "head.ply"), "w") as fh:
        fh.write("ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\n"
                 "property uchar red\nproperty uchar green\nproperty uchar blue\nelement face 4\n"
                 "property list uchar int vertex_indices\nend_header\n")
        for p in v:
            fh.write("%g %g %g %d %d %d\n" % (*p, random.randint(120, 230), 150, 120))
        for t in f:
            fh.write("3 %d %d %d\n" % t)
    for k, dz in (("a_piece.stl", 10), ("b_piece.stl", 40)):
        with open(os.path.join(dirpath, "mask", k), "wb") as fh:
            fh.write(b"\0" * 80 + struct.pack("<I", 4))
            for t in f:
                fh.write(struct.pack("<3f", 0, 0, 0))
                for i in t:
                    x, y, z = v[i]
                    fh.write(struct.pack("<3f", x * 0.8 - 5, y * 0.8 - 5, z * 0.5 + dz))
                fh.write(b"\0\0")
    return [os.path.join(dirpath, "head.ply"), os.path.join(dirpath, "mask")]


def check(paths, port=None, tree=None, views=None, movable=None, second=None, save_to=None, name=None):
    """Start the server in a thread, fetch /, /status.json, /models.json and every model, POST a sample pose
    (to a scratch path, never the real save path) when a group is movable. Prints one line per probe."""
    base = os.path.join(tempfile.gettempdir(), "cadkit-view")
    try:
        os.makedirs(base, exist_ok=True)
    except OSError:
        base = None
    tmp = tempfile.mkdtemp(prefix="check-", dir=base)
    if not paths:
        paths, movable = _sample(tmp), "mask"
    src = Sources(paths, movable=movable, second=second)
    V = Viewer(src, name=name, tree=tree, views=views, save_to=os.path.join(tmp, "pose.json") if movable else None)
    port = port or free_port(7895)
    srv = _bind(V, port)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base, rc = f"http://127.0.0.1:{port}", 0

    def get(p):
        with urllib.request.urlopen(base + p, timeout=30) as r:
            return r.status, r.read()

    try:
        c, body = get("/")
        has_undo = b'id="undo"' in body
        ok = c == 200 and has_undo
        print(f"/ {c} ({len(body)} bytes, undo button {'yes' if has_undo else 'NO'})")
        rc |= not ok
        c, body = get("/status.json")
        s = json.loads(body)
        print(f"/status.json {c} name={s['name']} mtime={s['mtime']} build_ok={s['build_ok']}")
        rc |= c != 200 or "mtime" not in s
        c, body = get("/models.json")
        m = json.loads(body)
        print(f"/models.json {c} {len(m['files'])} files in {len(m['groups'])} groups, movable={m['movable']}")
        for f in src.files():
            c, body = get(f"/models/{f['i']}")
            want = os.stat(f["path"]).st_size
            print(f"/models/{f['i']} {f['name']} {c} {len(body)} bytes (expected {want})")
            rc |= c != 200 or len(body) != want
        for p in ("/tree.json", "/views.json"):
            c, body = get(p)
            print(f"{p} {c}")
            rc |= c != 200
        if movable:
            pieces = [src.files()[i]["name"] for g in m["groups"] if g["movable"] for i in g["files"]]
            mm = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, -3, -12, 1]
            sample = dict(overlay=movable, pieces=pieces, matrix=mm,
                          matrix_rows=[[mm[c * 4 + r] for c in range(4)] for r in range(4)],
                          position_mm=[0, -3, -12], rotation_deg=[0, 0, 0], scale=[1, 1, 1],
                          frame="mm, Z up", check_sample=True)
            req = urllib.request.Request(base + "/pose", data=json.dumps(sample).encode(), method="POST",
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=10) as r:
                print(f"POST /pose {r.status} -> {json.loads(r.read())['saved']}")
            c, body = get("/pose")
            p = json.loads(body)
            ok = len(p.get("matrix", [])) == 16 and p.get("pieces") == pieces
            print(f"GET /pose {c} matrix {len(p.get('matrix', []))}, {len(p.get('pieces', []))} pieces")
            rc |= not ok
            if save_to:
                print(f"real save path: {os.path.abspath(save_to)}")
    except Exception as e:  # noqa: BLE001
        print(f"FAIL {e}")
        rc = 1
    finally:
        srv.shutdown()
        srv.server_close()
        shutil.rmtree(tmp, ignore_errors=True)
    print("ok" if rc == 0 else "FAIL")
    return int(rc)


def add_args(p):
    p.add_argument("paths", nargs="*", help=".glb, .stl, .ply files or directories (one group each)")
    p.add_argument("--port", type=int, help="port on 127.0.0.1 (default 7891; --check picks a free one)")
    p.add_argument("--no-open", action="store_true", help="do not open a browser tab")
    p.add_argument("--tree", help="sheet.json with components[].tree_path for the module tree")
    p.add_argument("--views", help="views.json with camera buttons (froggy /views.json format)")
    p.add_argument("--movable", metavar="NAME", help="group (directory basename) that gets the move gizmo and pose")
    p.add_argument("--save-to", metavar="POSE_JSON", help="pose file for --movable (default next to the first file)")
    p.add_argument("--second", metavar="DIR", help="extra hidden-by-default group that moves with --movable")
    p.add_argument("--name", help="name shown in the title and status.json")
    p.add_argument("--check", action="store_true", help="start, fetch every route, exit 0 when all answer")
    return p


def run(a):
    if a.check:
        return check(a.paths, port=a.port, tree=a.tree, views=a.views, movable=a.movable, second=a.second,
                     save_to=a.save_to, name=a.name)
    if not a.paths:
        log("cadkit view: give at least one .glb, .stl, .ply file or directory")
        return 2
    code, res = serve(a.paths, port=a.port or 7891, open_browser=not a.no_open, tree=a.tree, views=a.views,
                      save_to=a.save_to, name=a.name, movable=a.movable, second=a.second)
    print(json.dumps(dict(command="view", port=a.port or 7891, **res)))
    return code


def main(argv=None):
    return run(add_args(argparse.ArgumentParser(prog="cadkit view", description=__doc__.splitlines()[0])).parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
