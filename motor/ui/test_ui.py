"""Headless browser tests for the Froggy motor page (ui/index.html).

Starts the real dry-run API server (froggy_motor.py --dry-run ui) on a free port and a tiny static server that
serves this folder and forwards /api/* to it, so the page talks to the real routes exactly as it will on the Pi.

    cd motor && python3 -m pytest -q ui/test_ui.py
"""
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import pytest

sync_playwright = pytest.importorskip("playwright.sync_api").sync_playwright

UI = os.path.dirname(os.path.abspath(__file__))
MOTOR = os.path.dirname(UI)
# screenshots and the server log go outside the checkout so a test run leaves git clean
SHOTS = os.environ.get("FROGGY_UI_SHOTS") or os.path.join(tempfile.gettempdir(), "froggy-ui-shots")
API_PORT = 7998      # dry-run API server for these tests (7999 belongs to the orchestrator)
MOVE_CALLS = ("/api/arm", "/api/speed", "/api/ramp", "/api/cltest", "/api/enable")


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def get_json(url, body=None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"} if data else {})
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return json.loads(e.read())


PUMP = []     # open playwright pages: sleeping through one lets its request events dispatch (sync API)


def wait_for(fn, timeout=15.0, step=0.1, what="condition"):
    t_end = time.time() + timeout
    last = None
    while time.time() < t_end:
        last = fn()
        if last:
            return last
        if PUMP:
            PUMP[-1].wait_for_timeout(step * 1000)
        else:
            time.sleep(step)
    raise AssertionError("timed out waiting for %s (last=%r)" % (what, last))


@pytest.fixture(scope="module")
def api():
    port = API_PORT
    os.makedirs(SHOTS, exist_ok=True)
    log = open(os.path.join(SHOTS, "api-server.log"), "w")
    trials_dir = tempfile.mkdtemp(prefix="froggy-ui-trials-")     # keep test trials out of motor/trials/
    env = dict(os.environ, FROGGY_TRIALS_DIR=trials_dir)
    proc = subprocess.Popen([sys.executable, "froggy_motor.py", "--dry-run", "ui", "--no-open", "--port", str(port)],
                            cwd=MOTOR, stdout=log, stderr=subprocess.STDOUT, env=env)
    base = "http://127.0.0.1:%d" % port
    try:
        wait_for(lambda: _alive(base), timeout=20, what="dry-run API server")
        yield base
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.close()
        import shutil
        shutil.rmtree(trials_dir, ignore_errors=True)


def _alive(base):
    try:
        return get_json(base + "/api/status")
    except Exception:
        return None


@pytest.fixture(scope="module")
def site(api):
    """Static server for ui/ that forwards /api/* to the dry-run server (same origin for the page)."""

    class H(SimpleHTTPRequestHandler):
        def __init__(self, *a, **k):
            super().__init__(*a, directory=UI, **k)

        def log_message(self, *a):
            pass

        def _proxy(self, method):
            n = int(self.headers.get("Content-Length") or 0)
            body = self.rfile.read(n) if n else None
            hdrs = {}
            if self.headers.get("Content-Type"):
                hdrs["Content-Type"] = self.headers["Content-Type"]
            req = urllib.request.Request(api + self.path, data=body, method=method, headers=hdrs)
            try:
                with urllib.request.urlopen(req, timeout=5) as r:
                    code, data, rh = r.status, r.read(), r.headers
            except urllib.error.HTTPError as e:
                code, data, rh = e.code, e.read(), e.headers
            self.send_response(code)
            self.send_header("Content-Type", rh.get("Content-Type") or "application/json")
            if rh.get("Content-Disposition"):
                self.send_header("Content-Disposition", rh["Content-Disposition"])
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path.startswith("/api/"):
                return self._proxy("GET")
            return super().do_GET()

        def do_POST(self):
            if self.path.startswith("/api/"):
                return self._proxy("POST")
            self.send_error(404)

    srv = ThreadingHTTPServer(("127.0.0.1", free_port()), H)
    srv.daemon_threads = True
    srv.handle_error = lambda *a: None     # a page closing mid-poll resets its socket: not a test failure
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    try:
        yield "http://127.0.0.1:%d" % srv.server_address[1]
    finally:
        srv.shutdown()
        srv.server_close()


@pytest.fixture(scope="module")
def browser():
    with sync_playwright() as p:
        b = p.chromium.launch(headless=True)
        yield b
        b.close()


class Page:
    def __init__(self, browser, site, w=1440, h=900):
        self.ctx = browser.new_context(viewport={"width": w, "height": h}, device_scale_factor=1)
        self.page = self.ctx.new_page()
        self.posts = []
        self.errors = []
        self.page.on("request", self._req)
        self.page.on("pageerror", lambda e: self.errors.append(str(e)))
        PUMP.append(self.page)
        self.page.goto(site + "/index.html")
        self.page.wait_for_function("document.getElementById('hdrdriver').textContent.indexOf('...') < 0")

    def _req(self, r):
        if r.method == "POST":
            path = "/" + r.url.split("/", 3)[3].split("?")[0]
            self.posts.append((path, json.loads(r.post_data or "{}")))

    def text(self, sel):
        return self.page.locator(sel).inner_text().strip()

    def close(self):
        if self.page in PUMP:
            PUMP.remove(self.page)
        self.ctx.close()
        assert not self.errors, self.errors


@pytest.fixture
def pg(browser, site, api):
    get_json(api + "/api/stop", {})
    wait_for(lambda: get_json(api + "/api/status")["rpm_cmd"] == 0, what="motor at rest before test")
    p = Page(browser, site)
    yield p
    p.close()
    get_json(api + "/api/stop", {})


def test_no_motion_on_load_or_reload(pg, api):
    pg.page.wait_for_timeout(1200)
    pg.page.reload()
    pg.page.wait_for_function("document.getElementById('hdrdriver').textContent.indexOf('...') < 0")
    pg.page.wait_for_timeout(1200)
    assert [p for p, _ in pg.posts] == [], pg.posts
    st = get_json(api + "/api/status")
    assert st["phase"] in ("idle", "estop") and st["rpm_cmd"] == 0
    assert pg.text("#start") == "START"


def test_single_control_area(pg):
    for sel in ("#start", "#stop", "#estop", "input[type=range]", "#rpm", "#modeseg", "#dirseg", "#accel"):
        assert pg.page.locator(sel).count() == 1, sel
    assert pg.page.locator("section.control").count() == 1
    assert pg.page.locator("details").count() == 0
    html = pg.page.content()
    assert "—" not in html and "–" not in html


def test_start_ramps_to_rpm_apply_then_stop(pg, api):
    page = pg.page
    page.fill("#rpm", "250")
    page.click("#dirseg button[data-dir=ccw]")
    page.fill("#accel", "1")
    page.click("#start")
    assert page.locator("#confirm").is_visible()
    assert pg.posts == [], "first START must only open the confirmation"
    page.click("#confirmyes")
    wait_for(lambda: len(pg.posts) >= 2, what="arm + speed posts")
    (p1, b1), (p2, b2) = pg.posts[:2]
    assert p1 == "/api/arm"
    assert p2 == "/api/speed"
    assert b2["rpm"] == 250 and b2["dir"] == "ccw" and b2["accel_s"] == 1 and isinstance(b2.get("token"), str)

    wait_for(lambda: (lambda s: s["phase"] == "hold" and abs(s["rpm_cmd"] - 250) < 0.5)(get_json(api + "/api/status")),
             what="server holding 250 rpm")
    page.wait_for_function("document.getElementById('r_state').textContent === 'holding'")
    assert pg.text("#r_cmd") == "250"
    # the encoder readout comes from rpm_encoder and settles onto the commanded speed
    page.wait_for_function("Math.abs(parseFloat(document.getElementById('r_enc').textContent) - 250) < 5", timeout=5000)
    assert pg.text("#start") == "APPLY"
    assert page.locator("#dirseg button[data-dir=cw]").is_disabled()

    # APPLY: new speed without stopping, no second confirmation
    pg.posts.clear()
    page.fill("#rpm", "150")
    page.click("#start")
    assert not page.locator("#confirm").is_visible()
    wait_for(lambda: len(pg.posts) >= 2, what="apply posts")
    assert [p for p, _ in pg.posts[:2]] == ["/api/arm", "/api/speed"] and pg.posts[1][1]["rpm"] == 150
    wait_for(lambda: abs(get_json(api + "/api/status")["rpm_cmd"] - 150) < 0.5, what="server at 150 rpm")

    pg.posts.clear()
    page.click("#stop")
    wait_for(lambda: pg.posts, what="stop post")
    assert pg.posts[0][0] == "/api/stop"
    wait_for(lambda: (lambda s: s["rpm_cmd"] == 0 and s["phase"] == "idle")(get_json(api + "/api/status")),
             what="server ramped to zero")
    page.wait_for_function("document.getElementById('r_state').textContent === 'off'")
    assert pg.text("#start") == "START"
    assert float(pg.text("#r_elapsed")) > 1.0


def test_emergency_stop(pg, api):
    page = pg.page
    page.fill("#rpm", "200")
    page.fill("#accel", "1")
    page.click("#start")
    page.click("#confirmyes")
    wait_for(lambda: get_json(api + "/api/status")["rpm_cmd"] > 20, what="motor moving")
    pg.posts.clear()
    pg.page.wait_for_timeout(300)
    pg.posts.clear()
    page.click("#estop")
    wait_for(lambda: pg.posts, what="estop post")
    assert pg.posts[0][0] == "/api/estop"
    wait_for(lambda: get_json(api + "/api/status")["rpm_cmd"] == 0, what="motor stopped by e-stop")
    page.wait_for_function("document.getElementById('r_state').textContent === 'e-stop'")


def test_timed_and_closed_loop_modes(pg, api):
    page = pg.page
    assert not page.locator("#hold").is_visible()
    page.click("#modeseg button[data-mode=cltest]")
    assert page.locator("#hold").is_visible() and page.locator("#decel").is_visible()
    page.fill("#rpm", "120")
    page.fill("#accel", "0.5")
    page.fill("#hold", "1")
    page.fill("#decel", "0.5")
    page.click("#start")
    page.click("#confirmyes")
    wait_for(lambda: len(pg.posts) >= 2, what="cltest posts")
    path, body = pg.posts[1]
    assert path == "/api/cltest" and body["rpm"] == 120 and body["hold_s"] == 1 and body["decel_s"] == 0.5
    page.wait_for_function("document.getElementById('verdict').textContent.indexOf('Closed-loop test verdict') === 0",
                           timeout=20000)
    st = get_json(api + "/api/status")
    assert st["cl_verdict"] and st["cl_verdict"] in pg.text("#verdict")

    page.click("#modeseg button[data-mode=ramp]")
    pg.posts.clear()
    page.fill("#hold", "0.5")
    page.click("#start")       # already confirmed on this page load
    wait_for(lambda: len(pg.posts) >= 2, what="ramp posts")
    assert pg.posts[1][0] == "/api/ramp" and pg.posts[1][1]["hold_s"] == 0.5
    wait_for(lambda: (lambda s: s["phase"] == "idle" and s["rpm_cmd"] == 0)(get_json(api + "/api/status")),
             timeout=15, what="timed test finished")


def test_readouts_bind_encoder_fields(browser, site, api):
    base = get_json(api + "/api/status")
    fake = dict(base, phase="hold", rpm_cmd=300.0, target_rpm=300.0, enabled=True, rpm_encoder=287.6,
                follow_err_rev=1.25, follow_err_pct=4.2, enc_no_pulses=True, estop=False,
                alarm_latched=False, cl_verdict="PASS: tracked within 0.4 %", t_s=base["t_s"])
    p = Page.__new__(Page)
    p.ctx = browser.new_context(viewport={"width": 1440, "height": 900})
    p.page = p.ctx.new_page()
    p.posts, p.errors = [], []
    p.page.on("request", p._req)
    p.page.on("pageerror", lambda e: p.errors.append(str(e)))
    PUMP.append(p.page)
    p.page.route("**/api/status", lambda r: r.fulfill(status=200, content_type="application/json", body=json.dumps(fake)))
    p.page.goto(site + "/index.html")
    p.page.wait_for_function("document.getElementById('r_enc').textContent === '288'")
    assert p.text("#r_cmd") == "300"
    assert p.text("#r_ferr") == "1.25"
    assert "4.2 %" in p.text("#r_ferrpct")
    assert p.text("#r_state") == "holding"
    assert p.text("#r_drv") == "enabled"
    assert p.text("#r_estop") == "OK"
    assert p.page.locator("#nopulses").is_visible()
    assert "PASS: tracked within 0.4 %" in p.text("#verdict")
    assert p.text("#start") == "APPLY"
    p.page.click("#diagbtn")
    assert '"rpm_encoder": 287.6' in p.text("#rawjson")
    assert p.posts == []
    p.close()


def test_phone_width_and_screenshots(browser, site, api):
    get_json(api + "/api/stop", {})
    wait_for(lambda: get_json(api + "/api/status")["rpm_cmd"] == 0, what="rest")
    os.makedirs(SHOTS, exist_ok=True)
    desk = Page(browser, site, 1440, 900)
    desk.page.fill("#rpm", "300")
    desk.page.fill("#accel", "2")
    desk.page.click("#start")
    desk.page.click("#confirmyes")
    desk.page.wait_for_function("document.getElementById('r_state').textContent === 'holding'", timeout=15000)
    desk.page.wait_for_timeout(2000)
    desk.page.screenshot(path=os.path.join(SHOTS, "ui-1440x900.png"))
    desk.close()        # one renderer at a time (the Mac's ccmemguard caps headless Chrome processes); motor keeps holding

    phone = Page(browser, site, 390, 844)
    phone.page.wait_for_function("document.getElementById('r_state').textContent === 'holding'")
    phone.page.wait_for_timeout(1000)
    over = phone.page.evaluate("document.documentElement.scrollWidth - document.documentElement.clientWidth")
    assert over <= 0, "horizontal scroll at 390 px: %d px" % over
    wide = phone.page.evaluate("""[...document.querySelectorAll('body *')].filter(e => {
        const r = e.getBoundingClientRect(); return r.width > 0 && (r.right > 390.5 || r.left < -0.5);
    }).map(e => e.tagName + '#' + e.id + '.' + e.className).slice(0, 5)""")
    assert wide == [], wide
    for sel in ("#start", "#stop", "#estop", "#confirmyes", "#modeseg button", "#accel"):
        loc = phone.page.locator(sel).first
        if loc.is_visible():
            assert loc.bounding_box()["height"] >= 56, sel
    phone.page.screenshot(path=os.path.join(SHOTS, "ui-390x844.png"))
    phone.page.screenshot(path=os.path.join(SHOTS, "ui-390-fullpage.png"), full_page=True)
    phone.page.click("#stop")
    wait_for(lambda: get_json(api + "/api/status")["rpm_cmd"] == 0, what="stopped after screenshots")
    phone.close()
    for f in ("ui-1440x900.png", "ui-390x844.png"):
        assert os.path.getsize(os.path.join(SHOTS, f)) > 10000


def test_connection_field_reports_link_loss(browser, site, api):
    """Title-bar link field: CONNECTED with the host, then DISCONNECTED in red after 5 s of failed polls; START locks."""
    p = Page(browser, site, 1440, 900)
    page = p.page
    page.wait_for_function("document.getElementById('connfield').dataset.link === 'ok'")
    assert p.text("#conntxt").startswith("CONNECTED ")
    assert not page.locator("#cmdlink").is_visible()
    page.route("**/api/status", lambda r: r.abort())
    page.wait_for_function("document.getElementById('connfield').dataset.link === 'retry'", timeout=4000)
    assert p.text("#conntxt").startswith("RECONNECTING, last status ")
    page.wait_for_function("document.getElementById('connfield').dataset.link === 'down'", timeout=8000)
    assert p.text("#conntxt").startswith("DISCONNECTED from ")
    assert page.locator("#start").is_disabled()
    assert "DISCONNECTED" in p.text("#cmdlink")
    page.unroute("**/api/status")
    page.wait_for_function("document.getElementById('connfield').dataset.link === 'ok'", timeout=5000)
    assert not page.locator("#start").is_disabled()
    assert [x for x, _ in p.posts] == []
    p.close()


def test_trials_list_csv_and_pdf_export(browser, site, api):
    """A finished timed test shows in the Trials table; Export CSV downloads the server CSV, Export PDF builds a PDF."""
    get_json(api + "/api/stop", {})
    wait_for(lambda: get_json(api + "/api/status")["rpm_cmd"] == 0, what="rest")
    p = Page(browser, site, 1920, 1080)
    page = p.page
    page.evaluate("localStorage.setItem('froggy-theme','dark')")
    r = get_json(api + "/api/ramp", {"rpm": 180, "accel_s": 0.5, "hold_s": 1.0, "decel_s": 0.5, "dir": "cw"})
    assert r["ok"], r
    page.wait_for_function("document.querySelector('#tbody tr.rec') !== null", timeout=5000)   # open trial on top
    assert "recording" in page.locator("#tbody tr.rec").inner_text()
    wait_for(lambda: (lambda s: s["phase"] == "idle" and s["rpm_cmd"] == 0 and s["trial_id"] is None
                      and s["last_trial_id"])(get_json(api + "/api/status")), timeout=15, what="timed test finished")
    tid = get_json(api + "/api/status")["last_trial_id"]
    listed = get_json(api + "/api/trials")
    assert listed and listed[0]["id"] == tid
    row = page.locator("#tbody tr[data-id='%s']" % tid)
    row.wait_for(timeout=5000)
    page.wait_for_function("!document.querySelector('#tbody tr.rec')", timeout=5000)
    cells = [c.strip() for c in row.inner_text().split("\t")]
    assert cells[1] == "ramp" and cells[2].startswith("180") and cells[4] == "complete", cells
    assert page.locator("#t_csv").is_disabled() and page.locator("#t_pdf").is_disabled()
    row.click()
    assert row.get_attribute("aria-selected") == "true"
    assert not page.locator("#t_csv").is_disabled()

    with page.expect_download() as dl:
        page.click("#t_csv")
    d = dl.value
    assert d.suggested_filename == "froggy-trial-%s.csv" % tid
    text = open(d.path()).read()
    lines = text.splitlines()
    assert lines[0].startswith("# froggy trial %s" % tid)
    assert all(l.startswith("#") for l in lines[:5])
    header = next(l for l in lines if not l.startswith("#"))
    assert header.startswith("t_s,rpm_cmd,rpm_target,rpm_encoder"), header

    page.click("#t_show")
    page.wait_for_function("!document.getElementById('t_live').hidden")
    assert tid in p.text("#trendttl")
    page.wait_for_timeout(400)
    page.screenshot(path=os.path.join(SHOTS, "hmi3-show-1920x1080.png"))
    page.click("#t_live")
    page.wait_for_function("document.getElementById('t_live').hidden")

    with page.expect_download() as dl2:
        page.click("#t_pdf")
    assert dl2.value.suggested_filename == "froggy-trial-%s.pdf" % tid
    head = page.evaluate("window.__lastPdf && String.fromCharCode.apply(null, Array.from(window.__lastPdf.slice(0, 5)))")
    assert head == "%PDF-", head
    size = page.evaluate("window.__lastPdf.length")
    assert size > 20000, size                    # two chart images inside
    with open(dl2.value.path(), "rb") as f:
        assert f.read(5) == b"%PDF-"
    dl2.value.save_as(os.path.join(SHOTS, "hmi3-trial-report.pdf"))     # kept for a look at the layout
    page.wait_for_function("document.getElementById('tmsg').textContent.indexOf('PDF saved') === 0")
    page.screenshot(path=os.path.join(SHOTS, "hmi3-pdf-1920x1080.png"))
    page.click("#tbody tr[data-id='%s']" % tid)
    page.wait_for_timeout(300)
    page.screenshot(path=os.path.join(SHOTS, "hmi3-1920x1080.png"))
    assert [x for x, _ in p.posts] == []        # the page sent nothing that moves the motor
    p.close()
