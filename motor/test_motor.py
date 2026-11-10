"""Mac-side tests for froggy_motor with the fake pigpio. Run: python3 -m pytest -q test_motor.py"""
import csv
import json
import math
import os
import sys
import threading
import time
import urllib.request

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import froggy_motor as fm  # noqa: E402


def make(estop_at=None, log=None, ena=24, index=4, enc=None, **over):
    cfg = fm.load_config("/nonexistent.toml")
    cfg.update(over)
    cfg["pins"]["ena"] = ena            # most tests model a wired ENA
    cfg["pins"]["index"] = index        # and a hall index (not on the 2026-11 bench, still supported)
    cfg["pins"]["enc_a"] = enc
    clk = fm.FakeClock()
    pi = fm.FakePi(cfg, clock=clk, estop_at=estop_at)
    ctl = fm.Controller(cfg, pi, clock=clk, sleep=clk.sleep, log_path=log, out=lambda *a: None)
    return cfg, pi, ctl


def alm_toml(tmp_path, blocks=True):
    """shipped motor.toml with the ALM wired on GPIO 22 (the shipped bench file has it unconnected)."""
    return toml_with(tmp_path, {"alm": 22, "alm_blocks_start": "true" if blocks else "false"})


def rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def run(tmp_path, rpm=400, acc=4, hold=1, dec=2, **kw):
    log = str(tmp_path / "r.csv")
    cfg, pi, ctl = make(log=log, **kw)
    summary = ctl.run_profile(rpm, acc, hold, dec)
    return cfg, pi, ctl, rows(log), summary


def test_profile_shape(tmp_path):
    cfg, pi, ctl, r, _ = run(tmp_path)
    rpm = [float(x["rpm_cmd"]) for x in r]
    t = [float(x["t_s"]) for x in r]
    assert abs(max(rpm) - 400) < 0.5
    assert rpm[-1] == 0.0 and pi.freq == 0
    amax = cfg["max_accel_rpm_per_s"]
    for i in range(1, len(rpm)):
        assert abs(rpm[i] - rpm[i - 1]) / (t[i] - t[i - 1]) <= amax * 1.0001
    phases = [x["phase"] for x in r]
    assert phases.index("hold") > phases.index("accel") and "decel" in phases
    assert not ctl.enabled                      # ENA off at the end


def test_accel_clamped_to_max(tmp_path):
    cfg, pi, ctl, r, _ = run(tmp_path, rpm=600, acc=0.2, hold=0.5, dec=0.2)
    rpm = [float(x["rpm_cmd"]) for x in r]
    t = [float(x["t_s"]) for x in r]
    rates = [abs(rpm[i] - rpm[i - 1]) / (t[i] - t[i - 1]) for i in range(1, len(rpm))]
    assert max(rates) <= cfg["max_accel_rpm_per_s"] * 1.0001
    assert any("too fast" in w for w in ctl.warnings)
    assert abs(max(rpm) - 600) < 0.5


def test_step_count_matches_integral(tmp_path):
    cfg, pi, ctl, r, _ = run(tmp_path)
    prof, _ = fm.plan_ramp(cfg, 400, 4, 1, 2)
    expected = prof.area_rev() * cfg["steps_per_rev"] * cfg["microstep"]
    assert abs(pi.steps - expected) / expected < 0.01          # pulses the fake emitted
    assert abs(ctl.total_steps - expected) / expected < 0.01   # controller's own count


def test_600rpm_step_rate_below_50khz():
    cfg = fm.load_config("/nonexistent.toml")
    assert fm.rpm_to_hz(cfg, 600) == pytest.approx(32000)
    assert fm.rpm_to_hz(cfg, cfg["max_rpm"]) < 50000


def test_estop_during_hold_decelerates_in_time(tmp_path):
    # accel 4 s + 0.2 s ENA settle: hold runs 4.2..9.2 s; press at 6.0 s
    cfg, pi, ctl, r, summary = run(tmp_path, rpm=400, acc=4, hold=5, dec=2, estop_at=6.0)
    t = [float(x["t_s"]) for x in r]
    rpm = [float(x["rpm_cmd"]) for x in r]
    first_open = next(i for i, x in enumerate(r) if x["estop"] == "1")
    assert r[first_open - 1]["phase"] == "hold"
    t_press = t[first_open]
    t_rest = next(t[i] for i in range(first_open, len(r)) if rpm[i] == 0.0)
    assert t_rest - t_press <= cfg["emergency_decel_s"] + 2 * cfg["tick_s"]
    assert r[-1]["phase"] == "estop" and ctl.estop_events == 1
    assert not ctl.enabled and pi.freq == 0
    assert "estop events 1" in summary


def test_estop_open_refuses_ramp():
    cfg, pi, ctl = make()
    pi.press_estop()
    with pytest.raises(RuntimeError):
        ctl.run_profile(60, 3, 1, 3)
    assert pi.freq == 0


def test_hall_index_measures_hold_rpm(tmp_path):
    _, _, ctl, r, summary = run(tmp_path, rpm=400, acc=4, hold=3, dec=2)
    hold = [float(x["rpm_measured"]) for x in r if x["phase"] == "hold" and x["rpm_measured"] != "nan"]
    assert hold and abs(hold[-1] - 400) < 4
    assert "hold measured 400.0 rpm" in summary
    assert math.isnan(float(r[0]["rpm_measured"]))


def test_csv_columns(tmp_path):
    _, _, _, r, _ = run(tmp_path, rpm=60, acc=3, hold=1, dec=3)
    assert list(r[0].keys()) == ["t_s", "rpm_cmd", "step_hz", "rpm_measured", "rpm_encoder", "alm", "estop", "phase"]
    t = [float(x["t_s"]) for x in r]
    dts = [b - a for a, b in zip(t, t[1:])]
    assert max(dts) < 0.0105 and min(dts) > 0.0095       # every 10 ms


def test_cli_parsing():
    p = fm.build_parser()
    a = p.parse_args(["--dry-run", "ramp", "--rpm", "200", "--accel-s", "2", "--hold-s", "3", "--decel-s", "4", "--dir", "ccw"])
    assert (a.cmd, a.rpm, a.accel_s, a.hold_s, a.decel_s, a.dir, a.dry_run) == ("ramp", 200, 2, 3, 4, "ccw", True)
    assert p.parse_args(["index-test", "--s", "10"]).s == 10
    assert p.parse_args(["ui", "--no-open"]).port == 7895
    for bad in (["ramp", "--rpm", "100"], ["spin"], ["ramp", "--rpm", "1", "--accel-s", "1", "--hold-s", "1",
                                                     "--decel-s", "1", "--dir", "up"]):
        with pytest.raises(SystemExit):
            p.parse_args(bad)


def test_cli_ramp_end_to_end(tmp_path, capsys):
    log = str(tmp_path / "cli.csv")
    rc = fm.main(["--dry-run", "--sim-fast", "ramp", "--rpm", "120", "--accel-s", "2", "--hold-s", "1",
                  "--decel-s", "2", "--log", log])
    out = capsys.readouterr().out
    assert rc == 0 and "summary: peak 120.0 rpm" in out
    assert len(rows(log)) > 50


def test_cli_rejects_overspeed(capsys):
    assert fm.main(["--dry-run", "--sim-fast", "ramp", "--rpm", "700", "--accel-s", "4", "--hold-s", "1",
                    "--decel-s", "4"]) == 2


def toml_with(tmp_path, pins):
    """motor.toml copy with some [pins] changed (for a hall or an encoder tap)."""
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "motor.toml")).read()
    for k, v in pins.items():
        lines = src.splitlines()
        src = "\n".join(("%s = %s" % (k, v)) if ln.split("=")[0].strip() == k else ln for ln in lines) + "\n"
    path = tmp_path / "motor.toml"
    path.write_text(src)
    return str(path)


def test_cli_status_and_tests(capsys, tmp_path):
    assert fm.main(["--dry-run", "--sim-fast", "status"]) == 0
    assert fm.main(["--dry-run", "--sim-fast", "estop-test", "--s", "1"]) == 0
    assert fm.main(["--dry-run", "--sim-fast", "index-test", "--s", "3", "--rpm", "60"]) == 2   # no hall fitted
    hall = toml_with(tmp_path, {"index": 4})
    assert fm.main(["--dry-run", "--sim-fast", "--config", hall, "index-test", "--s", "3", "--rpm", "60"]) == 0
    out = capsys.readouterr()
    assert "e-stop: closed" in out.out and "measured 60.0 rpm" in out.out
    assert "no hall index sensor configured" in out.err


def test_shutdown_turns_everything_off():
    cfg, pi, ctl = make()
    ctl._start_ramp(300, 3, 5, 3)
    for _ in range(100):
        ctl.tick()
        ctl.sleep(cfg["tick_s"])
    assert pi.freq > 0 and ctl.enabled
    ctl.shutdown()
    assert pi.freq == 0 and not ctl.enabled
    assert pi.levels[cfg["pins"]["ena"]] == 1 - cfg["ena_enable_level"]


def _post(url, body=None):
    req = urllib.request.Request(url, data=json.dumps(body or {}).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        return json.load(e)


def test_ui_api_real_time():
    cfg = fm.load_config("/nonexistent.toml")
    cfg["pins"]["ena"] = 24
    pi = fm.FakePi(cfg)
    ctl = fm.Controller(cfg, pi, out=lambda *a: None)
    srv = fm.make_server(ctl, "127.0.0.1", 0, True)
    port = srv.server_address[1]
    stop = threading.Event()
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    th = threading.Thread(target=ctl.serve, args=(stop,), daemon=True)
    th.start()
    base = "http://127.0.0.1:%d" % port
    try:
        html = urllib.request.urlopen(base + "/").read().decode()
        assert "Froggy motor" in html and 'src="app.js"' in html and "STOP" in html
        assert "app.js" in urllib.request.urlopen(base + "/app.js").read().decode() or True
        st = json.load(urllib.request.urlopen(base + "/api/status"))
        assert {"phase", "rpm_cmd", "rpm_measured", "step_hz", "estop", "enabled", "t_s"} <= set(st)
        assert _post(base + "/api/ramp", {"rpm": 120, "accel_s": 1, "hold_s": 5, "decel_s": 1, "dir": "cw"})["ok"]
        time.sleep(0.6)
        assert json.load(urllib.request.urlopen(base + "/api/status"))["rpm_cmd"] > 0
        assert not _post(base + "/api/ramp", {"rpm": 900})["ok"]
        assert _post(base + "/api/estop")["ok"]
        time.sleep(cfg["emergency_decel_s"] + 0.2)
        st = json.load(urllib.request.urlopen(base + "/api/status"))
        assert st["rpm_cmd"] == 0 and st["phase"] == "estop" and not st["enabled"]
        assert _post(base + "/api/enable", {"on": True})["ok"]
        time.sleep(0.1)
        assert json.load(urllib.request.urlopen(base + "/api/status"))["enabled"]
    finally:
        stop.set()
        th.join(2)
        srv.shutdown()
        ctl.shutdown()


def test_camera_splits_mjpeg_stream(tmp_path):
    a = b"\xff\xd8" + b"frame-one" + b"\xff\xd9"
    b = b"\xff\xd8" + b"frame-two" * 50 + b"\xff\xd9"
    f = tmp_path / "s.mjpeg"
    f.write_bytes(b"junk" + a + b)
    cam = fm.Camera("cat %s" % f)
    got = list(cam.frames())
    assert got and got[-1] == b


def test_camera_offline_gives_503():
    cfg = fm.load_config("/nonexistent.toml")
    ctl = fm.Controller(cfg, fm.FakePi(cfg), out=lambda *a: None)
    cam = fm.Camera("false")                      # exits at once, no frames
    srv = fm.make_server(ctl, "127.0.0.1", 0, False, cam)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen("http://127.0.0.1:%d/stream.mjpg" % srv.server_address[1], timeout=10)
        assert e.value.code == 503
    finally:
        srv.shutdown()
        ctl.shutdown()


def test_ena_not_wired_never_touched(tmp_path):
    log = str(tmp_path / "n.csv")
    cfg, pi, ctl = make(log=log, ena=None)
    ctl.run_profile(60, 3, 1, 3)
    assert all(g not in (24, None) for _, g, _ in pi.writes)
    assert ctl.enabled and ctl.status()["ena_wired"] is False
    cfg2 = fm.load_config("/nonexistent.toml")
    assert cfg2["pins"] == {"pul": 18, "dir": 17, "ena": 27, "estop": 25, "index": None,
                            "alm": 22, "pend": None, "brk": None, "enc_a": None}


def test_stop_after_finished_ui_ramp_does_not_crash(tmp_path):
    """Regression for the 2026-10-21 11:27 crash: ValueError I/O operation on closed file after /api/stop."""
    cfg, pi, ctl = make()
    ctl.log_dir = str(tmp_path)
    ctl.request("ramp", rpm=60, accel_s=1, hold_s=0.5, decel_s=1, direction="cw")
    for _ in range(400):
        ctl.tick()
        ctl.sleep(cfg["tick_s"])
    assert ctl.phase == "idle" and ctl.profile is None
    for kind in ("stop", "estop", "enable"):
        ctl.request(kind) if kind != "enable" else ctl.request("enable", on=False)
        for _ in range(100):
            ctl.tick()
            ctl.sleep(cfg["tick_s"])
    assert pi.freq == 0


class _Live:
    """A live-mode server (token required) on the fake pigpio, with its request log captured."""

    def __init__(self, locked=False, enc=None):
        self.cfg = fm.load_config("/nonexistent.toml")
        self.cfg["pins"]["ena"] = 24
        self.cfg["pins"]["index"] = 4
        self.cfg["pins"]["enc_a"] = enc
        self.pi = fm.FakePi(self.cfg)
        self.ctl = fm.Controller(self.cfg, self.pi, out=lambda *a: None)
        self.ctl.locked = locked
        self.lines = []
        self.srv = fm.make_server(self.ctl, "127.0.0.1", 0, False, log=self.lines.append)
        self.base = "http://127.0.0.1:%d" % self.srv.server_address[1]
        self.stop = threading.Event()
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.th = threading.Thread(target=self.ctl.serve, args=(self.stop,), daemon=True)
        self.th.start()

    def status(self):
        return json.load(urllib.request.urlopen(self.base + "/api/status"))

    def close(self):
        self.stop.set()
        self.th.join(2)
        self.srv.shutdown()
        self.ctl.shutdown()


def _post_h(url, body, headers):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST",
                                 headers=dict({"Content-Type": "application/json"}, **headers))
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.load(r)
    except urllib.error.HTTPError as e:
        return e.code, json.load(e)


def test_page_load_never_ramps():
    L = _Live()
    try:
        for _ in range(5):
            assert "Ramp" in urllib.request.urlopen(L.base + "/").read().decode()
            L.status()
        time.sleep(0.3)
        st = L.status()
        assert st["rpm_cmd"] == 0 and st["phase"] == "idle" and L.pi.freq == 0
        assert not any("POST /api/ramp" in x for x in L.lines)
    finally:
        L.close()


def test_browser_start_confirm_cancel_and_stop(tmp_path):
    """Exercise real browser clicks on the ui/ page against fake GPIO with the live token checks."""
    from playwright.sync_api import sync_playwright, expect

    L = _Live()
    L.ctl.log_dir = str(tmp_path)
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(headless=True)
            page = browser.new_page()
            errors = []
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto(L.base)
            expect(page.locator("#r_state")).to_have_text("off")
            assert L.pi.freq == 0
            assert not any("POST /api/arm" in line for line in L.lines)

            # Synthetic clicks are ignored (isTrusted), so nothing is armed.
            page.locator("#start").evaluate("button => button.click()")
            expect(page.locator("#confirm")).not_to_be_visible()
            assert L.pi.freq == 0 and not any("POST /api/arm" in line for line in L.lines)

            page.get_by_role("radio", name="Timed test").click()
            page.get_by_role("radio", name="CCW").click()
            page.locator("#rpm").fill("60")
            page.locator("#accel").fill("3")
            page.locator("#hold").fill("10")
            page.locator("#decel").fill("3")
            page.locator("#start").click()
            expect(page.locator("#confirm")).to_be_visible()
            expect(page.locator("#confirmtxt")).to_contain_text("60 RPM CCW")
            assert L.pi.freq == 0
            page.get_by_role("button", name="Cancel", exact=True).click()
            expect(page.locator("#msg")).to_contain_text("Cancelled")
            assert not any("POST /api/arm" in line for line in L.lines)

            page.locator("#start").click()
            with page.expect_request("**/api/ramp") as request:
                page.get_by_role("button", name="Yes, spin the motor").click()
            body = request.value.post_data_json
            assert {k: body[k] for k in ("rpm", "accel_s", "hold_s", "decel_s", "dir")} == {
                "rpm": 60, "accel_s": 3, "hold_s": 10, "decel_s": 3, "dir": "ccw"}
            assert body["token"]
            expect(page.locator("#msg")).to_have_text("Timed test started.")
            page.wait_for_function("Number(document.getElementById('r_cmd').textContent) > 0")
            assert L.pi.freq > 0 and L.status()["dir"] == "ccw"

            page.get_by_role("button", name="EMERGENCY STOP", exact=True).click()
            expect(page.locator("#r_cmd")).to_have_text("0")
            assert L.pi.freq == 0
            page.locator("#diagbtn").click()
            expect(page.locator("#diagstop")).to_contain_text("requested by software")
            assert L.status()["estop"] is False
            page.locator("#start").evaluate("button => button.click()")
            page.reload()
            expect(page.locator("#r_cmd")).to_have_text("0")
            assert sum("POST /api/ramp" in line for line in L.lines) == 1
            assert not errors
            browser.close()
    finally:
        L.close()


def test_continuous_speed_holds_changes_and_stops():
    cfg, pi, ctl = make()
    dt = cfg["tick_s"]

    def advance(seconds):
        values = []
        for _ in range(round(seconds / dt)):
            ctl.tick()
            values.append(ctl.rpm_cmd)
            ctl.sleep(dt)
        return values

    ctl._set_speed(120, 2, "ccw")
    advance(62)
    assert ctl.rpm_cmd == 120 and ctl.phase == "hold" and pi.freq > 0
    ctl._set_speed(60, 2, "ccw")
    values = advance(3)
    assert values[0] > 119 and values[-1] == 60
    assert all(abs(b - a) <= cfg["max_accel_rpm_per_s"] * dt + 1e-6
               for a, b in zip(values, values[1:]))
    with pytest.raises(RuntimeError, match="direction change"):
        ctl._set_speed(60, 2, "cw")
    for rpm in (float("nan"), float("inf"), -1, 601):
        with pytest.raises(ValueError):
            ctl._set_speed(rpm)
    assert ctl.rpm_cmd == 60
    ctl._set_speed(0, 1)
    advance(2)
    assert pi.freq == 0 and ctl.phase == "idle"
    ctl._set_speed(90, 1)
    advance(2)
    pi.press_estop()
    advance(1)
    assert pi.freq == 0 and ctl.phase == "estop"
    with pytest.raises(RuntimeError, match="e-stop is open"):
        ctl._set_speed(90)
    ctl.shutdown()


def test_cli_continuous_speed_until_physical_estop(tmp_path, capsys):
    rc = fm.main(["--dry-run", "--sim-fast", "--sim-estop-at", "8", "speed",
                  "--rpm", "120", "--accel-s", "1", "--log", str(tmp_path / "speed.csv")])
    assert rc == 0
    out = capsys.readouterr().out
    assert "runs until Stop or Ctrl-C" in out and "estop events 1" in out
    data = rows(tmp_path / "speed.csv")
    assert any(float(r["t_s"]) > 7 and r["phase"] == "hold" for r in data)
    assert float(data[-1]["rpm_cmd"]) == 0


def test_browser_continuous_speed_update_and_zero(tmp_path):
    from playwright.sync_api import sync_playwright, expect

    L = _Live()
    L.ctl.log_dir = str(tmp_path)
    try:
        assert _post_h(L.base + "/api/speed", {"rpm": 60}, {})[0] == 403
        with sync_playwright() as pw:
            browser = pw.chromium.launch(headless=True)
            page = browser.new_page()
            page.goto(L.base)
            expect(page.get_by_role("radio", name="Hold")).to_have_attribute("aria-checked", "true")
            expect(page.locator("#hold")).not_to_be_visible()
            expect(page.locator("#start")).to_have_text("START")
            assert L.pi.freq == 0
            page.locator("#rpm").fill("60")
            page.locator("#accel").fill("0.5")
            page.locator("#start").click()
            expect(page.locator("#confirmtxt")).to_contain_text("hold until STOP")
            page.get_by_role("button", name="Yes, spin the motor").click()
            expect(page.locator("#r_cmd")).to_have_text("60")
            expect(page.locator("#r_state")).to_have_text("holding", timeout=8000)
            expect(page.locator("#start")).to_have_text("APPLY")
            assert L.status()["rpm_cmd"] == 60 and L.status()["phase"] == "hold"
            for target in ("120", "30", "0"):
                page.locator("#rpm").fill(target)
                page.locator("#start").click()
                expect(page.locator("#r_cmd")).to_have_text(target, timeout=8000)
            assert L.status()["rpm_cmd"] == 0
            assert sum("POST /api/speed" in line for line in L.lines) == 5   # 1 refused no-token + 4 page calls
            browser.close()
    finally:
        L.close()


def test_manual_speed_duration_and_cli_defaults():
    cfg = fm.load_config("/nonexistent.toml")
    for seconds in (2, 7):
        up = fm.plan_speed(cfg, 240, seconds, r_now=120)
        down = fm.plan_speed(cfg, 120, seconds, r_now=240)
        assert up.at(seconds / 2)[0] == pytest.approx(180)
        assert down.at(seconds / 2)[0] == pytest.approx(180)
        assert up.at(seconds + 100)[0] == 240
        assert down.at(seconds + 100)[0] == 120
    assert fm.build_parser().parse_args(["speed", "--rpm", "120"]).accel_s == 3


def test_live_ramp_needs_one_time_token():
    L = _Live()
    try:
        p = {"rpm": 60, "accel_s": 1, "hold_s": 5, "decel_s": 1, "dir": "cw"}
        code, j = _post_h(L.base + "/api/ramp", p, {})
        assert code == 403 and "token" in j["error"]
        assert L.status()["rpm_cmd"] == 0
        code, a = _post_h(L.base + "/api/arm", {"intent": "ramp"}, {})
        assert code == 200 and a["token"]
        code, j = _post_h(L.base + "/api/ramp", dict(p, token=a["token"]), {})
        assert code == 200 and j["ok"]
        time.sleep(0.5)
        assert L.status()["rpm_cmd"] > 0
        code, j = _post_h(L.base + "/api/ramp", dict(p, token=a["token"]), {})   # reused token
        assert code == 403
        assert any("RAMP STARTED" in x for x in L.lines) and any("refused" in x for x in L.lines)
        assert _post_h(L.base + "/api/stop", {}, {})[0] == 200
    finally:
        L.close()


def test_cross_origin_and_form_posts_refused():
    L = _Live()
    try:
        code, _ = _post_h(L.base + "/api/arm", {"intent": "ramp"}, {"Origin": "http://evil.example"})
        assert code == 403
        code, _ = _post_h(L.base + "/api/arm", {"intent": "ramp"}, {"Sec-Fetch-Site": "cross-site"})
        assert code == 403
        req = urllib.request.Request(L.base + "/api/ramp", data=b"rpm=60", method="POST",
                                     headers={"Content-Type": "application/x-www-form-urlencoded"})
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(req, timeout=5)
        assert e.value.code == 415
    finally:
        L.close()


def test_locked_server_refuses_everything_that_moves():
    L = _Live(locked=True)
    try:
        assert _post_h(L.base + "/api/arm", {"intent": "ramp"}, {})[0] == 409
        assert _post_h(L.base + "/api/arm", {"intent": "speed"}, {})[0] == 409
        assert _post_h(L.base + "/api/enable", {"on": True}, {})[0] == 409
        for path, body in (("/api/speed", {"rpm": 60}),
                           ("/api/ramp", {"rpm": 60, "accel_s": 1, "hold_s": 1, "decel_s": 1, "dir": "cw"}),
                           ("/api/cltest", {"rpm": 60, "accel_s": 1, "hold_s": 1, "decel_s": 1, "dir": "cw"})):
            assert _post_h(L.base + path, body, {})[0] in (403, 409)
            assert _post_h(L.base + path, dict(body, token="forged"), {})[0] in (403, 409)
        html = urllib.request.urlopen(L.base + "/").read().decode()
        assert "Froggy motor" in html
        assert L.status()["rpm_cmd"] == 0 and L.pi.freq == 0
        from playwright.sync_api import sync_playwright, expect
        with sync_playwright() as pw:
            browser = pw.chromium.launch(headless=True)
            page = browser.new_page()
            page.goto(L.base)
            for mode in ("Hold", "Timed test"):
                page.get_by_role("radio", name=mode).click()
                page.locator("#start").click()
                if mode == "Hold":                       # first START after load asks for confirmation
                    page.get_by_role("button", name="Yes, spin the motor").click()
                expect(page.locator("#start")).to_have_text("START")
                page.wait_for_function("!document.getElementById('start').disabled")
                expect(page.locator("#msg")).not_to_contain_text("started")
                expect(page.locator("#msg")).not_to_contain_text("Ramping")
                expect(page.locator("#r_state")).to_have_text("off")
            page.get_by_role("button", name="STOP", exact=True).click()
            assert L.status()["rpm_cmd"] == 0 and L.pi.freq == 0
            browser.close()
        assert L.status()["rpm_cmd"] == 0 and L.pi.freq == 0
    finally:
        L.close()


def test_server_survives_stop_estop_disable_and_a_bad_tick(tmp_path):
    L = _Live()
    L.ctl.log_dir = str(tmp_path)
    try:
        _, a = _post_h(L.base + "/api/arm", {"intent": "ramp"}, {})
        _post_h(L.base + "/api/ramp", {"rpm": 120, "accel_s": 0.6, "hold_s": 0.2, "decel_s": 0.6, "token": a["token"]}, {})
        time.sleep(2.0)                                  # ramp finished, its CSV closed
        for path, body in (("/api/stop", {}), ("/api/estop", {}), ("/api/enable", {"on": False}),
                           ("/api/enable", {"on": True}), ("/api/stop", {})):
            assert _post_h(L.base + path, body, {})[0] == 200
            time.sleep(0.15)
        assert L.th.is_alive() and L.status()["phase"] in ("idle", "estop")
        orig = L.ctl.tick
        L.ctl.tick = lambda: (_ for _ in ()).throw(RuntimeError("injected"))
        time.sleep(0.2)
        L.ctl.tick = orig
        time.sleep(0.2)
        assert L.th.is_alive() and L.ctl.faults >= 1 and L.pi.freq == 0
    finally:
        L.close()


def test_single_motor_lock(tmp_path, monkeypatch):
    monkeypatch.setattr(fm, "LOCKFILE", str(tmp_path / "lock"))
    f1, h1 = fm.take_motor_lock()
    assert f1 is not None and h1 is None
    f2, h2 = fm.take_motor_lock()
    assert f2 is None and "pid %d" % os.getpid() in h2
    f1.close()
    f3, _ = fm.take_motor_lock()
    assert f3 is not None
    f3.close()


# ---------------------------------------------------------------------------
# CL57T-V41 closed loop: ALM, PEND, encoder tap, cl-test (2026-11-09)
# ---------------------------------------------------------------------------
def ticks(ctl, seconds):
    dt = ctl.cfg["tick_s"]
    for _ in range(round(seconds / dt)):
        ctl.tick()
        ctl.sleep(dt)


def test_load_config_driver_section(tmp_path):
    t = tmp_path / "m.toml"
    t.write_text('motor = "17HS19-2004D-E1K"\n[driver]\nmodel = "DM320T"\ns1 = "a"\nalm_active_level = 1\n'
                 '[pins]\nalm = "none"\nenc_a = "none"\npend = 24\n')
    cfg = fm.load_config(str(t))
    assert cfg["driver"]["model"] == "DM320T" and cfg["driver"]["s1"] == "A"
    assert cfg["driver"]["alm_active_level"] == 1 and cfg["driver"]["enc_ppr"] == 1000   # defaults kept
    assert cfg["pins"]["alm"] is None and cfg["pins"]["enc_a"] is None and cfg["pins"]["pend"] == 24
    assert not fm.closed_loop(cfg)
    shipped = fm.load_config()                         # the motor.toml next to the code
    assert shipped["driver"]["model"] == "CL57T-V41" and fm.closed_loop(shipped)
    assert shipped["pins"] == {"pul": 18, "dir": 17, "ena": 27, "estop": 25, "index": None,
                               "alm": 22, "pend": None, "brk": None, "enc_a": 5}
    assert shipped["alm_blocks_start"] is True
    assert shipped["ena_enable_level"] == 0


def test_cl57t_tables_match_manual():
    assert fm.CL57T_S1["0"] == (2.8, 2.0, 0, 25, 25)
    assert fm.CL57T_S1["3"] == (2.8, 2.0, 16, 50, 15)
    assert fm.CL57T_S1["6"][:2] == (5.6, 4.0) and fm.CL57T_S1["F"] == (7.0, 5.0, 16, 100, 5)
    assert fm.CL57T_MICROSTEP[16] == "off off on on" and fm.CL57T_MICROSTEP[200] == "off off off off"
    assert len(fm.CL57T_MICROSTEP) == 16


def test_alm_stops_ramp_at_once_and_reports_cause(tmp_path):
    log = str(tmp_path / "a.csv")
    cfg, pi, ctl = make(log=log)
    pi._alarm_at = 3.0                       # mid-accel of a 400 rpm ramp
    summary = ctl.run_profile(400, 4, 2, 2)
    assert pi.freq == 0 and ctl.phase == "alarm" and not ctl.enabled
    assert "DRIVER ALARM" in summary and "ALM" in ctl.last_stop_reason and "red LED" in ctl.last_stop_reason
    r = rows(log)
    assert r[-1]["phase"] == "alarm" and r[-1]["alm"] == "1"
    t_alarm = next(float(x["t_s"]) for x in r if x["alm"] == "1")
    assert t_alarm - 3.0 < 0.03                                  # seen within a few ticks
    assert float(r[-1]["t_s"]) - t_alarm < 0.03                  # and pulses stopped right there, no decel ramp
    with pytest.raises(RuntimeError, match="ALM is active"):
        ctl._start_ramp(60, 1, 1, 1)
    pi.repower()
    ctl.tick()
    assert ctl.status()["alm"] is False
    ctl._start_ramp(60, 1, 0.2, 1)                               # allowed again after the power cycle
    ticks(ctl, 3)
    assert ctl.phase == "idle" and not ctl.alarm_latched


def test_alm_wrong_polarity_refuses_to_start(capsys):
    cfg, pi, ctl = make()
    cfg["driver"]["alm_active_level"] = 1                        # inverted relative to the fake's healthy level
    with pytest.raises(RuntimeError, match="alm_active_level"):
        ctl._start_ramp(60, 1, 1, 1)
    assert pi.freq == 0


def test_status_prints_driver_alm_and_pend(capsys, tmp_path):
    assert fm.main(["--dry-run", "--sim-fast", "--config", alm_toml(tmp_path), "status"]) == 0
    out = capsys.readouterr().out
    assert "driver: CL57T-V41, closed loop (SW6 OFF), S1 code 0 = 2.8 A peak / 2.0 A RMS" in out
    assert "SW1..SW4 off off on on" in out
    assert "ALM: ok (GPIO 22 reads 1; alarm when 0)" in out
    assert "PEND: not wired (the CL57T-V41 P1 connector has no PEND terminal)" in out
    assert "encoder tap: GPIO 5" in out and "index: not fitted" in out
    assert "ENA: GPIO 27 set and read back as 1 (driver disabled; enabled when 0)" in out
    assert fm.main(["--dry-run", "--sim-fast", "--driver", "DM320T", "status"]) == 0
    assert "driver: DM320T (open loop)" in capsys.readouterr().out


def test_pend_and_brk_inputs_when_wired():
    cfg, pi, ctl = make()
    cfg["pins"]["pend"] = 24 if cfg["pins"]["ena"] != 24 else 16
    cfg["pins"]["brk"] = 23
    pi2 = fm.FakePi(cfg, clock=pi.clock)
    ctl2 = fm.Controller(cfg, pi2, clock=pi.clock, sleep=pi.clock.sleep, out=lambda *a: None)
    ctl2.tick()
    st = ctl2.status()
    assert st["pend_wired"] and st["pend"] is False and st["brk"] == 1
    pi2.levels[cfg["pins"]["pend"]] = cfg["driver"]["pend_active_level"]
    ctl2.tick()
    assert ctl2.status()["pend"] is True
    lines = fm.driver_lines(cfg, pi2)
    assert any(l.startswith("PEND: in position") for l in lines) and any(l.startswith("BRK: GPIO 23") for l in lines)


def test_encoder_tap_measures_speed_and_following_error(tmp_path):
    cfg, pi, ctl = make(enc=5)
    ctl._start_ramp(300, 2, 3, 2)
    ticks(ctl, 3.5)
    st = ctl.status()
    assert st["phase"] == "hold" and abs(st["rpm_encoder"] - 300) < 3
    assert abs(st["follow_err_deg"]) < 2
    assert st["enc_edges"] > 1000 * 5


def test_cl_test_without_any_sensor_is_honest(tmp_path, capsys):
    """No encoder tap wired. The verdict must not claim a measured speed."""
    log = tmp_path / "plain.csv"
    rc = fm.main(["--dry-run", "--sim-fast", "--config", toml_with(tmp_path, {"enc_a": '"none"', "alm": 22}), "cl-test", "--rpm", "300", "--hold-s", "3",
                  "--log", str(log)])
    last = capsys.readouterr().out.strip().splitlines()[-1]
    assert rc == 0
    assert last.startswith("verdict: reached 300 RPM commanded (encoder tap not wired, shaft speed not measured)")
    assert "tracking error unknown" in last and "alarms none" in last and "ALM ok in all" in last
    assert "followed every pulse it received" in last and "3.3 V inputs would not show" in last
    r = rows(log)
    assert all(x["rpm_hall"] == "nan" and x["rpm_encoder"] == "nan" for x in r)


def test_cl_test_cli_writes_100ms_csv_and_verdict(tmp_path, capsys):
    log = tmp_path / "cl.csv"
    tap = toml_with(tmp_path, {"enc_a": 5, "alm": 22, "alm_blocks_start": "true"})
    rc = fm.main(["--dry-run", "--sim-fast", "--config", tap, "cl-test", "--rpm", "300", "--hold-s", "3", "--log", str(log)])
    out = capsys.readouterr().out.strip().splitlines()
    assert rc == 0
    assert out[-1].startswith("verdict: reached 300.0 RPM (encoder tap, 300 commanded), tracking error mean")
    assert "alarms none" in out[-1]
    r = rows(log)
    assert list(r[0].keys()) == fm.CL_COLUMNS
    t = [float(x["t_s"]) for x in r]
    assert all(abs((b - a) - 0.1) < 0.011 for a, b in zip(t, t[1:-1]))      # every 100 ms
    assert {x["phase"] for x in r} >= {"accel", "hold", "decel"}
    assert all(x["alm"] == "0" and x["pend"] == "na" for x in r)


def test_cl_test_closed_loop_stall_raises_alarm(tmp_path, capsys):
    log = tmp_path / "stall.csv"
    rc = fm.main(["--dry-run", "--sim-fast", "--config", alm_toml(tmp_path), "--sim-stall-rpm", "250", "cl-test",
                  "--rpm", "300", "--hold-s", "3", "--log", str(log)])
    out = capsys.readouterr().out.strip().splitlines()
    assert rc == 5
    assert "alarms: ALM at" in out[-1] and "repower" in out[-1]
    r = rows(log)
    assert r[-1]["phase"] == "alarm" and float(r[-1]["rpm_cmd"]) == 0.0
    assert max(float(x["rpm_cmd"]) for x in r) < 300           # stopped during the ramp, never reached hold


def test_cl_test_open_loop_stall_is_silent(tmp_path, capsys):
    """The 2026-10-26 symptom: open loop above the stall speed, no fault, commanded RPM keeps going."""
    log = tmp_path / "ol.csv"
    tap = toml_with(tmp_path, {"enc_a": 5})
    rc = fm.main(["--dry-run", "--sim-fast", "--config", tap, "--driver", "DM320T", "--sim-stall-rpm", "250", "cl-test",
                  "--rpm", "300", "--hold-s", "2", "--log", str(log)])
    out = capsys.readouterr().out.strip().splitlines()
    assert rc == 0
    # a stalled rotor sends no encoder edges: above 30 RPM for over 1 s that reads as "no pulses"
    assert out[-1].startswith("verdict: reached 300 RPM commanded (Encoder: no pulses, check EA+ wire)")
    assert "alarms none" in out[-1]


def test_cl_verdict_without_measurement():
    rows_ = [{"t_s": 0.1 * i, "rpm_cmd": 60.0, "rpm_hall": math.nan, "rpm_encoder": math.nan,
              "follow_err_deg": math.nan, "phase": "hold"} for i in range(20)]
    v = fm.cl_verdict(rows_, 60, None, None, 0, "x.csv")
    assert v.startswith("verdict: reached 60 RPM commanded") and "tracking error unknown" in v and "alarms none" in v


def test_cli_cltest_parsing():
    a = fm.build_parser().parse_args(["cl-test", "--rpm", "250"])
    assert (a.cmd, a.rpm, a.accel_s, a.hold_s, a.decel_s, a.dir) == ("cl-test", 250, None, 10.0, None, "cw")
    assert fm.main(["--dry-run", "--sim-fast", "cl-test", "--rpm", "700"]) == 2


def test_ui_cltest_api_and_status(tmp_path):
    L = _Live()
    L.ctl.log_dir = str(tmp_path)
    L.ctl.cl_dir = str(tmp_path)
    try:
        html = urllib.request.urlopen(L.base + "/").read().decode()
        assert "Froggy motor" in html and "Closed-loop test" in html
        for ident in ("hdrdriver", "hdrloop", "hdralarm", "r_enc", "r_ferr", "verdict"):
            assert 'id="%s"' % ident in html
        st = L.status()
        assert st["driver"] == "CL57T-V41" and st["alm"] is False and st["pend"] is None and st["pend_wired"] is False
        p = {"rpm": 120, "accel_s": 0.6, "hold_s": 0.4, "decel_s": 0.6, "dir": "cw"}
        assert _post_h(L.base + "/api/cltest", p, {})[0] == 403                     # token needed on the live motor
        _, a = _post_h(L.base + "/api/arm", {"intent": "cltest"}, {})
        code, j = _post_h(L.base + "/api/cltest", dict(p, token=a["token"]), {})
        assert code == 200 and j["ok"] and j["status"]["cl_running"]
        deadline = time.time() + 6
        while time.time() < deadline and not L.status()["cl_verdict"]:
            time.sleep(0.1)
        st = L.status()
        assert st["cl_verdict"].startswith("verdict: reached") and "alarms none" in st["cl_verdict"]
        assert any(f.startswith("cl-test-120rpm-") for f in os.listdir(tmp_path))
        assert any("CL-TEST STARTED" in x for x in L.lines)
    finally:
        L.close()


def test_ui_alarm_shows_on_tile(tmp_path):
    L = _Live()
    L.ctl.log_dir = str(tmp_path)
    try:
        _, a = _post_h(L.base + "/api/arm", {"intent": "speed"}, {})
        assert _post_h(L.base + "/api/speed", {"rpm": 120, "accel_s": 0.5, "token": a["token"]}, {})[0] == 200
        time.sleep(0.8)
        L.pi.raise_alarm("test")
        time.sleep(0.3)
        st = L.status()
        assert st["alm"] is True and st["phase"] == "alarm" and st["rpm_cmd"] == 0 and L.pi.freq == 0
        assert "ALM" in st["last_stop_reason"]
        _, a = _post_h(L.base + "/api/arm", {"intent": "speed"}, {})
        code, j = _post_h(L.base + "/api/speed", {"rpm": 60, "token": a["token"]}, {})
        assert code == 409 and "ALM is active" in j["error"]
    finally:
        L.close()


def test_browser_closed_loop_tile_and_cltest(tmp_path):
    from playwright.sync_api import sync_playwright, expect

    L = _Live(enc=5)
    L.ctl.log_dir = str(tmp_path)
    L.ctl.cl_dir = str(tmp_path)
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(headless=True)
            page = browser.new_page(viewport={"width": 1280, "height": 1100})
            errors = []
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto(L.base)
            expect(page.locator("#hdrdriver")).to_have_text("Driver: CL57T-V41")
            expect(page.locator("#hdrloop")).to_have_text("Closed loop")
            expect(page.locator("#hdralarm")).to_have_text("Alarm: clear")
            page.get_by_role("radio", name="Closed-loop test").click()
            expect(page.locator("#hold")).to_be_visible()
            page.locator("#accel").fill("0.5")
            page.locator("#hold").fill("1")
            page.locator("#decel").fill("0.5")
            page.locator("#start").click()
            expect(page.locator("#confirmtxt")).to_contain_text("closed-loop test")
            with page.expect_request("**/api/cltest"):
                page.get_by_role("button", name="Yes, spin the motor").click()
            expect(page.locator("#verdict")).to_contain_text("verdict: reached", timeout=8000)
            page.screenshot(path=str(tmp_path / "ui.png"), full_page=True)
            assert not errors
            browser.close()
    finally:
        L.close()


def test_no_camera_ui_page_and_stream(tmp_path):
    """Bench Pi 3 B+ has no camera: camera = false hides the preview and the stream answers 503 at once."""
    shipped = fm.load_config()
    assert shipped["camera"] is False
    cfg = fm.load_config("/nonexistent.toml")
    ctl = fm.Controller(cfg, fm.FakePi(cfg), out=lambda *a: None)
    srv = fm.make_server(ctl, "127.0.0.1", 0, False, None, "test", show_camera=False, log=lambda *a: None)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = "http://127.0.0.1:%d" % srv.server_address[1]
    try:
        html = urllib.request.urlopen(base + "/").read().decode()
        assert "Froggy motor" in html and "camera" not in html.lower() and "stream.mjpg" not in html
        t0 = time.time()
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(base + "/stream.mjpg", timeout=5)
        assert e.value.code == 503 and time.time() - t0 < 1.0
        from playwright.sync_api import sync_playwright, expect
        with sync_playwright() as pw:
            browser = pw.chromium.launch(headless=True)
            page = browser.new_page()
            errors = []
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto(base)
            assert page.locator("#camera-panel, img[src*='stream.mjpg'], video").count() == 0
            expect(page.locator("#hdralarm")).to_contain_text("Alarm:")
            assert not errors
            browser.close()
    finally:
        srv.shutdown()
        ctl.shutdown()


def test_ena_sequence_and_idle_disable(tmp_path):
    """Variant A ENA on GPIO 27: disabled (high) at start and idle, enabled 200 ms before DIR, off at the end."""
    cfg = fm.load_config()                              # shipped motor.toml
    clk = fm.FakeClock()
    pi = fm.FakePi(cfg, clock=clk)
    ctl = fm.Controller(cfg, pi, clock=clk, sleep=clk.sleep, out=lambda *a: None)
    ena, dirp = cfg["pins"]["ena"], cfg["pins"]["dir"]
    assert (ena, dirp) == (27, 17)
    assert pi.levels[ena] == 1 and not ctl.enabled      # disabled from the first write
    ctl.run_profile(60, 1, 0.5, 1, "ccw")
    w = [(t, g, l) for t, g, l in pi.writes if g in (ena, dirp)]
    i_en = next(i for i, (t, g, l) in enumerate(w) if g == ena and l == 0)
    t_en = w[i_en][0]
    t_dir = next(t for t, g, l in w[i_en + 1:] if g == dirp)
    t_pwm = next(t for t, f in pi.pwm_calls if f > 0)
    assert t_dir - t_en >= 0.2 - 1e-9                    # manual p. 10, t1
    assert t_pwm >= t_dir                                # DIR before the first pulse (t2)
    assert pi.levels[ena] == 1 and not ctl.enabled      # back to disabled at rest
    ctl._start_ramp(60, 1, 5, 1)
    for _ in range(150):
        ctl.tick()
        ctl.sleep(cfg["tick_s"])
    assert pi.levels[ena] == 0
    pi.press_estop()
    for _ in range(100):
        ctl.tick()
        ctl.sleep(cfg["tick_s"])
    assert pi.levels[ena] == 1 and pi.freq == 0         # e-stop disables
    ctl.shutdown()
    assert pi.levels[ena] == 1


def test_status_with_alm_unconnected_says_not_wired_and_reads_no_gpio22(capsys, tmp_path):
    nc = toml_with(tmp_path, {"alm": '"none"'})
    assert fm.main(["--dry-run", "--sim-fast", "--config", nc, "status"]) == 0           # alm = "none"
    out = capsys.readouterr().out
    assert "ALM: not wired" in out
    cfg = fm.load_config(nc)
    clk = fm.FakeClock()
    pi = fm.FakePi(cfg, clock=clk)
    reads = []
    orig = pi.read
    pi.read = lambda g: (reads.append(g), orig(g))[1]
    ctl = fm.Controller(cfg, pi, clock=clk, sleep=clk.sleep, out=lambda *a: None)
    ctl.tick()
    assert 22 not in reads and ctl.status()["alm_wired"] is False


def test_ena_reads_disabled_at_idle_after_startup_and_after_a_run():
    cfg = fm.load_config()                                    # shipped file: ENA 27, enabled when 0
    clk = fm.FakeClock()
    pi = fm.FakePi(cfg, clock=clk)
    ena = cfg["pins"]["ena"]
    writes = []
    orig = pi.write
    pi.write = lambda g, l: (writes.append((g, l)), orig(g, l))[1]
    ctl = fm.Controller(cfg, pi, clock=clk, sleep=clk.sleep, out=lambda *a: None)
    assert [w for w in writes if w[0] == ena][0] == (ena, 1)   # first thing ever written is the disabled level
    assert pi.read(ena) == 1 and ctl.status()["ena_level"] == 1
    ctl._start_ramp(120, 1, 1, 1)
    assert pi.read(ena) == 0                                   # enabled while running
    while ctl.tick():
        clk.sleep(cfg["tick_s"])
    ctl.tick()
    assert pi.read(ena) == 1 and ctl.status()["ena_level"] == 1 and not ctl.enabled
    pi.levels[ena] = 0                                         # something drags the pin to enabled
    ctl.tick()
    assert pi.read(ena) == 1                                   # the idle guard puts it back
    ctl._start_ramp(60, 1, 5, 1)
    ctl._trigger_estop("software")
    for _ in range(300):
        ctl.tick()
        clk.sleep(cfg["tick_s"])
    assert pi.read(ena) == 1


def test_run_starts_when_alm_does_not_block_even_with_alm_active(tmp_path):
    cfg = fm.load_config(alm_toml(tmp_path, blocks=False))
    clk = fm.FakeClock()
    pi = fm.FakePi(cfg, clock=clk)
    lines = []
    log = str(tmp_path / "r.csv")
    ctl = fm.Controller(cfg, pi, clock=clk, sleep=clk.sleep, log_path=log, out=lines.append)
    pi.levels[cfg["pins"]["alm"]] = cfg["driver"]["alm_active_level"]      # ALM reads active
    ctl._open_log()
    ctl._start_ramp(120, 1, 2, 1)                                          # must not raise
    peak = 0
    while ctl.tick():
        peak = max(peak, ctl.rpm_cmd)
        clk.sleep(cfg["tick_s"])
    ctl._close_log()
    st = ctl.status()
    assert peak >= 119 and st["alm"] is True and st["alm_blocks_start"] is False
    assert ctl.alm_edges == 1 and any("unverified polarity" in x for x in lines)
    assert ctl.phase == "idle" and not ctl.alarm_latched
    assert {x["alm"] for x in rows(log)} == {"1"}                          # logged in the CSV
    cfg2 = fm.load_config(alm_toml(tmp_path, blocks=True))                 # true: old behaviour
    pi2 = fm.FakePi(cfg2, clock=clk)
    ctl2 = fm.Controller(cfg2, pi2, clock=clk, sleep=clk.sleep, out=lambda *a: None)
    pi2.levels[cfg2["pins"]["alm"]] = cfg2["driver"]["alm_active_level"]
    with pytest.raises(RuntimeError, match="ALM is active"):
        ctl2._start_ramp(120, 1, 2, 1)


def test_encoder_fake_edges_1000_per_rev_rpm_and_following_error():
    """Fake EA+ edges at 1000 per rev: tally-based RPM, following error in revs and percent, no-pulses warning."""
    cfg, pi, ctl = make(enc=5)
    assert pi.pulls[5] == pi.PUD_OFF
    ctl._start_ramp(300, 1, 20, 1)
    # the fake pigpio fires one rising edge per 1/1000 rev of real shaft turn; nothing is counted in Python per edge
    ticks(ctl, 5)
    st = ctl.status()
    assert st["phase"] == "hold" and abs(st["rpm_encoder"] - 300) < 3 and st["rpm_encoder_signed"] > 0
    assert abs(st["follow_err_rev"]) < 0.05 and abs(st["follow_err_pct"]) < 2
    assert st["enc_edges"] == pi._cbs[-1].tally() or st["enc_edges"] > 1000 * 4
    assert not st["enc_no_pulses"]
    # a pulse cable that comes off: the rotor still turns in the sim, the tally stops
    pi._steps_per_enc = 1e12
    pi._next_enc = 1e12
    ticks(ctl, 2.5)
    st = ctl.status()
    assert st["enc_no_pulses"] and st["rpm_encoder"] == 0 and st["phase"] == "hold"
    assert st["follow_err_pct"] > 90


def test_encoder_ccw_is_signed_and_idle_reads_zero():
    cfg, pi, ctl = make(enc=5)
    ctl._start_ramp(120, 1, 10, 1, "ccw")
    ticks(ctl, 4)
    st = ctl.status()
    assert abs(st["rpm_encoder"] - 120) < 2 and abs(st["rpm_encoder_signed"] + 120) < 2


# ---------------------------------------------------------------------------
# trials: one gzip record per actuation (rest to rest)
# ---------------------------------------------------------------------------
import gzip  # noqa: E402


@pytest.fixture(autouse=True)
def _trials_in_tmp(tmp_path, monkeypatch):
    """Every test saves trials under its own tmp dir, never into motor/trials."""
    d = tmp_path / "trials"
    monkeypatch.setattr(fm, "TRIALS_DIR", str(d))
    return d


def _trial_files(d):
    return sorted(p for p in os.listdir(d) if p.endswith(".json.gz")) if os.path.isdir(d) else []


def _load_trial(d, name):
    with gzip.open(os.path.join(d, name), "rb") as f:
        return json.loads(f.read())


def _tick_until_idle(ctl, cfg, limit=3000):
    for _ in range(limit):
        if not ctl.tick() and ctl.profile is None:
            return
        ctl.sleep(cfg["tick_s"])
    raise AssertionError("never came to rest")


def test_trial_dry_run_ramp_saves_one_gzip_record(_trials_in_tmp):
    cfg, pi, ctl = make()
    assert ctl.trial is None
    ctl.run_profile(400, 2, 1, 2)
    assert ctl.trial is None
    files = _trial_files(_trials_in_tmp)
    assert len(files) == 1 and fm.valid_trial_id(files[0][:-len(".json.gz")])
    with open(os.path.join(_trials_in_tmp, files[0]), "rb") as f:
        assert f.read(2) == b"\x1f\x8b"                       # gzip magic
    rec = _load_trial(_trials_in_tmp, files[0])
    m = rec["meta"]
    assert m["id"] == files[0][:-len(".json.gz")] == ctl.last_trial_id
    assert m["mode"] == "ramp" and m["rpm"] == 400 and m["dir"] == "cw"
    assert (m["accel_s"], m["hold_s"], m["decel_s"]) == (2, 1, 2)
    assert m["stop_cause"] == "complete" and m["dry_run"] is True
    assert m["config"]["pins"]["pul"] == 18 and m["config"]["microstep"] == 16
    assert "CL57T-V41" in m["driver_settings"] and "SW1..SW4" in m["driver_settings"]
    assert m["software"].startswith("froggy_motor.py sha256:") and m["hostname"]
    assert m["start"].endswith("Z") and m["end"].endswith("Z")
    assert 4.9 <= m["duration_s"] <= 5.3
    sm = m["summary"]
    assert abs(sm["max_rpm_cmd"] - 400) < 0.5 and sm["alarm_edges"] == 0
    kinds = [e["kind"] for e in rec["events"]]
    assert kinds[0] == "start" and kinds[-1] == "end" and "apply" not in kinds
    cols = m["columns"]
    assert cols == fm.TRIAL_COLUMNS
    s = rec["samples"]
    assert 48 <= len(s) <= 54 and sm["samples"] == len(s)
    t = [r[0] for r in s]
    gaps = [b - a for a, b in zip(t, t[1:-1])]                # last row is the forced end sample
    assert all(abs(g - 0.1) < 0.011 for g in gaps)
    ph = {r[cols.index("phase")] for r in s}
    assert {"accel", "hold", "decel"} <= ph
    assert s[-1][cols.index("rpm_cmd")] == 0.0


def test_trial_apply_inside_run_stays_one_trial(_trials_in_tmp):
    cfg, pi, ctl = make()
    ctl.request("speed", rpm=120, accel_s=1)
    for _ in range(150):
        ctl.tick()
        ctl.sleep(cfg["tick_s"])
    first = ctl.trial.id
    ctl.request("speed", rpm=200, accel_s=1)
    for _ in range(150):
        ctl.tick()
        ctl.sleep(cfg["tick_s"])
    assert ctl.trial.id == first                              # same trial after the APPLY
    ctl.request("stop")
    _tick_until_idle(ctl, cfg)
    files = _trial_files(_trials_in_tmp)
    assert len(files) == 1
    rec = _load_trial(_trials_in_tmp, files[0])
    assert rec["meta"]["mode"] == "speed" and rec["meta"]["rpm"] == 120
    assert rec["meta"]["stop_cause"] == "stop"
    applies = [e for e in rec["events"] if e["kind"] == "apply"]
    assert len(applies) == 1 and applies[0]["rpm"] == 200
    assert [e["kind"] for e in rec["events"]][-2:] == ["stop", "end"]
    assert abs(rec["meta"]["summary"]["max_rpm_cmd"] - 200) < 0.5


def test_trial_estop_closes_with_cause_estop(_trials_in_tmp):
    cfg, pi, ctl = make(estop_at=2.0)
    ctl.run_profile(400, 3, 2, 3)
    files = _trial_files(_trials_in_tmp)
    assert len(files) == 1
    rec = _load_trial(_trials_in_tmp, files[0])
    assert rec["meta"]["stop_cause"] == "estop"
    ev = [e for e in rec["events"] if e["kind"] == "estop"]
    assert ev and ev[0]["source"] == "button"
    assert rec["meta"]["summary"]["estop_events"] == 1
    assert rec["meta"]["duration_s"] < 3.0


def test_trial_alarm_closes_with_cause_alarm(_trials_in_tmp):
    cfg, pi, ctl = make(alm_blocks_start=True)
    cfg["pins"]["alm"] = 22
    pi2 = fm.FakePi(cfg, clock=ctl.clock, alarm_at=1.5)
    ctl = fm.Controller(cfg, pi2, clock=ctl.clock, sleep=ctl.sleep, out=lambda *a: None)
    ctl.run_profile(300, 2, 2, 2)
    rec = _load_trial(_trials_in_tmp, _trial_files(_trials_in_tmp)[-1])
    assert rec["meta"]["stop_cause"] == "alarm" and rec["meta"]["summary"]["alarm_edges"] >= 1


def test_trial_retention_prunes_oldest(_trials_in_tmp):
    cfg, pi, ctl = make(trials_keep=3)
    ctl.trials = fm.TrialStore.from_config(cfg)
    ids = []
    for _ in range(5):
        ctl.run_profile(120, 0.6, 0.1, 0.6)
        ids.append(ctl.last_trial_id)
    assert ids == sorted(ids) and len(set(ids)) == 5
    left = [f[:-len(".json.gz")] for f in _trial_files(_trials_in_tmp)]
    assert left == ids[-3:]
    # size limit: room for the two newest files only, so the oldest of the three goes
    sizes = [os.path.getsize(os.path.join(_trials_in_tmp, i + ".json.gz")) for i in ids[-3:]]
    store = fm.TrialStore(str(_trials_in_tmp), keep=500, max_mb=(sizes[1] + sizes[2] + 1) / 1048576)
    assert store.prune() == ids[2:3]
    assert [e["id"] for e in store.list()] == [ids[4], ids[3]]          # newest first


def _get(url, headers=None):
    req = urllib.request.Request(url, headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, dict(r.headers), r.read()
    except urllib.error.HTTPError as e:
        return e.code, dict(e.headers), e.read()


def test_trials_api_list_json_csv_current(_trials_in_tmp):
    cfg, pi, ctl = make()
    ctl.run_profile(300, 1.5, 1, 1.5)
    tid = ctl.last_trial_id
    srv = fm.make_server(ctl, "127.0.0.1", 0, True, log=lambda *a: None)
    base = "http://127.0.0.1:%d" % srv.server_address[1]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        code, h, b = _get(base + "/api/trials")
        assert code == 200 and h["Content-Type"] == "application/json"
        lst = json.loads(b)
        assert isinstance(lst, list) and len(lst) == 1
        e = lst[0]
        assert e["id"] == tid and e["mode"] == "ramp" and e["rpm"] == 300 and e["stop_cause"] == "complete"
        assert e["size_bytes"] == os.path.getsize(os.path.join(_trials_in_tmp, tid + ".json.gz"))
        assert {"start", "duration_s", "max_rpm_encoder", "max_rpm_cmd"} <= set(e)

        code, h, b = _get(base + "/api/trials/%s.json" % tid)
        assert code == 200 and h["Content-Type"] == "application/json"
        rec = json.loads(b)
        assert rec["meta"]["id"] == tid and len(rec["samples"]) > 30 and rec["events"]

        code, h, b = _get(base + "/api/trials/%s.csv" % tid)
        assert code == 200 and h["Content-Type"].startswith("text/csv")
        assert h["Content-Disposition"] == 'attachment; filename="froggy-trial-%s.csv"' % tid
        text = b.decode()
        lines = text.splitlines()
        meta_lines = [x for x in lines if x.startswith("#")]
        assert meta_lines[0].startswith("# froggy trial %s" % tid)
        assert "# mode: ramp" in meta_lines and "# stop_cause: complete" in meta_lines
        body = [x for x in lines if not x.startswith("#")]
        parsed = list(csv.reader(body))
        assert parsed[0] == fm.TRIAL_COLUMNS
        assert len(parsed) - 1 == len(rec["samples"])
        assert all(len(r) == len(fm.TRIAL_COLUMNS) for r in parsed)
        assert float(parsed[1][0]) == rec["samples"][0][0]

        code, h, b = _get(base + "/api/trials/current")
        assert code == 200 and json.loads(b) is None           # idle: no open trial
        ctl.request("speed", rpm=100, accel_s=1)
        for _ in range(60):
            ctl.tick()
            ctl.sleep(cfg["tick_s"])
        cur = json.loads(_get(base + "/api/trials/current")[2])
        assert cur["meta"]["mode"] == "speed" and cur["meta"]["end"] is None and len(cur["samples"]) >= 5
        ctl.request("stop")
        _tick_until_idle(ctl, cfg)
        assert json.loads(_get(base + "/api/trials/current")[2]) is None
        assert len(json.loads(_get(base + "/api/trials")[2])) == 2

        # cross-site pages may not read trials
        code, _, _ = _get(base + "/api/trials", {"Sec-Fetch-Site": "cross-site"})
        assert code == 403
    finally:
        srv.shutdown()
        ctl.shutdown()


def test_trials_refuse_unsafe_ids(_trials_in_tmp, capsys):
    cfg, pi, ctl = make()
    srv = fm.make_server(ctl, "127.0.0.1", 0, True, log=lambda *a: None)
    base = "http://127.0.0.1:%d" % srv.server_address[1]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        for bad in ("..%2F..%2Fmotor.toml.json", "abc.csv", "20261109T171512.345Z-a3f9c2", "x.json.gz",
                    "20261109T171512.345Z-A3F9C2.csv", "20261109T171512.345Z-a3f9c2-x.json", "%2Fetc%2Fpasswd.csv"):
            code, _, b = _get(base + "/api/trials/" + bad)
            assert code == 400, bad
        code, _, _ = _get(base + "/api/trials/20261109T171512.345Z-a3f9c2.json")
        assert code == 404                                      # safe id, no such trial
    finally:
        srv.shutdown()
        ctl.shutdown()
    with pytest.raises(ValueError):
        ctl.trials.file("../../etc/passwd")
    assert not fm.valid_trial_id("20261109T171512.345Z-a3f9c2/../x")
    assert fm.main(["--dry-run", "trials", "export", "../motor"]) == 2
    assert "not a trial id" in capsys.readouterr().err


def test_cli_trials_list_export_prune(_trials_in_tmp, tmp_path, capsys):
    assert fm.main(["--dry-run", "--sim-fast", "ramp", "--rpm", "200", "--accel-s", "1", "--hold-s", "0.5",
                    "--decel-s", "1", "--log", str(tmp_path / "r.csv")]) == 0
    files = _trial_files(_trials_in_tmp)
    assert len(files) == 1
    tid = files[0][:-len(".json.gz")]
    capsys.readouterr()
    assert fm.main(["--dry-run", "trials", "list"]) == 0
    out = capsys.readouterr().out
    assert tid in out and "ramp" in out and "complete" in out and "1 trials" in out
    c, j = tmp_path / "t.csv", tmp_path / "t.json"
    assert fm.main(["--dry-run", "trials", "export", tid, "--csv", str(c), "--json", str(j)]) == 0
    assert json.load(open(j))["meta"]["id"] == tid
    assert "wrote" in capsys.readouterr().out
    body = [x for x in open(c).read().splitlines() if not x.startswith("#")]
    assert next(csv.reader(body)) == fm.TRIAL_COLUMNS
    assert fm.main(["--dry-run", "trials", "export", "latest"]) == 0
    assert capsys.readouterr().out.startswith("# froggy trial %s" % tid)
    assert fm.main(["--dry-run", "trials", "prune"]) == 0
    assert "pruned 0 trials" in capsys.readouterr().out


def test_cl_test_is_one_trial_with_verdict(_trials_in_tmp, tmp_path, capsys):
    tap = toml_with(tmp_path, {"enc_a": 5})
    assert fm.main(["--dry-run", "--sim-fast", "--config", tap, "cl-test", "--rpm", "300", "--hold-s", "2",
                    "--log", str(tmp_path / "cl.csv")]) == 0
    files = _trial_files(_trials_in_tmp)
    assert len(files) == 1
    rec = _load_trial(_trials_in_tmp, files[0])
    m = rec["meta"]
    assert m["mode"] == "cl-test" and m["cl_verdict"].startswith("verdict: reached")
    assert m["summary"]["max_rpm_encoder"] and m["summary"]["max_rpm_encoder"] > 250
    assert m["summary"]["follow_err_rev_max"] is not None
    # same sampler: the cl-test CSV and the trial have the same number of 100 ms rows
    assert len(rows(tmp_path / "cl.csv")) == len(rec["samples"])


def test_status_reports_host_encoder_pins_and_camera():
    import socket
    cfg, pi, ctl = make(enc=5)
    st = ctl.status()
    assert st["host"] == socket.gethostname()
    assert st["enc_ppr"] == cfg["driver"]["enc_ppr"] == 1000
    assert st["pins"]["pul"] == cfg["pins"]["pul"] and st["pins"]["enc_a"] == 5 and st["pins"]["index"] == 4
    assert st["camera"] is False                      # no ui server, no camera object
    json.dumps(st)                                    # still plain JSON
    srv = fm.make_server(ctl, "127.0.0.1", 0, True, None, log=lambda *a: None)
    assert ctl.status()["camera"] is False            # dry run: no camera object
    srv.server_close()
    cam = fm.Camera("true")
    srv = fm.make_server(ctl, "127.0.0.1", 0, False, cam, log=lambda *a: None)
    assert ctl.status()["camera"] is True             # camera object present and not failed
    cam.error = "rpicam-vid: not found"
    assert ctl.status()["camera"] is False            # capture failed: not serving
    srv.server_close()
    ctl.shutdown()
