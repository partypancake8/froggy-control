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


def make(estop_at=None, log=None, ena=24, **over):
    cfg = fm.load_config("/nonexistent.toml")
    cfg.update(over)
    cfg["pins"]["ena"] = ena            # most tests model the NPN variant with ENA wired
    clk = fm.FakeClock()
    pi = fm.FakePi(cfg, clock=clk, estop_at=estop_at)
    ctl = fm.Controller(cfg, pi, clock=clk, sleep=clk.sleep, log_path=log, out=lambda *a: None)
    return cfg, pi, ctl


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
    assert list(r[0].keys()) == ["t_s", "rpm_cmd", "step_hz", "rpm_measured", "estop", "phase"]
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


def test_cli_status_and_tests(capsys):
    assert fm.main(["--dry-run", "--sim-fast", "status"]) == 0
    assert fm.main(["--dry-run", "--sim-fast", "estop-test", "--s", "1"]) == 0
    assert fm.main(["--dry-run", "--sim-fast", "index-test", "--s", "3", "--rpm", "60"]) == 0
    out = capsys.readouterr().out
    assert "e-stop: closed" in out and "measured 60.0 rpm" in out


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
        assert "STOP" in html and "camera" in html.lower()
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
    assert cfg2["pins"] == {"pul": 18, "dir": 27, "ena": None, "estop": 25, "index": 17}


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

    def __init__(self, locked=False):
        self.cfg = fm.load_config("/nonexistent.toml")
        self.cfg["pins"]["ena"] = 24
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


def test_page_js_only_ramps_from_a_trusted_click():
    html = fm.UI_HTML
    assert "onclick=\"ramp(event)\"" in html and "if(!ev||!ev.isTrusted)return" in html
    assert html.count("post('/api/ramp'") == 1 and html.count("fetch('/api/arm'") == 1
    assert html.count("armAndRamp(p)") == 3          # its definition, ramp() and the confirm button handler
    assert "confirmedThisLoad=false" in html


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
        assert _post_h(L.base + "/api/enable", {"on": True}, {})[0] == 409
        assert "LOCKED" in urllib.request.urlopen(L.base + "/").read().decode()
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
