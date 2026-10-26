#!/usr/bin/env python3
"""froggy_motor: DM320T stepper test software for the Froggy Turntable (ROB 450 Team 03).

Runs on froggy-pi (Pi 3 B+, Raspberry Pi OS Lite Bookworm) with pigpio, and on the Mac
with a fake pigpio (--dry-run, or automatically when `import pigpio` fails off the Pi).

Step generation: pigpio hardware_PWM on GPIO 18 at 50 percent duty, frequency updated
every 10 ms along a trapezoidal velocity profile. Hardware PWM gives jitter-free pulses
from the BCM PWM block, so the step rate does not depend on Linux scheduling. Wave chains
would give exact per-step acceleration and exact step counts, but a centrifuge cares about
speed, not position, and at 32 kHz a wave chain needs thousands of pulses per 10 ms tick
refilled from Python. Hardware PWM is simpler and has no underrun failure mode.

Commands: ramp, stop, estop-test, index-test, status, ui.  Run with -h for details.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
import traceback
from dataclasses import dataclass, field

HERE = os.path.dirname(os.path.abspath(__file__))
PIDFILE = "/tmp/froggy-motor.pid"
LOCKFILE = "/tmp/froggy-motor.lock"   # one process drives the motor at a time (live only)


def take_motor_lock():
    """Exclusive, non-blocking. Returns (file, None) on success or (None, holder description)."""
    import fcntl
    f = open(LOCKFILE, "a+")
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        f.seek(0)
        holder = f.read().strip() or "unknown process"
        f.close()
        return None, holder
    f.seek(0)
    f.truncate()
    f.write("pid %d: %s" % (os.getpid(), " ".join(os.path.basename(x) if i == 0 else x for i, x in enumerate(sys.argv))))
    f.flush()
    return f, None

# ---------------------------------------------------------------------------
# Config. Values here are the defaults; motor.toml next to this file overrides them.
# ---------------------------------------------------------------------------
CONFIG = {
    "motor": "17HS08-1004",        # or "17HS19-2004"; sets the DIP current note only
    "steps_per_rev": 200,          # 1.8 degree motor
    "microstep": 16,               # DM320T SW4..SW6 setting; 600 RPM -> 32 kHz
    "max_rpm": 600,
    "max_accel_rpm_per_s": 200,    # normal ramps are clamped to this
    "stop_decel_s": 2.0,           # `stop` from the UI: controlled decel time (clamped to max_accel)
    "emergency_decel_s": 0.5,      # e-stop and `stop` command: time to rest from any speed
    "tick_s": 0.010,               # control and log period
    "ena_settle_s": 0.20,          # wait after enabling before the first pulse
    "ena_enable_level": 0,         # GPIO level on ENA that ENABLES the driver (NPN stage: low = enabled)
    "dir_cw_level": 0,             # GPIO level on DIR for "cw"; flip if the drum turns the wrong way
    "estop_open_level": 1,         # NC contact to ground + pull-up: open (pressed) reads 1
    "index_pulses_per_rev": 1,
    "index_stale_revs": 3.0,       # rpm_measured goes NaN after this many expected periods
    # as wired 2026-10-21 (two 2N7000 low-side switches): ENA not wired, driver always enabled.
    # ena = None (or "none" / -1 in motor.toml) means the code never touches an enable pin.
    "pins": {"pul": 18, "dir": 27, "ena": None, "estop": 25, "index": 17},
    "camera_cmd": "rpicam-vid -t 0 -n --codec mjpeg --width 960 --height 540 "
                  "--framerate 20 --rotation 180 -o -",
}

MOTORS = {
    # rated phase current; DM320T peak current is set on SW1..SW3 (see WIRING.md)
    "17HS08-1004": {"rated_a": 1.0, "dip_current": "1.3 A peak / 0.92 A RMS (SW1 ON, SW2 ON, SW3 OFF)"},
    "17HS19-2004": {"rated_a": 2.0, "dip_current": "2.2 A peak / 1.56 A RMS (SW1 OFF, SW2 OFF, SW3 OFF)"},
}

LOG_COLUMNS = ["t_s", "rpm_cmd", "step_hz", "rpm_measured", "estop", "phase"]


def load_config(path=None):
    cfg = json.loads(json.dumps(CONFIG))
    path = path or os.path.join(HERE, "motor.toml")
    if os.path.exists(path):
        import tomllib
        with open(path, "rb") as f:
            over = tomllib.load(f)
        pins = over.pop("pins", {})
        cfg.update(over)
        cfg["pins"].update(pins)
    if cfg["pins"].get("ena") in (None, "", "none", "None", -1, False):
        cfg["pins"]["ena"] = None
    return cfg


def ena_wired(cfg):
    return cfg["pins"].get("ena") is not None


def rpm_to_hz(cfg, rpm):
    return rpm / 60.0 * cfg["steps_per_rev"] * cfg["microstep"]


def on_pi():
    try:
        with open("/proc/device-tree/model") as f:
            return "Raspberry Pi" in f.read()
    except OSError:
        return False


# ---------------------------------------------------------------------------
# Fake pigpio: records what would have happened, simulates the rotor and the hall index.
# ---------------------------------------------------------------------------
class FakeClock:
    """Instant clock for tests and --sim-fast: sleep() advances time."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += max(0.0, s)


class _FakeCB:
    def __init__(self, pi, gpio, edge, func):
        self.pi, self.gpio, self.edge, self.func = pi, gpio, edge, func

    def cancel(self):
        if self in self.pi._cbs:
            self.pi._cbs.remove(self)


class FakePi:
    INPUT, OUTPUT, PUD_UP, PUD_DOWN, PUD_OFF = 0, 1, 2, 1, 0
    RISING_EDGE, FALLING_EDGE, EITHER_EDGE = 0, 1, 2

    def __init__(self, cfg, clock=time.monotonic, estop_at=None):
        self.cfg = cfg
        self.clock = clock
        self.t0 = clock()
        self.connected = True
        self.levels = {}
        self.modes = {}
        self.pulls = {}
        self.freq = 0
        self.pwm_calls = []        # (t, freq)
        self.writes = []           # (t, gpio, level)
        self.steps = 0.0           # pulses actually emitted, integrated over time
        self._last_t = self.t0
        self._cbs = []
        self._estop_at = estop_at
        self._lock = threading.RLock()
        p = cfg["pins"]
        self.levels[p["estop"]] = 1 - cfg["estop_open_level"]  # closed contact
        self.levels[p["index"]] = 1
        self._steps_per_pulse_rev = cfg["steps_per_rev"] * cfg["microstep"] / cfg["index_pulses_per_rev"]
        self._next_index = self._steps_per_pulse_rev * 0.25   # magnet a quarter turn away

    # time and simulation
    def _now(self):
        return self.clock() - self.t0

    def _tick_at(self, t):
        return int(round(t * 1e6)) & 0xFFFFFFFF

    def _advance(self):
        with self._lock:
            self._advance_locked()

    def _advance_locked(self):
        now = self._now()
        if self._estop_at is not None and now >= self._estop_at:
            self._estop_at = None
            self.press_estop(at=now)
        dt = now - (self._last_t - self.t0)
        if dt > 0 and self.freq > 0:
            s0 = self.steps
            self.steps += self.freq * dt
            while self.steps >= self._next_index:
                frac = (self._next_index - s0) / (self.steps - s0)
                t_edge = (self._last_t - self.t0) + frac * dt
                self._next_index += self._steps_per_pulse_rev
                self._fire(self.cfg["pins"]["index"], 0, t_edge)
                self._fire(self.cfg["pins"]["index"], 1, t_edge + 0.001)
        self._last_t = self.t0 + now

    def _fire(self, gpio, level, t):
        self.levels[gpio] = level
        for cb in list(self._cbs):
            if cb.gpio != gpio:
                continue
            if cb.edge == self.EITHER_EDGE or (cb.edge == self.FALLING_EDGE and level == 0) \
                    or (cb.edge == self.RISING_EDGE and level == 1):
                cb.func(gpio, level, self._tick_at(t))

    def press_estop(self, at=None):
        self._fire(self.cfg["pins"]["estop"], self.cfg["estop_open_level"], self._now() if at is None else at)

    def release_estop(self):
        self._fire(self.cfg["pins"]["estop"], 1 - self.cfg["estop_open_level"], self._now())

    # pigpio API subset
    def set_mode(self, g, m):
        self.modes[g] = m

    def set_pull_up_down(self, g, p):
        self.pulls[g] = p

    def set_glitch_filter(self, g, us):
        pass

    def write(self, g, level):
        self._advance()
        self.levels[g] = level
        self.writes.append((self._now(), g, level))

    def read(self, g):
        self._advance()
        return self.levels.get(g, 0)

    def hardware_PWM(self, g, freq, duty):
        self._advance()
        self.freq = int(freq) if duty else 0
        self.pwm_calls.append((self._now(), self.freq))
        return 0

    def get_PWM_frequency(self, g):
        return self.freq

    def get_current_tick(self):
        self._advance()
        return self._tick_at(self._now())

    def get_pigpio_version(self):
        return "fake"

    def callback(self, g, edge, func):
        cb = _FakeCB(self, g, edge, func)
        self._cbs.append(cb)
        return cb

    def stop(self):
        self.connected = False


def connect(cfg, dry_run=False, clock=time.monotonic, estop_at=None):
    """Return (pi, backend_name)."""
    if not dry_run:
        try:
            import pigpio  # noqa
        except ImportError:
            if on_pi():
                sys.exit("pigpio is not installed on this Pi: run deploy.sh (it installs the vendored .debs)")
            dry_run = True
    if dry_run:
        return FakePi(cfg, clock=clock, estop_at=estop_at), "fake pigpio (dry run)"
    import pigpio
    pi = pigpio.pi()
    if not pi.connected:
        sys.exit("pigpiod is not running: sudo systemctl enable --now pigpiod")
    # pigpio keeps its constants on the module; mirror them on the pi object like FakePi has them
    for k in ("INPUT", "OUTPUT", "PUD_UP", "PUD_DOWN", "PUD_OFF", "RISING_EDGE", "FALLING_EDGE", "EITHER_EDGE"):
        setattr(pi, k, getattr(pigpio, k))
    return pi, "pigpio %s" % pi.get_pigpio_version()


# ---------------------------------------------------------------------------
# Velocity profile
# ---------------------------------------------------------------------------
@dataclass
class Profile:
    segs: list = field(default_factory=list)   # (t0, t1, r0, r1, phase)

    def add(self, dur, r0, r1, phase):
        t0 = self.segs[-1][1] if self.segs else self.start
        self.segs.append((t0, t0 + max(0.0, dur), r0, r1, phase))
        return self

    start: float = 0.0

    @property
    def end(self):
        return self.segs[-1][1] if self.segs else self.start

    def at(self, t):
        """(rpm, phase) at absolute time t."""
        for t0, t1, r0, r1, ph in self.segs:
            if t < t1:
                if t < t0:
                    return r0, ph
                return r0 + (r1 - r0) * (t - t0) / (t1 - t0), ph
        return (self.segs[-1][3], "done") if self.segs else (0.0, "done")

    def area_rev(self):
        """Revolutions under the profile (rpm integrated over time / 60)."""
        return sum((t1 - t0) * (r0 + r1) / 2 for t0, t1, r0, r1, _ in self.segs) / 60.0


def plan_ramp(cfg, rpm, accel_s, hold_s, decel_s, start=0.0, r_now=0.0):
    """Trapezoid from r_now to rpm, hold, then to rest. Accel/decel are clamped to max_accel."""
    amax = cfg["max_accel_rpm_per_s"]
    warn = []
    up = abs(rpm - r_now) / max(rpm, 1e-9) * accel_s
    need_up = abs(rpm - r_now) / amax
    if up < need_up - 1e-9:
        warn.append("accel %.2f s too fast, using %.2f s" % (up, need_up))
        up = need_up
    need_dn = rpm / amax
    if decel_s < need_dn - 1e-9:
        warn.append("decel %.2f s too fast, using %.2f s" % (decel_s, need_dn))
        decel_s = need_dn
    p = Profile(start=start)
    p.add(up, r_now, rpm, "accel").add(hold_s, rpm, rpm, "hold").add(decel_s, rpm, 0.0, "decel")
    return p, warn


def plan_decel(start, r_now, dur, phase):
    return Profile(start=start).add(dur if r_now > 0 else 0.0, r_now, 0.0, phase)


# ---------------------------------------------------------------------------
# Hall index
# ---------------------------------------------------------------------------
class Index:
    def __init__(self, cfg, pi, start_tick):
        self.cfg, self.pi, self.start_tick = cfg, pi, start_tick
        self.lock = threading.Lock()
        self.t_us = []          # pulse times, microseconds since controller start

    def on_edge(self, gpio, level, tick):
        with self.lock:
            self.t_us.append((tick - self.start_tick) & 0xFFFFFFFF)
            if len(self.t_us) > 4000:
                del self.t_us[:2000]

    def now_us(self):
        return (self.pi.get_current_tick() - self.start_tick) & 0xFFFFFFFF

    def rpm(self, rpm_cmd=None):
        """Speed from the last full period; NaN when fewer than 2 pulses or the last is stale."""
        with self.lock:
            if len(self.t_us) < 2:
                return math.nan
            a, b = self.t_us[-2], self.t_us[-1]
        per = (b - a) / 1e6
        if per <= 0:
            return math.nan
        since = (self.now_us() - b) / 1e6
        if since > max(self.cfg["index_stale_revs"] * per, 0.2):
            return math.nan
        # a pulse that is late bounds the speed from above, so a slowing drum reads as slowing
        return 60.0 / max(per, since) / self.cfg["index_pulses_per_rev"]

    def rpm_between(self, t0_us, t1_us):
        with self.lock:
            ts = [t for t in self.t_us if t0_us <= t <= t1_us]
        if len(ts) < 2:
            return math.nan
        return 60.0 * (len(ts) - 1) / ((ts[-1] - ts[0]) / 1e6) / self.cfg["index_pulses_per_rev"]

    def count(self):
        with self.lock:
            return len(self.t_us)


# ---------------------------------------------------------------------------
# Controller: one control thread, monotonic time, sleeps between 10 ms ticks.
# ---------------------------------------------------------------------------
class Controller:
    def __init__(self, cfg, pi, clock=time.monotonic, sleep=time.sleep, log_path=None, out=print):
        self.cfg, self.pi, self.clock, self.sleep, self.out = cfg, pi, clock, sleep, out
        self.p = cfg["pins"]
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.requests = []
        self.t0 = clock()
        self.profile = None
        self.direction = "cw"
        self.enabled = False
        self.phase = "idle"
        self.rpm_cmd = 0.0
        self.step_hz = 0.0
        self.applied_hz = 0
        self.total_steps = 0.0
        self.peak_rpm = 0.0
        self.estop_events = 0
        self.estop_latched = False
        self._estop_cb_flag = False
        self._estop_highs = 0
        self.hold_window = None    # (t0_us, t1_us, rpm) of the last hold
        self.last_summary = ""
        self.warnings = []
        self.log_path = log_path
        self.log_dir = None          # UI: each ramp from rest gets its own CSV here
        self._log_f = None
        self._log_w = None
        self._last_apply_t = None
        self._rpm_meas = math.nan
        self.disable_at_rest = False
        self.locked = False          # ui --locked: every ramp and enable is refused
        self.faults = 0
        self._setup()
        self._estop_level = self.estop_open()

    # hardware setup and teardown
    def _setup(self):
        pi, p = self.pi, self.p
        for g in (p["pul"], p["dir"], p["ena"]):
            if g is not None:
                pi.set_mode(g, pi.OUTPUT)
        self._ena(False)
        pi.write(p["dir"], 0)
        pi.hardware_PWM(p["pul"], 0, 0)
        for g in (p["estop"], p["index"]):
            pi.set_mode(g, pi.INPUT)
            pi.set_pull_up_down(g, pi.PUD_UP)
        pi.set_glitch_filter(p["estop"], 1000)
        self.index = Index(self.cfg, pi, pi.get_current_tick())
        self._cbs = [
            pi.callback(p["estop"], pi.EITHER_EDGE, self._on_estop_edge),
            pi.callback(p["index"], pi.FALLING_EDGE, self.index.on_edge),
        ]

    def _ena(self, on):
        if self.p["ena"] is None:          # not wired: the driver is always enabled
            self.enabled = True
            return
        lvl = self.cfg["ena_enable_level"] if on else 1 - self.cfg["ena_enable_level"]
        self.pi.write(self.p["ena"], lvl)
        self.enabled = on

    def _on_estop_edge(self, gpio, level, tick):
        if level == self.cfg["estop_open_level"]:
            self._estop_cb_flag = True
            self.wake.set()

    def estop_open(self):
        return self.pi.read(self.p["estop"]) == self.cfg["estop_open_level"]

    def _safe_off(self):
        try:
            self.pi.hardware_PWM(self.p["pul"], 0, 0)
            self.applied_hz = 0
        finally:
            self.profile = None
            self.rpm_cmd = self.step_hz = 0.0
            self._ena(False)
            self._close_log()
            self.phase = "fault"

    def shutdown(self):
        """PWM off, ENA off, callbacks cancelled. Safe to call more than once."""
        try:
            self.pi.hardware_PWM(self.p["pul"], 0, 0)
            self._ena(False)
            for cb in getattr(self, "_cbs", []):
                cb.cancel()
            self._cbs = []
        finally:
            self._close_log()

    # logging
    def _open_log(self):
        if self.log_path and self._log_f is None:
            d = os.path.dirname(os.path.abspath(self.log_path))
            os.makedirs(d, exist_ok=True)
            self._log_f = open(self.log_path, "w", newline="")
            self._log_w = csv.writer(self._log_f)
            self._log_w.writerow(LOG_COLUMNS)

    def _close_log(self):
        # drop the writer with the file: a stale writer on a closed file crashed the ui on 2026-10-21
        self._log_w = None
        if self._log_f:
            self._log_f.close()
            self._log_f = None

    # requests (thread safe, used by the UI and signal handlers)
    def request(self, kind, **kw):
        with self.lock:
            self.requests.append((kind, kw))
        self.wake.set()

    def t(self):
        return self.clock() - self.t0

    def status(self):
        """Cached values from the control thread; no hardware calls from other threads."""
        rm = self._rpm_meas
        return {
            "phase": self.phase, "rpm_cmd": round(self.rpm_cmd, 2),
            "rpm_measured": None if math.isnan(rm) else round(rm, 2),
            "step_hz": round(self.step_hz, 1), "estop": bool(self._estop_level),
            "estop_latched": self.estop_latched, "enabled": self.enabled, "ena_wired": self.p["ena"] is not None,
            "dir": self.direction, "t_s": round(self.t(), 3), "total_steps": int(self.total_steps),
            "index_pulses": self.index.count(), "max_rpm": self.cfg["max_rpm"],
            "warnings": self.warnings[-3:], "summary": self.last_summary,
            "limits": "max %d rpm, accel up to %d rpm/s, e-stop decel %.2f s, 1/%d step (%.0f Hz at max)" % (
                self.cfg["max_rpm"], self.cfg["max_accel_rpm_per_s"], self.cfg["emergency_decel_s"],
                self.cfg["microstep"], rpm_to_hz(self.cfg, self.cfg["max_rpm"])),
        }

    def _start_ramp(self, rpm, accel_s, hold_s, decel_s, direction="cw"):
        if self.locked:
            raise RuntimeError("server is --locked: ramps are refused")
        if rpm < 0 or rpm > self.cfg["max_rpm"]:
            raise ValueError("rpm %.0f outside 0..%d" % (rpm, self.cfg["max_rpm"]))
        if self.estop_open():
            raise RuntimeError("e-stop is open: release it before ramping")
        moving = self.rpm_cmd > 0.01
        if moving and direction != self.direction:
            raise RuntimeError("direction change while moving: stop first")
        self.estop_latched = False
        self._estop_cb_flag = False
        if not moving:
            self.direction = direction
            cw = self.cfg["dir_cw_level"]
            self.pi.write(self.p["dir"], cw if direction == "cw" else 1 - cw)
            if not self.enabled:
                self._ena(True)
                self.sleep(self.cfg["ena_settle_s"])
            self.peak_rpm = 0.0
            self.total_steps = 0.0
            self.estop_events = 0
            if self.log_dir:
                self._close_log()
                self.log_path = os.path.join(self.log_dir, time.strftime("ui-ramp-%Y%m%d-%H%M%S.csv"))
                self._open_log()
        self.profile, warn = plan_ramp(self.cfg, rpm, accel_s, hold_s, decel_s, start=self.t(), r_now=self.rpm_cmd)
        self.warnings += warn
        for w in warn:
            self.out("warning: " + w)
        self.disable_at_rest = False
        self.hold_window = None

    def _start_decel(self, dur, phase, disable=True):
        t = self.t()
        r = self.rpm_cmd
        if r <= 0.01 and self.profile is None:      # already at rest: nothing to ramp, no log rows
            if disable:
                self._ena(False)
            if phase == "estop":
                self.phase = "estop"
            return
        if phase != "estop":
            dur = max(dur * r / max(self.cfg["max_rpm"], 1), r / self.cfg["max_accel_rpm_per_s"]) \
                if phase == "stopping" else dur
        self.profile = plan_decel(t, r, dur, phase)
        self.disable_at_rest = disable

    def _handle_requests(self):
        with self.lock:
            reqs, self.requests = self.requests, []
        for kind, kw in reqs:
            done = kw.pop("done", None)
            try:
                if kind == "ramp":
                    self._start_ramp(**kw)
                elif kind == "stop":
                    self._start_decel(self.cfg["stop_decel_s"], "stopping")
                elif kind == "estop":
                    self._trigger_estop("software")
                elif kind == "enable":
                    if kw.get("on"):
                        if self.locked:
                            raise RuntimeError("server is --locked: enable is refused")
                        if not self.estop_open():
                            self._ena(True)
                            self.estop_latched = False
                    elif self.rpm_cmd > 0.01:
                        self._start_decel(self.cfg["stop_decel_s"], "stopping")
                    else:
                        self._ena(False)
                if done:
                    done(None)
            except (ValueError, RuntimeError) as e:
                self.warnings.append(str(e))
                if done:
                    done(str(e))
                else:
                    raise

    def _trigger_estop(self, why):
        if not self.estop_latched:
            self.estop_events += 1
            self.out("e-stop (%s) at %.2f s, %.0f rpm: decel to rest in %.2f s"
                     % (why, self.t(), self.rpm_cmd, self.cfg["emergency_decel_s"]))
        self.estop_latched = True
        if self.rpm_cmd > 0.01 or self.profile is not None:
            self._start_decel(self.cfg["emergency_decel_s"], "estop")
        else:
            self._ena(False)
        self.phase = "estop"

    def _check_estop(self):
        lvl_open = self.estop_open()
        self._estop_highs = self._estop_highs + 1 if lvl_open else 0
        fired = self._estop_cb_flag or self._estop_highs >= 2
        self._estop_cb_flag = False
        if fired and not self.estop_latched:
            self._trigger_estop("button")
        self._estop_level = lvl_open
        return lvl_open

    def _apply(self, hz):
        now = self.t()
        if self._last_apply_t is not None:
            self.total_steps += self.applied_hz * (now - self._last_apply_t)
        self._last_apply_t = now
        hz_i = int(round(hz)) if hz >= 1.0 else 0
        if hz_i != self.applied_hz:
            self.pi.hardware_PWM(self.p["pul"], hz_i, 500000 if hz_i else 0)
            self.applied_hz = hz_i

    def tick(self):
        """One control step. Returns True while a profile is active."""
        self._handle_requests()
        lvl_open = self._check_estop()
        dt = self.cfg["tick_s"]
        t = self.t()
        if self.profile is None:
            self._apply(0)
            self.rpm_cmd = 0.0
            self.step_hz = 0.0
            self._rpm_meas = self.index.rpm(0)
            return False
        rpm, ph = self.profile.at(t + dt / 2)       # midpoint of the coming interval
        rpm = min(max(rpm, 0.0), self.cfg["max_rpm"])
        if ph == "done":
            rpm = 0.0
        hz = rpm_to_hz(self.cfg, rpm)
        self._apply(hz)
        if ph == "hold" and self.phase != "hold":
            self._hold_t0 = self.index.now_us()
        if self.phase == "hold" and ph != "hold":
            self.hold_window = (self._hold_t0, self.index.now_us(), self.rpm_cmd)
        self.rpm_cmd, self.step_hz = rpm, hz
        self.peak_rpm = max(self.peak_rpm, rpm)
        self.phase = ph if ph != "done" else ("estop" if self.estop_latched else "idle")
        rm = self.index.rpm(rpm)
        self._rpm_meas = rm
        if self._log_w is not None and self._log_f is not None:
            self._log_w.writerow(["%.3f" % t, "%.2f" % rpm, "%.1f" % hz,
                                  "nan" if math.isnan(rm) else "%.2f" % rm, int(lvl_open), self.phase])
        if ph == "done":
            self.profile = None
            self._apply(0)
            if self.disable_at_rest or self.estop_latched:
                self._ena(False)
            self.phase = "estop" if self.estop_latched else "idle"
            self._summarize()
            if self.log_dir:
                self._close_log()
            return False
        return True

    def _summarize(self):
        s = "summary: peak %.1f rpm, %d steps (%.2f rev), estop events %d" % (
            self.peak_rpm, round(self.total_steps), self.total_steps / (self.cfg["steps_per_rev"] * self.cfg["microstep"]),
            self.estop_events)
        if self.hold_window:
            m = self.index.rpm_between(self.hold_window[0], self.hold_window[1])
            if not math.isnan(m):
                c = self.hold_window[2]
                s += ", hold measured %.1f rpm vs %.1f commanded (%+.2f %%)" % (m, c, (m - c) / c * 100)
            else:
                s += ", hold measured n/a (index saw no pulses)"
        if self.log_path:
            s += ", log %s" % self.log_path
        self.last_summary = s

    def _sleep_until(self, deadline):
        rem = deadline - self.clock()
        if rem > 0:
            if isinstance(self.clock, FakeClock) or self.sleep is not time.sleep:
                self.sleep(rem)
            else:
                self.wake.wait(rem)      # wakes early on e-stop or a request
                self.wake.clear()

    def run_profile(self, rpm, accel_s, hold_s, decel_s, direction="cw"):
        """CLI: run one ramp to rest, logging every tick. Returns the summary line."""
        self._open_log()
        self._start_ramp(rpm, accel_s, hold_s, decel_s, direction)
        self.disable_at_rest = True
        dt = self.cfg["tick_s"]
        nxt = self.clock()
        try:
            while self.tick():
                nxt += dt
                if nxt < self.clock() - dt:     # overran: resync rather than burst
                    nxt = self.clock()
                self._sleep_until(nxt)
            self._ena(False)
        finally:
            self._close_log()
        return self.last_summary

    def serve(self, stop_event):
        """UI: run forever, 10 ms ticks while moving, 50 ms polls while idle."""
        dt = self.cfg["tick_s"]
        nxt = self.clock()
        while not stop_event.is_set():
            try:
                active = self.tick()
            except Exception:
                # never let one bad tick kill the server with the motor in an unknown state
                self.faults += 1
                self.out("fault in control tick (%d), motor brought to rest:\n%s" % (self.faults, traceback.format_exc()))
                self._safe_off()
                active = False
            nxt += dt if active else 0.05
            if nxt < self.clock() - dt:
                nxt = self.clock()
            self._sleep_until(nxt)


# ---------------------------------------------------------------------------
# Camera (Pi only): rpicam-vid MJPEG on stdout, split into frames, served as multipart.
# ---------------------------------------------------------------------------
class Camera:
    def __init__(self, cmd):
        self.cmd = cmd
        self.cond = threading.Condition()
        self.frame = None
        self.seq = 0
        self.proc = None
        self.clients = 0
        self.error = ""

    def _reader(self, proc):
        buf = b""
        while True:
            chunk = proc.stdout.read(65536)
            if not chunk:
                break
            buf += chunk
            while True:
                a = buf.find(b"\xff\xd8")
                b = buf.find(b"\xff\xd9", a + 2)
                if a < 0 or b < 0:
                    if a > 0:
                        buf = buf[a:]
                    break
                with self.cond:
                    self.frame = buf[a:b + 2]
                    self.seq += 1
                    self.cond.notify_all()
                buf = buf[b + 2:]
        with self.cond:
            self.proc = None
            self.cond.notify_all()

    def _ensure(self):
        with self.cond:
            if self.proc is None:
                try:
                    self.proc = subprocess.Popen(self.cmd.split(), stdout=subprocess.PIPE,
                                                 stderr=subprocess.DEVNULL)
                except OSError as e:
                    self.error = str(e)
                    return False
                threading.Thread(target=self._reader, args=(self.proc,), daemon=True).start()
            return True

    def frames(self):
        if not self._ensure():
            return
        self.clients += 1
        last = -1
        try:
            while True:
                with self.cond:
                    self.cond.wait_for(lambda: (self.frame is not None and self.seq != last)
                                       or self.proc is None, timeout=5)
                    if self.seq == last or self.frame is None:
                        return
                    last, fr = self.seq, self.frame
                yield fr
        finally:
            self.clients -= 1
            if self.clients <= 0:
                tm = threading.Timer(10, self._idle_stop)
                tm.daemon = True
                tm.start()

    def _idle_stop(self):
        with self.cond:
            if self.clients <= 0 and self.proc is not None:
                self.proc.terminate()


# ---------------------------------------------------------------------------
# UI server: stdlib http.server + tiny JSON API.
# ---------------------------------------------------------------------------
def ui_log(msg):
    """One line per event to stdout (ui.log on the Pi), with wall-clock time."""
    t = time.time()
    print("%s.%03d %s" % (time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t)), int(t * 1000) % 1000, msg), flush=True)


class RampTokens:
    """One-time tokens: the page asks for one in the Ramp button's click handler, never on load."""
    TTL_S = 30.0

    def __init__(self):
        self.lock = threading.Lock()
        self.live = {}

    def issue(self):
        import secrets
        tok = secrets.token_urlsafe(16)
        with self.lock:
            now = time.monotonic()
            self.live = {k: v for k, v in self.live.items() if v > now}
            self.live[tok] = now + self.TTL_S
        return tok

    def spend(self, tok):
        with self.lock:
            exp = self.live.pop(tok, None) if isinstance(tok, str) else None
        return exp is not None and exp > time.monotonic()


def make_server(ctl, host, port, dry_run, camera=None, label="", require_token=None, log=ui_log):
    """require_token defaults to True for the live motor; the dry run (fake pigpio) accepts plain POSTs."""
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    if require_token is None:
        require_token = not dry_run
    mode = "DRY RUN" if dry_run else ("LIVE, LOCKED" if ctl.locked else "LIVE")
    page = UI_HTML.replace("__MODE__", mode).replace("__LABEL__", label) \
                  .replace("__DRY__", "true" if dry_run else "false")
    tokens = RampTokens()

    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"     # one request per connection: no keep-alive retries of a POST

        def log_message(self, *a):
            pass

        def _src(self):
            h = self.headers
            return "from %s ua=%r origin=%s referer=%s sec-fetch-site=%s sec-fetch-mode=%s" % (
                self.client_address[0], (h.get("User-Agent") or "-")[:80], h.get("Origin") or "-",
                h.get("Referer") or "-", h.get("Sec-Fetch-Site") or "-", h.get("Sec-Fetch-Mode") or "-")

        def _same_origin(self):
            """Reject anything a different site's page could send: cross-site fetch or form posts."""
            site = self.headers.get("Sec-Fetch-Site")
            if site and site not in ("same-origin", "none"):
                return False
            origin = self.headers.get("Origin")
            if origin and origin != "null":
                hostport = origin.split("://", 1)[-1]
                if hostport != self.headers.get("Host"):
                    return False
            return True

        def _json(self, obj, code=200):
            b = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path != "/api/status":           # status polls every 200 ms: not logged
                log("GET %s %s" % (path, self._src()))
            if path == "/":
                b = page.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif path == "/api/status":
                st = ctl.status()
                st["dry_run"] = dry_run
                self._json(st)
            elif path == "/stream.mjpg":
                if camera is None:
                    return self._json({"error": "camera not available in dry run"}, 503)
                serve_mjpeg(self, camera)
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(n) if n else b""
            path = self.path.split("?")[0]
            log("POST %s %s body=%s" % (path, self._src(), raw[:200].decode("utf-8", "replace")))

            def refuse(code, why):
                log("  -> %d refused: %s" % (code, why))
                return self._json({"ok": False, "error": why}, code)

            if not self._same_origin():
                return refuse(403, "cross-origin request")
            if raw and "json" not in (self.headers.get("Content-Type") or ""):
                return refuse(415, "POST bodies must be application/json")
            try:
                body = json.loads(raw or b"{}")
                if not isinstance(body, dict):
                    raise ValueError
            except ValueError:
                return refuse(400, "bad json")
            res = {}
            ev = threading.Event()

            def done(err):
                res["err"] = err
                ev.set()

            if path == "/api/arm":
                if ctl.locked:
                    return refuse(409, "server is --locked: ramps are refused")
                tok = tokens.issue()
                log("  -> 200 one-time ramp token issued (valid %.0f s)" % tokens.TTL_S)
                return self._json({"ok": True, "token": tok, "ttl_s": tokens.TTL_S})
            if path == "/api/ramp":
                if require_token and not tokens.spend(body.get("token")):
                    return refuse(403, "ramp needs a one-time token from the Ramp button (POST /api/arm)")
                try:
                    kw = dict(rpm=float(body.get("rpm", 0)), accel_s=float(body.get("accel_s", 3)),
                              hold_s=float(body.get("hold_s", 5)), decel_s=float(body.get("decel_s", 3)),
                              direction=str(body.get("dir", "cw")))
                except (TypeError, ValueError):
                    return self._json({"ok": False, "error": "bad numbers"}, 400)
                if kw["direction"] not in ("cw", "ccw"):
                    return self._json({"ok": False, "error": "dir must be cw or ccw"}, 400)
                ctl.request("ramp", done=done, **kw)
            elif path == "/api/stop":
                ctl.request("stop", done=done)
            elif path == "/api/estop":
                ctl.request("estop", done=done)
            elif path == "/api/enable":
                ctl.request("enable", on=bool(body.get("on")), done=done)
            else:
                return self._json({"error": "not found"}, 404)
            ev.wait(2.0)        # the control thread picks requests up within one tick
            err = res.get("err")
            if path == "/api/ramp":
                log("  -> RAMP %s: %s" % ("STARTED" if err is None and ev.is_set() else "REFUSED", err or body))
            else:
                log("  -> %s" % ("ok" if err is None and ev.is_set() else "error: %s" % err))
            self._json({"ok": err is None and ev.is_set(), "error": err, "status": ctl.status()},
                       200 if err is None else 409)

    srv = ThreadingHTTPServer((host, port), H)
    srv.daemon_threads = True
    return srv


def serve_mjpeg(h, camera):
    """multipart/x-mixed-replace stream; 503 (so the page shows 'camera offline') if no frame comes."""
    gen = camera.frames()
    first = next(gen, None)
    if first is None:
        b = json.dumps({"error": "camera offline: %s" % (camera.error or "no frames (camera busy? froggy-cam-live --stop)")}).encode()
        h.send_response(503)
        h.send_header("Content-Type", "application/json")
        h.send_header("Content-Length", str(len(b)))
        h.end_headers()
        h.wfile.write(b)
        return
    h.send_response(200)
    h.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
    h.send_header("Cache-Control", "no-store")
    h.end_headers()
    try:
        fr = first
        while fr is not None:
            h.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(fr))
            h.wfile.write(fr + b"\r\n")
            fr = next(gen, None)
    except (BrokenPipeError, ConnectionResetError):
        pass
    finally:
        gen.close()


def make_camera_server(camera, port, host="127.0.0.1"):
    """Plain MJPEG at http://<pi>:8889/stream.mjpg for viewers other than the UI page."""
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class C(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path.split("?")[0] != "/stream.mjpg":
                self.send_error(404)
                return
            serve_mjpeg(self, camera)

    srv = ThreadingHTTPServer((host, port), C)
    srv.daemon_threads = True
    return srv


UI_HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Froggy Motor</title>
<style>
:root{--bg:#0d1117;--panel:#161b22;--line:#30363d;--fg:#e6edf3;--mut:#8b949e;--acc:#58a6ff;--meas:#f0883e;--ok:#3fb950;--bad:#f85149}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:18px/1.35 -apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif}
header{display:flex;align-items:center;gap:14px;padding:12px 20px;border-bottom:1px solid var(--line)}
h1{font-size:24px;margin:0;font-weight:650}.badge{font-size:14px;padding:3px 10px;border-radius:999px;border:1px solid var(--line);color:var(--mut)}
.badge.dry{color:#d29922;border-color:#d29922}.badge.live{color:var(--ok);border-color:var(--ok)}
main{display:grid;grid-template-columns:minmax(0,1.15fr) minmax(0,1fr);gap:14px;padding:14px 20px}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px}
.cam{position:relative;aspect-ratio:16/9;background:#000;border-radius:8px;overflow:hidden;display:flex;align-items:center;justify-content:center}
.cam img{width:100%;height:100%;object-fit:contain;display:block}
.tile{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center;color:var(--mut);font-size:22px;gap:6px;text-align:center;padding:10px}
.tile b{color:var(--fg);font-size:26px}
.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin-bottom:10px}
label{color:var(--mut);font-size:15px}input[type=number]{width:90px;font-size:20px;padding:6px 8px;background:#0d1117;color:var(--fg);border:1px solid var(--line);border-radius:6px}
input[type=range]{flex:1;min-width:160px;accent-color:var(--acc)}
select{font-size:18px;padding:6px;background:#0d1117;color:var(--fg);border:1px solid var(--line);border-radius:6px}
button{font-size:19px;padding:10px 16px;border-radius:8px;border:1px solid var(--line);background:#21262d;color:var(--fg);cursor:pointer}
button:hover{border-color:var(--mut)}button.go{background:#1f6feb;border-color:#1f6feb}
.big{width:100%;margin-top:6px;font-size:34px;font-weight:800;letter-spacing:2px;padding:18px;background:#b62324;border:2px solid var(--bad);color:#fff;border-radius:12px}
.strip{display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:10px;padding:0 20px}
.stat{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:8px 12px}
.stat .k{color:var(--mut);font-size:14px}.stat .v{font-size:28px;font-weight:700;font-variant-numeric:tabular-nums;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.v.ok{color:var(--ok)}.v.bad{color:var(--bad)}
.chartwrap{padding:14px 20px}canvas{width:100%;height:190px;display:block}
.legend{display:flex;gap:18px;color:var(--mut);font-size:15px;margin-bottom:6px}.sw{display:inline-block;width:14px;height:4px;vertical-align:middle;margin-right:6px}
#msg{color:#d29922;font-size:15px;min-height:20px}
.confirm{margin:6px 0 10px;padding:12px;border:2px solid #d29922;border-radius:10px;background:#2b2111;font-size:17px}
.note{margin-top:12px;font-size:15px;color:var(--fg)}.note .k{color:var(--mut);font-size:13px;text-transform:uppercase;letter-spacing:.5px}
@media (max-width:820px){main{grid-template-columns:1fr}.strip{grid-template-columns:repeat(3,minmax(0,1fr))}}
</style></head><body>
<header><h1>Froggy motor</h1><span class="badge" id="mode">__MODE__</span><span class="badge">__LABEL__</span><span id="msg"></span></header>
<main>
 <section class="panel"><div class="cam" id="cam">
   <img id="camimg" alt="camera stream" style="display:none">
   <div class="tile" id="camtile"><b>Camera</b><span id="camtxt">connecting to stream</span></div>
 </div></section>
 <section class="panel">
  <div class="row"><label for="rpm">Target RPM</label><input type="range" id="rpmr" min="0" max="600" step="10" value="60"><input type="number" id="rpm" min="0" max="600" value="60"></div>
  <div class="row"><label>Accel s</label><input type="number" id="acc" value="3" step="0.5" min="0"><label>Hold s</label><input type="number" id="hold" value="5" step="1" min="0"><label>Decel s</label><input type="number" id="dec" value="3" step="0.5" min="0"></div>
  <div class="row"><label>Direction</label><select id="dir"><option value="cw">CW</option><option value="ccw">CCW</option></select>
   <button class="go" onclick="ramp(event)">Ramp</button><button onclick="post('/api/stop')">Stop</button>
   <button onclick="post('/api/enable',{on:true})">Enable</button><button onclick="post('/api/enable',{on:false})">Disable</button></div>
  <div class="confirm" id="confirm" style="display:none"><div id="confirmtxt"></div>
   <div class="row" style="margin:10px 0 0"><button class="go" id="confirmyes">Yes, spin the motor</button><button id="confirmno">Cancel</button></div></div>
  <button class="big" onclick="post('/api/estop')">STOP</button>
  <div class="note"><div class="k">last run</div><div id="summary">no run yet</div></div>
  <div class="note"><div class="k">limits</div><div id="limits">-</div></div>
 </section>
</main>
<div class="strip">
 <div class="stat"><div class="k">phase</div><div class="v" id="s_phase">-</div></div>
 <div class="stat"><div class="k">rpm cmd</div><div class="v" id="s_cmd">-</div></div>
 <div class="stat"><div class="k">rpm measured</div><div class="v" id="s_meas">-</div></div>
 <div class="stat"><div class="k">step Hz</div><div class="v" id="s_hz">-</div></div>
 <div class="stat"><div class="k">e-stop</div><div class="v" id="s_es">-</div></div>
 <div class="stat"><div class="k">elapsed</div><div class="v" id="s_t">-</div></div>
</div>
<div class="chartwrap"><div class="panel"><div class="legend"><span><i class="sw" style="background:var(--acc)"></i>commanded RPM</span><span><i class="sw" style="background:var(--meas)"></i>measured RPM (hall index)</span><span>last 60 s</span></div><canvas id="chart"></canvas></div></div>
<script>
const DRY=__DRY__;const $=id=>document.getElementById(id);
const hist=[];let runStart=null;
$('rpmr').oninput=e=>$('rpm').value=e.target.value;$('rpm').oninput=e=>$('rpmr').value=e.target.value;
function camInit(){const img=$('camimg');
 if(DRY){$('camtxt').innerHTML='dry run on the Mac<br>no Pi camera; the live view streams from froggy-pi';return;}
 img.onload=()=>{img.style.display='block';$('camtile').style.display='none'};
 img.onerror=()=>{img.style.display='none';$('camtile').style.display='flex';$('camtxt').textContent='camera offline (retrying)';setTimeout(()=>{img.src='/stream.mjpg?'+Date.now()},4000)};
 img.src='/stream.mjpg?'+Date.now();}
async function post(u,b){try{const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})});const j=await r.json();$('msg').textContent=j.ok?'':(j.error||'request failed');}catch(e){$('msg').textContent='server not reachable';}}
// Ramps only ever start from a click on Ramp. Nothing on load, on a timer or on a key press posts /api/ramp.
// The first ramp after each page load needs a second click on the confirm panel, and every ramp first
// asks the server for a one-time token (POST /api/arm), which the server requires on the live motor.
let confirmedThisLoad=false;
function rampParams(){return {rpm:+$('rpm').value,accel_s:+$('acc').value,hold_s:+$('hold').value,decel_s:+$('dec').value,dir:$('dir').value};}
function ramp(ev){if(!ev||!ev.isTrusted)return;const p=rampParams();
 if(!confirmedThisLoad){$('confirmtxt').textContent=(DRY?'Dry run: ':'This spins the REAL motor: ')+'ramp to '+p.rpm+' rpm '+p.dir.toUpperCase()+', accel '+p.accel_s+' s, hold '+p.hold_s+' s, decel '+p.decel_s+' s. Area clear?';
  $('confirm').style.display='block';$('confirmyes').onclick=e=>{if(!e.isTrusted)return;confirmedThisLoad=true;$('confirm').style.display='none';armAndRamp(p)};
  $('confirmno').onclick=()=>{$('confirm').style.display='none'};return;}
 armAndRamp(p);}
async function armAndRamp(p){try{const r=await fetch('/api/arm',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({intent:'ramp'})});const a=await r.json();
 if(!a.token){$('msg').textContent=a.error||'no ramp token';return;}p.token=a.token;post('/api/ramp',p);}catch(e){$('msg').textContent='server not reachable';}}
function fmt(x,d){return x==null||isNaN(x)?'-':(+x).toFixed(d)}
async function poll(){try{const r=await fetch('/api/status',{cache:'no-store'});const s=await r.json();
 $('s_phase').textContent=s.phase+(s.enabled?'':' (off)');$('s_cmd').textContent=fmt(s.rpm_cmd,1);$('s_meas').textContent=fmt(s.rpm_measured,1);
 $('s_hz').textContent=fmt(s.step_hz,0);const es=$('s_es');es.textContent=s.estop?'OPEN':'closed';es.className='v '+(s.estop?'bad':'ok');
 if(s.rpm_cmd>0&&runStart==null)runStart=s.t_s;if(s.phase=='idle'&&s.rpm_cmd==0)runStart=null;
 $('s_t').textContent=runStart==null?'-':fmt(s.t_s-runStart,1)+' s';
 $('summary').textContent=s.summary?s.summary.replace(/^summary: /,''):'no run yet';$('limits').textContent=s.limits||'-';
 if(s.warnings&&s.warnings.length&&!$('msg').textContent)$('msg').textContent=s.warnings[s.warnings.length-1];
 hist.push([s.t_s,s.rpm_cmd,s.rpm_measured]);while(hist.length&&hist[0][0]<s.t_s-60)hist.shift();draw(s.t_s,s.max_rpm);
 }catch(e){$('msg').textContent='server not reachable';}}
function draw(now,maxr){const c=$('chart'),dpr=window.devicePixelRatio||1,W=c.clientWidth,Hh=c.clientHeight;c.width=W*dpr;c.height=Hh*dpr;
 const g=c.getContext('2d');g.scale(dpr,dpr);g.clearRect(0,0,W,Hh);const L=44,B=22,T=6,R=8;
 let top=100;for(const h of hist){top=Math.max(top,h[1]||0,h[2]||0)}top=Math.min(Math.ceil(top/100)*100,Math.max(maxr,100));
 const X=t=>L+(W-L-R)*(1-(now-t)/60),Y=v=>T+(Hh-T-B)*(1-v/top);
 g.strokeStyle='#30363d';g.fillStyle='#8b949e';g.font='13px -apple-system,sans-serif';g.lineWidth=1;
 for(let i=0;i<=4;i++){const v=top*i/4,y=Y(v);g.beginPath();g.moveTo(L,y);g.lineTo(W-R,y);g.stroke();g.fillText(v.toFixed(0),4,y+4)}
 for(let s=0;s<=60;s+=10){g.fillText('-'+(60-s)+'s',X(now-60+s)-14,Hh-5)}
 function line(i,col){g.strokeStyle=col;g.lineWidth=2.5;g.beginPath();let on=false;for(const h of hist){const v=h[i];if(v==null||isNaN(v)){on=false;continue}const x=X(h[0]),y=Y(v);if(!on){g.moveTo(x,y);on=true}else g.lineTo(x,y)}g.stroke()}
 line(1,'#58a6ff');line(2,'#f0883e');}
camInit();poll();setInterval(poll,200);
</script></body></html>
"""


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_parser():
    ap = argparse.ArgumentParser(prog="froggy_motor.py", allow_abbrev=False,
                                 description="DM320T stepper test tool for the Froggy Turntable")
    ap.add_argument("--dry-run", action="store_true", help="use the fake pigpio (automatic when pigpio is missing off the Pi)")
    ap.add_argument("--sim-fast", action="store_true", help="dry run only: simulated clock, runs instantly")
    ap.add_argument("--sim-estop-at", type=float, default=None, metavar="S", help="dry run only: press the fake e-stop S seconds in")
    ap.add_argument("--config", default=None, help="motor.toml path (default: next to this file)")
    ap.add_argument("--motor", choices=sorted(MOTORS), default=None, help="which NEMA 17 is fitted")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("ramp", help="ramp up, hold, ramp down to rest")
    r.add_argument("--rpm", type=float, required=True)
    r.add_argument("--accel-s", type=float, required=True)
    r.add_argument("--hold-s", type=float, required=True)
    r.add_argument("--decel-s", type=float, required=True)
    r.add_argument("--dir", choices=["cw", "ccw"], default="cw")
    r.add_argument("--log", default=None, help="CSV log path (default logs/ramp-<time>.csv)")
    sub.add_parser("stop", help="emergency decel of a running ramp (or of the live PWM), then ENA off")
    e = sub.add_parser("estop-test", help="read the e-stop input and watch it for a few seconds")
    e.add_argument("--s", type=float, default=5.0)
    i = sub.add_parser("index-test", help="count hall index pulses and report RPM")
    i.add_argument("--s", type=float, required=True)
    i.add_argument("--rpm", type=float, default=0.0, help="spin at this RPM during the count (0: turn by hand)")
    sub.add_parser("status", help="backend, config, input levels")
    u = sub.add_parser("ui", help="serve the control page and JSON API")
    u.add_argument("--port", type=int, default=7895)
    u.add_argument("--host", default=None, help="bind address (default 127.0.0.1; reach it through the froggy-motor tunnel)")
    u.add_argument("--no-open", action="store_true", help="do not open a browser (dry run on the Mac)")
    u.add_argument("--camera-port", type=int, default=8889)
    u.add_argument("--locked", action="store_true", help="refuse every ramp and enable (testing the page safely)")
    return ap


def _setup_signals(on_signal):
    def h(signum, frame):
        on_signal(signum)
    signal.signal(signal.SIGINT, h)
    signal.signal(signal.SIGTERM, h)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, h)
    if hasattr(signal, "SIGUSR1"):
        signal.signal(signal.SIGUSR1, h)


def main(argv=None):
    a = build_parser().parse_args(argv)
    cfg = load_config(a.config)
    if a.motor:
        cfg["motor"] = a.motor
    if a.cmd == "ramp" and not (0 < a.rpm <= cfg["max_rpm"]):
        print("error: --rpm must be in 0..%d" % cfg["max_rpm"], file=sys.stderr)
        return 2
    fast = a.sim_fast and (a.dry_run or not on_pi())
    clock = FakeClock() if fast else time.monotonic
    sleep = clock.sleep if fast else time.sleep
    pi, backend = connect(cfg, a.dry_run or a.sim_fast, clock=clock, estop_at=a.sim_estop_at)
    dry = isinstance(pi, FakePi)

    if a.cmd == "stop":
        return cmd_stop(cfg, pi, dry, clock, sleep)

    lockf = None
    if not dry:
        lockf, holder = take_motor_lock()
        if lockf is None:
            if a.cmd == "status":
                return cmd_status_readonly(cfg, pi, backend, holder)
            print("error: the motor is in use by %s; stop it first (froggy-motor stop, or Ctrl-C in its window)"
                  % holder, file=sys.stderr)
            return 4

    log = None
    if a.cmd == "ramp":
        log = a.log or os.path.join(HERE, "logs", time.strftime("ramp-%Y%m%d-%H%M%S.csv"))
    ctl = Controller(cfg, pi, clock=clock, sleep=sleep, log_path=log)
    hits = {"n": 0}

    def on_signal(signum):
        hits["n"] += 1
        hits["sig"] = signum
        if hits["n"] > 1 or signum != getattr(signal, "SIGUSR1", -1) and ctl.rpm_cmd <= 0.01:
            ctl.shutdown()
            if a.cmd == "ui":
                ui_log("signal %d: motor at rest, PWM off, %s, exiting"
                       % (signum, "ENA off" if ena_wired(cfg) else "ENA not wired"))
            os._exit(130)
        ctl.request("estop")
        if a.cmd == "ui" and signum != getattr(signal, "SIGUSR1", -1):
            stop_ev.set()

    stop_ev = threading.Event()
    _setup_signals(on_signal)
    try:
        if a.cmd in ("ramp", "ui", "index-test") and not dry:
            _write_pid()
        if a.cmd == "ramp":
            print("ramp %.0f rpm %s, accel %.1f s, hold %.1f s, decel %.1f s, microstep %d, peak step_hz %.0f [%s]"
                  % (a.rpm, a.dir, a.accel_s, a.hold_s, a.decel_s, cfg["microstep"], rpm_to_hz(cfg, a.rpm), backend))
            try:
                print(ctl.run_profile(a.rpm, a.accel_s, a.hold_s, a.decel_s, a.dir))
            except RuntimeError as e:
                print("error: %s" % e, file=sys.stderr)
                return 3
            return 130 if hits["n"] and hits.get("sig") != getattr(signal, "SIGUSR1", -1) else 0
        if a.cmd == "status":
            return cmd_status(cfg, ctl, backend)
        if a.cmd == "estop-test":
            return cmd_estop_test(cfg, ctl, a.s, dry, clock, sleep)
        if a.cmd == "index-test":
            return cmd_index_test(cfg, ctl, a.s, a.rpm, clock, sleep)
        if a.cmd == "ui":
            return cmd_ui(cfg, ctl, a, dry, stop_ev, backend)
    finally:
        ctl.shutdown()
        _clear_pid()
        try:
            pi.stop()
        except Exception:
            pass
    return 0


def _write_pid():
    try:
        with open(PIDFILE, "w") as f:
            f.write(str(os.getpid()))
    except OSError:
        pass


def _clear_pid():
    try:
        with open(PIDFILE) as f:
            if f.read().strip() == str(os.getpid()):
                os.remove(PIDFILE)
    except OSError:
        pass


def _running_ramp_pid():
    """pid of a running `froggy_motor.py ramp`, checked against its command line (stale pidfiles ignored)."""
    try:
        with open(PIDFILE) as f:
            pid = int(f.read().strip())
        cmd = subprocess.run(["ps", "-p", str(pid), "-o", "command="], capture_output=True, text=True).stdout
        return pid if "froggy_motor" in cmd else None
    except (OSError, ValueError):
        return None


def cmd_stop(cfg, pi, dry, clock, sleep):
    pid = _running_ramp_pid()
    if pid:
        os.kill(pid, signal.SIGUSR1)
        print("sent emergency decel to running ramp (pid %d)" % pid)
        return 0
    p = cfg["pins"]
    try:
        hz = pi.get_PWM_frequency(p["pul"]) if not dry else 0
    except Exception:
        hz = 0
    if hz and hz > 0:
        steps = max(2, int(cfg["emergency_decel_s"] / cfg["tick_s"]))
        for k in range(steps, -1, -1):
            pi.hardware_PWM(p["pul"], int(hz * k / steps), 500000 if k else 0)
            sleep(cfg["tick_s"])
    pi.hardware_PWM(p["pul"], 0, 0)
    if ena_wired(cfg):
        pi.set_mode(p["ena"], pi.OUTPUT)
        pi.write(p["ena"], 1 - cfg["ena_enable_level"])
    print("stopped: PWM off, %s (was %d Hz)" % ("ENA off" if ena_wired(cfg) else "ENA not wired, motor holds", hz or 0))
    return 0


def cmd_status_readonly(cfg, pi, backend, holder):
    """Status while another process drives the motor: reads only, never touches an output."""
    p = cfg["pins"]
    print("backend: %s" % backend)
    print("motor in use by %s (read-only status)" % holder)
    print("pins: %s" % json.dumps(p))
    print("e-stop: %s, PUL frequency now %d Hz, %s"
          % ("OPEN (stop)" if pi.read(p["estop"]) == cfg["estop_open_level"] else "closed (run allowed)",
             pi.get_PWM_frequency(p["pul"]) if pi.read(p["pul"]) is not None else 0,
             "ENA: not wired (driver always enabled)" if not ena_wired(cfg) else "ENA level %d" % pi.read(p["ena"])))
    return 0


def cmd_status(cfg, ctl, backend):
    m = MOTORS.get(cfg["motor"], {})
    print("backend: %s" % backend)
    print("motor: %s (%.1f A rated), DIP current %s" % (cfg["motor"], m.get("rated_a", 0), m.get("dip_current", "?")))
    print("microstep %d -> %d pulses/rev; max %d rpm = %.0f Hz; max accel %d rpm/s; e-stop decel %.2f s"
          % (cfg["microstep"], cfg["steps_per_rev"] * cfg["microstep"], cfg["max_rpm"],
             rpm_to_hz(cfg, cfg["max_rpm"]), cfg["max_accel_rpm_per_s"], cfg["emergency_decel_s"]))
    print("pins: %s" % json.dumps(cfg["pins"]))
    print("e-stop: %s, index input level %d, %s"
          % ("OPEN (stop)" if ctl.estop_open() else "closed (run allowed)", ctl.pi.read(cfg["pins"]["index"]),
             ("driver enabled" if ctl.enabled else "driver disabled") if ena_wired(cfg)
             else "ENA: not wired (driver always enabled)"))
    return 0


def cmd_estop_test(cfg, ctl, secs, dry, clock, sleep):
    st = ctl.estop_open()
    print("e-stop now: %s%s" % ("OPEN (stop)" if st else "closed (run allowed)", " [simulated]" if dry else ""))
    print("watching %.0f s: press and release the mushroom button" % secs)
    edges = []
    cb = ctl.pi.callback(cfg["pins"]["estop"], ctl.pi.EITHER_EDGE, lambda g, l, t: edges.append(l))
    t_end = clock() + secs
    last = st
    while clock() < t_end:
        sleep(0.05)
        now = ctl.estop_open()
        if now != last:
            print("  %.2f s: %s" % (secs - (t_end - clock()), "OPEN" if now else "closed"))
            last = now
    cb.cancel()
    print("estop-test: %d edges seen, ends %s" % (len(edges), "OPEN" if last else "closed"))
    return 0


def cmd_index_test(cfg, ctl, secs, rpm, clock, sleep):
    n0 = ctl.index.count()
    t0 = ctl.index.now_us()
    if rpm > 0:
        acc = max(rpm / cfg["max_accel_rpm_per_s"], 1.0)
        print("spinning at %.0f rpm for the count (accel %.1f s)" % (rpm, acc))
        ctl._open_log()
        ctl._start_ramp(rpm, acc, secs, acc)
        ctl.disable_at_rest = True
        dt = cfg["tick_s"]
        nxt = clock()
        while ctl.tick():
            nxt += dt
            ctl._sleep_until(nxt)
        ctl._ena(False)
        m = ctl.index.rpm_between(*ctl.hold_window[:2]) if ctl.hold_window else math.nan
        n = ctl.index.count() - n0
        print("index-test: %d pulses total; at hold measured %s rpm vs %.1f commanded"
              % (n, "n/a" if math.isnan(m) else "%.1f" % m, rpm))
        return 0
    print("counting hall pulses for %.0f s (turn the drum by hand or let it spin)" % secs)
    t_end = clock() + secs
    while clock() < t_end:
        sleep(0.1)
    n = ctl.index.count() - n0
    m = ctl.index.rpm_between(t0, ctl.index.now_us())
    print("index-test: %d pulses in %.1f s, mean %.1f rpm (from pulse count), %s rpm (from periods)"
          % (n, secs, n / secs * 60 / cfg["index_pulses_per_rev"], "n/a" if math.isnan(m) else "%.1f" % m))
    return 0


def cmd_ui(cfg, ctl, a, dry, stop_ev, backend):
    # loopback only: the Mac reaches it through the SSH tunnel, nothing on campus Wi-Fi can
    host = a.host or "127.0.0.1"
    camera = None if dry else Camera(cfg["camera_cmd"])
    label = "%s, %s, 1/%d step" % (backend, cfg["motor"], cfg["microstep"])
    ctl.log_dir = os.path.join(HERE, "logs")
    ctl.out = ui_log
    if a.locked:
        ctl.locked = True
        ctl._ena(False)
        label += ", LOCKED: ramps refused"
    srv = make_server(ctl, host, a.port, dry, camera, label)
    servers = [srv]
    if camera is not None:
        try:
            servers.append(make_camera_server(camera, a.camera_port, host))
        except OSError as e:
            print("camera port %d not served: %s" % (a.camera_port, e))
    for s in servers:
        threading.Thread(target=s.serve_forever, daemon=True).start()
    url = "http://%s:%d/" % ("127.0.0.1" if host in ("0.0.0.0", "127.0.0.1") else host, a.port)
    ui_log("froggy motor ui on %s [%s]%s%s, ramp token %s" % (url, backend, "" if camera is None else
                                              ", camera at :%d/stream.mjpg" % a.camera_port,
                                              ", LOCKED" if a.locked else "", "not required (dry run)" if dry else "required"))
    if dry and not a.no_open and sys.platform == "darwin":
        subprocess.Popen(["open", url])
    try:
        ctl.serve(stop_ev)
        # controlled stop on the way out
        ctl.request("estop")
        t_end = time.monotonic() + cfg["emergency_decel_s"] + 0.5
        while ctl.tick() and time.monotonic() < t_end:
            time.sleep(cfg["tick_s"])
    finally:
        for s in servers:
            s.shutdown()
        ctl.shutdown()
        ui_log("ui exiting: PWM off, %s" % ("ENA off" if ena_wired(cfg) else "ENA not wired (driver holds)"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
