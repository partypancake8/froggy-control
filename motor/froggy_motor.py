#!/usr/bin/env python3
"""froggy_motor: stepper test software for the Froggy Turntable (ROB 450 Team 03).

Driver: STEPPERONLINE CL57T-V41 closed loop (default since 2026-11-09) or the older open-loop
DM320T. Pick it with [driver] model in motor.toml. See WIRING-CL57T.md and TEST-PLAN-CL57T.md.

Runs on the main Pi 3 B+ (Raspberry Pi OS Lite 64-bit) with pigpio, and on a laptop
with a fake pigpio (--dry-run, or automatically when `import pigpio` fails off the Pi).

Step generation: pigpio hardware_PWM on GPIO 18 at 50 percent duty, frequency updated
every 10 ms along a trapezoidal velocity profile. Hardware PWM gives jitter-free pulses
from the BCM PWM block, so the step rate does not depend on Linux scheduling. Wave chains
would give exact per-step acceleration and exact step counts, but a centrifuge cares about
speed, not position, and at 32 kHz a wave chain needs thousands of pulses per 10 ms tick
refilled from Python. Hardware PWM is simpler and has no underrun failure mode.

Closed-loop data the Pi reads (CL57T-V41): the ALM fault output on a GPIO, the BRK output if
wired (raw level only), PEND if a driver has one (the CL57T-V41 does not), and an optional
passive tap on the motor encoder's EA+ line for real shaft speed and following error. The
driver's RS232 tuning port protocol is not published, so nothing here talks to it.

Commands: ramp, speed, cl-test, stop, estop-test, index-test, status, ui.  Run with -h for details.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
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
    "motor": "17HS19-2004D-E1K",   # sets the current note in `status` only
    "steps_per_rev": 200,          # 1.8 degree motor
    "microstep": 16,               # 3200 pulses/rev: CL57T SW1..SW4 or DM320T SW4..SW6; 600 RPM -> 32 kHz
    "max_rpm": 600,
    "max_accel_rpm_per_s": 200,    # normal ramps are clamped to this
    "stop_decel_s": 2.0,          # normal UI Stop, clamped to max acceleration
    "emergency_decel_s": 0.5,      # e-stop and `stop` command: time to rest from any speed
    "tick_s": 0.010,               # control and log period
    "ena_settle_s": 0.20,          # CL57T manual p. 10 (t1): ENA at least 200 ms ahead of DIR
    "ena_enable_level": 0,         # GPIO level on ENA that ENABLES the driver. CL57T direct (Variant A):
                                   # GPIO high lights the ENA opto, which disables the drive (manual p. 12:
                                   # ENA left unconnected = enabled), so low = enabled. Same value as the
                                   # old NPN stage.
    "alm_blocks_start": True,      # True: an active ALM refuses starts and stops a run. False: ALM is only shown and
                                   # logged ("unverified polarity"); set True once the polarity is confirmed.
    "dir_cw_level": 0,             # GPIO level on DIR for "cw"; flip if the drum turns the wrong way
    "estop_open_level": 1,         # NC contact to ground + pull-up: open (pressed) reads 1
    "index_pulses_per_rev": 1,
    "index_stale_revs": 3.0,       # rpm_measured goes NaN after this many expected periods
    # driver outputs read by the Pi. Levels are what the GPIO reads, after the level shift in
    # WIRING-CL57T.md (5 V pull-up + 10 k / 15 k divider, output transistor pulls the pin low).
    "driver": {
        "model": "CL57T-V41",      # or "DM320T"
        "s1": "0",                 # CL57T rotary switch code: 0..3 = 2.8 A peak / 2.0 A RMS
        "loop": "closed",          # SW6 OFF = closed loop, ON = open loop
        "alm_active_level": 0,     # GPIO level that means ALARM. Forum reports say the ALM transistor
                                   # conducts in alarm, which pulls the pin low. Confirm on the bench.
        "pend_active_level": 0,    # only for drivers that have an in-position output
        "enc_ppr": 1000,           # encoder lines per rev; the tap counts EA+ rising edges
        "alm_poll_count": 2,       # consecutive 10 ms polls that confirm ALM without an edge
    },
    # bench wiring from 2026-11-09 (WIRING-CL57T.md, Variant A): PUL, DIR and ENA straight from the
    # GPIOs to PUL+, DIR+, ENA+, the minus terminals to Pi GND. No hall sensor on this bench.
    # None (or "none" / -1 in motor.toml) means not wired: the code never touches that pin.
    # alm: CL57T ALM output. pend: in-position output (the CL57T-V41 has none). brk: CL57T BRK
    # output, logged raw. enc_a: optional passive tap on the motor encoder EA+ line.
    "pins": {"pul": 18, "dir": 17, "ena": 27, "estop": 25, "index": None,
             "alm": 22, "pend": None, "brk": None, "enc_a": None},
    # the bench Pi 3 B+ has no camera: motor.toml sets camera = false, and then the ui starts no
    # rpicam-vid, serves no :8889 stream and hides the preview panel. Nothing else uses the camera.
    "camera": True,
    # trial store (trials/<id>.json.gz, one per actuation): oldest deleted first past either limit
    "trials_keep": 500,
    "trials_max_mb": 200,
    "camera_cmd": "rpicam-vid -t 0 -n --codec mjpeg --width 960 --height 540 "
                  "--framerate 20 --rotation 180 -o -",
}

MOTORS = {
    # rated phase current; DM320T peak current is set on SW1..SW3 (see WIRING.md),
    # CL57T peak current on the S1 rotary switch (see WIRING-CL57T.md)
    "17HS08-1004": {"rated_a": 1.0, "dip_current": "1.3 A peak / 0.92 A RMS (SW1 ON, SW2 ON, SW3 OFF)",
                    "cl57t": "too small for the CL57T: its lowest setting is 2.8 A peak"},
    "17HS19-2004": {"rated_a": 2.0, "dip_current": "2.2 A peak / 1.56 A RMS (SW1 OFF, SW2 OFF, SW3 OFF)",
                    "cl57t": "S1 code 0..3 (2.8 A peak / 2.0 A RMS)"},
    "17HS19-2004D-E1K": {"rated_a": 2.0, "dip_current": "2.2 A peak / 1.56 A RMS (SW1 OFF, SW2 OFF, SW3 OFF)",
                         "cl57t": "S1 code 0..3 (2.8 A peak / 2.0 A RMS)"},
}

# CL57T(V4.0) manual p. 6 (section 5.1): code -> (peak A, RMS A, velocity Ki, position Kp, velocity Kp)
CL57T_S1 = {}
for _i, _c in enumerate("0123456789ABCDEF"):
    _amp = (2.8, 2.0) if _i < 4 else (5.6, 4.0) if _i < 10 else (7.0, 5.0)
    _gain = [(0, 25, 25), (0, 50, 15), (16, 25, 25), (16, 50, 15),
             (0, 25, 25), (0, 50, 15), (0, 100, 5), (16, 25, 25), (16, 50, 15), (16, 100, 5),
             (0, 25, 25), (0, 50, 15), (0, 100, 5), (16, 25, 25), (16, 50, 15), (16, 100, 5)][_i]
    CL57T_S1[_c] = _amp + _gain

# CL57T(V4.0) manual p. 7 (section 5.2.1): microstep -> SW1..SW4 (True = on)
CL57T_MICROSTEP = {
    1: "on on on on", 4: "off on on on", 8: "on off on on", 16: "off off on on",
    32: "on on off on", 64: "off on off on", 128: "on off off on", 256: "off off off on",
    5: "on on on off", 10: "off on on off", 20: "on off on off", 25: "off off on off",
    40: "on on off off", 50: "off on off off", 100: "on off off off", 200: "off off off off",
}

DRIVERS = {
    "DM320T": {"closed_loop": False, "max_pulse_hz": 60000,
               "outputs": "no fault or in-position output"},
    "CL57T-V41": {"closed_loop": True, "max_pulse_hz": 500000,
                  "outputs": "ALM, BRK, COM- on P1; no PEND terminal"},
}

OPTIONAL_PINS = ("ena", "index", "alm", "pend", "brk", "enc_a")

LOG_COLUMNS = ["t_s", "rpm_cmd", "step_hz", "rpm_measured", "rpm_encoder", "alm", "estop", "phase"]

# cl-test writes one row every 100 ms with these columns
CL_COLUMNS = ["t_s", "rpm_cmd", "step_hz", "rpm_hall", "rpm_encoder", "cmd_rev", "enc_rev",
              "follow_err_deg", "alm", "pend", "brk", "estop", "phase"]


def load_config(path=None):
    cfg = json.loads(json.dumps(CONFIG))
    path = path or os.path.join(HERE, "motor.toml")
    if os.path.exists(path):
        import tomllib
        with open(path, "rb") as f:
            over = tomllib.load(f)
        pins = over.pop("pins", {})
        drv = over.pop("driver", {})
        cfg.update(over)
        cfg["pins"].update(pins)
        cfg["driver"].update(drv)
    for k in OPTIONAL_PINS:
        if cfg["pins"].get(k) in (None, "", "none", "None", -1, False):
            cfg["pins"][k] = None
    cfg["driver"]["s1"] = str(cfg["driver"].get("s1", "0")).upper()
    return cfg


def ena_wired(cfg):
    return cfg["pins"].get("ena") is not None


def driver_info(cfg):
    return DRIVERS.get(cfg["driver"]["model"], {"closed_loop": False, "max_pulse_hz": 0, "outputs": "unknown driver"})


def closed_loop(cfg):
    return bool(driver_info(cfg)["closed_loop"]) and cfg["driver"].get("loop", "closed") == "closed"


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
        self.n = 0

    def tally(self):
        return self.n

    def cancel(self):
        if self in self.pi._cbs:
            self.pi._cbs.remove(self)


class FakePi:
    INPUT, OUTPUT, PUD_UP, PUD_DOWN, PUD_OFF = 0, 1, 2, 1, 0
    RISING_EDGE, FALLING_EDGE, EITHER_EDGE = 0, 1, 2

    def __init__(self, cfg, clock=time.monotonic, estop_at=None, stall_rpm=None, alarm_at=None):
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
        self.shaft = 0.0           # where the rotor really is, in step units (lags self.steps on a stall)
        self._last_t = self.t0
        self._cbs = []
        self._estop_at = estop_at
        self._alarm_at = alarm_at
        self._lock = threading.RLock()
        # simulated motor limits: above stall_rpm the rotor loses sync. Open loop (DM320T, or SW6 ON)
        # it just stops and whines with no fault, as on the 2026-10-26 bench. Closed loop, the driver
        # sees the following error grow and raises ALM, which frees the shaft until repower().
        self.stall_rpm = stall_rpm
        self.stalled = False
        self.alarm = False
        self.alarm_cause = ""
        self.follow_alarm_rev = 0.25       # simulated driver threshold, not a manual value
        self._closed = closed_loop(cfg)
        p = cfg["pins"]
        self.levels[p["estop"]] = 1 - cfg["estop_open_level"]  # closed contact
        if p.get("index") is not None:
            self.levels[p["index"]] = 1
        if p.get("alm") is not None:
            self.levels[p["alm"]] = 1 - cfg["driver"]["alm_active_level"]
        if p.get("pend") is not None:
            self.levels[p["pend"]] = 1 - cfg["driver"]["pend_active_level"]
        if p.get("brk") is not None:
            self.levels[p["brk"]] = 1
        if p.get("enc_a") is not None:
            self.levels[p["enc_a"]] = 0
        self._spr = cfg["steps_per_rev"] * cfg["microstep"]
        self._steps_per_pulse_rev = self._spr / cfg["index_pulses_per_rev"]
        self._next_index = self._steps_per_pulse_rev * 0.25   # magnet a quarter turn away
        self._steps_per_enc = self._spr / cfg["driver"]["enc_ppr"]
        self._next_enc = self._steps_per_enc * 0.5

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
        if self._alarm_at is not None and now >= self._alarm_at:
            self._alarm_at = None
            self.raise_alarm("simulated fault", at=now)
        t_prev = self._last_t - self.t0
        dt = now - t_prev
        if dt > 0 and self.freq > 0:
            self.steps += self.freq * dt
            rpm = self.freq / self._spr * 60.0
            if self.stall_rpm is not None and rpm >= self.stall_rpm:
                self.stalled = True
            if not self.stalled and not self.alarm:
                self._turn_shaft(self.freq * dt, t_prev, dt)
            elif self.stalled and self._closed and not self.alarm \
                    and (self.steps - self.shaft) / self._spr > self.follow_alarm_rev:
                self.raise_alarm("position following error", at=now)
        elif self.freq == 0:
            self.stalled = False                   # from rest the rotor pulls back into sync
            self.shaft = self.steps
        self._last_t = self.t0 + now

    def _turn_shaft(self, dsteps, t_prev, dt):
        s0 = self.shaft
        self.shaft += dsteps
        p = self.cfg["pins"]
        while p.get("index") is not None and self.shaft >= self._next_index:
            t_edge = t_prev + (self._next_index - s0) / dsteps * dt
            self._next_index += self._steps_per_pulse_rev
            self._fire(p["index"], 0, t_edge)
            self._fire(p["index"], 1, t_edge + 0.001)
        if p.get("enc_a") is None:
            return
        while self.shaft >= self._next_enc:
            t_edge = t_prev + (self._next_enc - s0) / dsteps * dt
            self._next_enc += self._steps_per_enc
            self._fire(p["enc_a"], 1, t_edge)
            self.levels[p["enc_a"]] = 0

    def raise_alarm(self, cause, at=None):
        """Driver fault: ALM output goes active, the shaft goes free (manual p. 11), latched."""
        self.alarm = True
        self.alarm_cause = cause
        g = self.cfg["pins"].get("alm")
        if g is not None:
            self._fire(g, self.cfg["driver"]["alm_active_level"], self._now() if at is None else at)

    def repower(self):
        """The only reset the manual gives for a CL57T alarm (p. 11)."""
        self.alarm = False
        self.stalled = False
        self.shaft = self.steps
        g = self.cfg["pins"].get("alm")
        if g is not None:
            self._fire(g, 1 - self.cfg["driver"]["alm_active_level"], self._now())

    def _fire(self, gpio, level, t):
        self.levels[gpio] = level
        for cb in list(self._cbs):
            if cb.gpio != gpio:
                continue
            if cb.edge == self.EITHER_EDGE or (cb.edge == self.FALLING_EDGE and level == 0) \
                    or (cb.edge == self.RISING_EDGE and level == 1):
                cb.n += 1
                if cb.func is not None:
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

    def callback(self, g, edge, func=None):
        cb = _FakeCB(self, g, edge, func)
        self._cbs.append(cb)
        return cb

    def stop(self):
        self.connected = False


def connect(cfg, dry_run=False, clock=time.monotonic, estop_at=None, stall_rpm=None, alarm_at=None):
    """Return (pi, backend_name)."""
    if not dry_run:
        try:
            import pigpio  # noqa
        except ImportError:
            if on_pi():
                sys.exit("pigpio is not installed on this Pi: run deploy.sh (it installs the vendored .debs)")
            dry_run = True
    if dry_run:
        return FakePi(cfg, clock=clock, estop_at=estop_at, stall_rpm=stall_rpm,
                      alarm_at=alarm_at), "fake pigpio (dry run)"
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


def plan_speed(cfg, rpm, change_s, start=0.0, r_now=0.0):
    """Change speed smoothly, then hold until another command arrives."""
    duration = max(change_s, abs(rpm - r_now) / cfg["max_accel_rpm_per_s"])
    phase = "accel" if rpm >= r_now else "decel"
    return Profile(start=start).add(duration, r_now, rpm, phase).add(math.inf, rpm, rpm, "hold")


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


class EncoderTap:
    """Counts rising edges on the motor encoder's EA+ line (passive tap, see WIRING-CL57T.md).

    One edge per encoder line, so enc_ppr (1000) edges per shaft rev, 10 kHz at 600 RPM. The edges are
    counted by pigpiod's own callback thread; Python only reads callback.tally() once every WINDOW_S
    (100 ms), so there is no Python work per edge. Speed is the tally change over that interval.
    Direction is not decoded: the run's DIR gives the sign.
    """
    WINDOW_S = 0.1

    def __init__(self, cfg, pi):
        self.cfg, self.pi = cfg, pi
        self.wired = cfg["pins"].get("enc_a") is not None
        self.cb = None
        self.n = 0
        self._t = None
        self._n = 0
        self._rpm = 0.0

    def attach(self, cb):
        self.cb = cb

    def refresh(self):
        if self.cb is not None:
            self.n = self.cb.tally()
        return self.n

    def count(self):
        return self.n

    def sample(self, t):
        """Read the tally if 100 ms passed. True when a new sample was taken."""
        if not self.wired:
            return False
        if self._t is None:
            self._t, self._n = t, self.refresh()
            return True
        if t - self._t < self.WINDOW_S - 1e-9:
            return False
        n = self.refresh()
        self._rpm = (n - self._n) / self.cfg["driver"]["enc_ppr"] / (t - self._t) * 60.0
        self._t, self._n = t, n
        return True

    def rpm(self):
        return self._rpm if self.wired else math.nan


def _num(x, fmt="%.2f"):
    return "nan" if x is None or (isinstance(x, float) and math.isnan(x)) else fmt % x


class ClTest:
    """cl-test: a 100 ms CSV of everything the Pi can see during one ramp, and a verdict line."""

    def __init__(self, ctl, rpm, path):
        self.rpm, self.path = rpm, path
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(CL_COLUMNS)
        self.rows = []
        self.alarm_t = None
        self.alarm_rpm = None
        self.verdict = ""

    def add_row(self, t, st):
        """One row from the controller's 100 ms sample (Controller._sample feeds this and the trial)."""
        row = ["%.3f" % t, st["rpm_cmd"], st["step_hz"], st["rpm_hall"], st["rpm_encoder"], st["cmd_rev"],
               st["enc_rev"], st["follow_err_deg"], st["alm"], st["pend"], st["brk"], st["estop"], st["phase"]]
        self.rows.append(dict(zip(CL_COLUMNS, [t] + row[1:])))
        self.w.writerow([_num(v) if isinstance(v, float) else ("na" if v is None else v) for v in row])
        self.f.flush()

    def note_alarm(self, t, rpm):
        if self.alarm_t is None:
            self.alarm_t, self.alarm_rpm = t, rpm

    def finish(self, ctl):
        self.f.close()
        self.verdict = cl_verdict(self.rows, self.rpm, self.alarm_t, self.alarm_rpm, ctl.estop_events, self.path,
                                  closed=closed_loop(ctl.cfg), enc_no_pulses=ctl.enc_no_pulses_run)
        return self.verdict


def cl_verdict(rows, rpm_target, alarm_t, alarm_rpm, estop_events, path, closed=True, enc_no_pulses=False):
    """One line: reached N RPM, tracking error ..., alarms ... (from the 100 ms rows).

    With no encoder tap (the 2026-11 bench before GPIO 5 was wired) the Pi cannot see the shaft, so the
    line says so instead of pretending: commanded RPM, the ALM state over the run, and what a clean
    closed loop run does and does not prove."""
    def ok(v):
        return v is not None and not (isinstance(v, float) and math.isnan(v))
    # hold rows, minus the first 0.2 s: the 100 ms speed window still straddles the end of the ramp there
    hold_all = [r for r in rows if r["phase"] == "hold"]
    hold = hold_all[2:] if len(hold_all) > 4 else hold_all
    src_key, src = None, "commanded only"
    for key, name in (("rpm_encoder", "encoder tap"), ("rpm_hall", "index")):
        if not enc_no_pulses and any(ok(r[key]) for r in (hold or rows)):
            src_key, src = key, name
            break
    peak_cmd = max([r["rpm_cmd"] for r in rows] or [0.0])
    if src_key is None:
        if enc_no_pulses:
            reached = "reached %.0f RPM commanded (Encoder: no pulses, check EA+ wire)" % peak_cmd
        else:
            reached = "reached %.0f RPM commanded (encoder tap not wired, shaft speed not measured)" % peak_cmd
        track = "tracking error unknown"
    else:
        hold_base = [r[src_key] for r in hold if ok(r[src_key])]
        if hold_base:
            meas = sum(hold_base) / len(hold_base)          # mean over the hold
        else:
            meas = max(r[src_key] for r in rows if ok(r[src_key]))   # no hold reached: best seen
        reached = "reached %.1f RPM (%s, %.0f commanded)" % (meas, src, rpm_target)
        errs = [abs(r[src_key] - r["rpm_cmd"]) / r["rpm_cmd"] * 100 for r in hold
                if ok(r[src_key]) and r["rpm_cmd"] > 1]
        if errs:
            track = "tracking error mean %.2f %% max %.2f %% over %.1f s of hold" % (
                sum(errs) / len(errs), max(errs), len(errs) * 0.1)
        else:
            track = "tracking error n/a (no measured hold)"
        fe = [abs(r["follow_err_deg"]) for r in rows if ok(r["follow_err_deg"])]
        if fe:
            end = [r["follow_err_deg"] for r in (hold or rows) if ok(r["follow_err_deg"])]
            track += ", following error %.3f rev at hold end, max %.3f rev" % (
                (end[-1] if end else fe[-1]) / 360.0, max(fe) / 360.0)
    samples = [r.get("alm") for r in rows if r.get("alm") is not None]
    active = sum(1 for a in samples if a)
    if not samples:
        alm_state = "ALM not wired"
    elif active:
        alm_state = "ALM active in %d of %d samples" % (active, len(samples))
    else:
        alm_state = "ALM ok in all %d samples" % len(samples)
    if alarm_t is not None:
        alarms = "alarms: ALM at %.2f s with %.0f rpm commanded (count the red LED blinks, then repower)" % (
            alarm_t, alarm_rpm)
    else:
        alarms = "alarms none"
    if estop_events:
        alarms += ", e-stop events %d" % estop_events
    note = ""
    if src_key is None and alarm_t is None and samples:
        note = ("; closed loop with no fault 7: the driver followed every pulse it received, but pulses missed "
                "at its 3.3 V inputs would not show" if closed else
                "; open loop: a stall or missed pulses would not show")
    return "verdict: %s, %s, %s (%s)%s; log %s" % (reached, track, alarms, alm_state, note, path)


# ---------------------------------------------------------------------------
# Trials: one record per actuation (rest to rest), saved as trials/<id>.json.gz.
# ---------------------------------------------------------------------------
TRIALS_DIR = os.environ.get("FROGGY_TRIALS_DIR") or os.path.join(HERE, "trials")   # Pi: ~/froggy-motor/trials
TRIAL_FORMAT = "froggy-trial-1"
TRIAL_ID_RE = re.compile(r"^\d{8}T\d{6}\.\d{3}Z-[0-9a-f]{6}$")
TRIAL_SAMPLE_S = 0.1
# one row every 100 ms; values are null when a signal is not wired or not measurable
TRIAL_COLUMNS = ["t_s", "rpm_cmd", "rpm_target", "rpm_encoder", "follow_err_rev", "follow_err_pct",
                 "ena", "alm", "estop", "phase"]
_trial_id_lock = threading.Lock()
_trial_last_ms = [0]
_sw_version = []


def new_trial_id(now=None):
    """UTC timestamp to the millisecond plus 6 random hex digits; strictly increasing within a process."""
    import secrets
    ms = int((time.time() if now is None else now) * 1000)
    with _trial_id_lock:
        ms = max(ms, _trial_last_ms[0] + 1)
        _trial_last_ms[0] = ms
    return "%s.%03dZ-%s" % (time.strftime("%Y%m%dT%H%M%S", time.gmtime(ms // 1000)), ms % 1000, secrets.token_hex(3))


def valid_trial_id(tid):
    return isinstance(tid, str) and bool(TRIAL_ID_RE.match(tid))


def software_version():
    """sha256 of this file, first 12 hex digits (the Pi copy is deployed with rsync, no git there)."""
    if not _sw_version:
        import hashlib
        try:
            with open(os.path.abspath(__file__), "rb") as f:
                _sw_version.append("froggy_motor.py sha256:" + hashlib.sha256(f.read()).hexdigest()[:12])
        except OSError:
            _sw_version.append("unknown")
    return _sw_version[0]


_host = []


def _hostname():
    if not _host:
        import socket
        _host.append(socket.gethostname())
    return _host[0]


def _iso(epoch):
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(epoch)) + ".%03dZ" % (int(epoch * 1000) % 1000)


def _clean(v, nd=4):
    """JSON-safe number: NaN and inf become null, floats are rounded."""
    if v is None or isinstance(v, bool):
        return v
    if isinstance(v, float):
        return None if not math.isfinite(v) else round(v, nd)
    return v


class Trial:
    """The open trial: metadata, events and 100 ms samples. Control thread writes, HTTP threads copy."""

    def __init__(self, ctl, mode, rpm, accel_s, hold_s, decel_s, direction, continuous):
        import socket
        self.lock = threading.Lock()
        self.t0 = ctl.t()
        self.epoch0 = time.time()
        self.id = new_trial_id(self.epoch0)
        cfg = ctl.cfg
        self.meta = {
            "id": self.id, "format": TRIAL_FORMAT, "start": _iso(self.epoch0), "start_epoch": round(self.epoch0, 3),
            "end": None, "end_epoch": None, "duration_s": None,
            "mode": mode, "rpm": _clean(float(rpm), 2), "dir": direction,
            "accel_s": _clean(float(accel_s), 3), "hold_s": None if continuous else _clean(float(hold_s), 3),
            "decel_s": None if continuous else _clean(float(decel_s), 3),
            "config": json.loads(json.dumps(cfg, default=str)),
            "driver_settings": driver_switch_text(cfg), "software": software_version(),
            "hostname": socket.gethostname(), "dry_run": isinstance(ctl.pi, FakePi),
            "stop_cause": None, "cl_verdict": None, "summary": None,
            "sample_period_s": TRIAL_SAMPLE_S, "columns": TRIAL_COLUMNS,
        }
        self.events = []
        self.samples = []
        self.cause = None
        self.event(ctl, "start", mode=mode, rpm=self.meta["rpm"], dir=direction, accel_s=self.meta["accel_s"],
                   hold_s=self.meta["hold_s"], decel_s=self.meta["decel_s"])

    def rel(self, ctl):
        return round(ctl.t() - self.t0, 3)

    def event(self, ctl, kind, **detail):
        ev = {"t_s": self.rel(ctl), "kind": kind}
        ev.update({k: _clean(v, 3) for k, v in detail.items()})
        with self.lock:
            self.events.append(ev)

    CAUSE_RANK = {"complete": 0, "stop": 1, "estop": 2, "exit": 2, "fault": 2, "alarm": 3}

    def set_cause(self, cause):
        """The most severe cause wins (alarm, then estop / exit / fault, then stop); the first among equals."""
        rank = self.CAUSE_RANK.get
        if self.cause is None or rank(cause, 1) > rank(self.cause, 1):
            self.cause = cause

    def add_sample(self, ctl, t, st):
        row = [round(t - self.t0, 3), _clean(st["rpm_cmd"], 2), _clean(float(ctl.target_rpm), 2),
               _clean(st["rpm_encoder"], 2),
               _clean(st["follow_err_deg"] / 360.0 if st["follow_err_deg"] is not None else None, 4),
               _clean(ctl._follow_pct, 2), ctl._ena_level, st["alm"], st["estop"], st["phase"]]
        with self.lock:
            self.samples.append(row)

    def snapshot(self):
        """Copy for /api/trials/current (any thread)."""
        with self.lock:
            return {"meta": dict(self.meta), "events": list(self.events), "samples": list(self.samples)}

    def close(self, ctl, cause, cl_verdict=None):
        self.set_cause(cause)
        dur = round(ctl.t() - self.t0, 3)
        self.event(ctl, "end", cause=self.cause)
        idx = {c: i for i, c in enumerate(TRIAL_COLUMNS)}
        with self.lock:
            s = self.samples
            enc = [r[idx["rpm_encoder"]] for r in s if r[idx["rpm_encoder"]] is not None]
            fe = [abs(r[idx["follow_err_rev"]]) for r in s if r[idx["follow_err_rev"]] is not None]
            self.meta.update({
                "end_epoch": round(self.epoch0 + dur, 3), "end": _iso(self.epoch0 + dur), "duration_s": dur,
                "stop_cause": self.cause, "cl_verdict": cl_verdict,
                "summary": {
                    "max_rpm_cmd": max([r[idx["rpm_cmd"]] or 0.0 for r in s] or [0.0]),
                    "max_rpm_encoder": max(enc) if enc else None,
                    "follow_err_rev_mean": round(sum(fe) / len(fe), 4) if fe else None,
                    "follow_err_rev_max": max(fe) if fe else None,
                    "alarm_edges": sum(1 for e in self.events if e["kind"] in ("alarm", "alm_edge")),
                    "estop_events": sum(1 for e in self.events if e["kind"] == "estop"),
                    "samples": len(s), "duration_s": dur,
                },
            })
            return {"meta": self.meta, "events": self.events, "samples": self.samples}


class TrialStore:
    """trials/<id>.json.gz, gzip level 6; oldest deleted first past trials_keep or trials_max_mb."""

    def __init__(self, path=None, keep=500, max_mb=200.0):
        self.path = path or TRIALS_DIR
        self.keep, self.max_mb = int(keep), float(max_mb)
        self.lock = threading.Lock()
        self._meta = {}          # file name -> (mtime, size, metadata): list calls do not re-read every file

    @classmethod
    def from_config(cls, cfg, path=None):
        return cls(path or TRIALS_DIR, cfg.get("trials_keep", 500), cfg.get("trials_max_mb", 200))

    def file(self, tid):
        if not valid_trial_id(tid):
            raise ValueError("bad trial id %r" % (tid,))
        return os.path.join(self.path, tid + ".json.gz")

    def save(self, rec):
        import gzip
        os.makedirs(self.path, exist_ok=True)
        final = self.file(rec["meta"]["id"])
        tmp = final + ".tmp"
        data = json.dumps(rec, separators=(",", ":"), allow_nan=False).encode()
        with open(tmp, "wb") as raw:
            with gzip.GzipFile(filename="", mode="wb", fileobj=raw, compresslevel=6, mtime=0) as gz:
                gz.write(data)
        os.replace(tmp, final)
        self.prune()
        return final

    def load(self, tid):
        import gzip
        with gzip.open(self.file(tid), "rb") as f:
            return json.loads(f.read())

    def _names(self):
        try:
            names = os.listdir(self.path)
        except OSError:
            return []
        return sorted(n for n in names if n.endswith(".json.gz") and valid_trial_id(n[:-len(".json.gz")]))

    def list(self):
        """Metadata of every saved trial, newest first, without samples, events or the config snapshot."""
        out = []
        with self.lock:
            names = self._names()
            for n in names:
                p = os.path.join(self.path, n)
                try:
                    stt = os.stat(p)
                except OSError:
                    continue
                hit = self._meta.get(n)
                if hit is None or hit[0] != stt.st_mtime or hit[1] != stt.st_size:
                    try:
                        m = self.load(n[:-len(".json.gz")])["meta"]
                    except Exception:
                        continue          # half-written or corrupt file: skip it, never crash a list
                    hit = (stt.st_mtime, stt.st_size, m)
                    self._meta[n] = hit
                out.append(trial_list_entry(hit[2], stt.st_size))
            for n in list(self._meta):
                if n not in names:
                    del self._meta[n]
        out.sort(key=lambda e: e["id"], reverse=True)
        return out

    def prune(self):
        """Delete oldest first while more than trials_keep files or more than trials_max_mb on disk."""
        removed = []
        with self.lock:
            names = self._names()
            sizes = {}
            for n in names:
                try:
                    sizes[n] = os.path.getsize(os.path.join(self.path, n))
                except OSError:
                    sizes[n] = 0
            total = sum(sizes.values())
            limit = self.max_mb * 1024 * 1024
            while names and (len(names) > self.keep or total > limit):
                n = names.pop(0)
                try:
                    os.remove(os.path.join(self.path, n))
                except OSError:
                    pass
                total -= sizes[n]
                self._meta.pop(n, None)
                removed.append(n[:-len(".json.gz")])
        return removed


def trial_list_entry(m, size):
    s = m.get("summary") or {}
    return {"id": m["id"], "start": m.get("start"), "start_epoch": m.get("start_epoch"),
            "duration_s": m.get("duration_s"), "mode": m.get("mode"), "rpm": m.get("rpm"), "dir": m.get("dir"),
            "stop_cause": m.get("stop_cause"), "max_rpm_cmd": s.get("max_rpm_cmd"),
            "max_rpm_encoder": s.get("max_rpm_encoder"), "dry_run": m.get("dry_run"), "size_bytes": size}


def trial_csv(rec):
    """Metadata as '# key: value' lines, then a header row, then one row per 100 ms sample."""
    import io
    m = rec["meta"]
    buf = io.StringIO()
    buf.write("# froggy trial %s (%s)\n" % (m["id"], m.get("format", TRIAL_FORMAT)))
    for k in ("start", "end", "duration_s", "mode", "rpm", "dir", "accel_s", "hold_s", "decel_s", "stop_cause",
              "dry_run", "hostname", "software", "driver_settings", "cl_verdict", "sample_period_s"):
        v = m.get(k)
        buf.write("# %s: %s\n" % (k, "" if v is None else v))
    for k, v in (m.get("summary") or {}).items():
        buf.write("# summary.%s: %s\n" % (k, "" if v is None else v))
    for e in rec.get("events", []):
        buf.write("# event: %s\n" % json.dumps(e, separators=(",", ":")))
    buf.write("# config: %s\n" % json.dumps(m.get("config"), separators=(",", ":"), sort_keys=True))
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(m.get("columns") or TRIAL_COLUMNS)
    for r in rec.get("samples", []):
        w.writerow(["" if v is None else v for v in r])
    return buf.getvalue()


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
        self.last_stop_reason = ""
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
        self.alarm_latched = False   # driver ALM seen; cleared when ALM reads inactive at the next start
        self.alarm_events = 0
        self._alm_cb_flag = False
        self._alm_count = 0
        self._alm_level = None       # cached for status(): True alarm, False ok, None not wired
        self._pend_level = None
        self._brk_level = None
        self._ena_level = None
        self.target_rpm = 0.0
        self.alm_edges = 0           # ALM edges seen during runs while alm_blocks_start is false
        self._alm_noted = False
        self._rpm_enc = math.nan
        self._follow_deg = math.nan
        self._enc0 = 0
        self._fhist = []             # (t, cmd_rev, enc_rev) at each encoder sample, last ~1.2 s
        self._follow_pct = math.nan
        self.enc_no_pulses = False   # live: no edges for more than 1 s above 30 RPM
        self.enc_no_pulses_run = False
        self._nop_t = None
        self._nop_n = 0
        self.cl = None               # ClTest while a cl-test runs
        self.cl_verdict = ""
        self.cl_dir = os.path.join(HERE, "logs")
        self.trial = None            # Trial while a run is active (rest to rest)
        self.trials = TrialStore.from_config(cfg)
        self.last_trial_id = None
        self._next_sample_t = 0.0
        self._trial_mode = None      # set by _start_cltest so the trial opens as "cl-test"
        self._setup()
        self._estop_level = self.estop_open()
        self._read_driver()
        self._ena_idle_guard()

    # hardware setup and teardown
    def _setup(self):
        pi, p = self.pi, self.p
        if p["ena"] is not None:
            # pigpio write() on a pin sets the latch, and the pin becomes an output only after set_mode:
            # latch the DISABLED level first so the pin never drives the enabled level, not even briefly.
            pi.write(p["ena"], 1 - self.cfg["ena_enable_level"])
        for g in (p["pul"], p["dir"], p["ena"]):
            if g is not None:
                pi.set_mode(g, pi.OUTPUT)
        self._ena(False)
        pi.write(p["dir"], 0)
        pi.hardware_PWM(p["pul"], 0, 0)
        for g in (p["estop"], p.get("index")):
            if g is None:
                continue
            pi.set_mode(g, pi.INPUT)
            pi.set_pull_up_down(g, pi.PUD_UP)
        pi.set_glitch_filter(p["estop"], 1000)
        self.index = Index(self.cfg, pi, pi.get_current_tick())
        self.enc = EncoderTap(self.cfg, pi)
        self._cbs = [pi.callback(p["estop"], pi.EITHER_EDGE, self._on_estop_edge)]
        if p.get("index") is not None:            # no hall sensor on the 2026-11 bench
            self._cbs.append(pi.callback(p["index"], pi.FALLING_EDGE, self.index.on_edge))
        # driver outputs: the external 5 V pull-up and divider set the level. The weak internal
        # pull-down only matters when a wire is off: ALM then reads 0 = alarm (fail safe, ramps are
        # refused) and the encoder tap reads a steady 0 instead of counting noise.
        for name in ("alm", "pend", "brk", "enc_a"):
            g = p.get(name)
            if g is None:
                continue
            pi.set_mode(g, pi.INPUT)
            # the encoder drives its own line (EA+ through 10 k): internal pull off; the others keep the pull-down
            pi.set_pull_up_down(g, pi.PUD_OFF if name == "enc_a" else pi.PUD_DOWN)
            if name == "alm":
                pi.set_glitch_filter(g, 1000)
                self._cbs.append(pi.callback(g, pi.EITHER_EDGE, self._on_alm_edge))
            elif name == "enc_a":
                try:
                    pi.set_glitch_filter(g, 3)        # 3 us: far below the 100 us period at 10 kHz
                except Exception:
                    pass
                # no Python function: pigpiod counts the edges, update_encoder reads tally() every 100 ms
                cb = pi.callback(g, pi.RISING_EDGE)
                self.enc.attach(cb)
                self._cbs.append(cb)

    def _ena(self, on):
        if self.p["ena"] is None:          # not wired: the driver is always enabled
            self.enabled = True
            return
        lvl = self.cfg["ena_enable_level"] if on else 1 - self.cfg["ena_enable_level"]
        self.pi.write(self.p["ena"], lvl)
        self.enabled = on
        self._ena_level = self.pi.read(self.p["ena"])      # what the pin really reads back

    def _ena_idle_guard(self):
        """Idle and disabled: the ENA pin must read the disabled level. Re-assert it if anything moved it."""
        if self.p["ena"] is None or self.enabled:
            return
        want = 1 - self.cfg["ena_enable_level"]
        got = self.pi.read(self.p["ena"])
        if got != want:
            self.out("ENA idle level was %s, wanted %d (disabled): rewriting it" % (got, want))
            self.pi.set_mode(self.p["ena"], self.pi.OUTPUT)
            self.pi.write(self.p["ena"], want)
            got = self.pi.read(self.p["ena"])
        self._ena_level = got

    def _on_estop_edge(self, gpio, level, tick):
        if level == self.cfg["estop_open_level"]:
            self._estop_cb_flag = True
            self.wake.set()

    def estop_open(self):
        return self.pi.read(self.p["estop"]) == self.cfg["estop_open_level"]

    # driver outputs (CL57T ALM, BRK; PEND on drivers that have it)
    def _on_alm_edge(self, gpio, level, tick):
        if level == self.cfg["driver"]["alm_active_level"]:
            self._alm_cb_flag = True
            self.wake.set()

    def alm_active(self):
        """True alarm, False ok, None when ALM is not wired."""
        g = self.p.get("alm")
        if g is None:
            return None
        return self.pi.read(g) == self.cfg["driver"]["alm_active_level"]

    def pend_active(self):
        g = self.p.get("pend")
        if g is None:
            return None
        return self.pi.read(g) == self.cfg["driver"]["pend_active_level"]

    def brk_level(self):
        g = self.p.get("brk")
        return None if g is None else self.pi.read(g)

    def _read_driver(self):
        self._alm_level = self.alm_active()
        self._pend_level = self.pend_active()
        self._brk_level = self.brk_level()
        return self._alm_level

    def _check_alm(self):
        """Edge callback or N polls in a row: the driver has faulted. Stop pulsing at once."""
        alm = self._read_driver()
        if alm is None:
            return False
        self._alm_count = self._alm_count + 1 if alm else 0
        fired = (self._alm_cb_flag and alm) or self._alm_count >= self.cfg["driver"]["alm_poll_count"]
        self._alm_cb_flag = False
        if not alm:
            self._alm_noted = False
        if fired and not self.alarm_latched:
            if self.cfg.get("alm_blocks_start", True):
                self._trigger_alarm()
            elif not self._alm_noted:
                self._note_alm_edge()
        return alm

    def _note_alm_edge(self):
        """alm_blocks_start = false: log and show the ALM edge, keep the run going."""
        self._alm_noted = True
        self.alm_edges += 1
        t, r = self.t(), self.rpm_cmd
        self.out("ALM went active at %.2f s, %.0f rpm commanded (unverified polarity: run continues)" % (t, r))
        if self.trial is not None:
            self.trial.event(self, "alm_edge", rpm_cmd=r)
        if self.cl is not None:
            self.cl.note_alarm(t, r)

    def _trigger_alarm(self):
        t, r = self.t(), self.rpm_cmd
        self.alarm_latched = True
        self.alarm_events += 1
        moving = r > 0.01 or self.profile is not None
        self.last_stop_reason = (
            "Driver alarm (ALM) at %.2f s with %.0f rpm commanded: pulses stopped. The CL57T frees the shaft "
            "on a fault. Count the red LED blinks (1 over-current, 2 over-voltage, 4 shaft not locked, "
            "7 position following error), fix the cause, then power-cycle the driver." % (t, r))
        self.out("driver ALM at %.2f s, %.0f rpm commanded%s" % (t, r, ": pulses stopped" if moving else ""))
        if self.trial is not None:
            self.trial.event(self, "alarm", rpm_cmd=r)
            self.trial.set_cause("alarm")
        if self.cl is not None:
            self.cl.note_alarm(t, r)
        self._apply(0)
        if self.phase == "hold":
            self.hold_window = (self._hold_t0, self.index.now_us(), self.rpm_cmd)
        self.profile = None
        self.rpm_cmd = self.step_hz = 0.0
        self._ena(False)
        self.phase = "alarm"
        if moving:
            self._log_row(t, 0.0, 0.0, self.index.rpm(0), int(self._estop_level))
            self._finish_run()

    def _finish_run(self):
        """End of a run (profile done, or stopped by ALM): summary, cl-test verdict, close the CSV."""
        self._summarize()
        self._sample(self.t(), force=True)
        verdict = None
        if self.cl is not None:
            self.cl_verdict = verdict = self.cl.finish(self)
            if self.log_dir:                  # UI: the verdict goes to ui.log; the CLI prints it itself
                self.out(self.cl_verdict)
            self.cl = None
        self._close_trial("complete", verdict)
        if self.log_dir:
            self._close_log()

    # trials: one per actuation from rest to rest
    def _sample(self, t, force=False):
        """The one 100 ms sampler: a row for the open trial and, during a cl-test, the cl-test CSV."""
        if self.trial is None and self.cl is None:
            return
        if not force:
            if t + 1e-9 < self._next_sample_t:
                return
            self._next_sample_t = max(self._next_sample_t + TRIAL_SAMPLE_S, t + TRIAL_SAMPLE_S / 2)
        st = self.cl_snapshot()
        if self.trial is not None:
            self.trial.add_sample(self, t, st)
        if self.cl is not None:
            self.cl.add_row(t, st)

    def _trial_event(self, kind, cause=None, **detail):
        if self.trial is not None:
            self.trial.event(self, kind, **detail)
            if cause:
                self.trial.set_cause(cause)

    def _close_trial(self, cause, verdict=None):
        """Save the open trial (never raises: a full disk must not stop the motor code)."""
        tr, self.trial = self.trial, None
        if tr is None:
            return None
        rec = tr.close(self, cause, verdict)
        self.last_trial_id = tr.id
        try:
            path = self.trials.save(rec)
        except Exception as e:
            self.out("trial %s not saved: %s" % (tr.id, e))
            return None
        self.out("trial %s saved: %s, %s, %.1f s, %d samples (%s)" % (
            tr.id, rec["meta"]["mode"], rec["meta"]["stop_cause"], rec["meta"]["duration_s"] or 0,
            len(rec["samples"]), path))
        return path

    def current_trial(self):
        tr = self.trial
        return None if tr is None else tr.snapshot()

    def cl_snapshot(self):
        """Values for one cl-test row (control thread only)."""
        spr = self.cfg["steps_per_rev"] * self.cfg["microstep"]
        enc_rev = ((self.enc.count() - self._enc0) / self.cfg["driver"]["enc_ppr"]
                   if self.enc.wired else math.nan)
        return {"rpm_cmd": round(self.rpm_cmd, 2), "step_hz": round(self.step_hz, 1),
                "rpm_hall": self._rpm_meas, "rpm_encoder": self._rpm_enc,
                "cmd_rev": round(self.total_steps / spr, 4), "enc_rev": enc_rev,
                "follow_err_deg": self._follow_deg,
                "alm": None if self._alm_level is None else int(self._alm_level),
                "pend": None if self._pend_level is None else int(self._pend_level),
                "brk": self._brk_level, "estop": int(bool(self._estop_level)), "phase": self.phase}

    def _update_encoder(self, t):
        new = self.enc.sample(t)
        self._rpm_enc = self.enc.rpm()
        if not self.enc.wired:
            self._follow_deg = self._follow_pct = math.nan
            return
        if new:
            # commanded revs since the run start, extended to now at the applied rate, minus encoder revs
            spr = self.cfg["steps_per_rev"] * self.cfg["microstep"]
            since = t - self._last_apply_t if self._last_apply_t is not None else 0.0
            cmd = (self.total_steps + self.applied_hz * max(since, 0.0)) / spr
            got = (self.enc.count() - self._enc0) / self.cfg["driver"]["enc_ppr"]
            self._follow_deg = (cmd - got) * 360.0
            self._fhist.append((t, cmd, got))
            while len(self._fhist) > 2 and self._fhist[1][0] <= t - 1.0:
                self._fhist.pop(0)
            t0, c0, g0 = self._fhist[0]
            dc = cmd - c0
            self._follow_pct = ((dc - (got - g0)) / dc * 100.0
                                if t - t0 >= 0.5 and dc > 0.02 else math.nan)
        # no pulses: above 30 RPM commanded and zero new edges for more than 1 s (the run is not stopped)
        if self.profile is not None and self.rpm_cmd > 30.0:
            if self._nop_t is None or self.enc.n != self._nop_n:
                self._nop_t, self._nop_n = t, self.enc.n
                self.enc_no_pulses = False
            elif t - self._nop_t > 1.0:
                self.enc_no_pulses = self.enc_no_pulses_run = True
        else:
            self._nop_t = None
            self.enc_no_pulses = False

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
            self._trial_event("fault", cause="fault")
            self._close_trial("fault")

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
            self._close_trial("exit")

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
            "last_stop_reason": self.last_stop_reason,
            "dir": self.direction, "t_s": round(self.t(), 3), "total_steps": int(self.total_steps),
            "index_pulses": self.index.count(), "max_rpm": self.cfg["max_rpm"],
            "warnings": self.warnings[-3:], "summary": self.last_summary,
            "driver": self.cfg["driver"]["model"], "closed_loop": closed_loop(self.cfg),
            "ena_level": self._ena_level, "ena_enable_level": self.cfg["ena_enable_level"],
            "target_rpm": self.target_rpm, "alm_blocks_start": bool(self.cfg.get("alm_blocks_start", True)),
            "alm_edges": self.alm_edges,
            "alm": self._alm_level, "alm_wired": self.p.get("alm") is not None,
            "alarm_latched": self.alarm_latched, "alarm_events": self.alarm_events,
            "pend": self._pend_level, "pend_wired": self.p.get("pend") is not None,
            "brk": self._brk_level, "enc_wired": self.enc.wired, "enc_edges": self.enc.count(),
            "rpm_encoder": None if math.isnan(self._rpm_enc) else round(self._rpm_enc, 2),
            "rpm_encoder_signed": None if math.isnan(self._rpm_enc) else round(
                self._rpm_enc * (1 if self.direction == "cw" else -1), 2),
            "follow_err_rev": None if math.isnan(self._follow_deg) else round(self._follow_deg / 360.0, 4),
            "follow_err_pct": None if math.isnan(self._follow_pct) else round(self._follow_pct, 2),
            "enc_no_pulses": self.enc_no_pulses,
            "follow_err_deg": None if math.isnan(self._follow_deg) else round(self._follow_deg, 1),
            "cl_running": self.cl is not None, "cl_verdict": self.cl_verdict,
            "trial_id": None if self.trial is None else self.trial.id, "last_trial_id": self.last_trial_id,
            "host": _hostname(), "enc_ppr": self.cfg["driver"]["enc_ppr"], "pins": dict(self.p),
            # true only when the ui server holds a Camera whose capture has not failed (dry run and camera = false: None)
            "camera": bool(getattr(self, "camera", None) is not None and not getattr(self.camera, "error", "")),
            "limits": "max %d rpm, accel up to %d rpm/s, e-stop decel %.2f s, 1/%d step (%.0f Hz at max)" % (
                self.cfg["max_rpm"], self.cfg["max_accel_rpm_per_s"], self.cfg["emergency_decel_s"],
                self.cfg["microstep"], rpm_to_hz(self.cfg, self.cfg["max_rpm"])),
        }

    def _start_ramp(self, rpm, accel_s, hold_s, decel_s, direction="cw", continuous=False):
        if self.locked:
            raise RuntimeError("server is --locked: ramps are refused")
        if not all(math.isfinite(v) and v >= 0 for v in (rpm, accel_s, hold_s, decel_s)):
            raise ValueError("speed and times must be finite, nonnegative numbers")
        if rpm < 0 or rpm > self.cfg["max_rpm"]:
            raise ValueError("rpm %.0f outside 0..%d" % (rpm, self.cfg["max_rpm"]))
        if self.estop_open():
            raise RuntimeError("e-stop is open: release it before ramping")
        if self.alm_active() and self.cfg.get("alm_blocks_start", True):
            self.alarm_latched = True
            raise RuntimeError("driver ALM is active: count the red LED blinks, fix the cause and power-cycle "
                               "the driver. If the red LED is off, alm_active_level in motor.toml is inverted.")
        moving = self.rpm_cmd > 0.01
        if moving and direction != self.direction:
            raise RuntimeError("direction change while moving: stop first")
        self.estop_latched = False
        self._estop_cb_flag = False
        self.alarm_latched = False
        self._alm_count = 0
        self._alm_noted = False
        self.last_stop_reason = ""
        self.target_rpm = rpm
        if not moving:
            self.direction = direction
            # manual p. 10: ENA at least 200 ms ahead of DIR, DIR at least 2 us ahead of PUL
            if not self.enabled:
                self._ena(True)
                self.sleep(self.cfg["ena_settle_s"])
            cw = self.cfg["dir_cw_level"]
            self.pi.write(self.p["dir"], cw if direction == "cw" else 1 - cw)
            self.peak_rpm = 0.0
            self.total_steps = 0.0
            self.estop_events = 0
            self._enc0 = self.enc.refresh()
            self._fhist = []
            self._follow_pct = math.nan
            self._follow_deg = 0.0 if self.enc.wired else math.nan
            self.enc_no_pulses = self.enc_no_pulses_run = False
            self._nop_t = None
            if self.log_dir:
                self._close_log()
                self.log_path = os.path.join(self.log_dir, time.strftime("ui-ramp-%Y%m%d-%H%M%S.csv"))
                self._open_log()
        if continuous:
            self.profile = plan_speed(self.cfg, rpm, accel_s, start=self.t(), r_now=self.rpm_cmd)
            warn = []
        else:
            self.profile, warn = plan_ramp(self.cfg, rpm, accel_s, hold_s, decel_s, start=self.t(), r_now=self.rpm_cmd)
        self.warnings += warn
        for w in warn:
            self.out("warning: " + w)
        mode, self._trial_mode = self._trial_mode or ("speed" if continuous else "ramp"), None
        if self.trial is None:
            self.trial = Trial(self, mode, rpm, accel_s, hold_s, decel_s, direction, continuous)
            self._next_sample_t = self.t()
        else:
            self.trial.event(self, "apply", mode=mode, rpm=rpm, accel_s=accel_s,
                             hold_s=None if continuous else hold_s, decel_s=None if continuous else decel_s)
        self.disable_at_rest = True     # every run ends with ENA back at the disabled level (was False: the UI left it enabled)
        self.hold_window = None

    def _set_speed(self, rpm, accel_s=3.0, direction="cw"):
        if not math.isfinite(rpm) or not math.isfinite(accel_s) or rpm < 0 or accel_s < 0:
            raise ValueError("speed and change time must be finite, nonnegative numbers")
        if rpm == 0:
            self.last_stop_reason = "Target speed set to zero."
            self._trial_event("stop", cause="stop", source="speed zero")
            duration = max(accel_s, self.rpm_cmd / self.cfg["max_accel_rpm_per_s"])
            self._start_decel(duration, "decel")
        else:
            self._start_ramp(rpm, accel_s, 0, 0, direction, continuous=True)

    def _start_cltest(self, rpm, accel_s, hold_s, decel_s, direction="cw", path=None):
        """Ramp with a 100 ms closed-loop log and a verdict at the end (CLI cl-test and the UI tile)."""
        if self.rpm_cmd > 0.01 or self.profile is not None:
            raise RuntimeError("cl-test starts from rest: stop first")
        path = path or os.path.join(self.cl_dir, "cl-test-%drpm-%s.csv" % (round(rpm), time.strftime("%Y%m%d-%H%M%S")))
        self._trial_mode = "cl-test"
        try:
            self._start_ramp(rpm, accel_s, hold_s, decel_s, direction)
        finally:
            self._trial_mode = None
        self.cl = ClTest(self, rpm, path)
        self.cl_verdict = ""
        self.out("cl-test %.0f rpm %s: 100 ms log %s" % (rpm, direction, path))

    def _start_decel(self, dur, phase, disable=True):
        t = self.t()
        r = self.rpm_cmd
        if r <= 0.01 and self.profile is None:      # already at rest: nothing to ramp, no log rows
            if disable:
                self._ena(False)
            if phase == "estop":
                self.phase = "estop"
            return
        self.target_rpm = 0.0
        if phase != "estop":
            if phase == "stopping":
                dur = max(dur * r / max(self.cfg["max_rpm"], 1), r / self.cfg["max_accel_rpm_per_s"])
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
                elif kind == "cltest":
                    self._start_cltest(**kw)
                elif kind == "speed":
                    self._set_speed(**kw)
                elif kind == "stop":
                    self.last_stop_reason = "Stop requested from the controls."
                    self._trial_event("stop", cause="stop", source="controls")
                    self._start_decel(self.cfg["stop_decel_s"], "stopping")
                elif kind == "estop":
                    self._trigger_estop(kw.get("why", "software"))
                elif kind == "enable":
                    if kw.get("on"):
                        if self.locked:
                            raise RuntimeError("server is --locked: enable is refused")
                        if not self.estop_open():
                            self._ena(True)
                            self.estop_latched = False
                    elif self.rpm_cmd > 0.01:
                        self._trial_event("stop", cause="stop", source="disable")
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
        self.last_stop_reason = ("Emergency stop requested by software (UI or command)." if why == "software"
                                 else "Physical e-stop input opened." if why == "button"
                                 else "Emergency stop: %s." % why)
        if not self.estop_latched:
            self.estop_events += 1
            self.out("e-stop (%s) at %.2f s, %.0f rpm: decel to rest in %.2f s"
                     % (why, self.t(), self.rpm_cmd, self.cfg["emergency_decel_s"]))
        self.estop_latched = True
        self._trial_event("estop", cause="estop", source=why, rpm_cmd=self.rpm_cmd)
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

    def _log_row(self, t, rpm, hz, rm, lvl_open):
        if self._log_w is not None and self._log_f is not None:
            self._log_w.writerow(["%.3f" % t, "%.2f" % rpm, "%.1f" % hz, _num(rm), _num(self._rpm_enc),
                                  "na" if self._alm_level is None else int(self._alm_level),
                                  int(lvl_open), self.phase])

    def tick(self):
        """One control step. Returns True while a profile is active."""
        self._handle_requests()
        lvl_open = self._check_estop()
        t = self.t()
        self._update_encoder(t)
        self._check_alm()
        dt = self.cfg["tick_s"]
        if self.profile is None:
            self._ena_idle_guard()
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
        self._log_row(t, rpm, hz, rm, lvl_open)
        self._sample(t)
        if ph == "done":
            self.profile = None
            self._apply(0)
            if self.disable_at_rest or self.estop_latched:
                self._ena(False)
            self.phase = "estop" if self.estop_latched else "idle"
            self._finish_run()
            return False
        return True

    def _summarize(self):
        s = "summary: peak %.1f rpm, %d steps (%.2f rev), estop events %d" % (
            self.peak_rpm, round(self.total_steps), self.total_steps / (self.cfg["steps_per_rev"] * self.cfg["microstep"]),
            self.estop_events)
        if self.alarm_latched:
            s += ", DRIVER ALARM (ALM) stopped the run"
        elif self._alm_noted:
            s += ", ALM went active (unverified polarity, run was not stopped)"
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

    def run_profile(self, rpm, accel_s, hold_s, decel_s, direction="cw", continuous=False):
        """CLI: run one ramp to rest, logging every tick. Returns the summary line."""
        self._open_log()
        self._start_ramp(rpm, accel_s, hold_s, decel_s, direction, continuous=continuous)
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


UI_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui")
UI_TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
            ".css": "text/css; charset=utf-8", ".txt": "text/plain; charset=utf-8",
            ".png": "image/png", ".svg": "image/svg+xml"}


def make_server(ctl, host, port, dry_run, camera=None, label="", require_token=None, log=ui_log, show_camera=True):
    """require_token defaults to True for the live motor; the dry run (fake pigpio) accepts plain POSTs."""
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    ctl.camera = camera              # read by Controller.status() for the "camera" field
    if require_token is None:
        require_token = not dry_run
    mode = "DRY RUN" if dry_run else ("LIVE, LOCKED" if ctl.locked else "LIVE")
    page = UI_HTML.replace("__MODE__", mode).replace("__LABEL__", label) \
                  .replace("__DRY__", "true" if dry_run else "false") \
                  .replace("__CAM__", "true" if show_camera else "false")
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
            rel = "index.html" if path == "/" else path.lstrip("/")
            full = os.path.realpath(os.path.join(UI_DIR, rel))
            ctype = UI_TYPES.get(os.path.splitext(full)[1])
            if not path.startswith("/api/") and path != "/stream.mjpg" and ctype \
                    and full.startswith(os.path.realpath(UI_DIR) + os.sep) and os.path.isfile(full):
                with open(full, "rb") as f:
                    b = f.read()
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif path == "/":                      # ui/ missing: fall back to the inline page
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
            elif path == "/api/trials" or path.startswith("/api/trials/"):
                self._trials(path)
            elif path == "/stream.mjpg":
                if camera is None:
                    return self._json({"error": "no camera on this controller (dry run, or camera = false)"}, 503)
                serve_mjpeg(self, camera)
            else:
                self._json({"error": "not found"}, 404)

        def _trials(self, path):
            """GET /api/trials, /api/trials/current, /api/trials/<id>.json, /api/trials/<id>.csv"""
            if not self._same_origin():
                return self._json({"error": "cross-origin request"}, 403)
            store = ctl.trials
            if path == "/api/trials":
                return self._json(store.list())
            name = path[len("/api/trials/"):]
            if name == "current":
                return self._json(ctl.current_trial())
            tid, ext = (name[:-5], "json") if name.endswith(".json") else \
                       (name[:-4], "csv") if name.endswith(".csv") else (name, None)
            if ext is None or not valid_trial_id(tid):
                return self._json({"error": "bad trial id (expected like 20261109T171512.345Z-a3f9c2.json or .csv)"}, 400)
            try:
                rec = store.load(tid)
            except OSError:
                return self._json({"error": "no trial %s" % tid}, 404)
            if ext == "json":
                return self._json(rec)
            b = trial_csv(rec).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", 'attachment; filename="froggy-trial-%s.csv"' % tid)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

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
            if path in ("/api/ramp", "/api/speed", "/api/cltest"):
                if require_token and not tokens.spend(body.get("token")):
                    return refuse(403, "motor command needs a one-time token from the start controls (POST /api/arm)")
                try:
                    kw = dict(rpm=float(body.get("rpm", 0)),
                              accel_s=float(body.get("accel_s", 3)),
                              hold_s=float(body.get("hold_s", 5)),
                              decel_s=float(body.get("decel_s", 3)),
                              direction=str(body.get("dir", "cw")))
                except (TypeError, ValueError):
                    return self._json({"ok": False, "error": "bad numbers"}, 400)
                if kw["direction"] not in ("cw", "ccw"):
                    return self._json({"ok": False, "error": "dir must be cw or ccw"}, 400)
                if path == "/api/speed":
                    kw.pop("hold_s")
                    kw.pop("decel_s")
                kind = {"/api/speed": "speed", "/api/ramp": "ramp", "/api/cltest": "cltest"}[path]
                ctl.request(kind, done=done, **kw)
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
            if path in ("/api/ramp", "/api/speed", "/api/cltest"):
                log("  -> %s %s: %s" % ({"/api/speed": "SPEED", "/api/ramp": "RAMP", "/api/cltest": "CL-TEST"}[path],
                    "STARTED" if err is None and ev.is_set() else "REFUSED", err or body))
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
<title>Froggy · Motor control</title>
<style>
:root{color-scheme:dark;--bg:#0b1017;--panel:#131b26;--line:#2c3848;--fg:#e7edf6;--mut:#a1adbe;--acc:#79a7ff;--meas:#edb365;--ok:#66d6a1;--bad:#f17682}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}button,input,select{font:inherit}button,input,select,summary{-webkit-tap-highlight-color:transparent}button:focus-visible,input:focus-visible,select:focus-visible,summary:focus-visible{outline:3px solid #91b6ff;outline-offset:3px}button{cursor:pointer;transition:background .12s,border-color .12s}button:disabled{cursor:default;opacity:.6}
.topbar{border-bottom:1px solid var(--line);background:var(--panel)}.topbar-inner{max-width:1240px;margin:auto;padding:22px 32px;display:flex;align-items:center;justify-content:space-between;gap:20px}.brand{display:flex;align-items:center;gap:12px}.mark{display:grid;place-items:center;width:38px;height:38px;border-radius:10px;background:#19332a;color:var(--ok);font-size:24px;font-weight:750}.brand strong{font-size:21px;letter-spacing:-.6px}.brand span{display:block;font-size:12px;color:var(--mut)}.connection{display:flex;align-items:center;gap:8px;font-size:13px;font-weight:600}.dot{height:8px;width:8px;border-radius:50%;background:var(--ok)}.connection.offline .dot{background:var(--bad)}#mode{font-size:11px;letter-spacing:.08em;color:var(--mut);border-left:1px solid var(--line);padding-left:14px;margin-left:7px}
main{max-width:1240px;padding:32px;margin:auto}.intro{margin:0 0 24px}.intro h1{font-size:28px;letter-spacing:-.8px;line-height:1.2;margin:0 0 7px;font-weight:650}.intro p{margin:0;color:var(--mut)}.workspace{display:grid;grid-template-columns:370px minmax(0,1fr);gap:24px;align-items:start}.panel{background:var(--panel);border:1px solid var(--line);border-radius:14px;overflow:hidden}.controls{padding:26px}.eyebrow{display:block;font-size:11px;letter-spacing:.1em;font-weight:700;color:var(--mut);text-transform:uppercase}.control-heading{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:20px}.control-heading select{border:0;background:#1b2635;font-size:12px;font-weight:600;padding:7px 25px 7px 9px;width:auto;max-width:170px}
.target{display:flex;align-items:baseline;gap:10px;margin-top:3px}.target input{appearance:textfield;-moz-appearance:textfield;font-size:76px;line-height:1.12;font-weight:600;letter-spacing:-4px;border:0;border-radius:6px;padding:0;width:100%;min-width:0;color:var(--fg);background:transparent;font-variant-numeric:tabular-nums}.target input::-webkit-inner-spin-button,.target input::-webkit-outer-spin-button{-webkit-appearance:none;margin:0}.target span{font-size:14px;font-weight:600;color:var(--mut);letter-spacing:.03em;padding-right:4px}input[type=range]{width:100%;margin:18px 0 2px;accent-color:var(--acc);height:20px;cursor:pointer}.range-labels{display:flex;justify-content:space-between;color:var(--mut);font-size:11px}.settings{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin:26px 0 22px}.field label{display:block;color:var(--mut);font-size:12px;font-weight:600;margin-bottom:6px}.field{min-width:0}.field input,.field select{width:100%;min-width:0}select,input[type=number]:not(#rpm){border:1px solid #3b485b;border-radius:7px;background:var(--panel);color:var(--fg);padding:9px 10px;font-size:14px;height:42px}.timed{gap:14px;margin:-8px 0 22px}.timed .field{flex:1}button{border:1px solid #3b485b;border-radius:8px;background:var(--panel);color:var(--fg);padding:11px 16px;font-weight:600;font-size:14px;min-height:44px}button:hover{background:#202d3e;border-color:#687c97}.go{background:#285fd8;border-color:#285fd8;color:#fff}.go:hover{background:#3470ec;border-color:#3470ec}.actions{display:grid;grid-template-columns:minmax(0,1fr) 90px;gap:10px}.actions button{height:48px;font-size:15px}#startmsg{font-size:12px;line-height:1.6;color:var(--mut);margin:13px 0 0;min-height:39px}#enablecontrols{margin-top:12px}#enablecontrols button{font-size:12px;padding:7px 10px;min-height:34px}.emergency{margin-top:23px;padding-top:22px;border-top:1px solid var(--line)}.emergency button{width:100%;border-color:#773c4a;color:#ffabb2;background:#301b24;font-size:14px;letter-spacing:.01em}.emergency button:hover{background:#44212b;border-color:#f17682}.emergency p{margin:7px 0 0;color:var(--mut);font-size:11px;text-align:center}.confirm{margin-top:15px;padding:15px;border-radius:9px;background:#17283f;border:1px solid #345781;font-size:13px}.confirm .row{display:flex;gap:8px;margin-top:13px;flex-wrap:wrap}.confirm button{font-size:12px;padding:9px 11px;min-height:38px}
.telemetry{min-width:0}.telemetry-heading{padding:23px 24px 0;display:flex;justify-content:space-between;align-items:center;gap:12px}.telemetry-heading h2{font-size:16px;letter-spacing:-.2px;font-weight:650;margin:0}.telemetry-heading span{font-size:11px;color:var(--mut)}.metrics{display:grid;grid-template-columns:1fr 1fr 1fr;padding:25px 24px 22px;gap:16px}.stat .k{font-size:11px;color:var(--mut);margin-bottom:5px}.stat .v{font-size:30px;line-height:1.25;letter-spacing:-1px;font-weight:600;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}.stat small{font-size:11px;color:var(--mut)}#s_phase{font-size:24px;text-transform:capitalize;padding-top:4px;letter-spacing:-.6px}.chartwrap{padding:0 20px}.legend{display:flex;align-items:center;gap:18px;padding:0 4px 16px;color:var(--mut);font-size:11px}.sw{display:inline-block;height:3px;width:17px;vertical-align:middle;margin-right:7px;border-radius:3px}canvas{width:100%;height:245px;display:block}.measurement{padding:16px 24px 20px;font-size:12px;color:var(--mut)}.telemetry-footer{border-top:1px solid var(--line);display:grid;grid-template-columns:1fr 1fr;padding:16px 24px;gap:16px;background:#101822}.compact{display:flex;justify-content:space-between;align-items:center;gap:10px;font-size:12px;color:var(--mut)}.compact strong{font-size:12px;font-weight:600;color:var(--fg);font-variant-numeric:tabular-nums}.drv{border-top:1px solid var(--line);padding:16px 24px 18px}.drv h3{margin:0 0 12px;font-size:13px;font-weight:650;display:flex;justify-content:space-between;gap:10px}.drv h3 span{font-size:11px;color:var(--mut);font-weight:500}.drvgrid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px}.drvgrid .k{font-size:11px;color:var(--mut)}.drvgrid .v{font-size:18px;font-weight:600;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}#clverdict{margin:12px 0 0;font-size:12px;color:var(--mut);overflow-wrap:anywhere}@media(max-width:760px){.drvgrid{grid-template-columns:1fr 1fr}}.extras.nocam{grid-template-columns:1fr}.ok{color:var(--ok)!important}.bad{color:var(--bad)!important}.extras{display:grid;grid-template-columns:1fr 1fr;gap:24px;margin-top:24px}details{background:var(--panel);border:1px solid var(--line);border-radius:10px;min-width:0;align-self:start}summary{cursor:pointer;padding:16px 20px;font-size:13px;font-weight:600}summary::marker{color:#a1adbe}.detail-body{border-top:1px solid var(--line);padding:18px 20px;font-size:12px;color:var(--mut)}.detail-body p{margin:0 0 14px;overflow-wrap:anywhere}.detail-body p:last-child{margin:0}.detail-body b{display:block;font-weight:600;color:var(--fg);margin-bottom:4px}.cam{position:relative;aspect-ratio:16/9;background:#0c131d;overflow:hidden;display:flex;align-items:center;justify-content:center;border-radius:6px}.cam img{width:100%;height:100%;object-fit:contain}.tile{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;text-align:center;padding:20px;color:var(--mut);font-size:13px}.footer{font-size:11px;color:var(--mut);margin:20px 0 0}#stopreason:empty{display:none}
@media(max-width:1000px){.workspace{grid-template-columns:330px minmax(0,1fr);gap:18px}.controls{padding:22px}.metrics{gap:10px;padding:24px 18px}.stat .v{font-size:26px}#s_phase{font-size:21px}.target input{font-size:68px}}
@media(max-width:760px){main{padding:24px 18px}.topbar-inner{padding:16px 18px}.workspace{grid-template-columns:1fr}.controls{padding:24px}.target input{font-size:76px}.extras{grid-template-columns:1fr;gap:14px}.intro h1{font-size:25px}.metrics{padding:24px;gap:12px}.stat .v{font-size:30px}#s_phase{font-size:24px}.connection{font-size:12px}#mode{display:none}.chartwrap{padding:0 14px}.telemetry-heading{padding:22px 24px 0}}
.simple{padding:26px;margin-bottom:28px;font-size:22px;color:var(--fg)}.simple label{display:block;font-size:20px;font-weight:700;margin-bottom:6px;color:var(--fg)}.simple input{font-size:44px;width:190px;padding:6px 12px;background:#1b2635;color:var(--fg);border:2px solid var(--line);border-radius:10px}.srow{display:flex;gap:20px;flex-wrap:wrap;align-items:flex-end;margin-bottom:20px}.simple #sdir{font-size:26px;font-weight:700;padding:16px 26px;background:#1b2635;color:var(--fg);border:2px solid var(--line);border-radius:10px}#son,#soff{flex:1;min-width:220px;font-size:56px;font-weight:800;padding:34px 20px;border-radius:16px;border:0;color:#04140c}#son{background:#3fd08a}#soff{background:#ff5a67;color:#2a0206}#sline1{font-size:34px;font-weight:750;margin-top:8px}#sline2{font-size:24px;margin-top:6px}#sline3{font-size:34px;font-weight:750;margin-top:8px;color:var(--fg)}#smsg{font-size:20px;margin-top:8px;min-height:1.4em}.detailsh{font-size:30px;margin:0 0 18px;font-weight:700}
@media(prefers-reduced-motion:reduce){*{transition:none!important}}
</style></head><body>
<header class="topbar"><div class="topbar-inner"><div class="brand"><div class="mark" aria-hidden="true">f</div><div><strong>Froggy</strong><span>Motor control</span></div></div><div class="connection" id="connectionwrap"><i class="dot" aria-hidden="true"></i><span id="connection" role="status">Connecting</span><span id="mode">__MODE__</span></div></div></header>
<main>
 <section class="panel simple" id="simple" aria-label="Simple on and off">
  <div class="srow"><div class="sfield"><label for="srpm">Speed (RPM)</label><input type="number" id="srpm" min="0" max="600" value="60"></div>
  <div class="sfield"><label for="sdir">Direction</label><button type="button" id="sdir" data-dir="cw" aria-live="polite">Clockwise</button></div></div>
  <div class="srow"><button type="button" id="son">ON</button><button type="button" id="soff">OFF</button></div>
  <div class="confirm" id="sconfirm" style="display:none"><div id="sconfirmtxt"></div><div class="row"><button class="go" id="sconfirmyes">Yes, spin the motor</button><button id="sconfirmno">Cancel</button></div></div>
  <div id="sline1" role="status" aria-live="polite">Motor: off</div>
  <div id="sline2">Driver: - &nbsp;&nbsp; ALM: -</div>
  <div id="sline3" role="status" aria-live="polite">Encoder: 0 rpm</div>
  <div id="smsg" role="status"></div>
 </section>
 <h2 class="detailsh">Details</h2>
 <div class="intro"><h1>Speed control</h1><p>Set a speed. Adjust it as you go.</p></div><span id="msg" hidden></span>
 <div class="workspace">
  <section class="panel controls" aria-label="Motor controls">
   <div class="control-heading"><label class="eyebrow" for="rpm">Target speed</label><select id="runmode" aria-label="Run mode"><option value="speed">Continuous speed</option><option value="ramp">Timed test</option><option value="cltest">Closed-loop test</option></select></div>
   <div class="target"><input type="number" id="rpm" min="0" max="600" value="60" aria-label="Target RPM"><span>RPM</span></div>
   <input type="range" id="rpmr" min="0" max="600" step="10" value="60" aria-label="Target RPM slider"><div class="range-labels"><span>0</span><span id="maxlabel">600 RPM</span></div>
   <div class="settings"><div class="field"><label for="dir">Direction</label><select id="dir"><option value="cw">Clockwise</option><option value="ccw">Counterclockwise</option></select></div><div class="field"><label for="acc">Change time · seconds</label><input type="number" id="acc" value="3" step="0.5" min="0"></div></div>
   <div class="timed" id="timedfields" style="display:none"><div class="field"><label for="hold">Hold · seconds</label><input type="number" id="hold" value="5" step="1" min="0"></div><div class="field"><label for="dec">Deceleration · seconds</label><input type="number" id="dec" value="3" step="0.5" min="0"></div></div>
   <div class="actions"><button type="button" class="go" id="startmotor">Set speed</button><button type="button" onclick="post('/api/stop')">Stop</button></div>
   <div id="startmsg" role="status" aria-live="polite">Runs until you press Stop. Apply a new speed anytime.</div>
   <div class="confirm" id="confirm" style="display:none"><div id="confirmtxt"></div><div class="row"><button class="go" id="confirmyes">Yes, spin the motor</button><button id="confirmno">Cancel</button></div></div>
   <div id="enablecontrols" hidden><button onclick="post('/api/enable',{on:true})">Enable driver</button> <button onclick="post('/api/enable',{on:false})">Disable driver</button></div>
   <div class="emergency"><button type="button" onclick="post('/api/estop')">Emergency STOP</button><p>Use Stop above for a controlled deceleration.</p></div>
  </section>
  <section class="panel telemetry" aria-label="Motor telemetry">
   <div class="telemetry-heading"><h2>Speed overview</h2><span>LAST 60 SECONDS</span></div>
   <div class="metrics"><div class="stat"><div class="k">Commanded</div><div class="v" id="s_cmd">-</div><small>RPM</small></div><div class="stat"><div class="k">Encoder RPM</div><div class="v" id="s_meas">-</div><small id="s_meassrc">RPM · encoder tap</small></div><div class="stat"><div class="k">Motor state</div><div class="v" id="s_phase">-</div></div></div>
   <div class="chartwrap"><div class="legend"><span><i class="sw" style="background:var(--acc)"></i>Commanded</span><span><i class="sw" style="background:var(--meas)"></i>Measured</span></div><canvas id="chart" role="img" aria-label="Commanded and measured speed over the last 60 seconds"></canvas></div>
   <div class="measurement" id="measurementnote">Waiting for motor status.</div>
   <div class="telemetry-footer"><div class="compact"><span>E-stop input</span><strong id="s_es">-</strong></div><div class="compact"><span>Elapsed</span><strong id="s_t">-</strong></div></div>
   <div class="drv" id="drivertile"><h3>Closed loop driver<span id="d_model">-</span></h3>
    <div class="drvgrid"><div><div class="k">ALM (fault)</div><div class="v" id="d_alm">-</div></div><div><div class="k">PEND (in position)</div><div class="v" id="d_pend">-</div></div><div><div class="k">Encoder RPM</div><div class="v" id="d_enc">-</div></div><div><div class="k">Following error</div><div class="v" id="d_fe">-</div></div></div>
    <p id="clverdict">No closed-loop test yet. Pick "Closed-loop test" above to run one.</p></div>
  </section>
 </div>
 <div class="extras" id="extras"><details id="diagnostics"><summary>Run details &amp; diagnostics</summary><div class="detail-body"><p><b>Last stop</b><span id="stopreason">No stop recorded.</span></p><p><b>Last run</b><span id="summary">No run yet.</span></p><p><b>Step frequency</b><span id="s_hz">0</span> Hz</p><p><b>Limits</b><span id="limits">-</span></p><p><b>Controller</b>__LABEL__</p></div></details>
 <details id="camera-panel"><summary>Camera preview</summary><div class="detail-body"><div class="cam" id="cam"><img id="camimg" alt="Camera stream" style="display:none"><div class="tile" id="camtile"><span id="camtxt">Open the preview to connect.</span></div></div></div></details></div>
 <p class="footer">Change time controls speed increases and decreases. Click Set speed to apply. Stop before reversing.</p>
</main>
<script>
const DRY=__DRY__,CAM=__CAM__;const $=id=>document.getElementById(id);
if(!CAM){$('camera-panel').hidden=true;$('extras').classList.add('nocam');}
const hist=[];let runStart=null,runElapsed=0,lastRpm=0,cameraRetry=null;
$('rpmr').oninput=e=>$('rpm').value=e.target.value;$('rpm').oninput=e=>$('rpmr').value=e.target.value;
function camInit(){const img=$('camimg');
 if(DRY){$('camtxt').innerHTML='dry run on the Mac<br>no Pi camera; the live view streams from the Pi 3 B+';return;}
 img.onload=()=>{img.style.display='block';$('camtile').style.display='none'};
 img.onerror=()=>{img.style.display='none';$('camtile').style.display='flex';$('camtxt').textContent='Camera unavailable. Check the camera connection.';if($('camera-panel').open)cameraRetry=setTimeout(()=>{if($('camera-panel').open)img.src='/stream.mjpg?'+Date.now()},4000)};
 img.src='/stream.mjpg?'+Date.now();}
$('camera-panel').addEventListener('toggle',()=>{if($('camera-panel').open)camInit();else{clearTimeout(cameraRetry);$('camimg').removeAttribute('src');}});
function feedback(text){$('msg').textContent=text;$('startmsg').textContent=text;}
window.addEventListener('error',e=>feedback('Page error: '+e.message+'. Reload the page and try again.'));
async function post(u,b){try{const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})});const j=await r.json();feedback(j.ok?(u==='/api/speed'?(b.rpm===0?'Target set to zero. Stopping motor.':'Target set to '+b.rpm+' RPM. Running until stopped.'):u==='/api/ramp'?'Timed run started.':u==='/api/cltest'?'Closed-loop test started.':u==='/api/stop'||u==='/api/estop'?'Stop requested.':'Driver setting updated.'):(j.error||'request failed'));}catch(e){feedback('Server not reachable. Keep froggy-motor ui running.');}}
// Ramps only ever start from a click on Start motor. Nothing on load or on a timer posts /api/ramp.
// The first ramp after each page load needs a second click on the confirm panel, and every ramp first
// asks the server for a one-time token (POST /api/arm), which the server requires on the live motor.
let confirmedThisLoad=false;
function rampParams(){return {mode:$('runmode').value,rpm:+$('rpm').value,accel_s:+$('acc').value,hold_s:+$('hold').value,decel_s:+$('dec').value,dir:$('dir').value};}
$('runmode').onchange=()=>{const m=$('runmode').value,timed=m!=='speed';$('timedfields').style.display=timed?'flex':'none';$('startmotor').textContent=m==='cltest'?'Start closed-loop test':timed?'Start timed run':'Set speed';feedback(m==='cltest'?'Ramps, holds, stops, and logs ALM, PEND and measured speed every 100 ms, then gives a verdict.':timed?'Accelerates, holds, then stops automatically.':'Runs until you press Stop. Change RPM and click Set speed to adjust while running.');$('confirm').style.display='none';};
function trustedStart(ev){if(ev&&ev.isTrusted)return true;feedback('Start click was not accepted by the browser. Click the blue button directly in the browser.');return false;}
function handleMotorStart(ev){if(!trustedStart(ev))return;const p=rampParams();
 if(!confirmedThisLoad&&p.rpm>0){$('confirmtxt').textContent=(DRY?'Dry run: ':'This spins the REAL motor: ')+p.rpm+' rpm '+p.dir.toUpperCase()+(p.mode==='speed'?', run until stopped.':(p.mode==='cltest'?' closed-loop test':'')+', accelerate '+p.accel_s+' s, hold '+p.hold_s+' s, decelerate '+p.decel_s+' s.')+' Area clear?';
  feedback('Confirm below to start the motor.');$('confirm').style.display='block';$('confirmyes').focus();
  $('confirmyes').onclick=e=>{if(!trustedStart(e))return;confirmedThisLoad=true;$('confirm').style.display='none';armAndRamp(p)};
  $('confirmno').onclick=()=>{$('confirm').style.display='none';feedback('Start cancelled. Motor was not started.');$('startmotor').focus()};return;}
 armAndRamp(p);}
$('startmotor').addEventListener('click',handleMotorStart);
async function armAndRamp(p){try{const r=await fetch('/api/arm',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({intent:p.mode})});const a=await r.json();
 if(!a.token){feedback(a.error||'no ramp token');return;}p.token=a.token;post(p.mode==='speed'?'/api/speed':p.mode==='cltest'?'/api/cltest':'/api/ramp',p);}catch(e){feedback('Server not reachable. Keep froggy-motor ui running.');}}

// Simple ON/OFF panel: same one-time token and first-click confirmation as the detailed controls.
function smsg(t){$('smsg').textContent=t;}
$('sdir').onclick=()=>{const b=$('sdir');const d=b.dataset.dir==='cw'?'ccw':'cw';b.dataset.dir=d;b.textContent=d==='cw'?'Clockwise':'Counterclockwise';};
async function simpleStart(p){try{const r=await fetch('/api/arm',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({intent:'speed'})});const a=await r.json();
 if(!a.token){smsg(a.error||'no ramp token');return;}p.token=a.token;
 const q=await fetch('/api/speed',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(p)});const j=await q.json();smsg(j.ok?'Starting.':(j.error||'request failed'));}catch(e){smsg('Server not reachable. Keep froggy-motor ui running.');}}
$('son').addEventListener('click',ev=>{if(!ev.isTrusted){smsg('Click was not accepted by the browser. Click the button directly.');return;}
 const p={rpm:+$('srpm').value,accel_s:+$('acc').value||3,dir:$('sdir').dataset.dir};
 if(!(p.rpm>0)){smsg('Enter a speed above 0 RPM.');return;}
 if(!confirmedThisLoad){$('sconfirmtxt').textContent=(DRY?'Dry run: ':'This spins the REAL motor: ')+p.rpm+' rpm '+p.dir.toUpperCase()+', run until OFF. Area clear?';$('sconfirm').style.display='block';
  $('sconfirmyes').onclick=e=>{if(!e.isTrusted)return;confirmedThisLoad=true;$('sconfirm').style.display='none';simpleStart(p)};
  $('sconfirmno').onclick=()=>{$('sconfirm').style.display='none';smsg('Cancelled. Motor was not started.')};return;}
 simpleStart(p);});
$('soff').addEventListener('click',()=>{$('sconfirm').style.display='none';post('/api/stop');smsg('Stopping.');});
function simpleStatus(s){const r=Math.round(s.rpm_cmd),t=Math.round(s.target_rpm);let m;
 if(s.rpm_cmd<0.5&&['idle','estop','alarm','fault'].includes(s.phase))m='off';
 else if(s.phase==='stopping'||s.phase==='estop'||t===0)m='stopping';
 else if(s.phase==='hold')m='holding '+r+' rpm';else m='ramping to '+t+' rpm';
 $('sline1').textContent='Motor: '+m;
 const drv=s.ena_wired?(s.enabled?'enabled':'disabled')+(s.ena_level==null?'':' (pin reads '+s.ena_level+')'):'always enabled (ENA not wired)';
 const alm=!s.alm_wired?'not wired':(s.alm?'active':'ok')+(s.alm_blocks_start?'':' (unverified polarity)');
 $('sline2').textContent='Driver: '+drv+'    ALM: '+alm;}
function encLine(s){let m;const sg=s.dir==='ccw'?-1:1;
 if(!s.enc_wired)m='Encoder: not wired';
 else if(s.enc_no_pulses)m='Encoder: no pulses, check EA+ wire';
 else if(s.rpm_cmd<0.5&&['idle','estop','alarm','fault'].includes(s.phase))m='Encoder: 0 rpm';
 else m='Encoder: '+Math.round(s.rpm_encoder_signed||0)+' rpm (commanded '+Math.round(sg*s.rpm_cmd)+' rpm), error '+(s.follow_err_pct==null?'-':s.follow_err_pct.toFixed(1))+' %';
 $('sline3').textContent=m;}
function fmt(x,d){return x==null||isNaN(x)?'-':(+x).toFixed(d)}
async function poll(){try{const r=await fetch('/api/status',{cache:'no-store'});const s=await r.json();
 $('connection').textContent=DRY?'Simulation connected':'Connected';$('connectionwrap').classList.remove('offline');
 simpleStatus(s);$('srpm').max=s.max_rpm;$('rpm').max=s.max_rpm;$('rpmr').max=s.max_rpm;$('maxlabel').textContent=s.max_rpm+' RPM';
 const meas=s.rpm_encoder;
 $('s_phase').textContent=s.phase+(s.enabled?'':' (off)');$('s_cmd').textContent=fmt(s.rpm_cmd,1);$('s_meas').textContent=fmt(meas,1);$('s_meassrc').textContent=s.enc_wired?'RPM · encoder tap':'RPM · encoder not wired';
 encLine(s);
 $('d_model').textContent=s.driver+(s.closed_loop?' · closed loop':' · open loop');
 const da=$('d_alm');da.textContent=s.alm==null?'not wired':(s.alm?'ALARM':'ok');da.className='v '+(s.alm?'bad':(s.alm==null?'':'ok'));
 $('d_pend').textContent=s.pend==null?'none on driver':(s.pend?'in position':'moving');
 $('d_enc').textContent=s.enc_wired?fmt(s.rpm_encoder,1):'not wired';$('d_fe').textContent=s.follow_err_deg==null?'-':fmt(s.follow_err_deg,1)+' deg';
 $('clverdict').textContent=s.cl_running?'Closed-loop test running: logging every 100 ms.':(s.cl_verdict||'No closed-loop test yet. Pick "Closed-loop test" above to run one.');
 $('enablecontrols').hidden=!s.ena_wired;
 $('stopreason').textContent=s.last_stop_reason||'No stop recorded.';
 $('measurementnote').textContent=meas==null?'No measured RPM: the blue line shows commands only and cannot confirm shaft rotation.':'Encoder tap is reporting measured shaft speed.';
 $('s_hz').textContent=fmt(s.step_hz,0);const es=$('s_es');es.textContent=s.estop?'OPEN':'closed';es.className='v '+(s.estop?'bad':'ok');
 if(s.rpm_cmd>0&&lastRpm<=0)runStart=s.t_s;if(runStart!=null&&(s.rpm_cmd>0||lastRpm>0))runElapsed=s.t_s-runStart;lastRpm=s.rpm_cmd;
 $('s_t').textContent=runStart==null?'-':fmt(runElapsed,1)+' s';
 $('summary').textContent=s.summary?s.summary.replace(/^summary: /,'').replace(/^peak /,'peak commanded ').replace(/, log .*/,''):'No run yet.';$('limits').textContent=s.limits||'-';
 if(s.warnings&&s.warnings.length&&!$('msg').textContent)$('msg').textContent=s.warnings[s.warnings.length-1];
 hist.push([s.t_s,s.rpm_cmd,meas]);while(hist.length&&hist[0][0]<s.t_s-60)hist.shift();draw(s.t_s,s.max_rpm);
 }catch(e){$('connection').textContent='Disconnected';$('connectionwrap').classList.add('offline');feedback('Connection lost. Keep froggy-motor ui running.');}}
function draw(now,maxr){const c=$('chart'),dpr=window.devicePixelRatio||1,W=c.clientWidth,Hh=c.clientHeight;c.width=W*dpr;c.height=Hh*dpr;
 const g=c.getContext('2d');g.scale(dpr,dpr);g.clearRect(0,0,W,Hh);const L=38,B=24,T=12,R=20;
 let top=100;for(const h of hist){top=Math.max(top,h[1]||0,h[2]||0)}top=Math.min(Math.ceil(top/100)*100,Math.max(maxr,100));
 const X=t=>L+(W-L-R)*(1-(now-t)/60),Y=v=>T+(Hh-T-B)*(1-v/top);
 g.strokeStyle='#2a3646';g.fillStyle='#a1adbe';g.font='11px -apple-system,sans-serif';g.lineWidth=1;
 for(let i=0;i<=4;i++){const v=top*i/4,y=Y(v);g.beginPath();g.moveTo(L,y);g.lineTo(W-R,y);g.stroke();g.fillText(v.toFixed(0),4,y+4)}
 for(let s=0;s<=60;s+=10){g.fillText('-'+(60-s)+'s',X(now-60+s)-14,Hh-5)}
 function line(i,col){g.strokeStyle=col;g.lineWidth=2.5;g.beginPath();let on=false;for(const h of hist){const v=h[i];if(v==null||isNaN(v)){on=false;continue}const x=X(h[0]),y=Y(v);if(!on){g.moveTo(x,y);on=true}else g.lineTo(x,y)}g.stroke()}
 line(1,'#79a7ff');line(2,'#edb365');}
poll();setInterval(poll,200);
</script></body></html>
"""


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_parser():
    ap = argparse.ArgumentParser(prog="froggy_motor.py", allow_abbrev=False,
                                 description="CL57T-V41 / DM320T stepper test tool for the Froggy Turntable")
    ap.add_argument("--dry-run", action="store_true", help="use the fake pigpio (automatic when pigpio is missing off the Pi)")
    ap.add_argument("--sim-fast", action="store_true", help="dry run only: simulated clock, runs instantly")
    ap.add_argument("--sim-estop-at", type=float, default=None, metavar="S", help="dry run only: press the fake e-stop S seconds in")
    ap.add_argument("--sim-stall-rpm", type=float, default=None, metavar="R",
                    help="dry run only: the fake rotor loses sync above R rpm (open loop: silent stall, closed loop: ALM)")
    ap.add_argument("--sim-alarm-at", type=float, default=None, metavar="S", help="dry run only: the fake driver raises ALM S seconds in")
    ap.add_argument("--driver", choices=sorted(DRIVERS), default=None, help="override [driver] model from motor.toml")
    ap.add_argument("--config", default=None, help="motor.toml path (default: next to this file)")
    ap.add_argument("--motor", choices=sorted(MOTORS), default=None, help="which NEMA 17 is fitted")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("ramp", help="ramp up, hold, ramp down to rest")
    r.add_argument("--rpm", type=float, required=True)
    r.add_argument("--accel-s", type=float, required=True, help="acceleration time in seconds")
    r.add_argument("--hold-s", type=float, required=True)
    r.add_argument("--decel-s", type=float, required=True, help="deceleration time in seconds")
    r.add_argument("--dir", choices=["cw", "ccw"], default="cw")
    r.add_argument("--log", default=None, help="CSV log path (default logs/ramp-<time>.csv)")
    s = sub.add_parser("speed", help="run at a constant speed until Stop or Ctrl-C")
    s.add_argument("--rpm", type=float, required=True)
    s.add_argument("--accel-s", type=float, default=3.0, help="seconds to reach target (default 3)")
    s.add_argument("--dir", choices=["cw", "ccw"], default="cw")
    s.add_argument("--log", default=None)
    c = sub.add_parser("cl-test", help="closed-loop test: ramp, hold, stop; 100 ms CSV of ALM, PEND, measured RPM; verdict")
    c.add_argument("--rpm", type=float, required=True)
    c.add_argument("--accel-s", type=float, default=None, help="seconds to reach rpm (default rpm / max accel, at least 2)")
    c.add_argument("--hold-s", type=float, default=10.0)
    c.add_argument("--decel-s", type=float, default=None, help="default: same as accel")
    c.add_argument("--dir", choices=["cw", "ccw"], default="cw")
    c.add_argument("--log", default=None, help="CSV path (default logs/cl-test-<rpm>rpm-<time>.csv)")
    sub.add_parser("stop", help="emergency decel of a running ramp (or of the live PWM), then ENA off")
    e = sub.add_parser("estop-test", help="read the e-stop input and watch it for a few seconds")
    e.add_argument("--s", type=float, default=5.0)
    i = sub.add_parser("index-test", help="count hall index pulses and report RPM")
    i.add_argument("--s", type=float, required=True)
    i.add_argument("--rpm", type=float, default=0.0, help="spin at this RPM during the count (0: turn by hand)")
    sub.add_parser("status", help="backend, config, input levels")
    tr = sub.add_parser("trials", help="saved trials (one per actuation): list, export, prune")
    tsub = tr.add_subparsers(dest="trials_cmd", required=True)
    tsub.add_parser("list", help="table of saved trials, newest first")
    te = tsub.add_parser("export", help="write one trial as CSV and/or JSON (CSV to stdout when no path is given)")
    te.add_argument("id", help="trial id, or 'latest'")
    te.add_argument("--csv", default=None, metavar="PATH")
    te.add_argument("--json", default=None, metavar="PATH")
    tsub.add_parser("prune", help="apply trials_keep / trials_max_mb now")
    tr.add_argument("--dir", default=None, help=argparse.SUPPRESS)     # tests: a different trials directory
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
    if a.driver:
        cfg["driver"]["model"] = a.driver
    if a.cmd == "trials":            # files only: no pigpio, no motor lock
        return cmd_trials(TrialStore.from_config(cfg, a.dir), a)
    if a.cmd in ("ramp", "speed", "cl-test") and not (0 < a.rpm <= cfg["max_rpm"]):
        print("error: --rpm must be in 0..%d" % cfg["max_rpm"], file=sys.stderr)
        return 2
    fast = a.sim_fast and (a.dry_run or not on_pi())
    clock = FakeClock() if fast else time.monotonic
    sleep = clock.sleep if fast else time.sleep
    pi, backend = connect(cfg, a.dry_run or a.sim_fast, clock=clock, estop_at=a.sim_estop_at,
                          stall_rpm=a.sim_stall_rpm, alarm_at=a.sim_alarm_at)
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
    if a.cmd in ("ramp", "speed"):
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
        ctl.request("estop", why="signal")
        if a.cmd == "ui" and signum != getattr(signal, "SIGUSR1", -1):
            stop_ev.set()

    stop_ev = threading.Event()
    _setup_signals(on_signal)
    try:
        if a.cmd in ("ramp", "speed", "ui", "index-test", "cl-test") and not dry:
            _write_pid()
        if a.cmd == "speed":
            print("speed %.0f rpm %s, change %.1f s, runs until Stop or Ctrl-C [%s]"
                  % (a.rpm, a.dir, a.accel_s, backend), flush=True)
            try:
                print(ctl.run_profile(a.rpm, a.accel_s, 0, 0, a.dir, continuous=True))
            except (RuntimeError, ValueError) as e:
                print("error: %s" % e, file=sys.stderr)
                return 3
            return 130 if hits["n"] and hits.get("sig") != getattr(signal, "SIGUSR1", -1) else 0
        if a.cmd == "ramp":
            print("ramp %.0f rpm %s, accel %.1f s, hold %.1f s, decel %.1f s, microstep %d, peak step_hz %.0f [%s]"
                  % (a.rpm, a.dir, a.accel_s, a.hold_s, a.decel_s, cfg["microstep"], rpm_to_hz(cfg, a.rpm), backend))
            try:
                print(ctl.run_profile(a.rpm, a.accel_s, a.hold_s, a.decel_s, a.dir))
            except RuntimeError as e:
                print("error: %s" % e, file=sys.stderr)
                return 3
            return 130 if hits["n"] and hits.get("sig") != getattr(signal, "SIGUSR1", -1) else 0
        if a.cmd == "cl-test":
            rc = cmd_cl_test(cfg, ctl, a, backend)
            return 130 if rc == 0 and hits["n"] and hits.get("sig") != getattr(signal, "SIGUSR1", -1) else rc
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
        pi.write(p["ena"], 1 - cfg["ena_enable_level"])
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
    for line in driver_lines(cfg, pi):
        print(line)
    return 0


def driver_switch_text(cfg):
    """One line: driver model and its switch settings, from the config only (no GPIO reads)."""
    d = cfg["driver"]
    if d["model"] == "CL57T-V41":
        s1 = CL57T_S1.get(d["s1"])
        sw = CL57T_MICROSTEP.get(cfg["microstep"])
        return ("driver: CL57T-V41, %s loop (SW6 %s), S1 code %s = %s, microstep %d = SW1..SW4 %s, S3 = 5V"
                % (d.get("loop", "closed"), "OFF" if closed_loop(cfg) else "ON", d["s1"],
                   "%.1f A peak / %.1f A RMS, velocity Ki %d, position Kp %d, velocity Kp %d" % s1 if s1 else "?",
                   cfg["microstep"], sw or "not a CL57T setting"))
    return "driver: %s (%s)" % (d["model"], "closed loop" if driver_info(cfg)["closed_loop"] else "open loop")


def driver_lines(cfg, pi, enc_edges=None):
    """status lines for the driver: model, switch settings, and the live ALM / PEND / BRK levels."""
    d, p = cfg["driver"], cfg["pins"]
    info = driver_info(cfg)
    out = [driver_switch_text(cfg)]
    out.append("driver outputs: %s" % info["outputs"])

    def lvl(name):
        g = p.get(name)
        return None if g is None else pi.read(g)
    a = lvl("alm")
    if a is None:
        out.append("ALM: not wired (the Pi cannot see driver faults)")
    else:
        out.append("ALM: %s (GPIO %d reads %d; alarm when %d)%s" % (
            "ALARM" if a == d["alm_active_level"] else "ok", p["alm"], a, d["alm_active_level"],
            "" if cfg.get("alm_blocks_start", True) else " unverified polarity, starts are not blocked"))
    pe = lvl("pend")
    if pe is None:
        out.append("PEND: not wired%s" % (" (the CL57T-V41 P1 connector has no PEND terminal)"
                                          if d["model"] == "CL57T-V41" else ""))
    else:
        out.append("PEND: %s (GPIO %d reads %d; in position when %d)" % (
            "in position" if pe == d["pend_active_level"] else "not in position", p["pend"], pe, d["pend_active_level"]))
    b = lvl("brk")
    out.append("BRK: not wired" if b is None else "BRK: GPIO %d reads %d (raw, meaning not documented)" % (p["brk"], b))
    if p.get("enc_a") is None:
        out.append("encoder tap: not wired")
    else:
        out.append("encoder tap: GPIO %d on EA+, %d lines/rev%s" % (
            p["enc_a"], d["enc_ppr"], "" if enc_edges is None else ", %d edges counted so far" % enc_edges))
    return out


def cmd_status(cfg, ctl, backend):
    m = MOTORS.get(cfg["motor"], {})
    print("backend: %s" % backend)
    for line in driver_lines(cfg, ctl.pi, ctl.enc.count())[:1]:
        print(line)
    if cfg["driver"]["model"] == "CL57T-V41":
        print("motor: %s (%.1f A rated), CL57T current %s" % (cfg["motor"], m.get("rated_a", 0), m.get("cl57t", "?")))
    else:
        print("motor: %s (%.1f A rated), DIP current %s" % (cfg["motor"], m.get("rated_a", 0), m.get("dip_current", "?")))
    print("microstep %d -> %d pulses/rev; max %d rpm = %.0f Hz; max accel %d rpm/s; e-stop decel %.2f s"
          % (cfg["microstep"], cfg["steps_per_rev"] * cfg["microstep"], cfg["max_rpm"],
             rpm_to_hz(cfg, cfg["max_rpm"]), cfg["max_accel_rpm_per_s"], cfg["emergency_decel_s"]))
    print("pins: %s" % json.dumps(cfg["pins"]))
    print("e-stop: %s, %s"
          % ("OPEN (stop)" if ctl.estop_open() else "closed (run allowed)",
             "index: not fitted" if cfg["pins"].get("index") is None
             else "index input level %d" % ctl.pi.read(cfg["pins"]["index"])))
    if not ena_wired(cfg):
        print("ENA: not wired (driver always enabled)")
    else:
        lvl = ctl.pi.read(cfg["pins"]["ena"])
        print("ENA: GPIO %d set and read back as %d (driver %s; enabled when %d)"
              % (cfg["pins"]["ena"], lvl, "enabled" if lvl == cfg["ena_enable_level"] else "disabled",
                 cfg["ena_enable_level"]))
    for line in driver_lines(cfg, ctl.pi, ctl.enc.count())[1:]:
        print(line)
    return 0


def cmd_cl_test(cfg, ctl, a, backend):
    """Ramp to --rpm, hold, stop; one CSV row per 100 ms; verdict line at the end."""
    acc = a.accel_s if a.accel_s is not None else max(a.rpm / cfg["max_accel_rpm_per_s"], 2.0)
    dec = a.decel_s if a.decel_s is not None else acc
    path = a.log or os.path.join(HERE, "logs", "cl-test-%drpm-%s.csv" % (round(a.rpm), time.strftime("%Y%m%d-%H%M%S")))
    print("cl-test %.0f rpm %s, accel %.1f s, hold %.1f s, decel %.1f s, %s %s loop, 1/%d step (%.0f Hz at target) [%s]"
          % (a.rpm, a.dir, acc, a.hold_s, dec, cfg["driver"]["model"], "closed" if closed_loop(cfg) else "open",
             cfg["microstep"], rpm_to_hz(cfg, a.rpm), backend), flush=True)
    for line in driver_lines(cfg, ctl.pi, ctl.enc.count())[2:]:
        print("  " + line)
    try:
        ctl._start_cltest(a.rpm, acc, a.hold_s, dec, a.dir, path=path)
    except (RuntimeError, ValueError) as e:
        print("error: %s" % e, file=sys.stderr)
        return 3
    ctl.disable_at_rest = True
    dt = cfg["tick_s"]
    nxt = ctl.clock()
    while ctl.tick():
        nxt += dt
        if nxt < ctl.clock() - dt:
            nxt = ctl.clock()
        ctl._sleep_until(nxt)
    ctl._ena(False)
    print(ctl.last_summary)
    print(ctl.cl_verdict)               # last line: the verdict
    return 5 if ctl.alarm_events else 0


def _fmt_size(n):
    return "%.0f B" % n if n < 1024 else "%.1f KB" % (n / 1024) if n < 1024 * 1024 else "%.1f MB" % (n / 1048576)


def cmd_trials(store, a):
    if a.trials_cmd == "list":
        rows_ = store.list()
        if not rows_:
            print("no trials in %s" % store.path)
            return 0
        print("%-28s  %-19s  %-7s  %7s  %9s  %-8s  %8s" % ("id", "start (UTC)", "mode", "rpm", "duration", "stop", "size"))
        for e in rows_:
            print("%-28s  %-19s  %-7s  %7s  %9s  %-8s  %8s" % (
                e["id"], (e["start"] or "")[:19].replace("T", " "), e["mode"] or "",
                "" if e["rpm"] is None else "%.0f" % e["rpm"],
                "" if e["duration_s"] is None else "%.1f s" % e["duration_s"],
                e["stop_cause"] or "", _fmt_size(e["size_bytes"])))
        print("%d trials, %s, in %s" % (len(rows_), _fmt_size(sum(e["size_bytes"] for e in rows_)), store.path))
        return 0
    if a.trials_cmd == "export":
        tid = a.id
        if tid == "latest":
            rows_ = store.list()
            if not rows_:
                print("error: no trials in %s" % store.path, file=sys.stderr)
                return 2
            tid = rows_[0]["id"]
        if not valid_trial_id(tid):
            print("error: %r is not a trial id (expected like 20261109T171512.345Z-a3f9c2)" % tid, file=sys.stderr)
            return 2
        try:
            rec = store.load(tid)
        except OSError:
            print("error: no trial %s in %s" % (tid, store.path), file=sys.stderr)
            return 2
        if not a.csv and not a.json:
            sys.stdout.write(trial_csv(rec))
            return 0
        if a.csv:
            with open(a.csv, "w", newline="") as f:
                f.write(trial_csv(rec))
            print("wrote %s" % a.csv)
        if a.json:
            with open(a.json, "w") as f:
                json.dump(rec, f, indent=1)
            print("wrote %s" % a.json)
        return 0
    removed = store.prune()
    print("pruned %d trials (keep %d, max %.0f MB)%s" % (len(removed), store.keep, store.max_mb,
                                                          ": " + ", ".join(removed) if removed else ""))
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
    if cfg["pins"].get("index") is None:
        print("index-test: no hall index sensor configured ([pins] index = \"none\"); nothing to count", file=sys.stderr)
        return 2
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
    camera = None if dry or not cfg.get("camera", True) else Camera(cfg["camera_cmd"])
    label = "%s, %s, 1/%d step" % (backend, cfg["motor"], cfg["microstep"])
    ctl.log_dir = os.path.join(HERE, "logs")
    ctl.out = ui_log
    if a.locked:
        ctl.locked = True
        ctl._ena(False)
        label += ", LOCKED: ramps refused"
    if not dry and camera is None:
        label += ", no camera (camera = false)"
    srv = make_server(ctl, host, a.port, dry, camera, label, show_camera=bool(cfg.get("camera", True)))
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
        # controlled stop on the way out; an open trial closes with cause "exit"
        ctl._trial_event("exit", cause="exit")
        ctl.request("estop", why="server exit")
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
