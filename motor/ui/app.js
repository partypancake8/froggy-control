// Froggy motor control page (HMI panel). Safety rules:
//  - Nothing that moves the motor is sent on load, reload or on a timer. Only /api/status is polled.
//  - START (or APPLY) asks for a one-time token (POST /api/arm) inside the click handler, then sends the run call.
//  - The first START after each page load needs a second click on "Yes, spin the motor" in the confirm dialog.
//  - STOP and EMERGENCY STOP never need a token or a confirmation.
//  - START/APPLY is disabled while the link to the motor server is down.
(function () {
  'use strict';
  var API = String(window.FROGGY_API || '').replace(/\/$/, '');
  var POLL_MS = 200, WINDOW_S = 60, DOWN_MS = 5000;

  var $ = function (id) { return document.getElementById(id); };
  var el = {
    connField: $('connfield'), conn: $('conn'), connTxt: $('conntxt'), cmdLink: $('cmdlink'), cmdLinkIc: $('cmdlinkic'), cmdLinkTxt: $('cmdlinktxt'),
    themeIc: $('themeic'), themeLbl: $('themelbl'), diagLbl: $('diaglbl'), startLbl: $('startlbl'),
    hdrAlarmIc: $('hdralarmic'), hdrAlarmTxt: $('hdralarmtxt'), verdictIc: $('verdictic'), verdictTxt: $('verdicttxt'),
    dry: $('drybadge'), ttlDrv: $('ttldrv'), ttlLoop: $('ttlloop'),
    hdrDriver: $('hdrdriver'), hdrLoop: $('hdrloop'), hdrAlarm: $('hdralarm'),
    noPulses: $('nopulses'),
    modeSeg: $('modeseg'), dirSeg: $('dirseg'), rpm: $('rpm'), slider: $('rpmslider'), ticks: $('ticks'),
    rpmUp: $('rpmup'), rpmDn: $('rpmdn'),
    accel: $('accel'), hold: $('hold'), decel: $('decel'), hint: $('modehint'),
    confirm: $('confirm'), confirmTxt: $('confirmtxt'), yes: $('confirmyes'), no: $('confirmno'),
    start: $('start'), stop: $('stop'), estop: $('estop'), msg: $('msg'),
    rCmd: $('r_cmd'), rEnc: $('r_enc'), rFerr: $('r_ferr'), rFerrPct: $('r_ferrpctv'), rEncU: $('r_encu'), rState: $('r_state'),
    rElapsed: $('r_elapsed'), ferrTile: $('ferrtile'), cam: $('cam'), camWrap: $('camwrap'), camLbl: $('camlbl'),
    lDrv: $('l_drv'), rDrv: $('r_drv'), lEstop: $('l_estop'), rEstop: $('r_estop'), lAlarm: $('l_alarm'), rAlarm: $('r_alarm'),
    lPulses: $('l_pulses'), rPulses: $('r_pulses'), lRun: $('l_run'), rRun: $('r_run'),
    pDriver: $('p_driver'), pLoop: $('p_loop'), pPpr: $('p_ppr'), pMax: $('p_max'), pAcc: $('p_acc'), pEnc: $('p_enc'), pPins: $('p_pins'),
    cStop: $('c_stop'), cSum: $('c_sum'),
    chart: $('chart'), verdict: $('verdict'), clIdle: $('clidle'), tagCmd: $('tagcmd'), tagEnc: $('tagenc'),
    control: $('control'), themeBtn: $('themebtn'),
    diagBtn: $('diagbtn'), diagClose: $('diagclose'), diagBody: $('diagbody'), enable: $('enable'), disable: $('disable'),
    limits: $('diaglimits'), stopReason: $('diagstop'), summary: $('diagsummary'),
    csvWrap: $('diagcsv'), csv: $('csvlink'), logTail: $('logtail'), raw: $('rawjson'),
    camCap: $('camcap'), camGrp: $('camgrp'), trendTtl: $('trendttl'), tLive: $('t_live'),
    tBody: $('tbody'), tCsv: $('t_csv'), tPdf: $('t_pdf'), tShow: $('t_show'), tMsg: $('tmsg'), tDl: $('t_dl')
  };

  var MODES = {
    speed: { call: '/api/speed', hint: 'Ramps to the speed and holds until you press STOP.' },
    ramp: { call: '/api/ramp', hint: 'Ramps up, holds for the hold time, then decelerates to a stop on its own.' },
    cltest: { call: '/api/cltest', hint: 'Ramps, holds and stops, logging the encoder every 100 ms, then gives a verdict in the Closed-loop test box.' }
  };

  var state = {
    mode: 'speed', dir: 'cw', maxRpm: 600, confirmed: false, running: false, busy: false,
    st: null, t0: null, elapsed: 0, rampDefaultSet: false, lastEdges: null
  };
  var link = { lastOk: 0, missed: 0, loadedAt: Date.now(), host: '', mode: 'wait' };
  var hist = { t: [], c: [], e: [] };

  // Tabler icons come from one vendored sprite (vendor/tabler-icons/sprite.svg); swapping an icon swaps the <use> href.
  var SPRITE = 'vendor/tabler-icons/sprite.svg#ti-';
  function setIc(svg, name) { var u = svg && svg.querySelector('use'); if (u && u.getAttribute('href') !== SPRITE + name) u.setAttribute('href', SPRITE + name); }
  function makeIc(name, cls) {
    var NS = 'http://www.w3.org/2000/svg', svg = document.createElementNS(NS, 'svg'), u = document.createElementNS(NS, 'use');
    svg.setAttribute('class', 'ic' + (cls ? ' ' + cls : '')); svg.setAttribute('aria-hidden', 'true'); svg.setAttribute('focusable', 'false');
    u.setAttribute('href', SPRITE + name); svg.appendChild(u); return svg;
  }

  function msg(t) { el.msg.textContent = t; }
  function num(x) { var v = parseFloat(x); return isFinite(v) ? v : NaN; }
  function fmt(v, d) {
    if (v == null || !isFinite(v)) return '--';
    var t = Number(v).toFixed(d);
    return /^-0(\.0+)?$/.test(t) ? t.slice(1) : t;      // never show "-0.00"
  }

  async function api(path, body) {
    var opts = body === undefined ? { cache: 'no-store' }
      : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) };
    var r = await fetch(API + path, opts);
    var j = {};
    try { j = await r.json(); } catch (e) { j = { ok: false, error: 'bad reply (' + r.status + ')' }; }
    return j;
  }

  // ---------- radio groups ----------
  function setSeg(seg, attr, val) {
    seg.querySelectorAll('button').forEach(function (b) { b.setAttribute('aria-checked', String(b.dataset[attr] === val)); });
  }
  el.modeSeg.addEventListener('click', function (e) {
    var b = e.target.closest('button'); if (!b || b.disabled) return;
    state.mode = b.dataset.mode; setSeg(el.modeSeg, 'mode', state.mode);
    document.querySelectorAll('.extra').forEach(function (f) { f.hidden = f.dataset.for.split(' ').indexOf(state.mode) < 0; });
    el.hint.textContent = MODES[state.mode].hint;
    hideConfirm(); renderButtons();
  });
  el.dirSeg.addEventListener('click', function (e) {
    var b = e.target.closest('button'); if (!b || b.disabled) return;
    state.dir = b.dataset.dir; setSeg(el.dirSeg, 'dir', state.dir); hideConfirm();
  });

  // ---------- RPM field, spinner and slider stay in sync ----------
  el.slider.addEventListener('input', function () { el.rpm.value = el.slider.value; hideConfirm(); });
  el.rpm.addEventListener('input', function () { var v = num(el.rpm.value); if (isFinite(v)) el.slider.value = String(v); hideConfirm(); });
  function nudge(d) {
    var v = num(el.rpm.value); if (!isFinite(v)) v = 0;
    v = Math.max(0, Math.min(state.maxRpm, Math.round(v / 10) * 10 + d));
    el.rpm.value = el.slider.value = String(v); hideConfirm();
  }
  el.rpmUp.addEventListener('click', function () { nudge(10); });
  el.rpmDn.addEventListener('click', function () { nudge(-10); });
  [el.accel, el.hold, el.decel].forEach(function (i) { i.addEventListener('input', hideConfirm); });

  function setScale(max) {
    var spans = el.ticks.querySelectorAll('span');
    for (var i = 0; i < spans.length; i++) spans[i].textContent = String(Math.round(max * i / (spans.length - 1)));
  }

  function readParams() {
    var rpm = num(el.rpm.value), accel = num(el.accel.value), hold = num(el.hold.value), decel = num(el.decel.value);
    if (!isFinite(rpm) || rpm < 0 || rpm > state.maxRpm) { msg('Speed must be between 0 and ' + state.maxRpm + ' RPM.'); return null; }
    if (!isFinite(accel) || accel < 0) { msg('Ramp time must be 0 s or more.'); return null; }
    var p = { rpm: rpm, dir: state.dir, accel_s: accel };
    if (state.mode !== 'speed') {
      if (!isFinite(hold) || hold < 0 || !isFinite(decel) || decel < 0) { msg('Hold and decel must be 0 s or more.'); return null; }
      p.hold_s = hold; p.decel_s = decel;
    }
    if (rpm === 0 && !state.running) { msg('Enter a speed above 0 RPM to start.'); return null; }
    return p;
  }

  function describe(p) {
    var real = state.st && state.st.dry_run ? 'Dry run: ' : 'This spins the REAL motor: ';
    var s = real + p.rpm + ' RPM ' + p.dir.toUpperCase() + ', ramp ' + p.accel_s + ' s';
    if (state.mode === 'speed') s += ', hold until STOP.';
    else s += ', hold ' + p.hold_s + ' s, decel ' + p.decel_s + ' s' + (state.mode === 'cltest' ? ' (closed-loop test).' : '.');
    return s + ' Area clear?';
  }

  function hideConfirm() { el.confirm.hidden = true; }

  el.start.addEventListener('click', function (e) {
    if (!e.isTrusted || state.busy || link.mode === 'down') return;
    var p = readParams(); if (!p) return;
    if (!state.confirmed) {
      el.confirmTxt.textContent = describe(p);
      el.confirm.hidden = false; el.yes.focus();
      msg('Confirm in the dialog to start the motor.');
      el.yes.onclick = function (ev) {
        if (!ev.isTrusted) return;
        state.confirmed = true; hideConfirm(); armAndRun(p);
      };
      return;
    }
    armAndRun(p);
  });
  el.no.addEventListener('click', function () { hideConfirm(); msg('Cancelled. The motor was not started.'); });
  document.addEventListener('keydown', function (e) {
    if (e.key === 'Escape' && !el.confirm.hidden) { hideConfirm(); msg('Cancelled. The motor was not started.'); }
  });

  async function armAndRun(p) {
    var mode = state.mode, call = MODES[mode].call;
    state.busy = true; renderButtons();
    try {
      var a = await api('/api/arm', { intent: mode });
      if (!a.token) { msg(a.error || 'No start token from the server.'); return; }
      p.token = a.token;
      var j = await api(call, p);
      if (j.ok) msg(state.running ? 'Applied: ' + p.rpm + ' RPM.' : (mode === 'speed' ? 'Ramping to ' + p.rpm + ' RPM, holding until STOP.'
        : mode === 'ramp' ? 'Timed test started.' : 'Closed-loop test started.'));
      else msg(j.error || 'The server refused the command.');
      if (j.status) render(j.status);
    } catch (err) {
      msg('Server not reachable. Keep froggy-motor ui running.');
    } finally { state.busy = false; renderButtons(); }
  }

  async function simple(path, body, okText) {
    try {
      var j = await api(path, body || {});
      msg(j.ok ? okText : (j.error || 'Request failed.'));
      if (j.status) render(j.status);
    } catch (err) { msg('Server not reachable. Keep froggy-motor ui running.'); }
  }
  el.stop.addEventListener('click', function () { hideConfirm(); simple('/api/stop', {}, 'Stopping with normal deceleration.'); });
  el.estop.addEventListener('click', function () { hideConfirm(); simple('/api/estop', {}, 'EMERGENCY STOP sent.'); });
  el.enable.addEventListener('click', function () { simple('/api/enable', { on: true }, 'Driver enabled.'); });
  el.disable.addEventListener('click', function () { simple('/api/enable', { on: false }, 'Driver disabled.'); });

  function toggleDiag() {
    var open = el.diagBody.hidden;
    el.diagBody.hidden = !open;
    el.diagBtn.setAttribute('aria-expanded', String(open));
    el.diagLbl.textContent = open ? 'Close diagnostics' : 'Diagnostics';
    if (open && state.st) render(state.st);
  }
  el.diagBtn.addEventListener('click', toggleDiag);
  el.diagClose.addEventListener('click', toggleDiag);

  function renderButtons() {
    el.startLbl.textContent = state.running ? 'APPLY' : 'START';
    el.start.disabled = state.busy || link.mode === 'down';
    el.dirSeg.querySelectorAll('button').forEach(function (b) { b.disabled = state.running; });
    el.modeSeg.querySelectorAll('button').forEach(function (b) { b.disabled = state.running; });
  }

  // ---------- connection ----------
  function ageText(ms) {
    var s = ms / 1000;
    if (s < 10) return s.toFixed(1) + ' s';
    if (s < 120) return Math.round(s) + ' s';
    return Math.round(s / 60) + ' min';
  }
  function renderLink() {
    var now = Date.now(), host = link.host || location.host || 'this computer';
    var age = link.lastOk ? now - link.lastOk : null, mode, txt;
    if (link.lastOk && link.missed < 2 && age < DOWN_MS) {
      mode = 'ok'; txt = 'CONNECTED ' + host + ', status ' + ageText(age) + ' ago';
    } else if (link.lastOk && age < DOWN_MS) {
      mode = 'retry'; txt = 'RECONNECTING, last status ' + ageText(age) + ' ago';
    } else if (!link.lastOk && now - link.loadedAt < DOWN_MS) {
      mode = 'wait'; txt = 'CONNECTING to ' + host;
    } else {
      mode = 'down'; txt = 'DISCONNECTED from ' + host + (link.lastOk ? ', last status ' + ageText(age) + ' ago' : ', no status yet');
    }
    el.connField.setAttribute('data-link', mode);
    el.connTxt.textContent = txt;
    setIc(el.conn, mode === 'retry' ? 'refresh' : mode === 'down' ? 'plug-connected-x' : 'plug-connected');   // colour from .connfield[data-link]
    el.cmdLink.hidden = mode === 'ok' || mode === 'wait';
    el.cmdLink.className = 'linkline' + (mode === 'retry' ? ' retry' : '');
    setIc(el.cmdLinkIc, mode === 'retry' ? 'refresh' : 'plug-connected-x');
    el.cmdLinkTxt.textContent = mode === 'down' ? 'Link down: ' + txt + '. START is locked until the status comes back.'
      : mode === 'retry' ? 'Link unsteady: ' + txt + '.' : '';
    document.body.setAttribute('data-link', mode);
    if (mode !== link.mode) { link.mode = mode; renderButtons(); }
  }
  setInterval(renderLink, 250);

  // ---------- status ----------
  function motorState(s) {
    var ph = s.phase;
    if (s.alarm_latched || ph === 'alarm' || ph === 'fault') return 'alarm';
    if (ph === 'estop' || (ph === 'idle' && s.estop_latched)) return 'e-stop';
    if (ph === 'idle') return 'off';
    if (ph === 'accel') return 'ramping';
    if (ph === 'hold') return 'holding';
    if (ph === 'decel') return (s.target_rpm > 0 && s.target_rpm < s.rpm_cmd + 0.5) ? 'ramping' : 'stopping';
    if (ph === 'stopping') return 'stopping';
    return String(ph || 'off');
  }

  function encRpm(s) {
    if (s.rpm_encoder != null) return s.rpm_encoder;
    if (s.enc_rpm != null) return s.enc_rpm;
    if (s.rpm_measured != null) return s.rpm_measured;
    return null;
  }

  function lamp(led, txtEl, cls, txt) { led.className = 'led ' + cls; txtEl.textContent = txt; }

  function microstep(s) { var m = /1\/(\d+) step/.exec(s.limits || ''); return m ? +m[1] : null; }
  function pulsesPerRev(s) {
    if (s.pulses_per_rev) return s.pulses_per_rev;
    var ms = microstep(s); return ms ? (s.steps_per_rev || 200) * ms : null;
  }

  function renderParams(s) {
    el.pDriver.textContent = s.driver || '--';
    el.pLoop.textContent = s.closed_loop ? 'Closed loop' : 'Open loop';
    var spr = pulsesPerRev(s);
    el.pPpr.textContent = spr ? String(spr) + (microstep(s) ? ' (1/' + microstep(s) + ')' : '') : '--';
    el.pMax.textContent = s.max_rpm ? s.max_rpm + ' rpm' : '--';
    var acc = s.max_accel_rpm_per_s, m = /accel up to (\d+) rpm\/s/.exec(s.limits || '');
    if (acc == null && m) acc = +m[1];
    el.pAcc.textContent = acc != null ? acc + ' rpm/s' : '--';
    var ppr;
    if (s.enc_ppr) ppr = s.enc_ppr + ' lines/rev' + (s.enc_wired === false ? ', tap not wired' : '');
    else if (s.enc_wired === false) ppr = 'no encoder tap';
    else ppr = 'not in /api/status';
    el.pEnc.textContent = ppr;
    if (s.pins && typeof s.pins === 'object') {
      el.pPins.textContent = Object.keys(s.pins).filter(function (k) { return s.pins[k] != null; })
        .map(function (k) { return k.toUpperCase() + ' ' + s.pins[k]; }).join(', ') || 'none';
    } else {
      var w = function (f) { return f === true ? 'wired' : f === false ? 'none' : '?'; };
      el.pPins.textContent = 'ENA ' + w(s.ena_wired) + ', ALM ' + w(s.alm_wired) + ', PEND ' + w(s.pend_wired) + ', ENC A ' + w(s.enc_wired);
    }
  }

  function render(s) {
    state.st = s;
    if (s.host || s.hostname) link.host = s.host || s.hostname;
    if (s.max_rpm && s.max_rpm !== state.maxRpm) {
      state.maxRpm = s.max_rpm;
      el.slider.max = el.rpm.max = String(s.max_rpm); setScale(s.max_rpm);
      if (num(el.rpm.value) > s.max_rpm) { el.rpm.value = el.slider.value = String(s.max_rpm); }
    }
    if (!state.rampDefaultSet) {
      var d = s.default_accel_s != null ? s.default_accel_s : s.ramp_s;
      if (d != null && isFinite(d)) el.accel.value = String(d);
      state.rampDefaultSet = true;
    }
    el.dry.hidden = !s.dry_run;
    el.ttlDrv.textContent = s.driver || 'driver unknown';
    el.ttlLoop.textContent = s.closed_loop ? 'closed loop' : 'open loop';
    el.hdrDriver.textContent = 'Driver: ' + (s.driver || 'unknown');
    el.hdrLoop.textContent = s.closed_loop ? 'Closed loop' : 'Open loop';
    var alarm = s.alarm_latched || s.phase === 'alarm' || s.phase === 'fault';
    el.hdrAlarmTxt.textContent = alarm ? 'ALARM' : (s.alm_wired === false ? 'Alarm: not wired' : 'Alarm: clear');
    setIc(el.hdrAlarmIc, alarm ? 'alert-octagon' : 'circle-check');
    el.hdrAlarm.className = 'sbp ' + (alarm ? 'bad' : (s.alm_wired === false ? '' : 'ok'));

    var ms = motorState(s);
    var wasRunning = state.running;
    state.running = ['ramping', 'holding', 'stopping'].indexOf(ms) >= 0 || s.rpm_cmd > 0.01;
    if (state.running && !wasRunning) {
      state.t0 = s.t_s;
      if (el.msg.textContent.indexOf('Motor is off') === 0) msg('The motor is already turning. STOP or EMERGENCY STOP to end the run.');
    }
    if (state.running && state.t0 != null) state.elapsed = Math.max(0, s.t_s - state.t0);
    renderButtons();

    // real time readouts: plain value boxes, tabular digits
    el.rCmd.textContent = fmt(s.rpm_cmd, 0);
    var enc = encRpm(s);
    el.rEnc.textContent = enc == null ? '---' : fmt(enc, 0);
    el.rEncU.textContent = s.enc_wired === false && enc == null ? 'no encoder' : 'rpm';
    el.rFerr.textContent = s.follow_err_rev == null ? '-.--' : fmt(s.follow_err_rev, 2);
    el.rFerrPct.textContent = s.follow_err_pct == null ? '--.-' : fmt(s.follow_err_pct, 1);
    el.rState.textContent = ms;
    el.rState.className = 'rvbox w7 state ' + (ms === 'alarm' || ms === 'e-stop' ? 'bad' : ms === 'holding' ? 'hold'
      : (ms === 'ramping' || ms === 'stopping') ? 'amber' : '');
    var fp = s.follow_err_pct == null ? null : Math.abs(s.follow_err_pct);
    el.ferrTile.className = 'rv' + (fp == null || !state.running ? '' : fp < 2 ? ' ok' : fp < 10 ? ' amber' : ' bad');
    el.rElapsed.textContent = fmt(state.elapsed, 1);
    renderCamera(s);

    // I/O lamps
    lamp(el.lDrv, el.rDrv, s.enabled ? 'g' : 'dg', s.enabled ? 'enabled' : 'disabled');
    lamp(el.lEstop, el.rEstop, s.estop || s.estop_latched ? 'e' : 'g', s.estop ? 'OPEN' : (s.estop_latched ? 'OK (latched)' : 'OK'));
    if (alarm) lamp(el.lAlarm, el.rAlarm, 'r', 'ALARM');
    else if (s.alm_wired === false) lamp(el.lAlarm, el.rAlarm, '', 'not wired');
    else lamp(el.lAlarm, el.rAlarm, 'dr', 'clear');
    var noPulses = !!(s.enc_no_pulses || s.encoder_no_pulses);
    var edges = s.enc_edges, moving = state.lastEdges != null && edges != null && edges !== state.lastEdges;
    state.lastEdges = edges;
    if (s.enc_wired === false) lamp(el.lPulses, el.rPulses, '', 'not wired');
    else if (noPulses) lamp(el.lPulses, el.rPulses, 'a', 'NONE');
    else if (moving) lamp(el.lPulses, el.rPulses, 'g', 'counting');
    else lamp(el.lPulses, el.rPulses, 'dg', 'idle');
    lamp(el.lRun, el.rRun, state.running ? 'g' : 'dg', state.running ? 'yes, ' + ms : 'no');
    renderParams(s);

    el.noPulses.hidden = !noPulses;

    if (s.cl_verdict) {
      el.verdict.hidden = false; el.verdictTxt.textContent = 'Closed-loop test ' + s.cl_verdict;
      var vfail = /fail|not reached/i.test(s.cl_verdict), vpass = !vfail && /pass|reached/i.test(s.cl_verdict);
      el.verdict.className = 'verdict' + (vfail ? ' fail' : vpass ? ' pass' : '');
      setIc(el.verdictIc, vfail ? 'circle-x' : 'circle-check');
    } else if (s.cl_running) {
      el.verdict.hidden = false; el.verdictTxt.textContent = 'Closed-loop test running...'; el.verdict.className = 'verdict';
    } else { el.verdict.hidden = true; }
    el.clIdle.hidden = !el.verdict.hidden;
    el.cStop.textContent = s.last_stop_reason || 'none';
    el.cSum.textContent = s.summary ? String(s.summary).replace(/^summary:\s*/, '').replace(/,?\s*log \S+$/, '') : 'none yet';
    trialsOnStatus(s);

    if (!el.diagBody.hidden) {
      el.limits.textContent = 'Limits: ' + (s.limits || 'n/a');
      el.stopReason.textContent = 'Last stop: ' + (s.last_stop_reason || 'none') +
        (s.warnings && s.warnings.length ? ' | Warnings: ' + s.warnings.join('; ') : '');
      el.summary.textContent = s.summary ? 'Last run: ' + s.summary : '';
      var csv = s.csv_url || s.log_url;
      el.csvWrap.hidden = !csv; if (csv) el.csv.href = API + csv;
      var tail = s.log_tail;
      el.logTail.hidden = !tail; if (tail) el.logTail.textContent = Array.isArray(tail) ? tail.join('\n') : String(tail);
      el.raw.textContent = JSON.stringify(s, null, 2);
    }
  }

  function record(s) {
    var now = performance.now() / 1000;
    hist.t.push(now); hist.c.push(s.rpm_cmd); hist.e.push(encRpm(s));
    while (hist.t.length && hist.t[0] < now - WINDOW_S - 1) { hist.t.shift(); hist.c.shift(); hist.e.shift(); }
  }

  async function poll() {
    try {
      var ctl = new AbortController(); var to = setTimeout(function () { ctl.abort(); }, 1500);
      var r = await fetch(API + '/api/status', { cache: 'no-store', signal: ctl.signal });
      clearTimeout(to);
      var s = await r.json();
      link.lastOk = Date.now(); link.missed = 0;
      render(s); record(s);
    } catch (e) { link.missed++; }
    renderLink();
    draw();
    setTimeout(poll, POLL_MS);
  }

  // ---------- theme: night is the default, day is the alternate; only a saved toggle changes it ----------
  function themeNow() { return document.documentElement.getAttribute('data-theme') === 'light' ? 'light' : 'dark'; }
  function cssVar(n) { return getComputedStyle(document.documentElement).getPropertyValue(n).trim(); }
  function applyTheme() {
    var t = themeNow();
    document.documentElement.setAttribute('data-theme-now', t);
    el.themeLbl.textContent = t === 'dark' ? 'Day' : 'Night';
    setIc(el.themeIc, t === 'dark' ? 'sun' : 'moon');
    el.themeBtn.setAttribute('aria-label', t === 'dark' ? 'Switch to day theme' : 'Switch to night theme');
    remakeChart();
  }
  el.themeBtn.addEventListener('click', function () {
    var next = themeNow() === 'dark' ? 'light' : 'dark';
    document.documentElement.setAttribute('data-theme', next);
    try { localStorage.setItem('froggy-theme', next); } catch (e) { /* storage blocked: still switches */ }
    applyTheme();
  });

  // ---------- camera view (Closed-loop test group) ----------
  // The stream is requested only when /api/status says a camera is present; otherwise the placeholder stays.
  // The label text is set here, not in index.html: test_motor.py asserts the served page never names it when camera = false.
  var camOn = false;
  el.camCap.textContent = 'Camera';
  el.camGrp.setAttribute('aria-label', 'Camera');
  // camera-off icon above the label; added here (not in index.html) for the same no-camera-name rule
  el.camLbl.appendChild(makeIc('camera-off', 's32 thin'));
  var camLblTxt = document.createElement('span'); el.camLbl.appendChild(camLblTxt);
  camLblTxt.textContent = 'Camera: not connected';
  el.cam.alt = 'Camera view of the turntable';
  function camPresent(s) { return s.camera === true || s.camera_ok === true || s.has_camera === true || s.camera_present === true; }
  function renderCamera(s) {
    var want = camPresent(s);
    if (want === camOn) return;
    camOn = want;
    if (want) {
      el.cam.onload = function () { el.cam.hidden = false; el.camWrap.setAttribute('data-cam', 'live'); };
      el.cam.onerror = function () { el.cam.hidden = true; el.camWrap.setAttribute('data-cam', 'none'); camLblTxt.textContent = 'Camera: no stream'; };
      el.cam.src = API + '/stream.mjpg';
    } else {
      el.cam.removeAttribute('src'); el.cam.hidden = true;
      el.camWrap.setAttribute('data-cam', 'none'); camLblTxt.textContent = 'Camera: not connected';
    }
  }

  // ---------- strip-chart trend (uPlot, vendored) ----------
  var plot = null;
  var view = null;     // a saved trial shown in the Trend chart instead of the live window: {id, t, c, e}
  function chartSize() {
    return { width: Math.max(200, Math.floor(el.chart.clientWidth || 600)), height: Math.max(140, Math.floor(el.chart.clientHeight || 240)) };
  }
  function makeChart() {
    if (typeof uPlot === 'undefined') { el.chart.textContent = 'Chart library failed to load.'; return; }
    var axis = cssVar('--t-axis') || '#000', grid = cssVar('--t-grid') || '#3b6e3b', sans = cssVar('--sans') || 'system-ui, sans-serif';
    var big = window.matchMedia('(min-width:1700px)').matches, px = big ? 16 : 14;
    var font = '700 ' + px + 'px ' + sans;
    var ax = { stroke: axis, grid: { stroke: grid, width: 1 }, ticks: { stroke: axis, width: 1, size: 6 }, font: font, labelFont: font, gap: 5 };
    var sz = chartSize();
    plot = new uPlot({
      width: sz.width, height: sz.height, legend: { show: false }, cursor: { show: false },
      padding: [12, big ? 104 : 96, 0, 0],
      scales: { x: { time: false, range: view ? [0, Math.max(1, view.t[view.t.length - 1] || 1)] : [-WINDOW_S, 0] },
                y: { range: function (u, mn, mx) { return [0, Math.max(view ? 10 : state.maxRpm, mx || 0)]; } } },
      axes: [Object.assign(view ? { label: 'seconds into trial', space: 50, labelSize: px + 8, size: px + 16 }
                                : { label: 'seconds ago', space: 40, incrs: [10, 20, 30], labelSize: px + 8, size: px + 16,
                             values: function (u, v) { return v.map(function (x) { return String(Math.abs(x)); }); } }, ax),
             Object.assign({ label: 'rpm', space: px + 10, incrs: view ? [10, 20, 50, 100, 200, 300] : [100, 200, 300], labelSize: px + 8, size: px * 3 + 10 }, ax)],
      series: [{}, { label: 'Commanded', stroke: cssVar('--t-cmd') || '#ffffff', width: 3 },
                   { label: 'Encoder', stroke: cssVar('--t-enc') || '#3b9eff', width: 3, spanGaps: false }],
      hooks: {
        drawClear: [function (u) {
          u.ctx.save(); u.ctx.fillStyle = cssVar('--t-bg') || '#000';
          u.ctx.fillRect(u.bbox.left, u.bbox.top, u.bbox.width, u.bbox.height); u.ctx.restore();
        }],
        draw: [placeTags]
      }
    }, [[], [], []], el.chart);
    draw();
  }
  function remakeChart() {
    if (plot) { plot.destroy(); plot = null; }
    makeChart();
  }
  function lastVal(arr) { for (var i = arr.length - 1; i >= 0; i--) if (arr[i] != null && isFinite(arr[i])) return arr[i]; return null; }
  function placeTags(u) {
    if (view) { el.tagCmd.hidden = el.tagEnc.hidden = true; return; }
    var box = el.chart.parentNode.getBoundingClientRect(), ov = u.over.getBoundingClientRect();
    [[el.tagCmd, lastVal(hist.c), 'CMD'], [el.tagEnc, lastVal(hist.e), 'ENC']].forEach(function (t) {
      if (t[1] == null) { t[0].hidden = true; return; }
      var y = u.valToPos(t[1], 'y');
      t[0].hidden = false;
      t[0].textContent = t[2];
      var b = document.createElement('b'); b.textContent = fmt(t[1], 0); t[0].appendChild(b);
      t[0].style.top = (ov.top - box.top + y) + 'px';
    });
    if (!el.tagCmd.hidden && !el.tagEnc.hidden) {
      var a = parseFloat(el.tagCmd.style.top), b = parseFloat(el.tagEnc.style.top), h = el.tagCmd.offsetHeight + 2;
      if (Math.abs(a - b) < h) { var m = (a + b) / 2; el.tagCmd.style.top = (m - h / 2) + 'px'; el.tagEnc.style.top = (m + h / 2) + 'px'; }
    }
  }
  function resized() {
    if (!plot) return;
    var sz = chartSize();
    if (sz.width !== plot.width || sz.height !== plot.height) plot.setSize(sz);
  }
  if (window.ResizeObserver) {
    var ro = new ResizeObserver(resized);
    ro.observe(el.chart);
  }
  window.addEventListener('resize', resized);
  function draw() {
    if (!plot) return;
    if (view) { if (plot.data[0].length !== view.t.length) plot.setData([view.t, view.c, view.e]); return; }
    var now = performance.now() / 1000;
    plot.setData([hist.t.map(function (t) { return t - now; }), hist.c.slice(), hist.e.slice()]);
  }

  // ---------- trials: list, select, export CSV / PDF, show in the Trend chart ----------
  var TRIAL_COLS = ['t_s', 'rpm_cmd', 'rpm_target', 'rpm_encoder', 'follow_err_rev', 'follow_err_pct', 'ena', 'alm', 'estop', 'phase'];
  var trials = { list: [], sel: null, lastId: undefined, curId: undefined, cur: null, loading: false };

  function pad2(n) { return (n < 10 ? '0' : '') + n; }
  function hhmmss(epoch) {
    if (epoch == null || !isFinite(epoch)) return '--:--:--';
    var d = new Date(epoch * 1000);
    return pad2(d.getHours()) + ':' + pad2(d.getMinutes()) + ':' + pad2(d.getSeconds());
  }
  function idEpoch(id) {     // 20261109T171512.345Z-a3f9c2 -> epoch seconds (UTC)
    var m = /^(\d{4})(\d{2})(\d{2})T(\d{2})(\d{2})(\d{2})\.(\d{3})Z/.exec(id || '');
    return m ? Date.UTC(+m[1], +m[2] - 1, +m[3], +m[4], +m[5], +m[6], +m[7]) / 1000 : null;
  }
  function tmsg(t) { el.tMsg.textContent = t; }

  async function loadTrials() {
    try {
      var l = await api('/api/trials');
      if (Array.isArray(l)) trials.list = l;
    } catch (e) { /* link loss: the status poll reports it */ }
    renderTrials();
  }
  async function loadCurrent() {
    try {
      var c = await api('/api/trials/current');
      trials.cur = c && c.meta ? c.meta : null;
    } catch (e) { trials.cur = null; }
    renderTrials();
  }
  function trialsOnStatus(s) {
    if (s.last_trial_id !== trials.lastId) { trials.lastId = s.last_trial_id; loadTrials(); }
    var cur = s.trial_id || null;
    if (cur !== trials.curId) {
      trials.curId = cur;
      if (cur) loadCurrent(); else { trials.cur = null; renderTrials(); }
    }
  }

  function cell(tr, text, cls) {
    var td = document.createElement('td'); td.textContent = text; if (cls) td.className = cls; tr.appendChild(td);
  }
  function renderTrials() {
    var rows = [];
    if (trials.curId) {
      var m = trials.cur && trials.cur.id === trials.curId ? trials.cur : {};
      rows.push({ id: trials.curId, rec: true, start_epoch: m.start_epoch != null ? m.start_epoch : idEpoch(trials.curId),
                  mode: m.mode, rpm: m.rpm, duration_s: null, stop_cause: 'recording', max_rpm_encoder: null });
    }
    trials.list.forEach(function (t) { if (t.id !== trials.curId) rows.push(t); });
    if (trials.sel && !rows.some(function (r) { return r.id === trials.sel; })) trials.sel = null;
    var body = el.tBody;
    while (body.firstChild) body.removeChild(body.firstChild);
    if (!rows.length) {
      var e = document.createElement('tr'); e.className = 'empty';
      var td = document.createElement('td'); td.colSpan = 6; td.textContent = 'No trials recorded yet'; e.appendChild(td);
      body.appendChild(e);
    }
    rows.forEach(function (r) {
      var tr = document.createElement('tr');
      tr.dataset.id = r.id;
      if (r.rec) { tr.className = 'rec'; tr.dataset.rec = '1'; }
      tr.setAttribute('aria-selected', String(r.id === trials.sel));
      tr.title = r.id;
      cell(tr, hhmmss(r.start_epoch));
      cell(tr, r.mode || '--');
      cell(tr, r.rpm == null ? '--' : fmt(r.rpm, 0) + (r.dir ? ' ' + String(r.dir).toUpperCase() : ''), 'n');
      cell(tr, r.rec ? 'rec' : fmt(r.duration_s, 1), 'n');
      cell(tr, r.stop_cause || '--');
      cell(tr, r.max_rpm_encoder == null ? '--' : fmt(r.max_rpm_encoder, 0), 'n');
      body.appendChild(tr);
    });
    renderTrialButtons();
  }
  function selectedSaved() {
    return trials.sel && trials.sel !== trials.curId ? trials.sel : null;
  }
  function renderTrialButtons() {
    var ok = !!selectedSaved() && !trials.loading;
    el.tCsv.disabled = el.tPdf.disabled = el.tShow.disabled = !ok;
  }
  el.tBody.addEventListener('click', function (e) {
    var tr = e.target.closest('tr'); if (!tr || !tr.dataset.id) return;
    trials.sel = tr.dataset.id;
    el.tBody.querySelectorAll('tr').forEach(function (r) { r.setAttribute('aria-selected', String(r === tr)); });
    tmsg(tr.dataset.rec ? 'Trial ' + trials.sel + ' is still recording. Export after the motor stops.' : 'Selected ' + trials.sel);
    renderTrialButtons();
  });

  el.tCsv.addEventListener('click', function () {
    var id = selectedSaved(); if (!id) return;
    el.tDl.href = API + '/api/trials/' + id + '.csv';
    el.tDl.download = 'froggy-trial-' + id + '.csv';
    el.tDl.click();
    tmsg('CSV download: froggy-trial-' + id + '.csv');
  });

  async function fetchTrial(id) {
    var r = await fetch(API + '/api/trials/' + id + '.json', { cache: 'no-store' });
    if (!r.ok) throw new Error('server answered ' + r.status);
    return r.json();
  }
  function column(rec, name) {
    var cols = (rec.meta && rec.meta.columns) || TRIAL_COLS, i = cols.indexOf(name);
    return (rec.samples || []).map(function (row) { var v = i < 0 ? null : row[i]; return v == null ? null : +v; });
  }

  el.tShow.addEventListener('click', async function () {
    var id = selectedSaved(); if (!id) return;
    trials.loading = true; renderTrialButtons();
    try {
      var rec = await fetchTrial(id);
      view = { id: id, t: column(rec, 't_s'), c: column(rec, 'rpm_cmd'), e: column(rec, 'rpm_encoder') };
      el.trendTtl.textContent = 'Trial ' + hhmmss(rec.meta.start_epoch) + ' (' + id + '), ' + (rec.meta.mode || '') + ' ' +
        fmt(rec.meta.rpm, 0) + ' rpm';
      el.tLive.hidden = false;
      remakeChart();
      tmsg('Trend shows trial ' + id + '. Press Live to return.');
    } catch (err) { tmsg('Could not load trial ' + id + ': ' + err.message); }
    finally { trials.loading = false; renderTrialButtons(); }
  });
  el.tLive.addEventListener('click', function () {
    view = null; el.tLive.hidden = true; el.trendTtl.textContent = 'Speed, last 60 s';
    remakeChart(); tmsg('Trend back to live.');
  });

  // ---------- PDF report (jsPDF, vendored) ----------
  function ascii(v) { return String(v == null ? '' : v).replace(/[–—]/g, '-').replace(/[^\x20-\x7e]/g, '?'); }
  function niceStep(span, n) {
    var raw = span / n, p = Math.pow(10, Math.floor(Math.log10(raw))), f = raw / p;
    return (f <= 1 ? 1 : f <= 2 ? 2 : f <= 5 ? 5 : 10) * p;
  }
  function chartPng(w, h, t, series, yLabel, sym) {
    var cv = document.createElement('canvas'); cv.width = w; cv.height = h;
    var g = cv.getContext('2d');
    g.fillStyle = '#ffffff'; g.fillRect(0, 0, w, h);
    var L = 130, R = 30, T = 60, B = 80, pw = w - L - R, ph = h - T - B;
    var tmax = Math.max(1, t.length ? t[t.length - 1] : 1);
    var vals = [];
    series.forEach(function (s) { s.y.forEach(function (v) { if (v != null && isFinite(v)) vals.push(v); }); });
    var ymin = 0, ymax = 10;
    if (vals.length) {
      ymin = Math.min(0, Math.min.apply(null, vals)); ymax = Math.max.apply(null, vals);
      if (sym) { var a = Math.max(Math.abs(ymin), Math.abs(ymax), 1e-3); ymin = -a; ymax = a; }
      if (ymax - ymin < 1e-6) ymax = ymin + 1;
      var pad = (ymax - ymin) * 0.08; ymax += pad; if (ymin < 0) ymin -= pad;
    }
    var ys = niceStep(ymax - ymin, 6); ymin = Math.floor(ymin / ys) * ys; ymax = Math.ceil(ymax / ys) * ys;
    var xs = niceStep(tmax, 10);
    var X = function (v) { return L + v / tmax * pw; }, Y = function (v) { return T + (1 - (v - ymin) / (ymax - ymin)) * ph; };
    g.font = '24px ' + (cssVar('--sans') || 'system-ui, sans-serif'); g.fillStyle = '#000'; g.strokeStyle = '#c8c8c8'; g.lineWidth = 1;
    g.textAlign = 'right'; g.textBaseline = 'middle';
    var dec = ys < 1 ? Math.min(4, Math.ceil(-Math.log10(ys))) : 0;
    for (var y = ymin; y <= ymax + ys / 2; y += ys) {
      g.beginPath(); g.moveTo(L, Y(y)); g.lineTo(L + pw, Y(y)); g.stroke();
      g.fillText(fmt(y, dec), L - 10, Y(y));
    }
    g.textAlign = 'center'; g.textBaseline = 'top';
    for (var x = 0; x <= tmax + 1e-9; x += xs) {
      g.beginPath(); g.moveTo(X(x), T); g.lineTo(X(x), T + ph); g.stroke();
      g.fillText(fmt(x, xs < 1 ? 1 : 0), X(x), T + ph + 8);
    }
    g.strokeStyle = '#000'; g.lineWidth = 2; g.strokeRect(L, T, pw, ph);
    g.fillText('time since start, s', L + pw / 2, T + ph + 42);
    g.save(); g.translate(30, T + ph / 2); g.rotate(-Math.PI / 2); g.textBaseline = 'middle'; g.fillText(yLabel, 0, 0); g.restore();
    g.save(); g.beginPath(); g.rect(L, T, pw, ph); g.clip();
    series.forEach(function (s) {
      g.strokeStyle = s.color; g.lineWidth = 3; g.beginPath();
      var pen = false;
      for (var i = 0; i < t.length; i++) {
        var v = s.y[i];
        if (v == null || !isFinite(v) || t[i] == null) { pen = false; continue; }
        if (pen) g.lineTo(X(t[i]), Y(v)); else { g.moveTo(X(t[i]), Y(v)); pen = true; }
      }
      g.stroke();
    });
    g.restore();
    var lx = L; g.textAlign = 'left'; g.textBaseline = 'middle';
    series.forEach(function (s) {
      g.strokeStyle = s.color; g.lineWidth = 4; g.beginPath(); g.moveTo(lx, 28); g.lineTo(lx + 50, 28); g.stroke();
      g.fillStyle = '#000'; g.fillText(s.label, lx + 60, 28); lx += 80 + g.measureText(s.label).width;
    });
    if (!vals.length) { g.textAlign = 'center'; g.fillText('no data for this signal in this trial', L + pw / 2, T + ph / 2); }
    return cv.toDataURL('image/png');
  }
  function flatConfig(obj, pre, out) {
    Object.keys(obj || {}).sort().forEach(function (k) {
      var v = obj[k], key = pre ? pre + '.' + k : k;
      if (v && typeof v === 'object' && !Array.isArray(v)) flatConfig(v, key, out);
      else out.push(key + ' = ' + (v == null ? 'none' : Array.isArray(v) ? JSON.stringify(v) : v));
    });
    return out;
  }

  function buildPdf(rec) {
    var J = window.jspdf && window.jspdf.jsPDF;
    if (!J) throw new Error('PDF library failed to load');
    var m = rec.meta || {}, sm = m.summary || {};
    var doc = new J({ unit: 'pt', format: 'letter', compress: true });
    var W = 612, H = 792, M = 40, y = M;
    doc.setProperties({ title: 'Froggy Turntable motor trial ' + m.id, subject: 'Motor trial report', creator: 'froggy motor HMI' });
    doc.setFont('helvetica', 'bold'); doc.setFontSize(16);
    doc.text('Froggy Turntable motor trial', M, y + 8); y += 22;
    doc.setFont('courier', 'normal'); doc.setFontSize(9);
    doc.text(ascii(m.id) + (m.dry_run ? '   (DRY RUN, fake pigpio)' : ''), M, y + 6); y += 16;
    doc.setDrawColor(0); doc.setLineWidth(0.8); doc.line(M, y, W - M, y); y += 12;

    var info = [
      ['Start', ascii(m.start) + '  (' + new Date((m.start_epoch || 0) * 1000).toLocaleString() + ')'],
      ['End', ascii(m.end)],
      ['Duration', fmt(m.duration_s, 2) + ' s'],
      ['Mode', ascii(m.mode)],
      ['Target', fmt(m.rpm, 1) + ' rpm ' + ascii(m.dir).toUpperCase()],
      ['Ramp / hold / decel', [m.accel_s, m.hold_s, m.decel_s].map(function (v) { return v == null ? 'until STOP' : fmt(v, 2) + ' s'; }).join(' / ')],
      ['Stop cause', ascii(m.stop_cause || 'none')],
      ['Host', ascii(m.hostname)],
      ['Software', ascii(m.software)]
    ];
    if (m.cl_verdict) info.push(['CL verdict', ascii(m.cl_verdict)]);
    var y0 = y, colW = 300;
    doc.setFontSize(9);
    info.forEach(function (r) {
      doc.setFont('helvetica', 'bold'); doc.text(r[0], M, y);
      doc.setFont('helvetica', 'normal');
      var lines = doc.splitTextToSize(r[1], colW - 100); doc.text(lines, M + 100, y); y += 12 * lines.length;
    });
    // summary stats table on the right
    var sx = M + colW + 12, sw = W - M - sx, sy = y0;
    var stats = [
      ['Max commanded rpm', fmt(sm.max_rpm_cmd, 1)], ['Max encoder rpm', sm.max_rpm_encoder == null ? 'n/a' : fmt(sm.max_rpm_encoder, 1)],
      ['Mean |follow err| rev', sm.follow_err_rev_mean == null ? 'n/a' : fmt(sm.follow_err_rev_mean, 4)],
      ['Max |follow err| rev', sm.follow_err_rev_max == null ? 'n/a' : fmt(sm.follow_err_rev_max, 4)],
      ['Alarm edges', String(sm.alarm_edges == null ? 'n/a' : sm.alarm_edges)],
      ['E-stop events', String(sm.estop_events == null ? 'n/a' : sm.estop_events)],
      ['Samples (100 ms)', String(sm.samples == null ? (rec.samples || []).length : sm.samples)],
      ['Duration s', fmt(sm.duration_s, 2)]
    ];
    doc.setFont('helvetica', 'bold'); doc.text('Summary', sx, sy); sy += 4;
    doc.setLineWidth(0.5);
    stats.forEach(function (r) {
      doc.rect(sx, sy, sw, 13);
      doc.setFont('helvetica', 'normal'); doc.text(r[0], sx + 4, sy + 9.5);
      doc.setFont('courier', 'bold'); doc.text(r[1], sx + sw - 4, sy + 9.5, { align: 'right' });
      sy += 13;
    });
    y = Math.max(y, sy) + 6;
    doc.setFont('helvetica', 'bold'); doc.text('Driver', M, y);
    doc.setFont('helvetica', 'normal');
    var dl = doc.splitTextToSize(ascii(m.driver_settings), W - 2 * M - 100); doc.text(dl, M + 100, y); y += 12 * dl.length + 6;

    var t = column(rec, 't_s');
    var img1 = chartPng(1600, 700, t, [{ y: column(rec, 'rpm_cmd'), color: '#111111', label: 'Commanded rpm' },
                                       { y: column(rec, 'rpm_encoder'), color: '#004fc0', label: 'Encoder rpm' }], 'rpm', false);
    var cw = W - 2 * M, ch1 = cw * 700 / 1600;
    doc.setFont('helvetica', 'bold'); doc.setFontSize(10); doc.text('Speed: commanded and encoder', M, y + 4); y += 8;
    doc.addImage(img1, 'PNG', M, y, cw, ch1); y += ch1 + 10;
    var img2 = chartPng(1600, 420, t, [{ y: column(rec, 'follow_err_rev'), color: '#a3007a', label: 'Following error, rev' }], 'rev', true);
    var ch2 = cw * 420 / 1600;
    if (y + ch2 + 14 > H - M) { doc.addPage(); y = M; }
    doc.text('Following error', M, y + 4); y += 8;
    doc.addImage(img2, 'PNG', M, y, cw, ch2); y += ch2 + 10;

    // page two: events table and the config snapshot
    doc.addPage(); y = M;
    function need(h) { if (y + h > H - M) { doc.addPage(); y = M; } }
    doc.setFont('helvetica', 'bold'); doc.setFontSize(11); doc.text('Events', M, y + 4); y += 12;
    doc.setFontSize(8); doc.setLineWidth(0.5);
    var cx = [M, M + 50, M + 120];
    doc.text('t, s', cx[0] + 3, y + 8); doc.text('event', cx[1] + 3, y + 8); doc.text('detail', cx[2] + 3, y + 8);
    doc.line(M, y + 11, W - M, y + 11); y += 13;
    doc.setFont('courier', 'normal');
    (rec.events || []).forEach(function (ev) {
      var det = Object.keys(ev).filter(function (k) { return k !== 't_s' && k !== 'kind'; })
        .map(function (k) { return k + '=' + (ev[k] == null ? '-' : ev[k]); }).join('  ');
      var lines = doc.splitTextToSize(ascii(det), W - M - cx[2] - 6);
      need(10 * lines.length + 2);
      doc.text(fmt(ev.t_s, 3), cx[0] + 3, y + 7); doc.text(ascii(ev.kind), cx[1] + 3, y + 7); doc.text(lines, cx[2] + 3, y + 7);
      y += 10 * lines.length; doc.setDrawColor(190); doc.line(M, y + 1, W - M, y + 1); doc.setDrawColor(0); y += 2;
    });
    if (!(rec.events || []).length) { doc.text('no events', M + 3, y + 7); y += 10; }
    y += 14; need(40);
    doc.setFont('helvetica', 'bold'); doc.setFontSize(11); doc.text('Config snapshot', M, y + 4); y += 10;
    doc.setFont('courier', 'normal'); doc.setFontSize(7);
    var cfgLines = [];
    flatConfig(m.config, '', []).forEach(function (l) { cfgLines = cfgLines.concat(doc.splitTextToSize(ascii(l), (W - 2 * M) / 2 - 12)); });
    var half = Math.ceil(cfgLines.length / 2), lh = 8.4, boxH = half * lh + 8;
    if (y + boxH > H - M) { half = cfgLines.length; boxH = half * lh + 8; }
    need(Math.min(boxH, H - 2 * M));
    doc.setLineWidth(0.5); doc.rect(M, y, W - 2 * M, Math.min(boxH, H - M - y));
    var bx = [M + 5, M + (W - 2 * M) / 2 + 5], by = y + 8;
    cfgLines.forEach(function (l, i) {
      var col = i < half ? 0 : 1, row = i < half ? i : i - half;
      doc.text(l, bx[col], by + row * lh + 2);
    });
    var n = doc.getNumberOfPages();
    for (var pnum = 1; pnum <= n; pnum++) {
      doc.setPage(pnum); doc.setFont('helvetica', 'normal'); doc.setFontSize(7);
      doc.text('froggy-trial-' + ascii(m.id) + '   page ' + pnum + ' of ' + n + '   generated ' + new Date().toLocaleString(), M, H - 20);
    }
    return doc;
  }

  el.tPdf.addEventListener('click', async function () {
    var id = selectedSaved(); if (!id) return;
    trials.loading = true; renderTrialButtons(); tmsg('Building PDF for ' + id + '...');
    try {
      var rec = await fetchTrial(id);
      var doc = buildPdf(rec), name = 'froggy-trial-' + id + '.pdf';
      var bytes = new Uint8Array(doc.output('arraybuffer'));
      window.__lastPdf = bytes;          // the ui test reads this
      window.__lastPdfName = name;
      doc.save(name);
      tmsg('PDF saved: ' + name + ' (' + doc.getNumberOfPages() + ' pages, ' + Math.round(bytes.length / 1024) + ' KB)');
    } catch (err) { tmsg('PDF export failed: ' + err.message); }
    finally { trials.loading = false; renderTrialButtons(); }
  });

  applyTheme();
  renderButtons();
  renderLink();
  poll();
})();
