/* fabrail glide scene (plan §3b). Owner: W2.
 *
 * What runs where:
 *   - The stage: the real Rust supervisor, S-curve planner, governor, terminus and cascade
 *     on the Carriage6 rigid-body plant (crates/fabrail-web through assets/js/engine.js),
 *     commanded with UserCommand Open / Close and stepped in real time. The distance to go
 *     |x_target − x| is plotted on a log axis against the closed-form terminus of
 *     PHYSICS §16.1 evaluated at the measured handover speed, and the capture window is
 *     fitted (ln d against t) for τ and R² as FAB-HAP-013 does.
 *   - Without the WebAssembly engine the stage animates the closed-form reference of the
 *     same move (formulas.js closeProfile) and says so.
 *   - S-curve explorer, governor and pinch dial: closed forms (PHYSICS §16, §18).
 *
 * Under node, require("docs/assets/js/scene-glide.js") returns the pure core:
 * { govCaps, planSpeed, capFor, semilogFit, pinchForce, K_BODY, recordMove, analyseMove }.
 */
(function (root) {
  "use strict";

  var TICK_HZ = 20000;
  var SAMPLE_TICKS = 40;          /* 2 ms samples while a move runs */
  var E_MAX_J = 1.69, P_MAX = 22.3;
  /* Governor planning fractions (PHYSICS §16; core motion::governor GOV_KE_PLAN_FRAC,
     GOV_P_PLAN_FRAC, supervisor GOV_PLAN_FRAC). */
  var KE_PLAN = 0.9, P_PLAN = 0.95, PLAN_FRAC = 0.98;
  var ARRIVE_D_M = 20e-6, ARRIVE_V_MPS = 1e-3;   /* FAB-HAP-016 arrival criteria */

  function formulas() {
    if (root.FabrailFormulas) return root.FabrailFormulas;
    if (typeof require === "function") {
      try { return require("./formulas.js"); } catch (e) { return null; }
    }
    return null;
  }
  var F = formulas();

  /* ISO/TS 15066:2016 Annex A body stiffness and the 10 N/mm edge seal, N/m (PHYSICS §18). */
  var K_BODY = {
    hand: 75e3, chest: 25e3,
    "hand-edge": 75e3 * 10e3 / (75e3 + 10e3),
    "chest-edge": 25e3 * 10e3 / (25e3 + 10e3)
  };

  /* Momentum basis: HEAVY has none (p = 0 means not applicable; presets). */
  function pMaxFor(name) { return name === "heavy" ? 0 : P_MAX; }

  /* Class cap for a direction (+1 open, −1 close): presets safety.v_open/close_max. */
  function capFor(c, dir) { return dir < 0 ? c.v_close_max_mps : c.v_open_max_mps; }

  /* Governor caps at m_gov = max(m_rated, 1.05·m̂) (PHYSICS §16; FAB-SAF-001). */
  function govCaps(name, mHatKg) {
    var c = F.CLASSES[name];
    var mGov = Math.max(c.mass_rated_kg, 1.05 * mHatKg);
    var p = pMaxFor(name);
    var vE = Math.sqrt((2 * E_MAX_J) / mGov);
    var vP = p > 0 ? p / mGov : Infinity;
    return {
      mGov: mGov, vE: vE, vP: vP,
      vEPlan: Math.sqrt(KE_PLAN) * vE, vPPlan: p > 0 ? P_PLAN * vP : Infinity,
      vCap: Math.min(vE, vP), vGov: Math.min(Math.sqrt(KE_PLAN) * vE, p > 0 ? P_PLAN * vP : Infinity)
    };
  }

  /* Planned cruise: PLAN_FRAC·(min(class cap, governor planning cap) − a_brk·t_react),
     a_brk = 2·a_max, t_react = 2 fast periods (PHYSICS §16, "Planning inside the envelope"). */
  function planSpeed(name, dir, mHatKg) {
    var c = F.CLASSES[name], g = govCaps(name, mHatKg);
    var tReact = (2 * c.fast_decim) / c.base_hz;
    var v = Math.min(capFor(c, dir), g.vGov) - 2 * c.a_max_mps2 * tReact;
    return PLAN_FRAC * Math.max(0, v);
  }

  /* Least-squares line through (t, ln d): τ = −1/slope and R². */
  function semilogFit(ts, ds) {
    var n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
    for (var i = 0; i < ts.length; i++) {
      if (!(ds[i] > 0)) continue;
      var x = ts[i], y = Math.log(ds[i]);
      n++; sx += x; sy += y; sxx += x * x; sxy += x * y; syy += y * y;
    }
    if (n < 5) return null;
    var cxx = sxx - (sx * sx) / n, cxy = sxy - (sx * sy) / n, cyy = syy - (sy * sy) / n;
    if (!(cxx > 0) || !(cyy > 0)) return null;
    var b = cxy / cxx;
    return { tau: -1 / b, r2: (cxy * cxy) / (cxx * cyy), n: n };
  }

  /* Quasi-static clamp force F = v·√(m·k) = √(2E·k) (PHYSICS §18; FAB-SAF-005). */
  function pinchForce(v, m, k) { return Math.abs(v) * Math.sqrt(m * k); }

  /* Terminus analysis of one recorded move: handover (first terminus-mode sample), the
     closed-form profile at the handover speed, the capture fit and the 20 µm arrival. */
  function analyseMove(rec, c) {
    var s = rec.samples, th = -1, vh = 0, i;
    for (i = 0; i < s.t.length; i++) {
      if (s.mode[i] === 10 || s.mode[i] === 11) { th = s.t[i]; vh = Math.abs(i > 0 ? s.v[i - 1] : s.v[i]); break; }
    }
    if (th < 0) return { th: null };
    var prof = F.terminusProfile(vh, c.j_term_mps3, c.tau_s, c.d_eps_m);
    var c0 = th + prof.tB, c1 = th + prof.tB + prof.tCapture;
    var ft = [], fd = [], tArr = null;
    for (i = 0; i < s.t.length; i++) {
      var d = Math.abs(rec.target - s.x[i]);
      if (s.t[i] >= c0 && s.t[i] <= c1) { ft.push(s.t[i]); fd.push(d); }
      if (tArr === null && s.t[i] > th && d < ARRIVE_D_M && Math.abs(s.v[i]) < ARRIVE_V_MPS) tArr = s.t[i] - th;
    }
    return { th: th, vh: vh, prof: prof, fit: semilogFit(ft, fd), tArrive: tArr, tArriveCf: prof.T,
      captureFrom: c0, captureTo: c1 };
  }

  /* Headless: command `cmd` on an engine handle and record until the move settles. */
  function recordMove(h, name, cmd, seconds) {
    var c = F.CLASSES[name], s0 = h.state(), dir = cmd === "open" ? 1 : -1;
    var rec = { cmd: cmd, dir: dir, target: dir > 0 ? c.travel_m : 0, t0: s0.t_s,
      samples: { t: [], x: [], v: [], mode: [], lev: [], e: [] } };
    h.setInputs({ command: cmd });
    var n = Math.round((seconds * TICK_HZ) / SAMPLE_TICKS);
    for (var k = 0; k < n; k++) {
      h.step(SAMPLE_TICKS);
      var st = h.state();
      pushSample(rec, st);
    }
    return rec;
  }

  function pushSample(rec, st) {
    var s = rec.samples;
    s.t.push(st.t_s - rec.t0); s.x.push(st.x_m); s.v.push(st.v_mps);
    s.mode.push(st.motion_mode); s.lev.push(st.lev_state); s.e.push(st.e_kin_gov_j);
  }

  var CORE = { govCaps: govCaps, planSpeed: planSpeed, capFor: capFor, semilogFit: semilogFit,
    pinchForce: pinchForce, K_BODY: K_BODY, recordMove: recordMove, analyseMove: analyseMove,
    pushSample: pushSample };
  if (typeof module === "object" && module.exports) module.exports = CORE;

  /* ==================================================================================== */
  /* DOM layer                                                                            */
  /* ==================================================================================== */

  var doc = root.document;
  if (!doc) return;
  var E = root.FabrailEngine, P = root.FabrailPlot, UI = root.FabrailUI;
  if (!F || !P || !UI) return;
  var fmt = UI.fmt, fmtSI = UI.fmtSI;

  function $(id) { return doc.getElementById(id); }
  function setText(id, s) { var el = $(id); if (el) el.textContent = s; }
  function clamp(v, a, b) { return Math.min(Math.max(v, a), b); }
  function mmStr(v, d) { return fmt(v * 1e3, d === undefined ? 4 : d) + " mm"; }
  function speedStr(v) { return Math.abs(v) < 1e-6 ? "0 mm/s" : Math.abs(v) < 0.01 ? fmt(v * 1e3, 3) + " mm/s" : fmt(v, 3) + " m/s"; }
  function wantForce() {
    try { return /(?:^|[?&])engine=wasm(?:&|$)/.test(root.location.search || ""); } catch (e) { return false; }
  }

  var LEV_NAMES = ["boot", "self-test", "seated", "landed", "lifting", "levitated, idle", "moving",
    "stopping", "landing", "seating", "fault reaction (SS1)", "fault (latched)", "seismic action",
    "escape", "manual"];
  var MODE_NAMES = ["propulsion off", "hold", "S-curve trajectory", "admittance", "soft-close", "reverse",
    "freeze", "centre", "isolate", "damp", "terminus (close)", "terminus (open)", "inertial smoothing"];

  /* ---- state ------------------------------------------------------------------------ */

  var S = {
    name: "drawer", c: F.CLASSES.drawer,
    engineMode: "loading", engineReason: "",
    h: null, st: null, tickAcc: 0, speed: 1,
    run: null, an: null, tail: 0, active: false, busy: false,
    fb: null,             /* closed-form fallback move */
    fbX: 0, lastChart: 0
  };

  var stage = $("gl-stage"), canvas = $("gl-canvas"), chip = $("gl-chip");
  var openBtn = $("gl-open"), closeBtn = $("gl-close");
  if (!stage || !canvas) return;

  var col = {}, geo = null, cv = null;
  function colours() {
    var names = ["deep", "panel", "ink", "ink2", "muted", "line", "grid", "axis", "accent", "good", "ref",
      "drawer", "door", "wall", "heavy", "s4"];
    for (var i = 0; i < names.length; i++) col[names[i]] = P.color(names[i]);
    col.flux = P.color("--flux"); col.steel = P.color("--steel");
    col.sans = P.theme().sans;
  }

  /* ---- engine ----------------------------------------------------------------------- */

  function destroyEngine() {
    if (S.h) { try { S.h.destroy(); } catch (e) { /* gone */ } }
    S.h = null;
  }

  function createEngine() {
    destroyEngine();
    S.run = null; S.an = null; S.fb = null; S.fbX = 0; S.tickAcc = 0; S.stopped = "";
    if (S.engineMode !== "wasm" || !E) return false;
    var h = E.open(S.name, "carriage6");
    if (!h) {
      S.engineMode = "formulas";
      S.engineReason = "the loaded engine has no Carriage6 handle (" + (E.lastError() || "ABI v2 needed") + ")";
      return false;
    }
    S.h = h;
    S.st = h.state();
    return true;
  }

  function liveX() {
    if (S.h && S.st) return S.st.x_m;
    return S.fbX;
  }

  function startMove(cmd) {
    var dir = cmd === "open" ? 1 : -1, c = S.c;
    var target = dir > 0 ? c.travel_m : 0;
    if (S.busy) return;
    if (Math.abs(target - liveX()) < 1e-3 && !(S.h && S.st && S.st.lev_state === 2 && dir > 0)) {
      setChip("Already " + (dir > 0 ? "open" : "closed") + ": press " + (dir > 0 ? "Close" : "Open") + ".");
      return;
    }
    if (S.h) {
      if (S.stopped) return;
      S.h.setInputs({ command: cmd });
      S.run = { cmd: cmd, dir: dir, target: target, t0: S.st ? S.st.t_s : 0,
        samples: { t: [], x: [], v: [], mode: [], lev: [], e: [] }, done: false };
    } else {
      var x0 = S.fbX, L = Math.abs(target - x0);
      if (L < 1e-4) return;
      var v = planSpeed(S.name, dir, c.m_star_kg);
      var prof = F.closeProfile({ lengthM: L, vMps: v, aMaxMps2: c.a_max_mps2, jMaxMps3: c.j_max_mps3,
        tauS: c.tau_s, jTermMps3: c.j_term_mps3, dEpsM: c.d_eps_m, seatS: S.name === "drawer" && dir < 0 ? 0.2 : 0 });
      S.fb = { prof: prof, x0: x0, dir: dir, target: target, t: 0 };
      S.run = { cmd: cmd, dir: dir, target: target, t0: 0, samples: { t: [], x: [], v: [], mode: [], lev: [], e: [] }, done: false, closed: true };
    }
    S.an = null; S.tail = 0; S.active = true;
    if (UI.reducedMotion()) computeAtOnce(); else loop.start();
    refreshCharts(true);
  }

  /* Advance simulated time dt; records samples while a move is active. */
  function advance(dt) {
    if (S.h) {
      if (S.stopped) return;
      S.tickAcc += dt * TICK_HZ;
      var n = Math.floor(S.tickAcc);
      S.tickAcc -= n;
      while (n > 0) {
        var k = Math.min(SAMPLE_TICKS, n);
        var got = S.h.step(k);
        n -= k;
        if (!got) break;
        S.st = S.h.state();
        if (S.run && !S.run.done) {
          pushSample(S.run, S.st);
          checkDone();
        }
      }
      var status = S.h.status();
      if (status && status !== "running") S.stopped = status + (E.lastError() ? ": " + E.lastError() : "");
    } else if (S.fb) {
      var fbs = S.fb, steps = Math.max(1, Math.round(dt / 0.002));
      for (var q = 0; q < steps; q++) {
        fbs.t += dt / steps;
        var smp = fbs.prof.sample(fbs.t);
        S.fbX = fbs.target - fbs.dir * smp.d;
        var ph = smp.phase, mode = ph === "accel" || ph === "cruise" ? 2 : ph === "blend" || ph === "capture" || ph === "quintic" ? (fbs.dir > 0 ? 11 : 10) : 1;
        if (S.run && !S.run.done) {
          var e = 0.5 * govCaps(S.name, S.c.m_star_kg).mGov * smp.v * smp.v;
          pushSample(S.run, { t_s: fbs.t, x_m: S.fbX, v_mps: fbs.dir * smp.v, motion_mode: mode, lev_state: 6, e_kin_gov_j: e });
          if (fbs.t >= fbs.prof.T) { S.run.done = true; S.an = analyseMove(S.run, S.c); }
        }
      }
    }
  }

  function checkDone() {
    var r = S.run, s = r.samples, n = s.t.length - 1;
    if (n < 10) return;
    var d = Math.abs(r.target - s.x[n]), mode = s.mode[n], lev = s.lev[n];
    var seated = lev === 2 || lev === 9;
    var sawTerminus = S.an && S.an.th !== null;
    if (!sawTerminus && (mode === 10 || mode === 11)) S.an = analyseMove(r, S.c);
    var inTerm = mode === 10 || mode === 11;
    if (!r.doneAt && ((sawTerminus && !inTerm) || seated || s.t[n] > 30 || (d < 2e-5 && Math.abs(s.v[n]) < 1e-3 && s.t[n] > 1))) r.doneAt = s.t[n];
    if (r.doneAt && s.t[n] - r.doneAt > 0.4) { r.done = true; S.an = analyseMove(r, S.c); }
    if (lev >= 10 && lev <= 11) { r.done = true; S.an = analyseMove(r, S.c); }
  }

  /* Reduced motion: step the whole move at once (in slices, so the page stays responsive)
     and draw its final frame with complete charts. */
  function computeAtOnce() {
    if (S.busy) return;
    S.busy = true;
    setChip("Computing the move (reduced motion: no animation, final frame only)…");
    var budget = 0;
    function slice() {
      for (var k = 0; k < 25 && S.run && !S.run.done && budget < 40; k++, budget += 0.02) advance(0.02);
      if (S.run && !S.run.done && budget < 40) { setTimeout(slice, 0); return; }
      for (var j = 0; j < 50; j++) advance(0.02);   /* settle, e.g. the DRAWER seat step */
      S.busy = false; S.active = false;
      renderAll(true);
    }
    setTimeout(slice, 0);
  }

  /* ---- geometry and drawing ----------------------------------------------------------- */

  function world() {
    if (S.name === "door") return { x0: -1.25, x1: 1.25, obj: 1.0 };
    return { x0: -0.72, x1: 0.52, obj: 0.6 };
  }

  function layout() {
    var w = Math.max(280, Math.round(stage.clientWidth || 640));
    var narrow = w < 640;
    var h = narrow ? Math.round(clamp(w * 0.8, 260, 360)) : Math.round(clamp(w * 0.34, 300, 380));
    cv = P.setupCanvas(canvas, w, h);
    var W = world(), pad = 18;
    var X = function (xm) { return pad + (w - 2 * pad) * (xm - W.x0) / (W.x1 - W.x0); };
    var rulerY = h - (narrow ? 40 : 30);
    geo = { w: w, h: h, narrow: narrow, X: X, pxm: (w - 2 * pad) / (W.x1 - W.x0), W: W,
      railY: narrow ? 34 : 104, floorY: rulerY - 16, rulerY: rulerY };
  }

  function line(ctx, x0, y0, x1, y1) { ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke(); }
  function text(ctx, s, x, y, o) {
    o = o || {};
    ctx.font = (o.weight || "400") + " " + (o.size || 11) + "px " + col.sans;
    ctx.fillStyle = o.color || col.muted;
    ctx.textAlign = o.align || "left"; ctx.textBaseline = o.base || "alphabetic";
    ctx.fillText(s, x, y);
  }

  function phaseName() {
    var r = S.run;
    if (!r) return S.h && S.st ? (LEV_NAMES[S.st.lev_state] || "–") : "at rest";
    var s = r.samples, n = s.t.length - 1;
    if (n < 0) return "command sent";
    var mode = s.mode[n], lev = s.lev[n];
    if (lev === 9) return "seating: 0.2 s z-step onto the captive seat";
    if (lev === 2) return "seated";
    if (lev === 4) return "lifting";
    if (mode === 10 || mode === 11) {
      var an = S.an && S.an.th !== null && S.an.th !== undefined ? S.an : analyseMove(r, S.c);
      if (an.th === null || an.th === undefined) return "terminus";
      var tr = s.t[n] - an.th;
      if (tr < an.prof.tB) return "blend: constant jerk out of cruise";
      if (tr < an.prof.tB + an.prof.tCapture) return "capture: d = d_b·e^(−t/τ)";
      return "quintic: the last " + fmt(S.c.d_eps_m * 1e3, 2) + " mm to rest";
    }
    if (mode === 2) {
      var v = Math.abs(s.v[n]), vp = planSpeed(S.name, r.dir, S.c.m_star_kg);
      return v > 0.97 * vp ? "cruise under the governor" : "S-curve: jerk-limited launch";
    }
    if (r.done) return "arrived";
    return MODE_NAMES[mode] || "–";
  }

  function draw() {
    if (!geo) layout();
    var ctx = cv.ctx, g = geo, c = S.c, X = g.X;
    ctx.clearRect(0, 0, g.w, g.h);
    ctx.fillStyle = col.deep; ctx.fillRect(0, 0, g.w, g.h);
    var x = liveX(), dirNow = S.run ? S.run.dir : 1;
    var colour = col[S.name];

    /* floor */
    ctx.strokeStyle = col.axis; ctx.lineWidth = 1.5; line(ctx, 0, g.floorY + 0.5, g.w, g.floorY + 0.5);

    /* end zones and the terminus zone on the ruler */
    var ez = 0.15;
    ctx.save();
    [[X(-ez), X(0)], [X(c.travel_m), X(c.travel_m + ez)]].forEach(function (zz) {
      ctx.fillStyle = P.alpha(col.ref, 0.08); ctx.fillRect(zz[0], g.rulerY - 10, zz[1] - zz[0], 12);
    });
    ctx.restore();
    if (S.an && S.an.th !== null && S.run) {
      var tz0 = S.run.target - S.run.dir * S.an.prof.d0;
      ctx.fillStyle = P.alpha(col.accent, 0.22);
      ctx.fillRect(Math.min(X(tz0), X(S.run.target)), g.rulerY - 10, Math.abs(X(S.run.target) - X(tz0)), 12);
    }
    /* ruler */
    ctx.strokeStyle = col.ink2; ctx.lineWidth = 1; line(ctx, X(0), g.rulerY, X(c.travel_m), g.rulerY);
    var step = S.name === "door" ? 0.1 : 0.05;
    for (var t = 0; t <= c.travel_m + 1e-9; t += step) {
      var major = Math.abs(t / (step * 2) - Math.round(t / (step * 2))) < 1e-6;
      line(ctx, X(t), g.rulerY, X(t), g.rulerY - (major ? 7 : 4));
      if (major || t === 0) text(ctx, fmt(t * 1e3, 4), X(t), g.rulerY + 13, { align: "center", size: 10 });
    }
    text(ctx, "mm", X(c.travel_m) + 18, g.rulerY + 13, { size: 10 });
    if (!g.narrow) {
      text(ctx, "end zone", X(-ez / 2), g.rulerY - 14, { align: "center", size: 9.5 });
      text(ctx, "end zone", X(c.travel_m + ez / 2), g.rulerY - 14, { align: "center", size: 9.5 });
    }
    /* target marker */
    if (S.run) {
      var tx = X(S.run.target);
      ctx.fillStyle = col.good;
      ctx.beginPath(); ctx.moveTo(tx, g.rulerY - 11); ctx.lineTo(tx - 5, g.rulerY - 19); ctx.lineTo(tx + 5, g.rulerY - 19); ctx.closePath(); ctx.fill();
    }
    /* live position marker on the ruler */
    ctx.fillStyle = colour; ctx.beginPath(); ctx.arc(X(clamp(x, -ez, c.travel_m + ez)), g.rulerY, 3.5, 0, 2 * Math.PI); ctx.fill();

    if (S.name === "drawer") drawDrawer(ctx, x, colour); else drawDoor(ctx, x, colour);

    /* phase label and speed arrow */
    var v = S.h && S.st ? S.st.v_mps : (S.run && S.run.samples.v.length ? S.run.samples.v[S.run.samples.v.length - 1] : 0);
    var ph = phaseName();
    if (g.narrow) text(ctx, ph, g.w / 2, g.h - 6, { align: "center", size: 12, color: col.ink, weight: "600" });
    else text(ctx, ph, g.w - 16, g.railY - 46, { align: "right", size: 12.5, color: col.ink, weight: "600" });
    if (Math.abs(v) > 0.002) {
      var ay = g.narrow ? g.railY + 18 : g.railY - 20, ax = X(x), len = clamp(Math.abs(v) * 260, 8, 90) * Math.sign(v);
      ctx.strokeStyle = colour; ctx.fillStyle = colour; ctx.lineWidth = 2;
      line(ctx, ax, ay, ax + len, ay);
      ctx.beginPath(); ctx.moveTo(ax + len, ay); ctx.lineTo(ax + len - 6 * Math.sign(v), ay - 4); ctx.lineTo(ax + len - 6 * Math.sign(v), ay + 4); ctx.closePath(); ctx.fill();
      text(ctx, speedStr(Math.abs(v)), ax + len + 8 * Math.sign(v), ay + 4, { align: v > 0 ? "left" : "right", size: 11, color: col.ink2 });
    }
  }

  function drawCarriage(ctx, xFront, lenM, colour, yRail) {
    var g = geo, X = g.X;
    var cx0 = X(xFront - lenM * 0.92), cx1 = X(xFront - lenM * 0.08);
    /* levitation gap glow */
    ctx.fillStyle = P.alpha(col.flux, 0.25); ctx.fillRect(cx0, yRail + 1, cx1 - cx0, 3);
    ctx.fillStyle = P.alpha(colour, 0.35); ctx.strokeStyle = colour; ctx.lineWidth = 1.2;
    P.roundRect(ctx, cx0, yRail + 4, cx1 - cx0, 10, 3); ctx.fill(); ctx.stroke();
  }

  function drawDrawer(ctx, x, colour) {
    var g = geo, X = g.X;
    var cabTop = g.railY - 8, cabBot = g.floorY;
    var cabX0 = X(-0.66), cabX1 = X(0.0);
    /* cabinet body */
    ctx.fillStyle = P.alpha(col.line, 0.35); ctx.strokeStyle = col.ink2; ctx.lineWidth = 1.5;
    ctx.fillRect(cabX0, cabTop, cabX1 - cabX0, cabBot - cabTop); ctx.strokeRect(cabX0, cabTop, cabX1 - cabX0, cabBot - cabTop);
    ctx.fillStyle = col.ink2; ctx.fillRect(cabX0 - 4, cabTop - 6, cabX1 - cabX0 + 8, 6);
    /* rail under the cabinet top */
    ctx.fillStyle = col.steel; ctx.fillRect(cabX0 + 6, g.railY - 2, cabX1 - cabX0 - 6, 5);
    /* drawer box: front face at x, box behind it */
    var boxTop = g.railY + 18, boxBot = cabTop + (cabBot - cabTop) * 0.62;
    drawCarriage(ctx, x, 0.6, colour, g.railY);
    ctx.fillStyle = P.alpha(colour, 0.18); ctx.strokeStyle = colour; ctx.lineWidth = 1.5;
    ctx.fillRect(X(x - 0.58), boxTop, X(x) - X(x - 0.58), boxBot - boxTop);
    ctx.strokeRect(X(x - 0.58), boxTop, X(x) - X(x - 0.58), boxBot - boxTop);
    ctx.fillStyle = colour; ctx.fillRect(X(x) - 2, boxTop - 6, 7, boxBot - boxTop + 12);
    ctx.fillStyle = col.ink2; ctx.fillRect(X(x) + 5, (boxTop + boxBot) / 2 - 8, 5, 16);
    text(ctx, "cabinet", cabX0 + 8, cabBot - 10, { size: 10.5 });
    if (!geo.narrow) text(ctx, "DRAWER · 26 kg body, rated 48 kg", X(x - 0.56), boxTop + 16, { size: 10.5, color: col.ink2 });
  }

  function drawDoor(ctx, x, colour) {
    var g = geo, X = g.X;
    /* wall with the doorway [−1.0, 0], the leaf parks over [0, 1.0] */
    var top = g.railY - 10, bot = g.floorY;
    ctx.fillStyle = P.alpha(col.line, 0.35);
    ctx.fillRect(X(-1.25), top, X(-1.02) - X(-1.25), bot - top);
    ctx.fillRect(X(0.02), top, X(1.25) - X(0.02), bot - top);
    ctx.fillRect(X(-1.25), top - 14, X(1.25) - X(-1.25), 14);
    ctx.strokeStyle = col.ink2; ctx.lineWidth = 1;
    ctx.strokeRect(X(-1.02), top, X(0.02) - X(-1.02), bot - top);
    text(ctx, "doorway", X(-0.5), bot - 10, { align: "center", size: 10.5 });
    /* rail */
    ctx.fillStyle = col.steel; ctx.fillRect(X(-1.1), g.railY - 2, X(1.1) - X(-1.1), 5);
    drawCarriage(ctx, x, 1.0, colour, g.railY);
    /* leaf: spans [x − 1.0, x] */
    var lt = g.railY + 16;
    ctx.fillStyle = P.alpha(col.flux, 0.1); ctx.strokeStyle = colour; ctx.lineWidth = 1.8;
    ctx.fillRect(X(x - 1.0), lt, X(x) - X(x - 1.0), bot - 6 - lt);
    ctx.strokeRect(X(x - 1.0), lt, X(x) - X(x - 1.0), bot - 6 - lt);
    ctx.fillStyle = col.ink2; ctx.fillRect(X(x - 0.94), (lt + bot) / 2 - 16, 4, 32);
    ctx.strokeStyle = P.alpha(col.flux, 0.35); ctx.lineWidth = 1;
    line(ctx, X(x - 0.7), lt + 20, X(x - 0.5), lt + 70);
    line(ctx, X(x - 0.64), lt + 20, X(x - 0.44), lt + 70);
    if (!geo.narrow) text(ctx, "DOOR · 100 kg glass leaf", X(x - 0.96), lt + 16, { size: 10.5, color: col.ink2 });
  }

  /* ---- readouts ------------------------------------------------------------------------ */

  var lastChip = "";
  function setChip(s) { if (s !== lastChip) { chip.textContent = s; lastChip = s; } }

  function updateReadouts() {
    var c = S.c, x = liveX();
    var v = S.h && S.st ? S.st.v_mps : (S.run && S.run.samples.v.length ? S.run.samples.v[S.run.samples.v.length - 1] : 0);
    var e = S.h && S.st ? S.st.e_kin_gov_j : (S.run && S.run.samples.e.length ? S.run.samples.e[S.run.samples.e.length - 1] : 0);
    var dir = S.run ? S.run.dir : 1;
    setText("gr-x", mmStr(x, 4));
    setText("gr-x-note", S.run ? "target " + mmStr(S.run.target, 4) + " · to go " + mmStr(Math.abs(S.run.target - x), 3) : "travel 0 to " + mmStr(c.travel_m, 4));
    setText("gr-v", speedStr(Math.abs(v)));
    var g = govCaps(S.name, S.h && S.st ? S.st.m_hat_kg : c.m_star_kg);
    setText("gr-v-note", "cap " + fmt(Math.min(capFor(c, dir), g.vCap), 4) + " m/s · plan " + fmt(planSpeed(S.name, dir, S.h && S.st ? S.st.m_hat_kg : c.m_star_kg), 4) + " m/s");
    setText("gr-e", e < 1e-4 ? "≈ 0 J" : fmt(e, 3) + " J");
    var bar = $("gr-e-bar"), capMark = $("gr-e-cap");
    if (bar) bar.style.width = clamp((e / E_MAX_J) * 100, 0, 100) + "%";
    if (capMark) capMark.style.left = KE_PLAN * 100 + "%";
    setText("gr-phase", phaseName());
    var modeTxt = S.h && S.st ? (MODE_NAMES[S.st.motion_mode] || "–") + " · " + (LEV_NAMES[S.st.lev_state] || "–") : S.run ? "closed-form reference" : "–";
    setText("gr-mode", modeTxt);
    var an = S.an;
    if (an && an.fit) {
      setText("gr-tau", fmt(an.fit.tau, 4) + " s");
      setText("gr-r2", "R² " + fmt(an.fit.r2, 6) + " · preset τ " + fmt(c.tau_s, 3) + " s · " + an.fit.n + " samples");
    } else { setText("gr-tau", "–"); setText("gr-r2", "R² – (fills during the capture)"); }
    if (an && an.th !== null && an.th !== undefined) {
      setText("gr-arr", an.tArrive !== null ? fmt(an.tArrive, 4) + " s" : "…");
      setText("gr-arr-note", "20 µm crossing after handover · closed form t_arrive " + fmt(an.tArriveCf, 4) + " s");
    } else { setText("gr-arr", "–"); setText("gr-arr-note", "closed form –"); }
    setText("gl-engine", "engine: " + (S.h ? "live: Rust controller + Carriage6 plant in WebAssembly" : S.engineMode === "loading" ? "loading" : "closed-form fallback" + (S.engineReason ? " (" + S.engineReason + ")" : "")));
    openBtn.disabled = S.busy; closeBtn.disabled = S.busy;
  }

  function updateChip() {
    if (S.busy) return;
    if (S.stopped) { setChip("The engine stopped (" + S.stopped + "). Switch the class to restart it."); return; }
    if (S.engineMode === "loading") { setChip("Loading the browser engine…"); return; }
    var who = S.name === "drawer" ? "DRAWER" : "DOOR";
    if (!S.h) {
      setChip("Closed-form fallback (" + (S.engineReason || "engine unavailable") + "): the stage shows the reference the controller follows, not the simulated carriage.");
      return;
    }
    if (!S.run) {
      setChip("Live " + who + " on Carriage6 in WebAssembly. Press " + (S.name === "drawer" ? "Open: it unseats, leaves the slot and glides out." : "Open or Close."));
      return;
    }
    if (S.run.done && S.an && S.an.fit) {
      setChip("Arrived. Capture fit: τ = " + fmt(S.an.fit.tau, 4) + " s against the preset " + fmt(S.c.tau_s, 3) + " s, R² = " + fmt(S.an.fit.r2, 6) + ".");
      return;
    }
    setChip("Live " + who + ": " + phaseName() + ".");
  }

  /* ---- charts ------------------------------------------------------------------------ */

  var semiChart = null, speedChart = null;

  function refreshCharts(force) {
    var now = Date.now();
    if (!force && now - S.lastChart < 150) return;
    S.lastChart = now;
    var r = S.run, c = S.c;
    if (r && r.samples.t.length) S.an = analyseMove(r, c);
    if (!semiChart || !speedChart) return;
    if (!r) {
      semiChart.update({ series: [], points: [], refs: [], desc: "Empty until a move runs." });
      speedChart.update({ series: [], points: [], refs: [], desc: "Empty until a move runs." });
      return;
    }
    var s = r.samples, dd = [], vv = [], stride = Math.max(1, Math.floor(s.t.length / 1500));
    for (var i = 0; i < s.t.length; i += stride) {
      var d = Math.abs(r.target - s.x[i]);
      if (d > 0) dd.push([s.t[i], d]);
      vv.push([s.t[i], Math.abs(s.v[i])]);
    }
    var an = S.an, cf = [], refs = [], tEnd = s.t.length ? s.t[s.t.length - 1] : 1;
    if (an && an.th !== null && an.th !== undefined) {
      for (var k = 0; k <= 240; k++) {
        var tt = (an.prof.T * 1.02 * k) / 240, q = an.prof.sample(tt);
        if (q.d > 0) cf.push([an.th + tt, q.d]);
      }
      refs.push({ axis: "x", value: an.th, label: "handover", color: "muted", dash: [2, 4] });
      refs.push({ axis: "x", value: an.captureFrom, label: "capture", color: "muted", dash: [2, 4], row: 1 });
    }
    refs.push({ axis: "y", value: c.d_eps_m, label: "d_ε " + fmt(c.d_eps_m * 1e3, 2) + " mm", color: "muted", dash: [2, 4] });
    refs.push({ axis: "y", value: ARRIVE_D_M, label: "20 µm arrival", color: "muted", dash: [1, 3] });
    var tMax = Math.max(1, an && an.th !== null && an.th !== undefined ? Math.max(tEnd, an.th + an.prof.T + 0.3) : tEnd + 0.5);
    semiChart.update({
      desc: "Semilog chart of the distance to the target against time since the command, with the closed-form terminus from the measured handover speed.",
      x: { label: "time since the command", unit: "s", min: 0, max: tMax },
      y: { label: "distance to go", unit: "m", si: true, min: 1e-5, max: Math.max(0.1, c.travel_m * 1.2) },
      series: [
        { key: "d", label: S.h ? "plant truth (WebAssembly)" : "closed-form reference", color: S.name, data: dd },
        { key: "cf", label: "closed-form terminus at the handover speed", color: "muted", dash: [5, 4], width: 1.5, data: cf }
      ],
      refs: refs, table: !!r.done, tableRows: 30
    });
    var g = govCaps(S.name, S.h && S.st ? S.st.m_hat_kg : c.m_star_kg);
    var cap = Math.min(capFor(c, r.dir), g.vCap), plan = Math.min(capFor(c, r.dir), g.vGov);
    speedChart.update({
      desc: "Speed against time for the move, with the class cap and the governor's planning cap.",
      x: { label: "time since the command", unit: "s", min: 0, max: tMax },
      y: { label: "speed", unit: "m/s", min: 0, max: cap * 1.15 },
      series: [{ key: "v", label: S.h ? "plant truth (WebAssembly)" : "closed-form reference", color: S.name, data: vv }],
      refs: [
        { axis: "y", value: cap, label: "cap " + fmt(cap, 4) + " m/s", color: "ref" },
        { axis: "y", value: plan, label: "governor plan cap " + fmt(plan, 4), color: "muted", dash: [5, 4], align: "left" }
      ],
      table: !!r.done, tableRows: 30
    });
  }

  function initCharts() {
    var se = $("semi-chart"), sp = $("speed-chart");
    if (se) semiChart = P.chart(se, { kind: "semilog", title: "Distance to go", desc: "Empty until a move runs.", x: { label: "time since the command", unit: "s", min: 0, max: 3 }, y: { label: "distance to go", unit: "m", si: true, min: 1e-5, max: 0.5 }, series: [], height: 270 });
    if (sp) speedChart = P.chart(sp, { kind: "line", title: "Speed", desc: "Empty until a move runs.", x: { label: "time since the command", unit: "s", min: 0, max: 3 }, y: { label: "speed", unit: "m/s", min: 0 }, series: [], height: 270 });
  }

  function renderAll(forceCharts) {
    draw(); updateReadouts(); updateChip(); refreshCharts(forceCharts);
  }

  /* ---- animation loop ---------------------------------------------------------------- */

  var loop = P.loop(function (dt) {
    advance(dt * S.speed);
    if (S.run && S.run.done) {
      S.tail += dt;
      if (S.tail > 2.5) { S.active = false; loop.stop(); renderAll(true); return; }
    }
    renderAll(false);
  }, { el: stage, still: function () { renderAll(true); }, maxDt: 0.05 });

  /* ---- S-curve explorer ------------------------------------------------------------- */

  function scurveSection() {
    var group = $("sc-class"), input = $("sc-L"), vEl = $("sc-v-chart"), aEl = $("sc-a-chart");
    var pick = "drawer", vChart = null, aChart = null;
    var ANCHOR = { drawer: 33.43e-3, door: 36.31e-3, wall: 8.86e-3, heavy: 4.91e-3 };
    function lengthOf(c) {
      var f = input ? parseFloat(input.value) / 1000 : 1;
      return 1e-3 * Math.pow(c.travel_m / 1e-3, f);
    }
    function render() {
      var c = F.CLASSES[pick], L = lengthOf(c);
      var vMax = c.v_open_max_mps, a = c.a_max_mps2, j = c.j_max_mps3;
      var sc = F.scurve(L, vMax, a, j), n = 300, vd = [], ad = [];
      for (var k = 0; k <= n; k++) {
        var t = (sc.T * k) / n, q = sc.sample(t);
        vd.push([t, q.v]); ad.push([t, q.a]);
      }
      /* the seven jerk segments, from the peak speed (PHYSICS §16) */
      var vp = sc.vPeak, tj, tc;
      if (vp >= (a * a) / j) { tj = a / j; tc = vp / a - tj; } else { tj = Math.sqrt(vp / j); tc = 0; }
      var tA = 2 * tj + tc, tCr = Math.max(0, sc.T - 2 * tA);
      var edges = [0, tj, tj + tc, tA, tA + tCr, tA + tCr + tj, tA + tCr + tj + tc, sc.T];
      var labels = ["+j", "0", "−j", "cruise", "−j", "0", "+j"], bands = [];
      for (var b = 0; b < 7; b++) {
        if (edges[b + 1] - edges[b] < 1e-9) continue;
        var wide = edges[b + 1] - edges[b] >= 0.12 * sc.T;
        bands.push({ axis: "x", from: edges[b], to: edges[b + 1], label: wide ? labels[b] : undefined, color: b % 2 ? "muted" : "accent", alpha: b % 2 ? 0.06 : 0.1 });
      }
      var vSpec = {
        kind: "line", title: "S-curve speed, " + c.label + ", " + fmt(L * 1e3, 4) + " mm",
        desc: "Speed of the seven-segment S-curve against time.",
        x: { label: "time", unit: "s", min: 0, max: sc.T }, y: { label: "speed", unit: "m/s", min: 0, max: vMax * 1.12 },
        series: [{ key: "v", label: "speed", color: pick, data: vd }],
        refs: [{ axis: "y", value: vMax, label: "cap " + fmt(vMax, 4) + " m/s", color: "ref" }],
        height: 230, table: true, tableRows: 25
      };
      var aSpec = {
        kind: "line", title: "S-curve acceleration, " + c.label, desc: "Acceleration of the S-curve against time, with the seven constant-jerk segments shaded.",
        x: { label: "time", unit: "s", min: 0, max: sc.T }, y: { label: "acceleration", unit: "m/s²", min: -a * 1.2, max: a * 1.2 },
        series: [{ key: "a", label: "acceleration", color: pick, data: ad }], bands: bands,
        refs: [{ axis: "y", value: a, label: "+a_max", color: "muted", dash: [3, 4] }, { axis: "y", value: -a, label: "−a_max", color: "muted", dash: [3, 4] }],
        height: 230, table: true, tableRows: 25
      };
      if (!vChart && vEl) vChart = P.chart(vEl, vSpec); else if (vChart) vChart.update(vSpec);
      if (!aChart && aEl) aChart = P.chart(aEl, aSpec); else if (aChart) aChart.update(aSpec);
      var ds = F.stopDistance(vMax, a, j);
      setText("sc-L-out", fmt(L * 1e3, 4) + " mm");
      setText("sc-T", fmt(sc.T, 4) + " s");
      setText("sc-vp", fmt(vp, 4) + " m/s");
      setText("sc-vp-note", vp < vMax * 0.9999 ? "too short to reach the cap" : "reaches the cap");
      setText("sc-lim", fmt(vMax, 4) + " m/s · " + fmt(a, 3) + " m/s² · " + fmt(j, 3) + " m/s³");
      setText("sc-dstop", fmt(ds * 1e3, 4) + " mm");
      setText("sc-dstop-note", "anchor " + fmt(ANCHOR[pick] * 1e3, 4) + " mm (FAB-PRP-005)");
      if (input) input.setAttribute("aria-valuetext", fmt(L * 1e3, 4) + " millimetres");
    }
    if (group) UI.segmented(group, function (v) { pick = v; render(); });
    if (input) input.addEventListener("input", render);
    render();
  }

  /* ---- governor ---------------------------------------------------------------------- */

  function governorSection() {
    var group = $("gv-class"), input = $("gv-m"), el = $("gv-chart");
    var pick = "door", chart = null;
    function mOf(c) { return c.mass_rated_kg * Math.pow(10, -0.6 + (1.0 * parseFloat(input.value)) / 1000); }
    function setM(c, m) { input.value = String(Math.round(((Math.log(m / c.mass_rated_kg) / Math.LN10 + 0.6) / 1.0) * 1000)); }
    function render() {
      var c = F.CLASSES[pick], mh = mOf(c), g = govCaps(pick, mh), p = pMaxFor(pick);
      var vPlan = planSpeed(pick, 1, mh);
      setText("gv-m-out", fmt(mh, 4) + " kg");
      input.setAttribute("aria-valuetext", fmt(mh, 4) + " kilograms");
      setText("gv-mgov", fmt(g.mGov, 4) + " kg");
      setText("gv-mgov-note", g.mGov > c.mass_rated_kg + 1e-9 ? "1.05·m̂ exceeds m_rated: slower" : "= m_rated (" + fmt(c.mass_rated_kg, 4) + " kg): m̂ cannot speed it up");
      setText("gv-ve", fmt(g.vE, 4) + " m/s");
      setText("gv-vp", p > 0 ? fmt(g.vP, 4) + " m/s" : "not applicable");
      setText("gv-plan", fmt(vPlan, 4) + " m/s");
      setText("gv-plan-note", "open; class cap " + fmt(c.v_open_max_mps, 4) + " m/s; ½·m_gov·v² = " + fmt(0.5 * g.mGov * vPlan * vPlan, 3) + " J");
      var mm = [1, 30000], lines = [
        { key: "e", label: "energy cap √(2E/m)", color: "ink2", fn: function (m) { return Math.sqrt(2 * E_MAX_J / m); }, samples: 120 },
        { key: "p", label: "momentum cap p/m", color: "muted", dash: [6, 4], fn: function (m) { return P_MAX / m; }, samples: 120 },
        { key: "ep", label: "planning: 0.90 of the energy", color: pick, dash: [3, 3], width: 1.5, fn: function (m) { return Math.sqrt(2 * KE_PLAN * E_MAX_J / m); }, samples: 120 }
      ];
      var spec = {
        kind: "loglog", title: "Speed caps against the governor mass",
        desc: "Log-log chart of the energy and momentum speed caps against mass, the governor's planning cap and the planned cruise at the governor mass.",
        x: { label: "governor mass m_gov", unit: "kg", min: mm[0], max: mm[1] },
        y: { label: "speed", unit: "m/s", min: 0.005, max: 2 },
        series: lines,
        refs: [
          { axis: "x", value: c.mass_rated_kg, label: "m_rated", color: "muted", dash: [2, 4] },
          { axis: "x", value: 1.05 * mh, label: "1.05·m̂", color: pick, dash: [2, 3], row: 1 }
        ],
        points: [{ x: g.mGov, y: vPlan, label: c.label + " plan " + fmt(vPlan, 3) + " m/s", color: pick }],
        height: 300, table: true, tableRows: 20
      };
      if (!chart && el) chart = P.chart(el, spec); else if (chart) chart.update(spec);
    }
    if (group) UI.segmented(group, function (v) { pick = v; setM(F.CLASSES[pick], F.CLASSES[pick].mass_rated_kg); render(); });
    if (input) input.addEventListener("input", render);
    var b1 = $("gv-sabotage"), b2 = $("gv-true"), b3 = $("gv-heavy");
    if (b1) b1.addEventListener("click", function () { var c = F.CLASSES[pick]; setM(c, 0.5 * c.mass_rated_kg); render(); });
    if (b2) b2.addEventListener("click", function () { var c = F.CLASSES[pick]; setM(c, c.mass_rated_kg); render(); });
    if (b3) b3.addEventListener("click", function () { var c = F.CLASSES[pick]; setM(c, 1.3 * c.mass_rated_kg); render(); });
    setM(F.CLASSES[pick], F.CLASSES[pick].mass_rated_kg);
    render();
  }

  /* ---- pinch dial ------------------------------------------------------------------- */

  function pinchSection() {
    var dc = $("dial-canvas"), vIn = $("pd-v"), mIn = $("pd-m"), kSel = $("pd-k");
    if (!dc || !vIn || !mIn || !kSel) return;
    var FMAX = 600, shown = 0, target = 0, anim = null;
    var LIMITS = [
      { f: 67, label: "67 N" }, { f: 80, label: "80 N" }, { f: 150, label: "150 N" }, { f: 400, label: "400 N" }
    ];
    function mOf() { return 5 * Math.pow(10, (parseFloat(mIn.value) / 1000) * Math.log(1000 / 5) / Math.LN10); }
    function setM(m) { mIn.value = String(Math.round((Math.log(m / 5) / Math.log(1000 / 5)) * 1000)); }
    function drawDial() {
      var w = Math.max(240, Math.round(dc.parentNode.clientWidth - 2 || 360)), h = Math.round(w * 0.66);
      var d = P.setupCanvas(dc, w, h), ctx = d.ctx;
      var cx = w / 2, cy = h * 0.74, R = Math.min(w * 0.42, h * 0.66);
      function ang(f) { return Math.PI + Math.PI * clamp(f / FMAX, 0, 1); }
      var zones = [[0, 67, col.good, 0.2], [67, 150, col.s4, 0.18], [150, 400, col.s4, 0.3], [400, FMAX, col.ref, 0.3]];
      for (var z = 0; z < zones.length; z++) {
        ctx.beginPath(); ctx.arc(cx, cy, R, ang(zones[z][0]), ang(zones[z][1])); ctx.arc(cx, cy, R * 0.8, ang(zones[z][1]), ang(zones[z][0]), true);
        ctx.closePath(); ctx.fillStyle = P.alpha(zones[z][2], zones[z][3]); ctx.fill();
      }
      ctx.strokeStyle = col.axis; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(cx, cy, R, Math.PI, 2 * Math.PI); ctx.stroke();
      for (var f = 0; f <= FMAX; f += 50) {
        var a = ang(f), maj = f % 100 === 0;
        ctx.beginPath(); ctx.moveTo(cx + Math.cos(a) * R, cy + Math.sin(a) * R); ctx.lineTo(cx + Math.cos(a) * (R - (maj ? 10 : 5)), cy + Math.sin(a) * (R - (maj ? 10 : 5))); ctx.stroke();
        if (maj) text(ctx, String(f), cx + Math.cos(a) * (R + 12), cy + Math.sin(a) * (R + 12) + 4, { align: "center", size: 10 });
      }
      for (var l = 0; l < LIMITS.length; l++) {
        var al = ang(LIMITS[l].f), lr = l === 1 ? 0.5 : 0.66;
        ctx.strokeStyle = col.ref; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(cx + Math.cos(al) * R * 0.78, cy + Math.sin(al) * R * 0.78); ctx.lineTo(cx + Math.cos(al) * R * 1.02, cy + Math.sin(al) * R * 1.02); ctx.stroke();
        text(ctx, LIMITS[l].label, cx + Math.cos(al) * R * lr, cy + Math.sin(al) * R * lr + 4, { align: "center", size: 10, color: col.ink2, weight: "600" });
      }
      var an = ang(Math.min(shown, FMAX));
      ctx.strokeStyle = col.ink; ctx.lineWidth = 3; ctx.lineCap = "round";
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + Math.cos(an) * R * 0.92, cy + Math.sin(an) * R * 0.92); ctx.stroke();
      ctx.fillStyle = col.ink; ctx.beginPath(); ctx.arc(cx, cy, 6, 0, 2 * Math.PI); ctx.fill();
      text(ctx, fmt(target, 4) + " N" + (target > FMAX ? " (off scale)" : ""), cx, cy + 34, { align: "center", size: 22, color: col.ink, weight: "700" });
      text(ctx, "peak quasi-static force", cx, cy + 52, { align: "center", size: 10.5 });
    }
    function update() {
      var v = parseFloat(vIn.value), m = mOf(), k = K_BODY[kSel.value] || K_BODY["hand-edge"];
      target = pinchForce(v, m, k);
      setText("pd-v-out", fmt(v, 3) + " m/s");
      setText("pd-m-out", fmt(m, 3) + " kg");
      vIn.setAttribute("aria-valuetext", fmt(v, 3) + " metres per second");
      mIn.setAttribute("aria-valuetext", fmt(m, 3) + " kilograms");
      var e = 0.5 * m * v * v, v150 = 150 / Math.sqrt(m * k), v67 = 67 / Math.sqrt(m * k);
      setText("dial-caption", "F = v·√(m·k) = " + fmt(target, 4) + " N with E = ½mv² = " + fmt(e, 3) + " J and k = " + fmt(k / 1e3, 3) +
        " N/mm. The same catch stays at 150 N up to " + fmt(v150, 3) + " m/s and at 67 N up to " + fmt(v67, 3) + " m/s. Closed form, PHYSICS §18.");
      dc.setAttribute("aria-label", "Dial gauge: peak clamp force " + fmt(target, 4) + " newtons, with marks at 67, 80, 150 and 400 newtons.");
      if (UI.reducedMotion()) { shown = target; drawDial(); return; }
      if (!anim) anim = P.loop(function (dt) {
        shown += (target - shown) * Math.min(1, dt * 9);
        drawDial();
        if (Math.abs(target - shown) < 0.05) { shown = target; drawDial(); anim.stop(); }
      }, { el: dc, still: function () { shown = target; drawDial(); } });
      anim.start();
    }
    var PRESET = {
      drawer: { v: F.CLASSES.drawer.v_open_max_mps, m: 48, k: "hand-edge" },
      door: { v: F.CLASSES.door.v_open_max_mps, m: 100, k: "hand-edge" },
      doorclose: { v: F.CLASSES.door.v_close_max_mps, m: 100, k: "hand-edge" },
      wall: { v: F.CLASSES.wall.v_open_max_mps, m: 500, k: "chest" },
      unattended: { v: F.unattendedCloseSpeed(F.CLASSES.drawer.v_close_max_mps, 150, 48, K_BODY["hand-edge"]), m: 48, k: "hand-edge" }
    };
    var btns = doc.querySelectorAll("[data-pinch]");
    for (var i = 0; i < btns.length; i++) {
      btns[i].addEventListener("click", function () {
        var p = PRESET[this.getAttribute("data-pinch")];
        if (!p) return;
        vIn.value = String(Math.round(p.v * 1000) / 1000); setM(p.m); kSel.value = p.k; update();
      });
    }
    vIn.addEventListener("input", update); mIn.addEventListener("input", update); kSel.addEventListener("change", update);
    setM(48);
    shown = 0;
    update();
    P.onTheme(function () { drawDial(); });
    if (root.ResizeObserver) new root.ResizeObserver(function () { drawDial(); }).observe(dc.parentNode);
  }

  /* ---- wiring ------------------------------------------------------------------------- */

  function setClass(name) {
    S.name = name; S.c = F.CLASSES[name];
    loop.stop(); S.active = false; S.busy = false;
    layout();
    createEngine();
    refreshCharts(true);
    renderAll(true);
  }

  openBtn.addEventListener("click", function () { startMove("open"); });
  closeBtn.addEventListener("click", function () { startMove("close"); });
  var g1 = $("gl-class"); if (g1) UI.segmented(g1, function (v) { setClass(v); });
  var g2 = $("gl-speed"); if (g2) UI.segmented(g2, function (v) { S.speed = parseFloat(v) || 1; });

  P.onTheme(function () { colours(); renderAll(true); });
  if (root.ResizeObserver) {
    var lastW = 0;
    new root.ResizeObserver(function () {
      var w = stage.clientWidth;
      if (w && Math.abs(w - lastW) >= 1) { lastW = w; layout(); draw(); }
    }).observe(stage);
  } else root.addEventListener("resize", function () { layout(); draw(); });

  colours();
  layout();
  initCharts();
  scurveSection();
  governorSection();
  pinchSection();

  function labelSims() {
    ["gl-sim", "semi-sim", "speed-sim"].forEach(function (id) {
      var el = $(id);
      if (!el) return;
      el.textContent = ""; el.classList.remove("sim-label"); el.removeAttribute("title");
      el.setAttribute("data-simulated", S.h ? "" : "closed");
    });
    UI.renderSimLabels(doc);
  }

  function onEngine(r) {
    S.engineMode = r && r.mode === "wasm" ? "wasm" : "formulas";
    S.engineReason = r && r.reason ? r.reason : "";
    createEngine();
    labelSims();
    renderAll(true);
  }

  if (E && typeof E.load === "function") {
    var force = wantForce();
    E.load(force ? { force: true } : undefined).then(function (r) {
      if (force) E.renderSelftestBadges();
      onEngine(r);
    }, function () { onEngine({ mode: "formulas", reason: "loader error" }); });
  } else onEngine({ mode: "formulas", reason: "engine.js missing" });
  renderAll(true);
})(typeof window !== "undefined" ? window : globalThis);
