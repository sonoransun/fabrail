/* fabrail levitate scene (plan §3b). Owner: W2.
 *
 * What runs where:
 *   - The float: the real Rust levitation controller (fabrail-core through fabrail-rt, the
 *     firmware's group code) on the Heave1 plant tier, compiled to WebAssembly
 *     (crates/fabrail-web) and stepped in real time through assets/js/engine.js. The
 *     browser ABI has no payload input, so a payload Δm is applied as the equivalent
 *     steady vertical floor acceleration a_z = g₀·Δm/m* (same downward force on the body,
 *     same zero-power equilibrium; the inertia stays m*), ramped at RAMP_FRAC_PER_S.
 *   - Control off: the closed-form open-loop magnet force of PHYSICS §3 (coil current 0,
 *     no eddy lag, no dissipation, rigid stops), integrated here by RK4. The engine is
 *     never switched off; its levitation safe state is SS1 Safe Landing.
 *   - Stop ratios, thermal shift, impact speeds: closed forms (PHYSICS §3, §12, §15),
 *     with the WebAssembly engine's own Params.derived() table when it is loaded.
 *   - Without the WebAssembly engine every live element falls back to the closed forms
 *     and says so.
 *
 * Under node, require("docs/assets/js/scene-levitate.js") returns the pure core (no DOM):
 * { EXTRA, forceRatio, zeroPowerGap, bandGap, biasCurrent, poleAt, impactClosedForm,
 *   openLoop, thermalShift, floorAzFor, runFloat } so the physics and the wasm loop can
 *   be checked headless.
 */
(function (root) {
  "use strict";

  var G0 = 9.80665;            /* standard gravity, m/s² (derive::G0_MPS2; PHYSICS §3) */
  var TICK_HZ = 20000;         /* base tick (fabrail_types::BASE_TICK_HZ; PHYSICS §24) */
  var RAMP_FRAC_PER_S = 0.2;   /* payload ramp, fraction of m* per second (page choice, keeps the
                                  emulated floor acceleration quasi-static for the detector) */
  var CHUNK_TICKS = 200;       /* 10 ms per engine call */
  var NUDGE_M = 5e-6;          /* control-off nudge, m */
  var SLOWMO = 0.1;            /* control-off playback speed */

  /* Preset data the FORMULAS block does not carry (fabrail_params::presets, read from
     Params.preset(c).to_json(): /design/suspension/{g_up_m, turns, i_cont_a, i_pulse_a,
     tempco_br_per_k} and Params.derived() m_pm_at). impact_* are the published
     derive::impact_speed anchors (PHYSICS §3 table; FAB-LEV-019). */
  var EXTRA = {
    drawer: { g_up_m: 0.0012, turns: 200, m_pm_a: 3231.603223687659, i_cont_a: 4, i_pulse_a: 12,
      tempco: -0.0012, impact_up_mps: 0.1096, impact_down_mps: 0.1973, blocks: [0.1, 0.2, 0.2],
      leaf: "drawer box" },
    door: { g_up_m: 0.004, turns: 300, m_pm_a: 7586.385616583833, i_cont_a: 6.5, i_pulse_a: 10,
      tempco: -0.0012, impact_up_mps: 0.05058, impact_down_mps: 0.05559, blocks: [0.05, 0.1, 0.1],
      leaf: "glass door leaf" },
    wall: { g_up_m: 0.008, turns: 300, m_pm_a: 17734.4079348713, i_cont_a: 15, i_pulse_a: 20,
      tempco: -0.0012, impact_up_mps: 0.06881, impact_down_mps: 0.07627, blocks: [0.05, 0.1, 0.1],
      leaf: "partition panel" },
    heavy: { g_up_m: 0.008, turns: 200, m_pm_a: 17734.4079348713, i_cont_a: 22, i_pulse_a: 40,
      tempco: -0.0012, impact_up_mps: 0.06881, impact_down_mps: 0.07627, blocks: [0.05, 0.1, 0.1],
      leaf: "archive shelving" }
  };

  function formulas() {
    if (root.FabrailFormulas) return root.FabrailFormulas;
    if (typeof require === "function") {
      try { return require("./formulas.js"); } catch (e) { return null; }
    }
    return null;
  }
  var F = formulas();

  /* ---- pure physics (PHYSICS §3, §12, §15) ------------------------------------------ */

  function cls(name) { return F.CLASSES[name]; }
  function hPrime(c) { return F.hPrime(c.h_m_m, c.mu_rec); }

  /* n·F_pm(g, i = 0)/(m·g₀) = m*·((2g* + h′)/(2g + h′))²/m (derive::stop_ratio). */
  function forceRatio(c, gapM, mKg) {
    return F.zeroPowerMass(gapM, c.g_star_m, hPrime(c), c.m_star_kg) / mKg;
  }

  function zeroPowerGap(c, mKg) {
    return F.zeroPowerGap(mKg, c.g_star_m, hPrime(c), c.m_star_kg);
  }

  /* The modified zero-power band clamp holds the band edge outside [lo, hi] (FAB-LEV-018). */
  function bandGap(c, mKg) {
    return Math.min(Math.max(zeroPowerGap(c, mKg), c.zp_band_lo_m), c.zp_band_hi_m);
  }

  /* Bias current holding m at gap g with all lift elements equal:
     i = M_pm·((2g + h′)/(2g* + h′)·√(m/m*) − 1)/N (derive::bias_current rewritten with the
     preset balance n·μ₀·A_p·M_pm² = m*·g₀·(2g* + h′)²; PHYSICS §3). */
  function biasCurrent(c, x, gapM, mKg) {
    var hp = hPrime(c);
    return x.m_pm_a * (((2 * gapM + hp) / (2 * c.g_star_m + hp)) * Math.sqrt(mKg / c.m_star_kg) - 1) / x.turns;
  }

  /* Open-loop pole at the zero-power float of mass m: p = √(4g₀/(2g(m) + h′)). */
  function poleAt(c, mKg) {
    return Math.sqrt((4 * G0) / (2 * zeroPowerGap(c, mKg) + hPrime(c)));
  }

  /* PM work ∫_{ga}^{gb} n·F dg = ½·m*·g₀·(2g* + h′)²·(1/(2ga + h′) − 1/(2gb + h′)), J. */
  function pmWork(c, ga, gb) {
    var hp = hPrime(c), ds = 2 * c.g_star_m + hp;
    return 0.5 * c.m_star_kg * G0 * ds * ds * (1 / (2 * ga + hp) - 1 / (2 * gb + hp));
  }

  /* Impact speed from rest at g0 (energy closed form, PHYSICS §3/§15). dir "up" | "down". */
  function impactClosedForm(c, x, mKg, g0, dir) {
    var e;
    if (dir === "up") e = pmWork(c, x.g_up_m, g0) - mKg * G0 * (g0 - x.g_up_m);
    else e = mKg * G0 * (c.g_land_m - g0) - pmWork(c, g0, c.g_land_m);
    return e > 0 ? Math.sqrt((2 * e) / mKg) : 0;
  }

  /* Open-loop gap dynamics with the coil current at zero: g̈ = g₀ − n·F_pm(g)/m (the gap
     g is measured down from the rail, so pull reduces it). RK4 at dt until a stop is hit
     (upper stop g_up, landing rollers g_land) or tMax. Returns samples every `every`. */
  function openLoop(c, x, mKg, gStart, vStart, opts) {
    opts = opts || {};
    var dt = opts.dt || 2e-5, tMax = opts.tMax || 1.5, every = opts.every || 5e-4;
    var hp = hPrime(c), ds = 2 * c.g_star_m + hp, k = c.m_star_kg * G0 * ds * ds / mKg;
    function acc(g) { var d = 2 * g + hp; return G0 - k / (d * d); }
    var g = gStart, v = vStart || 0, t = 0, next = 0;
    var ts = [], gs = [], hit = null;
    while (t < tMax) {
      if (t >= next - 1e-12) { ts.push(t); gs.push(g); next += every; }
      var k1v = acc(g), k1g = v;
      var k2v = acc(g + 0.5 * dt * k1g), k2g = v + 0.5 * dt * k1v;
      var k3v = acc(g + 0.5 * dt * k2g), k3g = v + 0.5 * dt * k2v;
      var k4v = acc(g + dt * k3g), k4g = v + dt * k3v;
      var gN = g + (dt / 6) * (k1g + 2 * k2g + 2 * k3g + k4g);
      var vN = v + (dt / 6) * (k1v + 2 * k2v + 2 * k3v + k4v);
      if (gN <= x.g_up_m || gN >= c.g_land_m) {
        var stop = gN <= x.g_up_m ? x.g_up_m : c.g_land_m;
        var f = (stop - g) / (gN - g);
        t += f * dt;
        v = v + f * (vN - v);
        g = stop;
        hit = gN <= x.g_up_m ? "up" : "down";
        ts.push(t); gs.push(g);
        break;
      }
      g = gN; v = vN; t += dt;
    }
    /* local divergence rate at the start, √(4k/(2g + h′)³) (= p at a zero-power float) */
    return { t: ts, g: gs, hit: hit, tHit: hit ? t : null, vHit: hit ? Math.abs(v) : null,
      p: Math.sqrt(Math.abs((4 * k) / Math.pow(2 * gStart + hp, 3))) };
  }

  /* Static zero-power gap shift for ΔT: Δg = (2g* + h′)·α_Br·ΔT/2 (derive::thermal_gap_shift). */
  function thermalShift(c, x, dT) {
    return ((2 * c.g_star_m + hPrime(c)) * x.tempco * dT) / 2;
  }

  /* Emulated payload: the steady vertical floor acceleration with the same force. */
  function floorAzFor(c, payloadKg) {
    return (G0 * payloadKg) / c.m_star_kg;
  }

  /* Headless helper (node checks): a Heave1 engine handle `h` (engine.js open()), one
     class, a payload ramp to `payloadKg`; returns the final state after `seconds`. */
  function runFloat(h, name, payloadKg, seconds) {
    var c = cls(name), applied = 0, rate = RAMP_FRAC_PER_S * c.m_star_kg, last = null;
    if (name === "wall" || name === "heavy") h.setInputs({ command: "lift" });
    var n = Math.round(seconds * TICK_HZ / CHUNK_TICKS);
    for (var k = 0; k < n; k++) {
      var dt = CHUNK_TICKS / TICK_HZ;
      applied += Math.max(-rate * dt, Math.min(rate * dt, payloadKg - applied));
      h.setInputs({ floorAz: floorAzFor(c, applied) });
      h.step(CHUNK_TICKS);
      last = h.state();
    }
    return last;
  }

  var CORE = {
    G0: G0, EXTRA: EXTRA, forceRatio: forceRatio, zeroPowerGap: zeroPowerGap, bandGap: bandGap,
    biasCurrent: biasCurrent, poleAt: poleAt, pmWork: pmWork, impactClosedForm: impactClosedForm,
    openLoop: openLoop, thermalShift: thermalShift, floorAzFor: floorAzFor, runFloat: runFloat,
    RAMP_FRAC_PER_S: RAMP_FRAC_PER_S
  };
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
  function mm(v, d) { return fmt(v * 1e3, d === undefined ? 4 : d) + " mm"; }
  function amps(v) {
    var a = Math.abs(v);
    if (a < 0.01) return fmt(v * 1e3, 2) + " mA";
    return fmt(v, 3) + " A";
  }

  var LEV_NAMES = ["boot", "self-test", "seated", "landed", "lifting", "levitated, idle", "moving",
    "stopping", "landing", "seating", "fault reaction (SS1)", "fault (latched)", "seismic action",
    "escape", "manual"];
  var MODE_NAMES = ["propulsion off", "hold", "trajectory", "admittance", "soft-close", "reverse",
    "freeze", "centre", "isolate", "damp", "terminus", "soft-open", "inertial smoothing"];

  function wantForce() {
    try { return /(?:^|[?&])engine=wasm(?:&|$)/.test(root.location.search || ""); } catch (e) { return false; }
  }

  /* ---- state ------------------------------------------------------------------------ */

  var S = {
    name: "door", c: cls("door"), x: EXTRA.door,
    engineMode: "loading", engineReason: "",
    phase: "on",                 /* on | off | rest */
    h: null, st: null, tickAcc: 0, simT: 0,
    payloadTarget: 0, payloadApplied: 0,
    fbGap: cls("door").g_star_m, fbI: 0,     /* closed-form fallback float */
    hist: { t: [], gap: [], i: [] },
    ol: null, olT: 0, derived: null,
    drag: null, running: false, lastChart: 0, flux: 0
  };

  var stage = $("lev-stage"), canvas = $("lev-canvas"), chip = $("lev-chip");
  var runBtn = $("lev-run"), payloadIn = $("lev-payload");
  var offUp = $("lev-off-up"), offDown = $("lev-off-down"), onBtn = $("lev-on");
  if (!canvas || !stage) return;

  var col = {}, geo = null, cv = null;
  function colours() {
    var names = ["deep", "panel", "ink", "ink2", "muted", "line", "grid", "axis", "accent", "good", "ref",
      "door", "wall", "heavy", "drawer", "s4", "s5"];
    for (var i = 0; i < names.length; i++) col[names[i]] = P.color(names[i]);
    col.flux = P.color("--flux"); col.steel = P.color("--steel"); col.hair = P.color("--hair");
    col.elev = P.color("--bg-elev"); col.caution = P.color("--s4");
    col.sans = P.theme().sans; col.mono = P.theme().mono;
  }

  /* ---- engine ----------------------------------------------------------------------- */

  function destroyEngine() {
    if (S.h) { try { S.h.destroy(); } catch (e) { /* already gone */ } }
    S.h = null;
  }

  function createEngine() {
    destroyEngine();
    S.stopped = "";
    S.hist = { t: [], gap: [], i: [] };
    S.simT = 0; S.tickAcc = 0; S.payloadApplied = 0; S.st = null;
    S.fbGap = S.name === "wall" || S.name === "heavy" ? S.c.g_land_m : S.c.g_star_m;
    S.fbI = 0;
    if (S.engineMode !== "wasm" || !E) return false;
    var h = E.open(S.name, "heave1");
    if (!h) {
      S.engineMode = "formulas";
      S.engineReason = "the loaded engine has no Heave1 handle (" + (E.lastError() || "ABI v2 needed") + ")";
      return false;
    }
    S.h = h;
    if (S.name === "wall" || S.name === "heavy") h.setInputs({ command: "lift" });
    S.st = h.state();
    return true;
  }

  function mTotal() { return S.c.m_star_kg + S.payloadApplied; }

  function pushHist(t, gap, i) {
    var H = S.hist;
    H.t.push(t); H.gap.push(gap); H.i.push(i);
    while (H.t.length && H.t[0] < t - 12) { H.t.shift(); H.gap.shift(); H.i.shift(); }
  }

  /* Advance the float by simulated time dt (engine or closed-form fallback). */
  function advanceFloat(dt) {
    var rate = RAMP_FRAC_PER_S * S.c.m_star_kg;
    if (S.stopped) return;
    if (S.h) {
      S.tickAcc += dt * TICK_HZ;
      var n = Math.floor(S.tickAcc);
      S.tickAcc -= n;
      while (n > 0) {
        var k = Math.min(CHUNK_TICKS, n);
        var dts = k / TICK_HZ;
        S.payloadApplied += clamp(S.payloadTarget - S.payloadApplied, -rate * dts, rate * dts);
        S.h.setInputs({ floorAz: floorAzFor(S.c, S.payloadApplied) });
        var got = S.h.step(k);
        n -= k;
        if (got === 0) break;
        S.st = S.h.state();
        S.simT = S.st.t_s;
        if (S.hist.t.length === 0 || S.simT - S.hist.t[S.hist.t.length - 1] >= 0.019) pushHist(S.simT, S.st.gap_m, S.st.coil_i_a);
      }
      var status = S.h.status();
      if (status && status !== "running") {
        S.stopped = status + (E.lastError() ? ": " + E.lastError() : "");
        onBtn.disabled = false;
      }
      return;
    }
    /* closed-form fallback: first-order approach to the band-clamped zero-power gap */
    S.payloadApplied += clamp(S.payloadTarget - S.payloadApplied, -rate * dt, rate * dt);
    var m = mTotal(), gT = bandGap(S.c, m);
    var a = 1 - Math.exp(-dt / 0.12);
    S.fbGap += (gT - S.fbGap) * a;
    var iT = Math.abs(gT - zeroPowerGap(S.c, m)) > 1e-9 ? biasCurrent(S.c, S.x, gT, m) : 0;
    S.fbI += (iT - S.fbI) * a;
    S.simT += dt;
    pushHist(S.simT, S.fbGap, S.fbI);
  }

  function liveGap() {
    if (S.phase !== "on") return S.ol ? sampleOl(S.olT) : S.c.g_star_m;
    return S.h && S.st ? S.st.gap_m : S.fbGap;
  }
  function liveI() {
    if (S.phase !== "on") return 0;
    return S.h && S.st ? S.st.coil_i_a : S.fbI;
  }

  function sampleOl(t) {
    var o = S.ol;
    if (!o || !o.t.length) return S.c.g_star_m;
    if (t >= o.t[o.t.length - 1]) return o.g[o.g.length - 1];
    var lo = 0, hi = o.t.length - 1;
    while (hi - lo > 1) { var mid = (lo + hi) >> 1; if (o.t[mid] <= t) lo = mid; else hi = mid; }
    var f = (t - o.t[lo]) / Math.max(1e-12, o.t[hi] - o.t[lo]);
    return o.g[lo] + f * (o.g[hi] - o.g[lo]);
  }

  /* ---- control off / on ----------------------------------------------------------- */

  function controlOff(dirSign) {
    if (S.phase !== "on") return;
    var gLive = liveGap(), m = mTotal(), g0 = gLive + dirSign * NUDGE_M;
    var traj = openLoop(S.c, S.x, m, g0, 0, { tMax: 2 });
    var vCf = traj.hit ? impactClosedForm(S.c, S.x, m, g0, traj.hit) : 0;
    S.ol = traj; S.ol.m = m; S.ol.g0 = g0; S.ol.vCf = vCf; S.ol.nudge = dirSign;
    S.ol.p = poleAt(S.c, m);
    S.ol.gzp = zeroPowerGap(S.c, m);
    S.olT = UI.reducedMotion() ? traj.t[traj.t.length - 1] : 0;
    S.phase = "off";
    offUp.disabled = true; offDown.disabled = true; onBtn.disabled = false;
    destroyEngine();
    olChart();
    var physicsDecides = Math.abs(gLive - S.ol.gzp) > 2 * NUDGE_M;
    var where = traj.hit === "up" ? "the upper stop" : traj.hit === "down" ? "the landing rollers" : "no stop within 2 s";
    setText("lev-off-result", "Control off at " + mm(g0, 4) + " with m = " + fmt(m, 4) + " kg: reached " + where +
      (traj.hit ? " after " + fmt(traj.tHit * 1e3, 3) + " ms at " + fmt(traj.vHit * 1e3, 3) + " mm/s (energy closed form " +
        fmt(vCf * 1e3, 3) + " mm/s)" : "") + ". Open-loop pole p = " + fmt(S.ol.p, 3) + " rad/s, 1/p = " + fmt(1e3 / S.ol.p, 3) + " ms." +
      (physicsDecides ? " The band clamp was holding the gap " + fmt(Math.abs(gLive - S.ol.gzp) * 1e3, 2) +
        " mm away from the zero-power point, so the magnets, not the nudge, chose the direction." : ""));
    ensureRunning();
  }

  function controlOn() {
    S.phase = "on"; S.ol = null;
    offUp.disabled = false; offDown.disabled = false; onBtn.disabled = true;
    createEngine();
    if (UI.reducedMotion() && !loop.running()) settle(3);
    ensureRunning();
  }

  /* ---- canvas geometry ------------------------------------------------------------ */

  function layout() {
    var w = Math.max(280, Math.round(stage.clientWidth || 640));
    var narrow = w < 640;
    var h = narrow ? Math.round(clamp(w * 1.05 + 110, 440, 640)) : Math.round(clamp(w * 0.62, 360, 480));
    cv = P.setupCanvas(canvas, w, h);
    var dev, gauge;
    if (narrow) {
      dev = { x: 0, y: 0, w: w, h: h - 170 };
      gauge = { x: 0, y: dev.h, w: w, h: h - dev.h, horizontal: true };
    } else {
      dev = { x: 0, y: 0, w: Math.round(w * 0.64), h: h };
      gauge = { x: dev.w, y: 0, w: w - dev.w, h: h, horizontal: false };
    }
    var top = narrow ? 20 : 96;
    var railY = dev.y + top + 26;
    var pxPerM = (dev.h * 0.075) / S.c.g_land_m;
    var cw = dev.w * 0.52, cx = dev.x + dev.w * 0.5;
    geo = { w: w, h: h, narrow: narrow, dev: dev, gauge: gauge, railY: railY, pxPerM: pxPerM,
      cw: cw, cx: cx, mh: 16, bodyH: 28 };
    geo.tray = { x: dev.x + dev.w - Math.min(118, dev.w * 0.28) - 10, y: dev.y + dev.h - 70, w: Math.min(118, dev.w * 0.28), h: 58 };
  }

  function carriageTop(gap) { return geo.railY + clamp(gap, 0, S.c.g_land_m * 1.2) * geo.pxPerM; }

  function blockRects() {
    var T = geo.tray, out = [], bl = S.x.blocks, bw = (T.w - 16) / bl.length;
    for (var i = 0; i < bl.length; i++) {
      var hh = 16 + 22 * bl[i] / 0.2;
      out.push({ i: i, frac: bl[i], x: T.x + 8 + i * bw + 2, y: T.y + T.h - 8 - hh, w: bw - 4, h: hh });
    }
    return out;
  }

  function loadRect() {
    var gap = liveGap(), yT = carriageTop(gap) + geo.mh + geo.bodyH;
    var lx = geo.cx + geo.cw * 0.5 - 6;
    var frac = S.payloadTarget / S.c.m_star_kg;
    var hh = clamp(18 + 70 * frac, 18, 70);
    return { x: lx - 30, y: yT + 34, w: 40, h: hh, hookY: yT };
  }

  function hit(r, px, py) { return px >= r.x && px <= r.x + r.w && py >= r.y && py <= r.y + r.h; }

  function dropZone() {
    var gap = liveGap(), yT = carriageTop(gap);
    return { x: geo.cx - geo.cw / 2 - 20, y: yT - 10, w: geo.cw + 40, h: geo.dev.h - (yT - geo.dev.y) - 60 };
  }

  /* ---- drawing -------------------------------------------------------------------- */

  function line(ctx, x0, y0, x1, y1) { ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke(); }
  function text(ctx, s, x, y, o) {
    o = o || {};
    ctx.font = (o.weight || "400") + " " + (o.size || 11) + "px " + (o.mono ? col.mono : col.sans);
    ctx.fillStyle = o.color || col.muted;
    ctx.textAlign = o.align || "left";
    ctx.textBaseline = o.base || "alphabetic";
    ctx.fillText(s, x, y);
  }
  function rrect(ctx, x, y, w, h, r) { P.roundRect(ctx, x, y, w, h, r); }

  function classColour() { return col[S.name] || col.accent; }

  function draw(tNow) {
    if (!geo) layout();
    var ctx = cv.ctx, g = geo, D = g.dev;
    ctx.clearRect(0, 0, g.w, g.h);
    ctx.fillStyle = col.deep; ctx.fillRect(0, 0, g.w, g.h);
    var gap = liveGap(), iNow = liveI();
    var yMag = carriageTop(gap), mh = g.mh, bh = g.bodyH, cw = g.cw, cx = g.cx;
    var x0 = cx - cw / 2;

    /* ceiling hangers and the rail */
    ctx.strokeStyle = col.axis; ctx.lineWidth = 2;
    var railTop = g.railY - 26, rx0 = D.x + 14, rx1 = D.x + D.w - 14;
    for (var hk = 0; hk < 3; hk++) {
      var hx = rx0 + 30 + hk * (rx1 - rx0 - 60) / 2;
      line(ctx, hx, D.y, hx, railTop);
    }
    ctx.fillStyle = col.steel;
    ctx.fillRect(rx0, railTop, rx1 - rx0, 26);
    ctx.save();
    ctx.beginPath(); ctx.rect(rx0, railTop, rx1 - rx0, 26); ctx.clip();
    ctx.strokeStyle = P.alpha(col.deep, 0.35); ctx.lineWidth = 1;
    for (var lam = rx0; lam < rx1 + 30; lam += 7) line(ctx, lam, railTop, lam - 16, railTop + 26);
    ctx.restore();
    text(ctx, "steel rail (laminated), pole face underneath", rx0 + 6, railTop - 5, { size: 10.5 });

    /* rail cheeks with the landing lip */
    var yLandBody = carriageTop(S.c.g_land_m) + mh + bh;
    ctx.strokeStyle = col.steel; ctx.lineWidth = 4;
    line(ctx, x0 - 26, g.railY, x0 - 26, yLandBody + 14);
    line(ctx, x0 + cw + 26, g.railY, x0 + cw + 26, yLandBody + 14);
    line(ctx, x0 - 26, yLandBody + 14, x0 + 18, yLandBody + 14);
    line(ctx, x0 + cw - 18, yLandBody + 14, x0 + cw + 26, yLandBody + 14);
    ctx.fillStyle = col.panel; ctx.strokeStyle = col.ink2; ctx.lineWidth = 1.3;
    [x0 + 6, x0 + cw - 6].forEach(function (rxp) {
      ctx.beginPath(); ctx.arc(rxp, yLandBody + 7, 6.5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    });
    if (!g.narrow) text(ctx, "landing rollers", x0 + cw + 32, yLandBody + 12, { size: 10.5 });

    /* flux lines, density ∝ (M_pm + N·i)/(2g + h′) relative to the float */
    var hp = hPrime(S.c);
    var bRel = ((S.x.m_pm_a + S.x.turns * iNow) / (2 * gap + hp)) / (S.x.m_pm_a / (2 * S.c.g_star_m + hp));
    var modules = [cx - cw * 0.26, cx + cw * 0.26], mw = cw * 0.3;
    ctx.save();
    ctx.lineWidth = 1.3;
    if (!UI.reducedMotion() && S.running) ctx.lineDashOffset = -S.flux;
    ctx.setLineDash([5, 5]);
    for (var m = 0; m < 2; m++) {
      var mx = modules[m];
      for (var k = 0; k < 4; k++) {
        var spread = (k + 1) / 4, a0 = mx - mw * 0.14 * spread - 3, a1 = mx + mw * 0.5 * (0.35 + 0.65 * spread);
        var b0 = mx + mw * 0.14 * spread + 3, b1 = mx - mw * 0.5 * (0.35 + 0.65 * spread);
        var peak = g.railY - 3 - 6 * spread;
        ctx.strokeStyle = P.alpha(col.flux, clamp(0.25 + 0.5 * bRel - 0.1 * k, 0.12, 0.95));
        ctx.beginPath(); ctx.moveTo(a0, yMag); ctx.bezierCurveTo(a0, peak, a1, peak, a1, yMag); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(b0, yMag); ctx.bezierCurveTo(b0, peak, b1, peak, b1, yMag); ctx.stroke();
      }
    }
    ctx.restore();

    /* upper-stop pads on the carriage */
    var padH = S.x.g_up_m * g.pxPerM;
    ctx.fillStyle = P.alpha(col.ink2, 0.75);
    ctx.fillRect(x0 + 2, yMag - padH, 10, padH + mh);
    ctx.fillRect(x0 + cw - 12, yMag - padH, 10, padH + mh);

    /* magnet modules: PM block between two coil bobbins */
    for (var mm2 = 0; mm2 < 2; mm2++) {
      var mxx = modules[mm2];
      ctx.fillStyle = col.steel; ctx.fillRect(mxx - mw / 2, yMag + mh - 4, mw, 4);
      ctx.fillStyle = P.alpha(col.s5, 0.8); ctx.fillRect(mxx - mw * 0.14, yMag, mw * 0.28, mh - 4);
      text(ctx, "PM", mxx, yMag + mh - 7, { size: 9, align: "center", color: col.ink, weight: "700" });
      var glow = clamp(Math.abs(iNow) / S.x.i_cont_a, 0, 1);
      var coilCol = iNow >= 0 ? col.accent : col.s4;
      [mxx - mw * 0.36, mxx + mw * 0.36].forEach(function (bx) {
        ctx.fillStyle = P.alpha(col.s4, 0.35);
        ctx.fillRect(bx - mw * 0.13, yMag, mw * 0.26, mh - 4);
        if (glow > 0.002) {
          ctx.fillStyle = P.alpha(coilCol, clamp(0.15 + 0.85 * Math.sqrt(glow), 0, 0.95));
          ctx.fillRect(bx - mw * 0.13, yMag, mw * 0.26, mh - 4);
        }
        ctx.strokeStyle = P.alpha(col.ink2, 0.5); ctx.lineWidth = 1;
        for (var wl = 1; wl < 4; wl++) line(ctx, bx - mw * 0.13, yMag + wl * (mh - 4) / 4, bx + mw * 0.13, yMag + wl * (mh - 4) / 4);
      });
    }

    /* carriage body */
    ctx.fillStyle = P.alpha(classColour(), 0.22); ctx.strokeStyle = classColour(); ctx.lineWidth = 1.6;
    rrect(ctx, x0, yMag + mh, cw, bh, 4); ctx.fill(); ctx.stroke();
    text(ctx, "carriage", x0 + 8, yMag + mh + bh / 2 + 4, { size: 11, color: col.ink2 });

    /* the leaf, panel or shelving below */
    var leafTop = yMag + mh + bh, leafBot = D.y + D.h - 8;
    var lw = cw * 0.86, lx = cx - lw / 2;
    ctx.strokeStyle = col.ink2; ctx.lineWidth = 2;
    line(ctx, cx - lw * 0.3, leafTop, cx - lw * 0.3, leafTop + 10);
    line(ctx, cx + lw * 0.3, leafTop, cx + lw * 0.3, leafTop + 10);
    ctx.fillStyle = S.name === "door" ? P.alpha(col.flux, 0.1) : P.alpha(col.line, 0.45);
    ctx.strokeStyle = P.alpha(col.ink2, 0.8); ctx.lineWidth = 1.3;
    ctx.fillRect(lx, leafTop + 10, lw, leafBot - leafTop - 10);
    ctx.strokeRect(lx, leafTop + 10, lw, leafBot - leafTop - 10);
    if (S.name === "door") {
      ctx.strokeStyle = P.alpha(col.flux, 0.35); ctx.lineWidth = 1;
      line(ctx, lx + lw * 0.2, leafTop + 30, lx + lw * 0.45, leafTop + 80);
      line(ctx, lx + lw * 0.28, leafTop + 30, lx + lw * 0.53, leafTop + 80);
    } else if (S.name === "heavy") {
      ctx.strokeStyle = P.alpha(col.ink2, 0.5); ctx.lineWidth = 1;
      for (var sh = leafTop + 40; sh < leafBot - 6; sh += 28) line(ctx, lx + 4, sh, lx + lw - 4, sh);
    }
    text(ctx, S.x.leaf + " · m* = " + fmt(S.c.m_star_kg, 4) + " kg", lx + 8, leafTop + 26, { size: 11, color: col.ink2 });

    /* hanging payload */
    if (S.payloadTarget > 0.01 && !(S.drag && S.drag.kind === "load")) drawLoad(ctx, loadRect(), S.payloadTarget, false);

    /* gap dimension */
    var dimX = x0 - 12;
    ctx.strokeStyle = col.ink2; ctx.lineWidth = 1;
    line(ctx, dimX, g.railY, dimX, yMag);
    line(ctx, dimX - 4, g.railY, dimX + 4, g.railY);
    line(ctx, dimX - 4, yMag, dimX + 4, yMag);
    text(ctx, "gap " + mm(gap, 4), dimX - 8, (g.railY + yMag) / 2 + 4, { align: "right", size: 11.5, color: col.ink, weight: "600" });

    /* payload tray */
    var T = g.tray;
    ctx.fillStyle = P.alpha(col.panel, 0.9); ctx.strokeStyle = col.line; ctx.lineWidth = 1;
    rrect(ctx, T.x, T.y, T.w, T.h, 6); ctx.fill(); ctx.stroke();
    text(ctx, "weights: drag onto the carriage", T.x + T.w, T.y - 6, { size: 10, align: "right" });
    var br = blockRects();
    for (var b = 0; b < br.length; b++) {
      if (S.drag && S.drag.kind === "block" && S.drag.i === b) continue;
      drawBlock(ctx, br[b], br[b].frac * S.c.m_star_kg);
    }
    if (S.drag && S.drag.kind === "block") {
      var bb = br[S.drag.i];
      drawBlock(ctx, { x: S.drag.px - bb.w / 2, y: S.drag.py - bb.h / 2, w: bb.w, h: bb.h }, bb.frac * S.c.m_star_kg, true);
    }
    if (S.drag && S.drag.kind === "load") {
      var lr = loadRect();
      drawLoad(ctx, { x: S.drag.px - lr.w / 2, y: S.drag.py - lr.h / 2, w: lr.w, h: lr.h }, S.payloadTarget, true);
    }
    if (S.drag) {
      var dz = dropZone();
      ctx.setLineDash([4, 4]); ctx.strokeStyle = P.alpha(col.accent, 0.8); ctx.lineWidth = 1.2;
      rrect(ctx, dz.x, dz.y, dz.w, dz.h, 8); ctx.stroke(); ctx.setLineDash([]);
    }

    /* open-loop banner */
    if (S.phase !== "on" && S.ol) {
      var msg = S.phase === "off" ? "CONTROL OFF · coil current 0 · slow motion ×" + fmt(SLOWMO, 1) : "CONTROL OFF · resting on " + (S.ol.hit === "up" ? "the upper stop" : "the landing rollers");
      ctx.fillStyle = P.alpha(col.ref, 0.16); ctx.strokeStyle = col.ref; ctx.lineWidth = 1;
      var tw = Math.min(D.w - 20, 330), by = (leafTop + leafBot) / 2 - 13;
      rrect(ctx, cx - tw / 2, by, tw, 26, 6); ctx.fill(); ctx.stroke();
      text(ctx, msg, cx, by + 13, { align: "center", size: 11, color: col.ink, weight: "600", base: "middle" });
    }

    drawGauge(ctx, gap, iNow);
  }

  function drawBlock(ctx, r, kg, lifted) {
    ctx.fillStyle = P.alpha(col.ink2, lifted ? 0.9 : 0.75); ctx.strokeStyle = col.ink; ctx.lineWidth = 1;
    rrect(ctx, r.x, r.y, r.w, r.h, 3); ctx.fill(); ctx.stroke();
    text(ctx, fmt(kg, 3), r.x + r.w / 2, r.y + r.h / 2 + 1, { align: "center", base: "middle", size: 10, color: col.deep, weight: "700" });
  }

  function drawLoad(ctx, r, kg, lifted) {
    if (!lifted) {
      ctx.strokeStyle = col.ink2; ctx.lineWidth = 1.2;
      line(ctx, r.x + r.w / 2, r.hookY, r.x + r.w / 2, r.y);
    }
    ctx.fillStyle = P.alpha(col.ink2, 0.85); ctx.strokeStyle = col.ink; ctx.lineWidth = 1;
    rrect(ctx, r.x, r.y, r.w, r.h, 4); ctx.fill(); ctx.stroke();
    text(ctx, "+" + fmt(kg, 3), r.x + r.w / 2, r.y + r.h / 2 - 5, { align: "center", base: "middle", size: 10, color: col.deep, weight: "700" });
    text(ctx, "kg", r.x + r.w / 2, r.y + r.h / 2 + 7, { align: "center", base: "middle", size: 9.5, color: col.deep, weight: "600" });
  }

  function drawGauge(ctx, gap, iNow) {
    var G = geo.gauge, c = S.c, x = S.x;
    ctx.fillStyle = P.alpha(col.panel, 0.92); ctx.fillRect(G.x, G.y, G.w, G.h);
    ctx.strokeStyle = col.line; ctx.lineWidth = 1;
    if (G.horizontal) line(ctx, G.x, G.y + 0.5, G.x + G.w, G.y + 0.5); else line(ctx, G.x + 0.5, G.y, G.x + 0.5, G.y + G.h);
    var lo = x.g_up_m - 0.25 * (c.g_land_m - x.g_up_m), hi = c.g_land_m + 0.12 * (c.g_land_m - x.g_up_m);
    var m = mTotal(), gzp = zeroPowerGap(c, m), gb = bandGap(c, m);
    var pos, a0, a1;
    if (G.horizontal) {
      a0 = G.x + 22; a1 = G.x + G.w - 22;
      var yAx = G.y + 40;
      pos = function (v) { return a0 + (a1 - a0) * (v - lo) / (hi - lo); };
      text(ctx, "gap, to scale on this axis (mm)", G.x + 14, G.y + 17, { size: 11, color: col.ink2, weight: "600" });
      ctx.fillStyle = P.alpha(col.accent, 0.16);
      ctx.fillRect(pos(c.zp_band_lo_m), yAx - 8, pos(c.zp_band_hi_m) - pos(c.zp_band_lo_m), 16);
      ctx.strokeStyle = col.axis; ctx.lineWidth = 1.5; line(ctx, a0, yAx, a1, yAx);
      [[x.g_up_m, "upper stop"], [c.g_star_m, "g*"], [c.g_land_m, "rollers"]].forEach(function (q, idx) {
        ctx.strokeStyle = idx === 1 ? col.ink2 : col.ref; ctx.lineWidth = 1.2;
        line(ctx, pos(q[0]), yAx - 10, pos(q[0]), yAx + 10);
        text(ctx, q[1] + " " + fmt(q[0] * 1e3, 3), pos(q[0]), yAx + 24, { size: 10, align: "center" });
      });
      ctx.strokeStyle = col.good; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(pos(gb), yAx, 6, 0, 2 * Math.PI); ctx.stroke();
      var px = pos(clamp(gap, lo, hi));
      ctx.fillStyle = classColour();
      ctx.beginPath(); ctx.moveTo(px, yAx - 3); ctx.lineTo(px - 6, yAx - 14); ctx.lineTo(px + 6, yAx - 14); ctx.closePath(); ctx.fill();
      drawCurrentBar(ctx, G.x + 22, G.y + G.h - 36, G.w - 44, iNow);
      return;
    }
    a0 = G.y + 88; a1 = G.y + G.h * 0.62;
    var xAx = G.x + Math.min(70, G.w * 0.3);
    pos = function (v) { return a0 + (a1 - a0) * (v - lo) / (hi - lo); };
    text(ctx, "gap, to scale", G.x + 14, G.y + 30, { size: 11.5, color: col.ink2, weight: "600" });
    text(ctx, "on this axis", G.x + 14, G.y + 45, { size: 10.5 });
    ctx.fillStyle = P.alpha(col.accent, 0.16);
    ctx.fillRect(xAx - 9, pos(c.zp_band_lo_m), 18, pos(c.zp_band_hi_m) - pos(c.zp_band_lo_m));
    text(ctx, "zero-power band", xAx + 14, pos(c.zp_band_hi_m) - 3, { size: 10 });
    ctx.strokeStyle = col.axis; ctx.lineWidth = 1.5; line(ctx, xAx, a0, xAx, a1);
    [[x.g_up_m, "upper stop"], [c.g_star_m, "g* (m*)"], [c.g_land_m, "landing rollers"]].forEach(function (q, idx) {
      ctx.strokeStyle = idx === 1 ? col.ink2 : col.ref; ctx.lineWidth = 1.2;
      line(ctx, xAx - 11, pos(q[0]), xAx + 11, pos(q[0]));
      text(ctx, q[1], xAx + 14, pos(q[0]) + 4, { size: 10.5, color: idx === 1 ? col.ink2 : col.muted });
      text(ctx, fmt(q[0] * 1e3, 3), xAx - 15, pos(q[0]) + 4, { size: 10, align: "right" });
    });
    ctx.strokeStyle = col.good; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(xAx, pos(gb), 6, 0, 2 * Math.PI); ctx.stroke();
    var py = pos(clamp(gap, lo, hi));
    ctx.fillStyle = classColour();
    ctx.beginPath(); ctx.moveTo(xAx - 4, py); ctx.lineTo(xAx - 16, py - 6); ctx.lineTo(xAx - 16, py + 6); ctx.closePath(); ctx.fill();
    text(ctx, "◯ closed-form target " + fmt(gb * 1e3, 4) + " mm", G.x + 14, a1 + 26, { size: 10.5, color: col.ink2 });
    if (Math.abs(gb - gzp) > 1e-7) {
      text(ctx, "zero-power point " + fmt(gzp * 1e3, 4) + " mm", G.x + 14, a1 + 42, { size: 10, color: col.caution });
      text(ctx, "lies outside the band: bias current", G.x + 14, a1 + 55, { size: 10, color: col.caution });
    }
    drawCurrentBar(ctx, G.x + 14, G.y + G.h - 50, G.w - 28, iNow);
  }

  function drawCurrentBar(ctx, x, y, w, iNow) {
    var lim = S.x.i_cont_a;
    text(ctx, "mean coil current " + amps(iNow), x, y - 8, { size: 11, color: col.ink2, weight: "600" });
    ctx.fillStyle = P.alpha(col.line, 0.6); ctx.fillRect(x, y, w, 10);
    var mid = x + w / 2, f = clamp(iNow / lim, -1, 1);
    ctx.fillStyle = iNow >= 0 ? col.accent : col.s4;
    if (Math.abs(f) * w / 2 < 1.5 && Math.abs(iNow) > 0) ctx.fillRect(mid - 1, y - 2, 2, 14);
    else ctx.fillRect(Math.min(mid, mid + f * w / 2), y, Math.abs(f) * w / 2, 10);
    ctx.strokeStyle = col.ink2; ctx.lineWidth = 1; line(ctx, mid, y - 3, mid, y + 13);
    text(ctx, "−" + fmt(lim, 3) + " A", x, y + 24, { size: 10 });
    text(ctx, "0", mid, y + 24, { size: 10, align: "center" });
    text(ctx, "+" + fmt(lim, 3) + " A (i_cont)", x + w, y + 24, { size: 10, align: "right" });
  }

  /* ---- readouts and chip ------------------------------------------------------------ */

  var lastChip = "";
  function setChip(s) { if (s !== lastChip) { chip.textContent = s; lastChip = s; } }

  function engineLabel() {
    if (S.phase !== "on") return "control off: closed-form open-loop model in JavaScript (engine stopped)";
    if (S.engineMode === "wasm" && S.h) return "live: Rust controller + Heave1 plant in WebAssembly";
    if (S.engineMode === "loading") return "loading";
    return "closed-form fallback" + (S.engineReason ? " (" + S.engineReason + ")" : "");
  }

  function updateReadouts() {
    var m = mTotal(), c = S.c, gzp = zeroPowerGap(c, m), gb = bandGap(c, m);
    var clamped = Math.abs(gb - gzp) > 1e-9;
    var iCf = clamped ? biasCurrent(c, S.x, gb, m) : 0;
    setText("ro-gap", mm(liveGap(), 4));
    var src = S.phase !== "on" ? "open-loop model · " : S.h ? "plant truth · " : "closed form · ";
    setText("ro-gap-note", src + "zero-power g(m): " + mm(gzp, 4) + (clamped ? " → band edge " + mm(gb, 3) : ""));
    setText("ro-i", S.phase === "on" ? amps(liveI()) : "0 (off)");
    setText("ro-i-note", "closed form: " + (clamped ? amps(iCf) + " bias (band clamp)" : "0 at the zero-power point"));
    var mh = S.phase === "on" && S.h && S.st ? S.st.m_hat_kg : NaN;
    setText("ro-mhat", isFinite(mh) ? fmt(mh, 4) + " kg" : S.phase === "on" ? "closed form" : "–");
    setText("ro-mhat-note", "true m* + Δm: " + fmt(m, 4) + " kg");
    var box = $("ro-state-box"), stateTxt, note;
    if (S.phase !== "on") { stateTxt = "control off"; note = "open-loop model, JavaScript"; }
    else if (S.h && S.st) {
      stateTxt = LEV_NAMES[S.st.lev_state] || "state " + S.st.lev_state;
      note = "motion: " + (MODE_NAMES[S.st.motion_mode] || S.st.motion_mode);
      var mismatch = isFinite(mh) && mh > 1.05 * c.mass_rated_kg;
      if (mismatch) note = "m̂ > 1.05·m_rated: PayloadMismatch, motion hold-to-run";
      box.setAttribute("data-alert", mismatch || S.st.safety_demand >= 2 ? "on" : "off");
    } else { stateTxt = "closed form"; note = "engine not running"; box.setAttribute("data-alert", "off"); }
    setText("ro-state", stateTxt);
    setText("ro-state-note", note);
    setText("lev-engine", "engine: " + engineLabel());
    if (S.phase === "on") {
      setText("lev-off-result", "Open-loop pole at this load: p = " + fmt(poleAt(c, m), 3) + " rad/s (1/p = " + fmt(1e3 / poleAt(c, m), 3) + " ms).");
    }
  }

  function updateChip() {
    if (S.stopped) { setChip("The engine stopped (" + S.stopped + "). Press Control on to restart it."); return; }
    if (S.phase === "off") { setChip("Control off: the magnets alone, no coil current. The gap departs as cosh(p·t) toward a stop (closed-form open-loop model, slow motion)."); return; }
    if (S.phase === "rest" && S.ol) {
      setChip((S.ol.hit === "up" ? "Snapped up to the upper stop" : "Dropped onto the landing rollers") + " at " + fmt(S.ol.vHit * 1e3, 3) +
        " mm/s. Press Control on to restart the controller.");
      return;
    }
    var m = mTotal(), pl = S.payloadApplied;
    var load = Math.abs(pl) < 0.05 ? "no payload" : (pl > 0 ? "+" : "−") + fmt(Math.abs(pl), 3) + " kg";
    if (S.engineMode === "wasm" && S.h) {
      var st = S.st, lifting = st && (st.lev_state === 3 || st.lev_state === 4);
      setChip(lifting ? "Lifting off the landing rollers (the " + S.name.toUpperCase() + " rests landed)…" :
        "Live " + S.name.toUpperCase() + " float, " + load + ": zero-power control walks the gap to " + mm(bandGap(S.c, m), 4) + " and the current back to zero.");
    } else if (S.engineMode === "loading") {
      setChip("Loading the browser engine…");
    } else {
      setChip("Closed-form fallback (" + (S.engineReason || "engine unavailable") + "): the float is drawn from m(g); the controller is not running.");
    }
  }

  /* ---- charts ------------------------------------------------------------------------ */

  var lawChart = null, gapChart = null, curChart = null, olChartObj = null, stopChart = null, thChart = null;

  function lawSeries() {
    var c = S.c, x = S.x, lo = x.g_up_m, hi = c.g_land_m, n = 120, data = [];
    for (var k = 0; k <= n; k++) {
      var gg = lo + (hi - lo) * k / n;
      data.push([gg * 1e3, F.zeroPowerMass(gg, c.g_star_m, hPrime(c), c.m_star_kg)]);
    }
    return data;
  }

  function initCharts() {
    var lawEl = $("law-chart");
    if (lawEl) {
      lawChart = P.chart(lawEl, {
        kind: "line", title: "Zero-power mass against the gap",
        desc: "The mass that floats at zero coil current falls with the gap. The live operating point moves along this curve as the load changes.",
        x: { label: "gap", unit: "mm" }, y: { label: "floating mass", unit: "kg" },
        series: [{ key: "law", label: "m(g) = m*·((2g* + h′)/(2g + h′))²", color: S.name, data: lawSeries() }],
        height: 260, table: true, tableRows: 25
      });
    }
    var gEl = $("gap-chart");
    if (gEl) gapChart = P.chart(gEl, { kind: "line", title: "Gap over the last 12 s", desc: "Live gap from the engine's plant truth.", x: { label: "time", unit: "s" }, y: { label: "gap", unit: "mm" }, series: [], height: 170 });
    var iEl = $("cur-chart");
    if (iEl) curChart = P.chart(iEl, { kind: "line", title: "Mean coil current over the last 12 s", desc: "Live mean coil current: it returns to zero after each load change unless the band clamp holds the gap.", x: { label: "time", unit: "s" }, y: { label: "coil current", unit: "A" }, series: [], height: 170 });
    var oEl = $("ol-chart");
    if (oEl) olChartObj = P.chart(oEl, { kind: "line", title: "Open-loop gap after control off", desc: "Empty until the experiment runs.", x: { label: "time after control off", unit: "ms" }, y: { label: "gap", unit: "mm" }, series: [], height: 260 });
    stopCharts();
    thermalChart();
  }

  function refreshLaw() {
    if (!lawChart) return;
    var c = S.c, m = mTotal(), gap = liveGap();
    lawChart.update({
      series: [{ key: "law", label: "m(g) = m*·((2g* + h′)/(2g + h′))²", color: S.name, data: lawSeries() }],
      bands: [{ axis: "x", from: c.zp_band_lo_m * 1e3, to: c.zp_band_hi_m * 1e3, label: "zero-power band", color: "accent" }],
      refs: [
        { axis: "y", value: c.mass_rated_kg * 1.05, label: "1.05·m_rated: PayloadMismatch above", color: "muted", dash: [3, 4] },
        { axis: "x", value: S.x.g_up_m * 1e3, label: "upper stop", color: "ref" },
        { axis: "x", value: c.g_land_m * 1e3, label: "rollers", color: "ref" }
      ],
      points: [{ x: gap * 1e3, y: m, label: (S.phase === "on" ? "live: " : "") + fmt(m, 4) + " kg at " + fmt(gap * 1e3, 4) + " mm", color: S.name, dy: 16 }]
    });
  }

  function refreshStrips() {
    var H = S.hist;
    if (!gapChart || !curChart || !H.t.length) return;
    var t1 = H.t[H.t.length - 1], t0 = Math.max(0, t1 - 12);
    var gd = [], id = [];
    for (var k = 0; k < H.t.length; k++) { gd.push([H.t[k], H.gap[k] * 1e3]); id.push([H.t[k], H.i[k]]); }
    var m = mTotal(), gb = bandGap(S.c, m);
    gapChart.update({
      x: { label: "time", unit: "s", min: t0, max: Math.max(t0 + 1, t1) },
      series: [{ key: "gap", label: "gap", color: S.name, data: gd }],
      refs: [{ axis: "y", value: gb * 1e3, label: "target " + fmt(gb * 1e3, 4) + " mm", color: "good", dash: [4, 4] }]
    });
    curChart.update({
      x: { label: "time", unit: "s", min: t0, max: Math.max(t0 + 1, t1) },
      series: [{ key: "i", label: "coil current", color: S.name, data: id }],
      refs: [{ axis: "y", value: 0, color: "muted", dash: [2, 3] }],
      bands: [{ axis: "y", from: -1e-3, to: 1e-3, color: "good", alpha: 0.18 }]
    });
  }

  function olChart() {
    if (!olChartObj || !S.ol) return;
    var o = S.ol, data = [], ref = [], p = o.p, d0 = o.g0 - o.gzp;
    for (var k = 0; k < o.t.length; k++) {
      data.push([o.t[k] * 1e3, o.g[k] * 1e3]);
      var r = o.gzp + d0 * Math.cosh(p * o.t[k]);
      if (r > S.x.g_up_m * 0.9 && r < S.c.g_land_m * 1.1) ref.push([o.t[k] * 1e3, r * 1e3]);
    }
    olChartObj.update({
      desc: "Nonlinear open-loop gap against time after control off, with the linear cosh(p·t) divergence for comparison.",
      series: [
        { key: "ol", label: "nonlinear magnet force (RK4 here)", color: S.name, data: data },
        { key: "lin", label: "linear: δ₀·cosh(p·t), p = " + fmt(p, 3) + " rad/s", color: "muted", dash: [5, 4], width: 1.5, data: ref }
      ],
      refs: [
        { axis: "y", value: S.x.g_up_m * 1e3, label: "upper stop " + fmt(S.x.g_up_m * 1e3, 3) + " mm", color: "ref" },
        { axis: "y", value: S.c.g_land_m * 1e3, label: "landing rollers " + fmt(S.c.g_land_m * 1e3, 3) + " mm", color: "ref" },
        { axis: "y", value: o.gzp * 1e3, label: "zero-power float", color: "muted", dash: [2, 4], align: "left" }
      ],
      reveal: UI.reducedMotion() ? undefined : S.olT * 1e3,
      table: true, tableRows: 30
    });
    setText("ol-caption", "Control off at " + mm(o.g0, 4) + " (nudged " + (o.nudge > 0 ? "down" : "up") + " 5 µm), m = " + fmt(o.m, 4) + " kg: " +
      (o.hit ? (o.hit === "up" ? "upper stop" : "landing rollers") + " after " + fmt(o.tHit * 1e3, 3) + " ms at " + fmt(o.vHit * 1e3, 3) +
        " mm/s; the energy closed form gives " + fmt(o.vCf * 1e3, 3) + " mm/s." : "no stop reached.") +
      " Solid: nonlinear model integrated in your browser (RK4, 20 µs). Dashed: linear divergence δ₀·cosh(p·t). PHYSICS §3, §15; FAB-LEV-003, FAB-LEV-019.");
  }

  function stopCharts() {
    var el = $("stop-chart");
    var group = $("stop-class");
    var pick = "door";
    function render() {
      var c = cls(pick), x = EXTRA[pick], lo = x.g_up_m * 0.8, hi = c.g_land_m * 1.08, up = [], land = [];
      for (var k = 0; k <= 140; k++) {
        var gg = lo + (hi - lo) * k / 140;
        up.push([gg * 1e3, forceRatio(c, gg, c.m_star_kg)]);
        land.push([gg * 1e3, forceRatio(c, gg, c.m_op_min_kg)]);
      }
      var rUp = forceRatio(c, x.g_up_m, c.m_star_kg), rLand = forceRatio(c, c.g_land_m, c.m_op_min_kg);
      var spec = {
        kind: "line", title: "Magnet pull over weight against the gap, " + c.label,
        desc: "Ratio of the zero-current magnet pull to the weight, for m* and for the lightest operating mass, with the upper-stop and stay-landed rules.",
        x: { label: "gap", unit: "mm" }, y: { label: "pull / weight", min: 0.5, max: Math.max(1.6, rUp * 1.08) },
        series: [
          { key: "up", label: "with m* = " + fmt(c.m_star_kg, 4) + " kg", color: pick, data: up },
          { key: "land", label: "with m_op,min = " + fmt(c.m_op_min_kg, 4) + " kg", color: "muted", dash: [6, 4], data: land }
        ],
        refs: [
          { axis: "y", value: 1.3, label: "upper-stop rule ≤ 1.30", color: "ref" },
          { axis: "y", value: 0.9, label: "stay-landed rule ≤ 0.90", color: "ref" },
          { axis: "y", value: 1.0, label: "floats", color: "muted", dash: [2, 4], align: "left" },
          { axis: "x", value: x.g_up_m * 1e3, label: "g_up", color: "muted", dash: [2, 4] },
          { axis: "x", value: c.g_land_m * 1e3, label: "g_land", color: "muted", dash: [2, 4] }
        ],
        points: [
          { x: x.g_up_m * 1e3, y: rUp, label: fmt(rUp, 4) + (pick === "drawer" ? " (rollers 600 N)" : ""), color: pick },
          { x: c.g_land_m * 1e3, y: rLand, label: fmt(rLand, 4), color: pick, dy: 12 }
        ],
        height: 280, table: true, tableRows: 20
      };
      if (!stopChart && el) stopChart = P.chart(el, spec); else if (stopChart) stopChart.update(spec);
    }
    if (group) UI.segmented(group, function (v) { pick = v; render(); });
    render();
    fillStopTable();
  }

  function fillStopTable() {
    var rows = doc.querySelectorAll("#stop-table tr[data-class]");
    for (var r = 0; r < rows.length; r++) {
      var name = rows[r].getAttribute("data-class"), c = cls(name), x = EXTRA[name];
      var d = S.derived && S.derived[name];
      var up = d && isFinite(d.stop_ratio_up) ? d.stop_ratio_up : forceRatio(c, x.g_up_m, c.m_star_kg);
      var land = d && isFinite(d.stay_landed_ratio) ? d.stay_landed_ratio : forceRatio(c, c.g_land_m, c.m_op_min_kg);
      var cells = rows[r].querySelectorAll("[data-k]");
      for (var q = 0; q < cells.length; q++) {
        var k = cells[q].getAttribute("data-k");
        if (k === "up") cells[q].textContent = fmt(up, 4) + (name === "drawer" ? " (rollers 600 N)" : "");
        if (k === "land") cells[q].textContent = fmt(land, 4);
      }
    }
    if (S.derived) setText("stop-source", "Values from Params.derived() (stop_ratio_up, stay_landed_ratio) computed in your browser by the WebAssembly engine. The rounded stay-landed quotes in SPECIFICATION (0.824 / 0.850) differ from the exact 0.8250 / 0.8487 through the balanced pole length (PHYSICS §3).");
  }

  function fillImpactTable() {
    var rows = doc.querySelectorAll("#impact-table tr[data-class]");
    for (var r = 0; r < rows.length; r++) {
      var name = rows[r].getAttribute("data-class"), c = cls(name), x = EXTRA[name];
      var up = openLoop(c, x, c.m_star_kg, c.g_star_m - 1e-7, 0, { tMax: 3, every: 1 });
      var dn = openLoop(c, x, c.m_star_kg, c.g_star_m + 1e-7, 0, { tMax: 3, every: 1 });
      var cells = rows[r].querySelectorAll("[data-k]");
      for (var q = 0; q < cells.length; q++) {
        var k = cells[q].getAttribute("data-k");
        if (k === "p") cells[q].textContent = fmt(F.openLoopPole(c.g_star_m, hPrime(c)), 3) + " rad/s";
        if (k === "up") cells[q].textContent = up.hit === "up" ? fmt(up.vHit * 1e3, 4) + " mm/s" : "–";
        if (k === "down") cells[q].textContent = dn.hit === "down" ? fmt(dn.vHit * 1e3, 4) + " mm/s" : "–";
      }
    }
  }

  function thermalChart() {
    var el = $("thermal-chart"), input = $("th-dt");
    var names = ["drawer", "door", "wall"];
    function render() {
      var dT = input ? parseFloat(input.value) : 40;
      var series = [], pts = [];
      for (var i = 0; i < names.length; i++) {
        var c = cls(names[i]), x = EXTRA[names[i]];
        series.push({ key: names[i], label: names[i] === "wall" ? "WALL, HEAVY" : c.label, color: names[i],
          data: [[-20, thermalShift(c, x, -20) * 1e3], [60, thermalShift(c, x, 60) * 1e3]] });
        pts.push({ x: dT, y: thermalShift(c, x, dT) * 1e3, label: fmt(thermalShift(c, x, dT) * 1e3, 3) + " mm", color: names[i] });
      }
      var spec = {
        kind: "line", title: "Zero-power gap shift against magnet temperature rise",
        desc: "Straight lines through zero: a warmer magnet floats closer to the rail. DRAWER, DOOR and WALL (HEAVY equals WALL).",
        x: { label: "ΔT", unit: "K", min: -20, max: 60 }, y: { label: "Δg", unit: "mm" },
        series: series, points: pts,
        refs: [{ axis: "x", value: 40, label: "+40 K anchors", color: "muted", dash: [2, 4] }, { axis: "y", value: 0, color: "muted", dash: [2, 3] }],
        height: 260, table: true
      };
      if (!thChart && el) thChart = P.chart(el, spec); else if (thChart) thChart.update(spec);
      var alpha = EXTRA.door.tempco, br = 1 + alpha * dT;
      setText("th-out", (dT >= 0 ? "+" : "−") + fmt(Math.abs(dT), 3) + " K");
      setText("th-br", fmt(br * 100, 4) + " %");
      setText("th-load", (1 / (br * br) - 1 >= 0 ? "+" : "−") + fmt(Math.abs(1 / (br * br) - 1) * 100, 3) + " %");
      setText("th-door", mm(cls("door").g_star_m + thermalShift(cls("door"), EXTRA.door, dT), 4));
      setText("th-wall", mm(cls("wall").g_star_m + thermalShift(cls("wall"), EXTRA.wall, dT), 4));
    }
    if (input) input.addEventListener("input", render);
    render();
  }

  /* ---- payload controls ------------------------------------------------------------ */

  function setPayloadKg(kg, fromSlider) {
    var c = S.c;
    kg = clamp(kg, -0.15 * c.m_star_kg, 0.3 * c.m_star_kg);
    S.payloadTarget = kg;
    if (!fromSlider && payloadIn) payloadIn.value = String(Math.round((kg / c.m_star_kg) * 200) / 2);
    var pct = (kg / c.m_star_kg) * 100;
    var out = $("lev-payload-out");
    if (out) out.textContent = (kg >= 0 ? "+" : "−") + fmt(Math.abs(kg), 3) + " kg (" + (pct >= 0 ? "+" : "−") + fmt(Math.abs(pct), 3) + " %)";
    if (payloadIn) payloadIn.setAttribute("aria-valuetext", out ? out.textContent : "");
    if (UI.reducedMotion() && !loop.running() && S.phase === "on") settle(3);
    ensureRunning();
  }

  /* Reduced motion: no animation, so a change is stepped to its settled state at once and
     drawn as one static frame (3 s of Heave1 is about 0.1 s of compute). */
  function settle(seconds) {
    var n = Math.ceil(seconds / 0.05);
    for (var k = 0; k < n; k++) advanceFloat(0.05);
  }

  if (payloadIn) payloadIn.addEventListener("input", function () {
    setPayloadKg((parseFloat(payloadIn.value) / 100) * S.c.m_star_kg, true);
  });

  function canvasPoint(ev) {
    var r = canvas.getBoundingClientRect();
    return { x: ev.clientX - r.left, y: ev.clientY - r.top };
  }

  canvas.addEventListener("pointerdown", function (ev) {
    if (!geo) return;
    var p = canvasPoint(ev), br = blockRects();
    for (var i = 0; i < br.length; i++) {
      if (hit(br[i], p.x, p.y)) { S.drag = { kind: "block", i: i, px: p.x, py: p.y }; break; }
    }
    if (!S.drag && S.payloadTarget > 0.01 && hit(loadRect(), p.x, p.y)) S.drag = { kind: "load", px: p.x, py: p.y };
    if (S.drag) {
      try { canvas.setPointerCapture(ev.pointerId); } catch (e) { /* capture unsupported */ }
      canvas.setAttribute("data-drag", "on");
      ev.preventDefault();
      frameOnce();
    }
  });
  canvas.addEventListener("pointermove", function (ev) {
    if (!geo) return;
    var p = canvasPoint(ev);
    if (S.drag) { S.drag.px = p.x; S.drag.py = p.y; ev.preventDefault(); frameOnce(); return; }
    var over = blockRects().some(function (b) { return hit(b, p.x, p.y); }) || (S.payloadTarget > 0.01 && hit(loadRect(), p.x, p.y));
    canvas.setAttribute("data-drag", over ? "over" : "");
  });
  function endDrag(ev) {
    if (!S.drag) return;
    var p = canvasPoint(ev), d = S.drag, inZone = hit(dropZone(), p.x, p.y);
    S.drag = null;
    canvas.setAttribute("data-drag", "");
    if (d.kind === "block" && inZone) setPayloadKg(Math.max(0, S.payloadTarget) + S.x.blocks[d.i] * S.c.m_star_kg);
    else if (d.kind === "load" && !inZone) setPayloadKg(0);
    frameOnce();
  }
  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", function () { S.drag = null; canvas.setAttribute("data-drag", ""); frameOnce(); });

  /* ---- class switch ------------------------------------------------------------------- */

  function setClass(name) {
    if (!EXTRA[name]) return;
    S.name = name; S.c = cls(name); S.x = EXTRA[name];
    S.phase = "on"; S.ol = null;
    offUp.disabled = false; offDown.disabled = false; onBtn.disabled = true;
    layout();
    createEngine();
    setPayloadKg(0);
    refreshLaw();
    ensureRunning();
  }

  /* ---- animation loop ---------------------------------------------------------------- */

  var loop = P.loop(function (dt) {
    frame(dt);
  }, { el: stage, still: function () { frameOnce(); }, maxDt: 0.05 });

  function frame(dt) {
    S.running = true;
    S.flux += dt * 22;
    if (S.phase === "on") advanceFloat(dt);
    else if (S.phase === "off") {
      S.olT += dt * SLOWMO;
      var end = S.ol.t[S.ol.t.length - 1];
      if (S.olT >= end) { S.olT = end; S.phase = "rest"; }
      olChart();
    }
    draw();
    var now = Date.now();
    if (now - S.lastChart > 160) { S.lastChart = now; updateReadouts(); updateChip(); refreshLaw(); refreshStrips(); }
  }

  function frameOnce() {
    if (!geo) layout();
    if (S.phase === "off" && UI.reducedMotion()) { S.olT = S.ol.t[S.ol.t.length - 1]; S.phase = "rest"; olChart(); }
    draw(); updateReadouts(); updateChip(); refreshLaw(); refreshStrips();
  }

  function ensureRunning() {
    if (UI.reducedMotion() && !loop.running()) {
      if (runBtn) runBtn.hidden = false;
      frameOnce();
      return;
    }
    loop.start(UI.reducedMotion() ? true : undefined);
  }

  if (runBtn) runBtn.addEventListener("click", function () {
    if (loop.running()) { loop.stop(); runBtn.textContent = "Run the live engine"; runBtn.setAttribute("aria-pressed", "false"); frameOnce(); }
    else { loop.start(true); runBtn.textContent = "Pause the live engine"; runBtn.setAttribute("aria-pressed", "true"); }
  });
  offUp.addEventListener("click", function () { controlOff(-1); });
  offDown.addEventListener("click", function () { controlOff(+1); });
  onBtn.addEventListener("click", controlOn);
  onBtn.disabled = true;

  var group = $("lev-class");
  if (group) UI.segmented(group, function (v) { setClass(v); });

  P.onTheme(function () { colours(); frameOnce(); });
  if (root.ResizeObserver) {
    var lastW = 0;
    new root.ResizeObserver(function () {
      var w = stage.clientWidth;
      if (w && Math.abs(w - lastW) >= 1) { lastW = w; layout(); frameOnce(); }
    }).observe(stage);
  } else root.addEventListener("resize", function () { layout(); frameOnce(); });

  /* ---- start -------------------------------------------------------------------------- */

  colours();
  layout();
  initCharts();
  fillImpactTable();
  setPayloadKg(0);
  var simEl = $("lev-sim");
  function labelSims() {
    var kind = S.engineMode === "wasm" ? "" : "closed";
    ["lev-sim", "law-sim", "strip-sim"].forEach(function (id) {
      var el = $(id);
      if (!el) return;
      el.textContent = "";
      el.setAttribute("data-simulated", kind);
      el.classList.remove("sim-label");
      el.removeAttribute("title");
    });
    UI.renderSimLabels(doc);
    if (simEl && S.engineMode !== "wasm") simEl.textContent = "closed form (fallback)";
  }

  function onEngine(r) {
    S.engineMode = r && r.mode === "wasm" ? "wasm" : "formulas";
    S.engineReason = r && r.reason ? r.reason : "";
    if (S.engineMode === "wasm") {
      try {
        S.derived = {};
        ["drawer", "door", "wall", "heavy"].forEach(function (n) { S.derived[n] = E.derivedAnchors(n); });
        if (!S.derived.door) S.derived = null;
      } catch (e) { S.derived = null; }
      fillStopTable();
    }
    createEngine();
    labelSims();
    ensureRunning();
  }

  if (E && typeof E.load === "function") {
    var force = wantForce();
    E.load(force ? { force: true } : undefined).then(function (r) {
      if (force) E.renderSelftestBadges();
      onEngine(r);
    }, function () { onEngine({ mode: "formulas", reason: "loader error" }); });
  } else {
    onEngine({ mode: "formulas", reason: "engine.js missing" });
  }
  frameOnce();
})(typeof window !== "undefined" ? window : globalThis);
