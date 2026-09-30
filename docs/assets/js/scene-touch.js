/* fabrail touch scene (plan §3b). Owner: W2.
 *
 * What runs where:
 *   - The playground: the real Rust observer, intent detector, admittance, effects,
 *     terminus, governor, obstruction detector and supervisor on the Carriage6 plant
 *     (crates/fabrail-web through assets/js/engine.js), stepped in 1 ms slices. The
 *     engine's hand input is a pure force source on the front, so the page turns the
 *     pointer into a force through a virtual hand spring and damper (COUPLING below):
 *     F = clamp(k·(x_pointer − x) + c·(v_pointer − v), ±F_max). The keyboard pushes
 *     with a constant force. Nothing else about the hand is modelled.
 *   - Without the WebAssembly engine the playground runs a JavaScript copy of the
 *     PHYSICS §17 admittance law (virtual mass and damper, detent, soft-close spring,
 *     governor clamp, rigid ends) and says so; it has no observer, intent, obstruction
 *     or supervisor.
 *   - The passivity lab: a JavaScript sandbox of a sampled virtual spring on a 1-D mass
 *     with the class's physical damping and fast period, with and without the
 *     Hannaford–Ryu passivity controller (stored-energy reference), mirroring
 *     core::haptics::passivity (PHYSICS §17). The engine never renders such a spring.
 *
 * Under node, require("docs/assets/js/scene-touch.js") returns the pure core:
 * { COUPLING, HAPTIC, handForce, MassEstimator, passivitySim, JsAdmittance, describe,
 *   runScript } for headless checks against the wasm engine.
 */
(function (root) {
  "use strict";

  var TICK_HZ = 20000;
  var SLICE_TICKS = 20;             /* 1 ms engine slices: the hand force is updated per slice */
  var E_MAX_J = 1.69, P_MAX = 22.3;

  function formulas() {
    if (root.FabrailFormulas) return root.FabrailFormulas;
    if (typeof require === "function") {
      try { return require("./formulas.js"); } catch (e) { return null; }
    }
    return null;
  }
  var F = formulas();

  /* Virtual hand (page choice): spring k, damper c on the pointer velocity, force cap,
     keyboard push force. Chosen so the coupled admittance is well damped and a firm
     push reaches the DRAWER push-to-open displacement. */
  var COUPLING = {
    drawer: { k: 1500, c: 150, fmax: 30, fkey: 14 },
    door: { k: 2000, c: 400, fmax: 60, fkey: 25 },
    wall: { k: 4000, c: 1500, fmax: 150, fkey: 80 }
  };

  /* Haptic presets (fabrail_params::presets control.haptic, PHYSICS §17 intent and
     observer figures, design.damping.b_dist, control.rates): assist ratio, B_v, detent,
     soft-close, intent F_th, obstruction th0, physical damping and fast period. */
  var HAPTIC = {
    drawer: { ratio: 1.5, bv: 10, detX: 0.15, detF: 3, detW: 0.002, scW: 2 * Math.PI * 2, scCap: 0.045,
      fth: 4.5, th0: 12, bDist: 2, T: 1e-4, len: 0.6, world: [-0.72, 0.52] },
    door: { ratio: 3, bv: 40, detX: 0.5, detF: 8, detW: 0.003, scW: 2 * Math.PI * 1.5, scCap: 0.065,
      fth: 10, th0: 30, bDist: 5, T: 1e-4, len: 1.0, world: [-1.25, 1.25] },
    wall: { ratio: 5, bv: 450, detX: 3.0, detF: 20, detW: 0.005, scW: 2 * Math.PI * 0.5, scCap: 0.115,
      fth: 25, th0: 60, bDist: 20, T: 2e-4, len: 2.0, world: [-2.3, 6.5] }
  };

  function clamp(v, a, b) { return Math.min(Math.max(v, a), b); }

  function handForce(cp, xTarget, vTarget, x, v) {
    return clamp(cp.k * (xTarget - x) + cp.c * (vTarget - v), -cp.fmax, cp.fmax);
  }

  /* Least squares for M in F − B_v·v = M·a with exponential forgetting (time constant tau). */
  function MassEstimator(tau) {
    this.tau = tau || 0.5; this.sfa = 0; this.saa = 0; this.sw = 0;
  }
  MassEstimator.prototype.update = function (f, a, v, bv, dt, active) {
    var w = Math.exp(-dt / this.tau);
    this.sfa *= w; this.saa *= w; this.sw *= w;
    if (!active) return;
    this.sfa += (f - bv * v) * a * dt; this.saa += a * a * dt; this.sw += dt;
  };
  MassEstimator.prototype.value = function () {
    if (!(this.sw > 0.05)) return null;
    var aRms = Math.sqrt(this.saa / this.sw);
    return aRms > 0.05 ? this.sfa / this.saa : null;
  };
  MassEstimator.prototype.reset = function () { this.sfa = 0; this.saa = 0; this.sw = 0; };

  /* Passivity sandbox (PHYSICS §17): mass m with physical damping b; the virtual spring
     K is sampled every T and applied one sample later (T_d = T) with a zero-order hold,
     so the spring force over [kT, (k+1)T) is −K·x_{k−1}. The observer counts the energy
     flowing into the virtual spring exactly, E ← E + K·x_{k−1}·Δx, against its stored
     energy W = ½K·x². With the controller on, when E − W < 0 and v ≠ 0:
     α = min(−(E − W)/(T·v²), α_max), f_pc = −α·v over the next hold, E ← E + α·v²·T
     (Hannaford and Ryu with the stored-energy reference). The plant is integrated
     exactly over each hold. */
  function passivitySim(o) {
    var m = o.m, b = o.b, T = o.T, K = o.K, aMax = o.alphaMax, pc = !!o.pc;
    var dur = o.dur || 20, every = o.every || 0.005, n = Math.round(dur / T);
    var x = 0, v = o.v0 === undefined ? 0.05 : o.v0, xPrev = 0, uS = 0, uP = 0, fpc = 0;
    var E = 0, ts = [0], xs = [0], es = [0], next = every, peak = 0, minEW = 0, alphaPeak = 0;
    var ebm = Math.exp(-(b * T) / m);
    for (var k = 1; k <= n; k++) {
      /* plant over one hold with force u */
      var x0 = x, v0 = v, vinf = (uS + uP) / b;
      v = vinf + (v0 - vinf) * ebm;
      x = x0 + vinf * T + (v0 - vinf) * (m / b) * (1 - ebm);
      E -= uS * (x - x0);
      /* controller at sample k: spring from the previous sample (T_d = T), PC on v_k */
      fpc = 0;
      if (pc) {
        var gap = E - 0.5 * K * x * x;
        if (gap < 0 && Math.abs(v) > 1e-6) {
          var alpha = Math.min(-gap / (T * v * v), aMax);
          fpc = -alpha * v;
          E += alpha * v * v * T;
          if (alpha > alphaPeak) alphaPeak = alpha;
        }
      }
      uS = -K * xPrev; uP = fpc;
      xPrev = x;
      var t = k * T;
      var ew = E - 0.5 * K * x * x;
      if (ew < minEW) minEW = ew;
      if (t > dur - 2 && Math.abs(x) > peak) peak = Math.abs(x);
      if (t >= next - 1e-12) { ts.push(t); xs.push(x); es.push(-(ew)); next += every; }
      if (!isFinite(x) || Math.abs(x) > 1) break;
    }
    return { t: ts, x: xs, egen: es, peakLast2s: peak, minEW: minEW, alphaPeak: alphaPeak,
      kMax: (2 * b) / (T + 2 * T), net: b - K * (T / 2 + T) };
  }

  /* Fallback admittance (PHYSICS §17), no observer: M_v·ẍ + B_v·ẋ = F + F_eff, governor
     clamp, ends at [0, travel]. F_eff = detent −F_d·sin(π(x − x_k)/w) within w, and the
     soft-close spring −M_v(ω²e + 2ωė)·s(|e|) inside the capture when no hand pushes. */
  function JsAdmittance(name) {
    var c = F.CLASSES[name], hp = HAPTIC[name];
    this.name = name; this.c = c; this.hp = hp;
    this.m = c.m_star_kg; this.mv = this.m / hp.ratio;
    this.x = 0; this.v = 0; this.a = 0; this.t = 0;
    this.vCap = Math.min(c.v_open_max_mps, Math.sqrt(2 * E_MAX_J / Math.max(c.mass_rated_kg, 1.05 * this.m)));
  }
  JsAdmittance.prototype.effects = function (fHand) {
    var hp = this.hp, f = 0, e = this.x - hp.detX;
    if (Math.abs(e) <= hp.detW) f += -hp.detF * Math.sin((Math.PI * e) / hp.detW);
    if (Math.abs(fHand) < 0.5 && this.x < hp.scCap) {
      var s = this.x < hp.scCap / 2 ? 1 : (function (u) { return u * u * (3 - 2 * u); })(2 * (hp.scCap - this.x) / hp.scCap);
      f += -this.mv * (hp.scW * hp.scW * this.x + 2 * hp.scW * this.v) * s;
    }
    return f;
  };
  JsAdmittance.prototype.step = function (dt, fHand) {
    var n = Math.max(1, Math.round(dt / 5e-4)), h = dt / n;
    for (var k = 0; k < n; k++) {
      var fNet = fHand + this.effects(fHand);
      var vN = (this.v + (h * fNet) / this.mv) / (1 + (h * this.hp.bv) / this.mv);
      vN = clamp(vN, -this.vCap, this.vCap);
      this.a = (vN - this.v) / h;
      this.v = vN;
      this.x += this.v * h;
      if (this.x < 0) { this.x = 0; this.v = Math.max(0, this.v); }
      if (this.x > this.c.travel_m) { this.x = this.c.travel_m; this.v = Math.min(0, this.v); }
      this.t += h;
    }
  };

  /* Names (fabrail_types wire values; FaultSet.bits[0] = registry indices 0…31). */
  var LEV_NAMES = ["boot", "self-test", "seated", "landed", "lifting", "levitated, idle", "moving",
    "stopping", "landing", "seating", "fault reaction (SS1)", "fault (latched)", "seismic action",
    "escape", "manual"];
  var MODE_NAMES = ["propulsion off", "hold", "S-curve trajectory", "admittance", "soft-close", "reverse",
    "freeze", "centre", "isolate", "damp", "terminus (close)", "terminus (open)", "inertial smoothing"];
  var DEMAND_NAMES = ["none", "limit force", "safe stop", "SS1 safe landing", "STO (LSM)", "STO + ASC (LSM)"];
  var FAULTS_LO = ["GapRange", "GapStuck", "GapDisagree", "EncoderLost", "EncoderJump", "AccelLost",
    "AccelStuck", "ImuLost", "RailAccelLost", "CurrentSense", "CoilOvercurrent", "CoilOpen", "CoilDriver",
    "CoilI2t", "LsmOvercurrent", "LsmPhaseSum", "LsmDriver", "LockFailed", "LatchFailed", "BusUnder",
    "BusOver", "MainsLost", "HoldupLow", "OverTemp", "FastOverrun", "SupervisorOverrun", "WatchdogReset",
    "ParamCrc", "ParamRange", "SelftestHash", "NonFinite", "HeartbeatLost"];

  /* Plain-language description of a controller state change (null when nothing to say). */
  function describe(name, prev, s) {
    var out = [];
    if (!prev || s.lev_state !== prev.lev_state) {
      var L = s.lev_state;
      if (L === 4) out.push([name === "drawer" ? "unseating: lift-off within the seat clearance" : "lifting off the landing rollers", ""]);
      else if (L === 9) out.push(["seating: 0.2 s z-step onto the captive seat", ""]);
      else if (L === 2) out.push(["seated on the captive seat", ""]);
      else if (L === 3) out.push(["landed on the rollers", ""]);
      else if (L === 8) out.push(["landing", ""]);
      else if (L === 10) out.push(["fault reaction: SS1 safe landing", "alert"]);
      else if (L === 11) out.push(["fault latched: Restart to clear", "alert"]);
      else if (L === 12) out.push(["seismic action", "alert"]);
    }
    if (!prev || s.motion_mode !== prev.motion_mode) {
      var M = s.motion_mode;
      if (M === 3) out.push(["admittance: rendering M_v = m̂/" + HAPTIC[name].ratio + ", B_v = " + HAPTIC[name].bv + " N·s/m", ""]);
      else if (M === 2) out.push([prev && prev.motion_mode === 3 ? "intent read: finishing the move on an S-curve under the governor" : "S-curve move under the governor", ""]);
      else if (M === 10 || M === 11) out.push(["log-damped terminus toward the " + (M === 10 ? "closed" : "open") + " end", ""]);
      else if (M === 5) out.push(["obstruction: stop, then reverse 50 mm", "alert"]);
      else if (M === 0 && s.lev_state === 7) out.push(["obstruction: propulsion released to 0 N", "alert"]);
      else if (M === 1 && prev && prev.motion_mode !== 1) out.push(["hold position", ""]);
      else if (M === 4) out.push(["soft-close spring", ""]);
    }
    if (prev && s.safety_demand !== prev.safety_demand && s.safety_demand > 0) {
      out.push(["monitor demand: " + (DEMAND_NAMES[s.safety_demand] || s.safety_demand), "alert"]);
    }
    if (prev && s.fault_bits_lo !== prev.fault_bits_lo) {
      var added = (s.fault_bits_lo >>> 0) & ~(prev.fault_bits_lo >>> 0), names = [];
      for (var b = 0; b < 32; b++) if (added & (1 << b)) names.push(FAULTS_LO[b] || "bit " + b);
      if (names.length) out.push(["fault: " + names.join(", "), "alert"]);
    }
    return out;
  }

  /* Headless: run a pointer script on an engine handle. script: [[t, xTarget | null]]
     (null = hand released), piecewise constant. Returns the event list and final state. */
  function runScript(h, name, script, seconds, autoRelease) {
    var cp = COUPLING[name], s = h.state(), prev = null, events = [], si = 0, xT = null, lastXT = null, vT = 0;
    var n = Math.round((seconds * TICK_HZ) / SLICE_TICKS), dt = SLICE_TICKS / TICK_HZ, fPeak = 0;
    var grab = null, released = false, stillFor = 0;
    for (var k = 0; k < n; k++) {
      var t = k * dt;
      while (si < script.length && script[si][0] <= t + 1e-12) {
        xT = script[si][1]; si++; released = false;
        if (xT !== null && !grab) grab = { startMode: s.motion_mode };
        if (xT === null) grab = null;
      }
      vT = xT !== null && lastXT !== null ? (xT - lastXT) / dt : 0;
      stillFor = vT === 0 ? stillFor + dt : 0;
      lastXT = xT;
      if (autoRelease && grab && !released && shouldRelease(grab.startMode, s.motion_mode, stillFor)) {
        released = true; events.push([+s.t_s.toFixed(3), "hand released (the controller took over)"]);
      }
      if (released) xT = null;
      var f = xT === null ? 0 : handForce(cp, xT, vT, s.x_m, s.v_mps);
      fPeak = Math.max(fPeak, Math.abs(f));
      h.setInputs({ handN: f });
      h.step(SLICE_TICKS);
      s = h.state();
      var d = describe(name, prev, s);
      for (var q = 0; q < d.length; q++) events.push([+s.t_s.toFixed(3), d[q][0]]);
      prev = s;
    }
    return { events: events, state: s, fPeak: fPeak };
  }

  /* The page's hand lets go when the controller takes over a gesture the hand started: a
     powered mode (S-curve, terminus) that began after the grab. A grab made during
     powered motion is a deliberate hold and is kept. (A mouse cannot feel the carriage
     pull away; a hand would.) RELEASE_STILL_S is the pointer stillness required, 0: the
     release is immediate. */
  var RELEASE_STILL_S = 0;
  function powered(mode) { return mode === 2 || mode === 10 || mode === 11; }
  function shouldRelease(startMode, mode, stillForS) {
    return powered(mode) && !powered(startMode) && stillForS >= RELEASE_STILL_S;
  }

  var CORE = { COUPLING: COUPLING, HAPTIC: HAPTIC, handForce: handForce, MassEstimator: MassEstimator,
    shouldRelease: shouldRelease,
    passivitySim: passivitySim, JsAdmittance: JsAdmittance, describe: describe, runScript: runScript,
    FAULTS_LO: FAULTS_LO };
  if (typeof module === "object" && module.exports) module.exports = CORE;

  /* ==================================================================================== */
  /* DOM layer                                                                            */
  /* ==================================================================================== */

  var doc = root.document;
  if (!doc) return;
  var E = root.FabrailEngine, P = root.FabrailPlot, UI = root.FabrailUI;
  if (!F || !P || !UI) return;
  var fmt = UI.fmt;

  function $(id) { return doc.getElementById(id); }
  function setText(id, s) { var el = $(id); if (el) el.textContent = s; }
  function wantForce() {
    try { return /(?:^|[?&])engine=wasm(?:&|$)/.test(root.location.search || ""); } catch (e) { return false; }
  }
  function nowS() { return (root.performance && root.performance.now ? root.performance.now() : Date.now()) / 1000; }

  var S = {
    name: "drawer", c: F.CLASSES.drawer, hp: HAPTIC.drawer, cp: COUPLING.drawer,
    engineMode: "loading", engineReason: "",
    h: null, st: null, prev: null, js: null, tickAcc: 0,
    grab: null, ptrX: null, ptrXPrev: null, ptrV: 0, ptrT: 0, key: 0,
    force: 0, est: new MassEstimator(0.5), hist: { t: [], f: [], fh: [], v: [] },
    events: [], lastInput: 0, idle: false, flash: 0, lastUi: 0
  };

  var stage = $("tc-stage"), canvas = $("tc-canvas"), chip = $("tc-chip");
  if (!stage || !canvas) return;

  var col = {}, geo = null, cv = null;
  function colours() {
    var names = ["deep", "panel", "ink", "ink2", "muted", "line", "grid", "axis", "accent", "good", "ref",
      "drawer", "door", "wall", "s4"];
    for (var i = 0; i < names.length; i++) col[names[i]] = P.color(names[i]);
    col.flux = P.color("--flux"); col.steel = P.color("--steel"); col.sans = P.theme().sans;
  }

  /* ---- engine ----------------------------------------------------------------------- */

  function destroyEngine() {
    if (S.h) { try { S.h.destroy(); } catch (e) { /* gone */ } }
    S.h = null;
  }

  function createEngine() {
    destroyEngine();
    S.st = null; S.prev = null; S.js = null; S.tickAcc = 0; S.stopped = "";
    S.est.reset(); S.lastMapp = null; S.hist = { t: [], f: [], fh: [], v: [] }; S.events = [];
    S.lastInput = nowS(); S.idle = false;
    if (S.engineMode === "wasm" && E) {
      var h = E.open(S.name, "carriage6");
      if (h) {
        S.h = h;
        S.st = h.state();
        if (S.name === "wall") { h.setInputs({ command: "lift" }); logEvent(0, "lift requested: the WALL rests landed and locked", ""); }
        renderLog();
        return;
      }
      S.engineMode = "formulas";
      S.engineReason = "the loaded engine has no Carriage6 handle (" + (E.lastError() || "ABI v2 needed") + ")";
    }
    S.js = new JsAdmittance(S.name);
    renderLog();
  }

  function state() {
    if (S.h && S.st) return S.st;
    var j = S.js;
    if (!j) return null;
    return { t_s: j.t, x_m: j.x, v_mps: j.v, a_mps2: j.a, f_hand_n: S.force, f_hand_hat_n: NaN,
      m_hat_kg: j.m, e_kin_gov_j: 0.5 * Math.max(S.c.mass_rated_kg, 1.05 * j.m) * j.v * j.v,
      lev_state: 5, motion_mode: 3, safety_demand: 0, fault_bits_lo: 0 };
  }

  function logEvent(t, text, kind) {
    S.events.unshift({ t: t, text: text, kind: kind || "" });
    if (S.events.length > 7) S.events.length = 7;
  }

  function renderLog() {
    var ol = $("tc-log");
    if (!ol) return;
    while (ol.firstChild) ol.removeChild(ol.firstChild);
    if (!S.events.length) {
      var li0 = doc.createElement("li");
      var a0 = doc.createElement("span"); a0.className = "t"; a0.textContent = "–";
      var b0 = doc.createElement("span"); b0.textContent = S.h ? "no events yet: touch the panel" : "fallback: no supervisor events";
      li0.appendChild(a0); li0.appendChild(b0); ol.appendChild(li0);
      return;
    }
    for (var i = 0; i < S.events.length; i++) {
      var li = doc.createElement("li");
      if (S.events[i].kind) li.setAttribute("data-kind", S.events[i].kind);
      var a = doc.createElement("span"); a.className = "t"; a.textContent = fmt(S.events[i].t, 4) + " s";
      var b = doc.createElement("span"); b.textContent = S.events[i].text;
      li.appendChild(a); li.appendChild(b); ol.appendChild(li);
    }
  }

  /* ---- input → force ------------------------------------------------------------------ */

  function handTarget(frac) {
    if (!S.grab || S.ptrX === null) return null;
    var xp = S.ptrXPrev === null ? S.ptrX : S.ptrXPrev + (S.ptrX - S.ptrXPrev) * frac;
    return xp - S.grab.offset;
  }

  function currentForce(frac, st) {
    if (S.grab && S.ptrX !== null) {
      var xT = handTarget(frac);
      return handForce(S.cp, xT, S.ptrV, st.x_m, st.v_mps);
    }
    if (S.key) return S.key;
    return 0;
  }

  function send(cmd) {
    S.lastInput = nowS(); S.idle = false;
    if (S.h) { S.h.setInputs({ command: cmd }); logEvent(S.st ? S.st.t_s : 0, "command: " + cmd, ""); renderLog(); }
    else if (S.js) {
      if (cmd === "open") S.js.v = Math.min(S.js.vCap, 0.8 * S.js.vCap);
      if (cmd === "close") S.js.v = -0.8 * S.js.vCap;
      if (cmd === "stop") S.js.v = 0;
    }
    loop.start(true);
  }

  /* ---- stepping -------------------------------------------------------------------- */

  function advance(dt) {
    var st = state();
    if (!st) return;
    var active = S.grab || S.key;
    if (S.h) {
      if (S.stopped) return;
      S.tickAcc += dt * TICK_HZ;
      var n = Math.floor(S.tickAcc), total = n;
      S.tickAcc -= n;
      while (n > 0) {
        var k = Math.min(SLICE_TICKS, n);
        var frac = 1 - (n - k) / Math.max(1, total);
        var f = currentForce(frac, S.st);
        S.force = f;
        S.h.setInputs({ handN: f });
        var got = S.h.step(k);
        n -= k;
        if (!got) break;
        S.st = S.h.state();
        var s = S.st;
        S.est.update(f, s.a_mps2, s.v_mps, S.hp.bv, k / TICK_HZ, s.motion_mode === 3 && Math.abs(f) > 1);
        var d = describe(S.name, S.prev, s);
        for (var q = 0; q < d.length; q++) { logEvent(s.t_s, d[q][0], d[q][1]); if (d[q][1] === "alert") S.flash = 0.6; }
        if (d.length) renderLog();
        S.prev = s;
        pushHist(s.t_s, f, s.f_hand_hat_n, s.v_mps);
      }
      var status = S.h.status();
      if (status && status !== "running") S.stopped = status + (E.lastError() ? ": " + E.lastError() : "");
      if (active || Math.abs(S.st.v_mps) > 1e-3 || [2, 5, 10, 11, 4].indexOf(S.st.motion_mode) >= 0 || [4, 6, 7, 8, 9].indexOf(S.st.lev_state) >= 0) S.lastInput = Math.max(S.lastInput, nowS() - 1.5);
    } else if (S.js) {
      var fj = currentForce(1, st);
      S.force = fj;
      S.js.step(dt, fj);
      var sj = state();
      S.est.update(fj, sj.a_mps2, sj.v_mps, S.hp.bv, dt, Math.abs(fj) > 1);
      pushHist(sj.t_s, fj, NaN, sj.v_mps);
      if (active || Math.abs(sj.v_mps) > 1e-3) S.lastInput = nowS();
    }
    S.ptrXPrev = S.ptrX;
    if (S.grab && nowS() - S.ptrT > 0.06) S.ptrV *= 0.6;
    if (S.grab && S.h && S.st && shouldRelease(S.grab.startMode, S.st.motion_mode, nowS() - S.grab.stillSince)) {
      logEvent(S.st.t_s, "your hand lets go: the controller took over", "");
      renderLog();
      S.grab = null; S.ptrX = null; S.ptrXPrev = null; S.ptrV = 0; S.lastPx = undefined;
      canvas.setAttribute("data-drag", "");
    }
  }

  function pushHist(t, f, fh, v) {
    var H = S.hist;
    if (H.t.length && t - H.t[H.t.length - 1] < 0.009) return;
    H.t.push(t); H.f.push(f); H.fh.push(fh); H.v.push(v);
    while (H.t.length && H.t[0] < t - 8) { H.t.shift(); H.f.shift(); H.fh.shift(); H.v.shift(); }
  }

  /* ---- geometry and drawing ---------------------------------------------------------- */

  function layout() {
    var w = Math.max(280, Math.round(stage.clientWidth || 640));
    var narrow = w < 640;
    var h = narrow ? Math.round(Math.min(Math.max(w * 0.85, 280), 380)) : Math.round(Math.min(Math.max(w * 0.33, 310), 380));
    cv = P.setupCanvas(canvas, w, h);
    var W = S.hp.world, pad = 18;
    var X = function (xm) { return pad + (w - 2 * pad) * (xm - W[0]) / (W[1] - W[0]); };
    var Xinv = function (px) { return W[0] + (px - pad) * (W[1] - W[0]) / (w - 2 * pad); };
    var rulerY = h - (narrow ? 34 : 28);
    geo = { w: w, h: h, narrow: narrow, X: X, Xinv: Xinv, railY: narrow ? 40 : 100, floorY: rulerY - 16, rulerY: rulerY };
  }

  function objectRect(x) {
    var g = geo, X = g.X, hp = S.hp;
    if (S.name === "drawer") return { x0: X(x - 0.58), x1: X(x) + 8, y0: g.railY + 12, y1: g.railY + 12 + (g.floorY - g.railY) * 0.5 };
    return { x0: X(x - hp.len), x1: X(x), y0: g.railY + 14, y1: g.floorY - 6 };
  }

  function handleX(x) {
    if (S.name === "drawer") return x + 0.012;
    if (S.name === "door") return x - 0.06;
    return x - 0.12;
  }

  function line(ctx, x0, y0, x1, y1) { ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke(); }
  function text(ctx, s, x, y, o) {
    o = o || {};
    ctx.font = (o.weight || "400") + " " + (o.size || 11) + "px " + col.sans;
    ctx.fillStyle = o.color || col.muted;
    ctx.textAlign = o.align || "left"; ctx.textBaseline = o.base || "alphabetic";
    ctx.fillText(s, x, y);
  }
  function arrow(ctx, x, y, len, colour, dash) {
    if (Math.abs(len) < 2) return;
    ctx.strokeStyle = colour; ctx.fillStyle = colour; ctx.lineWidth = 2.2; ctx.setLineDash(dash || []);
    line(ctx, x, y, x + len, y); ctx.setLineDash([]);
    var s = Math.sign(len);
    ctx.beginPath(); ctx.moveTo(x + len, y); ctx.lineTo(x + len - 7 * s, y - 4.5); ctx.lineTo(x + len - 7 * s, y + 4.5); ctx.closePath(); ctx.fill();
  }

  function draw() {
    if (!geo) layout();
    var ctx = cv.ctx, g = geo, X = g.X, c = S.c, hp = S.hp, st = state();
    ctx.clearRect(0, 0, g.w, g.h);
    ctx.fillStyle = col.deep; ctx.fillRect(0, 0, g.w, g.h);
    var x = st ? st.x_m : 0, colour = col[S.name];
    ctx.strokeStyle = col.axis; ctx.lineWidth = 1.5; line(ctx, 0, g.floorY + 0.5, g.w, g.floorY + 0.5);

    /* fixed structure: cabinet or wall and the rail */
    if (S.name === "drawer") {
      var cab0 = X(-0.66), cab1 = X(0);
      ctx.fillStyle = P.alpha(col.line, 0.35); ctx.strokeStyle = col.ink2; ctx.lineWidth = 1.5;
      ctx.fillRect(cab0, g.railY - 12, cab1 - cab0, g.floorY - g.railY + 12); ctx.strokeRect(cab0, g.railY - 12, cab1 - cab0, g.floorY - g.railY + 12);
      ctx.fillStyle = col.steel; ctx.fillRect(cab0 + 6, g.railY - 2, cab1 - cab0 - 6, 5);
      text(ctx, "cabinet", cab0 + 8, g.floorY - 10, { size: 10.5 });
    } else {
      var open0 = X(-hp.len - 0.02), open1 = X(0.02);
      ctx.fillStyle = P.alpha(col.line, 0.35);
      ctx.fillRect(0, g.railY - 24, open0, g.floorY - g.railY + 24);
      ctx.fillRect(open1, g.railY - 24, g.w - open1, g.floorY - g.railY + 24);
      ctx.fillRect(0, g.railY - 24, g.w, 14);
      ctx.strokeStyle = col.ink2; ctx.lineWidth = 1; ctx.strokeRect(open0, g.railY - 10, open1 - open0, g.floorY - g.railY + 10);
      text(ctx, S.name === "door" ? "doorway" : "opening", (open0 + open1) / 2, g.floorY - 10, { align: "center", size: 10.5 });
      ctx.fillStyle = col.steel; ctx.fillRect(X(hp.world[0] + 0.05), g.railY - 2, X(hp.world[1] - 0.05) - X(hp.world[0] + 0.05), 5);
    }
    /* rail features: detent and soft-close capture */
    ctx.fillStyle = P.alpha(col.accent, 0.16);
    ctx.fillRect(X(0), g.rulerY - 9, X(hp.scCap) - X(0), 10);
    ctx.strokeStyle = col.s4; ctx.lineWidth = 2; line(ctx, X(hp.detX), g.rulerY - 12, X(hp.detX), g.rulerY + 2);
    if (!g.narrow) {
      text(ctx, "soft-close capture", X(0), g.rulerY - 13, { size: 9.5 });
      text(ctx, "detent " + fmt(hp.detF, 2) + " N", X(hp.detX), g.rulerY - 15, { size: 9.5, align: "center", color: col.ink2 });
    }
    /* ruler */
    ctx.strokeStyle = col.ink2; ctx.lineWidth = 1; line(ctx, X(0), g.rulerY, X(c.travel_m), g.rulerY);
    var step = S.name === "drawer" ? 0.05 : S.name === "door" ? 0.1 : 1;
    for (var t = 0; t <= c.travel_m + 1e-9; t += step) {
      line(ctx, X(t), g.rulerY, X(t), g.rulerY - 4);
      if (Math.abs(t / (2 * step) - Math.round(t / (2 * step))) < 1e-6) text(ctx, S.name === "wall" ? fmt(t, 2) + " m" : fmt(t * 1e3, 4), X(t), g.rulerY + 13, { align: "center", size: 10 });
    }
    if (S.name !== "wall") text(ctx, "mm", X(c.travel_m) + 16, g.rulerY + 13, { size: 10 });

    /* the moving object */
    var r = objectRect(x);
    ctx.fillStyle = P.alpha(col.flux, 0.25); ctx.fillRect(r.x0 + 6, g.railY + 3, r.x1 - r.x0 - 12, 3);
    ctx.fillStyle = P.alpha(colour, S.name === "door" ? 0.12 : 0.2);
    ctx.strokeStyle = S.flash > 0 ? col.ref : colour; ctx.lineWidth = S.flash > 0 ? 3 : 1.7;
    ctx.fillRect(r.x0, r.y0, r.x1 - r.x0, r.y1 - r.y0); ctx.strokeRect(r.x0, r.y0, r.x1 - r.x0, r.y1 - r.y0);
    ctx.fillStyle = P.alpha(colour, 0.5); ctx.fillRect(r.x0 + 4, g.railY + 5, r.x1 - r.x0 - 8, 8);
    var hx = X(handleX(x)), hy = (r.y0 + r.y1) / 2;
    ctx.fillStyle = col.ink; P.roundRect(ctx, hx - 3, hy - 16, 6, 32, 3); ctx.fill();

    /* hand spring and force arrows */
    var fscale = 60 / S.cp.fmax;
    if (S.grab && S.ptrX !== null) {
      var px = X(S.ptrX - S.grab.offset + (handleX(x) - x));
      var f = S.force, sat = Math.abs(f) / S.cp.fmax;
      ctx.strokeStyle = sat > 0.98 ? col.ref : col.accent; ctx.lineWidth = 1.6;
      ctx.beginPath(); ctx.moveTo(hx, hy);
      var segs = 10;
      for (var z = 1; z < segs; z++) ctx.lineTo(hx + (px - hx) * (z / segs), hy + (z % 2 ? -5 : 5));
      ctx.lineTo(px, hy); ctx.stroke();
      ctx.fillStyle = P.alpha(col.accent, 0.3); ctx.strokeStyle = col.accent;
      ctx.beginPath(); ctx.arc(px, hy, 11, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    }
    if (st) {
      arrow(ctx, hx, r.y0 - 14, S.force * fscale, col.accent);
      if (isFinite(st.f_hand_hat_n)) arrow(ctx, hx, r.y0 - 26, st.f_hand_hat_n * fscale, col.muted, [4, 3]);
      if (!g.narrow) {
        text(ctx, "your force " + fmt(S.force, 3) + " N  →", g.w - 16, g.railY - 50, { size: 11.5, color: col.accent, weight: "600", align: "right" });
        if (isFinite(st.f_hand_hat_n)) text(ctx, "observer F̂ " + fmt(st.f_hand_hat_n, 3) + " N  ⇢", g.w - 16, g.railY - 34, { size: 11, color: col.muted, align: "right" });
      }
      ctx.fillStyle = colour; ctx.beginPath(); ctx.arc(X(Math.min(Math.max(x, -0.1), c.travel_m + 0.1)), g.rulerY, 3.5, 0, 2 * Math.PI); ctx.fill();
    }
    if (S.idle) text(ctx, "idle: the engine is paused until you touch it", g.w - 16, g.railY - 16, { size: 10.5, align: "right" });
  }

  /* ---- readouts ---------------------------------------------------------------------- */

  var lastChip = "";
  function setChip(s) { if (s !== lastChip) { chip.textContent = s; lastChip = s; } }

  function updateUi() {
    var st = state(), c = S.c;
    if (!st) return;
    setText("tr-f", fmt(S.force, 3) + " N");
    setText("tr-f-note", S.grab ? "hand spring " + fmt(S.cp.k / 1e3, 2) + " N/mm, cap " + S.cp.fmax + " N" : S.key ? "keyboard push" : "released");
    setText("tr-fh", isFinite(st.f_hand_hat_n) ? fmt(st.f_hand_hat_n, 3) + " N" : "–");
    setText("tr-fh-note", S.h ? "momentum observer, intent F_th " + S.hp.fth + " N" : "fallback: no observer");
    var mv = st.m_hat_kg / S.hp.ratio, mApp = S.est.value();
    if (mApp !== null) S.lastMapp = mApp;
    var shown = mApp !== null ? mApp : S.lastMapp;
    setText("tr-m", shown !== undefined && shown !== null ? fmt(shown, 3) + " kg" : "–");
    var resting = S.h && (st.lev_state === 2 || st.lev_state === 3);
    setText("tr-m-note", resting ? "resting: m̂ is held at m_rated until it levitates; the admittance renders m̂/" + S.hp.ratio :
      (mApp === null && shown !== undefined && shown !== null ? "last push · " : "") + "target M_v = m̂/" + S.hp.ratio + " = " + fmt(mv, 3) + " kg (m̂ " + fmt(st.m_hat_kg, 3) + " kg)");
    var vCap = Math.min(c.v_open_max_mps, Math.sqrt(2 * E_MAX_J / Math.max(c.mass_rated_kg, 1.05 * st.m_hat_kg)), P_MAX / Math.max(c.mass_rated_kg, 1.05 * st.m_hat_kg));
    setText("tr-v", Math.abs(st.v_mps) < 1e-6 ? "0 mm/s" : Math.abs(st.v_mps) < 0.01 ? fmt(st.v_mps * 1e3, 3) + " mm/s" : fmt(st.v_mps, 3) + " m/s");
    setText("tr-v-note", "cap " + fmt(vCap, 3) + " m/s · ½m_gov·v² " + (st.e_kin_gov_j < 1e-4 ? "≈ 0" : fmt(st.e_kin_gov_j, 3)) + " J");
    setText("tr-x", fmt(st.x_m * 1e3, 4) + " mm");
    setText("tr-x-note", "travel 0 to " + fmt(c.travel_m * 1e3, 4) + " mm");
    var box = $("tr-state-box");
    setText("tr-state", S.h ? (MODE_NAMES[st.motion_mode] || "–") : "JS admittance (fallback)");
    setText("tr-state-note", S.h ? (LEV_NAMES[st.lev_state] || "–") + (st.safety_demand ? " · monitor: " + (DEMAND_NAMES[st.safety_demand] || st.safety_demand) : "") : "no supervisor");
    if (box) box.setAttribute("data-alert", S.h && (st.safety_demand > 0 || st.lev_state >= 10) ? "on" : "off");
    setText("tc-engine", "engine: " + (S.h ? "live: Rust controller + Carriage6 plant in WebAssembly" : S.engineMode === "loading" ? "loading" : "JS admittance fallback" + (S.engineReason ? " (" + S.engineReason + ")" : "")));
    var lift = $("tc-lift"), land = $("tc-land");
    if (lift) lift.hidden = S.name !== "wall" || !S.h;
    if (land) land.hidden = S.name !== "wall" || !S.h;
    updateChip(st);
  }

  function updateChip(st) {
    if (S.stopped) { setChip("The engine stopped (" + S.stopped + "). Press Restart."); return; }
    if (S.engineMode === "loading") { setChip("Loading the browser engine…"); return; }
    if (!S.h) { setChip("Fallback (" + (S.engineReason || "engine unavailable") + "): a JavaScript copy of the admittance law, without the observer, intent or supervisor."); return; }
    var L = st.lev_state, M = st.motion_mode;
    if (S.grab) { setChip("You are " + (Math.abs(S.force) < 1 ? "holding" : S.force > 0 ? "pulling it open" : "pushing it closed") + " with " + fmt(Math.abs(S.force), 3) + " N. Let go to see what the controller does."); return; }
    if (L === 2) { setChip("Seated. Push it in firmly and let go (push-to-open), or press Open."); return; }
    if (L === 3) { setChip("Landed" + (S.name === "wall" ? " and locked. Press Lift." : ".")); return; }
    if (L === 4) { setChip("Lifting…"); return; }
    if (L >= 10) { setChip("Fault reaction: the carriage lands safely. Press Restart to clear it."); return; }
    if (M === 2) { setChip("Finishing the move on an S-curve under the governor."); return; }
    if (M === 10 || M === 11) { setChip("Log-damped terminus: the last centimetres by e^(−t/τ)."); return; }
    if (M === 5 || (M === 0 && L === 7)) { setChip("Obstruction: the controller yields."); return; }
    setChip("Levitated and listening. Drag it, or give it a push and let go.");
  }

  /* ---- charts -------------------------------------------------------------------------- */

  var fChart = null, vChart = null;
  function initCharts() {
    var fe = $("force-chart"), ve = $("speed-chart");
    if (fe) fChart = P.chart(fe, { kind: "line", title: "Hand force and observer estimate", desc: "Your applied hand force and the momentum observer's estimate over the last 8 seconds.", x: { label: "time", unit: "s" }, y: { label: "force", unit: "N" }, series: [], height: 220 });
    if (ve) vChart = P.chart(ve, { kind: "line", title: "Speed", desc: "Carriage speed over the last 8 seconds with the governor cap.", x: { label: "time", unit: "s" }, y: { label: "speed", unit: "m/s" }, series: [], height: 220 });
  }
  function refreshCharts() {
    var H = S.hist;
    if (!fChart || !vChart || !H.t.length) return;
    var t1 = H.t[H.t.length - 1], t0 = Math.max(0, t1 - 8), fd = [], hd = [], vd = [];
    for (var i = 0; i < H.t.length; i++) {
      fd.push([H.t[i], H.f[i]]);
      if (isFinite(H.fh[i])) hd.push([H.t[i], H.fh[i]]);
      vd.push([H.t[i], H.v[i]]);
    }
    var fmx = Math.max(S.cp.fmax, S.hp.fth * 2) * 1.1, c = S.c;
    var series = [{ key: "f", label: "your hand force", color: S.name, data: fd }];
    if (hd.length) series.push({ key: "fh", label: "observer estimate F̂", color: "muted", dash: [5, 4], width: 1.5, data: hd });
    fChart.update({
      x: { label: "time", unit: "s", min: t0, max: Math.max(t0 + 1, t1) }, y: { label: "force", unit: "N", min: -fmx, max: fmx },
      series: series,
      refs: [{ axis: "y", value: S.hp.fth, label: "F_th", color: "muted", dash: [2, 4] }, { axis: "y", value: -S.hp.fth, color: "muted", dash: [2, 4] }]
    });
    var cap = c.v_open_max_mps;
    vChart.update({
      x: { label: "time", unit: "s", min: t0, max: Math.max(t0 + 1, t1) }, y: { label: "speed", unit: "m/s", min: -cap * 1.2, max: cap * 1.2 },
      series: [{ key: "v", label: "speed", color: S.name, data: vd }],
      refs: [{ axis: "y", value: cap, label: "cap", color: "ref" }, { axis: "y", value: -cap, color: "ref" }, { axis: "y", value: 0, color: "muted", dash: [2, 3] }]
    });
  }

  /* ---- loop ---------------------------------------------------------------------------- */

  var loop = P.loop(function (dt) {
    if (S.flash > 0) S.flash -= dt;
    var idleNow = !S.grab && !S.key && nowS() - S.lastInput > 4;
    if (idleNow && !S.idle) { S.idle = true; draw(); updateUi(); loop.stop(); return; }
    advance(dt);
    draw();
    var n = Date.now();
    if (n - S.lastUi > 140) { S.lastUi = n; updateUi(); refreshCharts(); }
  }, { el: stage, still: function () { draw(); updateUi(); refreshCharts(); }, maxDt: 0.05 });

  function wake() {
    S.lastInput = nowS(); S.idle = false;
    loop.start(true);
  }

  /* ---- pointer and keyboard ------------------------------------------------------------ */

  function canvasPoint(ev) {
    var r = canvas.getBoundingClientRect();
    return { x: ev.clientX - r.left, y: ev.clientY - r.top };
  }

  canvas.addEventListener("pointerdown", function (ev) {
    var st = state();
    if (!geo || !st) return;
    var p = canvasPoint(ev), r = objectRect(st.x_m);
    if (p.x < r.x0 - 14 || p.x > r.x1 + 14 || p.y < r.y0 - 30 || p.y > r.y1 + 10) return;
    var wx = geo.Xinv(p.x);
    S.grab = { offset: wx - st.x_m, startMode: st.motion_mode, stillSince: nowS() };
    S.lastPx = p.x;
    S.ptrX = wx; S.ptrXPrev = wx; S.ptrV = 0; S.ptrT = nowS();
    try { canvas.setPointerCapture(ev.pointerId); } catch (e) { /* unsupported */ }
    canvas.setAttribute("data-drag", "on");
    S.est.reset();
    ev.preventDefault();
    wake();
  });
  canvas.addEventListener("pointermove", function (ev) {
    if (!S.grab || !geo) return;
    var p = canvasPoint(ev), wx = geo.Xinv(p.x), t = nowS(), dtp = Math.max(1e-3, t - S.ptrT);
    var vInst = (wx - S.ptrX) / dtp;
    S.ptrV += (vInst - S.ptrV) * Math.min(1, dtp / 0.04);
    if (Math.abs(p.x - (S.lastPx === undefined ? p.x : S.lastPx)) >= 1) S.grab.stillSince = t;
    S.lastPx = p.x;
    S.ptrX = wx; S.ptrT = t;
    S.lastInput = t;
    ev.preventDefault();
  });
  function release() {
    if (!S.grab) return;
    S.grab = null; S.ptrX = null; S.ptrXPrev = null; S.ptrV = 0; S.lastPx = undefined;
    canvas.setAttribute("data-drag", "");
    wake();
  }
  canvas.addEventListener("pointerup", release);
  canvas.addEventListener("pointercancel", release);
  canvas.addEventListener("lostpointercapture", release);

  canvas.addEventListener("keydown", function (ev) {
    var k = ev.key;
    if (k === "ArrowRight" || k === "ArrowLeft") {
      S.key = (k === "ArrowRight" ? 1 : -1) * S.cp.fkey * (ev.shiftKey ? 2 : 1);
      ev.preventDefault(); wake();
    } else if (k === "o" || k === "O") { send("open"); ev.preventDefault(); }
    else if (k === "c" || k === "C") { send("close"); ev.preventDefault(); }
    else if (k === "s" || k === "S") { send("stop"); ev.preventDefault(); }
  });
  canvas.addEventListener("keyup", function (ev) {
    if (ev.key === "ArrowRight" || ev.key === "ArrowLeft") { S.key = 0; wake(); }
  });
  canvas.addEventListener("blur", function () { S.key = 0; });

  function bind(id, fn) { var el = $(id); if (el) el.addEventListener("click", fn); }
  bind("tc-open", function () { send("open"); });
  bind("tc-close", function () { send("close"); });
  bind("tc-stop", function () { send("stop"); });
  bind("tc-lift", function () { send("lift"); });
  bind("tc-land", function () { send("land"); });
  bind("tc-restart", function () { createEngine(); layout(); wake(); });

  function setClass(name) {
    S.name = name; S.c = F.CLASSES[name]; S.hp = HAPTIC[name]; S.cp = COUPLING[name];
    S.grab = null; S.key = 0;
    layout();
    createEngine();
    wake();
  }
  var g1 = $("tc-class"); if (g1) UI.segmented(g1, function (v) { setClass(v); });

  P.onTheme(function () { colours(); draw(); });
  if (root.ResizeObserver) {
    var lastW = 0;
    new root.ResizeObserver(function () {
      var w = stage.clientWidth;
      if (w && Math.abs(w - lastW) >= 1) { lastW = w; layout(); draw(); }
    }).observe(stage);
  } else root.addEventListener("resize", function () { layout(); draw(); });

  /* ---- passivity lab ------------------------------------------------------------------- */

  function passivityLab() {
    var kIn = $("pl-k"), pcIn = $("pl-pc"), runBtn = $("pl-run"), group = $("pl-class");
    var lc = $("pl-canvas"), lstage = $("pl-stage"), lchip = $("pl-chip");
    var xEl = $("pl-x-chart"), eEl = $("pl-e-chart");
    if (!kIn || !lc) return;
    var pick = "drawer", res = null, xChart = null, eChart = null, playT = 0, lcv = null;
    function compute() {
      var c = F.CLASSES[pick], hp = HAPTIC[pick], b = hp.bDist, T = hp.T;
      var kMax = (2 * b) / (T + 2 * T), K = parseFloat(kIn.value) * kMax;
      var base = { m: c.m_star_kg, b: b, T: T, K: K, alphaMax: 5 * b, v0: 0.05, dur: 20 };
      res = { on: passivitySim(Object.assign({}, base, { pc: true })), off: passivitySim(Object.assign({}, base, { pc: false })), K: K, kMax: kMax, b: b, T: T, m: c.m_star_kg };
      setText("pl-k-out", fmt(parseFloat(kIn.value), 2) + " × K_max = " + fmt(K / 1e3, 3) + " kN/m");
      setText("pl-kmax", fmt(kMax / 1e3, 4) + " kN/m");
      setText("pl-kmax-note", "b = " + b + " N·s/m, T = T_d = " + fmt(T * 1e3, 2) + " ms, α_max = 5·b");
      var net = b - K * (T / 2 + T);
      setText("pl-net", fmt(net, 3) + " N·s/m");
      var p0 = 0.05 / Math.sqrt(K / c.m_star_kg);
      setText("pl-off", fmt(res.off.peakLast2s * 1e3, 3) + " mm");
      setText("pl-off-note", "×" + fmt(res.off.peakLast2s / p0, 3) + " the first swing" + (res.off.peakLast2s > p0 * 1.05 ? ": grows" : ": decays"));
      setText("pl-on", fmt(res.on.peakLast2s * 1e3, 3) + " mm");
      setText("pl-on-note", "min E_obs − W " + fmt(res.on.minEW * 1e3, 3) + " mJ · peak α " + fmt(res.on.alphaPeak, 3) + " N·s/m" +
        (net + 5 * b < 0 ? " · beyond α_max: the controller runs out of authority" : ""));
      var box = $("pl-off-box"); if (box) box.setAttribute("data-alert", res.off.peakLast2s > p0 * 1.05 ? "on" : "off");
      charts(true);
      playT = UI.reducedMotion() ? 20 : 0;
      if (!UI.reducedMotion()) anim.start(); else drawLab();
    }
    function series(r, key) {
      var d = [], stride = 2;
      for (var i = 0; i < r.t.length; i += stride) d.push([r.t[i], key === "x" ? r.x[i] * 1e3 : r.egen[i] * 1e3]);
      return d;
    }
    function charts(full) {
      if (!res) return;
      var pc = pcIn.checked;
      var sx = [
        { key: "sel", label: pc ? "controller on" : "controller off", color: pick, data: series(pc ? res.on : res.off, "x") },
        { key: "oth", label: pc ? "controller off (for comparison)" : "controller on (for comparison)", color: "muted", dash: [4, 4], width: 1.2, data: series(pc ? res.off : res.on, "x") }
      ];
      var se = [
        { key: "sel", label: pc ? "controller on" : "controller off", color: pick, data: series(pc ? res.on : res.off, "e") },
        { key: "oth", label: pc ? "controller off (for comparison)" : "controller on (for comparison)", color: "muted", dash: [4, 4], width: 1.2, data: series(pc ? res.off : res.on, "e") }
      ];
      var reveal = full || UI.reducedMotion() ? undefined : playT;
      var xs = { kind: "line", title: "Sandbox position", desc: "Position of the sandbox mass against time after a flick, with the passivity controller on and off.", x: { label: "time", unit: "s", min: 0, max: 20 }, y: { label: "position", unit: "mm" }, series: sx, reveal: reveal, height: 220, table: !!full, tableRows: 20 };
      var es = { kind: "line", title: "Generated energy", desc: "Energy created by the sampled loop, W minus the observed energy, against time.", x: { label: "time", unit: "s", min: 0, max: 20 }, y: { label: "W − E_obs", unit: "mJ" }, series: se, reveal: reveal, refs: [{ axis: "y", value: 1, label: "1 mJ (FAB-HAP-009)", color: "ref", dash: [3, 4] }], height: 220, table: !!full, tableRows: 20 };
      if (!xChart && xEl) xChart = P.chart(xEl, xs); else if (xChart) xChart.update(xs);
      if (!eChart && eEl) eChart = P.chart(eEl, es); else if (eChart) eChart.update(es);
    }
    function sample(r, t) {
      var i = Math.min(r.t.length - 1, Math.max(0, Math.round(t / 0.005)));
      return r.x[i] || 0;
    }
    function drawLab() {
      if (!res) return;
      var w = Math.max(240, Math.round(lstage.clientWidth || 300)), h = 150;
      lcv = P.setupCanvas(lc, w, h);
      var ctx = lcv.ctx, pc = pcIn.checked, r = pc ? res.on : res.off;
      ctx.fillStyle = col.deep; ctx.fillRect(0, 0, w, h);
      var x = sample(r, playT), amp = Math.max(Math.abs(res.off.peakLast2s), 0.004);
      var scale = (w * 0.3) / amp, cx = w * 0.55, y = h * 0.62;
      ctx.strokeStyle = col.axis; ctx.lineWidth = 1.5; line(ctx, 10, y + 20, w - 10, y + 20);
      ctx.fillStyle = col.steel; ctx.fillRect(14, y - 26, 8, 46);
      var mx = cx + clamp(x * scale, -w * 0.4, w * 0.4);
      ctx.strokeStyle = col.accent; ctx.lineWidth = 1.5; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.moveTo(22, y);
      for (var z = 1; z < 14; z++) ctx.lineTo(22 + (mx - 22 - 18) * (z / 14), y + (z % 2 ? -7 : 7));
      ctx.lineTo(mx - 18, y); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = P.alpha(col[pick], 0.35); ctx.strokeStyle = col[pick]; ctx.lineWidth = 1.5;
      ctx.fillRect(mx - 18, y - 18, 36, 36); ctx.strokeRect(mx - 18, y - 18, 36, 36);
      ctx.strokeStyle = col.muted; ctx.lineWidth = 1; line(ctx, cx, y + 22, cx, y + 28);
      text(ctx, "t = " + fmt(playT, 3) + " s · x = " + fmt(x * 1e3, 3) + " mm (drawn at " + fmt(scale / 1000, 2) + " px/mm)", 12, h - 8, { size: 10.5, color: col.ink2 });
      text(ctx, "virtual spring (sampled, one-period delay)", 26, y - 30, { size: 10 });
      lchip.textContent = (pc ? "Controller on" : "Controller off") + " · K = " + fmt(parseFloat(kIn.value), 2) + "·K_max · sandbox, not the engine";
    }
    var anim = P.loop(function (dt) {
      playT = Math.min(20, playT + dt);
      drawLab();
      charts(false);
      if (playT >= 20) { anim.stop(); charts(true); }
    }, { el: lstage, still: function () { playT = 20; drawLab(); charts(true); }, maxDt: 0.05 });
    kIn.addEventListener("input", compute);
    pcIn.addEventListener("change", function () { charts(!anim.running()); drawLab(); if (!UI.reducedMotion() && !anim.running()) { playT = 0; anim.start(); } });
    if (runBtn) runBtn.addEventListener("click", function () { playT = 0; if (UI.reducedMotion()) { playT = 20; drawLab(); charts(true); } else anim.start(); });
    if (group) UI.segmented(group, function (v) { pick = v; compute(); });
    P.onTheme(function () { drawLab(); });
    compute();
  }

  /* ---- start ----------------------------------------------------------------------------- */

  colours();
  layout();
  initCharts();
  passivityLab();

  function labelSims() {
    ["tc-sim", "force-sim", "speed-sim"].forEach(function (id) {
      var el = $(id);
      if (!el) return;
      el.textContent = ""; el.classList.remove("sim-label"); el.removeAttribute("title");
      el.setAttribute("data-simulated", S.h ? "" : "illustrative");
    });
    UI.renderSimLabels(doc);
    if (!S.h) { var t = $("tc-sim"); if (t) t.textContent = "JS fallback"; }
  }

  function onEngine(r) {
    S.engineMode = r && r.mode === "wasm" ? "wasm" : "formulas";
    S.engineReason = r && r.reason ? r.reason : "";
    createEngine();
    labelSims();
    draw(); updateUi();
    wake();
  }

  if (E && typeof E.load === "function") {
    var force = wantForce();
    E.load(force ? { force: true } : undefined).then(function (r) {
      if (force) E.renderSelftestBadges();
      onEngine(r);
    }, function () { onEngine({ mode: "formulas", reason: "loader error" }); });
  } else onEngine({ mode: "formulas", reason: "engine.js missing" });
  draw();
})(typeof window !== "undefined" ? window : globalThis);
