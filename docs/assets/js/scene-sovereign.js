/* sovereign.html scenes (owner W4): the Sovereign/Exotic tier (plan §4, §3b; EXOTIC.md).
 *
 * Everything here is either a closed form from EXOTIC.md evaluated in the browser, or a
 * small illustrative browser model labelled as such:
 *   SENTINEL   2-DOF host + damper; the RMS reduction under white-noise force is exact
 *              (Lyapunov equation), the animated gusts are illustrative (EXOTIC §X6, §X7).
 *   HAVEN-R    SDOF isolation, passive against skyhook; transmissibility closed forms from
 *              FORMULAS (PHYSICS §21), S_D and a_peak from the ASCE 7-16 Ch. 17 form (EXOTIC §X9).
 *   ARENA      static WLS pod allocation with active-set clamping and the 1.3× quadrant
 *              LAND rule, the algorithm of fabrail-exo alloc.rs, in f64 (EXOTIC §X1.1).
 *   BASTION    jerk-limited close time from FORMULAS.scurve; t_a is an input; the only
 *              distance reported is the bound R ≥ c0·t_a (EXOTIC §X11). No yield, no
 *              scaled distance.
 *   TIDEGATE   √(g·h) arrival over a two-segment path and Green's law (EXOTIC §X12).
 *   Ride-through: T_ride = max(300 s, shaking) + landing; P_hold = P_W·(a_rms/2g)² (EXOTIC §X4).
 *   Redundancy: mid-value select and the degradation ladder (EXOTIC §X13, §S3).
 * Generated seed-42 runs (EXO_RUN, schema fabrail.docs.exo_run/1) are listed at the end when
 * present; nothing on this page invents a run.
 */
(function () {
  "use strict";
  var F = window.FabrailFormulas, P = window.FabrailPlot, UI = window.FabrailUI;
  if (!F || !P || !UI) return;
  var doc = document, fmt = UI.fmt, fmtSI = UI.fmtSI;
  var G = F.CONST.G0_MPS2;

  function $(id) { return doc.getElementById(id); }
  function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }
  function el(tag, cls, parent, text) {
    var e = doc.createElement(tag);
    if (cls) e.className = cls;
    if (text !== undefined) e.textContent = text;
    if (parent) parent.appendChild(e);
    return e;
  }
  function clear(n) { while (n && n.firstChild) n.removeChild(n.firstChild); }
  function put(scope, key, v) { var n = scope && scope.querySelector('[data-read="' + key + '"]'); if (n) n.textContent = v; }
  function pct(x, d) { return fmt(100 * x, d === undefined ? 3 : d) + " %"; }
  /* deterministic pseudo-random numbers (mulberry32) so every visit draws the same scene */
  function rng(seed) {
    var a = seed >>> 0;
    return function () {
      a = (a + 0x6d2b79f5) >>> 0;
      var t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  function line(ctx, x0, y0, x1, y1) { ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke(); }
  var SANS = "system-ui, sans-serif", MUTED = "#6e788e";
  function text(ctx, s, x, y, o) {
    o = o || {};
    ctx.font = (o.weight || "400") + " " + (o.size || 11) + "px " + SANS;
    ctx.fillStyle = o.color || MUTED;
    ctx.textAlign = o.align || "left";
    ctx.textBaseline = o.base || "alphabetic";
    ctx.fillText(s, x, y);
  }
  function cols() {
    var c = {};
    ["deep", "panel", "ink", "ink2", "muted", "line", "grid", "axis", "accent", "good", "ref", "s1", "s2", "s3", "s4", "exotic", "frontier", "p", "s"].forEach(function (n) { c[n] = P.color(n); });
    c.flux = P.color("--flux"); c.steel = P.color("--steel");
    SANS = P.theme().sans; MUTED = c.muted;
    return c;
  }

  /* A small animated canvas scene: layout on resize, repaint on theme, a Play button, and a
     reduced-motion still frame (P.loop). draw(ctx, w, h, t, col). */
  function scene(o) {
    var stage = $(o.stage), canvas = $(o.canvas), btn = $(o.play);
    if (!stage || !canvas) return null;
    var cv = null, col = cols(), t = o.stillT || 0, loop = null;
    function layout() {
      var w = Math.max(260, Math.round(stage.clientWidth || 600));
      var h = Math.round(clamp(w * (o.aspect || 0.55), o.minH || 240, o.maxH || 360));
      cv = P.setupCanvas(canvas, w, h);
    }
    function draw(tt) {
      if (!cv) return;
      t = tt;
      var ctx = cv.ctx;
      ctx.setTransform(cv.dpr, 0, 0, cv.dpr, 0, 0);
      ctx.clearRect(0, 0, cv.w, cv.h);
      ctx.fillStyle = col.deep; ctx.fillRect(0, 0, cv.w, cv.h);
      o.draw(ctx, cv.w, cv.h, tt, col);
    }
    function setBtn(on) {
      if (!btn) return;
      btn.textContent = on ? "Pause" : "Play";
      btn.setAttribute("aria-pressed", on ? "true" : "false");
    }
    loop = P.loop(function (dt, tl) { if (o.step) o.step(dt); draw(tl); }, {
      el: stage, still: function () { setBtn(false); draw(o.stillT || t); }
    });
    if (btn) btn.addEventListener("click", function () {
      if (loop.running()) { loop.stop(); setBtn(false); } else { loop.reset(t); setBtn(loop.start(true)); }
    });
    layout();
    P.onTheme(function () { col = cols(); draw(t); });
    if (window.ResizeObserver) {
      var lw = 0;
      new ResizeObserver(function () { var w = stage.clientWidth; if (Math.abs(w - lw) >= 1) { lw = w; layout(); draw(t); } }).observe(stage);
    } else window.addEventListener("resize", function () { layout(); draw(t); });
    if (loop.start()) setBtn(true); else { setBtn(false); draw(o.stillT || 0); }
    return { redraw: function () { draw(t); }, loop: loop, time: function () { return t; } };
  }

  /* ---- linear algebra: Gaussian elimination and the Lyapunov variance ------------------ */
  function solve(A, b) {
    var n = b.length, M = A.map(function (r, i) { return r.concat([b[i]]); }), i, j, k;
    for (k = 0; k < n; k++) {
      var p = k;
      for (i = k + 1; i < n; i++) if (Math.abs(M[i][k]) > Math.abs(M[p][k])) p = i;
      var tmp = M[k]; M[k] = M[p]; M[p] = tmp;
      if (M[k][k] === 0) continue;
      for (i = k + 1; i < n; i++) { var f = M[i][k] / M[k][k]; if (f) for (j = k; j <= n; j++) M[i][j] -= f * M[k][j]; }
    }
    var x = new Array(n);
    for (i = n - 1; i >= 0; i--) { var s = M[i][n]; for (j = i + 1; j < n; j++) s -= M[i][j] * x[j]; x[i] = M[i][i] ? s / M[i][i] : 0; }
    return x;
  }
  /* Stationary variance of state idx for dz = A z dt + B dW with unit intensity:
     A·P + P·Aᵀ + B·Bᵀ = 0, solved as the Kronecker-sum linear system. */
  function lyapVar(A, B, idx) {
    var n = A.length, N = n * n, K = [], i, j, k;
    for (i = 0; i < N; i++) { K.push([]); for (j = 0; j < N; j++) K[i].push(0); }
    for (i = 0; i < n; i++) for (j = 0; j < n; j++) {
      var r = i * n + j;
      for (k = 0; k < n; k++) { K[r][k * n + j] += A[i][k]; K[r][i * n + k] += A[j][k]; }
    }
    var rhs = [];
    for (i = 0; i < n; i++) for (j = 0; j < n; j++) rhs.push(-B[i] * B[j]);
    return solve(K, rhs)[idx * n + idx];
  }

  /* =============================== SENTINEL ================================================ */
  var TS = 7.0, WS = 2 * Math.PI / TS;               /* host period 7 s (EXOTIC §1) */
  function sentinelTune(mu) { return F.warburtonForce(mu); }   /* f, ζ_d on the ω_d basis */
  /* Host RMS reduction under white-noise force for the passive force-form optimum (EXOTIC §X6). */
  function rmsReduction(mu, zs) {
    var t = sentinelTune(mu), m = mu, kd = m * t.f * t.f, cd = 2 * t.zeta * m * t.f, cs = 2 * zs;
    var A = [[0, 1, 0, 0], [-(1 + kd), -(cs + cd), kd, cd], [0, 0, 0, 1], [kd / m, cd / m, -kd / m, -cd / m]];
    var v1 = lyapVar(A, [0, 1, 0, 0], 0), v0 = lyapVar([[0, 1], [-1, -cs]], [0, 1], 0);
    return 1 - Math.sqrt(v1 / v0);
  }

  var sen = { mu: 0.01, zs: 0.01 };
  var senChart = null;
  function senChartSpec() {
    var mus = [0.005, 0.01, 0.02], series = [];
    mus.forEach(function (mu, i) {
      series.push({ key: "mu" + mu, label: "μ " + pct(mu, 2), color: ["muted", "ink2", "muted"][i], dash: [undefined, undefined, [5, 4]][i], width: 1.5,
        fn: function (zs) { return 100 * rmsReduction(mu, zs); }, samples: 48, endLabel: "μ " + pct(mu, 2) });
    });
    series.push({ key: "cur", label: "selected μ " + pct(sen.mu, 2), color: "accent", fn: function (zs) { return 100 * rmsReduction(sen.mu, zs); }, samples: 48 });
    return {
      kind: "line", title: "Host displacement RMS reduction against the tower's damping ratio",
      desc: "Line chart: the percentage reduction falls as the tower's own damping rises, for mass ratios of 0.5, 1 and 2 percent and the selected one.",
      x: { label: "tower damping ζ_s", unit: "%", min: 0.5, max: 4, format: function (v) { return fmt(v, 2) + " %"; } },
      y: { label: "RMS reduction", unit: "%", min: 0, max: 70 },
      series: series.map(function (s) { var fn = s.fn; s.fn = function (x) { return fn(x / 100); }; return s; }),
      points: [{ x: 100 * sen.zs, y: 100 * rmsReduction(sen.mu, sen.zs), label: pct(rmsReduction(sen.mu, sen.zs), 3), color: "accent" }],
      height: 250, table: true, tableRows: 20
    };
  }
  function senUpdate() {
    var r = rmsReduction(sen.mu, sen.zs), t = sentinelTune(sen.mu);
    var box = $("sentinel-readouts");
    put(box, "red", pct(r, 3));
    put(box, "tune", fmt(t.f, 5) + " / " + fmt(t.zeta, 4));
    put(box, "zadd", pct(sen.mu / 2 * 1.4, 2));
    put(box, "zaddNote", "(μ/2)(S/X), S/X 1.4 · wind at S/X 7: " + pct(sen.mu / 2 * 7, 2));
    if (senChart) senChart.update(senChartSpec());
    senReset();
  }

  /* Browser 2-DOF: locked damper (host mass M(1+μ)) against the tuned damper, same gusts. */
  var senSim = null;
  function senReset() {
    var r = rng(7), comps = [];
    for (var k = 0; k < 28; k++) {
      var f = 0.02 * Math.pow(0.6 / 0.02, k / 27), x = 1200 * f / 12;      /* Davenport shape, U10 12 m/s */
      var S = x * x / (f * Math.pow(1 + x * x, 4 / 3));
      comps.push([2 * Math.PI * f, Math.sqrt(S * f), 2 * Math.PI * r()]);
    }
    var norm = 0; comps.forEach(function (c) { norm += c[1] * c[1]; }); norm = Math.sqrt(norm);
    comps.forEach(function (c) { c[1] /= norm; });
    senSim = { t: 0, comps: comps, a: [0, 0], b: [0, 0, 0, 0], hist: [], scale: 1 };
    /* warm-up: 400 s of simulated time sets a common display scale from the locked tower */
    var s2 = 0, n = 0;
    for (var i = 0; i < 12000; i++) { senStep(0.02); if (i > 3000) { s2 += senSim.a[0] * senSim.a[0]; n++; } }
    senSim.scale = 1 / (3 * Math.sqrt(s2 / n));
    senSim.hist = [];
  }
  function gust(t) {
    var v = 0, c = senSim.comps;
    for (var k = 0; k < c.length; k++) v += c[k][1] * Math.sin(c[k][0] * t + c[k][2]);
    return v;
  }
  function senDeriv(t, a, b) {
    var w = gust(t), cs = 2 * sen.zs * WS, mu = sen.mu, tn = sentinelTune(mu);
    var wd = tn.f * WS, kd = mu * wd * wd, cd = 2 * tn.zeta * mu * wd;
    /* locked: one mass (1 + μ) with the host spring and damping */
    var da = [a[1], (w - cs * a[1] - WS * WS * a[0]) / (1 + mu)];
    var xs = b[0], vs = b[1], xd = b[2], vd = b[3];
    var fs = w - cs * vs - WS * WS * xs + cd * (vd - vs) + kd * (xd - xs);
    var fd = -cd * (vd - vs) - kd * (xd - xs);
    return [da, [vs, fs, vd, fd / mu]];
  }
  function senStep(h) {
    var s = senSim, t = s.t, a = s.a, b = s.b;
    function add(v, d, k) { return v.map(function (x, i) { return x + k * d[i]; }); }
    var k1 = senDeriv(t, a, b);
    var k2 = senDeriv(t + h / 2, add(a, k1[0], h / 2), add(b, k1[1], h / 2));
    var k3 = senDeriv(t + h / 2, add(a, k2[0], h / 2), add(b, k2[1], h / 2));
    var k4 = senDeriv(t + h, add(a, k3[0], h), add(b, k3[1], h));
    s.a = a.map(function (x, i) { return x + h / 6 * (k1[0][i] + 2 * k2[0][i] + 2 * k3[0][i] + k4[0][i]); });
    s.b = b.map(function (x, i) { return x + h / 6 * (k1[1][i] + 2 * k2[1][i] + 2 * k3[1][i] + k4[1][i]); });
    s.t = t + h;
  }
  function drawTower(ctx, x0, base, H, tip, rel, label, col, hot) {
    var wBase = Math.max(26, H * 0.13), wTop = wBase * 0.62, n = 22;
    function dx(y) { var u = (base - y) / H; return tip * Math.pow(u, 1.6); }
    ctx.fillStyle = P.alpha(col.panel, 0.95); ctx.strokeStyle = col.ink2; ctx.lineWidth = 1.4;
    ctx.beginPath();
    ctx.moveTo(x0 - wBase / 2, base);
    for (var i = 0; i <= n; i++) { var y = base - H * i / n, w = wBase + (wTop - wBase) * i / n; ctx.lineTo(x0 - w / 2 + dx(y), y); }
    for (var j = n; j >= 0; j--) { var y2 = base - H * j / n, w2 = wBase + (wTop - wBase) * j / n; ctx.lineTo(x0 + w2 / 2 + dx(y2), y2); }
    ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.strokeStyle = P.alpha(col.muted, 0.35); ctx.lineWidth = 1;
    for (var k = 1; k < 18; k++) { var yy = base - H * k / 18, ww = wBase + (wTop - wBase) * k / 18; line(ctx, x0 - ww / 2 + dx(yy) + 2, yy, x0 + ww / 2 + dx(yy) - 2, yy); }
    /* the damper chamber near the top: curved guideway and the mass */
    var cy = base - H * 0.9, cw = wTop * 0.8, off = dx(cy);
    ctx.strokeStyle = col.muted; ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.arc(x0 + off, cy - cw * 1.4, cw * 1.5, 0.35 * Math.PI, 0.65 * Math.PI); ctx.stroke();
    var mx = x0 + off + clamp(rel, -cw * 0.45, cw * 0.45);
    ctx.fillStyle = hot ? col.accent : col.steel;
    ctx.fillRect(mx - cw * 0.18, cy - 5, cw * 0.36, 7);
    text(ctx, label, x0, base + 16, { align: "center", size: 11, color: col.ink2, weight: "600" });
    return dx(base - H);
  }
  function initSentinel() {
    var ctl = $("sentinel-controls");
    if (!ctl) return;
    senReset();
    var chartEl = $("sentinel-chart");
    if (chartEl) senChart = P.chart(chartEl, senChartSpec());
    UI.bindSliders(ctl, function (v) { sen.mu = v.mu; sen.zs = v.zs; senUpdate(); });
    var chip = $("sentinel-chip"), lastChip = 0;
    scene({
      stage: "sentinel-stage", canvas: "sentinel-canvas", play: "sentinel-play", aspect: 0.62, minH: 260, maxH: 360, stillT: 40,
      step: function (dt) { var n = Math.round(dt * 3 / 0.02); for (var i = 0; i < n; i++) senStep(0.02); },
      draw: function (ctx, w, h, t, col) {
        if (!senSim) return;
        var base = h - 28, H = h - 88, sc = senSim.scale * Math.min(46, w * 0.075);
        var s = senSim;
        s.hist.push([s.a[0], s.b[0]]);
        if (s.hist.length > 3600) s.hist.shift();
        /* gust arrows on the left */
        var g = gust(s.t);
        ctx.strokeStyle = P.alpha(col.flux, 0.35 + 0.3 * clamp(Math.abs(g), 0, 1)); ctx.lineWidth = 1.4;
        for (var k = 0; k < 6; k++) {
          var y = base - H * (0.25 + 0.13 * k), len = 18 + 26 * clamp(0.5 + 0.5 * g, 0, 1);
          var x0 = 12 + ((s.t * 30 + k * 23) % 40);
          line(ctx, x0, y, x0 + len, y);
          ctx.beginPath(); ctx.moveTo(x0 + len, y); ctx.lineTo(x0 + len - 5, y - 3); ctx.lineTo(x0 + len - 5, y + 3); ctx.closePath(); ctx.fillStyle = ctx.strokeStyle; ctx.fill();
        }
        text(ctx, "synthetic gusts", 12, base - H * 0.12, { size: 10 });
        ctx.strokeStyle = col.axis; ctx.lineWidth = 1.5; line(ctx, 0, base + 0.5, w, base + 0.5);
        drawTower(ctx, w * 0.36, base, H, s.a[0] * sc, 0, "damper locked", col, false);
        drawTower(ctx, w * 0.72, base, H, s.b[0] * sc, (s.b[2] - s.b[0]) * sc * 0.6, "HMD tuned (passive optimum)", col, true);
        if (chip && Date.now() - lastChip > 700 && s.hist.length > 300) {
          lastChip = Date.now();
          var q0 = 0, q1 = 0;
          for (var i = 0; i < s.hist.length; i++) { q0 += s.hist[i][0] * s.hist[i][0]; q1 += s.hist[i][1] * s.hist[i][1]; }
          chip.textContent = "Tip-sway RMS over the last " + Math.round(s.hist.length / 60 * 3) + " s of this animation: " +
            pct(1 - Math.sqrt(q1 / q0), 2) + " lower with the damper (illustrative gusts; the readouts give the white-noise value).";
        }
      }
    });
    senUpdate();
  }

  /* ================================ HAVEN-R ================================================ */
  var hav = { T: 6, zeta: 0.2 };
  var ASCE_B = [[0.02, 0.8], [0.05, 1.0], [0.1, 1.2], [0.2, 1.5], [0.3, 1.7], [0.4, 1.9], [0.5, 2.0]];   /* ASCE 7-16 Table 17.5-1 */
  function bFactor(z) {
    if (z <= ASCE_B[0][0]) return ASCE_B[0][1];
    for (var i = 1; i < ASCE_B.length; i++) if (z <= ASCE_B[i][0]) {
      var a = ASCE_B[i - 1], b = ASCE_B[i];
      return a[1] + (b[1] - a[1]) * (z - a[0]) / (b[0] - a[0]);
    }
    return ASCE_B[ASCE_B.length - 1][1];
  }
  function havenDemand(s1, T, z) {
    var B = bFactor(z);
    return { B: B, sd: s1 * G * T / (4 * Math.PI * Math.PI * B), a: s1 / (B * T) * Math.sqrt(1 + 4 * z * z) };
  }
  var havChart = null;
  function havChartSpec() {
    var fi = 1 / hav.T;
    return {
      kind: "loglog", title: "Transmissibility from floor to platform",
      desc: "Log-log chart of passive and skyhook transmissibility against frequency for the selected isolation period and damping.",
      x: { label: "frequency", unit: "Hz", min: 0.03, max: 10 }, y: { label: "platform / floor", min: 0.001, max: 5 },
      series: [
        { key: "passive", label: "passive", color: "ink2", fn: function (f) { return F.transmissibility(f / fi, hav.zeta); }, samples: 220, endLabel: "passive" },
        { key: "sky", label: "skyhook", color: "accent", fn: function (f) { return F.tSky(f / fi, hav.zeta); }, samples: 220, endLabel: "skyhook" }
      ],
      refs: [{ axis: "y", value: 1, color: "muted", dash: [2, 4] }, { axis: "x", value: fi, label: "f_iso " + fmt(fi, 3) + " Hz", color: "muted", dash: [2, 4] }],
      points: [0.5, 1, 2].map(function (f) { return { x: f, y: F.tSky(f / fi, hav.zeta), label: fmt(F.tSky(f / fi, hav.zeta), 3), color: "accent" }; }),
      height: 250, table: false
    };
  }
  var havSim = null;
  function havReset() {
    /* a 24 s loop of illustrative floor motion: broadband sinusoids under a smooth envelope */
    var r = rng(11), comps = [];
    for (var k = 0; k < 10; k++) comps.push([2 * Math.PI * (0.45 + 2.4 * r()), 0.6 + 0.4 * r(), 2 * Math.PI * r()]);
    var dt = 1 / 200, L = 24, n = Math.round(L / dt), dg = new Float64Array(n);
    for (var i = 0; i < n; i++) {
      var t = i * dt, env = Math.pow(Math.sin(Math.PI * t / L), 2), v = 0;
      for (var j = 0; j < comps.length; j++) v += comps[j][1] / (comps[j][0] * comps[j][0]) * Math.sin(comps[j][0] * t + comps[j][2]);
      dg[i] = env * v;
    }
    var vg = new Float64Array(n), ag = new Float64Array(n);
    for (var m = 1; m < n - 1; m++) { vg[m] = (dg[m + 1] - dg[m - 1]) / (2 * dt); ag[m] = (dg[m + 1] - 2 * dg[m] + dg[m - 1]) / (dt * dt); }
    var wi = 2 * Math.PI / hav.T, out = [];
    [false, true].forEach(function (sky) {
      var x = 0, v = 0, xs = new Float64Array(n), sub = 4, h = dt / sub, amax = 0;
      /* two passes so the loop starts from the periodic state */
      for (var pass = 0; pass < 2; pass++) for (var q = 0; q < n; q++) {
        for (var s = 0; s < sub; s++) {
          var damp = sky ? 2 * hav.zeta * wi * (v + vg[q]) : 2 * hav.zeta * wi * v;
          var a = -ag[q] - damp - wi * wi * x;
          v += a * h; x += v * h;
          if (pass) amax = Math.max(amax, Math.abs(a + ag[q]));
        }
        xs[q] = x;
      }
      out.push({ x: xs, amax: amax });
    });
    var dmax = 1e-12, agmax = 1e-12;
    for (var z = 0; z < n; z++) { dmax = Math.max(dmax, Math.abs(dg[z])); agmax = Math.max(agmax, Math.abs(ag[z])); }
    havSim = { dt: dt, n: n, dg: dg, ag: ag, runs: out, dmax: dmax, agmax: agmax, L: L };
  }
  function havUpdate() {
    var R = G * hav.T * hav.T / (4 * Math.PI * Math.PI);
    var d = havenDemand(0.6, hav.T, hav.zeta), m = havenDemand(0.9, hav.T, hav.zeta);
    var box = $("haven-readouts");
    put(box, "R", fmtSI(R, "m", 3));
    put(box, "dbe", fmtSI(d.sd, "m", 3) + " / " + fmt(d.a, 3) + " g");
    put(box, "mce", fmtSI(m.sd, "m", 3) + " / " + fmt(m.a, 3) + " g");
    put(box, "tilt", fmt(Math.asin(clamp(m.sd / R, -1, 1)) * 180 / Math.PI, 3) + "°");
    if (havChart) havChart.update(havChartSpec());
    havReset();
  }
  function drawPlinth(ctx, cx, floorY, fOff, relPx, w, label, col, hot, R) {
    var pitW = w * 0.9, pitD = 26;
    /* pit and guideway move with the floor */
    ctx.fillStyle = P.alpha(col.deep, 0.9); ctx.fillRect(cx - pitW / 2 + fOff, floorY, pitW, pitD);
    ctx.strokeStyle = col.ink2; ctx.lineWidth = 1.4;
    var rr = pitW * 1.6;
    ctx.beginPath(); ctx.arc(cx + fOff, floorY + pitD - 4 - rr, rr, 0.5 * Math.PI - 0.28, 0.5 * Math.PI + 0.28); ctx.stroke();
    /* moat covers: passive plates bridging the gap at each side */
    ctx.fillStyle = col.steel;
    ctx.fillRect(cx - pitW / 2 + fOff - 2, floorY - 3, 16, 3); ctx.fillRect(cx + pitW / 2 + fOff - 14, floorY - 3, 16, 3);
    /* platform: sits on the arc at its relative offset, tilted by the arc slope (exaggerated) */
    var px = cx + fOff + relPx, ang = clamp(relPx / (rr * 0.6), -0.25, 0.25);
    ctx.save();
    ctx.translate(px, floorY + pitD - 8 - (rr - Math.sqrt(Math.max(0, rr * rr - relPx * relPx))));
    ctx.rotate(ang);
    ctx.fillStyle = P.alpha(col.exotic, 0.3); ctx.strokeStyle = col.exotic; ctx.lineWidth = 1.4;
    ctx.fillRect(-pitW * 0.36, -8, pitW * 0.72, 8); ctx.strokeRect(-pitW * 0.36, -8, pitW * 0.72, 8);
    ctx.fillStyle = P.alpha(col.flux, hot ? 0.35 : 0.2); ctx.fillRect(-pitW * 0.3, 0, pitW * 0.6, 3);
    ctx.fillStyle = P.alpha(col.line, 0.8); ctx.strokeStyle = col.ink2;
    ctx.fillRect(-12, -52, 24, 44); ctx.strokeRect(-12, -52, 24, 44);
    ctx.fillStyle = col.ink2; ctx.beginPath(); ctx.ellipse(0, -62, 6, 10, 0, 0, 2 * Math.PI); ctx.fill();
    ctx.restore();
    text(ctx, label, cx + fOff, floorY + pitD + 16, { align: "center", size: 11, weight: "600", color: col.ink2 });
  }
  function initHaven() {
    var ctl = $("haven-controls");
    if (!ctl) return;
    var chartEl = $("haven-chart");
    if (chartEl) havChart = P.chart(chartEl, havChartSpec());
    UI.bindSliders(ctl, function (v) { hav.T = v.T; hav.zeta = v.zeta; havUpdate(); });
    var chip = $("haven-chip");
    scene({
      stage: "haven-stage", canvas: "haven-canvas", play: "haven-play", aspect: 0.55, minH: 240, maxH: 340, stillT: 11.3,
      draw: function (ctx, w, h, t, col) {
        if (!havSim) return;
        var i = Math.floor(((t % havSim.L) / havSim.dt)) % havSim.n;
        var sc = Math.min(22, w * 0.04) / havSim.dmax, floorY = h * 0.52;
        var fOff = havSim.dg[i] * sc;
        ctx.fillStyle = P.alpha(col.line, 0.35); ctx.fillRect(0 + fOff, floorY, w, h - floorY);
        ctx.strokeStyle = col.axis; ctx.lineWidth = 1.5; line(ctx, 0, floorY + 0.5, w, floorY + 0.5);
        drawPlinth(ctx, w * 0.28, floorY, fOff, havSim.runs[0].x[i] * sc, w * 0.36, "passive, ζ " + fmt(hav.zeta, 2), col, false);
        drawPlinth(ctx, w * 0.72, floorY, fOff, havSim.runs[1].x[i] * sc, w * 0.36, "skyhook, ζ " + fmt(hav.zeta, 2), col, true);
        text(ctx, "floor motion · illustrative", 10, h - 10, { size: 10 });
        if (chip) chip.textContent = "Peak platform acceleration in this loop: passive " + pct(havSim.runs[0].amax / havSim.agmax, 2) +
          ", skyhook " + pct(havSim.runs[1].amax / havSim.agmax, 2) + " of the floor's (T " + fmt(hav.T, 3) + " s).";
      }
    });
    havUpdate();
  }

  /* ================================= ARENA ================================================= */
  var AR = { nx: 25, ny: 20, Lx: 123, Ly: 71, massKg: 8.6e6, k: 1.5, capMin: 1.3, firstFail: 13 * 25 + 6 };   /* the story: one pod fails */
  var pods = [], base = null, cur = null, arenaView = "delta", arenaRand = rng(2026);
  function initPods() {
    pods = [];
    var W = AR.massKg * G, fmax = AR.k * W / (AR.nx * AR.ny);
    for (var j = 0; j < AR.ny; j++) for (var i = 0; i < AR.nx; i++) {
      pods.push({ x: (i + 0.5) * AR.Lx / AR.nx - AR.Lx / 2, y: (j + 0.5) * AR.Ly / AR.ny - AR.Ly / 2, fmax: fmax, w: 1, healthy: true, i: i, j: j });
    }
  }
  /* WLS with active-set clamping (fabrail-exo alloc.rs, in f64): minimise Σw(f − f̄)² subject to
     Σf = Fz, Σf·y = Mx, −Σf·x = My, 0 ≤ f ≤ fmax; f̄ is equal utilisation over the healthy pods. */
  function allocate(Fz, Mx, My) {
    var n = pods.length, f = new Float64Array(n), bound = new Int8Array(n), L = Math.max(AR.Lx, AR.Ly), i;
    var cap = 0;
    for (i = 0; i < n; i++) if (pods[i].healthy) cap += pods[i].fmax;
    if (cap <= 0) return { f: f, ok: false };
    var fbar = new Float64Array(n);
    for (i = 0; i < n; i++) fbar[i] = pods[i].healthy ? Fz * pods[i].fmax / cap : 0;
    for (var pass = 0; pass < 64; pass++) {
      /* residual wrench after the clamped pods, and the free pods at f̄ */
      var tau = [Fz, Mx / L, My / L], Bf = [0, 0, 0], Mm = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
      for (i = 0; i < n; i++) {
        if (!pods[i].healthy) continue;
        var fi = bound[i] ? (bound[i] > 0 ? pods[i].fmax : 0) : fbar[i];
        var row = [1, pods[i].y / L, -pods[i].x / L];
        Bf[0] += row[0] * fi; Bf[1] += row[1] * fi; Bf[2] += row[2] * fi;
        if (!bound[i]) for (var a = 0; a < 3; a++) for (var b = 0; b < 3; b++) Mm[a][b] += row[a] * row[b] / pods[i].w;
      }
      var lam = solve(Mm, [tau[0] - Bf[0], tau[1] - Bf[1], tau[2] - Bf[2]]);
      var violated = false;
      for (i = 0; i < n; i++) {
        if (!pods[i].healthy) { f[i] = 0; continue; }
        if (bound[i]) { f[i] = bound[i] > 0 ? pods[i].fmax : 0; continue; }
        f[i] = fbar[i] + (lam[0] + lam[1] * pods[i].y / L - lam[2] * pods[i].x / L) / pods[i].w;
        if (f[i] > pods[i].fmax) { bound[i] = 1; violated = true; } else if (f[i] < 0) { bound[i] = -1; violated = true; }
      }
      if (!violated) break;
    }
    var sz = 0; for (i = 0; i < n; i++) sz += f[i];
    return { f: f, ok: Math.abs(sz - Fz) <= 1e-4 * Fz };
  }
  function quadrant(p) { return (p.x >= 0 ? 2 : 0) + (p.y >= 0 ? 1 : 0); }
  function arenaSolve() {
    var W = AR.massKg * G;
    cur = allocate(W, 0, 0);
    var share = [0, 0, 0, 0], capq = [0, 0, 0, 0];
    for (var i = 0; i < pods.length; i++) {
      share[quadrant(pods[i])] += base.f[i];
      if (pods[i].healthy) capq[quadrant(pods[i])] += pods[i].fmax;
    }
    var worst = Infinity;
    for (var q = 0; q < 4; q++) worst = Math.min(worst, capq[q] / share[q]);
    cur.worst = worst;
    cur.land = !cur.ok || worst < AR.capMin || !pods.some(function (p) { return p.healthy; });
  }
  function arenaReadouts() {
    var box = $("arena-readouts"), healthy = pods.filter(function (p) { return p.healthy; }).length;
    var peak = 0, peak0 = 0;
    for (var i = 0; i < pods.length; i++) { peak = Math.max(peak, cur.f[i] / pods[i].fmax); peak0 = Math.max(peak0, base.f[i] / pods[i].fmax); }
    put(box, "healthy", String(healthy));
    put(box, "peak", fmt(peak, 4));
    put(box, "peakNote", "all healthy: " + fmt(peak0, 4));
    put(box, "quad", fmt(cur.worst, 4));
    put(box, "land", cur.land ? "LAND" : "carry on");
    put(box, "landNote", cur.land ? "a quadrant is below 1.3× its share: land and lock" : "every quadrant above 1.3× its share");
  }
  var arenaCanvas = null, arenaCv = null;
  function arenaGeom() {
    var w = arenaCv.w, h = arenaCv.h, pad = 14;
    var s = Math.min((w - 2 * pad) / AR.Lx, (h - 2 * pad - 14) / AR.Ly);
    return { s: s, x0: (w - AR.Lx * s) / 2, y0: pad + 12, cw: AR.Lx * s / AR.nx, ch: AR.Ly * s / AR.ny };
  }
  function mix(c1, c2, t) {
    function rgb(c) {
      var m = /^#?([0-9a-f]{6})$/i.exec(c);
      if (m) { var n = parseInt(m[1], 16); return [(n >> 16) & 255, (n >> 8) & 255, n & 255]; }
      var r = /rgba?\(([^)]+)\)/.exec(c);
      if (r) return r[1].split(/[\s,]+/).slice(0, 3).map(Number);
      return [128, 128, 128];
    }
    var a = rgb(c1), b = rgb(c2);
    return "rgb(" + Math.round(a[0] + (b[0] - a[0]) * t) + "," + Math.round(a[1] + (b[1] - a[1]) * t) + "," + Math.round(a[2] + (b[2] - a[2]) * t) + ")";
  }
  function drawArena() {
    if (!arenaCanvas) return;
    var w = Math.max(260, Math.round(arenaCanvas.clientWidth || 600)), h = Math.round(clamp(w * 0.55, 220, 330));
    arenaCv = P.setupCanvas(arenaCanvas, w, h);
    var ctx = arenaCv.ctx, col = cols(), g = arenaGeom();
    ctx.fillStyle = col.deep; ctx.fillRect(0, 0, w, h);
    var maxD = 0.002, i;
    for (i = 0; i < pods.length; i++) if (pods[i].healthy) maxD = Math.max(maxD, Math.abs(cur.f[i] - base.f[i]) / pods[i].fmax);
    var neutral = mix(col.panel, col.muted, 0.28), lo = col.s1, hi = col.s2;
    for (i = 0; i < pods.length; i++) {
      var p = pods[i], x = g.x0 + p.i * g.cw, y = g.y0 + (AR.ny - 1 - p.j) * g.ch, c;
      if (!p.healthy) {
        ctx.fillStyle = P.alpha(col.muted, 0.25); ctx.fillRect(x + 1, y + 1, g.cw - 2, g.ch - 2);
        ctx.strokeStyle = col.ref; ctx.lineWidth = 1.5;
        line(ctx, x + 3, y + 3, x + g.cw - 3, y + g.ch - 3); line(ctx, x + g.cw - 3, y + 3, x + 3, y + g.ch - 3);
        continue;
      }
      if (arenaView === "delta") {
        var d = (cur.f[i] - base.f[i]) / p.fmax / maxD;
        c = d >= 0 ? mix(neutral, hi, clamp(d, 0, 1)) : mix(neutral, lo, clamp(-d, 0, 1));
      } else {
        var u = cur.f[i] / p.fmax;
        c = mix(neutral, col.accent, clamp((u - 0.55) / 0.45, 0, 1));
      }
      ctx.fillStyle = c; ctx.fillRect(x + 1, y + 1, g.cw - 2, g.ch - 2);
    }
    ctx.strokeStyle = col.muted; ctx.lineWidth = 1; ctx.setLineDash([4, 4]);
    line(ctx, g.x0 + AR.Lx * g.s / 2, g.y0 - 4, g.x0 + AR.Lx * g.s / 2, g.y0 + AR.Ly * g.s + 4);
    line(ctx, g.x0 - 4, g.y0 + AR.Ly * g.s / 2, g.x0 + AR.Lx * g.s + 4, g.y0 + AR.Ly * g.s / 2);
    ctx.setLineDash([]);
    text(ctx, "plan view · " + AR.Lx + " × " + AR.Ly + " m · quadrants dashed", g.x0, 14, { size: 10.5, color: col.ink2 });
    var lg = $("arena-legend");
    if (lg) {
      clear(lg);
      var sw = el("span", "pod-ramp", lg);
      if (arenaView === "delta") {
        sw.style.background = "linear-gradient(90deg," + lo + "," + neutral + "," + hi + ")";
        el("span", "", lg, "load change against the all-healthy allocation: −" + pct(maxD, 2) + " … +" + pct(maxD, 2) + " of f_max");
      } else {
        sw.style.background = "linear-gradient(90deg," + neutral + "," + col.accent + ")";
        el("span", "", lg, "utilisation f / f_max: 0.55 … 1.0");
      }
      el("span", "", lg, "crossed out: failed pod");
    }
  }
  function failPod(i) {
    if (i < 0 || i >= pods.length || !pods[i].healthy) return;
    pods[i].healthy = false;
  }
  function arenaUpdate() { arenaSolve(); arenaReadouts(); drawArena(); }
  function initArena() {
    arenaCanvas = $("arena-canvas");
    if (!arenaCanvas) return;
    initPods();
    base = allocate(AR.massKg * G, 0, 0);
    failPod(AR.firstFail);
    arenaSolve();
    arenaCanvas.addEventListener("click", function (ev) {
      var r = arenaCanvas.getBoundingClientRect(), g = arenaGeom();
      var i = Math.floor((ev.clientX - r.left - g.x0) / g.cw), j = AR.ny - 1 - Math.floor((ev.clientY - r.top - g.y0) / g.ch);
      if (i >= 0 && i < AR.nx && j >= 0 && j < AR.ny) { failPod(j * AR.nx + i); arenaUpdate(); }
    });
    function healthyIdx() { var o = []; pods.forEach(function (p, k) { if (p.healthy) o.push(k); }); return o; }
    $("arena-fail1").addEventListener("click", function () {
      var h = healthyIdx(); if (h.length) { failPod(h[Math.floor(arenaRand() * h.length)]); arenaUpdate(); }
    });
    $("arena-fail10").addEventListener("click", function () {
      var h = healthyIdx(); if (!h.length) return;
      var c = pods[h[Math.floor(arenaRand() * h.length)]];
      h.sort(function (a, b) { var pa = pods[a], pb = pods[b]; return Math.hypot(pa.x - c.x, pa.y - c.y) - Math.hypot(pb.x - c.x, pb.y - c.y); });
      for (var k = 0; k < Math.min(10, h.length); k++) failPod(h[k]);
      arenaUpdate();
    });
    $("arena-reset").addEventListener("click", function () { initPods(); failPod(AR.firstFail); arenaRand = rng(2026); arenaUpdate(); });
    UI.segmented($("arena-view"), function (v) { arenaView = v; drawArena(); });
    P.onTheme(drawArena);
    if (window.ResizeObserver) new ResizeObserver(function () { drawArena(); }).observe(arenaCanvas);
    else window.addEventListener("resize", drawArena);
    arenaUpdate();
  }

  /* ================================ BASTION ================================================ */
  var BA = { L: 3.0, j: 2.0, vmax: 1.2, seal: 0.6, c0: 343.0 };
  var bas = { ta: 8, td: 0, a: 0.48 };
  function basPlan() {
    var sc = F.scurve(BA.L, BA.vmax, bas.a, BA.j);
    var need = sc.T + BA.seal, have = bas.ta - bas.td;
    return { sc: sc, need: need, have: have, close: need <= have, tNeed: need + bas.td, rMin: BA.c0 * bas.ta };
  }
  function basVerdict() {
    var p = basPlan(), v = $("bastion-verdict");
    if (!v) return;
    clear(v);
    v.setAttribute("data-state", p.close ? "ok" : "no");
    var s = el("strong", "", v, p.close ? "CLOSE. " : "No close: fallback. ");
    s.setAttribute("data-state", p.close ? "ok" : "no");
    v.appendChild(doc.createTextNode("t_min " + fmtSI(p.sc.T, "s", 4) + " (peak " + fmt(p.sc.vPeak, 4) + " m/s) + seal and margin " + fmtSI(BA.seal, "s", 2) +
      " = " + fmtSI(p.need, "s", 4) + (p.close ? " ≤ " : " > ") + "t_a − t_detect = " + fmtSI(p.have, "s", 4) + ". " +
      (p.close ? "The door completes its close and lands to seal." : "The planner does not start a close it cannot finish; the door takes its configured fallback: land in place, or hold open landed.") +
      " A shock never travels slower than sound, so t_a = " + fmtSI(bas.ta, "s", 3) + " means a source at least " + fmtSI(p.rMin, "m", 3) + " away (t_a ≤ R/c0)."));
  }
  function initBastion() {
    var ctl = $("bastion-controls");
    if (!ctl) return;
    UI.bindSliders(ctl, function (v) { bas.ta = v.ta; bas.td = v.td; basVerdict(); if (bs) bs.redraw(); });
    UI.segmented($("bastion-accel"), function (v) { bas.a = parseFloat(v); basVerdict(); if (bs) bs.redraw(); });
    var chip = $("bastion-chip");
    var bs = scene({
      stage: "bastion-stage", canvas: "bastion-canvas", play: "bastion-play", aspect: 0.55, minH: 240, maxH: 340, stillT: 6.2,
      draw: function (ctx, w, h, t, col) {
        var p = basPlan(), span = Math.max(bas.ta, p.tNeed) + 2, u = t % span;
        var gy = h * 0.78, wallX = w * 0.34, openW = w * 0.3, top = h * 0.22;
        /* tunnel portal: wall with a 3 m opening, and the door pocket on the left */
        ctx.fillStyle = P.alpha(col.line, 0.5); ctx.fillRect(0, gy, w, h - gy);
        ctx.fillStyle = col.steel;
        ctx.fillRect(wallX - 16, top, 16, gy - top); ctx.fillRect(wallX + openW, top, 16, gy - top);
        ctx.fillRect(wallX - 16, top - 12, openW + 32, 12);
        ctx.strokeStyle = col.ink2; ctx.lineWidth = 1.2; ctx.setLineDash([3, 3]); ctx.strokeRect(wallX, top, openW, gy - top); ctx.setLineDash([]);
        /* the plug: jerk-limited S-curve from the pocket across the opening, then the land-to-seal step */
        var x = 0, sealed = 0;
        if (p.close && u >= bas.td) {
          var s = p.sc.sample(Math.min(u - bas.td, p.sc.T));
          x = s.x / BA.L;
          if (u - bas.td >= p.sc.T) sealed = clamp((u - bas.td - p.sc.T) / BA.seal, 0, 1);
        }
        var dx = wallX - openW - 18 + x * (openW + 18);
        ctx.fillStyle = P.alpha(col.exotic, 0.3); ctx.strokeStyle = col.exotic; ctx.lineWidth = 1.6;
        ctx.fillRect(dx, top + 4 + sealed * 3, openW, gy - top - 6); ctx.strokeRect(dx, top + 4 + sealed * 3, openW, gy - top - 6);
        if (sealed >= 1) { ctx.strokeStyle = col.good; ctx.lineWidth = 3; line(ctx, dx, gy - 1, dx + openW, gy - 1); }
        /* schematic pressure front: reaches the door face at t = t_a (no distance scale) */
        var fx = wallX + openW + 16 + (bas.ta - u) * (w * 0.1);
        if (fx < w + 40 && u <= bas.ta + 0.4) {
          for (var k = 0; k < 4; k++) { ctx.strokeStyle = P.alpha(col.s, 0.85 - k * 0.2); ctx.lineWidth = k ? 1 : 2.4; line(ctx, fx + k * 6, top - 20, fx + k * 6, gy); }
          text(ctx, "pressure front (schematic)", Math.min(w - 8, fx + 30), top - 26, { align: "right", size: 10.5, color: col.ink2 });
        }
        text(ctx, "3 m opening", wallX + openW / 2, gy + 16, { align: "center", size: 10.5 });
        text(ctx, "t = " + fmt(u, 3) + " s after detection", 10, h - 10, { size: 10.5, color: col.ink2 });
        if (chip) {
          var msg;
          if (!p.close) msg = u < bas.ta ? "Infeasible: the planner holds; the door lands in place." : "Arrival at t_a = " + fmt(bas.ta, 3) + " s: the door is open and landed (fallback).";
          else if (u < bas.td) msg = "Detecting…";
          else if (u < bas.td + p.sc.T) msg = "Closing: " + fmt(x * BA.L, 3) + " m of 3 m.";
          else if (u < bas.td + p.sc.T + BA.seal) msg = "Landing to seal.";
          else msg = u < bas.ta ? "Sealed " + fmtSI(bas.ta - p.tNeed, "s", 3) + " before t_a." : "Arrival at t_a: sealed.";
          if (chip.textContent !== msg) chip.textContent = msg;
        }
      }
    });
    basVerdict();
  }

  /* ================================ TIDEGATE =============================================== */
  var tide = { l1: 0, l2: 100e3, h2: 1000, shake: 120, cmd: 60, close: 90 };
  var TIDE = { h1: 4000, hGate: 10, check: 30 };
  var tideChart = null;
  function tideCalc() {
    var c1 = Math.sqrt(G * TIDE.h1), c2 = Math.sqrt(G * tide.h2);
    var arr = tide.l1 / c1 + tide.l2 / c2;
    var fail = !!($("tide-gapfail") && $("tide-gapfail").checked);
    var tCheck = tide.shake + TIDE.check, tCmd = tide.shake + tide.cmd;
    var tStart = Math.max(tCheck, tCmd), dur = fail ? 2 * tide.close : tide.close;
    var greens = Math.pow((tide.l1 > 0 ? TIDE.h1 : tide.h2) / TIDE.hGate, 0.25);
    return { arr: arr, tCheck: tCheck, tCmd: tCmd, tStart: tStart, tEnd: tStart + dur, fail: fail, greens: greens, c2: c2 };
  }
  function tideSpec() {
    var c = tideCalc(), m = 1 / 60;
    return {
      kind: "gantt", title: "Tide-gate timeline after the origin",
      desc: "Gantt chart in minutes: shaking, the rail-gap check, the external decision-system command, the gate close and the tsunami arrival.",
      x: { label: "after origin", unit: "min", min: 0, max: Math.max(c.arr, c.tEnd) * m * 1.08 },
      rows: [
        { label: "shaking", color: "s", spans: [{ from: 0, to: tide.shake * m, color: "s", label: "shaking" }] },
        { label: "rail-gap check", color: "muted", spans: [{ from: tide.shake * m, to: c.tCheck * m, color: c.fail ? "ref" : "muted", style: c.fail ? "hatch" : "solid", label: c.fail ? "fails" : "passes" }] },
        { label: "external command", color: "accent", spans: [{ from: c.tCmd * m, to: c.tCmd * m + 0.15, color: "accent", label: "" }] },
        { label: c.fail ? "close on wheels" : "close by maglev", color: "exotic", spans: [{ from: c.tStart * m, to: c.tEnd * m, color: "exotic", label: fmt((c.tEnd - c.tStart), 3) + " s" }] }
      ],
      refs: [{ axis: "x", value: c.arr * m, label: "tsunami " + fmt(c.arr * m, 3) + " min", color: "ref", dash: [6, 3] }],
      table: true
    };
  }
  function tideUpdate() {
    var c = tideCalc(), v = $("tide-verdict");
    if (tideChart) tideChart.update(tideSpec());
    if (!v) return;
    clear(v);
    var ok = c.tEnd <= c.arr;
    v.setAttribute("data-state", ok ? "ok" : "no");
    var s = el("strong", "", v, ok ? "Closed " + fmt((c.arr - c.tEnd) / 60, 3) + " min before arrival. " : "Too late by " + fmt((c.tEnd - c.arr) / 60, 3) + " min. ");
    s.setAttribute("data-state", ok ? "ok" : "no");
    v.appendChild(doc.createTextNode("Arrival " + fmt(c.arr / 60, 3) + " min (shelf speed √(g·h) = " + fmt(c.c2, 3) + " m/s); gate closed at " + fmt(c.tEnd / 60, 3) + " min" +
      (c.fail ? " on wheels, because the rail-gap check failed" : " by maglev after the rail-gap check passed") +
      ". Green's law amplitude growth to a 10 m approach: ×" + fmt(c.greens, 3) + ". Inputs: deep path " + fmtSI(tide.l1, "m", 3) + " at 4,000 m, shelf " + fmtSI(tide.l2, "m", 3) + " at " + fmt(tide.h2, 4) + " m."));
  }
  function initTide() {
    var ctl = $("tide-controls");
    if (!ctl) return;
    var chartEl = $("tide-chart");
    if (chartEl) tideChart = P.chart(chartEl, tideSpec());
    UI.bindSliders(ctl, function (v) { for (var k in v) tide[k] = v[k]; tideUpdate(); });
    var cb = $("tide-gapfail");
    if (cb) cb.addEventListener("change", tideUpdate);
    tideUpdate();
  }

  /* ============================== ride-through ============================================== */
  var ride = { mass: 1e7, arms: 0.2, tshake: 300, tland: 0 };
  var P_W_PER_KG = 1.7;                               /* 1.7 kW/t = 1.7 W/kg (Transrapid; EXOTIC §X4) */
  var EMALS_J = 121e6, JET_USABLE_J = 2.8e9;           /* EXOTIC §X5 benchmarks */
  function rideUpdate() {
    var kind = $("ride-kind") ? $("ride-kind").value : "hybrid", margin = $("ride-margin") && $("ride-margin").checked ? 2 : 1;
    var pw = P_W_PER_KG * ride.mass, tr = Math.max(300, ride.tshake) + ride.tland, p, pNote;
    if (kind === "hybrid") { p = pw * Math.pow(ride.arms / 2, 2); pNote = "P_W·(a_rms/2g)², P_W = " + fmtSI(pw, "W", 3); }
    else if (kind === "pure") { p = pw; pNote = "the whole weight: P_W = 1.7 kW/t"; }
    else { p = NaN; pNote = "sensors and propulsion only (kW scale)"; }
    var e = p * tr * margin, box = $("ride-readouts");
    put(box, "p", isFinite(p) ? fmtSI(p, "W", 3) : "kW scale");
    put(box, "pNote", pNote);
    put(box, "t", fmtSI(tr, "s", 3));
    put(box, "e", isFinite(e) ? fmtSI(e, "J", 3) : "–");
    put(box, "eNote", margin > 1 ? "with the 2× margin (" + (isFinite(p) ? fmtSI(p * tr, "J", 3) : "–") + " without)" : "no margin");
    var v = $("ride-verdict");
    if (!v) return;
    clear(v);
    var s;
    if (kind === "hts") {
      v.setAttribute("data-state", "ok");
      s = el("strong", "", v, "The cryogen is the buffer. ");
      v.appendChild(doc.createTextNode("Flux-pinned HTS holds its lift without power; the LN₂ reservoir (about 40 h static, 30 h dynamic) is what runs out, and after depletion the lift fades over tens of minutes (EXOTIC §X3; FAB-EXO-405)."));
      return;
    }
    var ok = e <= EMALS_J, ok2 = e <= JET_USABLE_J;
    v.setAttribute("data-state", ok2 ? "ok" : "no");
    s = el("strong", "", v, ok ? "Within one EMALS-class disk. " : ok2 ? "Beyond one EMALS-class disk; within a JET-class flywheel. " : "Flagged insufficient. ");
    s.setAttribute("data-state", ok2 ? "ok" : "no");
    v.appendChild(doc.createTextNode(fmtSI(e, "J", 3) + " against " + fmtSI(EMALS_J, "J", 3) + " delivered per EMALS cycle and ≈ " + fmtSI(JET_USABLE_J, "J", 2) +
      " usable in one JET-class flywheel (3.75 GJ stored). Storage size is not the only limit: power-electronics reliability at this scale needs reliability growth (EMALS reached 614 cycles between failures against a 4,166 requirement)."));
  }
  function initRide() {
    var ctl = $("ride-controls");
    if (!ctl) return;
    UI.bindSliders(ctl, function (v) { for (var k in v) ride[k] = v[k]; rideUpdate(); });
    ["ride-kind", "ride-margin"].forEach(function (id) { var n = $(id); if (n) n.addEventListener("change", rideUpdate); });
    rideUpdate();
  }

  /* ============================== redundancy ================================================= */
  function mvs(a, b, c) { return Math.max(Math.min(a, b), Math.min(Math.max(a, b), c)); }
  function initRedundancy() {
    var ctl = $("mvs-controls"), out = $("mvs-out");
    if (ctl && out) {
      var bind = UI.bindSliders(ctl, function (v) {
        var m = mvs(v.a, v.b, v.c), sorted = [v.a, v.b, v.c].sort(function (x, y) { return x - y; });
        out.textContent = "mvs = " + fmt(m, 4) + " mm · inputs " + sorted.map(function (x) { return fmt(x, 4); }).join(" ≤ ") + " mm · the output is the median";
      });
      var fb = $("mvs-fault");
      if (fb) fb.addEventListener("click", function () { bind.set("c", 20); });
    }
    var rung = 0, ladder = $("ladder"), lo = $("ladder-out");
    function show(msg) {
      var items = ladder ? ladder.querySelectorAll("li") : [];
      for (var i = 0; i < items.length; i++) { if (+items[i].getAttribute("data-rung") === rung) items[i].setAttribute("aria-current", "step"); else items[i].removeAttribute("aria-current"); }
      if (lo) lo.textContent = msg;
    }
    var msgs = ["2oo3 with three healthy channels.", "1oo2D: one channel lost; the pair runs with diagnostics and the diverse L(g) tie-breaker.",
      "1oo1D: a diagnosed second loss with L(g) healthy.", "Safe state: STO on the LSM, the brake, the energised landing. Latched until maintenance."];
    if ($("ladder-lose")) $("ladder-lose").addEventListener("click", function () { rung = Math.min(3, rung + 1); show(msgs[rung]); });
    if ($("ladder-double")) $("ladder-double").addEventListener("click", function () { rung = 3; show("Two losses on one cycle: common cause, straight to the safe state, whatever the diverse channel says."); });
    if ($("ladder-reset")) $("ladder-reset").addEventListener("click", function () { rung = 0; show("Maintenance reset: " + msgs[0]); });
    show(msgs[0]);
  }

  /* ================================ recorded runs ============================================ */
  function initRuns() {
    var R = window.EXO_RUN, list = $("runs-list"), note = $("runs-note");
    if (!list) return;
    if (!R || R.placeholder || !R.runs || !Object.keys(R.runs).length) {
      if (note) note.textContent = "The seed-42 exotic runs (EXO_RUN) are not generated yet. When they are, each appears here with its inputs, KPIs, badge and maturity, and a \"reduced model\" label for roadmap classes. Nothing above depends on them.";
      return;
    }
    if (note) note.textContent = "Seed " + R.seed + ", fabrail " + R.fabrail_version + ", generated by " + R.generated_by + ". Every alert in these runs is synthetic or injected.";
    Object.keys(R.runs).sort().forEach(function (key) {
      var r = R.runs[key], fig = el("div", "figure", list), head = el("div", "figure-title", fig, String(r["class"] || key).toUpperCase() + " · " + (r.scenario || "") + " ");
      head.appendChild(UI.badge(r.badge === "frontier" ? "f" : String(r.badge || "r").toLowerCase()));
      var meta = el("p", "small muted", fig, "maturity " + (r.maturity || "–") + (r.reduced_model ? " · reduced model" : "") + " · " + fmt(r.rate_hz, 3) + " Hz");
      meta.style.marginTop = "0.3rem";
      var names = r.channels ? Object.keys(r.channels).slice(0, 2) : [];
      names.forEach(function (nm) {
        var c = r.channels[nm], data = [];
        for (var i = 0; i < Math.min(c.data.length, r.t_s.length); i++) data.push([r.t_s[i], c.data[i]]);
        var box = el("div", "", fig);
        P.chart(box, { kind: "line", title: nm, desc: nm + " against time", x: { label: "time", unit: "s" }, y: { label: nm, unit: c.unit }, series: [{ key: nm, label: nm, color: "exotic", data: data }], height: 170 });
      });
      function kv(obj, title) {
        if (!obj || !Object.keys(obj).length) return;
        el("div", "fig-sub small", fig, title).style.marginTop = "0.6rem";
        var t = el("table", "kv", el("div", "table-wrap", fig)), tb = el("tbody", "", t);
        Object.keys(obj).sort().forEach(function (k) {
          var tr = el("tr", "", tb);
          el("td", "", tr, k);
          el("td", "num", tr, typeof obj[k] === "number" ? fmt(obj[k], 4) : String(obj[k]));
        });
      }
      kv(r.inputs, "Inputs");
      kv(r.kpis, "KPIs");
    });
  }

  function init() {
    initSentinel();
    initHaven();
    initArena();
    initBastion();
    initTide();
    initRide();
    initRedundancy();
    initRuns();
  }
  if (doc.readyState === "loading") doc.addEventListener("DOMContentLoaded", init); else init();
})();
