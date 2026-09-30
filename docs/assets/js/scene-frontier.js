/* frontier.html scenes (owner W4): the 🔭 frontier observatory (plan §4, §3b; FRONTIER.md).
 *
 * Every number on this page is a what-if number, never a capability claim. The browser
 * re-evaluates the closed forms of python/fabrail/exotic/frontier/ (numpy, the single
 * source; FAB-EXO-002) with the same constants:
 *   PEGS        M0 = 10^(1.5·Mw + 9.1); triangle moment rate, half-duration 2.26e-6·M0^(1/3);
 *               h ≈ κ_h·G·I₂(t)/R⁵ with κ_h 4.6; detection at SNR 5 in 0.1 Hz; P at 6.5 km/s;
 *               dense network at 22 km + 3.3 s (pegs.py; FRONTIER §F4). Checked against the
 *               Python sweep table to its printed 0.01 s.
 *   metabarrier m = ρ·(v_s/f)³, ρ 1,800 kg/m³; missing orders 3·log10(30 Hz/f) (§F5).
 *   RTSC        P̄(h) = (B0²/μ0)·e^(−2kh), B0 1.2 T, k 40.85 /m; p = min(η·j, 1)·P̄ with
 *               j = (1 − 300/Tc)^1.5/(1 − 77/92)^1.5 (§F2).
 *   cloak       μ_FM = (R2² + R1²)/(R2² − R1²); missing orders log10(R2/1 cm) (§F8).
 *   SELENE      q = ε·σ·T⁴; pole ×√(1.62/9.81) (§F7).
 * When the generated EXO_RUN.frontier cards exist (scripts/update_docs_run.py from
 * fabrail.exotic.frontier.meta.frontier_cards()), the missing-orders list uses them.
 */
(function () {
  "use strict";
  var P = window.FabrailPlot, UI = window.FabrailUI;
  if (!P || !UI) return;
  var doc = document, fmt = UI.fmt, fmtSI = UI.fmtSI;

  var MU0 = 1.25663706212e-6, SIGMA_SB = 5.670374419e-8, G0 = 9.80665, G_MOON = 1.62, G_N = 6.6743e-11;

  function $(id) { return doc.getElementById(id); }
  function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }
  function log10(x) { return Math.log(x) / Math.LN10; }
  function el(tag, cls, parent, text) {
    var e = doc.createElement(tag);
    if (cls) e.className = cls;
    if (text !== undefined) e.textContent = text;
    if (parent) parent.appendChild(e);
    return e;
  }
  function clear(n) { while (n && n.firstChild) n.removeChild(n.firstChild); }
  function put(scope, key, v) { var n = scope && scope.querySelector('[data-read="' + key + '"]'); if (n) n.textContent = v; }
  function sci(v, d) { return fmt(v, d === undefined ? 2 : d); }
  var SANS = "system-ui, sans-serif";
  function text(ctx, s, x, y, o) {
    o = o || {};
    ctx.font = (o.weight || "400") + " " + (o.size || 11) + "px " + SANS;
    ctx.fillStyle = o.color || P.color("muted");
    ctx.textAlign = o.align || "left";
    ctx.textBaseline = o.base || "alphabetic";
    ctx.fillText(s, x, y);
  }
  function cols() {
    var c = {};
    ["deep", "panel", "ink", "ink2", "muted", "line", "grid", "axis", "accent", "good", "ref", "frontier", "p", "s", "exotic"].forEach(function (n) { c[n] = P.color(n); });
    c.flux = P.color("--flux"); c.steel = P.color("--steel");
    SANS = P.theme().sans;
    return c;
  }
  function massT(t) {                                /* tonnes, SI-prefixed in the tonne */
    if (!(t > 0)) return "0 t";
    if (t >= 1e9) return fmt(t / 1e9, 3) + " Gt";
    if (t >= 1e6) return fmt(t / 1e6, 3) + " Mt";
    if (t >= 1e3) return fmt(t / 1e3, 3) + " kt";
    return fmt(t, 3) + " t";
  }

  /* ---- missing orders of magnitude (FRONTIER §1 and §F1) ------------------------------ */
  var GAPS = [
    { item: "PEGS on-site gravity gradiometer", badge: "f", missing: 5, href: "#pegs", note: "needs 1e-15/√Hz; best demonstrated 1e-10/√Hz (TOBA Phase-II)" },
    { item: "Building-scale active metabarrier", badge: "f", missing: 4.4, href: "#metabarrier", note: "at 1 Hz: 1.8 Mt of resonating mass at v_s 100 m/s; field tests start at 30 Hz" },
    { item: "Diamagnetic levitation", badge: "v", badgeText: "physics", missing: 3.3, href: "#relabel-title", note: "179 Pa at 1 T, an upper bound: 3.3 orders below EMS at the same field" },
    { item: "QZS isolation at building scale", badge: "r", missing: 2.7, href: "#relabel-title", note: "0.5 m of stroke against millimetre zero-power stages (0 at instrument scale)" },
    { item: "Structure-scale cloak or concentrator", badge: "f", missing: 2, href: "#cloak", note: "a 1 m shell against centimetre laboratory cylinders" },
    { item: "SELENE lunar hatch and cargo rail", badge: "f", missing: 2, href: "#selene", note: "5 m of hatch travel against about 5 cm FLOAT prototypes [UNCERTAIN]" },
    { item: "Ambient-pressure RTSC suspension", badge: "f", missing: 0.4, href: "#rtsc", note: "Tc ≈ 358 K needed against about 138 K; the pressure ceiling stays rail-limited" },
    { item: "Trapped-field bulk magnets", badge: "r", missing: 0, href: "#relabel-title", note: "the pressure cap (≤ 2.1 MPa against steel) is physics, not a missing capability" },
    { item: "NV-centre gap sensing", badge: null, badgeText: "documented only", missing: 0, href: "#relabel-title", note: "over-specified for gap control" }
  ];
  function hrefFor(item) {
    var s = String(item).toLowerCase();
    if (/pegs/.test(s)) return "#pegs";
    if (/metabarrier/.test(s)) return "#metabarrier";
    if (/rtsc|superconduct/.test(s)) return "#rtsc";
    if (/cloak|concentrator/.test(s) && !/lab/.test(s)) return "#cloak";
    if (/selene|lunar/.test(s)) return "#selene";
    return "#relabel-title";
  }
  function badgeKey(b) {
    var s = String(b || "");
    if (s.indexOf("🔭") >= 0) return "f";
    if (s.indexOf("✅") >= 0) return "v";
    if (s.indexOf("🧪") >= 0) return "r";
    if (s.indexOf("🔶") >= 0) return "s";
    if (s.indexOf("🗺") >= 0) return "m";
    return null;
  }
  function renderGaps() {
    var list = $("gap-list");
    if (!list) return;
    var R = window.EXO_RUN, rows = GAPS, src = $("gap-source");
    if (R && !R.placeholder && R.frontier && R.frontier.length) {
      rows = R.frontier.map(function (c) {
        return { item: c.item, badge: badgeKey(c.badge), badgeText: badgeKey(c.badge) ? (/physics/.test(c.badge) ? "physics" : "") : String(c.badge || "–"),
          missing: typeof c.missing_orders === "number" ? c.missing_orders : 0, href: hrefFor(c.item), note: c.note || "" };
      });
      if (src) src.textContent = "Values from the generated frontier cards (EXO_RUN, seed " + R.seed + "), the frontier module's own metadata.";
    }
    rows = rows.slice().sort(function (a, b) { return b.missing - a.missing; });
    clear(list);
    rows.forEach(function (r) {
      var li = el("li", "", list), nm = el("span", "gap-name", li);
      var a = el("a", "", nm, r.item);
      a.href = r.href;
      if (r.badge) nm.appendChild(UI.badge(r.badge, r.badgeText || ""));
      else if (r.badgeText) el("span", "badge badge--m", nm, r.badgeText);
      var tr = el("span", "gap-track", li), fill = el("span", "gap-fill", tr);
      fill.style.width = clamp(r.missing / 6, 0, 1) * 100 + "%";
      if (!(r.missing > 0)) fill.setAttribute("data-zero", "");
      tr.setAttribute("role", "img");
      tr.setAttribute("aria-label", fmt(r.missing, 2) + " orders of magnitude missing");
      el("span", "gap-val", li, r.missing > 0 ? fmt(r.missing, 2) + " orders" : "none");
      el("span", "gap-note", li, r.note);
    });
  }

  /* ---- PEGS (pegs.py; FRONTIER §F4) --------------------------------------------------- */
  var PG = { kappa: 4.6, snr: 5, bw: 0.1, vp: 6.5, horizon: 600, depth: 10, rNet: 22, latency: 3.3, need: 1e-15, best: 1e-10 };
  function m0(mw) { return Math.pow(10, 1.5 * mw + 9.1); }
  function i2(t, mw) {
    var M = m0(mw), th = 2.26e-6 * Math.cbrt(M);
    function q(x) { x = Math.max(x, 0); return x * x * x * x / 24; }
    return M / (th * th) * (q(t) - 2 * q(t - th) + q(t - 2 * th));
  }
  function snrAt(t, asd, mw, rM) { return PG.kappa * G_N * i2(t, mw) / Math.pow(rM, 5) / (asd * Math.sqrt(PG.bw)); }
  function tDetect(asd, mw, rM) {
    var need = PG.snr * asd * Math.sqrt(PG.bw) * Math.pow(rM, 5) / (PG.kappa * G_N), lo = 0, hi = PG.horizon;
    if (i2(hi, mw) < need) return Infinity;
    for (var k = 0; k < 64; k++) { var mid = 0.5 * (lo + hi); if (i2(mid, mw) >= need) hi = mid; else lo = mid; }
    return hi;
  }
  var pg = { asd: 1e-15, mw: 7, r: 100e3 };
  function pegsCalc() {
    var rKm = Math.hypot(pg.r / 1e3, PG.depth), tdet = tDetect(pg.asd, pg.mw, rKm * 1e3);
    return { rKm: rKm, tdet: tdet, tp: rKm / PG.vp, tnet: PG.rNet / PG.vp + PG.latency };
  }
  function pegsReadouts() {
    var c = pegsCalc(), box = $("pegs-readouts");
    put(box, "tdet", isFinite(c.tdet) ? fmtSI(c.tdet, "s", 3) : "not within 600 s");
    put(box, "tnet", fmtSI(c.tnet, "s", 3));
    put(box, "tp", fmtSI(c.tp, "s", 3));
    var adv = c.tnet - c.tdet;
    put(box, "gain", isFinite(adv) ? (adv > 0 ? "+" + fmtSI(adv, "s", 3) : "0 s") : "0 s");
    put(box, "gainNote", !isFinite(c.tdet) ? "never detected at this noise" : adv > 0 ? "vs on-site P: " + (c.tp - c.tdet > 0 ? "+" : "") + fmtSI(c.tp - c.tdet, "s", 3)
      : "the network is first by " + fmtSI(-adv, "s", 3) + (c.tdet > c.tp ? "; after P the model no longer applies" : ""));
    drawMeter();
  }
  var meterCanvas = null;
  function drawMeter() {
    meterCanvas = meterCanvas || $("pegs-meter");
    if (!meterCanvas) return;
    var w = Math.max(240, Math.round(meterCanvas.clientWidth || 400)), h = 78, cv = P.setupCanvas(meterCanvas, w, h), ctx = cv.ctx, col = cols();
    var x0 = 12, x1 = w - 12, lo = -19, hi = -9;
    function X(a) { return x0 + (log10(a) - lo) / (hi - lo) * (x1 - x0); }
    ctx.fillStyle = col.deep; ctx.fillRect(0, 0, w, h);
    var y = 44;
    ctx.fillStyle = col.grid; ctx.fillRect(x0, y - 3, x1 - x0, 6);
    ctx.fillStyle = P.alpha(col.frontier, 0.35); ctx.fillRect(X(PG.need), y - 3, X(PG.best) - X(PG.need), 6);
    ctx.strokeStyle = col.frontier; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(X(PG.need), y - 10); ctx.lineTo(X(PG.need), y - 16); ctx.lineTo(X(PG.best), y - 16); ctx.lineTo(X(PG.best), y - 10); ctx.stroke();
    text(ctx, "5 orders short", (X(PG.need) + X(PG.best)) / 2, y - 20, { align: "center", size: 11, weight: "700", color: col.ink });
    for (var e = lo; e <= hi; e++) { ctx.fillStyle = col.muted; ctx.fillRect(X(Math.pow(10, e)) - 0.5, y + 4, 1, 4); }
    text(ctx, "1e−19", x0, h - 6, { size: 10 }); text(ctx, "1e−9 /√Hz", x1, h - 6, { size: 10, align: "right" });
    text(ctx, "need 1e−15", X(PG.need), y + 20, { size: 10, align: "center", color: col.ink2 });
    text(ctx, "best 1e−10", X(PG.best), y + 20, { size: 10, align: "center", color: col.ink2 });
    var xs = X(pg.asd);
    ctx.fillStyle = col.accent; ctx.beginPath(); ctx.moveTo(xs, y - 5); ctx.lineTo(xs - 5, y - 12); ctx.lineTo(xs + 5, y - 12); ctx.closePath(); ctx.fill();
    ctx.fillRect(xs - 1, y - 5, 2, 10);
  }
  function initPegs() {
    var ctl = $("pegs-controls"), stage = $("pegs-stage"), canvas = $("pegs-canvas"), chip = $("pegs-chip"), btn = $("pegs-play");
    if (!ctl || !stage || !canvas) return;
    UI.bindSliders(ctl, function (v) { pg.asd = v.asd; pg.mw = v.mw; pg.r = v.r; pegsReadouts(); drawAt(cur); });
    var cv = null, col = cols(), cur = 0, LOOP = 9;
    function layout() {
      var w = Math.max(260, Math.round(stage.clientWidth || 500)), h = Math.round(clamp(w * 0.56, 250, 330));
      cv = P.setupCanvas(canvas, w, h);
    }
    function drawAt(u) {
      if (!cv) return;
      cur = u;
      var c = pegsCalc(), ctx = cv.ctx, w = cv.w, h = cv.h;
      var tEnd = Math.max(c.tp, c.tnet, isFinite(c.tdet) ? c.tdet : 0) * 1.12 + 1.5;
      var t = (u % LOOP) / LOOP * tEnd;
      ctx.setTransform(cv.dpr, 0, 0, cv.dpr, 0, 0);
      ctx.fillStyle = col.deep; ctx.fillRect(0, 0, w, h);
      var surf = h * 0.42, left = 30, right = w - 40, kmX = (right - left) / (pg.r / 1e3), depthPx = Math.min(h * 0.4, 70);
      var sx = left, sy = surf + depthPx;
      /* ground */
      ctx.fillStyle = P.alpha(col.line, 0.3); ctx.fillRect(0, surf, w, h - surf);
      ctx.strokeStyle = col.axis; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(0, surf + 0.5); ctx.lineTo(w, surf + 0.5); ctx.stroke();
      /* gravity change: instantaneous everywhere (a faint wash once the rupture starts) */
      if (t > 0) { ctx.fillStyle = P.alpha(col.frontier, 0.06 + 0.06 * Math.min(1, t / 5)); ctx.fillRect(0, 0, w, h); }
      /* P front: radius v_P·t in km, drawn with a vertical exaggeration (not to scale) */
      var rP = PG.vp * t;
      ctx.save(); ctx.beginPath(); ctx.rect(0, surf, w, h - surf); ctx.clip();
      ctx.strokeStyle = col.p; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.ellipse(sx, sy, Math.max(0.1, rP * kmX), Math.max(0.1, rP * depthPx / PG.depth), 0, 0, 2 * Math.PI); ctx.stroke();
      ctx.restore();
      /* source */
      ctx.fillStyle = col.s; ctx.beginPath(); ctx.arc(sx, sy, 6, 0, 2 * Math.PI); ctx.fill();
      text(ctx, "Mw " + fmt(pg.mw, 2) + " · " + PG.depth + " km deep", sx + 10, sy + 4, { size: 10.5, color: col.ink2 });
      /* dense network station, 22 km hypocentral */
      var netEpi = Math.sqrt(PG.rNet * PG.rNet - PG.depth * PG.depth), nx = left + netEpi * kmX;
      var netOn = t >= c.tnet;
      ctx.fillStyle = netOn ? col.accent : col.steel; ctx.beginPath(); ctx.moveTo(nx, surf - 12); ctx.lineTo(nx - 6, surf); ctx.lineTo(nx + 6, surf); ctx.closePath(); ctx.fill();
      text(ctx, netOn ? "network alert" : "network station", nx, surf - 16, { align: "center", size: 10.5, color: netOn ? col.ink : col.muted, weight: netOn ? "600" : "400" });
      /* site with its gradiometer gauge */
      var tx = right, snr = t > 0 ? snrAt(t, pg.asd, pg.mw, c.rKm * 1e3) / PG.snr : 0, det = isFinite(c.tdet) && t >= c.tdet, pOn = t >= c.tp;
      ctx.fillStyle = pOn ? col.p : col.ink2; ctx.fillRect(tx - 8, surf - 18, 16, 18);
      text(ctx, "site", tx, surf - 22, { align: "center", size: 10.5, color: col.ink2, weight: "600" });
      var gx = tx - 26, gh = 60, gy = surf - 30 - gh;
      ctx.strokeStyle = col.line; ctx.lineWidth = 1; ctx.strokeRect(gx, gy, 12, gh);
      var fillH = gh * clamp(snr, 0, 1);
      ctx.fillStyle = det ? col.frontier : P.alpha(col.frontier, 0.55); ctx.fillRect(gx + 1, gy + gh - fillH, 10, fillH);
      text(ctx, det ? "gradiometer: detected" : "gradiometer SNR", gx - 4, gy + 10, { align: "right", size: 10.5, color: det ? col.ink : col.muted, weight: det ? "600" : "400" });
      /* scale bar and clock */
      text(ctx, fmt(pg.r / 1e3, 3) + " km (horizontal) · depth exaggerated", left, h - 10, { size: 10 });
      text(ctx, "t = " + fmt(t, 3) + " s", w - 10, h - 10, { align: "right", size: 10.5, color: col.ink2 });
      if (chip) {
        var msg = t <= 0 ? "Rupture starts." : det && !netOn ? "The gradiometer detects before the network alert." :
          netOn && !det ? "The dense network alerts first; the gradiometer has not reached SNR 5." :
          pOn ? "P reaches the site." : "The gravity change is everywhere at once; the P front travels at 6.5 km/s.";
        if (chip.textContent !== msg) chip.textContent = msg;
      }
    }
    var loop = P.loop(function (dt, tl) { drawAt(tl); }, { el: stage, still: function () { setBtn(false); drawAt(LOOP * 0.85); } });
    function setBtn(on) { if (btn) { btn.textContent = on ? "Pause" : "Play"; btn.setAttribute("aria-pressed", on ? "true" : "false"); } }
    if (btn) btn.addEventListener("click", function () { if (loop.running()) { loop.stop(); setBtn(false); } else { loop.reset(cur); setBtn(loop.start(true)); } });
    layout();
    P.onTheme(function () { col = cols(); drawAt(cur); drawMeter(); });
    if (window.ResizeObserver) { var lw = 0; new ResizeObserver(function () { var w = stage.clientWidth; if (Math.abs(w - lw) >= 1) { lw = w; layout(); drawAt(cur); drawMeter(); } }).observe(stage); }
    else window.addEventListener("resize", function () { layout(); drawAt(cur); drawMeter(); });
    pegsReadouts();
    if (loop.start()) setBtn(true); else { setBtn(false); drawAt(LOOP * 0.85); }
  }

  /* ---- metabarrier (metabarrier.py; FRONTIER §F5) -------------------------------------- */
  var MB = { rho: 1800, fTest: 30, unitKg: 5000 }, mb = { f: 1, vs: 100 }, mbChart = null;
  function mbMassKg(f, vs) { return MB.rho * Math.pow(vs / f, 3); }
  function mbSpec() {
    function tf(vs) { return function (f) { return mbMassKg(f, vs) / 1000; }; }
    return {
      kind: "loglog", title: "Resonating mass against the target frequency",
      desc: "Log-log chart: the resonating mass falls as the cube of frequency; lines for soil shear-wave speeds of 100 and 200 metres per second and the selected one; field tests exist only above 30 hertz.",
      x: { label: "target frequency", unit: "Hz", min: 0.3, max: 60 },
      y: { label: "resonating mass", min: 1, max: 1e10, format: massT },
      series: [
        { key: "v100", label: "v_s 100 m/s", color: "muted", fn: tf(100), samples: 80, endLabel: "100 m/s" },
        { key: "v200", label: "v_s 200 m/s", color: "muted", dash: [5, 4], fn: tf(200), samples: 80, endLabel: "200 m/s" },
        { key: "sel", label: "selected v_s " + fmt(mb.vs, 3) + " m/s", color: "frontier", fn: tf(mb.vs), samples: 80 }
      ],
      bands: [{ axis: "x", from: MB.fTest, to: 60, label: "field tests", color: "good", alpha: 0.12 }],
      points: [
        { x: 1, y: mbMassKg(1, 100) / 1000, label: "1.8 Mt", color: "ink2" },
        { x: 1, y: mbMassKg(1, 200) / 1000, label: "14.4 Mt", color: "ink2" },
        { x: mb.f, y: mbMassKg(mb.f, mb.vs) / 1000, label: Math.abs(mb.f - 1) < 0.03 && (mb.vs === 100 || mb.vs === 200) ? "" : massT(mbMassKg(mb.f, mb.vs) / 1000), color: "frontier" }
      ],
      height: 270, table: false
    };
  }
  function mbUpdate() {
    var m = mbMassKg(mb.f, mb.vs), box = $("meta-readouts");
    put(box, "mass", massT(m / 1000));
    put(box, "units", fmt(m / MB.unitKg, 3));
    put(box, "lam", fmtSI(mb.vs / mb.f, "m", 3));
    put(box, "miss", fmt(Math.max(0, 3 * log10(MB.fTest / mb.f)), 2) + " orders");
    if (mbChart) mbChart.update(mbSpec());
  }
  function initMeta() {
    var ctl = $("meta-controls"), ch = $("meta-chart");
    if (!ctl) return;
    if (ch) mbChart = P.chart(ch, mbSpec());
    UI.bindSliders(ctl, function (v) { mb.f = v.f; mb.vs = v.vs; mbUpdate(); });
  }

  /* ---- RTSC (rtsc.py; FRONTIER §F2) ----------------------------------------------------- */
  var RT = { b0: 1.2, k: 40.85, tcRebco: 92, tLn2: 77, tAmb: 300, n: 1.5, tcBest: 138 };
  RT.etaToday = 32.9e3 / (4 * 492e-4 * RT.b0 * RT.b0 / MU0);
  var rt = { h: 0.010 }, rtChart = null;
  function rtCeil(h) { return RT.b0 * RT.b0 / MU0 * Math.exp(-2 * RT.k * h); }
  function rtJ(tc) {
    var b = Math.max(0, 1 - RT.tAmb / tc), b0 = 1 - RT.tLn2 / RT.tcRebco;
    return Math.pow(b, RT.n) / Math.pow(b0, RT.n);
  }
  function rtSpec() {
    var ceil = rtCeil(rt.h) / 1e6;
    function fn(eta) { return function (tc) { return Math.min(eta * rtJ(tc), 1) * ceil; }; }
    var need = RT.tAmb * RT.tcRebco / RT.tLn2;
    return {
      kind: "line", title: "Levitation pressure at 300 K against the critical temperature",
      desc: "Line chart: pressure is zero below 300 kelvin, rises with Tc and saturates at the rail-limited ceiling for shielding efficiencies 0.15, 0.5 and 1.",
      x: { label: "critical temperature Tc", unit: "K", min: 90, max: 600 }, y: { label: "pressure", unit: "MPa", min: 0, max: Math.max(0.2, ceil * 1.15) },
      series: [
        { key: "e15", label: "η 0.15 (today's pinning)", color: "muted", fn: fn(0.15), samples: 200 },
        { key: "e50", label: "η 0.5", color: "ink2", dash: [5, 4], fn: fn(0.5), samples: 200 },
        { key: "e100", label: "η 1 (ideal image)", color: "frontier", fn: fn(1), samples: 200 }
      ],
      refs: [
        { axis: "y", value: ceil, label: "ceiling " + fmt(ceil, 3) + " MPa at " + fmtSI(rt.h, "m", 3), color: "ref", dash: [6, 3], align: "left" },
        { axis: "x", value: RT.tcBest, label: "best ambient ≈ 138 K", color: "muted", dash: [2, 4] },
        { axis: "x", value: need, label: "needed ≈ " + fmt(need, 3) + " K", color: "frontier", dash: [2, 4], row: 1 }
      ],
      height: 260, table: false
    };
  }
  function rtUpdate() {
    var box = $("rtsc-readouts"), c = rtCeil(rt.h);
    put(box, "ceil", fmtSI(c, "Pa", 3));
    put(box, "today", fmtSI(RT.etaToday * c, "Pa", 3));
    put(box, "tc", fmt(RT.tAmb * RT.tcRebco / RT.tLn2, 4) + " K");
    if (rtChart) rtChart.update(rtSpec());
  }
  function initRtsc() {
    var ctl = $("rtsc-controls"), ch = $("rtsc-chart");
    if (!ctl) return;
    if (ch) rtChart = P.chart(ch, rtSpec());
    UI.bindSliders(ctl, function (v) { rt.h = v.h; rtUpdate(); });
  }

  /* ---- cloak (cloak.py; FRONTIER §F8) ------------------------------------------------- */
  var ck = { ratio: 1.2, r2: 1 }, ckChart = null;
  function muFm(q) { return (q * q + 1) / (q * q - 1); }
  function ckSpec() {
    return {
      kind: "semilog", title: "Ferromagnetic shell permeability for an ideal cloak",
      desc: "Semilog chart: the required relative permeability falls from about 20 near a thin shell to under 2 at a radius ratio of 2.",
      x: { label: "radius ratio R₂/R₁", min: 1.05, max: 4 }, y: { label: "μ_FM", min: 1, max: 30 },
      series: [{ key: "mu", label: "μ_FM = (R₂² + R₁²)/(R₂² − R₁²)", color: "frontier", fn: muFm, samples: 200 }],
      points: [{ x: 1.2, y: muFm(1.2), label: "5.5 at 1.2", color: "ink2" }, { x: 2, y: muFm(2), label: "1.67 at 2", color: "ink2" },
        { x: ck.ratio, y: muFm(ck.ratio), label: Math.abs(ck.ratio - 1.2) < 0.03 || Math.abs(ck.ratio - 2) < 0.05 ? "" : fmt(muFm(ck.ratio), 3), color: "frontier" }],
      height: 250, table: false
    };
  }
  function drawCloak() {
    var cnv = $("cloak-canvas");
    if (!cnv) return;
    var w = Math.max(220, Math.round(cnv.clientWidth || 300)), h = 230, cv = P.setupCanvas(cnv, w, h), ctx = cv.ctx, col = cols();
    ctx.fillStyle = col.deep; ctx.fillRect(0, 0, w, h);
    var cx = w / 2, cy = h / 2, R2 = Math.min(w, h) * 0.34, R1 = R2 / ck.ratio;
    ctx.fillStyle = P.alpha(col.steel, 0.25); ctx.beginPath(); ctx.arc(cx, cy, R2, 0, 2 * Math.PI); ctx.fill();
    ctx.fillStyle = col.deep; ctx.beginPath(); ctx.arc(cx, cy, R1, 0, 2 * Math.PI); ctx.fill();
    ctx.strokeStyle = col.frontier; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(cx, cy, R1, 0, 2 * Math.PI); ctx.stroke();
    ctx.strokeStyle = col.steel; ctx.beginPath(); ctx.arc(cx, cy, R2, 0, 2 * Math.PI); ctx.stroke();
    /* field lines: uniform outside; inside R₂ the transformation map r' = R₁ + r·(R₂ − R₁)/R₂ bends them round the core */
    ctx.strokeStyle = col.flux; ctx.lineWidth = 1.2;
    for (var k = -7; k <= 7; k++) {
      var y0 = k * h / 15;
      ctx.beginPath();
      for (var x = -w / 2; x <= w / 2; x += 2) {
        var r = Math.hypot(x, y0), px = x, py = y0;
        if (r < R2 && r > 0) { var rr = R1 + r * (R2 - R1) / R2; px = x * rr / r; py = y0 * rr / r; }
        if (x === -w / 2) ctx.moveTo(cx + px, cy + py); else ctx.lineTo(cx + px, cy + py);
      }
      ctx.stroke();
    }
    text(ctx, "field B₀ →", 8, 14, { size: 10.5, color: col.ink2 });
    text(ctx, "no field inside", cx, cy + 4, { align: "center", size: 10, color: col.muted });
    text(ctx, "ideal cloak, schematic", w - 8, h - 8, { align: "right", size: 10 });
  }
  function ckUpdate() {
    var box = $("cloak-readouts"), miss = Math.max(0, log10(ck.r2 / 0.01));
    put(box, "mu", fmt(muFm(ck.ratio), 3));
    put(box, "tier", ck.r2 <= 0.01 ? "🧪 research" : "🔭 frontier");
    put(box, "tierNote", "missing orders log₁₀(R₂/1 cm) = " + fmt(miss, 2));
    if (ckChart) ckChart.update(ckSpec());
    drawCloak();
  }
  function initCloak() {
    var ctl = $("cloak-controls"), ch = $("cloak-chart");
    if (!ctl) return;
    if (ch) ckChart = P.chart(ch, ckSpec());
    UI.bindSliders(ctl, function (v) { ck.ratio = v.ratio; ck.r2 = v.r2; ckUpdate(); });
    P.onTheme(drawCloak);
    var cnv = $("cloak-canvas");
    if (cnv && window.ResizeObserver) new ResizeObserver(drawCloak).observe(cnv);
  }

  /* ---- SELENE (lunar.py; FRONTIER §F7) --------------------------------------------------- */
  var sl = { T: 40, eps: 1 }, slChart = null;
  function slSpec() {
    return {
      kind: "semilog", title: "Radiated flux ε·σ·T⁴ against temperature",
      desc: "Semilog chart of the radiated flux of an ideal radiator against its temperature, with the 25 and 40 kelvin shadowed-crater marks.",
      x: { label: "temperature", unit: "K", min: 20, max: 80 }, y: { label: "radiated flux", unit: "W/m²", min: 0.005, max: 3 },
      series: [{ key: "q", label: "q = ε·σ·T⁴, ε " + fmt(sl.eps, 2), color: "frontier", fn: function (T) { return sl.eps * SIGMA_SB * Math.pow(T, 4); }, samples: 120 }],
      points: [
        { x: 25, y: SIGMA_SB * Math.pow(25, 4), label: "0.022 W/m² at 25 K", color: "ink2" },
        { x: 40, y: SIGMA_SB * Math.pow(40, 4), label: "0.145 W/m² at 40 K", color: "ink2" },
        { x: sl.T, y: sl.eps * SIGMA_SB * Math.pow(sl.T, 4), label: "", color: "frontier" }
      ],
      height: 240, table: false
    };
  }
  function slUpdate() {
    var q = sl.eps * SIGMA_SB * Math.pow(sl.T, 4), box = $("selene-readouts");
    put(box, "q", fmt(q, 3) + " W/m²");
    put(box, "area", fmt(1 / q, 3) + " m²/W");
    if (slChart) slChart.update(slSpec());
  }
  function initSelene() {
    var ctl = $("selene-controls"), ch = $("selene-chart");
    if (!ctl) return;
    if (ch) slChart = P.chart(ch, slSpec());
    UI.bindSliders(ctl, function (v) { sl.T = v.T; sl.eps = v.eps; slUpdate(); });
    /* the scaling table, recomputed from the laws it states */
    var rows = doc.querySelectorAll("#selene-table tr td.num");
    if (rows.length >= 4) {
      rows[0].textContent = "×" + fmt(Math.sqrt(G_MOON / G0), 3) + " (" + fmt(Math.sqrt(2 * G0 / 0.01), 3) + " → " + fmt(Math.sqrt(2 * G_MOON / 0.01), 3) + " rad/s at 10 mm)";
      rows[1].textContent = "×" + fmt(G0 / G_MOON, 3);
      rows[2].textContent = "×" + fmt(Math.sqrt(G0 / G_MOON), 3);
      rows[3].textContent = "≈ " + fmt(100e3 * G_MOON / 53e3, 2) + " m²";
    }
  }

  function init() {
    cols();
    renderGaps();
    initPegs();
    initMeta();
    initRtsc();
    initCloak();
    initSelene();
    if (UI.renderBanners) UI.renderBanners(doc);
  }
  if (doc.readyState === "loading") doc.addEventListener("DOMContentLoaded", init); else init();
})();
