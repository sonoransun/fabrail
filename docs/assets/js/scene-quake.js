/* fabrail quake page scenes (plan §3b). Owner: W3 (docs/quake.html and this file).
 *
 * Everything on quake.html that moves or plots, from the shared framework only
 * (FabrailFormulas closed forms, FabrailPlot charts and loop, FabrailUI sliders and
 * labels); no external resources:
 *   1. the plan-view map: P and S wavefronts leaving a hypocentre at depth, the house,
 *      the warning rings and a timeline strip with the lead bracket (PHYSICS §20);
 *   2. lead time against hypocentral distance with the 40.8 km crossover and an assumed
 *      crustal-velocity band (FAB-SEI-012);
 *   3. the detector: floor acceleration, STA/LTA, Pd and τc and the seismic state
 *      machine, replayed from the recorded seed-42 run in RUN when it exists, otherwise
 *      from an illustrative record generated here and labelled as such;
 *   4. the fused alert (local pick and a synthetic network alert) as a timeline;
 *   5. class action timelines (Gantt) from preset closed forms (PHYSICS §21);
 *   6. the land-first comparison of lateral capacity, floor shaking and LSM authority.
 *
 * RUN contract (docs/assets/data/run.js, schema fabrail.docs.run/1, written by
 * scripts/update_docs_run.py). The detector replays the first RUN.runs entry whose key
 * is in RECORDED_KEYS, else the first whose scenario starts with "pwave_". It reads
 *   channels  floor_az_mps2 (required), floor_ax_mps2, pd_m, tau_c_s, level,
 *             seismic_state, and sta_lta (optional: the detector's own ratio, which is
 *             not a recorder channel in this build; without it the STA/LTA panel stays
 *             illustrative and says so);
 *   events    Freeze, SeismicState (a = new state), Protect (a = level, b = seconds to
 *             the local S-ETA), plant:landed / Landed, plant:locked;
 *   info      optional additive object with the scenario info (magnitude, r_hyp_km,
 *             depth_km, t_p_s, t_s_s, pd_target_m); without it the catalog constants in
 *             CATALOG (python/fabrail/scenarios.py _PS_CASES, _T_P_S) are used.
 * While RUN.placeholder is true the page renders the illustrative record and quotes no
 * simulator number.
 *
 * Deep link: #r=<R_hyp km>&d=<depth km>&m=<magnitude>&t=<s after origin> opens the map
 * paused at that moment (handy for screenshots).
 */
(function (root) {
  "use strict";
  var doc = root.document;
  var F = root.FabrailFormulas, P = root.FabrailPlot, UI = root.FabrailUI;
  if (!doc || !F || !P || !UI) return;
  var C = F.CONST, fmt = UI.fmt, fmtSI = UI.fmtSI;
  var MINUS = "−";
  var VP = C.VP_KMPS, VS = C.VS_KMPS;

  function $(id) { return doc.getElementById(id); }
  function clamp(x, lo, hi) { return Math.min(hi, Math.max(lo, x)); }
  function num(v, dflt) { return typeof v === "number" && isFinite(v) ? v : dflt; }
  function sec(v, d) { return fmt(v, d === undefined ? 3 : d) + " s"; }
  function signed(v, d) { return (v >= 0 ? "+" : MINUS) + fmt(Math.abs(v), d === undefined ? 3 : d) + " s"; }

  /* ---- detector and policy constants ----------------------------------------------
     Mirrors of fabrail_params presets, identical in every class
     (control.seismic.detector.*, control.seismic.fusion.*; PHYSICS §20). */
  var DET = {
    FS_HZ: 100,            /* seismic group rate: 500 Hz ODR ÷ aa_decim 5 */
    STA_N: 50,             /* sta_n */
    LTA_N: 3000,           /* lta_n */
    TRIG_ON: 4.0,          /* trig_on */
    TRIG_OFF: 1.5,         /* trig_off */
    WIN_SHORT_S: 1.0,      /* win_short_s: the level is set after this window */
    WIN_LONG_S: 3.0,       /* win_long_s */
    TAUC_L3_S: 0.6,        /* tauc_l3_s */
    PD_L3_M: 0.005,        /* pd_l3_m */
    PD_ALERT_M: 0.0035,    /* pd_alert_m: PROTECT on Pd alone */
    FLOOR_MPS2_SQ: 1.4e-9, /* 1 LSB² of the ADXL355 (±2 g, 20 bit ≈ 3.8 µg) in (m/s²)² */
    /* Representative pick delay: the exact discrete STA/LTA response to a variance
       step K = 16 triggers at sample 13, 0.13 s at 100 Hz (PHYSICS §20; FAB-SEI-003). */
    PICK_DELAY_S: 0.13
  };

  /* Class action inputs not carried by FORMULAS.CLASSES (fabrail_params presets):
     control.levitation.land_speed_mps, control.seismic.policy.lock_after_land and
     control.motion.a_brk_ratio (2.0 in every class). PHYSICS §21. */
  var A_BRK_RATIO = 2.0;
  var LANDING = {
    drawer: { landSpeed: 0.005, lock: false, rest: "onto its travel rollers" },
    door: { landSpeed: 0.005, lock: false, rest: "onto its rollers and releases ASC" },
    wall: { landSpeed: 0.003, lock: true, rest: "and locks" },
    heavy: { landSpeed: 0.002, lock: true, rest: "and locks" }
  };
  var CLASS_KEYS = ["drawer", "door", "wall", "heavy"];

  /* Synthetic P+S catalog cases (python/fabrail/scenarios.py _PS_CASES: magnitude,
     epicentral km, depth km) and the P arrival in every record (_T_P_S: the 30 s
     STA/LTA warm-up plus 5 s). Used for the illustrative record and as the fallback
     when a RUN entry carries no info block. */
  var CATALOG = {
    pwave_m5_r10: { m: 5.0, rEpi: 10, depth: 10 },
    pwave_m7_r20: { m: 7.0, rEpi: 20, depth: 10 },
    pwave_m78_r100: { m: 7.8, rEpi: 100, depth: 10 }
  };
  var T_P_RECORD_S = 35.0;
  var RECORDED_KEYS = ["wall_pwave_m7_r20", "door_pwave_m7_r20", "drawer_pwave_m7_r20", "heavy_pwave_m7_r20"];

  /* Crustal-velocity band for the lead chart (an assumption, not a site model):
     V_p 5.5–6.5 km/s with V_p/V_s 1.65–1.80. The anchors use 6.0 and 3.5 km/s. */
  var BAND = { slow: { vp: 5.5, ratio: 1.8 }, fast: { vp: 6.5, ratio: 1.65 } };

  /* Wu & Kanamori (2005) τc–M regression, M = 3.373·log₁₀τc + 5.787 (the synthetic
     records' τc target; python/fabrail/pwave.py tau_c_wu_kanamori_s). */
  function tauCWuKanamori(m) { return Math.pow(10, (m - 5.787) / 3.373); }

  /* Level matrix and PROTECT rule (PHYSICS §20; FAB-SEI-005). */
  function levelFor(pdM, tauC) {
    var hiT = tauC >= DET.TAUC_L3_S, hiP = pdM >= DET.PD_L3_M;
    return hiT && hiP ? 3 : hiT ? 2 : hiP ? 1 : 0;
  }
  function isProtect(level, pdM) { return level >= 2 || pdM >= DET.PD_ALERT_M; }

  /* ---- the event model (closed forms) --------------------------------------------- */

  var Q = { r: 40, depth: 10, m: 7, tOnsite: C.ONSITE_DECISION_S, tNetwork: C.NETWORK_ALERT_S, network: true };

  function onsiteLead(r, vp, vs, tOn) { return r / vs - r / vp - tOn; }
  function networkLead(r, vs, tNet) { return r / vs - tNet; }

  function classTiming(cls) {
    var c = F.CLASSES[cls];
    var v = Math.max(c.v_open_max_mps, c.v_close_max_mps);
    var aBrk = A_BRK_RATIO * c.a_max_mps2;
    return {
      v: v, aBrk: aBrk,
      tFreeze: v / aBrk,                                   /* constant deceleration stop */
      dFreeze: (v * v) / (2 * aBrk),
      tLand: (c.g_land_m - c.g_star_m) / LANDING[cls].landSpeed   /* hover gap → landing gap */
    };
  }
  var TIMING = {};
  for (var ci = 0; ci < CLASS_KEYS.length; ci++) TIMING[CLASS_KEYS[ci]] = classTiming(CLASS_KEYS[ci]);

  function scenario(q) {
    var r = Math.max(q.r, 0.5), h = Math.min(q.depth, r);
    var s = { r: r, depth: h, rEpi: Math.sqrt(Math.max(0, r * r - h * h)), m: q.m, network: q.network };
    s.tP = r / VP;
    s.tS = r / VS;
    s.sp = s.tS - s.tP;
    s.pd = F.pdWuZhao(q.m, r);
    s.tauc = tauCWuKanamori(q.m);
    s.level = levelFor(s.pd, s.tauc);
    s.localProtect = isProtect(s.level, s.pd);
    s.tPick = s.tP + DET.PICK_DELAY_S;
    s.tDec = s.tP + q.tOnsite;
    s.tNet = q.network ? q.tNetwork : null;
    s.tProtect = null;
    s.source = null;
    if (s.localProtect) { s.tProtect = s.tDec; s.source = "local"; }
    if (s.tNet !== null && (s.tProtect === null || s.tNet < s.tProtect)) { s.tProtect = s.tNet; s.source = "network"; }
    s.tFirst = s.tProtect === null ? s.tPick : Math.min(s.tPick, s.tProtect);
    s.lead = s.tProtect === null ? null : s.tS - s.tProtect;
    s.leadOnsite = onsiteLead(r, VP, VS, q.tOnsite);
    s.leadNetwork = s.tNet === null ? null : networkLead(r, VS, s.tNet);
    s.r0 = q.tOnsite / (1 / VS - 1 / VP);           /* hypocentral radius of zero on-site lead */
    s.rNet = s.tNet === null ? null : VS * s.tNet;   /* inside it the alert arrives after S */
    s.lead20 = onsiteLead(20, VP, VS, q.tOnsite);
    /* per-class action timings for this event */
    s.cls = {};
    var need = 0;
    for (var i = 0; i < CLASS_KEYS.length; i++) {
      var k = CLASS_KEYS[i], tm = TIMING[k], o = { freezeStart: s.tFirst, freezeEnd: s.tFirst + tm.tFreeze };
      if (s.tProtect !== null) {
        o.landStart = Math.max(s.tProtect, o.freezeEnd);
        o.landEnd = o.landStart + tm.tLand;
        o.margin = s.tS - o.landEnd;
        need = Math.max(need, o.landEnd - s.tProtect);
      }
      s.cls[k] = o;
    }
    s.needAll = need;
    s.tEnd = s.tS + 2.5;
    return s;
  }
  var S = scenario(Q);

  /* ---- static numbers in the text ------------------------------------------------- */

  function fillText() {
    var tOn = C.ONSITE_DECISION_S, spk = F.spTime(1);
    var qv = {
      spPerKm: fmt(spk, 3) + " s", onsite: sec(tOn, 2), r0: fmt(tOn / spk, 3) + " km",
      lead20: sec(F.spTime(20) - tOn, 3), vp: fmt(VP, 2) + " km/s", vs: fmt(VS, 2) + " km/s",
      net: sec(C.NETWORK_ALERT_S, 2), crossover: fmt(F.leadCrossoverKm(), 3) + " km",
      crossBand: fmt(BAND.slow.vp * (C.NETWORK_ALERT_S - tOn), 2) + "–" + fmt(BAND.fast.vp * (C.NETWORK_ALERT_S - tOn), 2) + " km",
      pdM5: fmt(F.pdWuZhao(5, 10) * 100, 2) + " cm", tauM5: sec(tauCWuKanamori(5), 2),
      authWall: (F.CLASSES.wall.f_cont_n / F.CLASSES.wall.m_rated_kg / C.G0_MPS2).toFixed(3) + " g",
      authHeavy: (F.CLASSES.heavy.f_cont_n / F.CLASSES.heavy.m_rated_kg / C.G0_MPS2).toFixed(3) + " g"
    };
    var els = doc.querySelectorAll("[data-q]");
    for (var i = 0; i < els.length; i++) {
      var k = els[i].getAttribute("data-q");
      if (qv[k]) els[i].textContent = qv[k];
    }
    var rows = doc.querySelectorAll("#lead-table tr[data-r]");
    for (var j = 0; j < rows.length; j++) {
      var r = parseFloat(rows[j].getAttribute("data-r")), td = rows[j].querySelectorAll("td");
      if (td.length < 5) continue;
      var on = onsiteLead(r, VP, VS, tOn), nw = networkLead(r, VS, C.NETWORK_ALERT_S), best = Math.max(on, nw, 0);
      td[1].textContent = sec(F.spTime(r), 3);
      td[2].textContent = on > 0 ? sec(on, 3) : "0 s";
      td[3].textContent = nw > 0 ? sec(nw, 3) : "< 0";
      td[4].textContent = sec(best, 3) + (Math.abs(r - F.leadCrossoverKm()) < 0.05 ? " (crossover)" : "");
      if (best === 0) { td[4].textContent = ""; var st = doc.createElement("strong"); st.textContent = "0 s"; td[4].appendChild(st); }
    }
  }

  /* ---- readouts and verdict ---------------------------------------------------------- */

  function levelText(s) { return "L" + s.level + (s.localProtect ? " · PROTECT" : " · freeze only"); }

  function mapVerdict(s) {
    if (s.tProtect === null) {
      return { tone: "none", text: "Expected level L" + s.level + " with Pd " + fmt(s.pd * 100, 2) + " cm and τc " + sec(s.tauc, 2) +
        ": the pick means FREEZE only. The carriages stop and hold, nothing lands, and the trigger clears itself (FAB-SEI-010). They land later only if the shaking itself reaches 80 gal." };
    }
    var src = s.source === "network" ? "the synthetic network alert" : "the on-site decision";
    if (s.lead <= 0) {
      return { tone: "late", text: "The S-wave arrives " + sec(-s.lead, 2) + " before " + src + " at " + fmt(s.r, 3) +
        " km: the carriages can only freeze after the pick, if that. This is the near-source zone the design does not claim to cover." };
    }
    var late = [];
    for (var i = 0; i < CLASS_KEYS.length; i++) if (s.cls[CLASS_KEYS[i]].margin < 0) late.push(F.CLASSES[CLASS_KEYS[i]].label);
    if (late.length) {
      return { tone: "late", text: "Lead " + sec(s.lead, 2) + " from " + src + ": enough to freeze, but " + late.join(", ") +
        (late.length > 1 ? " are" : " is") + " still landing when the S-wave arrives. Near the source, early warning buys a stop, not a landing." };
    }
    return { tone: "ok", text: "Lead " + sec(s.lead, 3) + " from " + src + ": enough to freeze and land every class before the S-wave (the slowest needs " +
      sec(s.needAll, 2) + ")." + (s.source === "network" ? " Without a network feed the on-site decision would leave " + sec(Math.max(0, s.leadOnsite), 3) + "." : "") };
  }

  function updateReadouts() {
    var s = S;
    function out(key, text, cls) {
      var els = doc.querySelectorAll('[data-out="' + key + '"]');
      for (var i = 0; i < els.length; i++) {
        els[i].textContent = text;
        if (cls !== undefined) { els[i].classList.remove("is-bad", "is-good"); if (cls) els[i].classList.add(cls); }
      }
    }
    out("tP", sec(s.tP, 3));
    out("tS", sec(s.tS, 3));
    out("sp", "S − P = " + sec(s.sp, 3));
    out("tDec", sec(s.tDec, 3));
    out("decNote", "expected level " + levelText(s));
    out("tNet", s.tNet === null ? "off" : sec(s.tNet, 3));
    if (s.lead === null) {
      out("lead", "none", "is-bad");
      out("leadNote", "no PROTECT: freeze only");
    } else {
      out("lead", s.lead > 0 ? sec(s.lead, 3) : "0 s", s.lead > s.needAll ? "is-good" : "is-bad");
      out("leadNote", s.lead > 0 ? "from the " + (s.source === "network" ? "synthetic network alert" : "on-site decision") : "S arrives before the decision");
    }
    out("pd", fmt(s.pd * 100, 2) + " cm");
    out("tauc", "τc " + sec(s.tauc, 3) + " (Wu–Kanamori)");
    var v = mapVerdict(s), vEl = $("map-verdict");
    if (vEl) { vEl.textContent = v.text; vEl.setAttribute("data-tone", v.tone); }
  }

  /* ---- charts: helpers ------------------------------------------------------------ */

  /* Drop overlapping x-reference labels (estimated widths) so crowded markers keep
     their lines; earlier refs win. */
  function declutterX(refs, x0, x1, widthPx) {
    var placed = [], k = widthPx / Math.max(1e-9, x1 - x0);
    for (var i = 0; i < refs.length; i++) {
      var r = refs[i];
      if (!r.label || r.axis === "y") continue;
      var c = (r.value - x0) * k, half = r.label.length * 3.3 + 5, hit = false;
      for (var j = 0; j < placed.length; j++) if (c - half < placed[j][1] && c + half > placed[j][0]) { hit = true; break; }
      if (hit) r.label = ""; else placed.push([c - half, c + half]);
    }
    return refs;
  }

  /* Stagger x-reference labels over rows (FabrailPlot line charts support ref.row). */
  function staggerX(refs, x0, x1, widthPx) {
    var rows = [], k = widthPx / Math.max(1e-9, x1 - x0);
    for (var i = 0; i < refs.length; i++) {
      var r = refs[i];
      if (!r.label || r.axis === "y") continue;
      var a = (r.value - x0) * k, b = a + r.label.length * 6.4 + 10, row = 0;
      while (rows[row] !== undefined && rows[row] > a) row++;
      rows[row] = b;
      r.row = row;
    }
    return refs;
  }

  function ganttWidth(el, rows) {
    var w = (el && el.clientWidth) || 600, lab = 0;
    for (var i = 0; i < rows.length; i++) lab = Math.max(lab, (rows[i].label || "").length * 6.6);
    return Math.max(80, w - Math.min(lab + 14, w * 0.42) - 14);
  }

  /* Shade between two curves behind a FabrailPlot line chart (drawn under the existing
     pixels with destination-over). Feature-detected: without the hook the band edges,
     which are ordinary series, still show the band. */
  function shadeBetween(chart, loFn, hiFn, colorName) {
    if (!chart || typeof chart._drawXY !== "function") return;
    var base = chart._drawXY;
    chart._drawXY = function (cv, T) {
      base.call(this, cv, T);
      var g = this._geom;
      if (!g || !g.X || !g.Y || !g.xd) return;
      var ctx = cv.ctx, n = 160, i, x;
      ctx.save();
      ctx.globalCompositeOperation = "destination-over";
      ctx.beginPath(); ctx.rect(g.L, g.Tp, g.pw, g.ph); ctx.clip();
      ctx.beginPath();
      for (i = 0; i <= n; i++) {
        x = g.xd[0] + ((g.xd[1] - g.xd[0]) * i) / n;
        if (i) ctx.lineTo(g.X(x), g.Y(hiFn(x))); else ctx.moveTo(g.X(x), g.Y(hiFn(x)));
      }
      for (i = n; i >= 0; i--) {
        x = g.xd[0] + ((g.xd[1] - g.xd[0]) * i) / n;
        ctx.lineTo(g.X(x), g.Y(loFn(x)));
      }
      ctx.closePath();
      ctx.fillStyle = P.alpha(P.color(colorName, chart.el), 0.16);
      ctx.fill();
      ctx.restore();
    };
    chart.render();
  }

  /* ---- lead time chart ------------------------------------------------------------ */

  var leadChart = null;
  function bestLeadFor(r, band) {
    var vp = band ? band.vp : VP, vs = band ? band.vp / band.ratio : VS;
    return Math.max(onsiteLead(r, vp, vs, Q.tOnsite), networkLead(r, vs, Q.tNetwork));
  }
  function leadSpec() {
    var cross = VP * (Q.tNetwork - Q.tOnsite);
    var crossLo = BAND.slow.vp * (Q.tNetwork - Q.tOnsite), crossHi = BAND.fast.vp * (Q.tNetwork - Q.tOnsite);
    var yNow = Q.network ? Math.max(S.leadOnsite, S.leadNetwork) : S.leadOnsite;
    return {
      kind: "line",
      title: "Lead time before the S-wave against hypocentral distance",
      desc: "Line chart. On-site lead rises " + fmt(F.spTime(1), 3) + " s per km after the decision time; the synthetic network lead rises " +
        fmt(1 / VS, 3) + " s per km after the alert latency; they cross at " + fmt(cross, 3) + " km. Below about 20 km neither leaves more than about 1.2 s. A grey band shows the best lead over an assumed crustal velocity range.",
      x: { label: "hypocentral distance", unit: "km", min: 0, max: 200 },
      y: { label: "lead before S", unit: "s", min: -4, max: 60 },
      series: [
        { key: "best", label: "best lead (the larger)", color: P.alpha(P.color("ink"), 0.28), width: 7, fn: function (r) { return bestLeadFor(r, null); }, samples: 201 },
        { key: "onsite", label: "on-site pick", color: "p", fn: function (r) { return onsiteLead(r, VP, VS, Q.tOnsite); }, samples: 201 },
        { key: "network", label: "network alert (synthetic)", color: "ink2", dash: [6, 4], fn: function (r) { return networkLead(r, VS, Q.tNetwork); }, samples: 201 },
        { key: "slow", label: "band edge: Vp 5.5 km/s, Vp/Vs 1.80", color: "muted", dash: [2, 3], width: 1, fn: function (r) { return bestLeadFor(r, BAND.slow); }, samples: 201 },
        { key: "fast", label: "band edge: Vp 6.5 km/s, Vp/Vs 1.65", color: "muted", dash: [2, 3], width: 1, fn: function (r) { return bestLeadFor(r, BAND.fast); }, samples: 201 }
      ],
      refs: [
        { axis: "y", value: 0, color: "axis", dash: [] },
        { axis: "x", value: cross, label: "crossover " + fmt(cross, 3) + " km", color: "muted", dash: [2, 4], row: 1 }
      ],
      bands: [
        { axis: "x", from: 0, to: 20, color: "s", alpha: 0.08, label: "< 20 km: no useful warning" },
        { axis: "x", from: crossLo, to: crossHi, color: "muted", alpha: 0.12 }
      ],
      points: [{ x: S.r, y: clamp(yNow, -4, 60), label: "this event: " + (yNow > 0 ? sec(yNow, 2) : "0 s"), color: "accent" }],
      height: 320, table: true, tableRows: 21
    };
  }
  function buildLeadChart() {
    var el = $("lead-chart");
    if (!el) return;
    leadChart = P.chart(el, leadSpec());
    shadeBetween(leadChart, function (r) { return bestLeadFor(r, BAND.fast); }, function (r) { return bestLeadFor(r, BAND.slow); }, "ink2");
    P.onTheme(function () { if (leadChart) leadChart.update(leadSpec()); });
  }

  /* ---- fused alert timeline ------------------------------------------------------- */

  var fusionChart = null;
  function fusionSpec() {
    var s = S, t1 = s.tEnd, rows = [];
    rows.push({ label: "Local: Pd, τc window", spans: [{ from: s.tPick, to: Math.max(s.tDec, s.tPick + 0.02), color: "p", label: "" }] });
    rows.push({ label: "Local: decision", spans: [{ from: s.tDec, to: Math.max(t1, s.tDec + 0.05), color: "p", style: "hatch",
      label: "level L" + s.level + (s.localProtect ? ", PROTECT" : ", freeze only") }] });
    rows.push({ label: s.tNet === null ? "Network alert (off)" : "Network alert (synthetic)",
      spans: s.tNet === null ? [] : [{ from: s.tNet, to: Math.max(t1, s.tNet + 0.05), color: "ink2", style: "hatch", label: "S-ETA " + sec(s.tS, 3) + " after origin" }] });
    var tTrigEnd = s.tProtect === null ? s.tS : s.tProtect;
    rows.push({ label: "Fused: TRIGGERED, freeze", spans: s.tPick < tTrigEnd ? [{ from: s.tPick, to: tTrigEnd, color: "p", label: "" }] : [] });
    rows.push({ label: "Fused: PROTECT", spans: s.tProtect !== null && s.tProtect < s.tS ? [{ from: s.tProtect, to: s.tS, color: "ink2", label: "" }] : [] });
    rows.push({ label: s.tProtect === null ? "Fused: SHAKING if PGA ≥ 80 gal" : "Fused: SHAKING", spans: [{ from: s.tS, to: t1, color: "s", label: "" }] });
    var refs = [
      { axis: "x", value: s.tP, label: "P", color: "p", dash: [4, 3] },
      { axis: "x", value: s.tS, label: "S", color: "s", dash: [4, 3] }
    ];
    if (s.tNet !== null) refs.push({ axis: "x", value: s.tNet, label: "alert", color: "ink2", dash: [2, 3] });
    refs.push({ axis: "x", value: 0, label: "origin", color: "muted", dash: [2, 3] });
    declutterX(refs, 0, t1, ganttWidth($("chart-fusion"), rows));
    return {
      kind: "gantt", title: "Local pick, synthetic network alert and the fused state",
      desc: "Gantt chart against time after origin: the local pick window and decision, the synthetic network alert, and the fused states TRIGGERED, PROTECT and SHAKING.",
      x: { label: "time after origin", unit: "s" }, rows: rows, refs: refs, table: true
    };
  }
  function fusionCaption() {
    var s = S, el = $("cap-fusion");
    if (!el) return;
    var txt = "Time after origin at " + fmt(s.r, 3) + " km hypocentral. P arrives at " + sec(s.tP, 3) + ", the pick at " + sec(s.tPick, 3) +
      " (0.13 s: the discrete STA/LTA step response of PHYSICS §20), the on-site decision at " + sec(s.tDec, 3) + " (expected " + levelText(s) + ")";
    txt += s.tNet !== null ? ", the synthetic network alert at " + sec(s.tNet, 3) + " (an injected input, not a live feed)" : ", no network alert";
    txt += "; S arrives at " + sec(s.tS, 3) + ".";
    if (s.tProtect !== null) txt += " First PROTECT: " + sec(s.tProtect, 3) + " from the " + (s.source === "network" ? "network alert" : "local pick") + ".";
    el.textContent = txt;
  }

  /* ---- class action timelines ----------------------------------------------------- */

  var classChart = null;
  function classSpec() {
    var s = S, rows = [], i;
    var t0 = Math.max(0, Math.min(s.tFirst, s.tNet === null ? Infinity : s.tNet, s.tP) - 1), t1 = s.tEnd;
    for (i = 0; i < CLASS_KEYS.length; i++) {
      var k = CLASS_KEYS[i], o = s.cls[k], spans = [], land = LANDING[k];
      spans.push({ from: o.freezeStart, to: o.freezeEnd, color: k, label: "freeze" });
      if (s.tProtect === null) {
        spans.push({ from: o.freezeEnd, to: t1, color: k, style: "hatch", label: "frozen, holding (level ≤ 1)" });
      } else {
        if (o.landStart - o.freezeEnd > 0.02) spans.push({ from: o.freezeEnd, to: o.landStart, color: k, style: "hatch", label: "holding" });
        spans.push({ from: o.landStart, to: o.landEnd, color: k, label: "land" });
        if (o.landEnd < t1) spans.push({ from: o.landEnd, to: t1, color: k, style: "hatch", label: land.lock ? "landed + locked" : k === "door" ? "landed, ASC released" : "landed" });
      }
      rows.push({ label: F.CLASSES[k].label + (k === "drawer" ? " (open)" : ""), color: k, spans: spans });
    }
    var refs = [{ axis: "x", value: s.tS, label: "S", color: "s", dash: [4, 3] }];
    if (s.tProtect !== null) refs.push({ axis: "x", value: s.tProtect, label: "PROTECT", color: "ink2", dash: [2, 3] });
    refs.push({ axis: "x", value: s.tP, label: "P", color: "p", dash: [4, 3] });
    declutterX(refs, t0, t1, ganttWidth($("chart-classes"), rows));
    return {
      kind: "gantt", title: "Class action timelines for this event",
      desc: "Gantt chart against time after origin: for DRAWER, DOOR, WALL and HEAVY, the freeze from the first evidence, holding, the landing after PROTECT, and the landed state, with the S arrival marked.",
      x: { label: "time after origin", unit: "s" }, rows: rows, refs: refs, table: true
    };
  }
  function classVerdicts() {
    var ul = $("class-verdicts"), s = S;
    if (!ul) return;
    while (ul.firstChild) ul.removeChild(ul.firstChild);
    for (var i = 0; i < CLASS_KEYS.length; i++) {
      var k = CLASS_KEYS[i], o = s.cls[k], tm = TIMING[k], c = F.CLASSES[k];
      var li = doc.createElement("li");
      li.style.setProperty("--key", "var(--class-" + k + ")");
      var st = doc.createElement("strong");
      st.textContent = c.label + ". ";
      li.appendChild(st);
      var txt = "Stops from " + fmt(tm.v, 3) + " m/s in " + sec(tm.tFreeze, 2) + " (" + fmtSI(tm.dFreeze, "m", 2) + ")";
      if (s.tProtect === null) txt += ", then holds: no PROTECT at this level.";
      else if (o.margin >= 0) txt += ", lands in " + sec(tm.tLand, 2) + " " + LANDING[k].rest + ", " + sec(o.margin, 2) + " before the S-wave.";
      else txt += "; the landing (" + sec(tm.tLand, 2) + ") is still under way when the S-wave arrives, " + sec(-o.margin, 2) + " short.";
      if (k === "drawer") txt += " A drawer already on its seat simply stays seated.";
      if (k === "door") txt += " It never locks and stays manually operable.";
      li.appendChild(doc.createTextNode(txt));
      ul.appendChild(li);
    }
  }

  /* ---- land-first chart ----------------------------------------------------------- */

  function buildLandFirst() {
    var el = $("chart-landfirst");
    if (!el) return;
    var aw = F.CLASSES.wall.f_cont_n / F.CLASSES.wall.m_rated_kg / C.G0_MPS2;
    var ah = F.CLASSES.heavy.f_cont_n / F.CLASSES.heavy.m_rated_kg / C.G0_MPS2;
    P.chart(el, {
      kind: "gantt", title: "Lateral capacity, floor shaking and LSM authority",
      desc: "Range chart in units of g: guidance and fin lateral capacity 0.1 to 0.3 g, design-level floor shaking 0.3 to 1 g, WALL LSM authority up to " +
        aw.toFixed(3) + " g and HEAVY up to " + ah.toFixed(3) + " g.",
      x: { label: "acceleration", unit: "g", min: 0, max: 1 },
      rows: [
        { label: "Guidance + fin capacity (estimate)", spans: [{ from: 0.1, to: 0.3, color: "ink2", label: "" }] },
        { label: "Design-level floor shaking", spans: [{ from: 0.3, to: 1.0, color: "s", label: "" }] },
        { label: "WALL LSM authority F_cont/m", spans: [{ from: 0, to: aw, color: "wall", label: "" }] },
        { label: "HEAVY LSM authority F_cont/m", spans: [{ from: 0, to: ah, color: "heavy", label: "" }] }
      ],
      table: true
    });
  }

  /* ---- map scene -------------------------------------------------------------------- */

  var mapStage = $("map-stage"), mapCanvas = $("map-canvas"), mapChip = $("map-chip"), mapClock = $("map-clock");
  var mapPlay = $("map-play"), mapRestart = $("map-restart");
  var col = {}, cv = null, geo = null, mapT = 0, mapLoop = null, pinned = false, lastChip = "";
  var T_LEAD_IN = 0.6, HOLD_END = 1.8;
  /* Schematic regional stations (km offsets from the epicentre, drawn compressed). */
  var STATIONS = [[-9, -7], [6, -11], [11, 6], [-5, 12], [-16, 2], [2, 17]];

  function colours() {
    var names = ["deep", "panel", "ink", "ink2", "muted", "line", "grid", "axis", "accent", "p", "s", "good", "drawer", "wall"];
    for (var i = 0; i < names.length; i++) col[names[i]] = P.color(names[i]);
    col.steel = P.color("--steel");
    col.sans = P.theme().sans;
  }

  function mapSpeed() { return Math.max(1, (S.tEnd + T_LEAD_IN) / 9); }

  function layoutMap() {
    if (!mapStage || !mapCanvas) return;
    var w = Math.max(280, Math.round(mapStage.clientWidth || 640));
    var narrow = w < 640;
    var h = Math.round(narrow ? clamp(w * 1.02, 340, 470) : clamp(w * 0.46, 330, 470));
    cv = P.setupCanvas(mapCanvas, w, h);
    var stripH = 66, top = narrow ? 8 : 46;
    geo = { w: w, h: h, narrow: narrow, top: top, mapH: h - stripH - top - 4, stripY: h - stripH, stripH: stripH };
    geo.cy = top + geo.mapH * 0.55;
    rescaleMap();
  }

  function rescaleMap() {
    if (!geo) return;
    var span = Math.max(S.rEpi, 6);
    geo.ex = geo.w * (geo.narrow ? 0.15 : 0.12);
    var sx = geo.w * (geo.narrow ? 0.8 : 0.84);
    geo.pxKm = (sx - geo.ex) / span;
    geo.sx = geo.ex + S.rEpi * geo.pxKm;
    geo.stPx = clamp(geo.pxKm, 2.2, 4.6);
  }

  function surfR(rHypKm) {
    var h = S.depth;
    return rHypKm > h ? Math.sqrt(rHypKm * rHypKm - h * h) * geo.pxKm : 0;
  }

  function text(ctx, s, x, y, o) {
    o = o || {};
    ctx.font = (o.weight || "400") + " " + (o.size || 11) + "px " + col.sans;
    ctx.fillStyle = o.color || col.muted;
    ctx.textAlign = o.align || "left";
    ctx.textBaseline = o.base || "alphabetic";
    ctx.fillText(s, x, y);
  }
  /* Centred text kept inside the canvas. */
  function textIn(ctx, s, x, y, o) {
    o = o || {};
    ctx.font = (o.weight || "400") + " " + (o.size || 11) + "px " + col.sans;
    var half = ctx.measureText(s).width / 2;
    o.align = "center";
    text(ctx, s, clamp(x, half + 4, geo.w - half - 4), y, o);
  }
  function circle(ctx, x, y, r) { ctx.beginPath(); ctx.arc(x, y, Math.max(0, r), 0, 2 * Math.PI); }
  /* Label boxes placed in the current frame (reset by drawMap); later labels avoid them. */
  var boxes = [];
  function reserve(x0, y0, x1, y1) { boxes.push([x0, y0, x1, y1]); }
  function free(b) {
    for (var i = 0; i < boxes.length; i++) {
      var q = boxes[i];
      if (b[0] < q[2] && b[2] > q[0] && b[1] < q[3] && b[3] > q[1]) return false;
    }
    return true;
  }
  /* A label on a circle of radius r around the epicentre: at the top of the circle, else
     where it crosses the top or bottom edge of the map on the right, else skipped. */
  function labelOnRing(ctx, r, s, o) {
    if (r < 12) return;
    var g = geo, size = o.size || 10.5;
    ctx.font = (o.weight || "400") + " " + size + "px " + col.sans;
    var tw = ctx.measureText(s).width, cands = [], yTop = g.top + 14, yBot = g.top + g.mapH - 22;
    cands.push([g.ex + 4, g.cy - r - 3], [g.ex + 4, g.cy - r - 11], [g.ex + 4, g.cy - r - 19]);
    var dy1 = g.cy - yTop, dy2 = yBot - g.cy;
    if (r > dy1) cands.push([g.ex + Math.sqrt(r * r - dy1 * dy1) + 5, yTop]);
    if (r > dy2) cands.push([g.ex + Math.sqrt(r * r - dy2 * dy2) + 5, yBot]);
    for (var i = 0; i < cands.length; i++) {
      var x = cands[i][0], y = cands[i][1];
      var b = [x - 2, y - size, x + tw + 4, y + 4];
      if (y - size < g.top + 2 || y > g.top + g.mapH - 4 || b[2] > g.w - 4 || !free(b)) continue;
      reserve(b[0], b[1], b[2], b[3]);
      ctx.fillStyle = P.alpha(col.deep, 0.75);
      ctx.fillRect(b[0], b[1], b[2] - b[0], b[3] - b[1]);
      text(ctx, s, x, y, { size: size, color: o.color, weight: o.weight });
      return;
    }
  }

  function drawMapGrid(ctx) {
    var g = geo, stepKm = niceKm(70 / g.pxKm), step = stepKm * g.pxKm;
    if (step < 18) return;
    ctx.strokeStyle = P.alpha(col.grid, 0.7); ctx.lineWidth = 1;
    ctx.beginPath();
    for (var x = g.ex % step; x < g.w; x += step) { ctx.moveTo(Math.round(x) + 0.5, g.top); ctx.lineTo(Math.round(x) + 0.5, g.top + g.mapH); }
    for (var y = g.cy % step; y < g.top + g.mapH; y += step) { if (y < g.top) continue; ctx.moveTo(0, Math.round(y) + 0.5); ctx.lineTo(g.w, Math.round(y) + 0.5); }
    ctx.stroke();
  }

  function niceKm(raw) {
    var c = [0.5, 1, 2, 5, 10, 20, 25, 50, 100, 200];
    for (var i = 0; i < c.length; i++) if (c[i] >= raw) return c[i];
    return 500;
  }

  function drawRings(ctx) {
    var g = geo, s = S;
    var r0 = surfR(s.r0);
    if (r0 > 0) {
      circle(ctx, g.ex, g.cy, r0);
      ctx.fillStyle = P.alpha(col.s, 0.1); ctx.fill();
      ctx.strokeStyle = P.alpha(col.s, 0.55); ctx.lineWidth = 1; ctx.stroke();
      labelOnRing(ctx, r0, "on-site lead ≤ 0", { color: col.ink2 });
    }
    var r20 = surfR(20);
    if (r20 > 0) {
      circle(ctx, g.ex, g.cy, r20);
      ctx.setLineDash([5, 4]); ctx.strokeStyle = P.alpha(col.ink2, 0.6); ctx.lineWidth = 1; ctx.stroke(); ctx.setLineDash([]);
      labelOnRing(ctx, r20, "20 km: on-site lead " + sec(Math.max(0, s.lead20), 2), { color: col.ink2 });
    }
    if (s.rNet !== null) {
      var rn = surfR(s.rNet);
      if (rn > 0) {
        circle(ctx, g.ex, g.cy, rn);
        ctx.setLineDash([1.5, 3.5]); ctx.strokeStyle = P.alpha(col.ink2, 0.75); ctx.lineWidth = 1.2; ctx.stroke(); ctx.setLineDash([]);
        labelOnRing(ctx, rn, "alert after S inside " + fmt(s.rNet, 3) + " km", { color: col.ink2 });
      }
    }
  }

  function drawFronts(ctx, t) {
    var g = geo, s = S, k, rr;
    if (t <= 0) return;
    var rs = VS * t, rp = VP * t;
    if (rs > s.depth) {
      circle(ctx, g.ex, g.cy, surfR(rs));
      ctx.fillStyle = P.alpha(col.s, 0.07); ctx.fill();
    }
    /* P front and a short trailing train */
    for (k = 0; k < 4; k++) {
      rr = VP * (t - k * 0.2);
      if (rr <= s.depth) break;
      circle(ctx, g.ex, g.cy, surfR(rr));
      ctx.strokeStyle = P.alpha(col.p, 0.95 - k * 0.24); ctx.lineWidth = k === 0 ? 2 : 1; ctx.stroke();
    }
    /* S front: heavier, with its own train */
    for (k = 0; k < 3; k++) {
      rr = VS * (t - k * 0.25);
      if (rr <= s.depth) break;
      circle(ctx, g.ex, g.cy, surfR(rr));
      ctx.strokeStyle = P.alpha(col.s, 0.95 - k * 0.3); ctx.lineWidth = k === 0 ? 2.6 : 1.2; ctx.stroke();
    }
    if (rp > s.depth) labelOnRing(ctx, surfR(rp), "P-wave, " + fmt(VP, 2) + " km/s", { color: col.ink, size: 11, weight: "600" });
    if (rs > s.depth) labelOnRing(ctx, surfR(rs), "S-wave, " + fmt(VS, 2) + " km/s", { color: col.ink, size: 11, weight: "600" });
    /* still underground: an expanding marker at the epicentre */
    if (rp <= s.depth && s.depth > 0) {
      var f = rp / s.depth;
      circle(ctx, g.ex, g.cy, 4 + 10 * f);
      ctx.setLineDash([2, 3]); ctx.strokeStyle = P.alpha(col.p, 0.9); ctx.lineWidth = 1.2; ctx.stroke(); ctx.setLineDash([]);
    }
  }

  function stationXY(i) {
    return [geo.ex + STATIONS[i][0] * geo.stPx, geo.cy + STATIONS[i][1] * geo.stPx];
  }

  function drawStations(ctx, t) {
    var g = geo, s = S;
    for (var i = 0; i < STATIONS.length; i++) {
      var xy = stationXY(i), dPx = Math.hypot(xy[0] - g.ex, xy[1] - g.cy);
      var rHyp = Math.sqrt(Math.pow(dPx / g.pxKm, 2) + s.depth * s.depth);
      var hit = t >= rHyp / VP;
      ctx.beginPath();
      ctx.moveTo(xy[0], xy[1] - 5); ctx.lineTo(xy[0] + 4.5, xy[1] + 3.5); ctx.lineTo(xy[0] - 4.5, xy[1] + 3.5); ctx.closePath();
      ctx.fillStyle = hit ? col.p : P.alpha(col.ink2, 0.5); ctx.fill();
      ctx.strokeStyle = col.deep; ctx.lineWidth = 1; ctx.stroke();
    }
    if (!g.narrow || g.w > 420) {
      var lab = stationXY(3);
      text(ctx, "network stations (schematic)", lab[0] - 8, lab[1] + 17, { size: 10, color: col.muted });
    }
  }

  function drawEpicentre(ctx) {
    var g = geo, x = g.ex, y = g.cy, r1 = 7, r2 = 3;
    ctx.beginPath();
    for (var i = 0; i < 10; i++) {
      var a = -Math.PI / 2 + (i * Math.PI) / 5, r = i % 2 ? r2 : r1;
      if (i) ctx.lineTo(x + r * Math.cos(a), y + r * Math.sin(a)); else ctx.moveTo(x + r * Math.cos(a), y + r * Math.sin(a));
    }
    ctx.closePath();
    ctx.fillStyle = col.ink; ctx.fill();
    textIn(ctx, "epicentre · M" + fmt(S.m, 2) + " · h " + fmt(S.depth, 2) + " km", x, y - 13, { size: 10.5, color: col.ink2, weight: "600" });
  }

  function drawAlertLink(ctx, t) {
    var s = S, g = geo;
    if (s.tNet === null || t < s.tNet - 0.6) return;
    var st = stationXY(2), hx = g.sx, hy = g.cy - 10;
    ctx.setLineDash([3, 4]); ctx.strokeStyle = P.alpha(col.ink2, 0.55); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(st[0], st[1]); ctx.lineTo(hx, hy); ctx.stroke(); ctx.setLineDash([]);
    if (t < s.tNet) {
      var f = clamp(1 - (s.tNet - t) / 0.6, 0, 1);
      circle(ctx, st[0] + (hx - st[0]) * f, st[1] + (hy - st[1]) * f, 3.5);
      ctx.fillStyle = col.ink; ctx.fill();
    }
  }

  function carriageState(t) {
    var s = S;
    if (t < s.tFirst) return "carriages floating";
    if (s.tProtect === null || t < s.tProtect) return "FREEZE: carriages stopped";
    var done = t >= s.tProtect + s.needAll;
    return done ? "PROTECT: landed, WALL and HEAVY locked" : "PROTECT: landing";
  }

  function drawSite(ctx, t) {
    var g = geo, s = S, x = g.sx, base = g.cy + 16, w = g.narrow ? 24 : 30, hgt = w * 0.62;
    var shake = t >= s.tS ? 2.6 * Math.sin(t * 38) * Math.exp(-(t - s.tS) / 3) : 0;
    ctx.save();
    ctx.translate(shake, 0);
    ctx.fillStyle = col.panel; ctx.strokeStyle = col.ink; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x - w / 2, base); ctx.lineTo(x - w / 2, base - hgt); ctx.lineTo(x, base - hgt - w * 0.42);
    ctx.lineTo(x + w / 2, base - hgt); ctx.lineTo(x + w / 2, base); ctx.closePath();
    ctx.fill(); ctx.stroke();
    /* the structure accelerometer */
    var led = t >= s.tS ? col.s : t >= s.tP ? col.p : col.good;
    var blink = t >= s.tP && t < s.tDec && Math.sin(t * 18) < -0.2;
    circle(ctx, x + w * 0.22, base - 5, blink ? 2 : 3.2);
    ctx.fillStyle = led; ctx.fill();
    ctx.restore();
    textIn(ctx, "house · R " + fmt(s.r, 3) + " km", x, base + 15, { size: 10.5, color: col.ink2, weight: "600" });
    textIn(ctx, carriageState(t), x, base + 28, { size: 10, color: col.muted });
    if (s.tNet !== null && t >= s.tNet) textIn(ctx, "synthetic alert received", x, base - hgt - w * 0.42 - 8, { size: 10, color: col.ink2 });
  }

  function drawScaleBar(ctx) {
    var g = geo, km = niceKm(90 / g.pxKm), px = km * g.pxKm;
    if (px > g.w * 0.4) { km = niceKm(40 / g.pxKm); px = km * g.pxKm; }
    var x = 12, y = g.top + g.mapH - 12;
    ctx.strokeStyle = col.ink2; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(x, y - 4); ctx.lineTo(x, y); ctx.lineTo(x + px, y); ctx.lineTo(x + px, y - 4); ctx.stroke();
    text(ctx, fmt(km, 3) + " km", x + px + 6, y + 1, { size: 10.5, color: col.ink2 });
  }

  function drawStrip(ctx, t) {
    var g = geo, s = S, y0 = g.stripY, L = 16, R = g.w - 16;
    var t0 = -T_LEAD_IN, t1 = s.tEnd;
    function X(tt) { return L + ((tt - t0) / (t1 - t0)) * (R - L); }
    ctx.fillStyle = col.panel; ctx.fillRect(0, y0, g.w, g.stripH);
    ctx.strokeStyle = col.line; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0, y0 + 0.5); ctx.lineTo(g.w, y0 + 0.5); ctx.stroke();
    var axisY = y0 + 44, bandTop = y0 + 33;
    /* lead bracket */
    if (s.tProtect !== null && s.tProtect < s.tS) {
      var bx0 = X(s.tProtect), bx1 = X(s.tS);
      ctx.fillStyle = P.alpha(col.accent, 0.28); ctx.fillRect(bx0, bandTop, bx1 - bx0, 9);
      var lt = "lead " + sec(s.lead, 3);
      ctx.font = "600 10px " + col.sans;
      var lw = ctx.measureText(lt).width;
      if (lw + 8 < bx1 - bx0) text(ctx, lt, (bx0 + bx1) / 2, bandTop + 8, { align: "center", size: 10, weight: "600", color: col.ink });
    }
    ctx.strokeStyle = col.axis; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(L, axisY + 0.5); ctx.lineTo(R, axisY + 0.5); ctx.stroke();
    var tk = P.niceTicks(Math.max(0, t0), t1, Math.max(3, Math.floor((R - L) / 70)));
    for (var i = 0; i < tk.ticks.length; i++) {
      var tv = tk.ticks[i];
      if (tv < t0 || tv > t1) continue;
      ctx.beginPath(); ctx.moveTo(X(tv) + 0.5, axisY); ctx.lineTo(X(tv) + 0.5, axisY + 3); ctx.stroke();
      text(ctx, fmt(tv, 3) + " s", X(tv), axisY + 14, { align: "center", size: 10, color: col.muted });
    }
    /* markers with greedy two-row labels */
    var marks = [{ t: 0, label: "origin", color: col.muted }, { t: s.tP, label: "P", color: col.p }];
    if (s.tNet !== null) marks.push({ t: s.tNet, label: "alert", color: col.ink2 });
    marks.push({ t: s.tDec, label: s.localProtect ? "decision" : "L" + s.level, color: col.ink });
    marks.push({ t: s.tS, label: "S", color: col.s });
    marks.sort(function (a, b) { return a.t - b.t; });
    var rowsEnd = [-Infinity, -Infinity];
    ctx.font = "600 10.5px " + col.sans;
    for (var j = 0; j < marks.length; j++) {
      var m = marks[j], mx = X(m.t), tw = ctx.measureText(m.label).width;
      var lx = mx + tw + 4 > R ? mx - tw - 3 : mx + 3;         /* beside the marker, flipped at the right edge */
      var row = rowsEnd[0] < lx - 4 ? 0 : rowsEnd[1] < lx - 4 ? 1 : -1;
      ctx.strokeStyle = m.color; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(mx, row === 1 ? y0 + 16 : y0 + 5); ctx.lineTo(mx, axisY); ctx.stroke();
      if (row >= 0) {
        rowsEnd[row] = lx + tw;
        text(ctx, m.label, lx, row === 0 ? y0 + 13 : y0 + 26, { size: 10.5, weight: "600", color: col.ink2 });
      }
    }
    /* playhead */
    var px = X(clamp(t, t0, t1));
    ctx.strokeStyle = col.ink; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(px, y0 + 4); ctx.lineTo(px, axisY + 4); ctx.stroke();
    circle(ctx, px, axisY, 3.5); ctx.fillStyle = col.ink; ctx.fill();
  }

  function mapPhases() {
    var s = S, ev = [];
    ev.push({ t: -Infinity, text: "The fault ruptures " + fmt(s.depth, 2) + " km down; the clock starts at the origin time." });
    ev.push({ t: 0, text: "Both waves leave the hypocentre: P at " + fmt(VP, 2) + " km/s, S at " + fmt(VS, 2) + " km/s." });
    if (s.tNet !== null) ev.push({ t: s.tNet, text: s.source === "network"
      ? "Synthetic network alert received · PROTECT · S-wave ETA " + sec(s.tS, 3) + " after origin"
      : "Synthetic network alert received: it confirms the local decision; S-wave ETA " + sec(s.tS, 3) + " after origin" });
    ev.push({ t: s.tP, text: "The P-wave reaches the house; the structure accelerometer feels it." });
    var early = s.tProtect !== null && s.tProtect < s.tPick;
    ev.push({ t: s.tPick, text: early ? "STA/LTA picks the P-wave: local confirmation of the alert" : "STA/LTA picks the P-wave · FREEZE · measuring Pd and τc" });
    ev.push({ t: s.tDec, text: "On-site decision: level L" + s.level + (s.localProtect ? (early ? " confirms PROTECT" : " · PROTECT · land-first") : " · freeze only") });
    var sText;
    if (s.lead === null) sText = "The S-wave arrives; no PROTECT at this level, the carriages are frozen.";
    else if (s.lead <= 0) sText = "The S-wave arrives first: there was no time to land.";
    else if (s.lead >= s.needAll) sText = "The S-wave arrives " + sec(s.lead, 3) + " after PROTECT; every class is already down.";
    else sText = "The S-wave arrives " + sec(s.lead, 3) + " after PROTECT; some classes are still landing.";
    ev.push({ t: s.tS, text: sText });
    ev.sort(function (a, b) { return a.t - b.t; });
    return ev;
  }
  var PHASES = mapPhases();

  function updateMapText(t) {
    var cur = PHASES[0].text;
    for (var i = 0; i < PHASES.length; i++) if (t >= PHASES[i].t) cur = PHASES[i].text;
    if (cur !== lastChip && mapChip) { mapChip.textContent = cur; lastChip = cur; }
    if (mapClock) {
      var running = mapLoop && mapLoop.running();
      mapClock.textContent = "t = " + (t < 0 ? MINUS : "") + Math.abs(t).toFixed(1) + " s after origin" + (running ? " · shown at ×" + fmt(mapSpeed(), 2) : "");
    }
  }

  function drawMap(t) {
    if (!cv || !geo) return;
    var ctx = cv.ctx, g = geo;
    ctx.setTransform(cv.dpr, 0, 0, cv.dpr, 0, 0);
    ctx.clearRect(0, 0, g.w, g.h);
    ctx.fillStyle = col.deep; ctx.fillRect(0, 0, g.w, g.h);
    ctx.save();
    ctx.beginPath(); ctx.rect(0, g.top, g.w, g.mapH); ctx.clip();
    boxes = [];
    reserve(g.ex - 72, g.cy - 25, g.ex + 72, g.cy + 8);         /* epicentre and its label */
    reserve(g.sx - 70, g.cy - 40, g.sx + 70, g.cy + 50);        /* the house and its labels */
    reserve(0, g.top + g.mapH - 26, 150, g.top + g.mapH);       /* scale bar */
    drawMapGrid(ctx);
    drawFronts(ctx, t);
    drawRings(ctx);
    drawStations(ctx, t);
    drawAlertLink(ctx, t);
    drawEpicentre(ctx);
    drawSite(ctx, t);
    drawScaleBar(ctx);
    ctx.restore();
    drawStrip(ctx, t);
    mapT = t;
    updateMapText(t);
  }

  /* Static keyframe (reduced motion): the P front just short of the house with the S
     front behind it, the moment the page's title describes. */
  function stillTime() {
    var s = S;
    return Math.max(0.2, s.tP - 0.12 * s.sp);
  }

  function setMapPlaying(on) {
    if (!mapPlay) return;
    mapPlay.hidden = false;
    mapPlay.textContent = on ? "Pause" : "Play animation";
    mapPlay.setAttribute("aria-pressed", on ? "true" : "false");
    if (mapRestart) mapRestart.hidden = !on;
  }

  function stillMap() {
    setMapPlaying(false);
    drawMap(pinned ? mapT : stillTime());
  }

  function startMap() {
    if (!mapStage) return;
    mapLoop = P.loop(function (dt) {
      var t = mapT + dt * mapSpeed();
      if (t > S.tEnd + HOLD_END * mapSpeed()) t = -T_LEAD_IN;
      drawMap(Math.min(t, S.tEnd + HOLD_END * mapSpeed()));
      mapT = t;
    }, { el: mapStage, still: stillMap });
    if (mapPlay) {
      mapPlay.addEventListener("click", function () {
        if (mapLoop.running()) { mapLoop.stop(); setMapPlaying(false); drawMap(mapT); }
        else { pinned = false; if (mapT >= S.tEnd) mapT = -T_LEAD_IN; mapLoop.start(true); setMapPlaying(mapLoop.running()); }
      });
    }
    if (mapRestart) mapRestart.addEventListener("click", function () { mapT = -T_LEAD_IN; drawMap(mapT); });
  }

  /* ---- detector: sources ------------------------------------------------------------ */

  function mulberry32(a) {
    return function () {
      a = (a + 0x6d2b79f5) | 0;
      var t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  function gaussian(rng) {
    var u = 1 - rng(), v = rng();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }

  /* The recursive STA/LTA of PHYSICS §20 on e_k = x_k²: gains 1/min(k, N), the LTA not
     updated while triggered, on at k ≥ LTA_N and ratio ≥ 4.0, off below 1.5. */
  function staLta(x) {
    var n = x.length, out = new Array(n), sta = 0, lta = 0, trig = false, pick = -1;
    for (var i = 0; i < n; i++) {
      var k = i + 1, e = x[i] * x[i];
      sta += (e - sta) / Math.min(k, DET.STA_N);
      if (!trig) lta += (e - lta) / Math.min(k, DET.LTA_N);
      var r = sta / Math.max(lta, DET.FLOOR_MPS2_SQ);
      out[i] = r;
      if (!trig && k >= DET.LTA_N && r >= DET.TRIG_ON) { trig = true; if (pick < 0) pick = i; }
      else if (trig && r < DET.TRIG_OFF) trig = false;
    }
    return { ratio: out, pick: pick };
  }

  /* An illustrative record: the catalog M7 case (20 km epicentral, 10 km deep) with
     sensor and building noise, a P phase whose displacement peak follows Wu–Zhao and whose
     period follows Wu–Kanamori, then the S phase, mostly horizontal. Not a simulator
     output and not Boore's method: a schematic that shows what the detector sees. */
  function illustrativeSource() {
    var cat = CATALOG.pwave_m7_r20, m = cat.m;
    var rHyp = Math.sqrt(cat.rEpi * cat.rEpi + cat.depth * cat.depth);
    var fs = DET.FS_HZ, tP = T_P_RECORD_S, tS = tP + F.spTime(rHyp);
    var n = Math.round((tP + 12) * fs) + 1, i, j;
    var rng = mulberry32(42);
    var pdT = F.pdWuZhao(m, rHyp), tcT = tauCWuKanamori(m);
    var pc = [], mults = [1, 2.3, 4.9, 9.7];
    for (j = 0; j < mults.length; j++) pc.push({ w: (2 * Math.PI * mults[j]) / tcT, a: 1 / (mults[j] * mults[j]), ph: 2 * Math.PI * rng() });
    var sc = [], sf = [1.1, 1.9, 3.1, 4.7, 6.3];
    for (j = 0; j < sf.length; j++) sc.push({ w: 2 * Math.PI * sf[j], a: 1 / (1 + j * 0.35), ph: 2 * Math.PI * rng() });
    function envP(tau) { return (1 - Math.exp(-tau / 0.6)) * Math.exp(-tau / 5); }
    function envS(tau) { return (1 - Math.exp(-tau / 0.4)) * Math.exp(-tau / 7); }
    /* P displacement scale: max|u| over the first 3 s equals the Wu–Zhao Pd */
    var maxU = 0, maxZ = 0, maxS = 0;
    for (i = 0; i <= 3 * fs; i++) {
      var tau0 = i / fs, u0 = 0, z0 = 0;
      for (j = 0; j < pc.length; j++) { var s0 = Math.sin(pc[j].w * tau0 + pc[j].ph); u0 += pc[j].a * s0; z0 += pc[j].a * pc[j].w * pc[j].w * s0; }
      maxU = Math.max(maxU, Math.abs(u0 * envP(tau0)));
      maxZ = Math.max(maxZ, Math.abs(z0 * envP(tau0)));
    }
    for (i = 0; i <= 10 * fs; i++) {
      var ts0 = i / fs, x0 = 0;
      for (j = 0; j < sc.length; j++) x0 += sc[j].a * Math.sin(sc[j].w * ts0 + sc[j].ph);
      maxS = Math.max(maxS, Math.abs(x0 * envS(ts0)));
    }
    var kP = pdT / maxU;
    var kS = (2.6 * kP * maxZ) / maxS;          /* S horizontal peak ≈ 2.6 × the P vertical peak */
    var noise = 3.0e-3;                         /* sensor (≈ 1e-3 m/s² over 20 Hz) plus building noise, m/s² */
    var t = new Array(n), az = new Array(n), ax = new Array(n), u = new Array(n), ud = new Array(n);
    for (i = 0; i < n; i++) {
      var tt = i / fs, z = 0, x = 0, uu = 0, vv = 0, tau = tt - tP;
      if (tau >= 0) {
        var e = envP(tau);
        for (j = 0; j < pc.length; j++) {
          var sn = Math.sin(pc[j].w * tau + pc[j].ph), cs = Math.cos(pc[j].w * tau + pc[j].ph);
          uu += kP * pc[j].a * e * sn;
          vv += kP * pc[j].a * e * pc[j].w * cs;
          z -= kP * pc[j].a * e * pc[j].w * pc[j].w * sn;
        }
        x += 0.3 * z;                           /* Z/H > 1 in the P window */
      }
      if (tt >= tS) {
        var es = envS(tt - tS), sx = 0;
        for (j = 0; j < sc.length; j++) sx += sc[j].a * Math.sin(sc[j].w * (tt - tS) + sc[j].ph);
        x += kS * es * sx;
        z += 0.3 * kS * es * sx;
      }
      t[i] = tt;
      az[i] = z + noise * gaussian(rng);
      ax[i] = x + noise * gaussian(rng);
      u[i] = uu; ud[i] = vv;
    }
    var sl = staLta(az);
    var pick = sl.pick;
    var pd = new Array(n), tc = new Array(n), level = new Array(n), state = new Array(n);
    var su2 = 0, sv2 = 0, mu = 0, lv = 0, protectIdx = -1, sIdx = Math.ceil(tS * fs);
    for (i = 0; i < n; i++) {
      pd[i] = 0; tc[i] = 0; level[i] = 0; state[i] = 0;
      if (pick < 0 || i < pick) continue;
      var dt = (i - pick) / fs;
      if (dt <= DET.WIN_LONG_S) {
        su2 += u[i] * u[i]; sv2 += ud[i] * ud[i]; mu = Math.max(mu, Math.abs(u[i]));
      }
      if (dt >= DET.WIN_SHORT_S) {
        var tcv = sv2 > 0 ? 2 * Math.PI * Math.sqrt(su2 / sv2) : 0;
        pd[i] = dt <= DET.WIN_LONG_S ? mu : pd[i - 1];
        tc[i] = dt <= DET.WIN_LONG_S ? tcv : tc[i - 1];
        lv = Math.max(lv, levelFor(pd[i], tc[i]));
        if (protectIdx < 0 && isProtect(lv, pd[i])) protectIdx = i;
      }
      level[i] = lv;
      state[i] = i >= sIdx ? 3 : protectIdx >= 0 && i >= protectIdx ? 2 : 1;
    }
    return {
      kind: "illustrative", t: t, tP: tP, tS: tS, m: m, rHyp: rHyp, az: az, ax: ax, ratio: sl.ratio, ratioKind: "illustrative",
      pd: pd, tauc: tc, level: level, state: state, pdTarget: pdT, tauTarget: tcT,
      ev: {
        pick: pick >= 0 ? t[pick] : null, protect: protectIdx >= 0 ? t[protectIdx] : null,
        protectLevel: protectIdx >= 0 ? level[protectIdx] : null, shaking: tS, sEtaIn: null, landed: null, locked: null, moveStart: null, quiet: null
      }
    };
  }

  function normEvents(list) {
    var ev = { pick: null, protect: null, protectLevel: null, sEtaIn: null, shaking: null, quiet: null, inspect: null, landed: null, locked: null, moveStart: null };
    for (var i = 0; i < (list || []).length; i++) {
      var e = list[i];
      if (!e || e.length < 2) continue;
      var t = +e[0], kind = String(e[1]), a = +e[2], b = +e[3];
      if (kind === "Freeze" && ev.pick === null) ev.pick = t;
      else if (kind === "SeismicState") {
        if (a === 1 && ev.pick === null) ev.pick = t;
        if (a === 3 && ev.shaking === null) ev.shaking = t;
        if (a === 4 && ev.quiet === null) ev.quiet = t;
        if (a === 5 && ev.inspect === null) ev.inspect = t;
      } else if (kind === "Protect" && ev.protect === null) { ev.protect = t; ev.protectLevel = a; ev.sEtaIn = isFinite(b) && b > 0 ? b : null; }
      else if ((kind === "plant:landed" || kind === "Landed") && ev.landed === null) ev.landed = t;
      else if (kind === "plant:locked" && ev.locked === null) ev.locked = t;
      else if ((kind === "scenario:command" || kind === "Command") && ev.moveStart === null && ev.pick === null) ev.moveStart = t;
    }
    return ev;
  }

  function recordedSource() {
    var run = root.RUN;
    if (!run || run.placeholder || !run.runs) return null;
    var key = null, i, keys = Object.keys(run.runs);
    for (i = 0; i < RECORDED_KEYS.length && !key; i++) if (run.runs[RECORDED_KEYS[i]]) key = RECORDED_KEYS[i];
    for (i = 0; i < keys.length && !key; i++) if (/^pwave_/.test(String(run.runs[keys[i]].scenario || ""))) key = keys[i];
    if (!key) return null;
    var e = run.runs[key], ts = e.t_s || [], ch = e.channels || {};
    function arr(name) { var c = ch[name]; return c && c.data && c.data.length === ts.length ? c.data : null; }
    var az = arr("floor_az_mps2");
    if (!az || ts.length < 10) return null;
    var info = e.info || {}, cat = CATALOG[e.scenario] || {};
    var m = num(info.magnitude, cat.m), depth = num(info.depth_km, cat.depth);
    var rHyp = num(info.r_hyp_km, cat.rEpi !== undefined ? Math.sqrt(cat.rEpi * cat.rEpi + depth * depth) : NaN);
    if (!isFinite(rHyp) || !isFinite(m)) return null;
    var tP = num(info.t_p_s, T_P_RECORD_S), tS = num(info.t_s_s, tP + F.spTime(rHyp));
    var pdm = arr("pd_m");
    return {
      kind: "replay", key: key, cls: e.class, scenario: e.scenario, fidelity: e.fidelity, coil: e.coil, seed: run.seed,
      t: ts, tP: tP, tS: tS, m: m, rHyp: rHyp, az: az, ax: arr("floor_ax_mps2"),
      ratio: arr("sta_lta"), ratioKind: arr("sta_lta") ? "replay" : "illustrative",
      pd: pdm, tauc: arr("tau_c_s"), level: arr("level"), state: arr("seismic_state"),
      pdTarget: num(info.pd_target_m, F.pdWuZhao(m, rHyp)), tauTarget: num(info.tau_c_s, tauCWuKanamori(m)),
      infoFromRun: !!e.info, ev: normEvents(e.events), kpis: e.kpis || {}
    };
  }

  /* ---- detector: charts --------------------------------------------------------------- */

  var DET_X0 = -2, DET_X1 = 10;
  var det = { src: null, ill: null, charts: {}, loop: null, t: DET_X0, playing: false };

  function windowed(src, arrY, scale, x0, x1) {
    var out = [];
    if (!arrY) return out;
    for (var i = 0; i < src.t.length; i++) {
      var x = src.t[i] - src.tP;
      if (x < x0 || x > x1) continue;
      var y = arrY[i];
      out.push([x, typeof y === "number" ? y * (scale || 1) : NaN]);
    }
    return out;
  }
  function indexAt(src, tAbs) {
    var t = src.t, lo = 0, hi = t.length - 1;
    if (hi < 0) return -1;
    while (hi - lo > 1) { var mid = (lo + hi) >> 1; if (t[mid] < tAbs) lo = mid; else hi = mid; }
    return Math.abs(t[lo] - tAbs) <= Math.abs(t[hi] - tAbs) ? lo : hi;
  }
  function valueAt(src, arrY, tAbs) {
    if (!arrY) return null;
    var i = indexAt(src, tAbs);
    return i >= 0 ? arrY[i] : null;
  }

  function simSlot(name, kind) {
    var el = doc.querySelector('[data-sim-for="' + name + '"]');
    if (!el) return;
    while (el.firstChild) el.removeChild(el.firstChild);
    el.appendChild(UI.simLabel(kind === "replay" ? "replay" : "illustrative"));
  }

  function detRefsX(src, extra) {
    var refs = [
      { axis: "x", value: 0, label: "P arrives", color: "p", dash: [4, 3] },
      { axis: "x", value: src.tS - src.tP, label: "S arrives", color: "s", dash: [4, 3] }
    ];
    return refs.concat(extra || []);
  }

  function buildDetector() {
    var srcRec = recordedSource();
    det.ill = illustrativeSource();
    det.src = srcRec || det.ill;
    var src = det.src, ill = det.ill;
    var x1 = Math.min(DET_X1, src.t[src.t.length - 1] - src.tP);
    var pickX = src.ev.pick !== null ? src.ev.pick - src.tP : null;
    var protX = src.ev.protect !== null ? src.ev.protect - src.tP : null;

    /* source line */
    var lab = $("det-source-label"), txt = $("det-source-text");
    if (lab) {
      lab.setAttribute("data-simulated", src.kind === "replay" ? "replay" : "illustrative");
      lab.textContent = src.kind === "replay" ? "simulated replay" : "illustrative";
      lab.classList.add("sim-label");
    }
    if (txt) {
      txt.textContent = src.kind === "replay"
        ? "The recorded seed-" + src.seed + " simulator run " + src.key + " (" + String(src.cls || "").toUpperCase() + ", " + src.fidelity + ", " + src.coil +
          " coil mode): a synthetic M" + fmt(src.m, 2) + " record at " + fmt(src.rHyp, 3) + " km hypocentral, Boore stochastic method with the P phase scaled to Wu–Zhao." +
          (src.infoFromRun ? "" : " Event parameters from the scenario catalog.")
        : "An illustrative record generated in your browser (M" + fmt(src.m, 2) + ", " + fmt(src.rHyp, 3) +
          " km hypocentral, with sensor and building noise). It is not a simulator output; the recorded seed-42 run replaces it once the site data have been generated.";
    }
    var empty = $("det-empty");
    if (empty) empty.hidden = src.kind === "replay";
    simSlot("accel", src.kind); simSlot("pd", src.kind); simSlot("tauc", src.kind); simSlot("ratio", src.ratioKind);

    /* acceleration */
    var accSeries = [{ key: "z", label: "vertical (P shows first)", color: "p", data: windowed(src, src.az, 1, DET_X0, x1), width: 1.25 }];
    if (src.ax) accSeries.push({ key: "x", label: "horizontal (S carries it)", color: "s", data: windowed(src, src.ax, 1, DET_X0, x1), width: 1.25 });
    var accRefs = detRefsX(src, pickX !== null ? [{ axis: "x", value: pickX, label: "pick", color: "ink2", dash: [2, 3] }] : []);
    det.charts.accel = P.chart($("chart-accel"), {
      kind: "line", title: "Floor acceleration around the P arrival",
      desc: "Line chart of vertical and horizontal floor acceleration against time after the P arrival; the S arrival is marked.",
      x: { label: "time after P", unit: "s", min: DET_X0, max: x1 }, y: { label: "acceleration", unit: "m/s²" },
      series: accSeries, refs: staggerX(accRefs, DET_X0, x1, 320), height: 210, table: true, tableRows: 24
    });

    /* STA/LTA */
    var rsrc = src.ratio ? src : ill;
    var rData = windowed(rsrc, rsrc.ratio, 1, DET_X0, x1), rMax = 10;
    for (var i = 0; i < rData.length; i++) rMax = Math.max(rMax, rData[i][1] || 0);
    var rPick = rsrc.ev.pick !== null ? rsrc.ev.pick - rsrc.tP : null;
    det.charts.ratio = P.chart($("chart-ratio"), {
      kind: "semilog", title: "STA/LTA ratio on the vertical channel",
      desc: "Semilog chart of the STA/LTA ratio against time after the P arrival, with the trigger-on line at 4.0 and the trigger-off line at 1.5.",
      x: { label: "time after P", unit: "s", min: DET_X0, max: x1 }, y: { label: "STA/LTA", min: 0.3, max: Math.pow(10, Math.ceil(Math.log(rMax) / Math.LN10)) },
      series: [{ key: "ratio", label: "STA/LTA", color: "ink", data: rData }],
      refs: staggerX([
        { axis: "y", value: DET.TRIG_ON, label: "on 4.0", color: "ref", dash: [5, 4] },
        { axis: "y", value: DET.TRIG_OFF, label: "off 1.5", color: "muted", dash: [2, 3], align: "left" }
      ].concat(rPick !== null ? [{ axis: "x", value: rPick, label: "pick " + signed(rPick, 2), color: "ink2", dash: [2, 3] }] : []), DET_X0, x1, 320),
      height: 210, table: true, tableRows: 24
    });
    var capR = $("cap-ratio");
    if (capR && src.kind === "replay" && !src.ratio) {
      capR.textContent = "The detector's own ratio is not a recorder channel in this build, so this panel runs the same recursion (PHYSICS §20) on the illustrative record; the recorded pick was at " +
        (pickX !== null ? signed(pickX, 2) : "–") + " after the P arrival.";
    }

    /* Pd */
    var pdData = windowed(src, src.pd, 100, DET_X0, x1), pdMax = 0.6;
    for (i = 0; i < pdData.length; i++) pdMax = Math.max(pdMax, pdData[i][1] || 0);
    pdMax = P.niceTicks(0, Math.max(pdMax, src.pdTarget * 100) * 1.15, 4).hi;
    var winRefs = pickX !== null ? [
      { axis: "x", value: pickX + DET.WIN_SHORT_S, label: "1 s", color: "muted", dash: [2, 3] },
      { axis: "x", value: pickX + DET.WIN_LONG_S, label: "3 s", color: "muted", dash: [2, 3] }
    ] : [];
    det.charts.pd = P.chart($("chart-pd"), {
      kind: "line", title: "Pd since the pick",
      desc: "Line chart of the peak displacement Pd in centimetres against time after the P arrival, with the PROTECT threshold 0.35 cm, the L3 threshold 0.5 cm and the Wu–Zhao value.",
      x: { label: "time after P", unit: "s", min: DET_X0, max: x1 }, y: { label: "Pd", unit: "cm", min: 0, max: pdMax },
      series: [{ key: "pd", label: "Pd", color: "ink", data: pdData }],
      refs: staggerX([
        { axis: "y", value: DET.PD_ALERT_M * 100, label: "PROTECT alone 0.35 cm", color: "ref", dash: [5, 4], align: "left" },
        { axis: "y", value: DET.PD_L3_M * 100, label: "L3 needs 0.5 cm", color: "muted", dash: [2, 3], align: "left" },
        { axis: "y", value: src.pdTarget * 100, label: "Wu–Zhao " + fmt(src.pdTarget * 100, 2) + " cm", color: "p", dash: [6, 3] }
      ].concat(winRefs), DET_X0, x1, 320),
      height: 210, table: true, tableRows: 24
    });

    /* τc */
    var tcData = windowed(src, src.tauc, 1, DET_X0, x1), tcMax = 1;
    for (i = 0; i < tcData.length; i++) tcMax = Math.max(tcMax, tcData[i][1] || 0);
    tcMax = P.niceTicks(0, Math.max(tcMax, src.tauTarget) * 1.15, 4).hi;
    det.charts.tauc = P.chart($("chart-tauc"), {
      kind: "line", title: "τc since the pick",
      desc: "Line chart of the dominant period τc in seconds against time after the P arrival, with the 0.6 s level threshold and the Wu–Kanamori value.",
      x: { label: "time after P", unit: "s", min: DET_X0, max: x1 }, y: { label: "τc", unit: "s", min: 0, max: tcMax },
      series: [{ key: "tauc", label: "τc", color: "ink", data: tcData }],
      refs: staggerX([
        { axis: "y", value: DET.TAUC_L3_S, label: "L2/L3 need 0.6 s", color: "ref", dash: [5, 4], align: "left" },
        { axis: "y", value: src.tauTarget, label: "Wu–Kanamori " + sec(src.tauTarget, 3), color: "p", dash: [6, 3] }
      ].concat(winRefs.map(function (r) { return { axis: r.axis, value: r.value, label: r.label, color: r.color, dash: r.dash }; })), DET_X0, x1, 320),
      height: 210, table: true, tableRows: 24
    });

    detReadouts(src, pickX, protX);
    fsmTimes(src);
    if (src.kind === "replay") buildRecordedTimeline(src);
  }

  function statEl(label, value, note) {
    var d = doc.createElement("div");
    d.className = "stat";
    var l = doc.createElement("span"); l.className = "stat-label"; l.textContent = label;
    var v = doc.createElement("span"); v.className = "stat-value num"; v.textContent = value;
    d.appendChild(l); d.appendChild(v);
    if (note) { var n = doc.createElement("span"); n.className = "stat-note"; n.textContent = note; d.appendChild(n); }
    return d;
  }

  function detReadouts(src, pickX, protX) {
    var box = $("det-readouts");
    if (!box) return;
    while (box.firstChild) box.removeChild(box.firstChild);
    var tag = src.kind === "replay" ? "recorded" : "illustrative";
    box.appendChild(statEl("Pick", pickX !== null ? signed(pickX, 2) : "none", "after the P arrival · " + tag));
    box.appendChild(statEl("PROTECT", protX !== null ? signed(protX, 3) : "none",
      src.ev.protectLevel !== null && src.ev.protectLevel !== undefined ? "level L" + fmt(src.ev.protectLevel, 1) + " at the decision" : "no PROTECT"));
    var t3 = src.ev.pick !== null ? src.ev.pick + DET.WIN_LONG_S + 0.02 : null;
    var pd3 = t3 !== null ? valueAt(src, src.pd, t3) : null, tc3 = t3 !== null ? valueAt(src, src.tauc, t3) : null;
    box.appendChild(statEl("Pd after 3 s", pd3 !== null ? fmt(pd3 * 100, 3) + " cm" : "–", "Wu–Zhao " + fmt(src.pdTarget * 100, 3) + " cm at " + fmt(src.rHyp, 3) + " km"));
    box.appendChild(statEl("τc after 3 s", tc3 !== null ? sec(tc3, 3) : "–", "Wu–Kanamori " + sec(src.tauTarget, 3) + " for M" + fmt(src.m, 2)));
    if (src.kind === "replay") {
      if (src.ev.landed !== null) {
        var margin = src.tS - (src.ev.locked !== null ? src.ev.locked : src.ev.landed);
        box.appendChild(statEl(src.ev.locked !== null ? "Landed and locked" : "Landed", signed((src.ev.locked !== null ? src.ev.locked : src.ev.landed) - src.tP, 3),
          margin >= 0 ? sec(margin, 3) + " before the S-wave" : sec(-margin, 3) + " after the S-wave"));
      }
      if (src.ev.sEtaIn !== null && src.ev.protect !== null) {
        var eta = src.ev.protect + src.ev.sEtaIn, err = eta - src.tS;
        box.appendChild(statEl("Local S-ETA at PROTECT", signed(eta - src.tP, 3),
          "true S at " + signed(src.tS - src.tP, 3) + ": " + (err >= 0 ? "late by " : "early by ") + sec(Math.abs(err), 2)));
      }
    }
  }

  var STATE_NAMES = ["NORMAL", "TRIGGERED", "PROTECT", "SHAKING", "QUIET", "INSPECT"];
  function stateAt(src, tAbs) {
    if (!src.state) return null;
    var v = valueAt(src, src.state, tAbs);
    return v === null ? null : Math.round(v);
  }
  function fsmTimes(src) {
    var items = doc.querySelectorAll("#fsm li[data-state]");
    var first = {}, ev = src.ev || {};
    if (src.state) {
      for (var i = 0; i < src.t.length; i++) {
        var s = Math.round(src.state[i]);
        if (first[s] === undefined) first[s] = src.t[i];
      }
    }
    var fromEv = { 1: ev.pick, 3: ev.shaking, 4: ev.quiet, 5: ev.inspect };
    if (src.kind === "replay") fromEv[2] = ev.protect;
    for (var key in fromEv) if (fromEv[key] !== null && fromEv[key] !== undefined) first[key] = fromEv[key];
    for (var j = 0; j < items.length; j++) {
      var k = +items[j].getAttribute("data-state"), tt = items[j].querySelector(".fsm-t");
      if (first[k] !== undefined && k > 0) {
        items[j].setAttribute("data-reached", "");
        if (tt) tt.textContent = signed(first[k] - src.tP, 3);
      } else if (k === 0) {
        items[j].setAttribute("data-reached", "");
      } else if (tt) {
        tt.textContent = k === 5 ? "(needs a person)" : "";
      }
    }
  }
  function fsmCurrent(stateVal) {
    var items = doc.querySelectorAll("#fsm li[data-state]");
    for (var j = 0; j < items.length; j++) {
      if (stateVal !== null && +items[j].getAttribute("data-state") === stateVal) items[j].setAttribute("aria-current", "step");
      else items[j].removeAttribute("aria-current");
    }
  }

  function buildRecordedTimeline(src) {
    var fig = $("det-timeline-fig"), el = $("chart-det-timeline");
    if (!fig || !el || !src.state) return;
    fig.hidden = false;
    var tEnd = src.t[src.t.length - 1] - src.tP, x0 = DET_X0;
    var spansBy = {}, cur = null, start = null, i;
    for (i = 0; i < src.t.length; i++) {
      var s = Math.round(src.state[i]), x = src.t[i] - src.tP;
      if (s !== cur) {
        if (cur !== null && cur > 0) (spansBy[cur] = spansBy[cur] || []).push([Math.max(start, x0), x]);
        cur = s; start = x;
      }
    }
    if (cur !== null && cur > 0) (spansBy[cur] = spansBy[cur] || []).push([Math.max(start, x0), tEnd]);
    var colors = { 1: "p", 2: "ink2", 3: "s", 4: "muted", 5: "muted" };
    var rows = [];
    for (var st = 1; st <= 5; st++) {
      if (!spansBy[st]) continue;
      rows.push({ label: "state: " + STATE_NAMES[st], spans: spansBy[st].map(function (p) { return { from: p[0], to: p[1], color: colors[st], style: st >= 4 ? "hatch" : "solid", label: "" }; }) });
    }
    var cls = String(src.cls || "wall"), ev = src.ev, rel = function (t) { return t - src.tP; };
    var cRows = [];
    if (ev.moveStart !== null && ev.pick !== null && ev.pick > ev.moveStart) cRows.push({ label: "carriage: moving", spans: [{ from: Math.max(rel(ev.moveStart), x0), to: rel(ev.pick), color: cls, label: "" }] });
    if (ev.pick !== null) cRows.push({ label: "carriage: frozen", spans: [{ from: rel(ev.pick), to: ev.protect !== null ? rel(ev.protect) : tEnd, color: cls, style: "hatch", label: "" }] });
    if (ev.protect !== null && ev.landed !== null && ev.landed > ev.protect) cRows.push({ label: "carriage: landing", spans: [{ from: rel(ev.protect), to: rel(ev.landed), color: cls, label: "" }] });
    if (ev.landed !== null) cRows.push({ label: ev.locked !== null ? "carriage: landed, locked" : "carriage: landed", spans: [{ from: rel(ev.locked !== null ? ev.locked : ev.landed), to: tEnd, color: cls, style: "hatch", label: "" }] });
    rows = rows.concat(cRows);
    var refs = [
      { axis: "x", value: 0, label: "P", color: "p", dash: [4, 3] },
      { axis: "x", value: src.tS - src.tP, label: "S", color: "s", dash: [4, 3] }
    ];
    declutterX(refs, x0, tEnd, ganttWidth(el, rows));
    P.chart(el, {
      kind: "gantt", title: "Recorded seismic states and carriage events",
      desc: "Gantt chart against time after the P arrival of the recorded run: seismic states from the state channel and carriage events from the event log.",
      x: { label: "time after P", unit: "s" }, rows: rows, refs: refs, table: true
    });
    var cap = $("cap-det-timeline");
    if (cap) {
      cap.textContent = "Seismic state from the recorded seismic_state channel; carriage events (freeze, landing, touchdown" + (ev.locked !== null ? ", lock" : "") +
        ") from the run's event log. " + String(src.cls || "").toUpperCase() + " class, " + src.fidelity + " plant tier. The QUIET state follows 30 s without triggers; INSPECT needs a manual reset and is not part of the run.";
    }
  }

  /* ---- detector replay ---------------------------------------------------------------- */

  function setReveal(x) {
    for (var k in det.charts) {
      var ch = det.charts[k];
      if (!ch) continue;
      ch.spec.reveal = x;
      ch.render();
    }
  }
  function detFrame(x) {
    setReveal(x);
    var st = stateAt(det.src, det.src.tP + x);
    fsmCurrent(st);
    var clk = $("det-clock");
    if (clk) clk.textContent = "t − t_P = " + (x < 0 ? MINUS : "") + Math.abs(x).toFixed(2) + " s" + (st !== null ? " · " + STATE_NAMES[st] : "");
  }
  function detStop(showAll) {
    if (det.loop) det.loop.stop();
    det.playing = false;
    var btn = $("det-play");
    if (btn) { btn.textContent = "Replay the record"; btn.setAttribute("aria-pressed", "false"); }
    if (showAll) {
      setReveal(undefined);
      fsmCurrent(null);
      var clk = $("det-clock");
      if (clk) clk.textContent = "full record shown";
    }
  }
  function setupDetReplay() {
    var btn = $("det-play");
    if (!btn || !det.src) return;
    var x1 = Math.min(DET_X1, det.src.t[det.src.t.length - 1] - det.src.tP);
    det.loop = P.loop(function (dt) {
      det.t += dt;
      if (det.t >= x1) { detFrame(x1); detStop(false); return; }
      detFrame(det.t);
    }, { el: $("chart-accel") });
    btn.addEventListener("click", function () {
      if (det.playing) { detStop(true); return; }
      det.t = DET_X0;
      det.playing = true;
      btn.textContent = "Show the full record";
      btn.setAttribute("aria-pressed", "true");
      detFrame(det.t);
      det.loop.start(true);
    });
  }

  /* ---- controls ----------------------------------------------------------------------- */

  var pending = false;
  function refreshAll(keepTime) {
    S = scenario(Q);
    PHASES = mapPhases();
    lastChip = "";
    updateReadouts();
    if (leadChart) leadChart.update(leadSpec());
    if (fusionChart) fusionChart.update(fusionSpec());
    fusionCaption();
    if (classChart) classChart.update(classSpec());
    classVerdicts();
    rescaleMap();
    if (!(mapLoop && mapLoop.running())) drawMap(keepTime ? clamp(mapT, -T_LEAD_IN, S.tEnd) : stillTime());
    else if (mapT > S.tEnd) mapT = -T_LEAD_IN;
  }
  function scheduleRefresh() {
    if (pending) return;
    pending = true;
    var run = function () { pending = false; refreshAll(false); };
    if (root.requestAnimationFrame) root.requestAnimationFrame(run); else setTimeout(run, 16);
  }

  var binder = UI.bindSliders($("quake-controls"), function (v) {
    if (v.r_hyp_km !== undefined) Q.r = v.r_hyp_km;
    if (v.depth_km !== undefined) Q.depth = v.depth_km;
    if (v.magnitude !== undefined) Q.m = v.magnitude;
    if (v.t_onsite_s !== undefined) Q.tOnsite = v.t_onsite_s;
    if (v.t_network_s !== undefined) Q.tNetwork = v.t_network_s;
    pinned = false;
    scheduleRefresh();
  }, { initial: false });
  var srcSeg = UI.segmented($("src-choice"), function (v) {
    Q.network = v !== "local";
    scheduleRefresh();
  });
  if (srcSeg) Q.network = srcSeg.value() !== "local";
  var vals = binder.values();
  if (vals.r_hyp_km !== undefined) Q.r = vals.r_hyp_km;
  if (vals.depth_km !== undefined) Q.depth = vals.depth_km;
  if (vals.magnitude !== undefined) Q.m = vals.magnitude;
  if (vals.t_onsite_s !== undefined) Q.tOnsite = vals.t_onsite_s;
  if (vals.t_network_s !== undefined) Q.tNetwork = vals.t_network_s;

  /* #r=..&d=..&m=..&t=.. deep link */
  var hashT = null;
  (function parseHash() {
    var h = (root.location && root.location.hash || "").replace(/^#/, "");
    if (!h) return;
    var parts = h.split("&");
    for (var i = 0; i < parts.length; i++) {
      var kv = parts[i].split("="), v = parseFloat(kv[1]);
      if (!isFinite(v)) continue;
      if (kv[0] === "r") { Q.r = clamp(v, 2, 200); binder.set("r_hyp_km", Q.r, true); }
      else if (kv[0] === "d") { Q.depth = clamp(v, 0, 30); binder.set("depth_km", Q.depth, true); }
      else if (kv[0] === "m") { Q.m = clamp(v, 5, 8); binder.set("magnitude", Q.m, true); }
      else if (kv[0] === "t") hashT = v;
    }
  })();

  /* ---- start ------------------------------------------------------------------------ */

  S = scenario(Q);
  PHASES = mapPhases();
  fillText();
  updateReadouts();
  buildLeadChart();
  var fusEl = $("chart-fusion");
  if (fusEl) fusionChart = P.chart(fusEl, fusionSpec());
  fusionCaption();
  var clsEl = $("chart-classes");
  if (clsEl) classChart = P.chart(clsEl, classSpec());
  classVerdicts();
  buildLandFirst();
  try {
    buildDetector();
    setupDetReplay();
  } catch (e) {
    /* a malformed RUN block must not take the rest of the page down */
    var em = $("det-empty");
    if (em) { em.hidden = false; em.textContent = "The detector figures could not be drawn from the site data (" + (e && e.message ? e.message : "unknown error") + ")."; }
  }

  if (mapStage && mapCanvas) {
    colours();
    layoutMap();
    startMap();
    P.onTheme(function () { colours(); drawMap(mapT); });
    var relayout = function () { layoutMap(); drawMap(mapT); };
    if (root.ResizeObserver) {
      var lastW = 0;
      new root.ResizeObserver(function () {
        var w = mapStage.clientWidth;
        if (Math.abs(w - lastW) >= 1) { lastW = w; relayout(); }
      }).observe(mapStage);
    } else {
      root.addEventListener("resize", relayout);
    }
    if (hashT !== null) {
      pinned = true;
      setMapPlaying(false);
      drawMap(clamp(hashT, -T_LEAD_IN, S.tEnd));
    } else {
      mapT = -T_LEAD_IN;
      if (mapLoop.start()) setMapPlaying(true);
      else stillMap();
    }
  }
})(typeof window !== "undefined" ? window : this);
