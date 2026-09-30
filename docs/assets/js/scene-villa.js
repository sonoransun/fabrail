/* villa.html scene (owner W4): the flagship "villa_quake" replay (plan §4b, §3b).
 *
 * Data. The page is driven by the generated global VILLA_RUN (schema
 * fabrail.docs.villa_run/1 in assets/data/villa_run.js; written by
 * scripts/update_docs_run.py from the seed-42 `python -m fabrail example run villa_quake`
 * record, never hand-edited). While VILLA_RUN is still the placeholder, the page shows a
 * clearly labelled PREVIEW built from closed forms only: the drawer closes come from
 * FORMULAS.closeProfile at the unattended limit (the same terminus law the controller
 * runs, PHYSICS §16.1), and the event timing from the source geometry (PHYSICS §20).
 * Panels that need the run (waterfall, contents, ablation, bus power) stay empty in the
 * preview: they never show illustrative numbers.
 *
 * Wording rule (plan §4b; SPECIFICATION_EXAMPLES §0): "smooth", "calculated" and
 * "log-damped" are used only with their measurable definitions; a zero contents count is
 * worded "no Coulomb slip event in the sliding-block model"; every alert is
 * "synthetic/injected, not a live EEW feed".
 *
 * Building sway on the canvas is schematic: an illustrative ground motion filtered by a
 * single-degree-of-freedom isolation layer and a single storey mode per configuration
 * (PHYSICS §21, EXOTIC §X9). It is labelled as such and never reported as a result.
 *
 * Deep link: #t=<seconds after origin> opens the replay paused at that moment.
 */
(function () {
  "use strict";
  var F = window.FabrailFormulas, P = window.FabrailPlot, UI = window.FabrailUI;
  if (!F || !P || !UI) return;
  var doc = document;
  var fmt = UI.fmt, fmtSI = UI.fmtSI;
  var C = F.CONST, D = F.CLASSES.drawer;
  var G = C.G0_MPS2;

  /* ---- constants of the preview (plan §4b warned variant; sources inline) ------------- */
  var EVENT0 = {
    magnitude: 7.2, r_epi_km: 60, depth_km: 12,       /* plan §4b warned variant */
    alert_t_s: 7.5,                                  /* synthetic network feeds at 7.5 s (scene example) */
    sigma_s: [1, 2],                                 /* S-ETA σ range behind the plan's 13.2–14.8 s deadlines */
    margin_s: 1.0,                                   /* deadline = t_S,5% − 1 s (plan §4b) */
    z95: 1.6448536269514722                          /* standard-normal 95 % point: 5 % quantile of t_S */
  };
  var K_SERIES = F.seriesK(C.K_HAND_N_PER_M, D.k_edge_n_per_m);
  var V_UNATTENDED = F.unattendedCloseSpeed(D.v_close_max_mps, C.F_STATIC_SENSED_N, D.mass_rated_kg, K_SERIES);

  var RANK_TEXT = {
    seatedorlocked: "Seated or Landed+Locked", seated_or_locked: "Seated or Landed+Locked",
    closedlevitated: "Closed-levitated", closed_levitated: "Closed-levitated",
    standoffhold: "Standoff/Hold", standoff_hold: "Standoff/Hold",
    frozenopen: "Frozen-open", frozen_open: "Frozen-open"
  };
  var REASON_TEXT = {
    deadline: "cannot finish before the deadline",
    deadline_missed: "missed the deadline",
    power: "bus power budget exhausted",
    admission: "an admission check failed",
    obstruction: "obstruction during the single attempt",
    gate: "the alert gate refused motion (no independent confirmation)"
  };
  var POLICY_TEXT = {
    SmoothRideThrough: "smooth ride-through 🧪", SeatAndHold: "seat and hold", SeatAndLatch: "seat and latch",
    StopAndLand: "stop and land", LandAndLock: "land and lock", PreCentre: "pre-centre (moat clear only)",
    SeismicPassive: "seismic passive", core: "core class action"
  };
  var BUILDING_TEXT = {
    fixed_base: "fixed base", fps_isolated: "FPS isolated (T 3 s)", haven_f: "HAVEN-F reduced platform",
    tower_sentinel: "supertall penthouse with SENTINEL"
  };
  var CLASS_COLOR = { drawer: "drawer", door: "door", wall: "wall", heavy: "heavy", haven_r: "exotic", sentinel: "exotic" };

  /* ---- small helpers ---------------------------------------------------------------- */
  function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }
  function num(v, d) { return typeof v === "number" && isFinite(v) ? v : d; }
  function interp(series, t) {
    if (!series || !series.t || !series.t.length) return NaN;
    var ts = series.t, ms = series.m, n = ts.length;
    if (t <= ts[0]) return ms[0];
    if (t >= ts[n - 1]) return ms[n - 1];
    var lo = 0, hi = n - 1;
    while (hi - lo > 1) { var mid = (lo + hi) >> 1; if (ts[mid] <= t) lo = mid; else hi = mid; }
    var f = (t - ts[lo]) / ((ts[hi] - ts[lo]) || 1);
    return ms[lo] + f * (ms[hi] - ms[lo]);
  }
  function pretty(kind) { return String(kind || "").replace(/_/g, " ").replace(/:/g, " · "); }
  function rankText(r) {
    if (!r) return "–";
    var k = String(r).replace(/[^a-z_]/gi, "").toLowerCase();
    return RANK_TEXT[k] || String(r);
  }
  function el(tag, cls, parent, text) {
    var e = doc.createElement(tag);
    if (cls) e.className = cls;
    if (text !== undefined) e.textContent = text;
    if (parent) parent.appendChild(e);
    return e;
  }
  function clear(node) { while (node && node.firstChild) node.removeChild(node.firstChild); }
  function emptyNote(node, text) { clear(node); el("div", "figure-empty", node, text); }
  function fmtT(t) { return isFinite(t) ? t.toFixed(2) + " s" : "–"; }

  /* ---- event geometry (PHYSICS §20) ------------------------------------------------- */
  function eventTimes(e) {
    var rHyp = Math.sqrt(e.r_epi_km * e.r_epi_km + e.depth_km * e.depth_km);
    return { r_hyp_km: rHyp, t_p_s: rHyp / C.VP_KMPS, t_s_s: rHyp / C.VS_KMPS };
  }

  /* ---- the preview: a closed-form plan in the villa_run schema ---------------------- */
  function preview(variant) {
    var g = eventTimes(EVENT0);
    var tP = g.t_p_s, tS = g.t_s_s;
    var protect = tP + C.ONSITE_DECISION_S;
    var dl = [tS - EVENT0.z95 * EVENT0.sigma_s[1] - EVENT0.margin_s, tS - EVENT0.z95 * EVENT0.sigma_s[0] - EVENT0.margin_s];
    var deadline = dl[0];                            /* conservative end of the range */
    var start = variant === "onsite" ? protect : EVENT0.alert_t_s;
    var tEnd = Math.ceil(tS + 13);
    var devs = [
      { id: "drawer_1", cls: "drawer", room: "kitchen", open: 0.30, contents: "glassware", badge: "V", fidelity: "carriage6", hero: true },
      { id: "drawer_2", cls: "drawer", room: "kitchen", open: 0, contents: "porcelain", badge: "S", fidelity: "travel1d" },
      { id: "drawer_3", cls: "drawer", room: "kitchen", open: 0, badge: "S", fidelity: "travel1d" },
      { id: "drawer_4", cls: "drawer", room: "kitchen", open: 0.15, contents: "porcelain", badge: "S", fidelity: "travel1d" },
      { id: "drawer_5", cls: "drawer", room: "kitchen", open: 0, badge: "S", fidelity: "travel1d" },
      { id: "drawer_6", cls: "drawer", room: "kitchen", open: 0, contents: "glassware", badge: "S", fidelity: "travel1d" },
      { id: "slider_1", cls: "drawer", kind: "slider", room: "kitchen", open: 0, badge: "S", fidelity: "travel1d", latch: true },
      { id: "slider_2", cls: "drawer", kind: "slider", room: "kitchen", open: 0, badge: "S", fidelity: "travel1d", latch: true },
      { id: "pantry_door", cls: "door", room: "pantry", egress: "non_rated", badge: "S", fidelity: "carriage6" },
      { id: "salon_wall", cls: "wall", room: "salon", badge: "S", fidelity: "travel1d" },
      { id: "plinth", cls: "haven_r", room: "gallery", badge: "R", fidelity: "platform3dof" }
    ];
    var timeline = [
      [EVENT0.alert_t_s, null, variant === "onsite" ? "network feed (single, synthetic): freeze only" : "alert:fused (two synthetic network feeds)"],
      [tP, null, "P arrival"],
      [protect, null, "PROTECT (on-site P pick + " + fmt(C.ONSITE_DECISION_S, 2) + " s)"],
      [deadline, null, "deadline (t_S,5% − 1 s, σ_S 2 s)"],
      [tS, null, "S arrival"]
    ];
    var out = [];
    for (var i = 0; i < devs.length; i++) {
      var s = devs[i], d = {
        id: s.id, cls: s.cls, kind: s.kind || s.cls, room: s.room, egress: s.egress || "none", badge: s.badge,
        fidelity: s.fidelity, contents: s.contents || null, deadline: deadline, x: null, dtg: null,
        start: NaN, tSecured: null, status: "planned", reason: null, openAtAlert: s.open || 0, hero: !!s.hero
      };
      if (s.cls === "drawer" && s.open > 0) {
        var prof = F.closeProfile({
          lengthM: s.open, vMps: V_UNATTENDED, aMaxMps2: D.a_max_mps2, jMaxMps3: D.j_max_mps3,
          tauS: D.tau_s, jTermMps3: D.j_term_mps3, dEpsM: D.d_eps_m, seatS: C.SEAT_TIME_S
        });
        d.policy = "SeatAndHold"; d.rank = "SeatedOrLocked"; d.start = start; d.profile = prof;
        var finish = start + prof.T;
        var xs = { t: [], m: [] };
        if (finish <= deadline) {
          d.tSecured = finish; d.status = "secured"; d.end = finish;
          for (var t = 0; t <= tEnd + 1e-9; t += 0.01) {
            xs.t.push(t);
            xs.m.push(t < start ? s.open : prof.sample(t - start).d);
          }
          d.x = xs;
          d.dtg = xs;
          timeline.push([start, s.id, "choreo:planned close " + fmtSI(s.open, "m", 2) + " at " + fmt(prof.v, 3) + " m/s"]);
          timeline.push([start + prof.tCaptureStart, s.id, "terminus capture (log-damped)"]);
          timeline.push([start + prof.T - prof.tSeat, s.id, "arrived; z-seat step"]);
          timeline.push([finish, s.id, "seated: SeatedOrLocked"]);
        } else {
          /* cannot finish: the core class action (freeze) and NotSecured(deadline) */
          d.status = "not_secured"; d.reason = "deadline"; d.end = deadline; d.policy = "core";
          d.x = { t: [0, tEnd], m: [s.open, s.open] };
          timeline.push([start, s.id, "choreo:not_secured deadline (needs " + fmt(prof.T, 3) + " s, has " + fmt(deadline - start, 3) + " s): freeze"]);
        }
      } else if (s.cls === "drawer") {
        d.policy = s.latch ? "SeatAndLatch" : "SeatAndHold"; d.rank = "SeatedOrLocked";
        d.status = "secured"; d.tSecured = EVENT0.alert_t_s; d.already = true;
        d.x = { t: [0, tEnd], m: [0, 0] };
      } else if (s.cls === "door") {
        d.policy = "StopAndLand"; d.rank = "SeatedOrLocked"; d.timingFromRun = true;
        timeline.push([start, s.id, "plan: stop, land, release ASC, never lock (egress); timing from the run"]);
      } else if (s.cls === "wall") {
        d.policy = "LandAndLock"; d.rank = "SeatedOrLocked"; d.timingFromRun = true;
        timeline.push([start, s.id, "plan: land and lock (off the egress path); timing from the run"]);
      } else {
        d.policy = "PreCentre"; d.rank = "–"; d.timingFromRun = true; d.note = "keeps isolating; pre-centring needs a ClearVerified moat, which the scene does not sense";
        timeline.push([start, s.id, "keeps isolating: no pre-centre without a ClearVerified moat"]);
      }
      out.push(d);
    }
    timeline.sort(function (a, b) { return a[0] - b[0]; });
    return {
      mode: "preview", variant: variant, building: null, seed: 42, label: "preview from closed forms",
      alertLabel: "synthetic/injected, not a live EEW feed",
      ev: {
        magnitude: EVENT0.magnitude, r_epi_km: EVENT0.r_epi_km, depth_km: EVENT0.depth_km,
        t_p: tP, t_s: tS, alert: EVENT0.alert_t_s, protect: protect, start: start,
        deadline: deadline, deadlineRange: dl, t_s_p05: deadline + EVENT0.margin_s
      },
      devices: out, waterfall: null, contents: null, ablation: null, bus: null,
      timeline: timeline, tStart: 0, tEnd: tEnd
    };
  }

  /* ---- the run: VILLA_RUN (schema fabrail.docs.villa_run/1) -------------------------- */
  /* A schema time series { t_s: [...], m: [...] } (t accepted too) as { t, m }, or null. */
  function series(o) {
    if (!o) return null;
    var t = o.t_s || o.t, m = o.m;
    return t && m && t.length && t.length === m.length ? { t: t, m: m } : null;
  }

  function fromRun(run) {
    var e = run.event || {};
    var g = eventTimes({ r_epi_km: num(e.r_epi_km, 60), depth_km: num(e.depth_km, 12) });
    var ev = {
      magnitude: num(e.magnitude, NaN), r_epi_km: num(e.r_epi_km, NaN), depth_km: num(e.depth_km, NaN),
      t_p: num(e.t_p_s, g.t_p_s), t_s: num(e.t_s_s, g.t_s_s), alert: num(e.alert_t_s, NaN),
      protect: num(e.protect_t_s, NaN), t_s_p05: num(e.t_s_p05_s, NaN),
      z_over_h: e.z_over_h, topo: e.topo_factor, pct: e.gm_percentile
    };
    ev.deadline = isFinite(ev.t_s_p05) ? ev.t_s_p05 - EVENT0.margin_s : NaN;
    ev.start = isFinite(ev.alert) ? ev.alert : isFinite(ev.protect) ? ev.protect : ev.t_p;
    var tl = (run.timeline || []).filter(function (r) { return r && isFinite(r[0]); })
      .map(function (r) { return [r[0], r[1] === undefined ? null : r[1], r[2]]; });
    tl.sort(function (a, b) { return a[0] - b[0]; });
    var tMax = ev.t_s + 12;
    var devices = (run.devices || []).map(function (s) {
      var x = series(s.x), dtg = series(s.distance_to_go_m);
      if (x) tMax = Math.max(tMax, x.t[x.t.length - 1]);
      var d = {
        id: s.id, cls: String(s["class"] || "drawer").toLowerCase(), room: s.room || "", egress: s.egress || "none",
        policy: s.policy || "", badge: s.badge || "S", fidelity: s.fidelity || "", rank: s.rank_target || "",
        deadline: num(s.deadline_s, ev.deadline), tSecured: num(s.t_secured_s, null), status: s.status || "",
        reason: s.reason || null, x: x, dtg: dtg, start: NaN, contents: null, hero: s.fidelity === "carriage6"
      };
      d.kind = /slider/.test(d.id) ? "slider" : d.cls;
      /* motion start: the device's first timeline entry at or after the alert, else the first
         1 mm step of x after the alert */
      var t0 = isFinite(ev.start) ? ev.start : 0;
      for (var i = 0; i < tl.length; i++) if (tl[i][1] === d.id && tl[i][0] >= t0 - 1e-9) { d.start = tl[i][0]; break; }
      if (x) {
        var x0 = interp(x, t0);
        d.openAtAlert = x0;
        for (var j = 0; j < x.t.length; j++) {
          if (x.t[j] >= t0 && Math.abs(x.m[j] - x0) > 1e-3) { if (!isFinite(d.start)) d.start = x.t[j]; d.moved = true; break; }
        }
      }
      if (!isFinite(d.start)) d.start = t0;
      d.end = d.tSecured !== null ? d.tSecured : d.deadline;
      d.already = d.status === "secured" && !d.moved && d.tSecured !== null && d.tSecured <= t0 + 0.05;
      return d;
    });
    for (var k = 0; k < tl.length; k++) tMax = Math.max(tMax, tl[k][0] + 1);
    var bus = run.bus_power && run.bus_power.t_s && run.bus_power.t_s.length
      ? { t: run.bus_power.t_s, w: run.bus_power.w, budget: num(run.bus_power.budget_w, NaN) } : null;
    if (bus) tMax = Math.max(tMax, bus.t[bus.t.length - 1]);
    var ws = run.waterfall && run.waterfall.stages && run.waterfall.stages.length ? run.waterfall.stages : null;
    return {
      mode: "data", variant: run.variant || "warned", building: run.building || null, seed: run.seed,
      version: run.fabrail_version, generatedBy: run.generated_by, label: run.label || "simulated replay",
      alertLabel: run.alert_label || "synthetic/injected, not a live EEW feed",
      ev: ev, devices: devices, waterfall: ws, contents: run.contents && run.contents.length ? run.contents : null,
      ablation: run.smoothing_ablation || null, bus: bus, timeline: tl, tStart: 0, tEnd: Math.ceil(tMax)
    };
  }

  /* ---- schematic building response (illustrative; PHYSICS §21, EXOTIC §X9) ------------ */
  var SHAKE_CFG = {
    fixed_base: { iso: null, label: "fixed base: storey mode T 0.35 s, ζ 0.05" },
    fps_isolated: { iso: { T: 3.0, zeta: 0.15, sky: false }, label: "FPS isolated: T 3 s, linear-equivalent ζ 0.15" },
    haven_f: { iso: { T: 6.0, zeta: 0.2, sky: true }, label: "HAVEN-F platform: T 6 s, skyhook ζ 0.2" }
  };
  var STOREY = { T: 0.35, zeta: 0.05 };             /* example storey mode (scene example building) */
  var shake = null;
  function buildShake(tEnd, tP, tS) {
    var dt = 0.01, sub = 10, h = dt / sub, n = Math.ceil(tEnd / dt) + 2;
    var ag = new Float64Array(n), dg = new Float64Array(n);
    /* illustrative ground acceleration: a weak P coda and a broadband S phase ≈ 30 s long */
    var comps = [[0.33, 1.0, 0.3], [0.55, 0.9, 2.1], [0.9, 1.0, 4.0], [1.35, 0.85, 1.2], [2.1, 0.7, 5.3], [3.0, 0.55, 0.7], [4.4, 0.4, 3.3], [6.2, 0.28, 2.6]];
    function a(t) {
      var v = 0, u = t - tP;
      if (u > 0) v += 0.12 * (1 - Math.exp(-u / 0.05)) * Math.exp(-u / 2) * (Math.sin(2 * Math.PI * 6.8 * u) + 0.6 * Math.sin(2 * Math.PI * 9.7 * u + 1));
      var s = t - tS;
      if (s > 0) {
        var env = (1 - Math.exp(-s / 0.8)) * Math.exp(-s / 9);
        for (var k = 0; k < comps.length; k++) v += env * comps[k][1] * Math.sin(2 * Math.PI * comps[k][0] * s + comps[k][2]);
      }
      return v;
    }
    var vg = 0, xg = 0;
    for (var i = 0; i < n; i++) {
      ag[i] = a(i * dt);
      if (i > 0) { vg += (ag[i] - vg / 4) * dt; xg += (vg - xg / 4) * dt; }
      dg[i] = xg;
    }
    var out = { dt: dt, n: n, ag: ag, dg: dg, cfg: {} };
    var ws = 2 * Math.PI / STOREY.T;
    for (var key in SHAKE_CFG) {
      var c = SHAKE_CFG[key].iso;
      var xb = new Float64Array(n), ab = new Float64Array(n), yd = new Float64Array(n), a1 = new Float64Array(n);
      var x = 0, xv = 0, y = 0, yv = 0, vgs = 0;
      for (var j = 0; j < n; j++) {
        var agj = ag[j], abj = agj;
        for (var q = 0; q < sub; q++) {
          if (c) {
            var wi = 2 * Math.PI / c.T;
            vgs += (agj - vgs / 4) * h;
            var damp = c.sky ? 2 * c.zeta * wi * (xv + vgs) : 2 * c.zeta * wi * xv;
            var xa = -agj - damp - wi * wi * x;
            xv += xa * h; x += xv * h;
            abj = agj + xa;
          }
          var ya = -abj - 2 * STOREY.zeta * ws * yv - ws * ws * y;
          yv += ya * h; y += yv * h;
          a1[j] = abj + ya;
        }
        xb[j] = x; ab[j] = abj; yd[j] = y;
      }
      out.cfg[key] = { xb: xb, ab: ab, yd: yd, a1: a1 };
    }
    /* one display scale for all configurations, so the comparison stays honest */
    var mD = 1e-9, mA = 1e-9;
    for (var k2 in out.cfg) {
      var o = out.cfg[k2];
      for (var m = 0; m < n; m++) {
        mD = Math.max(mD, Math.abs(dg[m] + o.xb[m] + o.yd[m]));
        mA = Math.max(mA, Math.abs(o.a1[m]));
      }
    }
    out.maxD = mD; out.maxA = mA;
    return out;
  }
  function shakeAt(key, t) {
    if (!shake) return { dg: 0, xb: 0, yd: 0, a1: 0, ag: 0 };
    var i = clamp(Math.round(t / shake.dt), 0, shake.n - 1), o = shake.cfg[key] || shake.cfg.fixed_base;
    return { dg: shake.dg[i], xb: o.xb[i], yd: o.yd[i], a1: o.a1[i], ag: shake.ag[i] };
  }

  /* ---- page state -------------------------------------------------------------------- */
  var RUN = window.VILLA_RUN;
  var hasRun = !!(RUN && !RUN.placeholder && RUN.devices && RUN.devices.length);
  var variant = "network";
  var M = hasRun ? fromRun(RUN) : preview(variant);
  var building = (M.building && SHAKE_CFG[M.building]) ? M.building : "fps_isolated";
  var charts = {};
  var current = 0, loop = null, pinned = false, lastChartT = -1, lastChartWall = 0;

  /* ---- source note, jumps, readouts --------------------------------------------------- */
  function renderSource() {
    var box = doc.getElementById("villa-source");
    if (!box) return;
    clear(box);
    var p = el("p", "", box);
    if (M.mode === "data") {
      el("strong", "", p, "Simulated replay. ");
      p.appendChild(doc.createTextNode("Seed " + M.seed + ", fabrail " + (M.version || "?") + ", variant " + pretty(M.variant) +
        ", building " + (BUILDING_TEXT[M.building] || pretty(M.building)) + ". Generated by "));
      el("code", "", p, M.generatedBy || "scripts/update_docs_run.py");
      p.appendChild(doc.createTextNode(". Every alert is " + M.alertLabel + "."));
      var e = M.ev, p2 = el("p", "small", box);
      p2.textContent = "M" + fmt(e.magnitude, 2) + " at an epicentral distance of " + fmt(e.r_epi_km, 3) + " km, " + fmt(e.depth_km, 3) +
        " km deep: t_P = " + fmtT(e.t_p) + ", t_S = " + fmtT(e.t_s) + ", alert at " + fmtT(e.alert) +
        (e.z_over_h !== undefined ? "; z/h " + fmt(e.z_over_h, 2) : "") + (e.topo !== undefined ? ", topographic factor " + fmt(e.topo, 3) : "") +
        (e.pct !== undefined ? ", ground-motion percentile " + fmt(e.pct, 2) : "") + ".";
    } else {
      el("strong", "", p, "Preview. ");
      p.appendChild(doc.createTextNode("The seed-42 simulated replay has not been generated yet (VILLA_RUN is still the placeholder), so this page shows the choreographer's plan from closed forms only: drawer closes from the terminus law at the unattended limit (" +
        fmt(V_UNATTENDED, 3) + " m/s for 48 kg) and event timing from the source geometry. The waterfall, contents, ablation and bus-power panels stay empty until the run exists."));
      var e2 = M.ev, q = el("p", "small", box);
      q.textContent = "M" + fmt(e2.magnitude, 2) + " at " + fmt(e2.r_epi_km, 3) + " km epicentral, " + fmt(e2.depth_km, 3) + " km deep: t_P = " + fmtT(e2.t_p) +
        ", t_S = " + fmtT(e2.t_s) + ". Deadline t_S,5% − 1 s = " + fmtT(e2.deadlineRange[0]) + " to " + fmtT(e2.deadlineRange[1]) +
        " for an S-arrival σ of 2 s to 1 s; the preview plans against the earlier one. Every alert is synthetic/injected, not a live EEW feed.";
    }
    var sim = doc.getElementById("villa-simlabel");
    if (sim) {
      sim.className = "sim-label";
      sim.textContent = M.mode === "data" ? "simulated replay" : "closed-form preview";
      sim.setAttribute("title", M.mode === "data" ? "Computed by the fabrail simulator (seed " + M.seed + "); not a hardware measurement"
        : "Evaluated from closed-form expressions checked against the Rust implementation; not a simulation run");
    }
  }

  function renderJumps() {
    var box = doc.getElementById("villa-jumps");
    if (!box) return;
    clear(box);
    var e = M.ev, list = [["alert", isFinite(e.alert) ? e.alert : e.protect], ["t_P", e.t_p], ["t_S", e.t_s], ["deadline", e.deadline]];
    list.forEach(function (j) {
      if (!isFinite(j[1])) return;
      var b = el("button", "btn btn--ghost", box, j[0]);
      b.type = "button";
      b.setAttribute("aria-label", "Jump to " + j[0] + " at " + fmtT(j[1]));
      b.addEventListener("click", function () { pause(); setTime(Math.max(M.tStart, j[1] - 0.3)); });
    });
  }

  function securedAt(d, t) { return d.status === "secured" && d.tSecured !== null && t >= d.tSecured - 1e-9; }

  function narration() {
    var e = M.ev, out = [];
    var nOpen = M.devices.filter(function (d) { return d.cls === "drawer" && d.openAtAlert > 0.005; }).length;
    out.push([M.tStart, nOpen ? "Before the earthquake: " + nOpen + " drawer" + (nOpen > 1 ? "s are" : " is") + " open in the kitchen." : "Before the earthquake: the house is quiet."]);
    if (isFinite(e.alert)) {
      out.push([e.alert, M.mode === "preview" && M.variant === "onsite"
        ? "A single synthetic network feed arrives: it may only freeze, land or lock. Motion waits for the local P pick."
        : "Synthetic alert (" + M.alertLabel + "): the choreographer plans every device before the S-wave."]);
    }
    out.push([e.t_p, "P-wave at the house: the structure accelerometer picks it."]);
    if (isFinite(e.protect) && (M.mode === "data" || M.variant === "onsite")) out.push([e.protect, "PROTECT from the on-site P pick: " + fmt(e.deadline - e.protect, 2) + " s to the deadline."]);
    M.devices.forEach(function (d) {
      if (d.status === "secured" && !d.already && d.tSecured !== null) {
        if (isFinite(d.start) && d.cls === "drawer" && (d.profile || d.moved)) out.push([d.start + 0.01, "Closing " + d.id + " toward its seat at the unattended limit; the last few centimetres follow the log-damped terminus."]);
        out.push([d.tSecured, d.id + " secured at " + fmtT(d.tSecured) + " (" + rankText(d.rank) + ")."]);
      }
      if (d.status === "not_secured") out.push([Math.min(d.end, isFinite(d.start) ? d.start + 0.02 : d.end), d.id + " cannot be secured: " + (REASON_TEXT[d.reason] || pretty(d.reason)) + ". It takes the core class action and is reported."]);
    });
    if (isFinite(e.deadline)) out.push([e.deadline, "Deadline: the 5 % quantile of the predicted S arrival minus 1 s."]);
    out.push([e.t_s, "S-wave: the house sways; the secured devices hold their rank."]);
    out.push([e.t_s + 10, "Shaking continues. Nothing re-opens before QUIET, a manual INSPECT reset and a local restore command."]);
    out.sort(function (a, b) { return a[0] - b[0]; });
    return out;
  }
  var NARR = narration();

  function updateReadouts(t) {
    var e = M.ev;
    function put(k, v) { var n = doc.querySelector('[data-read="' + k + '"]'); if (n) n.textContent = v; }
    var phase = t < e.t_p ? (isFinite(e.alert) && t >= e.alert ? "alerted" : "quiet") : t < e.t_s ? "P-wave" : "S-wave shaking";
    put("phase", phase);
    put("phaseNote", t < e.t_p ? "t_P at " + fmtT(e.t_p) : t < e.t_s ? "t_S at " + fmtT(e.t_s) : "S arrived at " + fmtT(e.t_s));
    var tgt = M.devices.filter(function (d) { return d.status === "secured" || d.status === "not_secured"; });
    var sec = tgt.filter(function (d) { return securedAt(d, t); }).length;
    put("secured", tgt.length ? sec + " / " + tgt.length : "–");
    var ns = M.devices.filter(function (d) { return d.status === "not_secured"; }).length;
    var pend = M.devices.filter(function (d) { return d.status === "planned"; }).length;
    put("securedNote", (ns ? ns + " not secured (reported)" : "none reported not secured") + (pend ? "; " + pend + " timed by the run" : ""));
    var start = M.mode === "preview" ? e.start : (isFinite(e.alert) ? e.alert : e.protect);
    put("budget", isFinite(e.deadline) && isFinite(start) ? fmt(e.deadline - start, 3) + " s" : "–");
    put("budgetNote", M.mode === "preview" ? (M.variant === "onsite" ? "on-site P only (plan: 1.8–3.4 s)" : "after the network alert (plan: 5.7–7.3 s)") : "from the alert to the deadline");
    if (M.bus) {
      put("bus", fmt(interp({ t: M.bus.t, m: M.bus.w }, t), 3) + " W");
      put("busNote", "budget " + fmt(M.bus.budget, 4) + " W");
    } else {
      put("bus", "–");
      put("busNote", "from the run");
    }
  }

  /* ---- charts ------------------------------------------------------------------------ */
  function timeRefs(t) {
    var e = M.ev, r = [];
    if (isFinite(e.alert)) r.push({ axis: "x", value: e.alert, label: "alert", color: "accent", dash: [2, 3], row: 0 });
    if (M.mode === "preview" && M.variant === "onsite") r.push({ axis: "x", value: e.protect, label: "PROTECT", color: "accent", dash: [2, 3], row: 1 });
    r.push({ axis: "x", value: e.t_p, label: "t_P", color: "p", dash: [4, 3], row: 1 });
    if (isFinite(e.deadline)) r.push({ axis: "x", value: e.deadline, label: "deadline", color: "ref", dash: [6, 3], row: 0 });
    r.push({ axis: "x", value: e.t_s, label: "t_S", color: "s", dash: [4, 3], row: 1 });
    if (t !== undefined && t !== null) r.push({ axis: "x", value: t, color: "ink2", dash: [], width: 1 });
    return r;
  }

  function ganttSpec() {
    var e = M.ev, rows = [], held = [];
    M.devices.forEach(function (d) {
      if (d.already) { held.push(d.id); return; }
      var colr = CLASS_COLOR[d.cls] || "exotic", spans = [];
      if (d.status === "not_secured") {
        spans.push({ from: d.start, to: Math.max(d.start + 0.05, d.end), style: "hatch", color: "ref", label: "not secured: " + (REASON_TEXT[d.reason] || pretty(d.reason)) });
      } else if (d.status === "secured" && d.tSecured !== null) {
        spans.push({ from: d.start, to: Math.max(d.start + 0.05, d.tSecured), color: colr, label: (POLICY_TEXT[d.policy] || pretty(d.policy)) + " → " + fmtT(d.tSecured) });
      } else if (d.timingFromRun) {
        spans.push({ from: e.start, to: e.deadline, color: "muted", label: (POLICY_TEXT[d.policy] || pretty(d.policy)) + ": timing from the run" });
      }
      rows.push({ label: d.id, color: colr, spans: spans });
    });
    var bands = [];
    var cap = doc.getElementById("gantt-held");
    if (cap) cap.textContent = held.length ? "Already at their target rank when the alert arrived, so not moved: " + held.join(", ") + "." : "";
    return {
      kind: "gantt", title: "Device timeline against the P and S arrivals",
      desc: "Gantt chart, one row per device, seconds after origin; markers for the alert, the P arrival, the deadline and the S arrival.",
      x: { label: "time after origin", unit: "s", min: Math.max(0, e.alert - 1.5 || 0), max: e.t_s + 2.5 },
      rows: rows, refs: timeRefs(), bands: bands, table: true
    };
  }

  function movingDrawers() {
    return M.devices.filter(function (d) { return d.cls === "drawer" && d.x && (d.openAtAlert > 0.005 || d.moved); });
  }

  var DASHES = [undefined, [7, 4], [2, 3], [10, 3, 2, 3]];
  function xSpec(reveal, t) {
    var mv = movingDrawers(), series = [];
    mv.forEach(function (d, i) {
      var data = [];
      for (var k = 0; k < d.x.t.length; k++) data.push([d.x.t[k], d.x.m[k]]);
      series.push({ key: d.id, label: d.id + (d.status === "not_secured" ? " (not secured)" : ""), color: "drawer", dash: DASHES[i % DASHES.length], data: data, endLabel: d.id });
    });
    var e = M.ev;
    return {
      kind: "line", title: "Open drawers: position along the rail",
      desc: "Line chart of drawer position against time after origin; 0 is the seat.",
      x: { label: "time after origin", unit: "s", min: Math.max(0, (isFinite(e.alert) ? e.alert : e.t_p) - 2), max: e.t_s + 2 },
      y: { label: "opening", unit: "m", si: true, min: 0 },
      series: series, refs: timeRefs(t), reveal: reveal, height: 250, table: true, tableRows: 30
    };
  }

  /* Least-squares fit of ln d against t over a window of distances (the "log-damped" check). */
  function semilogFit(d, lo, hi) {
    var s = d.dtg || d.x;
    if (!s) return null;
    var n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0, t0 = NaN, t1 = NaN;
    for (var i = 0; i < s.t.length; i++) {
      var v = s.m[i];
      if (s.t[i] < d.start || !(v >= lo && v <= hi)) continue;
      var x = s.t[i], y = Math.log(v);
      n++; sx += x; sy += y; sxx += x * x; sxy += x * y; syy += y * y;
      if (!isFinite(t0)) t0 = x;
      t1 = x;
    }
    if (n < 5) return null;
    var cov = sxy - sx * sy / n, vx = sxx - sx * sx / n, vy = syy - sy * sy / n;
    if (!(vx > 0 && vy > 0)) return null;
    var slope = cov / vx, icpt = (sy - slope * sx) / n;
    return { slope: slope, tau: -1 / slope, r2: (cov * cov) / (vx * vy), n: n, t0: t0, t1: t1, icpt: icpt };
  }

  function semilogSpec(reveal) {
    var mv = movingDrawers().filter(function (d) { return d.status !== "not_secured"; }), series = [], lo = Infinity, hi = -Infinity;
    mv.forEach(function (d, i) {
      var s = d.dtg || d.x, data = [];
      for (var k = 0; k < s.t.length; k++) if (s.t[k] >= d.start - 0.3 && s.m[k] > 0) data.push([s.t[k], s.m[k]]);
      if (data.length) { lo = Math.min(lo, data[0][0]); hi = Math.max(hi, data[data.length - 1][0]); }
      series.push({ key: d.id, label: d.id + " distance to go", color: "drawer", dash: DASHES[i % DASHES.length], data: data });
    });
    var cap = doc.getElementById("semilog-fit");
    if (mv.length) {
      var d0 = mv[0];
      if (M.mode === "preview" && d0.profile) {
        var pr = d0.profile, tb = d0.start + pr.tCaptureStart, tq = d0.start + pr.tQuinticStart, ref = [];
        for (var q = 0; q <= 40; q++) {
          var tt = tb - 0.3 + (tq - tb + 0.5) * q / 40;
          ref.push([tt, pr.terminus.dB * Math.exp(-(tt - tb) / D.tau_s)]);
        }
        series.push({ key: "exp", label: "d_b·e^(−t/τ), τ = " + fmtSI(D.tau_s, "s", 3), color: "muted", dash: [5, 4], width: 1.5, data: ref });
        if (cap) cap.textContent = "In the preview the capture is the closed form itself (τ = " + fmtSI(D.tau_s, "s", 3) + ", d_b = " + fmtSI(pr.terminus.dB, "m", 3) +
          "), so the fit is exact by construction; the run's fit of the plant truth replaces this.";
      } else {
        var fit = semilogFit(d0, 5e-4, 15e-3);
        if (fit) {
          series.push({ key: "fit", label: "fit: τ̂ = " + fmtSI(fit.tau, "s", 3) + ", R² = " + fit.r2.toFixed(5), color: "muted", dash: [5, 4], width: 1.5,
            data: [[fit.t0 - 0.1, Math.exp(fit.icpt + fit.slope * (fit.t0 - 0.1))], [fit.t1 + 0.1, Math.exp(fit.icpt + fit.slope * (fit.t1 + 0.1))]] });
          if (cap) cap.textContent = d0.id + ", least-squares fit of ln d over 0.5–15 mm (" + fit.n + " samples): τ̂ = " + fmtSI(fit.tau, "s", 4) + ", R² = " + fit.r2.toFixed(5) + " (FAB-HAP-013 asks ≥ 0.999 on the capture window).";
        } else if (cap) cap.textContent = "Too few capture samples in the record to fit.";
      }
    } else if (cap) cap.textContent = "No drawer moved in this record.";
    return {
      kind: "semilog", title: "Distance to go during the automatic closes",
      desc: "Semilog chart of each closing drawer's distance to its seat against time after origin; the capture phase is a straight line.",
      x: { label: "time after origin", unit: "s", min: isFinite(lo) ? lo : 0, max: isFinite(hi) ? hi + 0.1 : 1 },
      y: { label: "distance to go", unit: "m", si: true, min: 1e-4, max: 0.5 },
      series: series, refs: [{ axis: "y", value: D.d_eps_m, label: "d_ε " + fmtSI(D.d_eps_m, "m", 2), color: "muted", dash: [2, 4] }],
      reveal: reveal, height: 250, table: true, tableRows: 30
    };
  }

  function busSpec(t) {
    var data = [];
    for (var i = 0; i < M.bus.t.length; i++) data.push([M.bus.t[i], M.bus.w[i]]);
    var r = timeRefs(t);
    if (isFinite(M.bus.budget)) r.push({ axis: "y", value: M.bus.budget, label: "budget " + fmt(M.bus.budget, 4) + " W", color: "ref", dash: [6, 3] });
    return {
      kind: "area", title: "House DC bus power", desc: "Area chart of the summed bus power against time, with the budget line.",
      x: { label: "time after origin", unit: "s", min: M.tStart, max: M.tEnd }, y: { label: "bus power", unit: "W", zero: true },
      series: [{ key: "bus", label: "summed bus power", color: "accent", data: data }], refs: r, height: 240, table: true
    };
  }

  function transferSpec() {
    function fixedFn(f) { return F.transmissibility(f * STOREY.T, STOREY.zeta); }
    function fpsFn(f) { return F.transmissibility(f * 3.0, 0.15); }
    function havenFn(f) { return F.tSky(f * 6.0, 0.2); }
    var list = [
      { key: "fixed_base", label: "fixed base: storey T 0.35 s, ζ 0.05 (example values)", fn: fixedFn },
      { key: "fps_isolated", label: "FPS isolated: T 3 s, linear-equivalent ζ 0.15 (illustrative)", fn: fpsFn },
      { key: "haven_f", label: "HAVEN-F: T 6 s, skyhook ζ 0.2", fn: havenFn }
    ];
    var series = list.map(function (s) {
      var on = s.key === building;
      return { key: s.key, label: s.label, fn: s.fn, samples: 240, color: on ? "accent" : "muted", width: on ? 2.5 : 1.25, dash: on ? undefined : [5, 4] };
    });
    return {
      kind: "loglog", title: "Transmissibility from ground to floor",
      desc: "Log-log chart of the closed-form transmissibility of three building configurations against frequency; the selected one is highlighted.",
      x: { label: "frequency", unit: "Hz", min: 0.05, max: 20 }, y: { label: "floor / ground", min: 0.003, max: 20 },
      series: series, refs: [{ axis: "y", value: 1, color: "muted", dash: [2, 4] }], height: 250, table: false
    };
  }

  function renderContents() {
    var box = doc.getElementById("villa-contents");
    if (!box) return;
    if (!M.contents) { emptyNote(box, M.mode === "data" ? "This record carries no contents entries." : "Peak cone utilisation per item appears with the seed-42 run."); return; }
    clear(box);
    var ul = el("ul", "util-list", box), maxU = 1.2;
    M.contents.forEach(function (c) { maxU = Math.max(maxU, num(c.cone_utilisation_max, 0) * 1.05); });
    M.contents.forEach(function (c) {
      var u = num(c.cone_utilisation_max, NaN), li = el("li", "", ul);
      el("span", "util-name", li, c.device + " · " + c.kind + " · μ " + fmt(num(c.mu_s, NaN), 3));
      var tr = el("span", "util-track", li);
      tr.setAttribute("role", "img");
      tr.setAttribute("aria-label", "peak utilisation " + fmt(u, 3) + " of the friction cone");
      var fill = el("span", "util-fill", tr);
      fill.style.width = clamp(u / maxU, 0, 1) * 100 + "%";
      fill.style.setProperty("--key", u > 1 ? "var(--plot-ref)" : "var(--class-drawer)");
      var mark = el("span", "util-mark", tr);
      mark.style.left = (100 / maxU) + "%";
      el("span", "util-val", li, fmt(u, 3));
      var n = num(c.slip_events, 0);
      el("span", "util-note", li, (n === 0 ? "no Coulomb slip event in the sliding-block model" : n + " slip event" + (n > 1 ? "s" : "") + " in the sliding-block model") +
        (c.rocking_flag ? "; rocking flag raised (abs(a_h) > g·b/h)" : ""));
    });
    var ax = el("div", "util-axis", box);
    el("span", "", ax, "");
    var mid = el("span", "", ax);
    el("span", "", mid, "0");
    el("span", "", mid, "slip onset at 1 (red tick)");
    el("span", "", mid, fmt(maxU, 2));
    el("span", "", ax, "");
  }

  function renderWaterfall() {
    var box = doc.getElementById("villa-waterfall");
    if (!box) return;
    if (!M.waterfall) { emptyNote(box, M.mode === "data" ? "This record carries no waterfall stages." : "The waterfall appears with the seed-42 run: its values come from the run only."); return; }
    clear(box);
    var row = el("div", "fig-row", box);
    var a = el("div", "", row), b = el("div", "", row);
    el("div", "fig-sub", a, "Peak horizontal abs(a_xy)");
    el("div", "fig-sub", b, "Peak vertical abs(a_z)");
    var ca = el("div", "", a), cb = el("div", "", b);
    var barsXY = [], barsZ = [];
    M.waterfall.forEach(function (s) {
      var note = s.note || "";
      if (isFinite(s.a_xy_mps2)) barsXY.push({ label: s.name, value: s.a_xy_mps2, color: "ink2", note: note });
      if (isFinite(s.a_z_mps2)) barsZ.push({ label: s.name, value: s.a_z_mps2, color: "muted", note: note });
    });
    charts.wxy = P.chart(ca, { kind: "bar", orient: "h", title: "Peak horizontal acceleration per stage", desc: "Horizontal bars, one per stage of the compensation stack.", y: { unit: "m/s²" }, bars: barsXY, table: true });
    if (barsZ.length) charts.wz = P.chart(cb, { kind: "bar", orient: "h", title: "Peak vertical acceleration per stage", desc: "Horizontal bars, one per stage; stages without a vertical value are omitted.", y: { unit: "m/s²" }, bars: barsZ, table: true });
    else el("div", "figure-empty", cb, "No vertical values in this record.");
  }

  function renderAblation() {
    var box = doc.getElementById("villa-ablation");
    if (!box) return;
    var a = M.ablation;
    if (!a) { emptyNote(box, M.mode === "data" ? "This record carries no paired smoothing runs." : "The paired on/off runs appear with the seed-42 run."); return; }
    clear(box);
    var wrap = el("div", "table-wrap", box), tbl = el("table", "ablation", wrap);
    var hr = el("tr", "", el("thead", "", tbl));
    ["Building", "x-slip events, smoothing off", "x-slip events, smoothing on", "Change"].forEach(function (h, i) { el("th", i ? "num" : "", hr, h).scope = "col"; });
    var tb = el("tbody", "", tbl);
    [["fixed_base", "fixed base"], ["isolated", "isolated"]].forEach(function (k) {
      var r = a[k[0]];
      if (!r) return;
      var tr = el("tr", "", tb), on = num(r.x_slip_on, NaN), off = num(r.x_slip_off, NaN);
      el("td", "", tr, k[1]);
      el("td", "num", tr, fmt(off, 4));
      el("td", "num", tr, fmt(on, 4));
      el("td", "num", tr, isFinite(on) && isFinite(off) ? (off === on ? "no change" : (on < off ? "−" : "+") + fmt(Math.abs(off - on), 4)) : "–");
    });
  }

  function renderBus() {
    var box = doc.getElementById("villa-bus");
    if (!box) return;
    if (!M.bus) {
      emptyNote(box, M.mode === "data" ? "This record carries no bus-power series." : "Bus power appears with the seed-42 run; the budget is stated in the scene JSON.");
      return;
    }
    clear(box);
    charts.bus = P.chart(box, busSpec(null));
    var peak = 0, over = 0;
    for (var i = 0; i < M.bus.w.length; i++) {
      peak = Math.max(peak, M.bus.w[i]);
      if (i && M.bus.w[i] > M.bus.budget) over += M.bus.t[i] - M.bus.t[i - 1];
    }
    var cap = doc.getElementById("bus-caption");
    if (cap) cap.textContent = "Peak " + fmt(peak, 4) + " W against a stated budget of " + fmt(M.bus.budget, 4) + " W" +
      (over > 0 ? "; over budget for " + fmt(over, 3) + " s, reported, not hidden." : "; within budget at every sync step.") +
      " Moves are staggered, deadline-constrained and parallel; devices that cannot finish get the core class action and are reported NotSecured.";
  }

  function renderTable() {
    var tb = doc.querySelector("#device-table tbody");
    if (!tb) return;
    clear(tb);
    M.devices.forEach(function (d) {
      var tr = el("tr", "", tb);
      var td0 = el("td", "", tr), dot = el("span", "class-dot", td0);
      dot.style.setProperty("--key", "var(--class-" + (CLASS_COLOR[d.cls] || "exotic") + ")");
      td0.appendChild(doc.createTextNode(d.id));
      el("td", "", tr, d.egress === "none" ? "no" : pretty(d.egress));
      el("td", "", tr, POLICY_TEXT[d.policy] || pretty(d.policy) || "–");
      var tdB = el("td", "", tr);
      tdB.appendChild(UI.badge(d.badge));
      el("td", "", tr, d.fidelity || "–");
      el("td", "", tr, rankText(d.rank));
      el("td", "num", tr, fmtT(d.deadline));
      el("td", "num", tr, d.tSecured !== null ? fmtT(d.tSecured) : "–");
      var st = el("td", "reason", tr);
      if (d.status === "secured") el("span", "status-ok", st, d.already ? "✓ at rank" : "✓ secured");
      else if (d.status === "not_secured") {
        el("span", "status-no", st, "✗ NotSecured");
        st.appendChild(doc.createTextNode(": " + (REASON_TEXT[d.reason] || pretty(d.reason))));
      } else {
        el("span", "status-plan", st, "plan");
        st.appendChild(doc.createTextNode(d.note ? ": " + d.note : d.timingFromRun ? ": timing from the run" : ""));
      }
    });
  }

  var logItems = [];
  function renderLog() {
    var ol = doc.getElementById("event-log");
    if (!ol) return;
    clear(ol);
    logItems = M.timeline.map(function (r) {
      var li = el("li", "", ol);
      el("span", "t", li, r[0].toFixed(3) + " s");
      li.appendChild(doc.createTextNode((r[1] ? r[1] + " · " : "") + pretty(r[2])));
      return { t: r[0], li: li };
    });
  }
  var lastLogIdx = -2;
  function updateLog(t) {
    var idx = -1;
    for (var i = 0; i < logItems.length; i++) if (logItems[i].t <= t + 1e-9) idx = i;
    if (idx === lastLogIdx) return;
    lastLogIdx = idx;
    for (var j = 0; j < logItems.length; j++) {
      if (j > idx) logItems[j].li.setAttribute("data-future", ""); else logItems[j].li.removeAttribute("data-future");
      if (j === idx) logItems[j].li.setAttribute("data-current", ""); else logItems[j].li.removeAttribute("data-current");
    }
    var ol = doc.getElementById("event-log");
    if (ol && idx >= 0) {
      var item = logItems[idx].li;
      ol.scrollTop = Math.max(0, item.offsetTop - ol.clientHeight / 2);
    }
  }

  function renderCharts() {
    ["gantt", "x", "semi", "bus", "wxy", "wz", "transfer"].forEach(function (k) { if (charts[k]) { charts[k].destroy(); charts[k] = null; } });
    var g = doc.getElementById("villa-gantt");
    if (g) { clear(g); charts.gantt = P.chart(g, ganttSpec()); }
    var xEl = doc.getElementById("villa-x");
    var mv = movingDrawers();
    if (xEl) {
      clear(xEl);
      if (mv.length) charts.x = P.chart(xEl, xSpec(undefined, null));
      else emptyNote(xEl, "No drawer was open when the alert arrived.");
    }
    var sEl = doc.getElementById("villa-semilog");
    if (sEl) {
      clear(sEl);
      if (mv.filter(function (d) { return d.status !== "not_secured"; }).length) charts.semi = P.chart(sEl, semilogSpec(undefined));
      else { emptyNote(sEl, M.mode === "data" ? "No drawer completed a close in this record." : "No drawer completed a close in this plan."); var cap = doc.getElementById("semilog-fit"); if (cap) cap.textContent = ""; }
    }
    var tEl = doc.getElementById("villa-transfer");
    if (tEl) { clear(tEl); charts.transfer = P.chart(tEl, transferSpec()); }
    renderContents();
    renderWaterfall();
    renderAblation();
    renderBus();
    renderTable();
    renderLog();
    lastLogIdx = -2;
    lastChartT = -1;
  }

  function updateCharts(t, force) {
    var now = Date.now();
    if (!force && (Math.abs(t - lastChartT) < 0.05 || now - lastChartWall < 110)) return;
    lastChartT = t; lastChartWall = now;
    var playing = loop && loop.running();
    var rev = playing ? Math.max(t, 1e-6) : undefined;
    if (charts.gantt) charts.gantt.update({ refs: timeRefs(t) });
    if (charts.x) charts.x.update({ refs: timeRefs(t), reveal: rev });
    if (charts.semi) charts.semi.update({ reveal: rev });
    if (charts.bus) {
      var br = timeRefs(t);
      if (isFinite(M.bus.budget)) br.push({ axis: "y", value: M.bus.budget, label: "budget " + fmt(M.bus.budget, 4) + " W", color: "ref", dash: [6, 3] });
      charts.bus.update({ refs: br });
    }
  }

  /* ---- friction-cone explorer (closed form; PLAN §4b, FAB-EXA-009) --------------------- */
  var cone = { mu: 0.35, ah: 1.5, az: 0, bh: 0.35 };
  var coneCanvas = doc.getElementById("cone-canvas"), coneState = doc.getElementById("cone-state");
  var coneCtl = null;
  function drawCone() {
    if (!coneCanvas) return;
    var w = Math.max(200, Math.round(coneCanvas.clientWidth || 300)), h = 250;
    var cv = P.setupCanvas(coneCanvas, w, h), ctx = cv.ctx;
    var ink = P.color("ink"), ink2 = P.color("ink2"), muted = P.color("muted"), ref = P.color("ref"), good = P.color("good"), glass = P.color("drawer"), sans = P.theme().sans;
    ctx.fillStyle = P.color("deep"); ctx.fillRect(0, 0, w, h);
    var geff = G + cone.az, u = geff > 0 ? cone.ah / (cone.mu * geff) : Infinity, slip = u > 1, rock = cone.ah > G * cone.bh;
    var cx = w * 0.5, baseY = h - 46, L = Math.min(h * 0.6, w * 0.4);
    /* drawer floor */
    ctx.fillStyle = P.alpha(P.color("line"), 0.8); ctx.fillRect(16, baseY, w - 32, 6);
    ctx.fillStyle = muted; ctx.font = "400 11px " + sans; ctx.textAlign = "left"; ctx.fillText("drawer floor", 18, baseY + 20);
    /* glass */
    var gw = 26, gh = 44;
    ctx.strokeStyle = glass; ctx.lineWidth = 2; ctx.fillStyle = P.alpha(glass, 0.15);
    ctx.beginPath(); ctx.moveTo(cx - gw / 2, baseY); ctx.lineTo(cx - gw / 2 + 4, baseY - gh); ctx.lineTo(cx + gw / 2 - 4, baseY - gh); ctx.lineTo(cx + gw / 2, baseY); ctx.closePath(); ctx.fill(); ctx.stroke();
    /* the cone: half-angle atan(μ) about the effective vertical (drawn up from the contact) */
    var half = Math.atan(cone.mu), oy = baseY;
    ctx.fillStyle = P.alpha(good, 0.12); ctx.strokeStyle = P.alpha(good, 0.7); ctx.lineWidth = 1.25; ctx.setLineDash([5, 4]);
    ctx.beginPath(); ctx.moveTo(cx, oy); ctx.lineTo(cx - Math.sin(half) * L, oy - Math.cos(half) * L); ctx.lineTo(cx + Math.sin(half) * L, oy - Math.cos(half) * L); ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.setLineDash([]);
    /* required reaction: m(g + a_z) up and m·a_h sideways; its angle from vertical against atan(μ) */
    var ang = geff > 0 ? Math.atan2(cone.ah, geff) : Math.PI / 2, rl = L * 0.92;
    var ex = cx + Math.sin(ang) * rl, ey = oy - Math.cos(ang) * rl;
    ctx.strokeStyle = slip ? ref : ink; ctx.fillStyle = slip ? ref : ink; ctx.lineWidth = 2.2;
    ctx.beginPath(); ctx.moveTo(cx, oy); ctx.lineTo(ex, ey); ctx.stroke();
    var ah = Math.atan2(ey - oy, ex - cx);
    ctx.beginPath(); ctx.moveTo(ex, ey); ctx.lineTo(ex - 9 * Math.cos(ah - 0.4), ey - 9 * Math.sin(ah - 0.4)); ctx.lineTo(ex - 9 * Math.cos(ah + 0.4), ey - 9 * Math.sin(ah + 0.4)); ctx.closePath(); ctx.fill();
    ctx.fillStyle = ink2; ctx.font = "600 11px " + sans; ctx.textAlign = "left";
    ctx.fillText("required reaction", Math.min(w - 110, ex + 6), Math.max(14, ey - 4));
    ctx.fillStyle = muted; ctx.font = "400 11px " + sans; ctx.textAlign = "center";
    ctx.fillText("cone half-angle atan(μ) = " + fmt(half * 180 / Math.PI, 3) + "°", cx, 16);
    ctx.textAlign = "left";
    ctx.fillText("g + a_z = " + fmt(geff, 3) + " m/s²", 18, 34);
    var state = slip ? "slip" : "stick";
    if (coneState) {
      clear(coneState);
      var s = el("strong", "", coneState, slip ? "Slips" : "Sticks");
      s.setAttribute("data-state", state);
      coneState.appendChild(doc.createTextNode(": utilisation abs(a_h)/(μ(g + a_z)) = " + (isFinite(u) ? fmt(u, 3) : "∞") +
        (rock ? "; rocking flag (abs(a_h) > g·b/h = " + fmt(G * cone.bh, 3) + " m/s²)" : "; no rocking flag")));
    }
  }
  function initCone() {
    var ctlBox = doc.getElementById("cone-controls");
    if (!ctlBox || !coneCanvas) return;
    coneCtl = UI.bindSliders(ctlBox, function (v) {
      cone.mu = v.mu; cone.ah = v.ah; cone.az = v.az; cone.bh = v.bh;
      drawCone();
    });
    UI.segmented(doc.getElementById("cone-presets"), function (val) {
      var mu = { glassware: 0.35, porcelain: 0.40, lined: 0.80 }[val];
      if (mu && coneCtl) coneCtl.set("mu", mu);
    });
    P.onTheme(drawCone);
    if (window.ResizeObserver) new ResizeObserver(function () { drawCone(); }).observe(coneCanvas);
    else window.addEventListener("resize", drawCone);
  }

  /* ---- the stage --------------------------------------------------------------------- */
  var stage = doc.getElementById("villa-stage"), canvas = doc.getElementById("villa-canvas");
  var chip = doc.getElementById("villa-chip"), clock = doc.getElementById("villa-clock");
  var scrub = doc.getElementById("villa-scrub"), playBtn = doc.getElementById("villa-play");
  var col = {}, geo = null, cv = null, lastNarr = -1;

  function colours() {
    ["deep", "panel", "ink", "ink2", "muted", "line", "grid", "axis", "accent", "p", "s", "drawer", "door", "wall", "exotic", "good", "ref", "frontier"].forEach(function (n) { col[n] = P.color(n); });
    col.flux = P.color("--flux"); col.steel = P.color("--steel"); col.sans = P.theme().sans; col.mono = P.theme().mono;
  }

  function layout() {
    if (!stage || !canvas) return;
    var w = Math.max(300, Math.round(stage.clientWidth || 800));
    var narrow = w < 640;
    var h = Math.round(narrow ? clamp(w * 1.12, 380, 560) : clamp(w * 0.46, 340, 540));
    cv = P.setupCanvas(canvas, w, h);
    var groundY = Math.round(h * (narrow ? 0.86 : 0.84));
    var hx0 = narrow ? 10 : Math.round(w * 0.1), hx1 = narrow ? w - 10 : Math.round(w * 0.965);
    var roofH = 10, baseH = 14;
    var top = narrow ? 44 : 96;             /* desktop: room for the phase chip and the trace */
    var sh = Math.floor((groundY - baseH - roofH - top) / 2);
    geo = { w: w, h: h, narrow: narrow, groundY: groundY, hx0: hx0, hx1: hx1, hw: hx1 - hx0, sh: sh, roofH: roofH, baseH: baseH, top: top };
  }

  function line(ctx, x0, y0, x1, y1) { ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke(); }
  function text(ctx, s, x, y, o) {
    o = o || {};
    ctx.font = (o.weight || "400") + " " + (o.size || 11) + "px " + (o.mono ? col.mono : col.sans);
    ctx.fillStyle = o.color || col.muted;
    ctx.textAlign = o.align || "left";
    ctx.textBaseline = o.base || "alphabetic";
    ctx.fillText(s, x, y);
  }
  function statusMark(ctx, x, y, d, t) {
    var r = 7;
    var st = d.status === "not_secured" && t >= d.start ? "no" : securedAt(d, t) ? "ok" : (t >= M.ev.start && (d.status === "planned" || d.status === "secured")) ? "busy" : "idle";
    if (st === "idle") return;
    ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI);
    ctx.fillStyle = st === "ok" ? col.good : st === "no" ? col.ref : col.panel; ctx.fill();
    ctx.strokeStyle = st === "busy" ? col.accent : col.deep; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.strokeStyle = st === "busy" ? col.accent : col.deep; ctx.lineWidth = 2;
    if (st === "ok") { ctx.beginPath(); ctx.moveTo(x - 3.5, y); ctx.lineTo(x - 1, y + 3); ctx.lineTo(x + 4, y - 3); ctx.stroke(); }
    else if (st === "no") { line(ctx, x - 3, y - 3, x + 3, y + 3); line(ctx, x + 3, y - 3, x - 3, y + 3); }
    else { ctx.beginPath(); ctx.arc(x, y, 2.5, 0, 2 * Math.PI); ctx.fillStyle = col.accent; ctx.fill(); }
  }

  function frontX(ts, tArrive, lead) {
    var hx = geo.hx0 - 4;
    var v = (hx + 70) / lead;
    return hx + (ts - tArrive) * v;
  }

  function drawGround(ctx, t) {
    var g = geo, gy = g.groundY;
    /* hillside behind the terrace */
    ctx.fillStyle = P.alpha(col.line, 0.22);
    ctx.beginPath(); ctx.moveTo(0, gy - g.h * 0.3); ctx.quadraticCurveTo(g.w * 0.05, gy - g.h * 0.12, g.hx0 - 20, gy); ctx.lineTo(0, gy); ctx.closePath(); ctx.fill();
    ctx.fillStyle = P.alpha(col.line, 0.3);
    ctx.fillRect(0, gy, g.w, g.h - gy);
    var sk = shakeAt(building, t), jig = shake ? (sk.dg / shake.maxD) * (g.narrow ? 6 : 10) : 0;
    ctx.strokeStyle = col.grid; ctx.lineWidth = 1;
    for (var k = 1; k <= 3; k++) {
      var y = gy + ((g.h - gy) * k) / 4;
      ctx.beginPath();
      for (var x = 0; x <= g.w; x += 12) {
        var yy = y + 0.8 * Math.sin(x / 40 + k) + (k === 1 ? 0.2 * jig : 0);
        if (x === 0) ctx.moveTo(x + jig * 0.4, yy); else ctx.lineTo(x + jig * 0.4, yy);
      }
      ctx.stroke();
    }
    ctx.strokeStyle = col.axis; ctx.lineWidth = 1.5; line(ctx, 0, gy + 0.5, g.w, gy + 0.5);
    /* wavefronts in the soil, reaching the house at t_P and t_S */
    var xp = frontX(t, M.ev.t_p, 1.3);
    if (xp > -60 && xp < g.w + 80) {
      for (var i = 0; i < 5; i++) { ctx.strokeStyle = P.alpha(col.p, 0.8 - i * 0.14); ctx.lineWidth = i ? 1 : 2; line(ctx, xp - i * 7, gy + 2, xp - i * 7, g.h); }
      text(ctx, "P-wave", Math.max(4, xp + 6), g.h - 8, { color: col.ink2, weight: "600" });
    }
    var xs = frontX(t, M.ev.t_s, 1.9);
    if (xs > -60 && xs < g.w + 80) {
      ctx.strokeStyle = col.s; ctx.lineWidth = 2; ctx.beginPath();
      for (var yy2 = gy + 2; yy2 <= g.h; yy2 += 3) { var xx = xs + 6 * Math.sin((yy2 - gy) / 7 - t * 9); if (yy2 === gy + 2) ctx.moveTo(xx, yy2); else ctx.lineTo(xx, yy2); }
      ctx.stroke();
      text(ctx, "S-wave", Math.max(4, xs + 9), g.h - 8, { color: col.ink2, weight: "600" });
    }
  }

  /* Returns the offset (px) of the building at height y, given the base offset and drift. */
  function drawHouse(ctx, t) {
    var g = geo, gy = g.groundY, sk = shakeAt(building, t);
    var scale = shake ? (g.narrow ? 10 : 16) / shake.maxD : 0;
    var baseOff = (sk.dg + sk.xb) * scale, driftOff = sk.yd * scale, groundOff = sk.dg * scale;
    var yBase = gy - g.baseH, yF1 = yBase - g.sh, yRoof = yF1 - g.sh;
    function off(y) { return baseOff + driftOff * clamp((yBase - y) / (2 * g.sh), 0, 1); }
    var cfg = building;
    ctx.save();
    ctx.lineJoin = "round";
    /* foundation, isolation layer or platform */
    if (cfg === "fixed_base") {
      ctx.fillStyle = col.steel; ctx.fillRect(g.hx0 - 6 + groundOff, yBase, g.hw + 12, g.baseH + 10);
    } else {
      /* pit below grade with a moat gap at each side */
      ctx.fillStyle = P.alpha(col.deep, 0.9); ctx.fillRect(g.hx0 - 16 + groundOff, gy - 1, g.hw + 32, 18);
      ctx.fillStyle = col.steel; ctx.fillRect(g.hx0 - 16 + groundOff, gy + 13, g.hw + 32, 6);
      ctx.fillStyle = col.steel; ctx.fillRect(g.hx0 - 20 + groundOff, gy - 6, 4, 24); ctx.fillRect(g.hx1 + 16 + groundOff, gy - 6, 4, 24);
      if (cfg === "fps_isolated") {
        var nIso = Math.max(4, Math.round(g.hw / 110));
        for (var i = 0; i < nIso; i++) {
          var ix = g.hx0 + 12 + (g.hw - 24) * i / (nIso - 1);
          ctx.strokeStyle = col.ink2; ctx.lineWidth = 1.5;
          ctx.beginPath(); ctx.arc(ix + groundOff, gy + 4, 12, 0.25 * Math.PI, 0.75 * Math.PI); ctx.stroke();
          ctx.fillStyle = col.ink2; ctx.fillRect(ix + baseOff - 5, yBase + g.baseH - 4, 10, 6);
        }
        if (!g.narrow) text(ctx, "friction-pendulum isolators", g.hx0 + 14 + groundOff, gy + 30, { size: 10 });
      } else {
        /* HAVEN-F reduced platform: levitated on a flux-glow gap (research tier) */
        var ph = 0.5 + 0.5 * Math.sin(t * 5);
        ctx.fillStyle = P.alpha(col.flux, 0.2 + 0.2 * ph); ctx.fillRect(g.hx0 - 6 + baseOff, gy + 1, g.hw + 12, 8);
        ctx.fillStyle = col.steel; ctx.fillRect(g.hx0 - 6 + groundOff, gy + 9, g.hw + 12, 4);
        if (!g.narrow) text(ctx, "HAVEN-F reduced platform · research tier", g.hx0 + 14 + groundOff, gy + 30, { size: 10, color: col.frontier });
      }
      ctx.fillStyle = col.steel; ctx.fillRect(g.hx0 - 6 + baseOff, yBase, g.hw + 12, g.baseH);
    }
    /* storeys: walls, slabs, roof */
    ctx.strokeStyle = col.ink2; ctx.lineWidth = 1.6; ctx.fillStyle = P.alpha(col.panel, 0.92);
    ctx.beginPath();
    ctx.moveTo(g.hx0 + off(yBase), yBase); ctx.lineTo(g.hx0 + off(yRoof), yRoof);
    ctx.lineTo(g.hx1 + off(yRoof), yRoof); ctx.lineTo(g.hx1 + off(yBase), yBase); ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.fillStyle = col.steel;
    ctx.fillRect(g.hx0 - 3 + off(yF1), yF1 - 3, g.hw + 6, 6);
    ctx.fillRect(g.hx0 - 18 + off(yRoof), yRoof - g.roofH, g.hw + 36, g.roofH);
    /* interior partitions */
    var xs = { kit: 0.44, pan: 0.58, gal: 0.36 };
    ctx.strokeStyle = P.alpha(col.ink2, 0.6); ctx.lineWidth = 1.2;
    line(ctx, g.hx0 + g.hw * xs.kit + off(yF1), yF1 + 3, g.hx0 + g.hw * xs.kit + off(yBase), yBase - 26);
    line(ctx, g.hx0 + g.hw * xs.pan + off(yF1), yF1 + 3, g.hx0 + g.hw * xs.pan + off(yBase), yBase);
    line(ctx, g.hx0 + g.hw * xs.gal + off(yRoof), yRoof, g.hx0 + g.hw * xs.gal + off(yF1), yF1 - 3);
    /* room labels */
    var lab = { size: g.narrow ? 9.5 : 10.5, color: col.muted, mono: true };
    text(ctx, "KITCHEN", g.hx0 + 8 + off(yF1), yF1 + 16, lab);
    text(ctx, "PANTRY", g.hx0 + g.hw * xs.kit + 6 + off(yF1), yF1 + 16, lab);
    text(ctx, "SALON", g.hx0 + g.hw * xs.pan + 8 + off(yF1), yF1 + 16, lab);
    text(ctx, "GALLERY", g.hx0 + g.hw * xs.gal + 8 + off(yRoof), yRoof + 16, lab);
    if (!g.narrow) text(ctx, "STUDY", g.hx0 + 8 + off(yRoof), yRoof + 16, lab);
    ctx.restore();
    return { off: off, yBase: yBase, yF1: yF1, yRoof: yRoof, xs: xs, sk: sk };
  }

  function drawDevices(ctx, t, H) {
    var g = geo, off = H.off, yBase = H.yBase, yF1 = H.yF1, yRoof = H.yRoof;
    var drawers = M.devices.filter(function (d) { return d.cls === "drawer" && d.kind !== "slider"; });
    var sliders = M.devices.filter(function (d) { return d.kind === "slider"; });
    var kitW = g.hw * H.xs.kit;
    /* base cabinets: two columns of stacked drawers opening to the right (+x) */
    var perCab = Math.max(1, Math.ceil(drawers.length / 2));
    var cabW = Math.max(34, Math.min(80, kitW * 0.2)), cabH = Math.min(g.sh * 0.5, 22 * perCab + 10);
    var pxPerM = cabW / 0.6;                         /* a 0.6 m deep cabinet; 0.30 m travel is half of it */
    var cabs = [g.hx0 + 8, g.hx0 + kitW * 0.5];
    var cabTop = yBase - cabH;
    for (var c = 0; c < 2; c++) {
      var cx = cabs[c] + off(cabTop);
      ctx.fillStyle = P.alpha(col.line, 0.55); ctx.strokeStyle = col.ink2; ctx.lineWidth = 1.2;
      ctx.fillRect(cx, cabTop, cabW, cabH); ctx.strokeRect(cx, cabTop, cabW, cabH);
      ctx.fillStyle = col.ink2; ctx.fillRect(cx - 2, cabTop - 4, cabW + 8, 4);
    }
    var dh = (cabH - 6) / perCab;
    drawers.forEach(function (d, i) {
      var c2 = i < perCab ? 0 : 1, row = i % perCab;
      var y = cabTop + 3 + row * dh, x0 = cabs[c2] + off(y);
      var xm = d.x ? interp(d.x, t) : 0;
      var open = clamp(num(xm, 0), 0, 0.6) * pxPerM;
      var colr = col.drawer, hot = d.status === "not_secured" && t >= d.start;
      ctx.fillStyle = P.alpha(colr, 0.16); ctx.strokeStyle = hot ? col.ref : colr; ctx.lineWidth = 1.4;
      ctx.fillRect(x0 + 2 + open, y + 1, cabW - 4, dh - 3); ctx.strokeRect(x0 + 2 + open, y + 1, cabW - 4, dh - 3);
      ctx.fillStyle = hot ? col.ref : colr; ctx.fillRect(x0 + cabW - 3 + open, y, 4, dh - 1);
      /* contents: glasses or plates inside the box */
      if (d.contents) {
        ctx.strokeStyle = col.ink2; ctx.lineWidth = 1;
        var slipNow = contentsSlip(d.id, t), wob = slipNow ? Math.sin(t * 40) * 1.5 : 0;
        for (var k = 0; k < 3; k++) {
          var gx = x0 + 8 + open + k * (cabW - 14) / 3 + wob, gb = y + dh - 3;
          if (d.contents === "glassware") { line(ctx, gx, gb, gx + 3, gb - (dh - 7) * 0.8); line(ctx, gx + 6, gb, gx + 3, gb - (dh - 7) * 0.8); }
          else { ctx.beginPath(); ctx.ellipse(gx + 3, gb - 2, 4, 1.6, 0, 0, 2 * Math.PI); ctx.stroke(); }
        }
      }
      if (open > 1.5 || d.status === "not_secured" || (d.moved && t >= d.start)) {
        statusMark(ctx, x0 + cabW + open + 12, y + dh / 2, d, t);
        if (!g.narrow) text(ctx, d.id + (open > 1 ? " · " + fmtSI(xm, "m", 2) : ""), x0 + cabW + open + 23, y + dh / 2 + 4, { size: 10.5, color: col.ink2 });
      }
    });
    /* wall cabinet with sliders (elevation) */
    if (sliders.length) {
      var wy = yF1 + 24, wh = Math.max(16, g.sh * 0.2), wx = g.hx0 + 8, ww = Math.min(kitW * 0.62, 160);
      var ox = off(wy);
      ctx.strokeStyle = col.ink2; ctx.lineWidth = 1.2; ctx.fillStyle = P.alpha(col.line, 0.45);
      ctx.fillRect(wx + ox, wy, ww, wh); ctx.strokeRect(wx + ox, wy, ww, wh);
      var pw = ww / sliders.length;
      sliders.forEach(function (d, i) {
        var xm = d.x ? interp(d.x, t) : 0, sh2 = clamp(num(xm, 0) / 0.4, 0, 1) * pw * 0.8;
        ctx.fillStyle = P.alpha(col.drawer, 0.28); ctx.strokeStyle = col.drawer;
        ctx.fillRect(wx + ox + i * pw + 2 - sh2, wy + 2, pw - 4, wh - 4); ctx.strokeRect(wx + ox + i * pw + 2 - sh2, wy + 2, pw - 4, wh - 4);
        if (d.status === "secured" && t >= M.ev.start) statusMark(ctx, wx + ox + (i + 0.5) * pw - sh2, wy + wh / 2, d, t);
      });
      if (!g.narrow) text(ctx, "cabinet sliders", wx + ox + ww + 6, wy + wh / 2 + 4, { size: 10 });
    }
    /* pantry door: a sliding panel across the doorway; lands after the stop */
    var door = M.devices.filter(function (d) { return d.cls === "door"; })[0];
    if (door) {
      var dx = g.hx0 + g.hw * H.xs.kit, dw = g.hw * (H.xs.pan - H.xs.kit), dTop = yF1 + 28, dH = yBase - dTop;
      var open2 = door.x ? clamp(num(interp(door.x, t), 0) / 1.0, 0, 1) : 0.15;
      var landed = door.timingFromRun ? false : securedAt(door, t);
      var ox2 = off(dTop + dH / 2), drop = landed ? 2 : 0;
      ctx.fillStyle = P.alpha(col.door, 0.22); ctx.strokeStyle = col.door; ctx.lineWidth = 1.5;
      ctx.fillRect(dx + 4 + ox2 + open2 * dw * 0.7, dTop + drop, dw * 0.62, dH - 2); ctx.strokeRect(dx + 4 + ox2 + open2 * dw * 0.7, dTop + drop, dw * 0.62, dH - 2);
      ctx.fillStyle = col.steel; ctx.fillRect(dx + ox2, dTop - 5, dw, 4);
      statusMark(ctx, dx + dw / 2 + ox2, dTop - 14, door, t);
      if (!g.narrow) text(ctx, "door · egress, never locks", dx + 2 + ox2, yBase - 6, { size: 9.5, color: col.ink2 });
    }
    /* salon partition wall: lands and locks */
    var wall = M.devices.filter(function (d) { return d.cls === "wall"; })[0];
    if (wall) {
      var sx0 = g.hx0 + g.hw * H.xs.pan, sw = g.hw * (1 - H.xs.pan), wTop = yF1 + 28, wH = yBase - wTop;
      var wOpen = wall.x ? clamp(num(interp(wall.x, t), 0) / 6.0, 0, 1) : 0;
      var locked = !wall.timingFromRun && securedAt(wall, t);
      var ox3 = off(wTop + wH / 2), pX = sx0 + sw * 0.45 + ox3 + wOpen * sw * 0.4;
      ctx.fillStyle = col.steel; ctx.fillRect(sx0 + sw * 0.4 + ox3, wTop - 5, sw * 0.58, 4);
      ctx.fillStyle = P.alpha(col.wall, 0.2); ctx.strokeStyle = col.wall; ctx.lineWidth = 1.5;
      ctx.fillRect(pX, wTop + (locked ? 2 : 0), sw * 0.12, wH - 2); ctx.strokeRect(pX, wTop + (locked ? 2 : 0), sw * 0.12, wH - 2);
      if (locked) {
        ctx.strokeStyle = col.ink; ctx.lineWidth = 1.5;
        var lx = pX + sw * 0.06, ly = wTop + wH * 0.55;
        ctx.strokeRect(lx - 5, ly, 10, 8); ctx.beginPath(); ctx.arc(lx, ly, 4, Math.PI, 0); ctx.stroke();
      }
      statusMark(ctx, pX + sw * 0.06, wTop - 14, wall, t);
      if (!g.narrow) text(ctx, "partition wall · lands and locks", sx0 + 8 + ox3, yBase - 6, { size: 9.5, color: col.ink2 });
    }
    /* gallery plinth on an isolated platform (HAVEN-R, research tier) */
    var plinth = M.devices.filter(function (d) { return d.cls === "haven_r"; })[0];
    if (plinth) {
      var gx0 = g.hx0 + g.hw * (H.xs.gal + 0.28), pw2 = Math.min(90, g.hw * 0.16), fy = yF1 - 3;
      var skP = H.sk, scale = shake ? (g.narrow ? 10 : 16) / shake.maxD : 0;
      /* the platform lags the floor: relative motion schematic from a T 6 s filter of the floor drift */
      var rel = -(skP.yd * 0.6 + (building === "fixed_base" ? skP.dg * 0.5 : 0)) * scale;
      var ox4 = off(fy);
      ctx.fillStyle = P.alpha(col.deep, 0.8); ctx.fillRect(gx0 - 10 + ox4, fy - 6, pw2 + 20, 6);
      ctx.fillStyle = P.alpha(col.exotic, 0.25); ctx.strokeStyle = col.exotic; ctx.lineWidth = 1.4;
      ctx.fillRect(gx0 + ox4 + rel, fy - 10, pw2, 5); ctx.strokeRect(gx0 + ox4 + rel, fy - 10, pw2, 5);
      var bx = gx0 + ox4 + rel + pw2 * 0.3, bw = pw2 * 0.4, bh = Math.min(g.sh * 0.3, 34);
      ctx.fillStyle = P.alpha(col.line, 0.7); ctx.fillRect(bx, fy - 10 - bh, bw, bh); ctx.strokeRect(bx, fy - 10 - bh, bw, bh);
      ctx.fillStyle = col.ink2; ctx.beginPath(); ctx.ellipse(bx + bw / 2, fy - 10 - bh - 9, 5, 9, 0, 0, 2 * Math.PI); ctx.fill();
      statusMark(ctx, bx + bw / 2, fy - 10 - bh - 26, plinth, t);
      if (!g.narrow) text(ctx, "art plinth · HAVEN-R, research tier", gx0 + ox4 + rel + pw2 + 10, fy - 14, { size: 9.5, color: col.ink2 });
    }
  }

  var contentsEvents = [];
  function indexContentsEvents() {
    contentsEvents = M.timeline.filter(function (r) { return r[1] && /^contents:(slip|rock)/.test(String(r[2])); });
  }
  function contentsSlip(id, t) {
    for (var i = 0; i < contentsEvents.length; i++) if (contentsEvents[i][1] === id && Math.abs(contentsEvents[i][0] - t) < 0.25) return true;
    return false;
  }

  function drawAlert(ctx, t, H) {
    var e = M.ev;
    var ta = M.mode === "preview" && M.variant === "onsite" ? e.protect : e.alert;
    if (!isFinite(ta) || t < ta || t > ta + 3) return;
    var g = geo, cx = (g.narrow ? g.hx1 - 30 : g.hx0 + g.hw * 0.62) + H.off(H.yRoof), cy = H.yRoof - g.roofH - 14, a = 1 - (t - ta) / 3;
    ctx.strokeStyle = P.alpha(col.accent, 0.9 * a); ctx.lineWidth = 1.6;
    for (var k = 1; k <= 3; k++) {
      var r = 5 + k * 6 + ((t - ta) * 14) % 6;
      ctx.beginPath(); ctx.arc(cx, cy, r, -0.8 * Math.PI, -0.2 * Math.PI); ctx.stroke();
    }
    ctx.fillStyle = col.accent; ctx.beginPath(); ctx.arc(cx, cy, 3, 0, 2 * Math.PI); ctx.fill();
    text(ctx, M.mode === "preview" && M.variant === "onsite" ? "PROTECT (on-site P)" : "synthetic alert", cx - 12, cy - 22, { align: "right", size: 10.5, color: col.ink2, weight: "600" });
  }

  function drawAccelTrace(ctx, t) {
    var g = geo;
    if (!shake || g.narrow) return;
    var w = Math.min(250, g.w * 0.22), h = 38, win = 8, x0 = g.w - w - 12, y0 = 12;
    ctx.fillStyle = P.alpha(col.panel, 0.9); ctx.strokeStyle = col.line; ctx.lineWidth = 1;
    P.roundRect(ctx, x0, y0, w, h + 16, 7); ctx.fill(); ctx.stroke();
    var mid = y0 + 14 + h / 2, amp = h / 2 - 2;
    ctx.strokeStyle = col.grid; line(ctx, x0 + 6, mid, x0 + w - 6, mid);
    ctx.strokeStyle = col.accent; ctx.lineWidth = 1.3; ctx.beginPath();
    var n = Math.floor(w - 12), o = shake.cfg[building];
    for (var i = 0; i <= n; i++) {
      var tt = t - win + win * i / n, j = clamp(Math.round(tt / shake.dt), 0, shake.n - 1);
      var v = tt < 0 ? 0 : o.a1[j] / shake.maxA;
      var y = mid - clamp(v, -1, 1) * amp;
      if (i === 0) ctx.moveTo(x0 + 6 + i, y); else ctx.lineTo(x0 + 6 + i, y);
    }
    ctx.stroke();
    text(ctx, "floor acceleration · schematic, common scale", x0 + 8, y0 + 11, { size: 9.5 });
  }

  function drawFrame(t) {
    if (!cv || !geo) return;
    var ctx = cv.ctx;
    ctx.setTransform(cv.dpr, 0, 0, cv.dpr, 0, 0);
    ctx.clearRect(0, 0, geo.w, geo.h);
    ctx.fillStyle = col.deep; ctx.fillRect(0, 0, geo.w, geo.h);
    drawGround(ctx, t);
    var H = drawHouse(ctx, t);
    drawDevices(ctx, t, H);
    drawAlert(ctx, t, H);
    drawAccelTrace(ctx, t);
    current = t;
    updateText(t);
  }

  function updateText(t) {
    var idx = 0;
    for (var i = 0; i < NARR.length; i++) if (NARR[i][0] <= t + 1e-9) idx = i;
    if (idx !== lastNarr) { lastNarr = idx; if (chip) chip.textContent = NARR[idx][1]; }
    if (clock) clock.textContent = "t = " + t.toFixed(1) + " s after origin";
    if (scrub && doc.activeElement !== scrub) scrub.value = String(t);
    updateReadouts(t);
    updateLog(t);
    updateCharts(t, false);
  }

  function setTime(t) {
    t = clamp(t, M.tStart, M.tEnd);
    if (loop) loop.reset(t - M.tStart);
    drawFrame(t);
    updateCharts(t, true);
  }

  function stillTime() {
    var e = M.ev, last = e.t_s - 0.4;
    M.devices.forEach(function (d) { if (d.status === "secured" && d.tSecured !== null && !d.already) last = Math.max(last, Math.min(d.tSecured + 0.3, e.t_s + 1)); });
    return clamp(last, M.tStart, M.tEnd);
  }

  function setPlaying(on) {
    if (!playBtn) return;
    playBtn.textContent = on ? "Pause" : "Play replay";
    playBtn.setAttribute("aria-pressed", on ? "true" : "false");
  }
  function pause() { if (loop && loop.running()) loop.stop(); setPlaying(false); updateCharts(current, true); }

  var HOLD = 1.5;
  function setupLoop() {
    loop = P.loop(function (dt, tl) {
      var span = M.tEnd - M.tStart, u = tl % (span + HOLD);
      drawFrame(M.tStart + Math.min(u, span));
    }, { el: stage, still: function () { setPlaying(false); drawFrame(stillTime()); updateCharts(current, true); } });
  }

  /* ---- wiring -------------------------------------------------------------------------- */
  function rebuild() {
    NARR = narration();
    lastNarr = -1;
    shake = buildShake(M.tEnd + 1, M.ev.t_p, M.ev.t_s);
    indexContentsEvents();
    if (scrub) { scrub.min = String(M.tStart); scrub.max = String(M.tEnd); }
    renderSource();
    renderJumps();
    renderCharts();
    var vU = doc.querySelectorAll('[data-story="vU"]');
    for (var i = 0; i < vU.length; i++) vU[i].textContent = fmt(V_UNATTENDED, 3) + " m/s";
    buildingNote();
  }

  function buildingNote() {
    var tb = doc.querySelector(".villa-toolbar");
    if (!tb) return;
    var n = doc.getElementById("building-note");
    if (!n) { n = el("span", "small muted", tb); n.id = "building-note"; }
    if (M.mode === "data" && M.building && building !== M.building) {
      n.textContent = "This record ran the " + (BUILDING_TEXT[M.building] || pretty(M.building)) + " building; the toggle changes only the schematic sway and the closed-form transfer chart. Run with --building " + building + " for that record.";
    } else if (M.mode === "data") {
      n.textContent = "The record's building.";
    } else {
      n.textContent = "Preview: the toggle changes the schematic sway and the transfer chart; device timing does not depend on it here.";
    }
  }

  function init() {
    colours();
    layout();
    rebuild();
    initCone();
    drawCone();
    var bSeg = UI.segmented(doc.getElementById("villa-building"), function (v) {
      building = v;
      buildingNote();
      if (charts.transfer) charts.transfer.update(transferSpec());
      drawFrame(current);
    });
    if (bSeg) bSeg.set(building, true);
    var vg = doc.getElementById("villa-variant"), vw = doc.getElementById("variant-wrap");
    if (vg && M.mode === "preview") {
      vg.hidden = false; if (vw) vw.hidden = false;
      UI.segmented(vg, function (v) {
        variant = v;
        M = preview(variant);
        rebuild();
        drawFrame(clamp(current, M.tStart, M.tEnd));
        updateCharts(current, true);
      });
    }
    setupLoop();
    if (playBtn) playBtn.addEventListener("click", function () {
      if (loop.running()) pause();
      else { loop.reset(current >= M.tEnd - 0.05 ? 0 : current - M.tStart); if (loop.start(true)) setPlaying(true); }
    });
    if (scrub) {
      scrub.addEventListener("input", function () { pause(); setTime(parseFloat(scrub.value)); });
    }
    P.onTheme(function () { colours(); drawFrame(current); });
    if (window.ResizeObserver && stage) {
      var lastW = 0;
      new ResizeObserver(function () {
        var w = stage.clientWidth;
        if (Math.abs(w - lastW) >= 1) { lastW = w; layout(); drawFrame(current); }
      }).observe(stage);
    } else window.addEventListener("resize", function () { layout(); drawFrame(current); });

    var m = /^#t=(\d+(?:\.\d+)?)$/.exec(window.location.hash || "");
    if (m) { pinned = true; setPlaying(false); setTime(parseFloat(m[1])); }
    else if (loop.start()) setPlaying(true);
    else { setPlaying(false); drawFrame(stillTime()); updateCharts(current, true); }
  }

  init();
})();
