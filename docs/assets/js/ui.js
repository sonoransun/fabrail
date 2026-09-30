/* fabrail UI helpers (plan §3b). Owner: A13.
 *
 * Small, dependency-free building blocks shared by every page: number formatting,
 * sliders bound to data-param attributes, segmented choices, tier badges
 * (✅ V · 🔶 S · 🧪 R · 🗺️ M · 🔭 frontier, SPECIFICATION §0.5), the frontier and
 * research banners, and "simulated" labels. Text goes in through textContent only.
 *
 * API (window.FabrailUI; the pure helpers also as module.exports under node):
 *   fmt(v, digits), fmtSI(v, unit, digits), fmtValue(v, { unit, scale, digits, si })
 *   bindSliders(root, onChange(values, param, input), { initial }) ->
 *       { values(), get(param), set(param, siValue), inputs }
 *     <input type="range" data-param="mass_kg" data-scale="1" data-unit="kg"
 *            data-digits="3" [data-log] min max step value>
 *     values are SI: SI = shown value ÷ data-scale (data-scale="1000" shows metres as mm).
 *     With data-log the input's min/max/value are log₁₀ of the shown value.
 *     The readout is <output data-readout="mass_kg"> (or <output for="input-id">).
 *   segmented(group, onChange(value)) -> { value(), set(v) }
 *     <div data-choice-group aria-label="Class"><button data-value="drawer">…</button>…</div>
 *   TIERS, tier(name) -> TIERS entry, badge(name[, suffix]) -> <span class="badge …">
 *   renderTiers(root)        fills empty [data-tier="v|v(i)|s|r|m|f"] elements
 *   frontierBanner({ missing, text }), researchBanner({ text }), renderBanners(root)
 *     fills [data-frontier-banner data-missing="5 orders of magnitude"] and
 *     [data-research-banner] (existing content is kept after the standard wording)
 *   simLabel(kind), renderSimLabels(root)   fills [data-simulated="replay"] etc.
 *   setText(target, text), reducedMotion(), onVisible(el, fn(visible)), renderAll(root)
 * renderAll(document) runs once on DOMContentLoaded.
 */
(function (root) {
  "use strict";

  var doc = root.document || null;
  var MINUS = "−";

  /* ---- formatting (same rules as FabrailPlot.fmt, kept standalone) ---------------- */

  function trimZeros(s) {
    return s.indexOf(".") < 0 ? s : s.replace(/\.?0+$/, "");
  }

  function fmt(v, digits) {
    if (typeof v !== "number" || isNaN(v)) return "–";
    if (!isFinite(v)) return v > 0 ? "∞" : MINUS + "∞";
    if (v === 0) return "0";
    var d = digits === undefined ? 3 : Math.max(1, digits);
    var a = Math.abs(v), s;
    if (a < 1e-4 || a >= 1e9) {
      var e = a.toExponential(d - 1).split("e");
      s = trimZeros(e[0]) + "e" + e[1].replace("+", "").replace("-", MINUS);
    } else {
      var decimals = Math.max(0, d - 1 - Math.floor(Math.log(a) / Math.LN10 + 1e-12));
      s = trimZeros(a.toFixed(Math.min(decimals, 12)));
      var parts = s.split(".");
      parts[0] = parts[0].replace(/\B(?=(\d{3})+(?!\d))/g, ",");
      s = parts.join(".");
    }
    return (v < 0 ? MINUS : "") + s;
  }

  var PREFIXABLE = { m: 1, s: 1, N: 1, J: 1, W: 1, A: 1, V: 1, Hz: 1, Pa: 1, T: 1, Wh: 1,
    "m/s": 1, "m/s²": 1, "N/m": 1 };
  var PREFIXES = [[1e9, "G"], [1e6, "M"], [1e3, "k"], [1, ""], [1e-3, "m"], [1e-6, "µ"], [1e-9, "n"]];
  var DOWNSHIFT_BELOW = { s: 0.1, "m/s": 0.01, "m/s²": 0.01 };

  function fmtSI(v, unit, digits) {
    if (typeof v !== "number" || isNaN(v)) return "–";
    if (!unit) return fmt(v, digits);
    var a = Math.abs(v);
    if (v === 0 || !isFinite(v) || !PREFIXABLE[unit]) return fmt(v, digits) + " " + unit;
    var p = [1, ""];
    if (DOWNSHIFT_BELOW[unit]) {
      if (a < DOWNSHIFT_BELOW[unit]) p = a >= 1e-6 ? [1e-3, "m"] : [1e-6, "µ"];
    } else {
      p = PREFIXES[PREFIXES.length - 1];
      for (var i = 0; i < PREFIXES.length; i++) if (a >= PREFIXES[i][0] * 0.9995) { p = PREFIXES[i]; break; }
    }
    return fmt(v / p[0], digits) + " " + p[1] + unit;
  }

  /* A value for display: SI × scale in the stated unit, or SI-prefixed when si is set. */
  function fmtValue(v, o) {
    o = o || {};
    if (o.si) return fmtSI(v, o.unit, o.digits);
    var shown = v * (o.scale === undefined ? 1 : o.scale);
    return fmt(shown, o.digits) + (o.unit ? " " + o.unit : "");
  }

  /* ---- sliders ---------------------------------------------------------------------- */

  function num(attr, dflt) {
    var x = parseFloat(attr);
    return isFinite(x) ? x : dflt;
  }

  function sliderInfo(input) {
    return {
      param: input.getAttribute("data-param"),
      scale: num(input.getAttribute("data-scale"), 1) || 1,
      unit: input.getAttribute("data-unit") || "",
      digits: num(input.getAttribute("data-digits"), 3),
      log: input.hasAttribute("data-log")
    };
  }

  function shownValue(input, info) {
    var raw = parseFloat(input.value);
    return info.log ? Math.pow(10, raw) : raw;
  }

  function readoutsFor(scope, input, param) {
    var out = [];
    var sel = scope.querySelectorAll('output[data-readout="' + param + '"]');
    for (var i = 0; i < sel.length; i++) out.push(sel[i]);
    if (input.id) {
      var byFor = scope.querySelectorAll("output[for]");
      for (var j = 0; j < byFor.length; j++) {
        var f = (byFor[j].getAttribute("for") || "").split(/\s+/);
        if (f.indexOf(input.id) >= 0 && out.indexOf(byFor[j]) < 0) out.push(byFor[j]);
      }
    }
    return out;
  }

  function bindSliders(scope, onChange, opts) {
    scope = scope || doc;
    opts = opts || {};
    var inputs = scope ? scope.querySelectorAll('input[type="range"][data-param]') : [];
    var byParam = {};
    var values = {};

    function refresh(input) {
      var info = sliderInfo(input);
      var shown = shownValue(input, info);
      values[info.param] = shown / info.scale;
      var text = fmt(shown, info.digits) + (info.unit ? " " + info.unit : "");
      input.setAttribute("aria-valuetext", text);
      var outs = readoutsFor(doc || scope, input, info.param);
      for (var i = 0; i < outs.length; i++) outs[i].textContent = text;
      return info.param;
    }

    function fire(param, input) {
      if (typeof onChange !== "function") return;
      var copy = {};
      for (var k in values) copy[k] = values[k];
      onChange(copy, param, input);
    }

    for (var i = 0; i < inputs.length; i++) {
      (function (input) {
        var p = refresh(input);
        byParam[p] = input;
        input.addEventListener("input", function () { fire(refresh(input), input); });
      })(inputs[i]);
    }
    if (opts.initial !== false) fire(null, null);

    return {
      inputs: byParam,
      values: function () { var c = {}; for (var k in values) c[k] = values[k]; return c; },
      get: function (param) { return values[param]; },
      set: function (param, si, silent) {
        var input = byParam[param];
        if (!input) return false;
        var info = sliderInfo(input);
        var shown = si * info.scale;
        input.value = String(info.log ? Math.log(shown) / Math.LN10 : shown);
        refresh(input);
        if (!silent) fire(param, input);
        return true;
      }
    };
  }

  /* ---- segmented choice ------------------------------------------------------------- */

  function segmented(group, onChange) {
    if (!group) return null;
    var buttons = group.querySelectorAll("button[data-value]");
    if (!group.hasAttribute("role")) group.setAttribute("role", "group");
    var current = null;
    function set(v, silent) {
      for (var i = 0; i < buttons.length; i++) {
        var on = buttons[i].getAttribute("data-value") === v;
        buttons[i].setAttribute("aria-pressed", on ? "true" : "false");
        if (on) current = v;
      }
      group.setAttribute("data-current", current || "");
      if (!silent && typeof onChange === "function") onChange(current);
    }
    for (var i = 0; i < buttons.length; i++) {
      buttons[i].type = "button";
      buttons[i].addEventListener("click", function () { set(this.getAttribute("data-value")); });
    }
    var pressed = group.querySelector('button[aria-pressed="true"]') || buttons[0];
    if (pressed) set(pressed.getAttribute("data-value"), true);
    return { value: function () { return current; }, set: set };
  }

  /* ---- tier badges (SPECIFICATION §0.5; capability-matrix legend) ------------------- */

  var TIERS = {
    v: { key: "v", emoji: "✅", text: "V", title: "verified in simulation", cls: "badge--v" },
    "v(i)": { key: "v(i)", emoji: "✅", text: "V(I)", title: "verified by inspection", cls: "badge--v" },
    s: { key: "s", emoji: "🔶", text: "S", title: "simplified model", cls: "badge--s" },
    r: { key: "r", emoji: "🧪", text: "R", title: "research tier, disabled by default", cls: "badge--r" },
    m: { key: "m", emoji: "🗺️", text: "M", title: "roadmap, not built", cls: "badge--m" },
    f: { key: "f", emoji: "🔭", text: "frontier", title: "theoretical what-if, never a product", cls: "badge--f" }
  };
  var TIER_ALIASES = {
    verified: "v", vi: "v(i)", inspection: "v(i)", simplified: "s", research: "r",
    roadmap: "m", frontier: "f", "✅": "v", "🔶": "s", "🧪": "r", "🗺️": "m", "🗺": "m", "🔭": "f"
  };

  function tier(name) {
    var k = String(name || "").trim().toLowerCase();
    k = TIER_ALIASES[k] || k;
    return TIERS[k] || null;
  }

  function badge(name, suffix) {
    var t = tier(name);
    var el = doc.createElement("span");
    if (!t) { el.className = "badge badge--m"; el.textContent = String(name || "?"); return el; }
    fillBadge(el, t, suffix);
    return el;
  }

  function fillBadge(el, t, suffix) {
    el.classList.add("badge");
    el.classList.add(t.cls);
    el.textContent = t.emoji + " " + t.text + (suffix ? " " + suffix : "");
    el.setAttribute("title", t.title);
    if (!el.hasAttribute("aria-label")) el.setAttribute("aria-label", "Tier " + t.text + ": " + t.title + (suffix ? ", " + suffix : ""));
  }

  function renderTiers(scope) {
    var els = (scope || doc).querySelectorAll("[data-tier]");
    for (var i = 0; i < els.length; i++) {
      if (els[i].textContent.trim()) continue;
      var t = tier(els[i].getAttribute("data-tier"));
      if (t) fillBadge(els[i], t, els[i].getAttribute("data-tier-note") || "");
    }
  }

  /* ---- banners ----------------------------------------------------------------------- */

  var FRONTIER_TEXT = "A theoretical what-if study with its assumptions stated. It is not a product and not a design commitment.";
  var RESEARCH_TEXT = "Disabled by default in every preset. Shown for study, not as a product feature.";

  function fillBanner(el, kind, lead, text, missing) {
    el.classList.add("banner");
    el.classList.add(kind);
    if (!el.hasAttribute("role")) el.setAttribute("role", "note");
    var keep = [];
    while (el.firstChild) keep.push(el.removeChild(el.firstChild));
    var strong = doc.createElement("strong");
    strong.className = "banner-title";
    strong.textContent = lead;
    el.appendChild(strong);
    el.appendChild(doc.createTextNode(" " + text));
    if (missing) {
      var m = doc.createElement("span");
      m.className = "banner-missing";
      m.textContent = "Missing: " + missing + ".";
      el.appendChild(doc.createTextNode(" "));
      el.appendChild(m);
    }
    if (keep.length) {
      el.appendChild(doc.createTextNode(" "));
      for (var i = 0; i < keep.length; i++) el.appendChild(keep[i]);
    }
    el.setAttribute("data-rendered", "");
    return el;
  }

  function frontierBanner(o) {
    o = o || {};
    var el = doc.createElement("div");
    if (o.text) el.textContent = o.text;
    return fillBanner(el, "frontier", "🔭 Frontier.", FRONTIER_TEXT, o.missing);
  }

  function researchBanner(o) {
    o = o || {};
    var el = doc.createElement("div");
    if (o.text) el.textContent = o.text;
    return fillBanner(el, "research", "🧪 Research tier.", RESEARCH_TEXT, null);
  }

  function renderBanners(scope) {
    var s = scope || doc, i;
    var f = s.querySelectorAll("[data-frontier-banner]:not([data-rendered])");
    for (i = 0; i < f.length; i++) {
      fillBanner(f[i], "frontier", "🔭 Frontier.", FRONTIER_TEXT, f[i].getAttribute("data-missing"));
    }
    var r = s.querySelectorAll("[data-research-banner]:not([data-rendered])");
    for (i = 0; i < r.length; i++) fillBanner(r[i], "research", "🧪 Research tier.", RESEARCH_TEXT, null);
  }

  /* ---- "simulated" labels ------------------------------------------------------------ */

  var SIM_KINDS = {
    "": "simulated",
    replay: "simulated replay",
    closed: "closed form",
    illustrative: "illustrative",
    synthetic: "synthetic input"
  };

  function simLabel(kind) {
    var el = doc.createElement("span");
    fillSim(el, kind || "");
    return el;
  }

  function fillSim(el, kind) {
    el.classList.add("sim-label");
    el.textContent = SIM_KINDS[kind] || kind || SIM_KINDS[""];
    if (!el.hasAttribute("title")) {
      el.setAttribute("title", kind === "closed" ? "Evaluated from closed-form expressions, checked against the Rust implementation"
        : kind === "illustrative" ? "Schematic timing and waveform for explanation, not a recorded run"
        : "Computed by the fabrail simulator; not a hardware measurement");
    }
  }

  function renderSimLabels(scope) {
    var els = (scope || doc).querySelectorAll("[data-simulated]");
    for (var i = 0; i < els.length; i++) {
      if (els[i].textContent.trim()) { els[i].classList.add("sim-label"); continue; }
      fillSim(els[i], els[i].getAttribute("data-simulated") || "");
    }
  }

  /* ---- small helpers --------------------------------------------------------------- */

  function setText(target, text) {
    var els = typeof target === "string" ? doc.querySelectorAll(target) : target && target.length !== undefined && !target.nodeType ? target : [target];
    for (var i = 0; i < els.length; i++) if (els[i]) els[i].textContent = text;
  }

  function reducedMotion() {
    if (root.FabrailTheme && root.FabrailTheme.reducedMotion) return root.FabrailTheme.reducedMotion();
    try {
      return !!(root.matchMedia && root.matchMedia("(prefers-reduced-motion: reduce)").matches);
    } catch (e) {
      return false;
    }
  }

  function onVisible(el, fn) {
    if (!el) return;
    if (!root.IntersectionObserver) { fn(true); return; }
    try {
      new root.IntersectionObserver(function (entries) {
        fn(entries[entries.length - 1].isIntersecting);
      }, { threshold: 0.01 }).observe(el);
    } catch (e) {
      fn(true);
    }
  }

  function renderAll(scope) {
    renderTiers(scope);
    renderBanners(scope);
    renderSimLabels(scope);
  }

  var api = {
    fmt: fmt, fmtSI: fmtSI, fmtValue: fmtValue, bindSliders: bindSliders, segmented: segmented,
    TIERS: TIERS, tier: tier, badge: badge, renderTiers: renderTiers,
    frontierBanner: frontierBanner, researchBanner: researchBanner, renderBanners: renderBanners,
    simLabel: simLabel, renderSimLabels: renderSimLabels, setText: setText,
    reducedMotion: reducedMotion, onVisible: onVisible, renderAll: renderAll
  };

  if (doc) {
    root.FabrailUI = api;
    var start = function () { renderAll(doc); };
    if (doc.readyState === "loading") doc.addEventListener("DOMContentLoaded", start);
    else start();
  }
  if (typeof module === "object" && module.exports) module.exports = api;
})(typeof window !== "undefined" ? window : globalThis);
