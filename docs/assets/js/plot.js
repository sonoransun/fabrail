/* fabrail canvas plotting micro-library (plan §3b). Owner: A13.
 *
 * No dependencies and no external resources. Charts are drawn on a <canvas> scaled for
 * the device pixel ratio, read their colours from the site's CSS custom properties (so
 * they follow the theme and repaint on "fabrail:themechange"), and ship with the hover
 * layer, a legend for two or more series and an optional data table. Mark specs follow
 * the house design system: 2 px lines, 8 px markers with a 2 px surface ring, area washes
 * at 10 %, bars ≤ 24 px with 4 px rounded data ends, hairline recessive grids, one y-axis.
 *
 * API (window.FabrailPlot; the pure helpers also as module.exports under node):
 *   chart(container, spec) -> Chart        container: an element (the chart fills it)
 *     spec.kind   "line" | "semilog" | "loglog" | "area" | "gantt" | "bar"
 *     spec.title, spec.desc                accessible name and description
 *     spec.x, spec.y   { label, unit, si, min, max, log, zero, format(v), ticks }
 *     spec.series [{ key, label, color, data: [[x, y], …] | { x: [], y: [] },
 *                    fn(x) + samples, dash: [on, off], width, area, legend: false,
 *                    endLabel, reveal }]
 *     spec.stack   stack area series on the first series' x (kind "area")
 *     spec.refs   [{ axis: "x" | "y", value, label, color, dash }]
 *     spec.bands  [{ axis: "x" | "y", from, to, label, color }]
 *     spec.points [{ x, y, label, color }]           annotated markers
 *     spec.rows   [{ label, color, spans: [{ from, to, label, style: "solid" | "hatch" }] }]  (gantt)
 *     spec.bars   [{ label, value, color, note }], spec.orient "h" | "v"            (bar)
 *     spec.reveal x value up to which series are drawn (progressive animation)
 *     spec.height css px (default 260) or spec.aspect (height = width·aspect)
 *     spec.legend true | false (default: ≥ 2 legend-eligible series), spec.tooltip,
 *     spec.table  true renders a <details> data table (≤ spec.tableRows rows)
 *   Chart: update(patch), render(), resize(), destroy(), el, canvas
 *   loop(frame(dt, t), { el, still(), force, maxDt }) -> { start, stop, toggle, running,
 *     reduced, time, reset }   requestAnimationFrame loop that honours
 *     prefers-reduced-motion (calls still() instead unless forced), pauses off-screen and
 *     in hidden tabs, and clamps dt.
 *   setupCanvas(canvas, cssW, cssH) -> { ctx, dpr, w, h }
 *   theme(el) -> resolved colours, color(name, el), alpha(color, a)
 *   fmt(v, digits), fmtSI(v, unit, digits), niceTicks(min, max, n), logTicks(min, max)
 *   onTheme(fn) -> unsubscribe;  reducedMotion()
 * Colour names: s1…s5, drawer | door | wall | heavy | exotic (class tokens), accent,
 * good, frontier, ref (anchor/limit lines), p | s (P and S waves), ink, ink2, muted,
 * grid, axis, or any "--custom-property" / literal CSS colour.
 */
(function (root) {
  "use strict";

  var doc = root.document || null;
  var MINUS = "−";

  /* ---- number formatting ---------------------------------------------------------- */

  function trimZeros(s) {
    return s.indexOf(".") < 0 ? s : s.replace(/\.?0+$/, "");
  }

  function group(intStr) {
    return intStr.replace(/\B(?=(\d{3})+(?!\d))/g, ",");
  }

  /* Significant-digit formatting with grouping, trailing zeros trimmed and a true minus. */
  function fmt(v, digits) {
    if (v === null || v === undefined || typeof v !== "number" || isNaN(v)) return "–";
    if (!isFinite(v)) return v > 0 ? "∞" : MINUS + "∞";
    var d = digits === undefined ? 3 : Math.max(1, digits);
    if (v === 0) return "0";
    var a = Math.abs(v);
    var s;
    if (a < 1e-4 || a >= 1e9) {
      var e = a.toExponential(d - 1).split("e");
      s = trimZeros(e[0]) + "e" + e[1].replace("+", "").replace("-", MINUS);
    } else {
      var decimals = Math.max(0, d - 1 - Math.floor(Math.log(a) / Math.LN10 + 1e-12));
      s = trimZeros(a.toFixed(Math.min(decimals, 12)));
      var parts = s.split(".");
      parts[0] = group(parts[0]);
      s = parts.join(".");
    }
    return (v < 0 ? MINUS : "") + s;
  }

  var PREFIXABLE = { m: 1, s: 1, N: 1, J: 1, W: 1, A: 1, V: 1, Hz: 1, Pa: 1, T: 1, Wh: 1,
    "m/s": 1, "m/s²": 1, "N/m": 1, "N·s/m": 1, "m²": 0 };
  var PREFIXES = [[1e9, "G"], [1e6, "M"], [1e3, "k"], [1, ""], [1e-3, "m"], [1e-6, "µ"], [1e-9, "n"]];

  /* Time and speed read best unprefixed ("0.98 s", "0.265 m/s"); they shift down only
     below these magnitudes ("45 ms", "0.8 mm/s") and never up. */
  var DOWNSHIFT_BELOW = { s: 0.1, "m/s": 0.01, "m/s²": 0.01 };

  function prefixFor(a, unit) {
    if (!PREFIXABLE[unit] || !(a > 0) || !isFinite(a)) return [1, ""];
    if (DOWNSHIFT_BELOW[unit]) return a >= DOWNSHIFT_BELOW[unit] ? [1, ""] : a >= 1e-6 ? [1e-3, "m"] : [1e-6, "µ"];
    for (var i = 0; i < PREFIXES.length; i++) {
      if (a >= PREFIXES[i][0] * 0.9995) return PREFIXES[i];
    }
    return PREFIXES[PREFIXES.length - 1];
  }

  /* Value with an SI-prefixed unit: fmtSI(0.03343, "m") -> "33.4 mm". */
  function fmtSI(v, unit, digits) {
    if (typeof v !== "number" || isNaN(v)) return "–";
    if (!unit) return fmt(v, digits);
    if (v === 0 || !isFinite(v)) return fmt(v, digits) + " " + unit;
    var p = prefixFor(Math.abs(v), unit);
    return fmt(v / p[0], digits) + " " + p[1] + unit;
  }

  /* A value in an axis' unit: SI-prefixed when the axis asks for it. */
  function fmtAxis(axis, v) {
    if (axis && typeof axis.format === "function") return axis.format(v);
    if (axis && axis.si) return fmtSI(v, axis.unit, 4);
    return fmt(v, 4) + (axis && axis.unit ? " " + axis.unit : "");
  }

  /* ---- ticks ----------------------------------------------------------------------- */

  function niceStep(raw) {
    var e = Math.pow(10, Math.floor(Math.log(raw) / Math.LN10));
    var f = raw / e;
    return (f <= 1 ? 1 : f <= 2 ? 2 : f <= 2.5 ? 2.5 : f <= 5 ? 5 : 10) * e;
  }

  /* Evenly spaced "nice" ticks covering [min, max]: { ticks, step, lo, hi }. */
  function niceTicks(min, max, n) {
    if (!(isFinite(min) && isFinite(max))) return { ticks: [], step: 1, lo: 0, hi: 1 };
    if (min === max) { min -= 0.5 * (Math.abs(min) || 1); max += 0.5 * (Math.abs(max) || 1); }
    var step = niceStep((max - min) / Math.max(1, n || 5));
    var lo = Math.floor(min / step + 1e-9) * step;
    var hi = Math.ceil(max / step - 1e-9) * step;
    var ticks = [];
    for (var v = lo, i = 0; v <= hi + step * 1e-6 && i < 200; v = lo + ++i * step) {
      ticks.push(Math.abs(v) < step * 1e-9 ? 0 : v);
    }
    return { ticks: ticks, step: step, lo: lo, hi: hi };
  }

  /* Decade ticks for a log axis: { major, minor }; 2 and 5 are labelled on short ranges. */
  function logTicks(min, max) {
    var major = [], minor = [];
    if (!(min > 0 && max > min)) return { major: major, minor: minor };
    var k0 = Math.floor(Math.log(min) / Math.LN10 + 1e-9);
    var k1 = Math.ceil(Math.log(max) / Math.LN10 - 1e-9);
    var dense = k1 - k0 <= 2;
    for (var k = k0; k <= k1; k++) {
      var base = Math.pow(10, k);
      for (var m = 1; m < 10; m++) {
        var v = base * m;
        if (v < min * 0.9999 || v > max * 1.0001) continue;
        if (m === 1 || (dense && (m === 2 || m === 5))) major.push(v);
        else minor.push(v);
      }
    }
    return { major: major, minor: minor };
  }

  function stepDecimals(step, scale) {
    var s = step / (scale || 1);
    if (!(s > 0)) return 0;
    var d = -Math.floor(Math.log(s) / Math.LN10 + 1e-9);
    if (Math.abs(s * Math.pow(10, d) - 2.5) < 1e-6) d += 1;
    return Math.max(0, Math.min(10, d));
  }

  /* The axis spec with its effective log flag (the chart kind may imply it). */
  function withLog(axis, log) {
    if (!!axis.log === !!log) return axis;
    var o = {};
    for (var k in axis) o[k] = axis[k];
    o.log = !!log;
    return o;
  }

  function tickFormatter(axis, ticks, step) {
    if (typeof axis.format === "function") return axis.format;
    var unit = axis.si ? axis.unit : "";
    var maxAbs = 0;
    for (var i = 0; i < ticks.length; i++) maxAbs = Math.max(maxAbs, Math.abs(ticks[i]));
    if (axis.log) {
      return function (v) { return unit ? fmtSI(v, unit, 3) : fmt(v, 3); };
    }
    var p = unit ? prefixFor(maxAbs, unit) : [1, ""];
    var dec = stepDecimals(step, p[0]);
    return function (v) {
      var x = v / p[0];
      var s = Math.abs(x) >= 1e5 ? fmt(x, 3) : group(Math.abs(x).toFixed(dec).split(".")[0]) +
        (dec > 0 ? "." + Math.abs(x).toFixed(dec).split(".")[1] : "");
      if (x < 0 && Number(Math.abs(x).toFixed(dec)) !== 0) s = MINUS + s;
      return unit ? s + " " + p[1] + unit : s;
    };
  }

  /* ---- theme ------------------------------------------------------------------------- */

  var ALIASES = {
    s1: "--s1", s2: "--s2", s3: "--s3", s4: "--s4", s5: "--s5",
    drawer: "--class-drawer", door: "--class-door", wall: "--class-wall",
    heavy: "--class-heavy", exotic: "--class-exotic",
    accent: "--accent", good: "--good", frontier: "--frontier", ref: "--plot-ref",
    p: "--wave-p", s: "--wave-s", ink: "--ink", ink2: "--ink-2", muted: "--ink-3",
    grid: "--grid", axis: "--axis", panel: "--bg-panel", bg: "--bg", deep: "--bg-deep",
    line: "--line", warn: "--s4", caution: "--s4"
  };
  var FALLBACK = { "--ink": "#e6e9f2", "--ink-2": "#a9b1c4", "--ink-3": "#6e788e",
    "--grid": "#1d2537", "--axis": "#3a445f", "--bg-panel": "#0f1522", "--plot-ref": "#ff6b6b" };

  function cssVar(name, el) {
    if (!doc || !root.getComputedStyle) return FALLBACK[name] || "";
    try {
      var v = root.getComputedStyle(el || doc.documentElement).getPropertyValue(name).trim();
      return v || FALLBACK[name] || "";
    } catch (e) {
      return FALLBACK[name] || "";
    }
  }

  function color(c, el) {
    if (!c) return cssVar("--ink-2", el);
    if (c.charAt(0) === "#" || /^(rgb|hsl|oklch|color)\(/.test(c)) return c;
    if (c.slice(0, 2) === "--") return cssVar(c, el) || c;
    if (ALIASES[c]) return cssVar(ALIASES[c], el) || c;
    return c;
  }

  /* rgba() of a colour at opacity a (hex and rgb() inputs; others fall back to globalAlpha use). */
  function alpha(c, a) {
    var m;
    if (/^#([0-9a-f]{3}){1,2}$/i.test(c)) {
      var h = c.slice(1);
      if (h.length === 3) h = h.charAt(0) + h.charAt(0) + h.charAt(1) + h.charAt(1) + h.charAt(2) + h.charAt(2);
      var n = parseInt(h, 16);
      return "rgba(" + ((n >> 16) & 255) + "," + ((n >> 8) & 255) + "," + (n & 255) + "," + a + ")";
    }
    if ((m = /^rgba?\(([^)]+)\)$/.exec(c))) {
      var p = m[1].split(/[\s,/]+/).filter(Boolean);
      return "rgba(" + p[0] + "," + p[1] + "," + p[2] + "," + a + ")";
    }
    return c;
  }

  function theme(el) {
    return {
      ink: cssVar("--ink", el), ink2: cssVar("--ink-2", el), muted: cssVar("--ink-3", el),
      grid: cssVar("--grid", el), axis: cssVar("--axis", el), panel: cssVar("--bg-panel", el),
      bg: cssVar("--bg", el), accent: cssVar("--accent", el), ref: cssVar("--plot-ref", el),
      sans: cssVar("--font-sans", el) || "system-ui, sans-serif",
      mono: cssVar("--font-mono", el) || "ui-monospace, monospace"
    };
  }

  var themeListeners = [];
  function onTheme(fn) {
    themeListeners.push(fn);
    return function () {
      var i = themeListeners.indexOf(fn);
      if (i >= 0) themeListeners.splice(i, 1);
    };
  }
  function fireTheme() {
    var list = themeListeners.slice();
    for (var i = 0; i < list.length; i++) {
      try { list[i](); } catch (e) { /* one listener never blocks the rest */ }
    }
  }
  if (doc) {
    doc.addEventListener("fabrail:themechange", function () {
      /* let the stylesheet apply the new [data-theme] before reading tokens */
      if (root.requestAnimationFrame) root.requestAnimationFrame(fireTheme); else fireTheme();
    });
    try {
      var mqTheme = root.matchMedia("(prefers-color-scheme: light)");
      if (mqTheme.addEventListener) mqTheme.addEventListener("change", fireTheme);
      else if (mqTheme.addListener) mqTheme.addListener(fireTheme);
    } catch (e) { /* matchMedia unavailable */ }
  }

  function reducedMotion() {
    if (root.FabrailTheme && root.FabrailTheme.reducedMotion) return root.FabrailTheme.reducedMotion();
    try {
      return !!(root.matchMedia && root.matchMedia("(prefers-reduced-motion: reduce)").matches);
    } catch (e) {
      return false;
    }
  }

  /* ---- canvas --------------------------------------------------------------------------- */

  function setupCanvas(canvas, w, h) {
    var dpr = Math.max(1, Math.min(root.devicePixelRatio || 1, 3));
    var W = Math.max(1, Math.round(w * dpr)), H = Math.max(1, Math.round(h * dpr));
    if (canvas.width !== W) canvas.width = W;
    if (canvas.height !== H) canvas.height = H;
    canvas.style.width = w + "px";
    canvas.style.height = h + "px";
    var ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    return { ctx: ctx, dpr: dpr, w: w, h: h };
  }

  /* Rounded rectangle path with per-corner radii [tl, tr, br, bl]. */
  function roundRect(ctx, x, y, w, h, r) {
    var rr = typeof r === "number" ? [r, r, r, r] : r;
    var lim = Math.min(Math.abs(w), Math.abs(h)) / 2;
    var tl = Math.min(rr[0], lim), tr = Math.min(rr[1], lim), br = Math.min(rr[2], lim), bl = Math.min(rr[3], lim);
    ctx.beginPath();
    ctx.moveTo(x + tl, y);
    ctx.lineTo(x + w - tr, y);
    ctx.arcTo(x + w, y, x + w, y + tr, tr);
    ctx.lineTo(x + w, y + h - br);
    ctx.arcTo(x + w, y + h, x + w - br, y + h, br);
    ctx.lineTo(x + bl, y + h);
    ctx.arcTo(x, y + h, x, y + h - bl, bl);
    ctx.lineTo(x, y + tl);
    ctx.arcTo(x, y, x + tl, y, tl);
    ctx.closePath();
  }

  /* ---- animation loop --------------------------------------------------------------- */

  function loop(frame, opts) {
    opts = opts || {};
    var running = false, raf = 0, last = 0, t = 0, visible = true, force = !!opts.force;
    var maxDt = opts.maxDt || 0.1;
    var caf = root.cancelAnimationFrame || function () {};
    var rafFn = root.requestAnimationFrame || function (cb) { return setTimeout(function () { cb(Date.now()); }, 16); };

    function tick(ts) {
      raf = 0;
      if (!running || !visible || (doc && doc.hidden)) { last = 0; return; }
      var dt = last ? Math.min(Math.max((ts - last) / 1000, 0), maxDt) : 0;
      last = ts;
      t += dt;
      try { frame(dt, t); } catch (e) { running = false; throw e; }
      schedule();
    }
    function schedule() {
      if (running && visible && !(doc && doc.hidden) && !raf) raf = rafFn(tick);
    }
    function still() {
      if (typeof opts.still === "function") opts.still(t);
    }
    var ctl = {
      start: function (forceNow) {
        if (forceNow !== undefined) force = !!forceNow;
        if (reducedMotion() && !force) { running = false; still(); return false; }
        if (!running) { running = true; last = 0; schedule(); }
        return true;
      },
      stop: function () {
        running = false;
        if (raf) { caf(raf); raf = 0; }
        last = 0;
      },
      toggle: function (forceNow) { return running ? (ctl.stop(), false) : ctl.start(forceNow); },
      running: function () { return running; },
      reduced: reducedMotion,
      time: function () { return t; },
      reset: function (t0) { t = t0 || 0; last = 0; }
    };
    if (opts.el && root.IntersectionObserver) {
      try {
        new root.IntersectionObserver(function (entries) {
          visible = entries[entries.length - 1].isIntersecting;
          if (visible) { last = 0; schedule(); }
        }, { threshold: 0.01 }).observe(opts.el);
      } catch (e) { /* observe unsupported: always visible */ }
    }
    if (doc) {
      doc.addEventListener("visibilitychange", function () { last = 0; schedule(); });
    }
    try {
      var mq = root.matchMedia("(prefers-reduced-motion: reduce)");
      var onMq = function () { if (mq.matches && running && !force) { ctl.stop(); still(); } };
      if (mq.addEventListener) mq.addEventListener("change", onMq);
      else if (mq.addListener) mq.addListener(onMq);
    } catch (e) { /* matchMedia unavailable */ }
    return ctl;
  }

  /* ---- data --------------------------------------------------------------------------- */

  function seriesXY(s, xDomain, xLog) {
    var xs = [], ys = [], i;
    if (typeof s.fn === "function" && xDomain) {
      var n = Math.max(2, s.samples || 200);
      for (i = 0; i < n; i++) {
        var f = i / (n - 1);
        var x = xLog ? Math.pow(10, Math.log(xDomain[0]) / Math.LN10 + f * (Math.log(xDomain[1] / xDomain[0]) / Math.LN10))
          : xDomain[0] + f * (xDomain[1] - xDomain[0]);
        xs.push(x);
        ys.push(s.fn(x));
      }
    } else if (s.data && s.data.x && s.data.y) {
      xs = Array.prototype.slice.call(s.data.x);
      ys = Array.prototype.slice.call(s.data.y);
    } else if (s.x && s.y) {
      xs = Array.prototype.slice.call(s.x);
      ys = Array.prototype.slice.call(s.y);
    } else if (s.data) {
      for (i = 0; i < s.data.length; i++) { xs.push(s.data[i][0]); ys.push(s.data[i][1]); }
    }
    return { x: xs, y: ys };
  }

  function extent(arrs, log) {
    var lo = Infinity, hi = -Infinity;
    for (var a = 0; a < arrs.length; a++) {
      var v = arrs[a];
      for (var i = 0; i < v.length; i++) {
        var z = v[i];
        if (typeof z !== "number" || !isFinite(z) || (log && z <= 0)) continue;
        if (z < lo) lo = z;
        if (z > hi) hi = z;
      }
    }
    return lo <= hi ? [lo, hi] : null;
  }

  function nearestIndex(xs, x) {
    var lo = 0, hi = xs.length - 1;
    if (hi < 0) return -1;
    if (xs[0] > xs[hi]) { /* descending: linear scan */
      var best = 0, bd = Infinity;
      for (var k = 0; k < xs.length; k++) { var d = Math.abs(xs[k] - x); if (d < bd) { bd = d; best = k; } }
      return best;
    }
    while (hi - lo > 1) {
      var mid = (lo + hi) >> 1;
      if (xs[mid] < x) lo = mid; else hi = mid;
    }
    return Math.abs(xs[lo] - x) <= Math.abs(xs[hi] - x) ? lo : hi;
  }

  function scale(d0, d1, r0, r1, log) {
    if (log) {
      var l0 = Math.log(d0), l1 = Math.log(d1);
      var f = function (v) { return r0 + ((Math.log(v) - l0) / (l1 - l0 || 1)) * (r1 - r0); };
      f.invert = function (p) { return Math.exp(l0 + ((p - r0) / (r1 - r0 || 1)) * (l1 - l0)); };
      return f;
    }
    var g = function (v) { return r0 + ((v - d0) / (d1 - d0 || 1)) * (r1 - r0); };
    g.invert = function (p) { return d0 + ((p - r0) / (r1 - r0 || 1)) * (d1 - d0); };
    return g;
  }

  /* ---- chart ---------------------------------------------------------------------------- */

  var live = [];

  function el(tag, cls, parent) {
    var e = doc.createElement(tag);
    if (cls) e.className = cls;
    if (parent) parent.appendChild(e);
    return e;
  }

  function Chart(container, spec) {
    this.el = container;
    this.spec = {};
    this.hover = null;
    this.hits = [];
    container.classList.add("plot");
    if (!container.hasAttribute("role")) container.setAttribute("role", "group");
    this.legendEl = el("div", "plot-legend", container);
    this.area = el("div", "plot-area", container);
    this.canvas = el("canvas", "plot-canvas", this.area);
    this.canvas.setAttribute("role", "img");
    this.canvas.tabIndex = 0;
    this.tip = el("div", "plot-tip", this.area);
    this.tip.hidden = true;
    this.tip.setAttribute("aria-live", "polite");
    this.tableEl = null;
    this._bind();
    this.update(spec || {});
    live.push(this);
  }

  Chart.prototype.update = function (patch) {
    for (var k in patch) if (Object.prototype.hasOwnProperty.call(patch, k)) this.spec[k] = patch[k];
    this._legend();
    this._table();
    var s = this.spec;
    var label = (s.title || "Chart") + (s.desc ? ". " + s.desc : "");
    this.canvas.setAttribute("aria-label", label);
    this.render();
    return this;
  };

  Chart.prototype.destroy = function () {
    var i = live.indexOf(this);
    if (i >= 0) live.splice(i, 1);
    if (this._ro) this._ro.disconnect();
    while (this.el.firstChild) this.el.removeChild(this.el.firstChild);
  };

  Chart.prototype.resize = function () { this.render(); };

  Chart.prototype._kind = function () {
    var k = this.spec.kind || "line";
    return k;
  };

  Chart.prototype._axes = function () {
    var s = this.spec, k = this._kind();
    var x = s.x || {}, y = s.y || {};
    return {
      x: x, y: y,
      xLog: !!(x.log || k === "loglog"),
      yLog: !!(y.log || k === "semilog" || k === "loglog")
    };
  };

  Chart.prototype._seriesList = function () {
    var s = this.spec, ax = this._axes(), out = [];
    var list = s.series || [];
    var xDom = (ax.x.min !== undefined && ax.x.max !== undefined) ? [ax.x.min, ax.x.max] : null;
    for (var i = 0; i < list.length; i++) {
      var d = seriesXY(list[i], xDom, ax.xLog);
      out.push({ src: list[i], x: d.x, y: d.y, color: color(list[i].color || "s" + ((i % 5) + 1), this.el) });
    }
    if (s.stack && out.length > 1) {
      var base = out[0].x;
      var acc = out[0].y.slice();
      for (var j = 1; j < out.length; j++) {
        out[j].y0 = acc.slice();
        for (var m = 0; m < acc.length; m++) acc[m] += (out[j].y[m] || 0);
        out[j].y = acc.slice();
        out[j].x = base;
      }
      out[0].y0 = base.map(function () { return 0; });
    }
    return out;
  };

  Chart.prototype._legend = function () {
    var s = this.spec, k = this._kind(), items = [];
    var le = this.legendEl;
    while (le.firstChild) le.removeChild(le.firstChild);
    if (k === "gantt" || k === "bar") {
      /* rows and bars label themselves; an explicit legend list may still be given */
      items = s.legendItems || [];
    } else {
      var list = s.series || [];
      for (var i = 0; i < list.length; i++) {
        if (list[i].legend === false) continue;
        items.push({ label: list[i].label || list[i].key || "series " + (i + 1),
          color: list[i].color || "s" + ((i % 5) + 1), dash: list[i].dash,
          shape: (k === "area" || list[i].area) ? "area" : "line" });
      }
    }
    var show = s.legend === true || (s.legend !== false && items.length >= 2);
    le.hidden = !show;
    if (!show) return;
    for (var j = 0; j < items.length; j++) {
      var it = el("span", "plot-legend-item", le);
      var key = el("span", "plot-key plot-key--" + (items[j].shape || "line") + (items[j].dash ? " plot-key--dash" : ""), it);
      key.style.setProperty("--key", color(items[j].color, this.el));
      var t = el("span", "", it);
      t.textContent = items[j].label;
    }
  };

  Chart.prototype._table = function () {
    var s = this.spec;
    if (!s.table) {
      if (this.tableEl) { this.tableEl.parentNode.removeChild(this.tableEl); this.tableEl = null; }
      return;
    }
    if (!this.tableEl) {
      this.tableEl = el("details", "plot-table", this.el);
      el("summary", "", this.tableEl).textContent = "Data table";
      el("div", "table-wrap", this.tableEl);
    }
    var wrap = this.tableEl.lastChild;
    while (wrap.firstChild) wrap.removeChild(wrap.firstChild);
    var tbl = el("table", "", wrap);
    var thead = el("thead", "", tbl), tbody = el("tbody", "", tbl);
    var hr = el("tr", "", thead);
    function th(text) { el("th", "", hr).textContent = text; }
    function row(cells) {
      var tr = el("tr", "", tbody);
      for (var c = 0; c < cells.length; c++) el("td", "num", tr).textContent = cells[c];
    }
    var k = this._kind(), ax = this._axes(), i;
    var xl = (ax.x.label || "x") + (ax.x.unit ? " (" + ax.x.unit + ")" : "");
    if (k === "gantt") {
      th("row"); th("from" + (ax.x.unit ? " (" + ax.x.unit + ")" : "")); th("to"); th("label");
      var rows = s.rows || [];
      for (i = 0; i < rows.length; i++) {
        for (var q = 0; q < (rows[i].spans || []).length; q++) {
          var sp = rows[i].spans[q];
          row([rows[i].label, fmt(sp.from, 4), fmt(sp.to, 4), sp.label || ""]);
        }
      }
      return;
    }
    if (k === "bar") {
      th("item"); th((ax.y.label || "value") + (ax.y.unit ? " (" + ax.y.unit + ")" : ""));
      var bars = s.bars || [];
      for (i = 0; i < bars.length; i++) row([bars[i].label, fmt(bars[i].value, 4)]);
      return;
    }
    var ser = this._seriesList();
    if (!ser.length) return;
    th(xl);
    for (i = 0; i < ser.length; i++) th((ser[i].src.label || "series " + (i + 1)) + (ax.y.unit ? " (" + ax.y.unit + ")" : ""));
    var xs = ser[0].x, n = xs.length, maxRows = s.tableRows || 40;
    var stride = Math.max(1, Math.ceil(n / maxRows));
    for (var r = 0; r < n; r += stride) {
      var cells = [fmt(xs[r], 4)];
      for (var c2 = 0; c2 < ser.length; c2++) {
        var idx = ser[c2].x === xs ? r : nearestIndex(ser[c2].x, xs[r]);
        cells.push(idx >= 0 ? fmt(ser[c2].y[idx], 4) : "–");
      }
      row(cells);
    }
  };

  Chart.prototype._size = function () {
    var s = this.spec;
    var w = Math.max(120, Math.floor(this.area.clientWidth || this.el.clientWidth || 480));
    var h;
    if (s.height) h = s.height;
    else if (s.aspect) h = Math.round(w * s.aspect);
    else if (this._kind() === "gantt") h = 34 + (s.rows ? s.rows.length : 1) * 30 + (s.x && s.x.label ? 18 : 0);
    else if (this._kind() === "bar" && (s.orient || "h") === "h") h = 20 + (s.bars ? s.bars.length : 1) * 34;
    else h = 260;
    if (s.minHeight) h = Math.max(h, s.minHeight);
    if (s.maxHeight) h = Math.min(h, s.maxHeight);
    return { w: w, h: h };
  };

  Chart.prototype.render = function () {
    if (!doc) return;
    var sz = this._size();
    var cv = setupCanvas(this.canvas, sz.w, sz.h);
    this.hits = [];
    var k = this._kind();
    var T = theme(this.el);
    if (k === "gantt") this._drawGantt(cv, T);
    else if (k === "bar") this._drawBars(cv, T);
    else this._drawXY(cv, T);
  };

  function font(T, size, weight) {
    return (weight || "400") + " " + size + "px " + T.sans;
  }

  function hline(ctx, x0, x1, y, dpr) {
    var yy = Math.round(y * dpr) / dpr + 0.5 / dpr;
    ctx.beginPath(); ctx.moveTo(x0, yy); ctx.lineTo(x1, yy); ctx.stroke();
  }
  function vline(ctx, x, y0, y1, dpr) {
    var xx = Math.round(x * dpr) / dpr + 0.5 / dpr;
    ctx.beginPath(); ctx.moveTo(xx, y0); ctx.lineTo(xx, y1); ctx.stroke();
  }

  function dot(ctx, x, y, fill, ring) {
    ctx.beginPath(); ctx.arc(x, y, 6, 0, 2 * Math.PI); ctx.fillStyle = ring; ctx.fill();
    ctx.beginPath(); ctx.arc(x, y, 4, 0, 2 * Math.PI); ctx.fillStyle = fill; ctx.fill();
  }

  /* Line, semilog, loglog and area charts. */
  Chart.prototype._drawXY = function (cv, T) {
    var ctx = cv.ctx, s = this.spec, ax = this._axes(), k = this._kind();
    var ser = this._seriesList();
    var xs = [], ys = [], i;
    for (i = 0; i < ser.length; i++) { xs.push(ser[i].x); ys.push(ser[i].y); if (ser[i].y0) ys.push(ser[i].y0); }
    var pts = s.points || [];
    for (i = 0; i < pts.length; i++) { xs.push([pts[i].x]); ys.push([pts[i].y]); }
    var ex = extent(xs, ax.xLog) || [0, 1], ey = extent(ys, ax.yLog) || [ax.yLog ? 1 : 0, ax.yLog ? 10 : 1];
    var xd = [ax.x.min !== undefined ? ax.x.min : ex[0], ax.x.max !== undefined ? ax.x.max : ex[1]];
    var yd = [ax.y.min !== undefined ? ax.y.min : ey[0], ax.y.max !== undefined ? ax.y.max : ey[1]];
    if (!ax.yLog && (ax.y.zero || k === "area") && ax.y.min === undefined) yd[0] = Math.min(0, yd[0]);
    var xt, yt, yMinor = [], xMinor = [];
    if (ax.xLog) { var lx = logTicks(xd[0], xd[1]); xt = { ticks: lx.major, step: 0 }; xMinor = lx.minor; }
    else {
      xt = niceTicks(xd[0], xd[1], Math.max(2, Math.min(8, Math.floor(cv.w / 90))));
      if (ax.x.min === undefined && ax.x.nice) xd[0] = xt.lo;
      if (ax.x.max === undefined && ax.x.nice) xd[1] = xt.hi;
      xt.ticks = xt.ticks.filter(function (v) { return v >= xd[0] - 1e-12 && v <= xd[1] + 1e-12; });
    }
    if (ax.yLog) { var ly = logTicks(yd[0], yd[1]); yt = { ticks: ly.major, step: 0 }; yMinor = ly.minor; }
    else {
      yt = niceTicks(yd[0], yd[1], Math.max(2, Math.min(6, Math.floor(cv.h / 50))));
      if (ax.y.min === undefined) yd[0] = yt.lo;
      if (ax.y.max === undefined) yd[1] = yt.hi;
    }
    var fx = tickFormatter(withLog(ax.x, ax.xLog), xt.ticks, xt.step), fy = tickFormatter(withLog(ax.y, ax.yLog), yt.ticks, yt.step);

    ctx.font = font(T, 11.5);
    var labW = 0;
    for (i = 0; i < yt.ticks.length; i++) labW = Math.max(labW, ctx.measureText(fy(yt.ticks[i])).width);
    var endW = 0;
    for (i = 0; i < ser.length; i++) {
      if (ser[i].src.endLabel) endW = Math.max(endW, ctx.measureText(String(ser[i].src.endLabel === true ? ser[i].src.label : ser[i].src.endLabel)).width + 18);
    }
    var yTitle = ax.y.label ? (ax.y.label + (ax.y.unit && !ax.y.si ? ", " + ax.y.unit : "")) : "";
    var xTitle = ax.x.label ? (ax.x.label + (ax.x.unit && !ax.x.si ? ", " + ax.x.unit : "")) : "";
    var L = Math.ceil(labW) + 10, R = Math.max(12, Math.ceil(endW)), Tp = yTitle ? 22 : 10, B = 24 + (xTitle ? 16 : 0);
    var pw = cv.w - L - R, ph = cv.h - Tp - B;
    if (pw < 40 || ph < 30) return;
    var X = scale(xd[0], xd[1], L, L + pw, ax.xLog), Y = scale(yd[0], yd[1], Tp + ph, Tp, ax.yLog);
    this._geom = { X: X, Y: Y, L: L, R: R, Tp: Tp, pw: pw, ph: ph, ser: ser, fx: fx, fy: fy, xd: xd, yd: yd };

    /* bands */
    var bands = s.bands || [];
    for (i = 0; i < bands.length; i++) {
      var b = bands[i], bc = color(b.color || "accent", this.el);
      ctx.fillStyle = alpha(bc, b.alpha || 0.1);
      if (b.axis === "y") {
        var y0 = Y(Math.min(Math.max(b.from, yd[0]), yd[1])), y1 = Y(Math.min(Math.max(b.to, yd[0]), yd[1]));
        ctx.fillRect(L, Math.min(y0, y1), pw, Math.abs(y1 - y0));
      } else {
        var x0 = X(Math.max(b.from, xd[0])), x1 = X(Math.min(b.to, xd[1]));
        ctx.fillRect(Math.min(x0, x1), Tp, Math.abs(x1 - x0), ph);
        if (b.label) { ctx.fillStyle = T.muted; ctx.font = font(T, 11); ctx.textAlign = "left"; ctx.textBaseline = "top"; ctx.fillText(b.label, Math.min(x0, x1) + 4, Tp + 3); }
      }
    }

    /* grid */
    ctx.lineWidth = 1;
    ctx.strokeStyle = alpha(T.grid, 0.55);
    for (i = 0; i < yMinor.length; i++) hline(ctx, L, L + pw, Y(yMinor[i]), cv.dpr);
    for (i = 0; i < xMinor.length; i++) vline(ctx, X(xMinor[i]), Tp, Tp + ph, cv.dpr);
    ctx.strokeStyle = T.grid;
    for (i = 0; i < yt.ticks.length; i++) hline(ctx, L, L + pw, Y(yt.ticks[i]), cv.dpr);
    if (ax.xLog || s.xgrid) for (i = 0; i < xt.ticks.length; i++) vline(ctx, X(xt.ticks[i]), Tp, Tp + ph, cv.dpr);
    ctx.strokeStyle = T.axis;
    hline(ctx, L, L + pw, Tp + ph, cv.dpr);

    /* series */
    ctx.save();
    ctx.beginPath(); ctx.rect(L, Tp - 1, pw, ph + 2); ctx.clip();
    for (i = 0; i < ser.length; i++) this._drawSeries(ctx, ser[i], X, Y, ax, k, cv, yd, i);
    ctx.restore();

    /* reference lines */
    var refs = s.refs || [];
    ctx.font = font(T, 11);
    for (i = 0; i < refs.length; i++) {
      var r = refs[i], rc = color(r.color || "ref", this.el);
      ctx.strokeStyle = rc; ctx.lineWidth = r.width || 1.25; ctx.setLineDash(r.dash || [5, 4]);
      if (r.axis === "x") {
        if (r.value < xd[0] || r.value > xd[1]) { ctx.setLineDash([]); continue; }
        vline(ctx, X(r.value), Tp, Tp + ph, cv.dpr);
        ctx.setLineDash([]);
        if (r.label) {
          ctx.fillStyle = T.ink2; ctx.textBaseline = "top";
          var lw = ctx.measureText(r.label).width, lx2 = X(r.value) + 4;
          ctx.textAlign = lx2 + lw > L + pw ? "right" : "left";
          ctx.fillText(r.label, ctx.textAlign === "right" ? X(r.value) - 4 : lx2, Tp + 2 + (r.row || 0) * 14);
        }
      } else {
        var yy = Y(r.value);
        if (!(yy >= Tp - 0.5 && yy <= Tp + ph + 0.5)) { ctx.setLineDash([]); continue; }
        hline(ctx, L, L + pw, yy, cv.dpr);
        ctx.setLineDash([]);
        if (r.label) {
          ctx.fillStyle = T.ink2; ctx.textAlign = r.align === "left" ? "left" : "right"; ctx.textBaseline = "bottom";
          ctx.fillText(r.label, r.align === "left" ? L + 4 : L + pw - 4, yy - 3);
        }
      }
    }
    ctx.setLineDash([]);

    /* annotated points */
    for (i = 0; i < pts.length; i++) {
      var p = pts[i], px = X(p.x), py = Y(p.y);
      if (!isFinite(px) || !isFinite(py)) continue;
      dot(ctx, px, py, color(p.color || "accent", this.el), T.panel);
      if (p.label) {
        ctx.fillStyle = T.ink2; ctx.font = font(T, 11.5, "600"); ctx.textBaseline = "middle";
        var tw = ctx.measureText(p.label).width;
        var right = px + 10 + tw < L + pw;
        ctx.textAlign = right ? "left" : "right";
        ctx.fillText(p.label, right ? px + 10 : px - 10, py + (p.dy || -10));
      }
      this.hits.push({ kind: "point", x: px, y: py, r: 12, item: p });
    }

    /* end labels */
    ctx.font = font(T, 11.5);
    for (i = 0; i < ser.length; i++) {
      var el2 = ser[i].src.endLabel;
      if (!el2 || !ser[i].x.length) continue;
      var n = ser[i].x.length - 1;
      var ex2 = X(ser[i].x[n]), ey2 = Y(ser[i].y[n]);
      if (!isFinite(ey2)) continue;
      ctx.fillStyle = ser[i].color; ctx.beginPath(); ctx.arc(ex2 + 6, ey2, 3, 0, 2 * Math.PI); ctx.fill();
      ctx.fillStyle = T.ink2; ctx.textAlign = "left"; ctx.textBaseline = "middle";
      ctx.fillText(String(el2 === true ? ser[i].src.label : el2), ex2 + 12, ey2);
    }

    /* tick labels and titles */
    ctx.fillStyle = T.muted; ctx.font = font(T, 11.5);
    ctx.textAlign = "right"; ctx.textBaseline = "middle";
    for (i = 0; i < yt.ticks.length; i++) ctx.fillText(fy(yt.ticks[i]), L - 6, Y(yt.ticks[i]));
    ctx.textAlign = "center"; ctx.textBaseline = "top";
    var lastRight = -Infinity;
    for (i = 0; i < xt.ticks.length; i++) {
      var tx = X(xt.ticks[i]), lab = fx(xt.ticks[i]), w2 = ctx.measureText(lab).width;
      if (tx - w2 / 2 < lastRight + 6) continue;
      ctx.fillText(lab, Math.min(Math.max(tx, L + w2 / 2), L + pw + R - w2 / 2), Tp + ph + 6);
      lastRight = tx + w2 / 2;
    }
    ctx.fillStyle = T.ink2; ctx.font = font(T, 11.5, "600");
    if (yTitle) { ctx.textAlign = "left"; ctx.textBaseline = "top"; ctx.fillText(yTitle, Math.max(2, L - labW - 6), 2); }
    if (xTitle) { ctx.textAlign = "right"; ctx.textBaseline = "bottom"; ctx.fillText(xTitle, L + pw, cv.h - 1); }

    /* crosshair */
    if (this.hover && this.hover.kind === "x" && ser.length) this._crosshair(ctx, T, cv);
  };

  Chart.prototype._drawSeries = function (ctx, sd, X, Y, ax, k, cv, yd, idx) {
    var src = sd.src, xs = sd.x, ys = sd.y, n = xs.length;
    if (!n) return;
    var reveal = src.reveal !== undefined ? src.reveal : this.spec.reveal;
    var path = [], i;
    for (i = 0; i < n; i++) {
      var x = xs[i], y = ys[i];
      if (reveal !== undefined && x > reveal) {
        if (i > 0 && xs[i - 1] <= reveal && x !== xs[i - 1]) {
          var f = (reveal - xs[i - 1]) / (x - xs[i - 1]);
          var yi = ys[i - 1] + f * (y - ys[i - 1]);
          if (!(ax.yLog && yi <= 0)) path.push([X(reveal), Y(yi)]);
        }
        break;
      }
      if (typeof y !== "number" || !isFinite(y) || (ax.yLog && y <= 0) || (ax.xLog && x <= 0)) { path.push(null); continue; }
      path.push([X(x), Y(y)]);
    }
    /* decimate dense series to min/max per pixel column */
    if (path.length > cv.w * 3) {
      var dec = [], col = null, mn = null, mx = null, first = null, last = null;
      for (i = 0; i < path.length; i++) {
        var q = path[i];
        if (!q) { if (first) { dec.push(first, mn, mx, last); first = null; } dec.push(null); col = null; continue; }
        var c = Math.round(q[0]);
        if (c !== col) {
          if (first) dec.push(first, mn, mx, last);
          col = c; first = q; mn = q; mx = q; last = q;
        } else {
          if (q[1] < mn[1]) mn = q;
          if (q[1] > mx[1]) mx = q;
          last = q;
        }
      }
      if (first) dec.push(first, mn, mx, last);
      path = dec;
    }
    var fillArea = k === "area" || src.area;
    if (fillArea) {
      var baseY = ax.yLog ? Y(yd[0]) : Y(Math.max(yd[0], Math.min(0, yd[1])));
      ctx.fillStyle = alpha(sd.color, this.spec.stack ? 0.22 : 0.1);
      ctx.beginPath();
      var started = false, firstX = 0, lastX = 0;
      for (i = 0; i < path.length; i++) {
        var pt = path[i];
        if (!pt) continue;
        if (!started) { ctx.moveTo(pt[0], baseY); ctx.lineTo(pt[0], pt[1]); firstX = pt[0]; started = true; }
        else ctx.lineTo(pt[0], pt[1]);
        lastX = pt[0];
      }
      if (started) {
        if (sd.y0) {
          for (i = Math.min(sd.y0.length, xs.length) - 1; i >= 0; i--) {
            if (reveal !== undefined && xs[i] > reveal) continue;
            ctx.lineTo(X(xs[i]), Y(sd.y0[i]));
          }
        } else {
          ctx.lineTo(lastX, baseY); ctx.lineTo(firstX, baseY);
        }
        ctx.closePath(); ctx.fill();
      }
    }
    ctx.strokeStyle = sd.color;
    ctx.lineWidth = src.width || 2;
    ctx.lineJoin = "round"; ctx.lineCap = "round";
    ctx.setLineDash(src.dash || []);
    ctx.beginPath();
    var pen = false;
    for (i = 0; i < path.length; i++) {
      var p2 = path[i];
      if (!p2) { pen = false; continue; }
      if (!pen) { ctx.moveTo(p2[0], p2[1]); pen = true; } else ctx.lineTo(p2[0], p2[1]);
    }
    ctx.stroke();
    ctx.setLineDash([]);
    if (src.markers) {
      for (i = 0; i < path.length; i++) if (path[i]) dot(ctx, path[i][0], path[i][1], sd.color, theme(this.el).panel);
    }
  };

  Chart.prototype._crosshair = function (ctx, T, cv) {
    var g = this._geom, h = this.hover;
    var x = h.x;
    var ref = g.ser[0];
    var ix = nearestIndex(ref.x, x);
    if (ix < 0) return;
    var xv = ref.x[ix];
    var reveal = this.spec.reveal;
    if (reveal !== undefined && xv > reveal) return;
    var px = g.X(xv);
    ctx.strokeStyle = T.muted; ctx.lineWidth = 1;
    vline(ctx, px, g.Tp, g.Tp + g.ph, cv.dpr);
    var rows = [];
    for (var i = 0; i < g.ser.length; i++) {
      var sd = g.ser[i];
      var j = sd.x === ref.x ? ix : nearestIndex(sd.x, xv);
      if (j < 0) continue;
      var yv = sd.y[j];
      if (typeof yv !== "number" || !isFinite(yv)) continue;
      var py = g.Y(yv);
      if (isFinite(py) && py >= g.Tp - 1 && py <= g.Tp + g.ph + 1) dot(ctx, g.X(sd.x[j]), py, sd.color, T.panel);
      var shown = sd.y0 ? yv - sd.y0[j] : yv;
      rows.push({ color: sd.color, label: sd.src.label || sd.src.key || "series " + (i + 1),
        value: fmtAxis(this._axes().y, shown) });
    }
    var axx = this._axes();
    var head = (axx.x.label ? axx.x.label + " " : "") + fmtAxis(axx.x, xv);
    this._showTip(px, h.y, head, rows);
  };

  Chart.prototype._showTip = function (px, py, head, rows) {
    if (this.spec.tooltip === false) return;
    var tip = this.tip;
    while (tip.firstChild) tip.removeChild(tip.firstChild);
    if (head) el("div", "plot-tip-head", tip).textContent = head;
    for (var i = 0; i < rows.length; i++) {
      var r = el("div", "plot-tip-row", tip);
      if (rows[i].color) { var key = el("span", "plot-key plot-key--line", r); key.style.setProperty("--key", rows[i].color); }
      el("strong", "", r).textContent = rows[i].value;
      el("span", "", r).textContent = rows[i].label;
    }
    tip.hidden = false;
    var aw = this.area.clientWidth, tw = tip.offsetWidth, th = tip.offsetHeight;
    var left = px + 14;
    if (left + tw > aw) left = Math.max(0, px - 14 - tw);
    var top = Math.max(0, Math.min((py || 20) - th / 2, this.canvas.clientHeight - th));
    tip.style.left = left + "px";
    tip.style.top = top + "px";
  };

  /* Gantt: rows of time spans with vertical markers from refs (axis "x"). */
  Chart.prototype._drawGantt = function (cv, T) {
    var ctx = cv.ctx, s = this.spec, ax = this._axes(), rows = s.rows || [], i, j;
    var lo = Infinity, hi = -Infinity;
    for (i = 0; i < rows.length; i++) for (j = 0; j < (rows[i].spans || []).length; j++) {
      lo = Math.min(lo, rows[i].spans[j].from); hi = Math.max(hi, rows[i].spans[j].to);
    }
    var refs = s.refs || [];
    for (i = 0; i < refs.length; i++) if (refs[i].axis !== "y") { lo = Math.min(lo, refs[i].value); hi = Math.max(hi, refs[i].value); }
    if (!(lo <= hi)) { lo = 0; hi = 1; }
    var xd = [ax.x.min !== undefined ? ax.x.min : lo, ax.x.max !== undefined ? ax.x.max : hi];
    var xt = niceTicks(xd[0], xd[1], Math.max(2, Math.min(8, Math.floor(cv.w / 90))));
    if (ax.x.min === undefined) xd[0] = xt.lo;
    if (ax.x.max === undefined) xd[1] = xt.hi;
    var fx = tickFormatter(ax.x, xt.ticks, xt.step);
    ctx.font = font(T, 12);
    var labW = 0;
    for (i = 0; i < rows.length; i++) labW = Math.max(labW, ctx.measureText(rows[i].label || "").width);
    var xTitle = ax.x.label ? (ax.x.label + (ax.x.unit && !ax.x.si ? ", " + ax.x.unit : "")) : "";
    var hasTopLabels = refs.some(function (r) { return r.label; });
    var L = Math.min(Math.ceil(labW) + 14, Math.floor(cv.w * 0.42)), R = 14, Tp = hasTopLabels ? 20 : 8, B = 24 + (xTitle ? 16 : 0);
    var pw = cv.w - L - R, ph = cv.h - Tp - B;
    if (pw < 40 || ph < 10) return;
    var X = scale(xd[0], xd[1], L, L + pw, false);
    var rowH = ph / Math.max(1, rows.length);
    this._geom = { X: X, L: L, Tp: Tp, pw: pw, ph: ph };
    ctx.strokeStyle = T.grid; ctx.lineWidth = 1;
    for (i = 0; i < xt.ticks.length; i++) vline(ctx, X(xt.ticks[i]), Tp, Tp + ph, cv.dpr);
    for (i = 0; i < rows.length; i++) {
      var r = rows[i], cy = Tp + (i + 0.5) * rowH, bh = Math.min(18, rowH - 8);
      ctx.fillStyle = T.ink2; ctx.font = font(T, 12); ctx.textAlign = "right"; ctx.textBaseline = "middle";
      ctx.fillText(r.label || "", L - 8, cy, L - 10);
      for (j = 0; j < (r.spans || []).length; j++) {
        var sp = r.spans[j], c = color(sp.color || r.color || "s1", this.el);
        var x0 = X(Math.max(sp.from, xd[0])), x1 = X(Math.min(sp.to, xd[1]));
        var w = Math.max(2, x1 - x0 - 2);
        var hovered = this.hover && this.hover.kind === "mark" && this.hover.item === sp;
        if (sp.style === "hatch") {
          ctx.fillStyle = alpha(c, 0.18); roundRect(ctx, x0 + 1, cy - bh / 2, w, bh, 4); ctx.fill();
          ctx.save(); roundRect(ctx, x0 + 1, cy - bh / 2, w, bh, 4); ctx.clip();
          ctx.strokeStyle = alpha(c, 0.7); ctx.lineWidth = 1.25;
          for (var hx = x0 - bh; hx < x0 + w + bh; hx += 6) { ctx.beginPath(); ctx.moveTo(hx, cy + bh / 2); ctx.lineTo(hx + bh, cy - bh / 2); ctx.stroke(); }
          ctx.restore();
        } else {
          ctx.fillStyle = c; roundRect(ctx, x0 + 1, cy - bh / 2, w, bh, 4); ctx.fill();
        }
        if (hovered) { ctx.strokeStyle = T.ink; ctx.lineWidth = 2; roundRect(ctx, x0 + 1, cy - bh / 2, w, bh, 4); ctx.stroke(); }
        if (sp.label) {
          ctx.font = font(T, 11, "600");
          var tw = ctx.measureText(sp.label).width;
          if (tw + 10 < w && sp.style !== "hatch") {
            ctx.fillStyle = "#fff"; ctx.textAlign = "left"; ctx.fillText(sp.label, x0 + 6, cy + 0.5);
          }
        }
        this.hits.push({ kind: "rect", x: x0, y: cy - rowH / 2, w: Math.max(w, 8), h: rowH, item: sp, row: r });
      }
    }
    ctx.strokeStyle = T.axis; hline(ctx, L, L + pw, Tp + ph, cv.dpr);
    ctx.font = font(T, 11);
    for (i = 0; i < refs.length; i++) {
      var rf = refs[i];
      if (rf.axis === "y" || rf.value < xd[0] || rf.value > xd[1]) continue;
      ctx.strokeStyle = color(rf.color || "ref", this.el); ctx.lineWidth = rf.width || 1.5; ctx.setLineDash(rf.dash || []);
      vline(ctx, X(rf.value), Tp - 4, Tp + ph, cv.dpr);
      ctx.setLineDash([]);
      if (rf.label) {
        ctx.fillStyle = T.ink2; ctx.textBaseline = "top";
        var lw = ctx.measureText(rf.label).width;
        ctx.textAlign = X(rf.value) + lw / 2 > L + pw ? "right" : X(rf.value) - lw / 2 < L ? "left" : "center";
        ctx.fillText(rf.label, X(rf.value), 2);
      }
    }
    ctx.fillStyle = T.muted; ctx.font = font(T, 11.5); ctx.textAlign = "center"; ctx.textBaseline = "top";
    for (i = 0; i < xt.ticks.length; i++) ctx.fillText(fx(xt.ticks[i]), X(xt.ticks[i]), Tp + ph + 6);
    if (xTitle) { ctx.fillStyle = T.ink2; ctx.font = font(T, 11.5, "600"); ctx.textAlign = "right"; ctx.textBaseline = "bottom"; ctx.fillText(xTitle, L + pw, cv.h - 1); }
  };

  /* Bars from a zero baseline; horizontal by default (labels read left to right). */
  Chart.prototype._drawBars = function (cv, T) {
    var ctx = cv.ctx, s = this.spec, ax = this._axes(), bars = s.bars || [], i;
    var horiz = (s.orient || "h") === "h";
    var vals = bars.map(function (b) { return b.value; });
    var e = extent([vals], false) || [0, 1];
    var dLo = ax.y.min !== undefined ? ax.y.min : Math.min(0, e[0]);
    var dHi = ax.y.max !== undefined ? ax.y.max : Math.max(0, e[1]);
    var tk = niceTicks(dLo, dHi, horiz ? Math.max(2, Math.floor(cv.w / 110)) : 5);
    if (ax.y.min === undefined) dLo = tk.lo;
    if (ax.y.max === undefined) dHi = tk.hi;
    var fv = tickFormatter(ax.y, tk.ticks, tk.step);
    var fVal = function (v) { return ax.y.si ? fmtSI(v, ax.y.unit, 3) : fmt(v, 3) + (ax.y.unit ? " " + ax.y.unit : ""); };
    ctx.font = font(T, 12);
    var labW = 0;
    for (i = 0; i < bars.length; i++) labW = Math.max(labW, ctx.measureText(bars[i].label || "").width);
    var valW = 0;
    ctx.font = font(T, 11.5, "600");
    for (i = 0; i < bars.length; i++) valW = Math.max(valW, ctx.measureText(fVal(bars[i].value)).width);
    var L, R, Tp, B, pw, ph, V, band;
    if (horiz) {
      L = Math.min(Math.ceil(labW) + 12, Math.floor(cv.w * 0.45)); R = Math.ceil(valW) + 12; Tp = 6; B = 22;
      pw = cv.w - L - R; ph = cv.h - Tp - B;
      if (pw < 30 || ph < 10) return;
      V = scale(dLo, dHi, L, L + pw, false);
      band = ph / Math.max(1, bars.length);
    } else {
      ctx.font = font(T, 11.5);
      var tw = 0;
      for (i = 0; i < tk.ticks.length; i++) tw = Math.max(tw, ctx.measureText(fv(tk.ticks[i])).width);
      L = Math.ceil(tw) + 10; R = 8; Tp = 20; B = 26;
      pw = cv.w - L - R; ph = cv.h - Tp - B;
      if (pw < 30 || ph < 30) return;
      V = scale(dLo, dHi, Tp + ph, Tp, false);
      band = pw / Math.max(1, bars.length);
    }
    ctx.strokeStyle = T.grid; ctx.lineWidth = 1;
    for (i = 0; i < tk.ticks.length; i++) {
      if (horiz) vline(ctx, V(tk.ticks[i]), Tp, Tp + ph, cv.dpr); else hline(ctx, L, L + pw, V(tk.ticks[i]), cv.dpr);
    }
    var thick = Math.min(24, band * 0.62);
    var zero = V(Math.min(Math.max(0, dLo), dHi));
    for (i = 0; i < bars.length; i++) {
      var b = bars[i], c = color(b.color || "s1", this.el), v = b.value;
      var hovered = this.hover && this.hover.kind === "mark" && this.hover.item === b;
      var p = V(v), mid = (horiz ? Tp : L) + (i + 0.5) * band;
      ctx.fillStyle = c;
      if (horiz) {
        var x0 = Math.min(zero, p), w = Math.abs(p - zero);
        roundRect(ctx, x0, mid - thick / 2, Math.max(w, 1), thick, v >= 0 ? [0, 4, 4, 0] : [4, 0, 0, 4]); ctx.fill();
        if (hovered) { ctx.strokeStyle = T.ink; ctx.lineWidth = 2; ctx.stroke(); }
        ctx.fillStyle = T.ink2; ctx.font = font(T, 12); ctx.textAlign = "right"; ctx.textBaseline = "middle";
        ctx.fillText(b.label || "", L - 8, mid, L - 10);
        ctx.font = font(T, 11.5, "600"); ctx.fillStyle = T.ink;
        ctx.textAlign = v >= 0 ? "left" : "right";
        ctx.fillText(fVal(v), v >= 0 ? p + 6 : p - 6, mid);
        this.hits.push({ kind: "rect", x: L, y: mid - band / 2, w: pw + R, h: band, item: b });
      } else {
        var y0 = Math.min(zero, p), hgt = Math.abs(p - zero);
        roundRect(ctx, mid - thick / 2, y0, thick, Math.max(hgt, 1), v >= 0 ? [4, 4, 0, 0] : [0, 0, 4, 4]); ctx.fill();
        if (hovered) { ctx.strokeStyle = T.ink; ctx.lineWidth = 2; ctx.stroke(); }
        ctx.fillStyle = T.ink2; ctx.font = font(T, 11.5); ctx.textAlign = "center"; ctx.textBaseline = "top";
        ctx.fillText(b.label || "", mid, Tp + ph + 6, band - 4);
        ctx.fillStyle = T.ink; ctx.font = font(T, 11.5, "600"); ctx.textBaseline = v >= 0 ? "bottom" : "top";
        var vl = fVal(v);
        if (ctx.measureText(vl).width < band - 2) ctx.fillText(vl, mid, v >= 0 ? p - 4 : p + 4);
        this.hits.push({ kind: "rect", x: mid - band / 2, y: Tp, w: band, h: ph, item: b });
      }
    }
    ctx.strokeStyle = T.axis;
    if (horiz) vline(ctx, zero, Tp, Tp + ph, cv.dpr); else hline(ctx, L, L + pw, zero, cv.dpr);
    if (!horiz) {
      ctx.fillStyle = T.muted; ctx.font = font(T, 11.5); ctx.textAlign = "right"; ctx.textBaseline = "middle";
      for (i = 0; i < tk.ticks.length; i++) ctx.fillText(fv(tk.ticks[i]), L - 6, V(tk.ticks[i]));
    } else {
      ctx.fillStyle = T.muted; ctx.font = font(T, 11.5); ctx.textAlign = "center"; ctx.textBaseline = "top";
      for (i = 0; i < tk.ticks.length; i++) ctx.fillText(fv(tk.ticks[i]), V(tk.ticks[i]), Tp + ph + 6);
    }
    this._fVal = fVal;
  };

  Chart.prototype._markTip = function (hit) {
    var k = this._kind(), it = hit.item, ax = this._axes();
    if (k === "gantt") {
      var u = ax.x.unit ? " " + ax.x.unit : "";
      this._showTip(hit.x + hit.w / 2, hit.y + hit.h / 2, hit.row ? hit.row.label : "",
        [{ color: color(it.color || (hit.row && hit.row.color) || "s1", this.el),
          value: fmt(it.from, 3) + "–" + fmt(it.to, 3) + u, label: it.label || "" }]);
    } else if (k === "bar") {
      this._showTip(hit.x + hit.w / 2, hit.y + hit.h / 2, it.label || "",
        [{ color: color(it.color || "s1", this.el), value: this._fVal ? this._fVal(it.value) : fmt(it.value, 3), label: it.note || "" }]);
    } else if (hit.kind === "point") {
      this._showTip(hit.x, hit.y, it.label || "",
        [{ color: color(it.color || "accent", this.el), value: fmtAxis(ax.y, it.y),
          label: "at " + fmtAxis(ax.x, it.x) }]);
    }
  };

  Chart.prototype._bind = function () {
    var self = this, cv = this.canvas;
    function pos(ev) {
      var r = cv.getBoundingClientRect();
      return { x: ev.clientX - r.left, y: ev.clientY - r.top };
    }
    function hitAt(p) {
      for (var i = self.hits.length - 1; i >= 0; i--) {
        var h = self.hits[i];
        if (h.kind === "point" && Math.abs(p.x - h.x) <= h.r && Math.abs(p.y - h.y) <= h.r) return h;
        if (h.kind === "rect" && p.x >= h.x && p.x <= h.x + h.w && p.y >= h.y && p.y <= h.y + h.h) return h;
      }
      return null;
    }
    function move(ev) {
      if (self.spec.tooltip === false) return;
      var p = pos(ev), k = self._kind();
      var hit = hitAt(p);
      if (k === "gantt" || k === "bar" || (hit && hit.kind === "point")) {
        self.hover = hit ? { kind: "mark", item: hit.item } : null;
        self.tip.hidden = true;
        self.render();
        if (hit) self._markTip(hit);
        return;
      }
      var g = self._geom;
      if (!g || p.x < g.L - 8 || p.x > g.L + g.pw + 8) { leave(); return; }
      self.hover = { kind: "x", x: g.X.invert(Math.min(Math.max(p.x, g.L), g.L + g.pw)), y: p.y };
      self.render();
    }
    function leave() {
      self.hover = null;
      self.tip.hidden = true;
      self.render();
    }
    cv.addEventListener("pointermove", move);
    cv.addEventListener("pointerdown", move);
    cv.addEventListener("pointerleave", leave);
    cv.addEventListener("blur", leave);
    cv.addEventListener("keydown", function (ev) {
      var k = self._kind();
      if (ev.key === "Escape") { leave(); return; }
      if (["ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown", "Home", "End"].indexOf(ev.key) < 0) return;
      ev.preventDefault();
      var dir = ev.key === "ArrowLeft" || ev.key === "ArrowUp" ? -1 : 1;
      if (k === "gantt" || k === "bar" || !self._geom || !self._geom.ser || !self._geom.ser.length) {
        var marks = self.hits;
        if (!marks.length) return;
        var cur = -1;
        for (var m = 0; self.hover && m < marks.length; m++) if (marks[m].item === self.hover.item) cur = m;
        var nxt = ev.key === "Home" ? 0 : ev.key === "End" ? marks.length - 1 : (cur + dir + marks.length) % marks.length;
        self.hover = { kind: "mark", item: marks[nxt].item };
        self.render();
        if (self.hits[nxt]) self._markTip(self.hits[nxt]);
        return;
      }
      var g = self._geom, xs = g.ser[0].x;
      var ix = self.hover && self.hover.kind === "x" ? nearestIndex(xs, self.hover.x) : -1;
      var stepN = Math.max(1, Math.round(xs.length / 40));
      ix = ev.key === "Home" ? 0 : ev.key === "End" ? xs.length - 1 : Math.min(xs.length - 1, Math.max(0, (ix < 0 ? 0 : ix + dir * stepN)));
      self.hover = { kind: "x", x: xs[ix], y: g.Tp + g.ph / 2 };
      self.render();
    });
    if (root.ResizeObserver) {
      var lastW = 0;
      this._ro = new root.ResizeObserver(function () {
        var w = self.area.clientWidth;
        if (w && Math.abs(w - lastW) >= 1) { lastW = w; self.render(); }
      });
      this._ro.observe(this.area);
    } else if (root.addEventListener) {
      root.addEventListener("resize", function () { self.render(); });
    }
  };

  onTheme(function () {
    for (var i = 0; i < live.length; i++) {
      live[i]._legend();
      live[i].render();
    }
  });

  function chart(container, spec) {
    if (!doc || !container) return null;
    return new Chart(container, spec);
  }

  var api = {
    chart: chart, loop: loop, setupCanvas: setupCanvas, roundRect: roundRect,
    theme: theme, color: color, alpha: alpha, cssVar: cssVar, onTheme: onTheme,
    reducedMotion: reducedMotion, fmt: fmt, fmtSI: fmtSI, niceTicks: niceTicks, logTicks: logTicks
  };
  if (doc) root.FabrailPlot = api;
  if (typeof module === "object" && module.exports) module.exports = api;
})(typeof window !== "undefined" ? window : globalThis);
