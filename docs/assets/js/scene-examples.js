/* examples.html (owner W4): the sixteen-example gallery (plan §4b; SPECIFICATION_EXAMPLES §1).
 *
 * The cards are static HTML (they read the same without JavaScript). This script adds:
 *   - schematic thumbnails drawn on canvas in the card's class colour (no data, no numbers);
 *   - the generated figure docs/images/examples/<name>.png in place of the drawing once the
 *     Wave-3 generators have run (probed only when the generated data files are no longer
 *     placeholders, or with ?images=1, so a fresh checkout logs no 404s);
 *   - the badge filter, and a copy button on every command.
 */
(function () {
  "use strict";
  var P = window.FabrailPlot, UI = window.FabrailUI;
  var doc = document;
  if (!P) return;

  var IMG_DIR = "images/examples/";

  function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }
  function line(ctx, x0, y0, x1, y1) { ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke(); }

  /* ---- thumbnails ------------------------------------------------------------------- */
  var DRAW = {
    drawer: function (c, w, h, k) {
      var x = w * 0.18, y = h * 0.25, cw = w * 0.3, ch = h * 0.55;
      c.fillStyle = P.alpha(k.line, 0.6); c.fillRect(x, y, cw, ch); c.strokeStyle = k.ink2; c.strokeRect(x, y, cw, ch);
      c.fillStyle = P.alpha(k.key, 0.25); c.strokeStyle = k.key; c.lineWidth = 2;
      c.fillRect(x + cw * 0.35, y + ch * 0.18, cw * 0.95, ch * 0.34); c.strokeRect(x + cw * 0.35, y + ch * 0.18, cw * 0.95, ch * 0.34);
      c.fillStyle = k.key; c.fillRect(x + cw * 1.3, y + ch * 0.12, 5, ch * 0.46);
      /* a log-damped approach on semilog axes */
      var gx = w * 0.64, gy = h * 0.2, gw = w * 0.3, gh = h * 0.6;
      c.strokeStyle = k.grid; c.lineWidth = 1; line(c, gx, gy + gh, gx + gw, gy + gh); line(c, gx, gy, gx, gy + gh);
      c.strokeStyle = k.key; c.lineWidth = 2; c.beginPath();
      for (var i = 0; i <= 40; i++) { var u = i / 40, d = u < 0.25 ? 1 - 0.3 * u * u * 8 : 0.85 - (u - 0.25) * 1.05; c.lineTo(gx + u * gw, gy + gh * (1 - clamp(d, 0, 1))); }
      c.stroke();
    },
    slider: function (c, w, h, k) {
      var x = w * 0.14, y = h * 0.22, cw = w * 0.72, ch = h * 0.56;
      c.fillStyle = P.alpha(k.line, 0.5); c.fillRect(x, y, cw, ch); c.strokeStyle = k.ink2; c.lineWidth = 1.2; c.strokeRect(x, y, cw, ch);
      c.fillStyle = P.alpha(k.key, 0.28); c.strokeStyle = k.key; c.lineWidth = 2;
      c.fillRect(x + cw * 0.12, y + 4, cw * 0.46, ch - 8); c.strokeRect(x + cw * 0.12, y + 4, cw * 0.46, ch - 8);
      c.strokeStyle = k.muted; c.lineWidth = 1.5;
      for (var i = 0; i < 5; i++) line(c, x + cw * (0.2 + i * 0.15), y + ch + 6, x + cw * (0.2 + i * 0.15), y + ch + 12);
    },
    door: function (c, w, h, k) {
      var gy = h * 0.86;
      c.strokeStyle = k.axis; c.lineWidth = 1.5; line(c, 0, gy, w, gy);
      c.fillStyle = k.steel; c.fillRect(w * 0.12, h * 0.14, w * 0.62, 5);
      c.fillStyle = P.alpha(k.key, 0.25); c.strokeStyle = k.key; c.lineWidth = 2;
      c.fillRect(w * 0.18, h * 0.2, w * 0.26, gy - h * 0.2); c.strokeRect(w * 0.18, h * 0.2, w * 0.26, gy - h * 0.2);
      c.strokeStyle = k.ink2; c.lineWidth = 2; c.beginPath(); c.arc(w * 0.56, h * 0.36, h * 0.08, 0, 2 * Math.PI); c.stroke();
      line(c, w * 0.56, h * 0.44, w * 0.56, h * 0.7); line(c, w * 0.56, h * 0.7, w * 0.52, gy); line(c, w * 0.56, h * 0.7, w * 0.6, gy);
      c.strokeStyle = k.ref; c.setLineDash([3, 3]); line(c, w * 0.47, h * 0.3, w * 0.47, gy - 4); c.setLineDash([]);
    },
    wall: function (c, w, h, k) {
      var gy = h * 0.86;
      c.strokeStyle = k.axis; c.lineWidth = 1.5; line(c, 0, gy, w, gy);
      c.fillStyle = k.steel; c.fillRect(w * 0.06, h * 0.12, w * 0.88, 5);
      for (var i = 0; i < 4; i++) {
        c.fillStyle = P.alpha(k.key, 0.2 + 0.05 * i); c.strokeStyle = k.key; c.lineWidth = 1.8;
        c.fillRect(w * (0.1 + i * 0.13), h * 0.18, w * 0.12, gy - h * 0.2); c.strokeRect(w * (0.1 + i * 0.13), h * 0.18, w * 0.12, gy - h * 0.2);
      }
      c.strokeStyle = k.ink2; c.lineWidth = 1.5; line(c, w * 0.72, h * 0.5, w * 0.88, h * 0.5);
      c.beginPath(); c.moveTo(w * 0.72, h * 0.5); c.lineTo(w * 0.75, h * 0.45); c.moveTo(w * 0.72, h * 0.5); c.lineTo(w * 0.75, h * 0.55); c.stroke();
    },
    heavy: function (c, w, h, k) {
      var gy = h * 0.84;
      c.strokeStyle = k.steel; c.lineWidth = 3; line(c, w * 0.05, gy + 4, w * 0.95, gy + 4);
      for (var i = 0; i < 4; i++) {
        var x = w * (0.12 + i * 0.2) + (i === 1 ? w * 0.05 : 0);
        c.fillStyle = P.alpha(k.key, 0.2); c.strokeStyle = k.key; c.lineWidth = 1.8;
        c.fillRect(x, h * 0.16, w * 0.14, gy - h * 0.16); c.strokeRect(x, h * 0.16, w * 0.14, gy - h * 0.16);
        c.strokeStyle = P.alpha(k.key, 0.6); c.lineWidth = 1;
        for (var j = 1; j < 4; j++) line(c, x + 2, h * 0.16 + (gy - h * 0.16) * j / 4, x + w * 0.14 - 2, h * 0.16 + (gy - h * 0.16) * j / 4);
      }
      c.fillStyle = k.ink2; c.beginPath(); c.arc(w * 0.9, h * 0.3, 7, 0, 2 * Math.PI); c.fill();
    },
    power: function (c, w, h, k) {
      DRAW.drawer(c, w * 0.62, h, k);
      c.fillStyle = k.ref; c.beginPath();
      var x = w * 0.78, y = h * 0.16;
      c.moveTo(x, y); c.lineTo(x - 16, y + 30); c.lineTo(x - 4, y + 30); c.lineTo(x - 12, y + 56); c.lineTo(x + 12, y + 20); c.lineTo(x, y + 20); c.closePath(); c.fill();
    },
    pwave: function (c, w, h, k) {
      var gy = h * 0.5;
      c.fillStyle = P.alpha(k.line, 0.3); c.fillRect(0, gy, w, h - gy);
      c.strokeStyle = k.axis; line(c, 0, gy, w, gy);
      for (var i = 0; i < 4; i++) { c.strokeStyle = P.alpha(k.p, 0.9 - i * 0.2); c.lineWidth = i ? 1 : 2; line(c, w * 0.55 - i * 7, gy + 2, w * 0.55 - i * 7, h); }
      c.strokeStyle = k.s; c.lineWidth = 2; c.beginPath();
      for (var y = gy + 2; y < h; y += 3) c.lineTo(w * 0.25 + 5 * Math.sin(y / 5), y);
      c.stroke();
      c.fillStyle = P.alpha(k.panel, 0.95); c.strokeStyle = k.ink2; c.lineWidth = 1.5;
      c.fillRect(w * 0.7, gy - h * 0.3, w * 0.2, h * 0.3); c.strokeRect(w * 0.7, gy - h * 0.3, w * 0.2, h * 0.3);
      c.fillStyle = k.key; c.fillRect(w * 0.74, gy - h * 0.24, w * 0.04, h * 0.24);
    },
    villa: function (c, w, h, k) {
      var gy = h * 0.86;
      c.strokeStyle = k.axis; c.lineWidth = 1.5; line(c, 0, gy, w, gy);
      c.fillStyle = P.alpha(k.panel, 0.95); c.strokeStyle = k.ink2;
      c.fillRect(w * 0.12, h * 0.2, w * 0.76, gy - h * 0.2); c.strokeRect(w * 0.12, h * 0.2, w * 0.76, gy - h * 0.2);
      c.fillStyle = k.steel; c.fillRect(w * 0.09, h * 0.14, w * 0.82, 6); c.fillRect(w * 0.12, h * 0.52, w * 0.76, 3);
      var cols = [k.drawer, k.drawer, k.door, k.wall];
      for (var i = 0; i < 3; i++) { c.fillStyle = P.alpha(cols[0], 0.3); c.strokeStyle = cols[0]; c.fillRect(w * 0.16, h * 0.6 + i * h * 0.07, w * 0.1, h * 0.06); c.strokeRect(w * 0.16, h * 0.6 + i * h * 0.07, w * 0.1, h * 0.06); }
      c.fillStyle = P.alpha(cols[2], 0.3); c.strokeStyle = cols[2]; c.fillRect(w * 0.44, h * 0.58, w * 0.08, gy - h * 0.6); c.strokeRect(w * 0.44, h * 0.58, w * 0.08, gy - h * 0.6);
      c.fillStyle = P.alpha(cols[3], 0.3); c.strokeStyle = cols[3]; c.fillRect(w * 0.7, h * 0.58, w * 0.06, gy - h * 0.6); c.strokeRect(w * 0.7, h * 0.58, w * 0.06, gy - h * 0.6);
      c.fillStyle = k.exotic; c.fillRect(w * 0.62, h * 0.44, w * 0.12, 4);
      c.strokeStyle = k.s; c.lineWidth = 1.8; c.beginPath();
      for (var x = 0; x < w * 0.1; x += 2) c.lineTo(x, gy + 6 + 3 * Math.sin(x / 3));
      c.stroke();
    },
    tower: function (c, w, h, k) {
      var gy = h * 0.92;
      c.strokeStyle = k.axis; line(c, 0, gy, w, gy);
      c.fillStyle = P.alpha(k.panel, 0.95); c.strokeStyle = k.ink2; c.lineWidth = 1.5;
      c.beginPath(); c.moveTo(w * 0.44, gy); c.quadraticCurveTo(w * 0.45, h * 0.4, w * 0.49, h * 0.06); c.lineTo(w * 0.55, h * 0.06); c.quadraticCurveTo(w * 0.57, h * 0.4, w * 0.58, gy); c.closePath(); c.fill(); c.stroke();
      c.fillStyle = k.key; c.fillRect(w * 0.495, h * 0.14, w * 0.04, h * 0.06);
      c.strokeStyle = P.alpha(k.flux, 0.7); c.lineWidth = 1.5;
      for (var i = 0; i < 4; i++) { var y = h * (0.25 + i * 0.15); line(c, w * 0.12, y, w * 0.3, y); c.beginPath(); c.moveTo(w * 0.3, y); c.lineTo(w * 0.28, y - 3); c.lineTo(w * 0.28, y + 3); c.closePath(); c.fillStyle = c.strokeStyle; c.fill(); }
    },
    plinth: function (c, w, h, k) {
      var fy = h * 0.66;
      c.fillStyle = P.alpha(k.line, 0.35); c.fillRect(0, fy, w, h - fy);
      c.strokeStyle = k.ink2; c.lineWidth = 1.5; c.beginPath(); c.arc(w / 2, fy - w * 0.6, w * 0.72, 0.36 * Math.PI, 0.64 * Math.PI); c.stroke();
      c.fillStyle = P.alpha(k.key, 0.3); c.strokeStyle = k.key; c.fillRect(w * 0.34, fy + 2, w * 0.32, 6); c.strokeRect(w * 0.34, fy + 2, w * 0.32, 6);
      c.fillStyle = P.alpha(k.line, 0.8); c.fillRect(w * 0.46, fy - h * 0.34, w * 0.08, h * 0.36);
      c.fillStyle = k.ink2; c.beginPath(); c.ellipse(w / 2, fy - h * 0.43, 7, 11, 0, 0, 2 * Math.PI); c.fill();
      c.fillStyle = k.steel; c.fillRect(w * 0.2, fy - 3, 20, 3); c.fillRect(w * 0.8 - 20, fy - 3, 20, 3);
    },
    shield: function (c, w, h, k) {
      var gy = h * 0.88;
      c.fillStyle = P.alpha(k.line, 0.5); c.fillRect(w * 0.08, h * 0.1, w * 0.18, gy - h * 0.1); c.fillRect(w * 0.64, h * 0.1, w * 0.28, gy - h * 0.1);
      c.fillStyle = P.alpha(k.key, 0.3); c.strokeStyle = k.key; c.lineWidth = 2; c.fillRect(w * 0.3, h * 0.16, w * 0.3, gy - h * 0.18); c.strokeRect(w * 0.3, h * 0.16, w * 0.3, gy - h * 0.18);
      c.strokeStyle = k.ref; c.setLineDash([6, 4]); line(c, 0, h * 0.5, w * 0.28, h * 0.5); c.setLineDash([]);
      c.strokeStyle = k.p; c.lineWidth = 1.5; for (var i = 0; i < 3; i++) line(c, w * (0.72 + i * 0.05), gy, w * (0.72 + i * 0.05), h * 0.7);
    },
    vault: function (c, w, h, k) {
      var gy = h * 0.86;
      c.fillStyle = k.steel; c.fillRect(w * 0.3, h * 0.14, 8, gy - h * 0.14); c.fillRect(w * 0.62, h * 0.14, 8, gy - h * 0.14); c.fillRect(w * 0.3, h * 0.1, w * 0.32 + 8, 6);
      c.fillStyle = P.alpha(k.key, 0.3); c.strokeStyle = k.key; c.lineWidth = 2; c.fillRect(w * 0.14, h * 0.18, w * 0.3, gy - h * 0.2); c.strokeRect(w * 0.14, h * 0.18, w * 0.3, gy - h * 0.2);
      for (var i = 0; i < 3; i++) { c.strokeStyle = P.alpha(k.s, 0.9 - 0.25 * i); c.lineWidth = i ? 1 : 2.5; line(c, w * (0.8 + i * 0.05), h * 0.08, w * (0.8 + i * 0.05), gy); }
      c.strokeStyle = k.ink2; c.lineWidth = 1.5; line(c, w * 0.46, h * 0.5, w * 0.58, h * 0.5);
    },
    tray: function (c, w, h, k) {
      var nx = 16, ny = 7, x0 = w * 0.1, y0 = h * 0.14, cw = w * 0.8 / nx, ch = h * 0.72 / ny;
      for (var j = 0; j < ny; j++) for (var i = 0; i < nx; i++) {
        var failed = i === 4 && j === 2;
        c.fillStyle = failed ? P.alpha(k.muted, 0.3) : P.alpha(k.key, 0.2 + 0.25 * Math.max(0, 1 - Math.hypot(i - 4, j - 2) / 8));
        c.fillRect(x0 + i * cw + 1, y0 + j * ch + 1, cw - 2, ch - 2);
        if (failed) { c.strokeStyle = k.ref; c.lineWidth = 1.5; line(c, x0 + i * cw + 3, y0 + j * ch + 3, x0 + (i + 1) * cw - 3, y0 + (j + 1) * ch - 3); line(c, x0 + (i + 1) * cw - 3, y0 + j * ch + 3, x0 + i * cw + 3, y0 + (j + 1) * ch - 3); }
      }
    },
    gate: function (c, w, h, k) {
      var wl = h * 0.55;
      c.fillStyle = P.alpha(k.flux, 0.18); c.fillRect(0, wl, w * 0.62, h - wl);
      c.strokeStyle = k.flux; c.lineWidth = 2; c.beginPath();
      for (var x = 0; x <= w * 0.62; x += 3) c.lineTo(x, wl - (x > w * 0.25 && x < w * 0.45 ? 10 * Math.sin((x - w * 0.25) / (w * 0.2) * Math.PI) : 0));
      c.stroke();
      c.fillStyle = P.alpha(k.key, 0.3); c.strokeStyle = k.key; c.fillRect(w * 0.62, h * 0.2, w * 0.06, h * 0.7); c.strokeRect(w * 0.62, h * 0.2, w * 0.06, h * 0.7);
      c.fillStyle = P.alpha(k.line, 0.5); c.fillRect(w * 0.68, h * 0.45, w * 0.32, h * 0.55);
    },
    pegs: function (c, w, h, k) {
      var cx = w * 0.2, cy = h * 0.72;
      c.strokeStyle = P.alpha(k.key, 0.8); c.lineWidth = 1.2;
      for (var i = 1; i <= 6; i++) { c.globalAlpha = 1 - i * 0.13; c.beginPath(); c.arc(cx, cy, i * w * 0.12, -Math.PI * 0.5, 0); c.stroke(); }
      c.globalAlpha = 1;
      c.strokeStyle = k.p; c.lineWidth = 2; c.beginPath(); c.arc(cx, cy, w * 0.16, -Math.PI * 0.5, 0); c.stroke();
      c.fillStyle = k.s; c.beginPath(); c.arc(cx, cy, 5, 0, 2 * Math.PI); c.fill();
      c.fillStyle = k.ink2; c.fillRect(w * 0.8, h * 0.42, 10, 16);
    },
    moon: function (c, w, h, k) {
      c.fillStyle = P.alpha(k.line, 0.7); c.beginPath(); c.arc(w * 0.72, h * 0.34, h * 0.22, 0, 2 * Math.PI); c.fill();
      c.fillStyle = k.deep; c.beginPath(); c.arc(w * 0.78, h * 0.3, h * 0.2, 0, 2 * Math.PI); c.fill();
      var gy = h * 0.8;
      c.fillStyle = P.alpha(k.line, 0.45); c.beginPath(); c.moveTo(0, gy); c.quadraticCurveTo(w * 0.3, gy - h * 0.25, w * 0.55, gy); c.lineTo(w, gy); c.lineTo(w, h); c.lineTo(0, h); c.closePath(); c.fill();
      c.fillStyle = P.alpha(k.key, 0.35); c.strokeStyle = k.key; c.lineWidth = 1.8; c.fillRect(w * 0.18, gy - h * 0.14, w * 0.18, h * 0.08); c.strokeRect(w * 0.18, gy - h * 0.14, w * 0.18, h * 0.08);
      c.fillStyle = P.alpha(k.flux, 0.35); c.fillRect(w * 0.18, gy - h * 0.06, w * 0.18, 3);
    }
  };

  function tileColor(card) {
    var v = "";
    try { v = getComputedStyle(card).getPropertyValue("--tile").trim(); } catch (e) { v = ""; }
    return v || P.color("accent");
  }

  var thumbs = [];
  function colours() {
    var k = {};
    ["deep", "panel", "ink", "ink2", "muted", "line", "grid", "axis", "accent", "ref", "p", "s", "drawer", "door", "wall", "exotic", "frontier"].forEach(function (n) { k[n] = P.color(n); });
    k.flux = P.color("--flux"); k.steel = P.color("--steel");
    return k;
  }
  function drawThumb(t, k) {
    if (t.img) return;
    var w = Math.max(160, Math.round(t.box.clientWidth || 320)), h = Math.max(70, Math.round(t.box.clientHeight || w * 7 / 16));
    var cv = P.setupCanvas(t.canvas, w, h), ctx = cv.ctx;
    ctx.fillStyle = k.deep; ctx.fillRect(0, 0, w, h);
    k.key = tileColor(t.card);
    var fn = DRAW[t.kind];
    ctx.save();
    if (fn) fn(ctx, w, h, k);
    ctx.restore();
  }
  function drawAll() { var k = colours(); for (var i = 0; i < thumbs.length; i++) drawThumb(thumbs[i], k); }

  function generatedFiguresExpected() {
    try { if (/[?&]images=1\b/.test(window.location.search)) return true; } catch (e) { /* no location */ }
    var A = window.ANCHORS;
    return !!(A && A.placeholder === false);
  }
  function probeImage(t) {
    var name = t.box.getAttribute("data-example");
    if (!name) return;
    var img = new Image();
    img.alt = "Figure written by python -m fabrail example run " + name;
    img.decoding = "async";
    img.onload = function () {
      t.img = img;
      if (t.canvas.parentNode) t.canvas.parentNode.removeChild(t.canvas);
      t.box.appendChild(img);
      t.box.removeAttribute("aria-hidden");
    };
    img.onerror = function () { /* keep the schematic drawing */ };
    img.src = IMG_DIR + name + ".png";
  }

  function initThumbs() {
    var boxes = doc.querySelectorAll(".ex-thumb[data-thumb]");
    for (var i = 0; i < boxes.length; i++) {
      var box = boxes[i], canvas = doc.createElement("canvas");
      canvas.setAttribute("aria-hidden", "true");
      box.setAttribute("aria-hidden", "true");
      box.appendChild(canvas);
      thumbs.push({ box: box, canvas: canvas, kind: box.getAttribute("data-thumb"), card: box.closest ? box.closest(".ex-card") : box.parentNode, img: null });
    }
    drawAll();
    if (generatedFiguresExpected()) for (var j = 0; j < thumbs.length; j++) probeImage(thumbs[j]);
    P.onTheme(drawAll);
    if (window.ResizeObserver) {
      var lastW = 0, g = doc.getElementById("gallery");
      if (g) new ResizeObserver(function () { var w = g.clientWidth; if (Math.abs(w - lastW) >= 1) { lastW = w; drawAll(); } }).observe(g);
    } else window.addEventListener("resize", drawAll);
  }

  /* ---- filter ------------------------------------------------------------------------- */
  function initFilter() {
    var group = doc.getElementById("ex-filter"), count = doc.getElementById("ex-count");
    if (!group || !UI) return;
    var cards = doc.querySelectorAll(".ex-card");
    function apply(v) {
      var n = 0;
      for (var i = 0; i < cards.length; i++) {
        var show = v === "all" || cards[i].getAttribute("data-badge") === v;
        cards[i].hidden = !show;
        if (show) n++;
      }
      if (count) count.textContent = n + (n === 1 ? " example" : " examples");
    }
    UI.segmented(group, apply);
    apply("all");
    /* a #name link opens the gallery unfiltered so the target card is visible */
    window.addEventListener("hashchange", function () { apply("all"); var s = group.querySelector('[data-value="all"]'); if (s) s.click(); });
  }

  /* ---- copy buttons ------------------------------------------------------------------ */
  function copyText(text, done) {
    function fallback() {
      var ta = doc.createElement("textarea");
      ta.value = text; ta.setAttribute("readonly", ""); ta.style.position = "absolute"; ta.style.left = "-9999px";
      doc.body.appendChild(ta); ta.select();
      var ok = false;
      try { ok = doc.execCommand("copy"); } catch (e) { ok = false; }
      doc.body.removeChild(ta);
      done(ok);
    }
    try {
      if (navigator.clipboard && navigator.clipboard.writeText) navigator.clipboard.writeText(text).then(function () { done(true); }, fallback);
      else fallback();
    } catch (e) { fallback(); }
  }
  function initCopy() {
    var btns = doc.querySelectorAll("[data-copy]");
    for (var i = 0; i < btns.length; i++) {
      (function (b) {
        var code = b.parentNode.querySelector("code");
        if (!code) return;
        b.setAttribute("aria-label", "Copy the command " + code.textContent);
        b.addEventListener("click", function () {
          copyText(code.textContent, function (ok) {
            b.textContent = ok ? "Copied" : "Select and copy";
            setTimeout(function () { b.textContent = "Copy"; }, 1600);
          });
        });
      })(btns[i]);
    }
  }

  function init() { initThumbs(); initFilter(); initCopy(); }
  if (doc.readyState === "loading") doc.addEventListener("DOMContentLoaded", init); else init();
})();
