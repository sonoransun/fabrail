/* fabrail browser engine loader (plan §3b). Owner: A13 (loader, fallback); the wasm ABI
 * is W1's (crates/fabrail-web/src/lib.rs, whose tests read this file).
 *
 * The live engine is the real controller and plant: crates/fabrail-web compiled for
 * wasm32-unknown-unknown with hand-written extern "C" exports (no wasm-bindgen). This
 * loader instantiates it without a build step and without external URLs. When the wasm
 * is unavailable (not built yet, or blocked on file://), pages fall back to the closed
 * forms of assets/js/formulas.js (the FORMULAS block, which node checks against the Rust
 * anchors, FAB-SW-013). Nothing here ever throws into a page: every call degrades.
 *
 * Frozen JS API (W0), on window.FabrailEngine and as module.exports under node:
 *   await load(opts)     -> { mode: "wasm" | "formulas", reason }   never rejects
 *   mode()               -> "unloaded" | "wasm" | "formulas"
 *   selftestHash()       -> BigInt | null   (u64 computed in this browser)
 *   selftestHex()        -> "0x…" | null
 *   verifySelftest()     -> { state: "ok" | "mismatch" | "unavailable", computed, golden,
 *                             pending, reason }   pending = computed, golden not frozen yet
 *   create(preset[, tier]) -> boolean       preset in PRESETS, tier in TIERS (ABI v2)
 *   setInputs({ handN, floorAx, floorAy, floorAz, command })   command: key of COMMANDS
 *   step(nTicks)         -> ticks stepped (0 in formulas mode)
 *   state()              -> { <STATE_FIELDS[i]>: number } | null
 *   renderSelftestBadges()
 *   PRESETS, COMMANDS, STATE_FIELDS, WASM_EXPORTS, WASM_URL, FORMULAS
 * Additive (A13; each feature-detects its export and returns null/false without it):
 *   abiVersion(), status(), tier(), goTo(xM), lastError(), anchor(id), anchorName(id),
 *   derivedAnchors(preset), scenarioJson(preset[, tier]), open(preset[, tier]) -> handle
 *   { handle, setInputs, step, state, status, goTo, destroy } (several engines per page),
 *   formulas(), TIERS, STATUS.
 * load(opts): opts.bytes (ArrayBuffer | Uint8Array, e.g. under node), opts.url,
 *   opts.force (fetch even before ANCHORS announces a built wasm).
 * Event: document dispatches "fabrail:engine" with detail { mode, reason } after a load.
 *
 * wasm sources, in order: opts.bytes; window.FABRAIL_WASM_B64 (base64 from an optional
 * assets/wasm/fabrail_web.wasm.js, which works on file://; on file:// the loader injects
 * that script once ANCHORS.wasm announces a build); then fetch(opts.url || WASM_URL). A
 * fetch is attempted only when opts.force is set or ANCHORS.wasm.sha256 is present
 * (written when the wasm is built), so pages log no 404 before Wave 3.
 */
(function (root) {
  "use strict";

  var WASM_URL = "assets/wasm/fabrail_web.wasm";

  /* Preset ids passed to fabrail_create (order frozen). */
  var PRESETS = ["drawer", "door", "wall", "heavy"];

  /* Plant tiers passed to fabrail_create_tier (ABI v2; = fabrail_web::TIERS order). */
  var TIERS = ["carriage6", "heave1", "travel1d"];

  /* Command codes passed to fabrail_set_inputs (mirror types::cmd::UserCommand; frozen,
     append-only: manual is ABI v2). */
  var COMMANDS = {
    none: 0, open: 1, close: 2, stop: 3, lift: 4, land: 5, reset: 6,
    hold_to_run_on: 7, hold_to_run_off: 8, manual: 9
  };

  /* fabrail_status / fr_status codes (ABI v2). */
  var STATUS = ["running", "no_engine", "failed", "ended"];

  /* State buffer layout: fabrail_state_ptr() points at fabrail_state_len() little-endian
     f64 values in this order (frozen; W1 implements it, append-only). */
  var STATE_FIELDS = [
    "tick", "t_s", "x_m", "v_mps", "a_mps2", "gap_m", "coil_i_a", "iq_a",
    "f_lsm_n", "f_hand_n", "f_hand_hat_n", "m_hat_kg", "e_kin_gov_j",
    "lev_state", "motion_mode", "seismic_state", "seismic_level", "safety_demand",
    "fault_bits_lo", "floor_ax_mps2"
  ];

  /* Exports the wasm module must provide (frozen names; signatures in wasm types). */
  var WASM_EXPORTS = {
    memory: "Memory",
    fabrail_abi_version: "() -> u32",
    fabrail_create: "(preset: u32) -> u32  (0 = ok, else error code)",
    fabrail_set_inputs: "(hand_n: f64, floor_ax: f64, floor_ay: f64, floor_az: f64, command: u32)",
    fabrail_step: "(n_ticks: u32) -> u32  (ticks stepped)",
    fabrail_state_ptr: "() -> u32  (byte offset of the f64 state buffer)",
    fabrail_state_len: "() -> u32  (number of f64 values)",
    fabrail_selftest_hash: "() -> u64  (BigInt in JS)",
    fabrail_anchor: "(id: u32) -> f64  (NaN for an unknown id)"
  };

  /* Anchor id stride per preset: id = 64·preset + k (fabrail_web::ANCHOR_STRIDE). */
  var ANCHOR_STRIDE = 64;

  var doc = root.document || null;

  /* Directory of this script, so assets resolve from pages in subdirectories (read/). */
  var scriptBase = (function () {
    try {
      var s = doc && doc.currentScript && doc.currentScript.src;
      return s ? s.slice(0, s.lastIndexOf("/") + 1) : "";
    } catch (e) {
      return "";
    }
  })();

  function assetUrl(rel) {
    if (!scriptBase) return rel;
    try {
      return new URL(rel, scriptBase).href;
    } catch (e) {
      return rel;
    }
  }

  var defaultWasmUrl = scriptBase ? assetUrl("../wasm/fabrail_web.wasm") : WASM_URL;

  /* ---- FORMULAS (formulas.js) -------------------------------------------------------- */

  function formulas() {
    if (root.FabrailFormulas) return root.FabrailFormulas;
    if (!doc && typeof require === "function") {
      try {
        return require("./formulas.js");
      } catch (e) {
        return null;
      }
    }
    return null;
  }

  function injectScript(src) {
    return new Promise(function (resolve) {
      if (!doc || !doc.createElement) return resolve(false);
      var el = doc.createElement("script");
      el.src = src;
      el.async = true;
      el.onload = function () { resolve(true); };
      el.onerror = function () { resolve(false); };
      (doc.head || doc.documentElement).appendChild(el);
    });
  }

  /* Pages normally include formulas.js themselves; if one does not, fetch it next to
     this script so the fallback always has the closed forms. */
  function ensureFormulas() {
    if (formulas()) return Promise.resolve(true);
    if (!doc) return Promise.resolve(false);
    return injectScript(assetUrl("formulas.js")).then(function () { return !!formulas(); });
  }

  /* ---- loading -------------------------------------------------------------------- */

  var st = { mode: "unloaded", reason: "", exports: null, memory: null, pending: null };

  function anchors() {
    return root.ANCHORS && !root.ANCHORS.placeholder ? root.ANCHORS : null;
  }

  function wasmAnnounced() {
    var a = anchors();
    return !!(a && a.wasm && a.wasm.sha256);
  }

  function b64ToBytes(b64) {
    var bin = root.atob ? root.atob(b64) : Buffer.from(b64, "base64").toString("binary");
    var out = new Uint8Array(bin.length);
    for (var i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
    return out;
  }

  function fallback(reason) {
    st.mode = "formulas";
    st.reason = reason;
    st.exports = null;
    st.memory = null;
    return { mode: st.mode, reason: reason };
  }

  function announce(result) {
    try {
      if (doc && typeof CustomEvent === "function") {
        doc.dispatchEvent(new CustomEvent("fabrail:engine", { detail: result }));
      }
    } catch (e) {
      /* no listeners possible */
    }
    return result;
  }

  async function wasmBytes(opts) {
    if (opts.bytes) {
      return opts.bytes instanceof Uint8Array ? opts.bytes : new Uint8Array(opts.bytes);
    }
    if (typeof root.FABRAIL_WASM_B64 === "string" && root.FABRAIL_WASM_B64.length > 0) {
      return b64ToBytes(root.FABRAIL_WASM_B64);
    }
    var url = opts.url || defaultWasmUrl;
    if (!opts.force && !wasmAnnounced()) return "browser engine not built yet";
    var onFile = !!(root.location && root.location.protocol === "file:");
    if (onFile && doc) {
      /* fetch() is blocked on file://; the optional base64 companion script is not. */
      var ok = await injectScript(url + ".js");
      if (ok && typeof root.FABRAIL_WASM_B64 === "string" && root.FABRAIL_WASM_B64.length > 0) {
        return b64ToBytes(root.FABRAIL_WASM_B64);
      }
      return "wasm not loadable from file:// (serve the docs/ directory over http)";
    }
    if (typeof fetch !== "function") return "fetch unavailable";
    var resp = await fetch(url);
    if (!resp.ok) return "wasm fetch failed: HTTP " + resp.status;
    return new Uint8Array(await resp.arrayBuffer());
  }

  async function loadOnce(opts) {
    await ensureFormulas();
    if (typeof WebAssembly !== "object") return fallback("WebAssembly unavailable");
    try {
      var bytes = await wasmBytes(opts);
      if (typeof bytes === "string") return fallback(bytes);
      var inst = await WebAssembly.instantiate(bytes, {});
      var ex = inst.instance.exports;
      for (var name in WASM_EXPORTS) {
        if (!(name in ex)) return fallback("wasm export missing: " + name);
      }
      st.exports = ex;
      st.memory = ex.memory;
      st.mode = "wasm";
      st.reason = "";
      return { mode: "wasm", reason: "" };
    } catch (e) {
      return fallback("wasm unavailable: " + (e && e.message ? e.message : String(e)));
    }
  }

  /* Memoised: pages and the footer badge share one instance unless opts ask otherwise. */
  function load(opts) {
    opts = opts || {};
    var fresh = !!(opts.bytes || opts.url || opts.force);
    if (st.pending && !fresh) return st.pending;
    st.pending = loadOnce(opts).then(announce, function (e) {
      return announce(fallback("loader error: " + (e && e.message ? e.message : String(e))));
    });
    return st.pending;
  }

  /* ---- selftest ----------------------------------------------------------------------- */

  function selftestHash() {
    if (st.mode !== "wasm") return null;
    try {
      return BigInt.asUintN(64, BigInt(st.exports.fabrail_selftest_hash()));
    } catch (e) {
      return null;
    }
  }

  function hex(v) {
    return v === null || v === undefined ? null : "0x" + v.toString(16).padStart(16, "0");
  }

  function selftestHex() { return hex(selftestHash()); }

  /* The frozen golden value: ANCHORS.golden_selftest_hash (C1/C2a), else RUN's copy. */
  function goldenHex() {
    var a = anchors();
    var g = a && a.golden_selftest_hash;
    if (!g && root.RUN && !root.RUN.placeholder) g = root.RUN.golden_selftest_hash;
    if (g === null || g === undefined || g === "") return null;
    if (typeof g === "number" || typeof g === "bigint") g = hex(BigInt(g));
    return String(g).toLowerCase();
  }

  function isUnfrozen(g) {
    return !g || /^0x0*$/.test(g);
  }

  function verifySelftest() {
    var computed = selftestHex();
    var golden = goldenHex();
    if (!computed) {
      return { state: "unavailable", computed: null, golden: golden, pending: false,
        reason: st.reason || "browser engine not loaded" };
    }
    if (isUnfrozen(golden)) {
      return { state: "unavailable", computed: computed, golden: golden, pending: true,
        reason: "golden hash pending freeze" };
    }
    return { state: computed === golden ? "ok" : "mismatch", computed: computed,
      golden: golden, pending: false, reason: "" };
  }

  /* ---- page engine (singleton layer) ------------------------------------------------ */

  function has(name) {
    return st.mode === "wasm" && typeof st.exports[name] === "function";
  }

  function abiVersion() {
    return has("fabrail_abi_version") ? st.exports.fabrail_abi_version() >>> 0 : null;
  }

  function create(preset, tier) {
    var id = PRESETS.indexOf(preset);
    if (st.mode !== "wasm" || id < 0) return false;
    if (tier !== undefined && tier !== null) {
      var t = TIERS.indexOf(tier);
      if (t < 0 || !has("fabrail_create_tier")) return false;
      return st.exports.fabrail_create_tier(id, t) === 0;
    }
    return st.exports.fabrail_create(id) === 0;
  }

  function commandCode(c) {
    if (typeof c === "number") return c >>> 0;
    var code = COMMANDS[c || "none"];
    return code === undefined ? 0 : code;
  }

  function num(x) {
    x = +x;
    return isFinite(x) ? x : 0;
  }

  function setInputs(inp) {
    if (st.mode !== "wasm") return;
    inp = inp || {};
    st.exports.fabrail_set_inputs(num(inp.handN), num(inp.floorAx), num(inp.floorAy),
      num(inp.floorAz), commandCode(inp.command));
  }

  function step(n) {
    if (st.mode !== "wasm") return 0;
    return st.exports.fabrail_step(Math.max(0, Math.floor(n) || 0)) >>> 0;
  }

  function readState(ptr) {
    var len = st.exports.fabrail_state_len() >>> 0;
    if (!ptr) return null;
    var view = new Float64Array(st.memory.buffer, ptr >>> 0, len);
    var out = {};
    for (var i = 0; i < STATE_FIELDS.length && i < len; i++) out[STATE_FIELDS[i]] = view[i];
    return out;
  }

  function state() {
    if (st.mode !== "wasm") return null;
    return readState(st.exports.fabrail_state_ptr() >>> 0);
  }

  function statusName(code) {
    return STATUS[code >>> 0] || "unknown";
  }

  function status() {
    return has("fabrail_status") ? statusName(st.exports.fabrail_status()) : null;
  }

  function tier() {
    if (!has("fabrail_tier")) return null;
    return TIERS[st.exports.fabrail_tier() >>> 0] || null;
  }

  function goTo(xM) {
    return has("fabrail_go_to") ? st.exports.fabrail_go_to(num(xM)) === 1 : false;
  }

  /* ---- text buffer, anchors (ABI v2) ------------------------------------------------ */

  function readText(len) {
    len = len >>> 0;
    if (!len || !has("fr_text_ptr")) return null;
    var bytes = new Uint8Array(st.memory.buffer, st.exports.fr_text_ptr() >>> 0, len);
    try {
      return new TextDecoder("utf-8").decode(bytes.slice());
    } catch (e) {
      var s = "";
      for (var i = 0; i < bytes.length; i++) s += String.fromCharCode(bytes[i]);
      return s;
    }
  }

  function lastError() {
    return has("fr_last_error") ? readText(st.exports.fr_last_error()) : null;
  }

  function scenarioJson(preset, tierName) {
    var p = PRESETS.indexOf(preset);
    var t = tierName === undefined ? 0 : TIERS.indexOf(tierName);
    if (p < 0 || t < 0 || !has("fr_scenario_json")) return null;
    return readText(st.exports.fr_scenario_json(p, t));
  }

  function anchor(id) {
    return st.mode === "wasm" ? st.exports.fabrail_anchor(id >>> 0) : NaN;
  }

  function anchorName(id) {
    return has("fr_anchor_name") ? readText(st.exports.fr_anchor_name(id >>> 0)) : null;
  }

  /* Params.derived() of a preset as computed in the wasm engine (needs fr_anchor_name). */
  function derivedAnchors(preset) {
    var p = PRESETS.indexOf(preset);
    if (p < 0 || !has("fr_anchor_name")) return null;
    var out = {};
    for (var k = 0; k < ANCHOR_STRIDE; k++) {
      var id = ANCHOR_STRIDE * p + k;
      var name = anchorName(id);
      if (!name) break;
      out[name] = anchor(id);
    }
    return out;
  }

  /* ---- handle layer: several engines per page --------------------------------------- */

  function open(preset, tierName) {
    var p = PRESETS.indexOf(preset);
    if (p < 0 || !has("fr_create") || !has("fr_state_ptr")) return null;
    var h;
    if (tierName !== undefined && tierName !== null) {
      var t = TIERS.indexOf(tierName);
      if (t < 0 || !has("fr_create_tier")) return null;
      h = st.exports.fr_create_tier(p, t) >>> 0;
    } else {
      h = st.exports.fr_create(p) >>> 0;
    }
    if (!h) return null;
    var ex = st.exports;
    var live = true;
    return {
      handle: h,
      preset: preset,
      setInputs: function (inp) {
        if (!live) return false;
        inp = inp || {};
        return ex.fr_set_inputs(h, num(inp.handN), num(inp.floorAx), num(inp.floorAy),
          num(inp.floorAz), commandCode(inp.command)) === 1;
      },
      step: function (n) {
        return live ? ex.fr_step(h, Math.max(0, Math.floor(n) || 0)) >>> 0 : 0;
      },
      state: function () {
        return live ? readState(ex.fr_state_ptr(h) >>> 0) : null;
      },
      status: function () {
        return live && typeof ex.fr_status === "function" ? statusName(ex.fr_status(h)) : null;
      },
      goTo: function (xM) {
        return live && typeof ex.fr_go_to === "function" ? ex.fr_go_to(h, num(xM)) === 1 : false;
      },
      destroy: function () {
        if (!live) return false;
        live = false;
        return typeof ex.fr_destroy === "function" ? ex.fr_destroy(h) === 1 : false;
      }
    };
  }

  /* ---- badges -------------------------------------------------------------------------- */

  function short(h) {
    return h ? h.slice(0, 10) + "…" : "";
  }

  /* Footer badges <span data-selftest> on every page, plus optional detail fields
     [data-selftest-field="text|state|computed|golden|mode|reason"]. */
  function renderSelftestBadges() {
    if (!doc) return;
    var v = verifySelftest();
    var domState = v.pending ? "pending" : v.state;
    var text;
    if (v.state === "ok") {
      text = "selftest hash computed in your browser = firmware GOLDEN_SELFTEST_HASH ✓";
    } else if (v.state === "mismatch") {
      text = "selftest hash mismatch: browser " + v.computed + " vs golden " + v.golden;
    } else if (v.pending) {
      text = "selftest hash computed in your browser: " + short(v.computed) +
        " (golden value pending freeze)";
    } else {
      text = "selftest: " + (st.reason || "browser engine unavailable") + " (closed-form fallback)";
    }
    var els = doc.querySelectorAll("[data-selftest]");
    for (var i = 0; i < els.length; i++) {
      els[i].setAttribute("data-state", domState);
      els[i].textContent = text;
      if (v.computed) els[i].setAttribute("title", "computed " + v.computed + (v.golden ? " · golden " + v.golden : ""));
    }
    var fields = {
      text: text,
      state: domState,
      computed: v.computed || "not computed",
      golden: isUnfrozen(v.golden) ? "pending freeze" : v.golden,
      mode: st.mode === "wasm" ? "WebAssembly engine" : st.mode === "formulas" ? "closed-form fallback" : "loading",
      reason: v.reason || ""
    };
    var fs = doc.querySelectorAll("[data-selftest-field]");
    for (var j = 0; j < fs.length; j++) {
      var key = fs[j].getAttribute("data-selftest-field");
      if (key in fields) fs[j].textContent = fields[key];
      fs[j].setAttribute("data-state", domState);
    }
  }

  var api = {
    load: load, mode: function () { return st.mode; }, reason: function () { return st.reason; },
    selftestHash: selftestHash, selftestHex: selftestHex, verifySelftest: verifySelftest,
    create: create, setInputs: setInputs, step: step, state: state,
    renderSelftestBadges: renderSelftestBadges,
    abiVersion: abiVersion, status: status, tier: tier, goTo: goTo, lastError: lastError,
    anchor: anchor, anchorName: anchorName, derivedAnchors: derivedAnchors,
    scenarioJson: scenarioJson, open: open, formulas: formulas,
    PRESETS: PRESETS, TIERS: TIERS, COMMANDS: COMMANDS, STATUS: STATUS,
    STATE_FIELDS: STATE_FIELDS, WASM_EXPORTS: WASM_EXPORTS, WASM_URL: WASM_URL
  };
  /* FORMULAS resolves lazily, so the script order of engine.js and formulas.js is free. */
  Object.defineProperty(api, "FORMULAS", { enumerable: true, get: formulas });

  if (doc) {
    root.FabrailEngine = api;
    var start = function () {
      if (doc.querySelector("[data-selftest], [data-selftest-field]")) load().then(renderSelftestBadges);
    };
    if (doc.readyState === "loading") doc.addEventListener("DOMContentLoaded", start);
    else start();
  }
  if (typeof module === "object" && module.exports) module.exports = api;
})(typeof window !== "undefined" ? window : globalThis);
