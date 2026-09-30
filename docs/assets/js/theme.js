/* fabrail site theme (plan §3b). Owner after W0: A13.
 *
 * Dark-first. The stylesheet treats bare :root as dark and switches to light from
 * prefers-color-scheme, unless the [data-theme] stamp on <html> says otherwise.
 *
 * Load this file synchronously in <head>, before the stylesheet paints, so a stored
 * choice is stamped before first paint:
 *     <script src="assets/js/theme.js"></script>
 *
 * Every button with [data-theme-toggle] toggles the theme. The choice is kept in
 * localStorage under "fabrail-theme"; every storage access is wrapped in try/catch,
 * because private windows, file:// pages and blocked site data can throw.
 *
 * Frozen API (W0):
 *   window.FabrailTheme = { KEY, get(), set("light"|"dark"), toggle(), reducedMotion() }
 * Event: document dispatches "fabrail:themechange" with detail { theme } after a change,
 * so canvas scenes can repaint with the new CSS custom properties.
 */
(function (root) {
  "use strict";
  var KEY = "fabrail-theme";
  var doc = root.document;
  var html = doc.documentElement;

  function readStored() {
    try {
      var v = root.localStorage.getItem(KEY);
      return v === "light" || v === "dark" ? v : null;
    } catch (e) {
      return null;
    }
  }

  function writeStored(v) {
    try {
      root.localStorage.setItem(KEY, v);
    } catch (e) {
      /* storage unavailable: the choice lasts for this page only */
    }
  }

  function systemTheme() {
    try {
      return root.matchMedia && root.matchMedia("(prefers-color-scheme: light)").matches
        ? "light"
        : "dark";
    } catch (e) {
      return "dark";
    }
  }

  function get() {
    var s = html.getAttribute("data-theme");
    return s === "light" || s === "dark" ? s : systemTheme();
  }

  function reducedMotion() {
    try {
      return !!(root.matchMedia && root.matchMedia("(prefers-reduced-motion: reduce)").matches);
    } catch (e) {
      return false;
    }
  }

  function syncButtons() {
    var next = get() === "dark" ? "light" : "dark";
    var buttons = doc.querySelectorAll("[data-theme-toggle]");
    for (var i = 0; i < buttons.length; i++) {
      buttons[i].setAttribute("aria-label", "Switch to " + next + " theme");
      buttons[i].setAttribute("title", "Switch to " + next + " theme");
    }
  }

  function announce(v) {
    try {
      doc.dispatchEvent(new CustomEvent("fabrail:themechange", { detail: { theme: v } }));
    } catch (e) {
      /* CustomEvent unsupported: nothing listens in that case */
    }
  }

  function set(v) {
    if (v !== "light" && v !== "dark") return;
    html.setAttribute("data-theme", v);
    writeStored(v);
    syncButtons();
    announce(v);
  }

  function toggle() {
    set(get() === "dark" ? "light" : "dark");
  }

  /* Pre-paint: stamp a stored choice immediately. */
  var stored = readStored();
  if (stored) html.setAttribute("data-theme", stored);

  function wire() {
    var buttons = doc.querySelectorAll("[data-theme-toggle]");
    for (var i = 0; i < buttons.length; i++) buttons[i].addEventListener("click", toggle);
    syncButtons();
    try {
      var mq = root.matchMedia("(prefers-color-scheme: light)");
      /* Without a stored choice the page follows the system: canvases repaint too. */
      var onChange = function () {
        if (readStored() || html.hasAttribute("data-theme")) return;
        syncButtons();
        announce(get());
      };
      if (mq.addEventListener) mq.addEventListener("change", onChange);
      else if (mq.addListener) mq.addListener(onChange);
    } catch (e) {
      /* matchMedia unavailable */
    }
  }

  if (doc.readyState === "loading") doc.addEventListener("DOMContentLoaded", wire);
  else wire();

  root.FabrailTheme = { KEY: KEY, get: get, set: set, toggle: toggle, reducedMotion: reducedMotion };
})(window);
