/* fabrail site navigation (plan §3b). Owner after W0: A13.
 *
 * PAGES is the one canonical page list. Every page carries a static copy of these
 * links inside <nav data-site-nav> (so navigation works without JavaScript);
 * this script re-renders that container from PAGES and marks the current page
 * with aria-current="page". tests/test_site.py checks that the static links of
 * every page equal PAGES (FAB-SW-012).
 *
 * Attributes read:
 *   <body data-page="KEY">          which entry is current
 *   <nav data-site-nav data-base="../">   optional prefix for pages in subdirectories
 *
 * Frozen API (W0): window.FabrailNav = { PAGES, render(container, currentKey, base) }
 * In node, require("./nav.js").PAGES returns the same list.
 */
(function (root) {
  "use strict";

  var PAGES = [
    { key: "index", href: "index.html", label: "Overview" },
    { key: "levitate", href: "levitate.html", label: "Levitate" },
    { key: "glide", href: "glide.html", label: "Glide" },
    { key: "touch", href: "touch.html", label: "Touch" },
    { key: "quake", href: "quake.html", label: "Quake" },
    { key: "villa", href: "villa.html", label: "Villa" },
    { key: "sovereign", href: "sovereign.html", label: "Sovereign" },
    { key: "frontier", href: "frontier.html", label: "Frontier" },
    { key: "examples", href: "examples.html", label: "Examples" }
  ];

  function render(container, currentKey, base) {
    if (!container || !container.ownerDocument) return;
    var doc = container.ownerDocument;
    var prefix = typeof base === "string" ? base : container.getAttribute("data-base") || "";
    while (container.firstChild) container.removeChild(container.firstChild);
    for (var i = 0; i < PAGES.length; i++) {
      var p = PAGES[i];
      var a = doc.createElement("a");
      a.href = prefix + p.href;
      a.textContent = p.label;
      if (p.key === currentKey) a.setAttribute("aria-current", "page");
      container.appendChild(a);
    }
  }

  function init() {
    var doc = root.document;
    var current = (doc.body && doc.body.getAttribute("data-page")) || "";
    var navs = doc.querySelectorAll("[data-site-nav]");
    for (var i = 0; i < navs.length; i++) render(navs[i], current);
    var active = doc.querySelector('[data-site-nav] a[aria-current="page"]');
    if (active && active.scrollIntoView) {
      try {
        active.scrollIntoView({ block: "nearest", inline: "nearest" });
      } catch (e) {
        /* old browsers: harmless */
      }
    }
  }

  var api = { PAGES: PAGES, render: render };

  if (root.document) {
    if (root.document.readyState === "loading") root.document.addEventListener("DOMContentLoaded", init);
    else init();
    root.FabrailNav = api;
  }
  if (typeof module === "object" && module.exports) module.exports = api;
})(typeof window !== "undefined" ? window : globalThis);
