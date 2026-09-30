/* fabrail closed forms in the browser (plan §3b). Owner: A13.
 *
 * Pure-JavaScript mirrors of fabrail_params::derive (f64). Every page falls back to
 * these when the WebAssembly engine is unavailable, and the landing-page hero is
 * animated from them. They are not a second implementation of the physics: the Rust
 * functions are the single source, and node evaluates this block against them
 * (tests/test_site.py::test_formulas_match_anchors, FAB-SW-013). Each formula carries
 * its source and the PHYSICS § that defines it; argument order equals the Rust
 * signature.
 *
 * The block between the FORMULAS:BEGIN and FORMULAS:END markers is self-contained
 * (no references outside it), so a checker may extract it and evaluate
 * `<block>; FORMULAS` on its own.
 *
 * Contents of FORMULAS:
 *   CONST                         physical constants and imported caps (with sources)
 *   closed forms                  one function per derive:: function (names below)
 *   CLASSES, CLASS_FIELDS         preset inputs per class and their JSON pointers in
 *                                 Params.to_json() (drift-checked against the presets)
 *   derivedFor(cls)               the Params.derived() subset these inputs determine,
 *                                 with identical key names
 *   scurve, terminusProfile, closeProfile   time profiles for plots and animation
 *   CASES                         cross-check table: JS call ↔ fabrail._native call
 *
 * Exposed as window.FabrailFormulas, as FabrailEngine.FORMULAS, and as module.exports
 * under node (require("docs/assets/js/formulas.js")).
 */
/* FORMULAS:BEGIN */
var FORMULAS = (function () {
  "use strict";

  /* ---- constants ------------------------------------------------------------------ */
  var CONST = {
    /* Standard gravity g₀, m/s². Source: 3rd CGPM (1901); = derive::G0_MPS2 (PHYSICS §3). */
    G0_MPS2: 9.80665,
    /* Air density at 20 °C, kg/m³. Source: ISO 2533; = derive::RHO_AIR_KG_PER_M3 (PHYSICS §11). */
    RHO_AIR_KG_PER_M3: 1.204,
    /* Sharp-edged orifice discharge coefficient. Source: ISO 5167-2; = derive::ORIFICE_CD (PHYSICS §11). */
    ORIFICE_CD: 0.6,
    /* Pedestrian kinetic-energy cap, J. Source: fabrail_params::presets::PEDESTRIAN_E_MAX_J
       (ANSI A156.19 / A156.38 low-energy limit; PHYSICS §18; FAB-SAF-001). Mirrored here for
       the browser, drift-checked against _native.v_low_energy's default. */
    E_MAX_J: 1.69,
    /* Pedestrian momentum cap, kg·m/s. Source: fabrail_params::presets::PEDESTRIAN_P_MAX_KGMPS
       (PHYSICS §18; FAB-SAF-002). Drift-checked against _native.momentum_speed_cap's default. */
    P_MAX_KGMPS: 22.3,
    /* Static clamp limits, N: unsensed / sensed or last 50 mm / final. Source:
       fabrail_params::presets PEDESTRIAN_F_STATIC_{,SENSED_,FINAL_}N (PHYSICS §18; FAB-SAF-005). */
    F_STATIC_N: 67.0,
    F_STATIC_SENSED_N: 150.0,
    F_STATIC_FINAL_N: 80.0,
    /* Body stiffness, N/m: hand 75 N/mm, chest 25 N/mm. Source: ISO/TS 15066:2016 Annex A;
       fabrail_params::presets::ISO15066_{HAND,CHEST}_K_N_PER_M (PHYSICS §18). */
    K_HAND_N_PER_M: 75.0e3,
    K_CHEST_N_PER_M: 25.0e3,
    /* Closing-edge seal stiffness ceiling, N/m. Source: presets::EDGE_K_MAX_N_PER_M (PHYSICS §18). */
    K_EDGE_N_PER_M: 10.0e3,
    /* Seismic speeds, km/s (crustal P and S). Source: _native.sp_time defaults; PHYSICS §20. */
    VP_KMPS: 6.0,
    VS_KMPS: 3.5,
    /* On-site decision time after the P arrival, s. Source: derive::ONSITE_DECISION_S
       (core research journal C7; PHYSICS §20). */
    ONSITE_DECISION_S: 1.2,
    /* Network alert latency after origin, s. Source: derive::NETWORK_ALERT_S (C7; PHYSICS §20). */
    NETWORK_ALERT_S: 8.0,
    /* Terminus quintic entry distance d_ε, m. Source: _native.terminus_t_arrive default;
       PHYSICS §16.1. */
    D_EPS_M: 2.0e-4,
    /* DRAWER z-seat step after arrival, s. Source: plan §4b terminus table (+0.2 s seat);
       = presets control.levitation.seat_time_s (PHYSICS §16.1). */
    SEAT_TIME_S: 0.2
  };

  function isPos(x) { return typeof x === "number" && x > 0; }

  /* ---- levitation (PHYSICS §2, §3) ---------------------------------------------- */

  /* Magnet reluctance length h′ = h_m/μ_rec, m (PHYSICS §2; derive::h_prime). */
  function hPrime(hMM, muRec) {
    return muRec <= 0 ? 0 : hMM / muRec;
  }

  /* Open-loop heave pole p = √(4g₀/(2g* + h′)), rad/s (PHYSICS §3; derive::open_loop_pole;
     FAB-LEV-003 anchors 69.49 / 47.57 / 32.50 rad/s). */
  function openLoopPole(gStarM, hPrimeM) {
    var d = 2 * gStarM + hPrimeM;
    return d <= 0 ? 0 : Math.sqrt((4 * CONST.G0_MPS2) / d);
  }

  /* Zero-power mass m(g) = m*·((2g* + h′)/(2g + h′))², kg (PHYSICS §3; derive::zero_power_mass;
     FAB-LEV-005 DRAWER 1.5 mm → 45.76 kg). */
  function zeroPowerMass(gapM, gStarM, hPrimeM, mStarKg) {
    var r = (2 * gStarM + hPrimeM) / (2 * gapM + hPrimeM);
    return mStarKg * r * r;
  }

  /* Zero-power gap g(m) = ((2g* + h′)·√(m* ÷ m) − h′)/2, m (PHYSICS §3; derive::zero_power_gap;
     DOOR +10 kg → 4.5967 mm). */
  function zeroPowerGap(mKg, gStarM, hPrimeM, mStarKg) {
    if (mKg <= 0) return Infinity;
    return ((2 * gStarM + hPrimeM) * Math.sqrt(mStarKg / mKg) - hPrimeM) / 2;
  }

  /* ---- motion, governor, clamps (PHYSICS §16, §18) ----------------------------------- */

  /* Low-energy speed cap √(2E/m), m/s (PHYSICS §16; derive::v_low_energy; FAB-SAF-001). */
  function vLowEnergy(mKg, eJ) {
    var e = eJ === undefined ? CONST.E_MAX_J : eJ;
    return mKg <= 0 || e < 0 ? 0 : Math.sqrt((2 * e) / mKg);
  }

  /* Momentum speed cap p/m, m/s (PHYSICS §16; derive::momentum_cap; FAB-SAF-002). */
  function momentumCap(mKg, pKgmps) {
    var p = pKgmps === undefined ? CONST.P_MAX_KGMPS : pKgmps;
    return mKg <= 0 ? 0 : p / mKg;
  }

  /* Unsensed cap min(√(2E/m), p/m); p ≤ 0 means "not applicable", m/s
     (PHYSICS §16; derive::unsensed_speed_cap). */
  function unsensedSpeedCap(mKg, eJ, pKgmps) {
    var v = vLowEnergy(mKg, eJ);
    var p = pKgmps === undefined ? CONST.P_MAX_KGMPS : pKgmps;
    return p > 0 ? Math.min(v, momentumCap(mKg, p)) : v;
  }

  /* Jerk-limited stop distance: v²/(2a) + v·a/(2j) when v ≥ a²/j, else v·√(v/j), m
     (PHYSICS §16; derive::stop_distance; FAB-PRP-005 anchors 33.43 / 36.31 / 8.86 / 4.91 mm). */
  function stopDistance(vMps, aMps2, jMps3) {
    var v = Math.abs(vMps);
    if (aMps2 <= 0 || jMps3 <= 0) return Infinity;
    if (v >= (aMps2 * aMps2) / jMps3) return (v * v) / (2 * aMps2) + (v * aMps2) / (2 * jMps3);
    return v * Math.sqrt(v / jMps3);
  }

  /* Quasi-static clamp force F = √(2E·k), N (PHYSICS §18; derive::clamp_force; FAB-SAF-005:
     1.69 J → chest 290.7 N, hand 503.5 N, hand ⊕ edge 173 N). */
  function clampForce(eJ, kNPerM) {
    return eJ <= 0 || kNPerM <= 0 ? 0 : Math.sqrt(2 * eJ * kNPerM);
  }

  /* Springs in series k₁k₂/(k₁ + k₂), N/m (PHYSICS §18; derive::series_k). */
  function seriesK(k1, k2) {
    var s = k1 + k2;
    return s <= 0 ? 0 : (k1 * k2) / s;
  }

  /* ISO/TS 15066 transient-contact speed F/√(μ·k), μ = m_h·m/(m_h + m), m/s
     (PHYSICS §18; derive::iso15066_speed; ISO/TS 15066:2016 Annex A). */
  function iso15066Speed(fN, mHKg, mKg, kNPerM) {
    var mu = mHKg + mKg > 0 ? (mHKg * mKg) / (mHKg + mKg) : 0;
    return mu <= 0 || kNPerM <= 0 ? 0 : fN / Math.sqrt(mu * kNPerM);
  }

  /* Unattended automatic-close speed min(v_cap, F_pinch/√(m_gov·k_series)), m/s
     (plan §4b; derive::unattended_close_speed: 0.23 m/s at 48 kg, 150 N, 75 ⊕ 10 N/mm). */
  function unattendedCloseSpeed(vCapMps, fPinchN, mGovKg, kSeriesNPerM) {
    if (mGovKg <= 0 || kSeriesNPerM <= 0) return 0;
    return Math.min(vCapMps, fPinchN / Math.sqrt(mGovKg * kSeriesNPerM));
  }

  /* Bumper energy rating ½·m·v², J (PHYSICS §8; derive::bumper_rating; FAB-SAF-014). */
  function bumperRating(mKg, vMps) {
    return 0.5 * mKg * vMps * vMps;
  }

  /* Terminus blend out of cruise: t_b = √(τ² + 2v/j) − τ, d_b = j·t_b·τ²,
     d_0 = d_b + v·t_b − j·t_b³/6 (PHYSICS §16.1; derive::terminus_blend). */
  function terminusBlend(vMps, jMps3, tauS) {
    var v = Math.abs(vMps);
    if (jMps3 <= 0 || tauS <= 0) return { tB: 0, dB: 0, d0: 0 };
    var tb = Math.sqrt(tauS * tauS + (2 * v) / jMps3) - tauS;
    var db = jMps3 * tb * tauS * tauS;
    return { tB: tb, dB: db, d0: db + v * tb - (jMps3 * tb * tb * tb) / 6 };
  }

  /* Terminus arrival t_b + τ·ln(d_b/d_ε) + T_q with T_q = τ, s (PHYSICS §16.1;
     derive::terminus_t_arrive; DRAWER 0.98 s before the +0.2 s seat step). */
  function terminusTArrive(vMps, jMps3, tauS, dEpsM) {
    var eps = dEpsM === undefined ? CONST.D_EPS_M : dEpsM;
    var b = terminusBlend(vMps, jMps3, tauS);
    var capture = eps > 0 && b.dB > eps ? tauS * Math.log(b.dB / eps) : 0;
    return b.tB + capture + tauS;
  }

  /* Minimum terminus time constant τ_min = v_app/a_max − a_max/(2j), s (PHYSICS §16.1;
     derive::tau_min; enforced by params::validate). */
  function tauMin(vAppMps, aMaxMps2, jMps3) {
    if (aMaxMps2 <= 0 || jMps3 <= 0) return Infinity;
    return Math.abs(vAppMps) / aMaxMps2 - aMaxMps2 / (2 * jMps3);
  }

  /* ---- haptics, air (PHYSICS §11, §17) ------------------------------------------------ */

  /* Passivity stiffness bound K_max = 2b/(T + 2T_d), N/m (Colgate–Schenkel; PHYSICS §17;
     derive::passivity_k_max; T_d defaults to T as in _native). */
  function passivityKMax(bNsPerM, tS, tDS) {
    var den = tS + 2 * (tDS === undefined ? tS : tDS);
    return den <= 0 ? Infinity : (2 * bNsPerM) / den;
  }

  /* Cavity piston force ½ρ(v·A_p/(C_d·A_leak))²·A_p, N (PHYSICS §11; derive::piston_force;
     FAB-ENV-003: 0.06 m², 0.5 m/s, 40 cm² → 5.6 N). */
  function pistonForce(aPM2, vMps, aLeakM2) {
    if (aLeakM2 <= 0) return 0;
    var u = (vMps * aPM2) / (CONST.ORIFICE_CD * aLeakM2);
    return 0.5 * CONST.RHO_AIR_KG_PER_M3 * u * u * aPM2;
  }

  /* ---- seismic response (PHYSICS §16.2, §21) -------------------------------------------- */

  /* Den Hartog optimum {f = 1/(1+μ), ζ = √(3μ/(8(1+μ)³))} (Den Hartog 1956; PHYSICS §21;
     derive::den_hartog; FAB-SEI-016: μ 0.02 → 0.98039 / 0.08407). */
  function denHartog(mu) {
    if (mu <= 0) return { f: 1, zeta: 0 };
    var q = 1 + mu;
    return { f: 1 / q, zeta: Math.sqrt((3 * mu) / (8 * q * q * q)) };
  }

  /* Den Hartog equal-peak amplification √(1 + 2/μ) (PHYSICS §21; derive::den_hartog_peak). */
  function denHartogPeak(mu) {
    return mu <= 0 ? Infinity : Math.sqrt(1 + 2 / mu);
  }

  /* Warburton optimum, base acceleration, white noise (Warburton 1982; PHYSICS §21;
     derive::warburton; μ 0.02 → 0.97548 / 0.07019). */
  function warburton(mu) {
    if (mu <= 0) return { f: 1, zeta: 0 };
    var q = 1 + mu;
    return {
      f: Math.sqrt(1 - mu / 2) / q,
      zeta: Math.sqrt((mu * (1 - mu / 4)) / (4 * q * (1 - mu / 2)))
    };
  }

  /* Warburton optimum, force (wind) excitation (Warburton 1982; PHYSICS §21;
     derive::warburton_force; plan §4: f 0.99257 / 0.98528 at μ 0.01 / 0.02). */
  function warburtonForce(mu) {
    if (mu <= 0) return { f: 1, zeta: 0 };
    var q = 1 + mu;
    return {
      f: Math.sqrt(1 + mu / 2) / q,
      zeta: Math.sqrt((mu * (1 + 0.75 * mu)) / (4 * q * (1 + mu / 2)))
    };
  }

  /* Passive transmissibility √((1 + (2ζr)²)/((1 − r²)² + (2ζr)²)) (PHYSICS §21;
     derive::transmissibility; FAB-SEI-014: T(3, 0.2) = 0.1931). */
  function transmissibility(r, zeta) {
    var c = 2 * zeta * r;
    var a = 1 - r * r;
    return Math.sqrt((1 + c * c) / (a * a + c * c));
  }

  /* Skyhook transmissibility 1/√((1 − r²)² + (2ζr)²) (PHYSICS §16.2, §21; derive::t_sky). */
  function tSky(r, zeta) {
    var c = 2 * zeta * r;
    var a = 1 - r * r;
    return 1 / Math.sqrt(a * a + c * c);
  }

  /* Minimum isolation period 2π·√(m·s/F_cont), s (PHYSICS §21; derive::t_min_iso;
     plan §2b item 5: WALL 6.3 s). */
  function tMinIso(mKg, strokeM, fContN) {
    return fContN <= 0 ? Infinity : 2 * Math.PI * Math.sqrt((mKg * strokeM) / fContN);
  }

  /* ---- seismic detection (PHYSICS §20) ----------------------------------------------- */

  /* Wu–Zhao peak displacement, log₁₀ Pd[cm] = −3.463 + 0.729·M − 1.374·log₁₀ R[km];
     returns metres (Wu & Zhao 2006; PHYSICS §20; derive::pd_wu_zhao; M7 R20 → 0.7118 cm). */
  function pdWuZhao(magnitude, rKm) {
    if (rKm <= 0) return Infinity;
    var logCm = -3.463 + 0.729 * magnitude - 1.374 * (Math.log(rKm) / Math.LN10);
    return 0.01 * Math.pow(10, logCm);
  }

  /* S−P time r/v_s − r/v_p, s (PHYSICS §20; derive::sp_time; 0.1190 s/km at 6.0 / 3.5 km/s). */
  function spTime(rHypKm, vpKmps, vsKmps) {
    var vp = vpKmps === undefined ? CONST.VP_KMPS : vpKmps;
    var vs = vsKmps === undefined ? CONST.VS_KMPS : vsKmps;
    return vp <= 0 || vs <= 0 ? 0 : rHypKm / vs - rHypKm / vp;
  }

  /* Lead time at the site r/v_s − t_alert (both from origin), s (PHYSICS §20; derive::lead_time). */
  function leadTime(rHypKm, vsKmps, tAlertS) {
    return vsKmps <= 0 ? 0 : rHypKm / vsKmps - tAlertS;
  }

  /* On-site / network crossover r = v_p·(t_network − t_onsite), km (PHYSICS §20;
     derive::lead_crossover_km; 6.0 km/s, 1.2 s, 8 s → 40.8 km). */
  function leadCrossoverKm(vpKmps, tOnsiteS, tNetworkS) {
    var vp = vpKmps === undefined ? CONST.VP_KMPS : vpKmps;
    var a = tOnsiteS === undefined ? CONST.ONSITE_DECISION_S : tOnsiteS;
    var b = tNetworkS === undefined ? CONST.NETWORK_ALERT_S : tNetworkS;
    return vp * (b - a);
  }

  /* ---- class inputs (mirrors of fabrail_params::presets; drift-checked) ---------------- */

  /* JSON pointer of each CLASSES field in Params.preset(cls).to_json(). A node check
     compares every value with the preset (exact); the generated ANCHORS data override
     nothing here, they are compared against derivedFor(). */
  var CLASS_FIELDS = {
    m_rated_kg: "/design/carriage/m_rated_kg",
    m_star_kg: "/design/carriage/m_star_kg",
    m_op_min_kg: "/design/carriage/m_op_min_kg",
    g_star_m: "/design/suspension/g_star_m",
    h_m_m: "/design/suspension/h_m_m",
    mu_rec: "/design/suspension/mu_rec",
    zp_band_lo_m: "/design/suspension/zp_band_m/0",
    zp_band_hi_m: "/design/suspension/zp_band_m/1",
    g_seat_m: "/design/suspension/g_seat_m",
    g_land_m: "/design/suspension/g_land_m",
    travel_m: "/design/rail/travel_m",
    k_edge_n_per_m: "/design/damping/k_edge_n_per_m",
    b_dist_n_s_per_m: "/design/damping/b_dist_n_s_per_m",
    f_cont_n: "/design/lsm/f_cont_n",
    mass_rated_kg: "/safety/mass_rated_kg",
    v_open_max_mps: "/safety/v_open_max_mps",
    v_close_max_mps: "/safety/v_close_max_mps",
    a_max_mps2: "/control/motion/a_max_mps2",
    j_max_mps3: "/control/motion/j_max_mps3",
    tau_s: "/control/motion/terminus/tau_s",
    j_term_mps3: "/control/motion/terminus/j_mps3",
    d_eps_m: "/control/motion/terminus/d_eps_m",
    base_hz: "/control/rates/base_hz",
    fast_decim: "/control/rates/fast_decim",
    iso_stroke_m: "/control/seismic/isolation/stroke_m"
  };

  /* Preset inputs (fabrail_params::presets; core design §C.2 as corrected by plan §2b).
     Speed caps written as the preset computes them (SAF-002: DRAWER √(3.38/48),
     DOOR open √(3.38/100), WALL 22.3/500; DOOR close 0.152 and HEAVY 0.025 are policy). */
  var CLASSES = {
    drawer: {
      label: "DRAWER", m_rated_kg: 48, m_star_kg: 26, m_op_min_kg: 8,
      g_star_m: 0.0025, h_m_m: 0.00328, mu_rec: 1.05, zp_band_lo_m: 0.0015, zp_band_hi_m: 0.006,
      g_seat_m: 0.0035, g_land_m: 0.0065, travel_m: 0.3, k_edge_n_per_m: 10e3,
      b_dist_n_s_per_m: 2, f_cont_n: 20, mass_rated_kg: 48,
      v_open_max_mps: vLowEnergy(48), v_close_max_mps: vLowEnergy(48),
      a_max_mps2: 1.5, j_max_mps3: 20, tau_s: 0.15, j_term_mps3: 20, d_eps_m: 2e-4,
      base_hz: 20000, fast_decim: 2, iso_stroke_m: 0.2
    },
    door: {
      label: "DOOR", m_rated_kg: 100, m_star_kg: 100, m_op_min_kg: 90,
      g_star_m: 0.005, h_m_m: 0.0077, mu_rec: 1.05, zp_band_lo_m: 0.0044, zp_band_hi_m: 0.0058,
      g_seat_m: 0, g_land_m: 0.00625, travel_m: 1.0, k_edge_n_per_m: 10e3,
      b_dist_n_s_per_m: 5, f_cont_n: 50, mass_rated_kg: 100,
      v_open_max_mps: vLowEnergy(100), v_close_max_mps: 0.152,
      a_max_mps2: 0.75, j_max_mps3: 5, tau_s: 0.25, j_term_mps3: 5, d_eps_m: 2e-4,
      base_hz: 20000, fast_decim: 2, iso_stroke_m: 0.2
    },
    wall: {
      label: "WALL", m_rated_kg: 500, m_star_kg: 500, m_op_min_kg: 450,
      g_star_m: 0.01, h_m_m: 0.018, mu_rec: 1.05, zp_band_lo_m: 0.0088, zp_band_hi_m: 0.0115,
      g_seat_m: 0, g_land_m: 0.0125, travel_m: 6.0, k_edge_n_per_m: 10e3,
      b_dist_n_s_per_m: 20, f_cont_n: 100, mass_rated_kg: 500,
      v_open_max_mps: momentumCap(500), v_close_max_mps: momentumCap(500),
      a_max_mps2: 0.15, j_max_mps3: 1.5, tau_s: 0.6, j_term_mps3: 1.5, d_eps_m: 2e-4,
      base_hz: 20000, fast_decim: 4, iso_stroke_m: 0.2
    },
    heavy: {
      label: "HEAVY", m_rated_kg: 5000, m_star_kg: 5000, m_op_min_kg: 4500,
      g_star_m: 0.01, h_m_m: 0.018, mu_rec: 1.05, zp_band_lo_m: 0.0088, zp_band_hi_m: 0.0115,
      g_seat_m: 0, g_land_m: 0.0125, travel_m: 12.0, k_edge_n_per_m: 10e3,
      b_dist_n_s_per_m: 100, f_cont_n: 500, mass_rated_kg: 5000,
      v_open_max_mps: 0.025, v_close_max_mps: 0.025,
      a_max_mps2: 0.08, j_max_mps3: 1, tau_s: 1.0, j_term_mps3: 1, d_eps_m: 2e-4,
      base_hz: 20000, fast_decim: 4, iso_stroke_m: 0.2
    }
  };

  /* The Params.derived() entries these inputs determine, with the same keys and the same
     expressions as derive::derived (PHYSICS §3, §16, §16.1, §17, §18, §21). */
  function derivedFor(cls) {
    var c = typeof cls === "string" ? CLASSES[cls] : cls;
    if (!c) return null;
    var hp = hPrime(c.h_m_m, c.mu_rec);
    var vCap = Math.max(c.v_open_max_mps, c.v_close_max_mps);
    var eCap = bumperRating(c.mass_rated_kg, vCap);
    var tb = terminusBlend(vCap, c.j_term_mps3, c.tau_s);
    var tFast = c.base_hz > 0 ? c.fast_decim / c.base_hz : 0;
    var gFloat = zeroPowerGap(c.m_rated_kg, c.g_star_m, hp, c.m_star_kg);
    return {
      pole_rad_s: openLoopPole(c.g_star_m, hp),
      h_prime_m: hp,
      float_gap_rated_m: Math.min(Math.max(gFloat, c.zp_band_lo_m), c.zp_band_hi_m),
      v_open_cap_mps: c.v_open_max_mps,
      v_close_cap_mps: c.v_close_max_mps,
      ke_at_cap_j: eCap,
      d_stop_m: stopDistance(vCap, c.a_max_mps2, c.j_max_mps3),
      clamp_hand_edge_n: clampForce(eCap, seriesK(CONST.K_HAND_N_PER_M, c.k_edge_n_per_m)),
      tau_min_s: tauMin(vCap, c.a_max_mps2, c.j_term_mps3),
      terminus_d0_m: tb.d0,
      terminus_db_m: tb.dB,
      terminus_t_arrive_s: terminusTArrive(vCap, c.j_term_mps3, c.tau_s, c.d_eps_m),
      k_max_passive_n_per_m: passivityKMax(c.b_dist_n_s_per_m, tFast, tFast),
      t_min_iso_s: tMinIso(c.m_rated_kg, c.iso_stroke_m, c.f_cont_n)
    };
  }

  /* ---- time profiles (display; the controller's generators live in core::motion) ----- */

  /* Piecewise-constant-jerk integrator: segments [[duration, jerk], ...] from (x0, v0, a0).
     Returns { T, end: {x, v, a}, sample(t) -> {x, v, a} }. */
  function jerkPath(segments, x0, v0, a0) {
    var starts = [];
    var x = x0 || 0, v = v0 || 0, a = a0 || 0, t = 0;
    for (var i = 0; i < segments.length; i++) {
      var d = Math.max(0, segments[i][0]), j = segments[i][1];
      starts.push({ t: t, d: d, j: j, x: x, v: v, a: a });
      x += v * d + (a * d * d) / 2 + (j * d * d * d) / 6;
      v += a * d + (j * d * d) / 2;
      a += j * d;
      t += d;
    }
    var end = { x: x, v: v, a: a };
    return {
      T: t,
      end: end,
      sample: function (tq) {
        if (!(tq > 0)) return { x: x0 || 0, v: v0 || 0, a: a0 || 0 };
        for (var k = 0; k < starts.length; k++) {
          var s = starts[k];
          if (tq <= s.t + s.d) {
            var u = tq - s.t;
            return {
              x: s.x + s.v * u + (s.a * u * u) / 2 + (s.j * u * u * u) / 6,
              v: s.v + s.a * u + (s.j * u * u) / 2,
              a: s.a + s.j * u
            };
          }
        }
        return { x: end.x, v: end.v, a: end.a };
      }
    };
  }

  /* Jerk-limited acceleration from rest to v: [[t_j, +j], [t_a, 0], [t_j, −j]] with the
     distance of stopDistance(v, a, j) (PHYSICS §16). */
  function accelSegments(vMps, aMps2, jMps3) {
    if (vMps >= (aMps2 * aMps2) / jMps3) {
      var tj = aMps2 / jMps3;
      return [[tj, jMps3], [vMps / aMps2 - tj, 0], [tj, -jMps3]];
    }
    var tj2 = Math.sqrt(vMps / jMps3);
    return [[tj2, jMps3], [0, 0], [tj2, -jMps3]];
  }

  /* Symmetric seven-segment rest-to-rest S-curve of length L under v_max, a_max, j_max;
     the peak speed drops when L is too short to reach v_max (PHYSICS §16; FAB-PRP-005).
     Returns { T, vPeak, sample(t) -> {x, v, a} }. */
  function scurve(lengthM, vMax, aMax, jMax) {
    var L = Math.abs(lengthM);
    if (!isPos(L) || !isPos(vMax) || !isPos(aMax) || !isPos(jMax)) {
      return { T: 0, vPeak: 0, sample: function () { return { x: 0, v: 0, a: 0 }; } };
    }
    var v = vMax;
    if (2 * stopDistance(v, aMax, jMax) > L) {
      var vA = (aMax / 2) * (-aMax / jMax + Math.sqrt((aMax * aMax) / (jMax * jMax) + (4 * L) / aMax));
      v = vA >= (aMax * aMax) / jMax ? vA : Math.cbrt((L / 2) * (L / 2) * jMax);
    }
    var acc = accelSegments(v, aMax, jMax);
    var tCruise = Math.max(0, (L - 2 * stopDistance(v, aMax, jMax)) / v);
    var segs = acc.concat([[tCruise, 0]], [[acc[0][0], -jMax], [acc[1][0], 0], [acc[2][0], jMax]]);
    var path = jerkPath(segs, 0, 0, 0);
    return { T: path.T, vPeak: v, sample: path.sample };
  }

  /* Log-damped terminus from cruise speed v (PHYSICS §16.1): constant-jerk blend to d_b,
     exponential capture d_b·e^(−s/τ) down to d_ε (a straight line on a semilog plot), then
     the quintic from (d_ε, −d_ε/τ, d_ε/τ²) over T_q = τ to rest. sample(t) returns the
     distance to go d ≥ 0, the closing speed v ≥ 0 and a = dv/dt. */
  function terminusProfile(vMps, jMps3, tauS, dEpsM) {
    var v0 = Math.abs(vMps);
    var eps = dEpsM === undefined ? CONST.D_EPS_M : dEpsM;
    var b = terminusBlend(v0, jMps3, tauS);
    var tCap = eps > 0 && b.dB > eps ? tauS * Math.log(b.dB / eps) : 0;
    var dq = tCap > 0 ? eps : b.dB;
    var T = b.tB + tCap + tauS;
    function sample(t) {
      if (!(t > 0)) return { d: b.d0, v: v0, a: 0, phase: "blend" };
      if (t < b.tB) {
        return {
          d: b.d0 - v0 * t + (jMps3 * t * t * t) / 6,
          v: v0 - (jMps3 * t * t) / 2,
          a: -jMps3 * t,
          phase: "blend"
        };
      }
      if (t < b.tB + tCap) {
        var d = b.dB * Math.exp(-(t - b.tB) / tauS);
        return { d: d, v: d / tauS, a: -d / (tauS * tauS), phase: "capture" };
      }
      if (t < T && tauS > 0) {
        var u = (t - b.tB - tCap) / tauS;
        var u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
        return {
          d: dq * (1 - u + u2 / 2 - 5.5 * u3 + 8.5 * u4 - 3.5 * u5),
          v: -(dq / tauS) * (-1 + u - 16.5 * u2 + 34 * u3 - 17.5 * u4),
          a: -(dq / (tauS * tauS)) * (1 - 33 * u + 102 * u2 - 70 * u3),
          phase: "quintic"
        };
      }
      return { d: 0, v: 0, a: 0, phase: "arrived" };
    }
    return { tB: b.tB, dB: b.dB, d0: b.d0, tCapture: tCap, tQ: tauS, T: T, sample: sample };
  }

  /* A complete automatic close of length L: jerk-limited acceleration to v, cruise, the
     log-damped terminus and an optional seat step (plan §4b: DRAWER 0.30 m in ≈ 2 s +
     0.2 s seat). When L is too short, v is reduced by bisection until acceleration plus
     the terminus fit. sample(t) -> { d, v, phase, seat } with seat ∈ [0, 1]. */
  function closeProfile(o) {
    var L = Math.abs(o.lengthM);
    var aMax = o.aMaxMps2, jMax = o.jMaxMps3, tau = o.tauS;
    var jT = o.jTermMps3 === undefined ? jMax : o.jTermMps3;
    var tSeat = o.seatS || 0;
    function need(v) { return stopDistance(v, aMax, jMax) + terminusBlend(v, jT, tau).d0; }
    var v = Math.abs(o.vMps);
    if (need(v) > L) {
      var lo = 0, hi = v;
      for (var i = 0; i < 60; i++) {
        var mid = (lo + hi) / 2;
        if (need(mid) > L) hi = mid; else lo = mid;
      }
      v = lo;
    }
    var acc = jerkPath(accelSegments(v, aMax, jMax), 0, 0, 0);
    var term = terminusProfile(v, jT, tau, o.dEpsM);
    var dCruise = Math.max(0, L - acc.end.x - term.d0);
    var tCruise = v > 0 ? dCruise / v : 0;
    var t1 = acc.T, t2 = t1 + tCruise, t3 = t2 + term.T, T = t3 + tSeat;
    function sample(t) {
      if (!(t > 0)) return { d: L, v: 0, phase: "rest", seat: 0 };
      if (t < t1) { var s = acc.sample(t); return { d: L - s.x, v: s.v, phase: "accel", seat: 0 }; }
      if (t < t2) return { d: L - acc.end.x - v * (t - t1), v: v, phase: "cruise", seat: 0 };
      if (t < t3) { var q = term.sample(t - t2); return { d: q.d, v: q.v, phase: q.phase, seat: 0 }; }
      if (t < T) return { d: 0, v: 0, phase: "seat", seat: (t - t3) / tSeat };
      return { d: 0, v: 0, phase: "done", seat: tSeat > 0 ? 1 : 0 };
    }
    return {
      L: L, v: v, T: T, tAccel: t1, tCruise: tCruise, tTerminus: term.T, tSeat: tSeat,
      tCaptureStart: t2 + term.tB, tQuinticStart: t2 + term.tB + term.tCapture,
      terminus: term, sample: sample
    };
  }

  /* ---- cross-check table ------------------------------------------------------------- */

  /* Each case: js function + args ↔ fabrail._native function + the same args; `out`
     maps a structured JS result onto the native result (tuple index or dict key).
     `expect`/`tol` state the published anchor where one exists (core design §E.5,
     plan §4/§4b); node evaluates `js`, Python evaluates `py`, and both are compared
     (relative 1e-12) and against `expect` (absolute `tol`). */
  var hpDrawer = hPrime(0.00328, 1.05), hpDoor = hPrime(0.0077, 1.05), hpWall = hPrime(0.018, 1.05);
  var kHandEdge = seriesK(CONST.K_HAND_N_PER_M, CONST.K_EDGE_N_PER_M);
  var CASES = [
    { id: "pole_drawer", js: "openLoopPole", args: [0.0025, hpDrawer], py: "open_loop_pole", expect: 69.49, tol: 0.005, src: "core §E.5; FAB-LEV-003" },
    { id: "pole_door", js: "openLoopPole", args: [0.005, hpDoor], py: "open_loop_pole", expect: 47.57, tol: 0.005, src: "core §E.5; FAB-LEV-003" },
    { id: "pole_wall", js: "openLoopPole", args: [0.01, hpWall], py: "open_loop_pole", expect: 32.50, tol: 0.005, src: "core §E.5; FAB-LEV-003" },
    { id: "zp_mass_drawer_1p5mm", js: "zeroPowerMass", args: [0.0015, 0.0025, hpDrawer, 26], py: "zero_power_mass", expect: 45.76, tol: 0.005, src: "FAB-LEV-005" },
    { id: "zp_mass_drawer_5mm", js: "zeroPowerMass", args: [0.005, 0.0025, hpDrawer, 26], py: "zero_power_mass", expect: 9.96, tol: 0.005, src: "FAB-LEV-005" },
    { id: "zp_gap_door_plus10kg", js: "zeroPowerGap", args: [110, 0.005, hpDoor, 100], py: "zero_power_gap", expect: 4.5967e-3, tol: 5e-8, src: "core §E.5 DOOR +98.1 N" },
    { id: "v_cap_drawer", js: "vLowEnergy", args: [48], py: "v_low_energy", expect: 0.2654, tol: 5e-5, src: "core §E.5; FAB-SAF-002" },
    { id: "v_cap_door_open", js: "vLowEnergy", args: [100], py: "v_low_energy", expect: 0.1838, tol: 5e-5, src: "core §E.5; FAB-SAF-002" },
    { id: "v_cap_wall", js: "momentumCap", args: [500], py: "momentum_speed_cap", expect: 0.0446, tol: 5e-6, src: "core §E.5; FAB-SAF-002" },
    { id: "v_cap_explicit", js: "vLowEnergy", args: [26, 1.0], py: "v_low_energy" },
    { id: "d_stop_drawer", js: "stopDistance", args: [0.2654, 1.5, 20], py: "stop_distance", expect: 33.43e-3, tol: 5e-6, src: "core §E.5; FAB-PRP-005 (v rounded as in the anchor)" },
    { id: "d_stop_door", js: "stopDistance", args: [0.1838, 0.75, 5], py: "stop_distance", expect: 36.31e-3, tol: 5e-6, src: "core §E.5; FAB-PRP-005 (v rounded as in the anchor)" },
    { id: "d_stop_drawer_cap", js: "stopDistance", args: [vLowEnergy(48), 1.5, 20], py: "stop_distance" },
    { id: "d_stop_wall", js: "stopDistance", args: [momentumCap(500), 0.15, 1.5], py: "stop_distance", expect: 8.86e-3, tol: 5e-6, src: "core §E.5; FAB-PRP-005" },
    { id: "d_stop_heavy", js: "stopDistance", args: [0.025, 0.08, 1], py: "stop_distance", expect: 4.91e-3, tol: 5e-6, src: "core §E.5; FAB-PRP-005" },
    { id: "d_stop_low_v", js: "stopDistance", args: [0.01, 1.5, 20], py: "stop_distance" },
    { id: "clamp_chest", js: "clampForce", args: [CONST.E_MAX_J, CONST.K_CHEST_N_PER_M], py: "clamp_force", expect: 290.7, tol: 0.05, src: "core §E.5; FAB-SAF-005" },
    { id: "clamp_hand", js: "clampForce", args: [CONST.E_MAX_J, CONST.K_HAND_N_PER_M], py: "clamp_force", expect: 503.5, tol: 0.05, src: "core §E.5; FAB-SAF-005" },
    { id: "clamp_hand_edge", js: "clampForce", args: [CONST.E_MAX_J, kHandEdge], py: "clamp_force", expect: 173, tol: 0.5, src: "core §E.5; FAB-SAF-005" },
    { id: "series_hand_edge", js: "seriesK", args: [CONST.K_HAND_N_PER_M, CONST.K_EDGE_N_PER_M], py: "series_stiffness", expect: 8823.5, tol: 0.05, src: "plan §4b (8.82 N/mm)" },
    { id: "iso15066_chest_500kg", js: "iso15066Speed", args: [140, 40, 500, CONST.K_CHEST_N_PER_M], py: "iso15066_speed", expect: 0.145, tol: 5e-4, src: "derive::iso15066_speed doc" },
    { id: "unattended_close_48kg", js: "unattendedCloseSpeed", args: [vLowEnergy(48), CONST.F_STATIC_SENSED_N, 48, kHandEdge], py: "unattended_close_speed", expect: 0.23, tol: 5e-3, src: "plan §4b" },
    { id: "bumper_drawer", js: "bumperRating", args: [48, 1.0], py: "bumper_rating", expect: 24, tol: 1e-12, src: "FAB-SAF-014" },
    { id: "terminus_blend_drawer", js: "terminusBlend", args: [0.265, 20, 0.15], py: "terminus_blend", out: { tB: "t_b_s", dB: "d_b_m", d0: "d_0_m" } },
    { id: "terminus_d0_drawer", js: "terminusBlend", args: [0.265, 20, 0.15], py: "terminus_blend", out: { d0: "d_0_m" }, expect: 49.8e-3, tol: 5e-5, src: "plan §4b terminus table" },
    { id: "terminus_db_door_close", js: "terminusBlend", args: [0.152, 5, 0.25], py: "terminus_blend", out: { dB: "d_b_m" }, expect: 31.6e-3, tol: 5e-5, src: "plan §4b terminus table" },
    { id: "t_arrive_drawer", js: "terminusTArrive", args: [0.265, 20, 0.15, 2e-4], py: "terminus_t_arrive", expect: 0.98, tol: 0.005, src: "plan §4b terminus table" },
    { id: "t_arrive_door_close", js: "terminusTArrive", args: [0.152, 5, 0.25, 2e-4], py: "terminus_t_arrive", expect: 1.62, tol: 0.005, src: "plan §4b terminus table" },
    { id: "t_arrive_wall", js: "terminusTArrive", args: [0.0446, 1.5, 0.6, 2e-4], py: "terminus_t_arrive", expect: 3.56, tol: 0.005, src: "plan §4b terminus table" },
    { id: "t_arrive_heavy", js: "terminusTArrive", args: [0.025, 1, 1.0, 2e-4], py: "terminus_t_arrive", expect: 5.84, tol: 0.005, src: "plan §4b terminus table" },
    { id: "tau_min_drawer", js: "tauMin", args: [vLowEnergy(48), 1.5, 20], py: "tau_min" },
    { id: "passivity_drawer", js: "passivityKMax", args: [2, 1e-4, 1e-4], py: "passivity_k_max" },
    { id: "piston_env003", js: "pistonForce", args: [0.06, 0.5, 40e-4], py: "piston_force", expect: 5.64, tol: 0.005, src: "FAB-ENV-003" },
    { id: "den_hartog_002", js: "denHartog", args: [0.02], py: "den_hartog", out: { f: 0, zeta: 1 }, expect: [0.98039, 0.08407], tol: 5e-6, src: "FAB-SEI-016; plan §4" },
    { id: "den_hartog_001", js: "denHartog", args: [0.01], py: "den_hartog", out: { f: 0, zeta: 1 }, expect: [0.99010, 0.06033], tol: 5e-6, src: "plan §4 exotic anchors" },
    { id: "den_hartog_peak_001", js: "denHartogPeak", args: [0.01], py: "den_hartog_peak", expect: 14.177, tol: 5e-4, src: "plan §4 exotic anchors" },
    { id: "warburton_002", js: "warburton", args: [0.02], py: "warburton", out: { f: 0, zeta: 1 }, expect: [0.97548, 0.07019], tol: 5e-6, src: "FAB-SEI-016; plan §4" },
    { id: "warburton_001", js: "warburton", args: [0.01], py: "warburton", out: { f: 0, zeta: 1 }, expect: [0.98762, 0.04981], tol: 5e-6, src: "plan §4 exotic anchors" },
    { id: "warburton_force_001", js: "warburtonForce", args: [0.01], py: "warburton_force", out: { f: 0, zeta: 1 } },
    { id: "warburton_force_002_f", js: "warburtonForce", args: [0.02], py: "warburton_force", out: { f: 0 }, expect: 0.98528, tol: 5e-6, src: "plan §4 exotic anchors" },
    { id: "transmissibility_3_02", js: "transmissibility", args: [3, 0.2], py: "transmissibility", expect: 0.1931, tol: 5e-5, src: "core §E.5; FAB-SEI-014" },
    { id: "transmissibility_1_02", js: "transmissibility", args: [1, 0.2], py: "transmissibility", expect: 2.693, tol: 5e-4, src: "FAB-SEI-014" },
    { id: "t_sky_2hz", js: "tSky", args: [2, 0.7], py: "skyhook_transmissibility" },
    { id: "t_min_iso_wall", js: "tMinIso", args: [500, 0.2, 100], py: "t_min_isolation", expect: 6.283, tol: 5e-4, src: "plan §2b item 5" },
    { id: "pd_m7_r20", js: "pdWuZhao", args: [7, 20], py: "pd_wu_zhao", expect: 0.7118e-2, tol: 5e-7, src: "core §E.5; FAB-SEI-012" },
    { id: "pd_m65_r30", js: "pdWuZhao", args: [6.5, 30], py: "pd_wu_zhao", expect: 0.1762e-2, tol: 5e-7, src: "FAB-SEI-012" },
    { id: "sp_per_km", js: "spTime", args: [1, 6.0, 3.5], py: "sp_time", expect: 0.1190, tol: 5e-5, src: "core §E.5; FAB-SEI-012" },
    { id: "lead_time_40km", js: "leadTime", args: [40, 3.5, 8], py: "lead_time" },
    { id: "crossover", js: "leadCrossoverKm", args: [6.0, 1.2, 8.0], py: "lead_crossover_km", expect: 40.8, tol: 1e-9, src: "core §E.5; FAB-SEI-012" }
  ];

  return {
    CONST: CONST,
    hPrime: hPrime,
    openLoopPole: openLoopPole,
    zeroPowerMass: zeroPowerMass,
    zeroPowerGap: zeroPowerGap,
    vLowEnergy: vLowEnergy,
    momentumCap: momentumCap,
    unsensedSpeedCap: unsensedSpeedCap,
    stopDistance: stopDistance,
    clampForce: clampForce,
    seriesK: seriesK,
    iso15066Speed: iso15066Speed,
    unattendedCloseSpeed: unattendedCloseSpeed,
    bumperRating: bumperRating,
    terminusBlend: terminusBlend,
    terminusTArrive: terminusTArrive,
    tauMin: tauMin,
    passivityKMax: passivityKMax,
    pistonForce: pistonForce,
    denHartog: denHartog,
    denHartogPeak: denHartogPeak,
    warburton: warburton,
    warburtonForce: warburtonForce,
    transmissibility: transmissibility,
    tSky: tSky,
    /* alias kept for the Wave-0 FORMULAS name */
    skyhookTransmissibility: tSky,
    tMinIso: tMinIso,
    pdWuZhao: pdWuZhao,
    spTime: spTime,
    leadTime: leadTime,
    leadCrossoverKm: leadCrossoverKm,
    CLASSES: CLASSES,
    CLASS_FIELDS: CLASS_FIELDS,
    derivedFor: derivedFor,
    jerkPath: jerkPath,
    scurve: scurve,
    terminusProfile: terminusProfile,
    closeProfile: closeProfile,
    CASES: CASES
  };
})();
/* FORMULAS:END */

(function (root) {
  "use strict";
  if (root && typeof root === "object") root.FabrailFormulas = FORMULAS;
  if (typeof module === "object" && module.exports) module.exports = FORMULAS;
})(typeof window !== "undefined" ? window : globalThis);
