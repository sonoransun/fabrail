"""``python -m fabrail exo …``: the Sovereign/Exotic command line (exotic design §7.3 seam S3).

:func:`register` receives the top-level subparsers object from
:func:`fabrail.__main__.make_parser` and adds the ``exo`` parser. Every parser sets a
``handler(args) -> int`` default, which the top-level ``main`` calls. Registration
imports nothing native, so ``--help`` works without the compiled extension.

Subcommands::

    fabrail exo classes [--json] [--admission] [--hts] [--stray] [--iso13855] [--jog]
    fabrail exo scenarios [--json]
    fabrail exo simulate --class sentinel --scenario XS12 [--set NAME=VALUE] [--json PATH] [--strict]
    fabrail exo study NAME|all [--list] [--quick]
    fabrail exo pfh [--lambda-d 1e-6 --dc 0.99 --beta 0.02 --beta-d 0.01 --t1 8760 --mttr 8 --mrt 8] [--json]
    fabrail exo ride-through [--mass-kg 1e7 --suspension hybrid --a-rms-g 0.2 --shaking-s 300] [--json]
    fabrail exo eew [--site-km 120 --sensor-km 34 --depth-km 20] [--json]
    fabrail exo blast --t-a S [--t-detect S] [--ps-kpa P --td-ms T --b B] [--json]

Every alert is synthetic/injected (fabrail is not an EEW provider); frontier outputs
carry the 🔭 banner; nothing is certified. Every lead-time and energy output prints its
inputs.

Owner: W0-C2 (stub), then X7 (Wave 2).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

__all__ = ["register"]

_BANNER = "Sovereign/Exotic tier: research-tier classes, reduced models, synthetic alerts; nothing certified."


def _emit_json(obj: Any, path: Path | None) -> None:
    from ..export import dumps, write_json

    if path is None:
        print(dumps(obj))
    else:
        print(f"wrote {write_json(path, obj)}")


def _parse_sets(items: list[str] | None) -> dict[str, float]:
    out: dict[str, float] = {}
    for item in items or []:
        if "=" not in item:
            raise ValueError(f"--set expects NAME=VALUE (got {item!r})")
        k, v = item.split("=", 1)
        try:
            out[k.strip()] = float(v)
        except ValueError as exc:
            raise ValueError(f"--set {k.strip()}: not a number ({v!r})") from exc
    return out


# ---- classes -----------------------------------------------------------------------------


def _cmd_classes(args: argparse.Namespace) -> int:
    from . import classes, presets
    from ..export import to_jsonable as jsonable

    ps = presets.all_presets()
    if args.json:
        doc: dict[str, Any] = {"schema": "fabrail.exo_classes/1", "classes": [jsonable(p) for p in ps]}
        if args.admission:
            doc["admission"] = [jsonable(r) for r in classes.admission_table()]
        if args.hts:
            doc["hts_area"] = [jsonable(r) for r in classes.hts_area_table()]
        if args.stray:
            doc["stray_field"] = [jsonable(r) for r in classes.stray_field_report()]
        if args.iso13855:
            doc["iso13855"] = classes.iso13855_distances()
        _emit_json(doc, args.json_path)
        return 0
    print(_BANNER)
    print(f"{'class':10s} {'badge':5s} {'maturity':22s} {'mass':>10s}  {'regime':17s} {'safe state':19s} seismic action")
    for p in ps:
        print(
            f"{p.name:10s} {p.badge_symbol:5s} {p.maturity:22s} {p.design_mass_kg / 1e3:>8,.0f} t  "
            f"{p.regime.name:17s} {p.safe_state:19s} {p.seismic_action}"
        )
    if args.admission:
        print("\nAdmission (EXOTIC §X2; A = k·m·g/(η_fill·min(B_pole, B_rail, B_gap,max)²/2μ0), k 1.5, η 0.45, rail 1.6 T, gap 1.2 T):")
        print(classes.render_admission_table())
    if args.hts:
        print("\nHTS area at 10 mm (EXOTIC §X3; 53 kPa, ATZ cryostat-250 units):")
        print(classes.render_hts_area_table())
    if args.stray:
        print("\nStray field at the accessible surface (FAB-EXO-104; bare magnets, no iron return path, no FEM):")
        for r in classes.stray_field_report():
            flag = "shroud required" if r.shroud_required else "meets target"
            print(f"  {r.name:10s} {r.b_accessible_t * 1e3:8.2f} mT at {r.accessible_m * 100:.0f} cm; ≤ {r.target_t * 1e3:.1f} mT beyond {r.distance_to_target_m:.2f} m: {flag}")
    if args.iso13855:
        print("\nISO 13855 separation S = K·(t1 + v/a_brake) + C (checked against numeric limits from ISO 13855 (2010) only):")
        for r in classes.iso13855_distances():
            print(f"  {r['class']:10s} v {r['v_max_mps']:.3f} m/s, a_brake {r['a_brake_mps2']:.3f} m/s², K {r['k_mps']:.1f} m/s, t1 {r['t1_s']:.2f} s, C {r['c_m']:.2f} m -> S {r['s_m']:.2f} m")
    if args.jog:
        print("\nAttended jog at rated mass (imported pedestrian energy cap; momentum stated, not capped):")
        for p in ps:
            if p.regime.kind == "GZ" and p.regime.jog_v_mps:
                j = classes.attended_jog(p)
                print(f"  {p.name:10s} v_jog {j['v_jog_mps'] * 1e3:.2f} mm/s, momentum {j['momentum_kgmps']:.0f} kg·m/s, peak edge force {j['f_edge_peak_n']:.0f} N")
    return 0


# ---- scenarios / simulate ----------------------------------------------------------------


def _cmd_scenarios(args: argparse.Namespace) -> int:
    from . import scenarios

    cat = scenarios.catalog()
    if args.json:
        from ..export import to_jsonable as jsonable

        _emit_json({"schema": "fabrail.exo_scenarios/1", "scenarios": [jsonable(s) for s in cat]}, args.json_path)
        return 0
    for s in cat:
        print(f"{s.name:9s} {s.input}")
        print(f"          pass: {s.criteria}")
        print(f"          classes: {', '.join(s.classes)}; requirements: {', '.join(s.requirements) or '—'}")
    return 0


def _cmd_simulate(args: argparse.Namespace) -> int:
    from . import scenarios

    run = scenarios.simulate(args.exo_class, args.scenario, args.seed, _parse_sets(args.set), channels=True)
    print(run.summary())
    out_dir = Path(args.output) / "exo"
    if not args.no_plots:
        from .plots import plot_run

        fig = plot_run(run)
        out_dir.mkdir(parents=True, exist_ok=True)
        png = out_dir / f"{run.rail_class}_{run.scenario.lower()}.png"
        fig.savefig(png)
        print(f"wrote {png}")
    if args.json_path is not None:
        from ..export import payload_sha256, write_json

        doc = run.to_record(include_channels=True)
        doc["sha256"] = payload_sha256(doc)
        print(f"wrote {write_json(args.json_path, doc)}")
    return 1 if (args.strict and not run.passed) else 0


# ---- study -------------------------------------------------------------------------------


def _cmd_study(args: argparse.Namespace) -> int:
    from ..config import RunConfig
    from .studies import discover, run_study

    studies = discover()
    if args.list_studies:
        for name, spec in studies.items():
            print(f"  {name:26s} tier {spec.tier}  {spec.question}")
        return 0
    name = args.name or "all"
    if name == "all":
        names = [n for n, s in studies.items() if args.tier is None or s.tier == args.tier]
    elif name in studies:
        names = [name]
    else:
        print(f"fabrail exo study: unknown study {name!r} (available: {', '.join(studies)})", file=sys.stderr)
        return 1
    cfg = RunConfig(seed=args.seed, output=Path(args.output), plots=not args.no_plots)
    for n in names:
        paths = run_study(n, cfg, quick=args.quick)
        print(f"  {n}: " + ", ".join(str(p) for p in paths))
    print(f"completed {len(names)} exotic studies")
    return 0


# ---- pfh ---------------------------------------------------------------------------------


def _cmd_pfh(args: argparse.Namespace) -> int:
    from . import pfh

    p = pfh.PfhInputs(args.lambda_d, args.dc, args.beta, args.beta_d, args.t1, args.mttr, args.mrt)
    r = pfh.pfh_2oo3(p)
    if args.json:
        _emit_json(r.as_dict(), args.json_path)
        return 0
    print("2oo3 PFH (EXOTIC §X13). Placeholder rates, no FMEDA; designed toward, not certified.")
    print(f"inputs: λ_D {p.lambda_d_per_h:g}/h, DC {p.dc:g}, β {p.beta:g}, β_D {p.beta_d:g}, T1 {p.t1_h:g} h, MTTR {p.mttr_h:g} h, MRT {p.mrt_h:g} h")
    print(f"t_CE                      {r.t_ce_h:.4g} h")
    print(f"conservative form         {r.conservative:.4g} /h   (CTMC {r.ctmc:.4g}; numpy CTMC {r.py_ctmc:.4g})   {pfh.sil_target_band(r.conservative)}")
    ed2_ctmc = "—" if r.ctmc_ed2 is None else f"{r.ctmc_ed2:.4g}"
    print(f"IEC 61508-6 Ed. 2 form    {r.ed2:.4g} /h   (CTMC {ed2_ctmc}; numpy CTMC {r.py_ctmc_ed2:.4g})   {pfh.sil_target_band(r.ed2)}")
    print(f"the forms differ by {r.ratio_forms:.1f}x: always state the edition. A value in a band is a numeric band only, not a SIL.")
    return 0


# ---- ride-through --------------------------------------------------------------------------


def _cmd_ride_through(args: argparse.Namespace) -> int:
    from . import energy

    r = energy.ride_through(args.mass_kg, args.suspension, a_rms_g=args.a_rms_g, t_shaking_s=args.shaking_s, t_land_s=args.land_s, margin=args.margin)
    if args.json:
        from ..export import to_jsonable as jsonable

        _emit_json(jsonable(r), args.json_path)
        return 0
    print("Ride-through (EXOTIC §X4): T_ride = max(300 s, shaking) + landing; E = P_hold·T_ride")
    print("inputs: " + ", ".join(f"{k}={v}" for k, v in r.inputs.items()))
    print(f"P_hold {r.p_hold_w / 1e3:.4g} kW, T_ride {r.t_ride_s:.0f} s, E {r.e_ride_j / 1e6:.4g} MJ, with {args.margin:g}x margin {r.e_with_margin_j / 1e6:.4g} MJ")
    print(f"verdict: {r.verdict}")
    return 0


# ---- eew -----------------------------------------------------------------------------------


def _cmd_eew(args: argparse.Namespace) -> int:
    from . import eew

    w = eew.warning(args.site_km, args.sensor_km, args.depth_km, latency_s=args.latency_s, t_act_s=args.t_act_s)
    g = eew.gains()
    if args.json:
        from ..export import to_jsonable as jsonable

        _emit_json({"warning": jsonable(w), "gains": g, "note": "synthetic timing; fabrail is not an EEW provider"}, args.json_path)
        return 0
    print("EEW timing (EXOTIC §X10). Synthetic; fabrail is not an EEW provider.")
    print("inputs: " + ", ".join(f"{k}={v:g}" for k, v in w.inputs.items()))
    print(f"alert at {w.t_alert_s:.2f} s after origin; S at the site {w.t_s_site_s:.2f} s; warning {w.warning_s:.2f} s; blind zone {w.blind_zone_km:.1f} km")
    print(f"gain per km of sensor advance: {g['gain_p_s_per_km']:.5f} s (P, V_p {g['v_p_kmps']:g}), {g['gain_plum_s_per_km']:.5f} s (PLUM, V_s {g['v_s_kmps']:g})")
    return 0


# ---- blast ---------------------------------------------------------------------------------


def _cmd_blast(args: argparse.Namespace) -> int:
    from . import pressure

    d = pressure.closure_decision(args.t_a, t_detect_s=args.t_detect, zone_clear=not args.zone_occupied)
    out: dict[str, Any] = {
        "decision": {"close": d.close, "t_close_s": d.t_close_s, "reason": d.reason, "t_available_s": d.t_available_s},
        "inputs": d.inputs,
        "arrival_bound": (
            f"t_a ≤ R/c0 (the front is never slower than c0 = {d.inputs['c0_mps']:g} m/s): an arrival at t_a = {args.t_a:g} s "
            f"implies a source at R ≥ c0·t_a = {d.arrival_bound_range_m:.0f} m; t_a is an input, never derived from R"
        ),
    }
    if args.ps_kpa is not None:
        ps_pa = args.ps_kpa * 1e3
        out["friedlander"] = {
            "p_s_pa": ps_pa,
            "t_d_s": args.td_ms * 1e-3,
            "b": args.b,
            "impulse_pa_s": pressure.friedlander_impulse(ps_pa, args.td_ms * 1e-3, args.b),
            "reflected_pa": pressure.reflected_pressure(ps_pa),
            "reflection_factor": pressure.reflection_factor(ps_pa),
            "shock_speed_mps": pressure.shock_speed(ps_pa),
        }
    if args.json:
        _emit_json(out, args.json_path)
        return 0
    print("Closure race (EXOTIC §X11). Generic closed forms; no yield-to-distance table or scaled-distance law; not a protective-design tool.")
    print("inputs: " + ", ".join(f"{k}={v}" for k, v in d.inputs.items()))
    verdict = f"CLOSE (done at {d.t_close_s:.3f} s incl. seal)" if d.close else f"do not close: {d.reason}; take the configured fallback"
    print(f"decision: {verdict}; available {d.t_available_s:.3f} s")
    print(out["arrival_bound"])
    if "friedlander" in out:
        f = out["friedlander"]
        print(f"Friedlander: impulse {f['impulse_pa_s']:.4g} Pa·s; reflected {f['reflected_pa'] / 1e3:.4g} kPa (factor {f['reflection_factor']:.4g}); front speed {f['shock_speed_mps']:.1f} m/s")
    return 0


# ---- registration ------------------------------------------------------------------------


def _json_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--json", action="store_true", help="print (or with --json-out write) JSON instead of text")
    p.add_argument("--json-out", dest="json_path", type=Path, default=None, metavar="PATH", help="write the JSON to PATH (implies --json)")


def _fix_json(handler: Any) -> Any:
    def wrapped(args: argparse.Namespace) -> int:
        if getattr(args, "json_path", None) is not None and hasattr(args, "json"):
            args.json = True
        return handler(args)

    return wrapped


def register(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    """Add the ``exo`` subcommand to ``subparsers`` and return its parser."""
    parser = subparsers.add_parser(
        "exo",
        help="Sovereign/Exotic tier: classes, scenarios, simulations and studies",
        description=(
            "Sovereign/Exotic tier (docs/EXOTIC.md). Research-tier classes on reduced models; every alert is "
            "synthetic/injected, not a live EEW feed; frontier outputs carry the 🔭 banner; nothing is certified."
        ),
    )

    def show_help(args: argparse.Namespace) -> int:
        parser.print_help()
        return 0

    parser.set_defaults(handler=show_help)
    sub = parser.add_subparsers(dest="exo_command", metavar="EXO_COMMAND")

    p = sub.add_parser("classes", help="exotic class table (badge, maturity, regime, safe state) and class algebra")
    _json_args(p)
    p.add_argument("--admission", action="store_true", help="also the pole-area admission table (EXOTIC §X2)")
    p.add_argument("--hts", action="store_true", help="also the HTS area and logistics table (EXOTIC §X3)")
    p.add_argument("--stray", action="store_true", help="also the stray field at the accessible surface (FAB-EXO-104)")
    p.add_argument("--iso13855", action="store_true", help="also the ISO 13855 separation distances (EXOTIC §X14)")
    p.add_argument("--jog", action="store_true", help="also the attended-jog limits (EXOTIC §X14)")
    p.set_defaults(handler=_fix_json(_cmd_classes))

    p = sub.add_parser("scenarios", help="exotic scenario catalog (XS/XC) with pass criteria")
    _json_args(p)
    p.set_defaults(handler=_fix_json(_cmd_scenarios))

    p = sub.add_parser("simulate", help="run one exotic scenario (Rust reduced model)")
    p.add_argument("--class", dest="exo_class", required=True, metavar="NAME", help="exotic class, e.g. sentinel")
    p.add_argument("--scenario", required=True, metavar="ID", help="scenario id, e.g. XS12")
    p.add_argument("-s", "--seed", type=int, default=42, help="master seed (default: 42)")
    p.add_argument("--set", action="append", default=None, metavar="NAME=VALUE", help="scenario input override (repeatable; see 'exo scenarios')")
    p.add_argument("-o", "--output", type=Path, default=Path("output"), help="output directory (default: ./output)")
    p.add_argument("--json", dest="json_path", type=Path, default=None, metavar="PATH", help="write the result as JSON (fabrail.exo_run/1, with channels)")
    p.add_argument("--no-plots", action="store_true", help="skip figures")
    p.add_argument("--strict", action="store_true", help="exit 1 when a pass criterion fails")
    p.set_defaults(handler=_cmd_simulate)

    p = sub.add_parser("study", help="exotic and frontier studies (output/studies/<name>.{png,json})")
    p.add_argument("name", nargs="?", default=None, help="study name, or 'all'")
    p.add_argument("--list", dest="list_studies", action="store_true", help="list studies and exit")
    p.add_argument("--tier", type=int, choices=(1, 2), default=None, help="with 'all': only this tier")
    p.add_argument("--quick", action="store_true", help="reduced grids and durations")
    p.add_argument("-s", "--seed", type=int, default=42, help="master seed (default: 42)")
    p.add_argument("-o", "--output", type=Path, default=Path("output"), help="output directory (default: ./output)")
    p.add_argument("--no-plots", action="store_true", help="skip figures (JSON only)")
    p.set_defaults(handler=_cmd_study)

    p = sub.add_parser("pfh", help="PFH calculator (both forms, CTMC cross-check) for the 2oo3 architecture")
    p.add_argument("--lambda-d", type=float, default=1e-6, metavar="PER_H", help="dangerous failure rate λ_D, 1/h (default 1e-6)")
    p.add_argument("--dc", type=float, default=0.99, help="diagnostic coverage (default 0.99)")
    p.add_argument("--beta", type=float, default=0.02, help="common-cause factor β (default 0.02)")
    p.add_argument("--beta-d", type=float, default=0.01, help="common-cause factor of detected failures β_D (default 0.01)")
    p.add_argument("--t1", type=float, default=8760.0, metavar="H", help="proof-test interval T1, h (default 8760)")
    p.add_argument("--mttr", type=float, default=8.0, metavar="H", help="mean time to restoration, h (default 8)")
    p.add_argument("--mrt", type=float, default=8.0, metavar="H", help="mean repair time, h (default 8)")
    _json_args(p)
    p.set_defaults(handler=_fix_json(_cmd_pfh))

    p = sub.add_parser("ride-through", help="ride-through energy calculator (EXOTIC §X4)")
    p.add_argument("--mass-kg", type=float, default=1.0e7, help="suspended mass, kg (default 1e7 = 10 kt)")
    p.add_argument("--suspension", choices=("hybrid", "pure", "hts"), default="hybrid", help="suspension kind (default hybrid)")
    p.add_argument("--a-rms-g", type=float, default=0.2, help="RMS vertical acceleration at the suspension, g (default 0.2)")
    p.add_argument("--shaking-s", type=float, default=300.0, help="shaking duration of the class scenario, s (default 300)")
    p.add_argument("--land-s", type=float, default=0.0, help="landing time, s (default 0)")
    p.add_argument("--margin", type=float, default=2.0, help="discharge margin (default 2)")
    _json_args(p)
    p.set_defaults(handler=_fix_json(_cmd_ride_through))

    p = sub.add_parser("eew", help="EEW warning time for a site and a sensor distance (synthetic timing)")
    p.add_argument("--site-km", type=float, default=120.0, help="site epicentral distance, km (default 120)")
    p.add_argument("--sensor-km", type=float, default=34.0, help="k-th sensor epicentral distance, km (default 34)")
    p.add_argument("--depth-km", type=float, default=20.0, help="focal depth, km (default 20)")
    p.add_argument("--latency-s", type=float, default=3.3, help="τ_win + t_pkt + t_tel + t_proc + t_diss, s (default 3.3)")
    p.add_argument("--t-act-s", type=float, default=0.0, help="actuation time, s (default 0)")
    _json_args(p)
    p.set_defaults(handler=_fix_json(_cmd_eew))

    p = sub.add_parser("blast", help="closure race against a given blast arrival time t_a (EXOTIC §X11)")
    p.add_argument("--t-a", type=float, required=True, metavar="S", help="blast arrival time after detection, s (an input, never derived)")
    p.add_argument("--t-detect", type=float, default=0.0, metavar="S", help="detection delay, s (default 0)")
    p.add_argument("--zone-occupied", action="store_true", help="the guarded zone is not clear")
    p.add_argument("--ps-kpa", type=float, default=None, help="optional peak overpressure P_s, kPa (Friedlander and reflection)")
    p.add_argument("--td-ms", type=float, default=10.0, help="positive-phase duration t_d, ms (default 10)")
    p.add_argument("--b", type=float, default=1.0, help="Friedlander decay parameter b (default 1)")
    _json_args(p)
    p.set_defaults(handler=_fix_json(_cmd_blast))
    return parser
