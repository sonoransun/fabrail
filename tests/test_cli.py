"""Contract: the ``python -m fabrail`` command line (core design §B.9; FAB-SW-001).

What this file checks:

- Every subcommand of FAB-SW-001 (presets, params, scenarios, simulate, compare,
  ensemble, seismic, detect, study, anchors, trace, selftest, data, exo, example) is
  registered and prints its ``--help`` with exit status 0.
- ``simulate`` writes PNG figures and a ``fabrail.run/1`` JSON record whose ``sha256``
  seals everything but ``timing``; the record's ``analysis`` block holds every number
  its figures draw (FAB-SW-009 rule, checked with ``export.missing_plotted`` on figures
  re-drawn from the JSON alone), and the same seed gives the same hash.
- ``presets`` prints the class table from ``Params.derived()`` and ``--json`` records
  those values exactly; ``params --markdown`` is ``config.parameters_markdown()``.
- ``trace`` and ``--help`` work in a process where ``fabrail._native`` cannot be
  imported, and a native-only subcommand fails there with exit 1 and the build hint.
- ``seismic`` (P+S synthesis, synthetic alert and cancel, record file, rejected flag
  combinations), ``detect``, ``ensemble``, ``compare``, ``anchors``, ``selftest`` and
  ``data fetch`` produce their documents; the analysis reducers are exact on synthetic
  runs; the state and fault name tables equal the Rust registries; the analysis channel
  list equals the recorder's default set plus the documented extras.

Requirement row (docs/SPECIFICATION.md §3.9): FAB-SW-001 (subcommands_help). The other
tests are the core §E.3 CLI catalogue (simulate_json, presets_table, params_markdown,
trace_without_native) and B1 additions; SPEC lists no token for them, so they carry no
req marker. Every test keeps to short durations or Travel1d so it stays under the 15 s
per-test limit (plan §2b item 13).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from fabrail import analysis, export
from fabrail.__main__ import main, make_parser
from fabrail.results import EnsembleResult, RunEvent, RunResult

#: FAB-SW-001: the subcommands the CLI shall provide.
SW001_SUBCOMMANDS = (
    "presets",
    "params",
    "scenarios",
    "simulate",
    "compare",
    "ensemble",
    "seismic",
    "detect",
    "study",
    "anchors",
    "trace",
    "selftest",
    "data",
    "exo",
    "example",
)

ROOT = Path(__file__).resolve().parents[1]


def _run_cli(capsys: pytest.CaptureFixture[str], *argv: object) -> tuple[int, str, str]:
    rc = main([str(a) for a in argv])
    out, err = capsys.readouterr()
    return rc, out, err


def _load(path: Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _replot_missing(doc: dict) -> list[str]:
    """Re-draw every figure of a run record from its JSON ``analysis`` block; list unrecorded plotted values."""
    from fabrail import visualization as viz

    plotters: dict[str, Any] = {
        "levitation": viz.plot_levitation,
        "haptics": viz.plot_haptics,
        "safety": viz.plot_safety,
        "seismic": viz.plot_seismic,
    }
    missing: list[str] = []
    for kind, data in doc["analysis"].items():
        a = analysis.analysis_from_jsonable(kind, data)
        if a.t_s.size == 0:
            continue
        fig = plotters[kind](a)
        missing += [f"{kind}: {m}" for m in export.missing_plotted(fig, doc, scales=viz.PLOT_SCALES)]
    return missing


# ---- FAB-SW-001 ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-SW-001")
def test_subcommands_help(capsys: pytest.CaptureFixture[str]) -> None:
    """Every FAB-SW-001 subcommand is registered and its ``--help`` exits 0 with its usage line.

    Tolerance (E): the registered set equals the SPEC list exactly; each help run exits 0.
    Source: docs/SPECIFICATION.md FAB-SW-001; core design §B.9.
    """
    parser = make_parser()
    sub = next(a for a in parser._actions if isinstance(a, argparse._SubParsersAction))
    registered = set(sub.choices)
    assert registered == set(SW001_SUBCOMMANDS), (
        f"(E) subcommand set must equal FAB-SW-001: missing {sorted(set(SW001_SUBCOMMANDS) - registered)}, "
        f"extra {sorted(registered - set(SW001_SUBCOMMANDS))}"
    )
    rc, out, _ = _run_cli(capsys)
    assert rc == 0 and all(name in out for name in SW001_SUBCOMMANDS), (
        "(E) bare `fabrail` prints the help listing every subcommand"
    )
    for name in SW001_SUBCOMMANDS:
        with pytest.raises(SystemExit) as exc:
            main([name, "--help"])
        out, err = capsys.readouterr()
        assert exc.value.code == 0, f"(E) `fabrail {name} --help` must exit 0 (got {exc.value.code}; stderr: {err[:300]})"
        assert f"usage: fabrail {name}" in out, f"(E) `fabrail {name} --help` must print its usage line (FAB-SW-001)"


# ---- core §E.3 CLI catalogue --------------------------------------------------------------------


def test_simulate_json(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """``simulate`` writes PNGs and a sealed fabrail.run/1 record holding every plotted number; same seed, same hash.

    Uses the DOOR log-damped terminus (Carriage6, voltage coil mode) so the haptics figure
    carries the semilog panel. Tolerances: (E) schema, seal and file set; (E) every plotted
    value found in the record at rel 1e-12 (export.missing_plotted, FAB-SW-009); (E)
    identical sha256 for two runs of seed 42 (FAB-SW-002 rule, wall_s excluded); (P) the
    recorded-sample semilog τ within 1 % of the Rust base-rate KPI (PHYSICS §16.1).
    """
    out1, out2 = tmp_path / "a", tmp_path / "b"
    args = ["simulate", "-c", "door", "--scenario", "terminus_soft_close", "--coil", "voltage"]
    rc, out, err = _run_cli(capsys, *args, "-o", out1, "--json", out1 / "door.json")
    assert rc == 0, f"(E) simulate must exit 0 (stderr: {err[-500:]})"
    doc = _load(out1 / "door.json")
    assert doc["schema"] == export.RUN_SCHEMA, "(E) the record is fabrail.run/1 (core §B.8)"
    assert doc["sha256"] == export.payload_sha256(doc), "(E) sha256 seals the record without timing/wall_s (plan §2b item 16)"
    assert "wall_s" in doc["timing"] and "wall_s" not in doc["meta"], "(E) wall_s lives only in the unhashed timing block"
    assert set(doc["analysis"]) == {"levitation", "haptics", "safety", "seismic"}, "(E) every per-run analysis is recorded"
    assert doc["config"]["seed"] == 42 and doc["config"]["coil"] == "voltage", "(E) the config block records seed and coil mode"
    expected = {f"door_terminus_soft_close_{k}.png" for k in ("levitation", "haptics", "safety")}
    assert set(doc["figures"]) == expected, f"(E) figure list {doc['figures']} != {sorted(expected)}"
    for name in expected:
        assert (out1 / name).stat().st_size > 10_000, f"(E) {name} must be a written PNG"
    missing = _replot_missing(doc)
    assert missing == [], f"(E) every plotted number must be in the JSON record (FAB-SW-009); missing: {missing[:5]}"

    rc2, _, err2 = _run_cli(capsys, *args, "-o", out2, "--json", out2 / "door.json")
    assert rc2 == 0, f"(E) second simulate must exit 0 (stderr: {err2[-300:]})"
    assert _load(out2 / "door.json")["sha256"] == doc["sha256"], (
        "(E) seed 42 twice must give the same record hash (FAB-SW-002 rule)"
    )

    hap = doc["analysis"]["haptics"]
    tau_kpi = doc["kpis"]["terminus_tau_fit_s"]
    tau_rec = -1.0 / hap["terminus_fit_slope_per_s"]
    assert abs(tau_rec - tau_kpi) <= 0.01 * tau_kpi, (
        f"(P) recorded-sample semilog τ {tau_rec:.5f} s vs base-rate KPI {tau_kpi:.5f} s must agree within 1 % (PHYSICS §16.1)"
    )
    assert hap["terminus_fit_source"] == "kpi" and hap["terminus_tau_fit_s"] == tau_kpi, (
        "(E) the analysis reports the KPI when present"
    )


def test_presets_table(tmp_path: Path, capsys: pytest.CaptureFixture[str], native) -> None:
    """``presets`` prints every class with values from ``Params.derived()``; ``--json`` records them exactly.

    Tolerance (E): printed pole values equal the 5-significant-digit rendering of the
    native derived value; JSON derived dicts equal ``Params.preset(c).derived()``
    bit for bit. Source: core §B.9 (presets), fabrail_params::derive.
    """
    path = tmp_path / "presets.json"
    rc, out, err = _run_cli(capsys, "presets", "--json", path)
    assert rc == 0, f"(E) presets must exit 0 (stderr: {err[-300:]})"
    header = out.splitlines()[0].split()
    assert header == ["DRAWER", "DOOR", "WALL", "HEAVY"], f"(E) one column per class in canonical order (got {header})"
    pole_line = next(line for line in out.splitlines() if line.startswith("open-loop pole p"))
    cells = pole_line.split()[-4:]
    for cls, cell in zip(("drawer", "door", "wall", "heavy"), cells, strict=True):
        want = f"{native.Params.preset(cls).derived()['pole_rad_s']:.5g}"
        assert cell == want, f"(E) {cls} pole printed {cell} != derived {want} (PHYSICS §3; core §E.5 69.49/47.57/32.50 rad/s)"
    doc = _load(path)
    assert doc["schema"] == "fabrail.presets/1" and doc["sha256"] == export.payload_sha256(doc), "(E) sealed presets document"
    for cls in ("drawer", "door", "wall", "heavy"):
        derived = {k: float(v) for k, v in native.Params.preset(cls).derived().items()}
        assert doc["classes"][cls]["derived"] == derived, f"(E) {cls} derived values recorded exactly"
    rc, out, _ = _run_cli(capsys, "presets", "-c", "wall")
    assert rc == 0 and out.splitlines()[0].split() == ["WALL"], "(E) -c restricts the table to one class"


def test_params_markdown(capsys: pytest.CaptureFixture[str], native) -> None:
    """``params --markdown`` prints ``config.parameters_markdown()`` with every described parameter.

    Tolerance (E): byte-identical to the generator; every ``Params.describe()`` path
    appears as a code span. Source: plan §2b item 16 (frozen generator), core §B.9.
    """
    from fabrail.config import parameters_markdown

    rc, out, _ = _run_cli(capsys, "params", "--markdown")
    assert rc == 0, "(E) params --markdown must exit 0"
    assert out == parameters_markdown(), "(E) the CLI output must equal config.parameters_markdown() byte for byte"
    assert out.startswith("# fabrail parameters"), "(E) docs/PARAMETERS.md heading"
    paths = [str(r["path"]) for r in native.Params.describe()]
    absent = [p for p in paths if f"`{p}`" not in out]
    assert not absent, f"(E) every described parameter must be listed; absent: {absent[:5]}"


def test_trace_without_native(tmp_path: Path) -> None:
    """``trace`` and ``--help`` run where ``fabrail._native`` cannot be imported; ``simulate`` fails cleanly there.

    A subprocess blocks the extension (``sys.modules['fabrail._native'] = None``).
    Tolerance (E): trace exits 0 or 1 (1 only while listed tests are missing) and prints
    the fabrail.trace JSON report; simulate exits 1 with the maturin build hint.
    Source: core §B.8 (trace needs no native module), plan §2b item 16 (lazy _native).
    """
    code = (
        "import sys, json\n"
        "sys.modules['fabrail._native'] = None\n"
        "from fabrail.__main__ import main\n"
        "import io, contextlib\n"
        "buf = io.StringIO()\n"
        "with contextlib.redirect_stdout(buf):\n"
        "    rc_trace = main(['trace', '--json', '--allow-missing'])\n"
        "report = json.loads(buf.getvalue())\n"
        "try:\n"
        "    main(['--help'])\n"
        "except SystemExit as e:\n"
        "    rc_help = e.code\n"
        "rc_sim = main(['simulate', '--scenario', 'hover', '-o', sys.argv[1], '--no-plots'])\n"
        "print(json.dumps({'rc_trace': rc_trace, 'rc_help': rc_help, 'rc_sim': rc_sim,\n"
        "                  'keys': sorted(report), 'total': report['total']}), file=sys.stderr)\n"
    )
    env = {**os.environ, "PYTHONPATH": str(ROOT / "python"), "MPLBACKEND": "Agg"}
    proc = subprocess.run(
        [sys.executable, "-c", code, str(tmp_path)], cwd=ROOT, env=env, capture_output=True, text=True, timeout=60
    )
    assert proc.returncode == 0, f"(E) the probe process must finish (stderr: {proc.stderr[-800:]})"
    lines = [line for line in proc.stderr.splitlines() if line.startswith("{")]
    assert lines, f"(E) the probe must report its results (stderr: {proc.stderr[-800:]})"
    res = json.loads(lines[-1])
    assert res["rc_trace"] in (0, 1), f"(E) trace must run without _native (exit {res['rc_trace']})"
    assert {"ok", "total", "tokens", "structure_ok"} <= set(res["keys"]) and res["total"] > 200, (
        "(E) trace prints its JSON report"
    )
    assert res["rc_help"] == 0, "(E) --help works without _native"
    assert res["rc_sim"] == 1, f"(E) simulate without _native must exit 1 (got {res['rc_sim']})"
    assert "maturin develop --release" in proc.stderr, "(E) the failure names the build command (fabrail.require_native hint)"


# ---- B1 additions ---------------------------------------------------------------------------------


def _rust_enum(text: str, name: str) -> list[str]:
    block = re.search(r"pub enum " + name + r" \{(.*?)\n    \}", text, re.S)
    assert block, f"enum {name} not found in the Rust source"
    return [m.group(1) for m in re.finditer(r"^\s+([A-Z][A-Za-z0-9]*) = (\d+),", block.group(1), re.M)]


def test_state_tables_match_rust() -> None:
    """The analysis name tables equal the ``u8_enum`` registries in crates/fabrail-types.

    Tolerance (E): identical names in identical code order; every FaultCode wire value
    and severity equal. Source: crates/fabrail-types/src/{state,fault}.rs.
    """
    state = (ROOT / "crates/fabrail-types/src/state.rs").read_text(encoding="utf-8")
    for rust_name, table in (
        ("LevState", analysis.LEV_STATE_NAMES),
        ("SeismicState", analysis.SEISMIC_STATE_NAMES),
        ("MotionMode", analysis.MOTION_MODE_NAMES),
        ("SafetyDemand", analysis.DEMAND_NAMES),
        ("SeismicLevel", analysis.LEVEL_NAMES),
        ("Severity", analysis.SEVERITY_NAMES),
    ):
        assert tuple(_rust_enum(state, rust_name)) == table, f"(E) analysis table for {rust_name} drifted from state.rs"
    fault = (ROOT / "crates/fabrail-types/src/fault.rs").read_text(encoding="utf-8")
    rust = {
        int(v, 16): (n, s) for n, v, s in re.findall(r"^\s+([A-Z][A-Za-z0-9]*) = (0x[0-9A-Fa-f]+) => ([A-Za-z]+),", fault, re.M)
    }
    ours = {c: (analysis.FAULT_NAMES[c], analysis.FAULT_SEVERITY[c]) for c in analysis.FAULT_NAMES}
    assert ours == rust, f"(E) fault table drifted from fault.rs: {sorted(set(rust.items()) ^ set(ours.items()))[:5]}"


def test_analysis_channels_match_native(native) -> None:
    """``analysis.default_channel_names`` equals the recorder's default set, and the extras are recordable.

    Tolerance (E): identical ordered names for DOOR (4 magnets) and HEAVY (8 magnets); a
    run recording the analysis set returns exactly those channels. Source:
    fabrail_sim::recorder::default_channels / EXTRA_CHANNELS.
    """
    from fabrail import scenarios
    from fabrail.config import RunConfig

    for cls in ("door", "heavy"):
        cfg = RunConfig(rail_class=cls, duration_s=0.01, coil="voltage", fidelity="heave1")
        params = cfg.params()
        spec, series = scenarios.spec("hover", params, cfg, fidelity="heave1")
        run = native.run(params, native.Scenario.from_json(scenarios.to_json(spec), series))
        n_mag = int(params.get("design.suspension.n_magnets"))
        assert run.channel_names() == analysis.default_channel_names(n_mag), f"(E) {cls} default channel list drifted"
        record: Any = spec["record"]
        record["channels"] = analysis.analysis_channel_names(n_mag)
        run = native.run(params, native.Scenario.from_json(scenarios.to_json(spec), series))
        assert run.channel_names() == analysis.analysis_channel_names(n_mag), (
            f"(E) {cls} analysis channels must all be recordable"
        )


def test_seismic_pwave_timeline(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """``seismic --ground pwave`` (WALL, M7 at 20 km, Travel1d) records the land-first timeline against t_P and t_S.

    Tolerance (E): trigger after t_P, PROTECT after the trigger, touchdown finite, level
    ≥ 2, the printed timeline in time order, plotted numbers recorded. This checks the CLI
    and analysis plumbing; the SEI-010 margins are asserted by tests/test_seismic_policy.py.
    Source: core §B.9 (seismic), plan §2b item 6.
    """
    path = tmp_path / "wall.json"
    rc, out, err = _run_cli(
        capsys,
        "seismic",
        "-c",
        "wall",
        "--ground",
        "pwave",
        "--magnitude",
        "7",
        "--distance-km",
        "20",
        "--coil",
        "voltage",
        "-o",
        tmp_path,
        "--json",
        path,
    )
    assert rc == 0, f"(E) seismic must exit 0 (stderr: {err[-400:]})"
    doc = _load(path)
    sei = doc["analysis"]["seismic"]
    assert doc["config"]["mode"] == "land" and doc["config"]["ground"] == "pwave", "(E) config records mode and ground"
    assert sei["t_p_s"] is not None and sei["trigger_t_s"] > sei["t_p_s"], "(E) the STA/LTA trigger follows the P arrival"
    assert sei["protect_t_s"] >= sei["trigger_t_s"] and sei["land_t_s"] is not None, "(E) PROTECT then touchdown"
    assert sei["level_max"] >= 2, f"(E) M7 at 20 km reaches at least L2 (got L{sei['level_max']})"
    times = [float(m.group(1)) for m in re.finditer(r"^\s+([0-9.]+) s  ", out, re.M)]
    assert times == sorted(times) and len(times) >= 5, "(E) the printed timeline is in time order"
    assert set(doc["figures"]) == {"wall_seismic_pwave_m7_r20_land_seismic.png", "wall_seismic_pwave_m7_r20_land_safety.png"}
    assert _replot_missing(doc) == [], "(E) every plotted number is in the record (FAB-SW-009)"


def test_seismic_alert_and_cancel(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A synthetic network alert and its cancellation land in the spec at origin + delay and origin + cancel.

    Tolerance (E): event times equal origin + 5 s and origin + 8 s exactly; the output
    says alerts are synthetic/injected. Source: core §B.9 (--network-alert-delay,
    --cancel-at); plan §7 (no live EEW claim).
    """
    path = tmp_path / "door.json"
    rc, out, err = _run_cli(
        capsys,
        "seismic",
        "-c",
        "door",
        "--network-alert-delay",
        "5",
        "--cancel-at",
        "8",
        "--distance-km",
        "60",
        "--no-plots",
        "-o",
        tmp_path,
        "--json",
        path,
    )
    assert rc == 0, f"(E) seismic with an alert must exit 0 (stderr: {err[-400:]})"
    doc = _load(path)
    from fabrail import scenarios

    origin = float(scenarios.info(doc["meta"]["spec"])["origin_t_s"])
    kinds = {e["kind"]: e["t_s"] for e in doc["meta"]["spec"]["events"]}
    assert kinds["network_alert"] == pytest.approx(origin + 5.0, rel=0, abs=1e-12), "(E) alert at origin + 5 s"
    assert kinds["alert_cancel"] == pytest.approx(origin + 8.0, rel=0, abs=1e-12), "(E) cancel at origin + 8 s"
    sei = doc["analysis"]["seismic"]
    assert sei["alert_t_s"] is not None and sei["alert_cancel_t_s"] is not None, "(E) alert and cancel reach the analysis"
    assert "synthetic/injected, never a live EEW feed" in out, "(E) the CLI states the alert is synthetic (plan §7)"


def test_seismic_record_file_and_rejections(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """``--ground file`` drives a controlled device (no 'manual' release); invalid flag combinations exit 1.

    Tolerance (E): spec name ``door_seismic_file``, no manual command, record SHA-256 in
    the config; ``--cancel-at`` without an alert, ``--magnitude`` with a record and
    isolation on a DOOR are rejected with exit 1. Source: core §B.9; params validate
    (isolation needs WALL/HEAVY, core §C.2).
    """
    fs = 100.0
    t = np.arange(int(4 * fs)) / fs
    rec = tmp_path / "rec.csv"
    rec.write_text("\n".join(f"{v:.9f}" for v in 0.05 * np.sin(2 * np.pi * 1.5 * t) * np.sin(np.pi * t / 4.0)) + "\n")
    path = tmp_path / "file.json"
    rc, _, err = _run_cli(
        capsys,
        "seismic",
        "-c",
        "door",
        "--ground",
        "file",
        "--record",
        rec,
        "--record-fs",
        fs,
        "--no-plots",
        "-o",
        tmp_path,
        "--json",
        path,
    )
    assert rc == 0, f"(E) seismic --ground file must exit 0 (stderr: {err[-400:]})"
    doc = _load(path)
    spec = doc["meta"]["spec"]
    assert spec["name"] == "door_seismic_file", f"(E) spec renamed for the controlled run (got {spec['name']})"
    assert not any(e.get("cmd") == "manual" for e in spec["events"]), "(E) the SEI-013 manual release is removed"
    assert len(doc["config"]["record_sha256"]) == 64, "(E) the record file hash is recorded"
    for argv in (
        ["--cancel-at", "8"],
        ["--ground", "kt", "--magnitude", "7"],
        ["--ground", "kt", "--mode", "isolate"],
    ):
        rc, _, err = _run_cli(capsys, "seismic", "-c", "door", "--no-plots", "-o", tmp_path, *argv)
        assert rc == 1 and err.strip(), f"(E) `seismic {' '.join(argv)}` must be rejected with exit 1 and a message"


def test_detect_csv(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """``detect`` replays a CSV record (time column, z,x,y order, units g) and records trigger and PROTECT.

    Synthetic M6.5 at R_epi 20 km (pwave.boore_ps, P at 35 s) plus site noise. Tolerance
    (E): a Trigger after t_P within 1 s, a PROTECT, the report's sample count and every
    plotted number recorded. Source: core §B.9 (detect), PHYSICS §20.
    """
    from fabrail import pwave

    rng = np.random.default_rng(7)
    ev = pwave.boore_ps(rng, 6.5, 20.0, 10.0, fs_hz=500.0, duration_s=50.0)
    a = ev.a_xyz_mps2 + pwave.site_noise(rng, 50.0, 500.0)
    g = 9.80665
    n = a.shape[1]
    table = np.column_stack([np.arange(n) / 500.0, a[2] / g, a[0] / g, a[1] / g])
    rec = tmp_path / "rec.csv"
    np.savetxt(rec, table, delimiter=",", header="t,z,x,y", comments="", fmt="%.10e")
    path = tmp_path / "det.json"
    rc, _, err = _run_cli(capsys, "detect", "--input", rec, "--columns", "z,x,y", "-c", "wall", "-o", tmp_path, "--json", path)
    assert rc == 0, f"(E) detect must exit 0 (stderr: {err[-400:]})"
    doc = _load(path)
    assert doc["schema"] == "fabrail.detect/1" and doc["sha256"] == export.payload_sha256(doc), "(E) sealed detect document"
    assert doc["report"]["samples"] == n, "(E) every sample reached the detector"
    trig = [e["t_s"] for e in doc["events"] if e["kind"] == "Trigger"]
    assert trig and ev.t_p_s < trig[0] <= ev.t_p_s + 1.0, f"(E) trigger {trig[:1]} within 1 s after t_P {ev.t_p_s} (PHYSICS §20)"
    assert any(e["kind"] == "Protect" for e in doc["events"]), "(E) M6.5 at 20 km reaches PROTECT"
    from fabrail import visualization as viz

    fig = viz.plot_seismic(analysis.analysis_from_jsonable("seismic", doc["analysis"]["seismic"]))
    assert export.missing_plotted(fig, doc, scales=viz.PLOT_SCALES) == [], "(E) plotted numbers recorded (FAB-SW-009)"


def test_ensemble_json(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """``ensemble`` writes a sealed fabrail.ensemble/1 record with the analysis and threshold statistics.

    Tolerance (E): n members in index order, P50/P95 equal numpy's linear quantiles of
    the recorded KPIs, the Wilson bound equals analysis.wilson_upper(k, n), and every
    plotted number is in the record. Source: core §B.9 (ensemble), core §E.1 (MC class).
    """
    path = tmp_path / "ens.json"
    rc, out, err = _run_cli(
        capsys,
        "ensemble",
        "-c",
        "door",
        "--scenario",
        "hover",
        "-n",
        "3",
        "--duration",
        "0.5",
        "--coil",
        "voltage",
        "--threshold",
        "hover_gap_rms_m=1e-6",
        "-o",
        tmp_path,
        "--json",
        path,
    )
    assert rc == 0, f"(E) ensemble must exit 0 (stderr: {err[-400:]})"
    doc = _load(path)
    assert doc["schema"] == export.ENSEMBLE_SCHEMA and doc["n_members"] == 3, "(E) fabrail.ensemble/1 with 3 members"
    assert doc["sha256"] == export.payload_sha256(doc), "(E) sealed record"
    ea = analysis.analysis_from_jsonable("ensemble", doc["analysis"]["ensemble"])
    x = np.asarray(doc["kpis"]["hover_gap_rms_m"], dtype=np.float64)
    assert ea.p95["hover_gap_rms_m"] == float(np.quantile(x, 0.95)), "(E) P95 is numpy's linear quantile of the recorded KPIs"
    k = int(np.count_nonzero(x > 1e-6))
    assert ea.exceedances["hover_gap_rms_m"] == k, "(E) exceedance count"
    assert ea.wilson_upper["hover_gap_rms_m"] == analysis.wilson_upper(k, 3), "(E) Wilson 95 % upper bound (MC class)"
    from fabrail import visualization as viz

    fig = viz.plot_ensemble(ea)
    assert export.missing_plotted(fig, doc, scales=viz.PLOT_SCALES) == [], "(E) plotted numbers recorded (FAB-SW-009)"
    assert (tmp_path / "door_hover_ensemble.png").is_file(), "(E) the ensemble figure is written"


def test_compare_json(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """``compare`` runs one scenario per class and records each run plus the haptic analyses it plots.

    Tolerance (E): one run record per requested class, a skipped class is named on stderr,
    and every plotted number of the comparison figure is in the document. Source: core §B.9.
    """
    path = tmp_path / "cmp.json"
    rc, _, err = _run_cli(
        capsys,
        "compare",
        "--scenario",
        "lateral_push",
        "--classes",
        "drawer,door,wall",
        "--duration",
        "0.5",
        "--coil",
        "voltage",
        "-o",
        tmp_path,
        "--json",
        path,
    )
    assert rc == 0, f"(E) compare must exit 0 (stderr: {err[-400:]})"
    assert "does not apply to drawer" in err, "(E) a class the scenario does not cover is skipped with a message"
    doc = _load(path)
    assert doc["schema"] == "fabrail.compare/1" and doc["classes"] == ["door", "wall"], f"(E) classes {doc['classes']}"
    assert set(doc["runs"]) == {"door", "wall"} and all(r["schema"] == export.RUN_SCHEMA for r in doc["runs"].values())
    from fabrail import visualization as viz

    hap = {c: analysis.analysis_from_jsonable("haptics", d) for c, d in doc["analysis"]["haptics"].items()}
    fig = viz.plot_class_comparison(hap)
    assert export.missing_plotted(fig, doc, scales=viz.PLOT_SCALES) == [], "(E) plotted numbers recorded (FAB-SW-009)"


def test_anchors_quick_roundtrip(tmp_path: Path, capsys: pytest.CaptureFixture[str], native) -> None:
    """``anchors --quick --write-tests --dest`` writes fabrail.anchors/1; ``--check`` of a fresh run passes.

    Tolerance (E): schema and seal; the selftest hash anchor equals the native hash
    split into 32-bit halves; a second computation is within the recorded (P)
    tolerances (exit 0). Source: core §E.5; export.anchors_record.
    """
    dest = tmp_path / "anchors.json"
    rc, _, err = _run_cli(capsys, "anchors", "--quick", "--write-tests", "--dest", dest)
    assert rc == 0, f"(E) anchors must exit 0 (stderr: {err[-400:]})"
    doc = _load(dest)
    assert doc["schema"] == export.ANCHORS_SCHEMA and doc["sha256"] == export.payload_sha256(doc), "(E) sealed anchors document"
    h = int(native.selftest_hash(2000))
    assert doc["anchors"]["golden.selftest_hash_2000_hi_lo"]["value"] == [h >> 32, h & 0xFFFFFFFF], "(E) selftest hash anchor"
    assert all(v.get("source") for v in doc["anchors"].values()), "(E) every anchor names its source (core §E.5)"
    rc, out, err = _run_cli(capsys, "anchors", "--quick", "--check", dest)
    assert rc == 0 and "0 outside tolerance" in out, (
        f"(E) a fresh computation must be within tolerance ({out[-300:]} {err[-300:]})"
    )


def test_selftest_quick(tmp_path: Path, capsys: pytest.CaptureFixture[str], native) -> None:
    """``selftest --quick`` reports the native selftest hash and a clean, repeatable DOOR hover smoke run.

    Tolerance (E): printed hash equals ``_native.selftest_hash(2000)``; the smoke case has
    no fault at SafeStop or above, monitor demand 0 and a bit-identical repeat. While
    GOLDEN_SELFTEST_HASH is the placeholder 0 the verdict says so. Source: core §B.9
    (selftest), plan §2b item 16 (voltage-mode SIL claim).
    """
    path = tmp_path / "st.json"
    rc, out, err = _run_cli(capsys, "selftest", "--quick", "--json", path)
    assert rc == 0, f"(E) selftest must pass (stdout: {out[-400:]} stderr: {err[-300:]})"
    doc = _load(path)
    assert doc["hash"] == int(native.selftest_hash(2000)) and f"{doc['hash']:#018x}" in out, "(E) hash printed and recorded"
    assert doc["ok"] and doc["smoke"][0]["repeat_identical"], "(E) clean, bit-identical DOOR hover smoke"
    if int(native.GOLDEN_SELFTEST_HASH) == 0:
        assert "not frozen yet" in doc["verdict"], "(E) the placeholder golden hash is reported, not compared"


def test_data_fetch_file_url(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """``data fetch elcentro --url file://…`` pins the file's SHA-256; without a URL it exits 1 naming --url.

    Tolerance (E): pinned hash equals hashlib's; the no-URL error mentions --url. Source:
    core §B.9 (data fetch), records.fetch_elcentro (trust on first use).
    """
    import hashlib

    src = tmp_path / "src" / "elcentro_copy.csv"
    src.parent.mkdir()
    src.write_text("t,x\n0,0\n0.01,0.1\n0.02,-0.1\n", encoding="utf-8")
    dest = tmp_path / "data"
    rc, out, _ = _run_cli(capsys, "data", "fetch", "elcentro", "--url", src.as_uri(), "--dest", dest)
    digest = hashlib.sha256(src.read_bytes()).hexdigest()
    assert rc == 0 and digest in out, "(E) the fetched file's SHA-256 is printed"
    assert digest in (dest / "SHA256SUMS").read_text(encoding="utf-8"), "(E) the hash is pinned on first use"
    env_saved = os.environ.pop("FABRAIL_ELCENTRO_URL", None)
    try:
        rc, _, err = _run_cli(capsys, "data", "fetch", "elcentro", "--dest", tmp_path / "none")
    finally:
        if env_saved is not None:
            os.environ["FABRAIL_ELCENTRO_URL"] = env_saved
    assert rc == 1 and "--url" in err, "(E) no URL: exit 1 and name the --url flag (records are never vendored)"


def test_errors_exit_1(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """Rejected input exits 1 with a one-line message: unknown scenario, wrong class, bad override path.

    Tolerance (E): exit status 1 and the offending name in stderr. Source: core §B.9;
    params ``ValueError("path: reason")``.
    """
    for argv, needle in (
        (["simulate", "--scenario", "nosuch"], "nosuch"),
        (["simulate", "-c", "heavy", "--scenario", "unseat"], "unseat"),
        (["simulate", "--scenario", "hover", "--set", "design.nosuch_m=1"], "design.nosuch_m"),
    ):
        rc, _, err = _run_cli(capsys, *argv, "-o", tmp_path, "--no-plots")
        assert rc == 1 and needle in err, f"(E) `{' '.join(argv)}` must exit 1 naming {needle!r} (stderr: {err[-300:]})"


# ---- analysis reducers on synthetic runs ----------------------------------------------------------


def _synthetic_terminus(tau: float = 0.15, d_b: float = 0.03214847275914768) -> RunResult:
    rate = 1000.0
    t = np.arange(3000, dtype=np.float64) / rate
    t_c = 0.5
    x = np.where(t < t_c, 0.05, d_b * np.exp(-(t - t_c) / tau))
    v = np.gradient(x, 1.0 / rate)
    gap = 0.0025 + 1e-6 * np.sin(2 * np.pi * 7.0 * t)
    ch = {
        "x_m": x,
        "x_ref_m": np.where(t < t_c, 0.05, d_b * np.exp(-(t - t_c) / tau)) * (t < 2.9),  # reaches the target 0 at arrival
        "v_mps": v,
        "a_mps2": np.gradient(v, 1.0 / rate),
        "floor_ax_mps2": np.zeros_like(t),
        "gap_0_m": gap,
        "gap_1_m": gap,
        "lev_state": np.full(t.size, 5, dtype=np.uint8),
        "motion_mode": np.full(t.size, 10, dtype=np.uint8),
    }
    events = [RunEvent(t_c, "TerminusCapture", d_b, tau), RunEvent(2.9, "Arrived", 1e-4, 0.0)]
    return RunResult(
        rate_hz=rate,
        t_s=t,
        channels=ch,
        units={},
        events=events,
        kpis={},
        meta={"scenario": "drawer_terminus_soft_close", "params": {"design": {"class": "drawer"}}},
    )


def test_analysis_terminus_and_psd_exact() -> None:
    """On an exact exponential capture the semilog fit returns τ, R² = 1; the Welch PSD peaks at the injected 7 Hz.

    Tolerance (E): τ within 1e-9 relative; R² ≥ 1 − 1e-12; PSD argmax within one Welch
    bin (fs/nperseg) of 7 Hz. Source: PHYSICS §16.1 (log-damped definition), scipy.signal.welch.
    """
    run = _synthetic_terminus()
    hap = analysis.analyze_haptics(run)
    assert hap.terminus_fit_source == "recorded", "(E) no KPI present, so the recorded-sample fit is reported"
    assert abs(hap.terminus_tau_fit_s - 0.15) <= 1e-9 * 0.15, f"(E) τ {hap.terminus_tau_fit_s!r} vs 0.15 s (rel 1e-9)"
    assert hap.terminus_semilog_r2 >= 1.0 - 1e-12, f"(E) R² {hap.terminus_semilog_r2!r} of an exact exponential"
    assert hap.terminus_d_b_m == run.events[0].a and hap.terminus_count == 1, "(E) capture payload read from the event"
    lev = analysis.analyze_levitation(run)
    df = lev.psd_f_hz[1] - lev.psd_f_hz[0]
    assert abs(lev.psd_f_hz[int(np.argmax(lev.gap_psd_m2_per_hz))] - 7.0) <= df, "(E) Welch PSD peak at 7 Hz within one bin"
    assert lev.gap_mean_m == pytest.approx(0.0025, rel=1e-6), "(E) mean gap over the levitated stretch"


def test_plot_decimation_keeps_peaks() -> None:
    """Decimated series keep every channel's extreme samples, so a plotted peak is a real recorded sample.

    Tolerance (E): the 2 ms force spike survives 50× decimation exactly. Source: this module's docstring rule.
    """
    rate = 1000.0
    t = np.arange(200_000, dtype=np.float64) / rate
    f = np.zeros_like(t)
    f[123_457] = 93.6
    run = RunResult(
        rate_hz=rate,
        t_s=t,
        channels={"f_obstacle_n": f, "e_kin_gov_j": np.zeros_like(t)},
        events=[],
        kpis={},
        meta={"scenario": "door_obstruction_chest"},
    )
    saf = analysis.analyze_safety(run, max_points=4000)
    assert saf.t_s.size <= 4003 and float(saf.contact_force_n.max()) == 93.6, "(E) the peak sample is kept"
    assert 123.457 in saf.t_s, "(E) at its own time stamp"


def test_ensemble_analysis_counts() -> None:
    """``analyze_ensemble`` counts strict exceedances over members that report the KPI and bounds them with Wilson.

    Tolerance (E): k and n exact; the bound equals wilson_upper(k, n). Source: core §E.1 (MC).
    """
    members = [RunResult(rate_hz=1.0, t_s=np.zeros(0), kpis=({"e": float(i)} if i != 2 else {})) for i in range(5)]
    ens = EnsembleResult.from_members(42, members, noise_seeds=[0] * 5, scenario="x")
    ea = analysis.analyze_ensemble(ens, {"e": 1.0})
    assert ea.n_reporting["e"] == 4 and ea.exceedances["e"] == 2, "(E) members 3 and 4 exceed 1.0; member 2 lacks the KPI"
    assert ea.wilson_upper["e"] == analysis.wilson_upper(2, 4), "(E) Wilson 95 % upper over the 4 reporting members"
    assert math.isclose(ea.p50["e"], float(np.quantile([0.0, 1.0, 3.0, 4.0], 0.5))), "(E) P50 over the finite values"
