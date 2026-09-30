"""Contract: the JSON export records every plotted number and round-trips (FAB-SW-009).

- ``fabrail.run/1``: ``run_record`` → canonical JSON text → ``run_from_record`` gives
  back the same channels (bit for bit, NaN included, uint8 state codes as uint8),
  events, KPIs and metadata, and re-exporting reproduces the document exactly.
- ``wall_s`` is kept in the unhashed ``timing`` block: the ``sha256`` of a record does
  not change with wall-clock time, and does change with any recorded number (plan §2b
  item 16).
- ``fabrail.ensemble/1``, ``fabrail.anchors/1`` and ``fabrail.study/1`` round-trip the
  same way; ``check_anchors`` passes exactly the values within tolerance.
- ``missing_plotted`` finds every plotted value of a house-style figure in its record,
  and is not vacuous: an unrecorded point is reported.

Requirement row (docs/SPECIFICATION.md §3): FAB-SW-009 (json_roundtrip,
every_plotted_number). The unmarked tests repeat the round trip on a native run and as
a hypothesis property.
"""

from __future__ import annotations

import json
import math

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from fabrail import export
from fabrail.results import EnsembleResult, RunEvent, RunResult


def _synthetic_run(seed: int = 0, wall_s: float = 1.25) -> RunResult:
    """A log-damped terminus approach (PHYSICS §16.1 shape) with awkward floats and a state channel."""
    rate = 1000.0
    t = np.arange(400, dtype=np.float64) / rate
    tau, d_b = 0.15, 0.03214847275914768
    x = 0.30 - d_b * np.exp(-t / tau)
    v = d_b / tau * np.exp(-t / tau)
    rng = np.random.default_rng(seed)
    noise = rng.normal(0.0, 1e-6, t.size)
    odd = noise.copy()
    odd[:6] = [math.nan, -0.0, 5e-324, 1.7976931348623157e308, -1e-310, 0.1 + 0.2]
    state = (np.arange(t.size) // 100).astype(np.uint8)
    return RunResult(
        rate_hz=rate,
        t_s=t,
        channels={"x_m": x, "v_mps": v, "residual_n": odd, "motion_mode": state},
        units={"x_m": "m", "v_mps": "m/s", "residual_n": "N", "motion_mode": ""},
        events=[RunEvent(0.0, "TerminusStart", d_b, tau), RunEvent(0.25, "Arrived", 1.2e-4, 0.0)],
        kpis={"terminus_tau_fit_s": 0.150000001, "terminus_r2_min": 0.99995, "overshoot_m": 0.0},
        meta={
            "version": "0.1.0",
            "scenario": "throw_soft_close",
            "noise_seed": 12345678901234567890 % (2**64),
            "fidelity": "carriage6",
            "coil": "voltage",
            "params": {"design": {"carriage": {"m_star_kg": 26.0}}, "safety": {"crc": 955749387}},
            "spec": {"name": "throw_soft_close", "duration_s": 0.4},
            "golden": 0,
            "build_profile": "release",
            "wall_s": wall_s,
        },
    )


def _same_bits(a: np.ndarray, b: np.ndarray) -> bool:
    return a.dtype == b.dtype and a.shape == b.shape and bytes(a.tobytes()) == bytes(b.tobytes())


@pytest.mark.req("FAB-SW-009")
def test_json_roundtrip(tmp_path):
    """run/ensemble/anchors/study records round-trip through canonical JSON text and files; wall_s is unhashed."""
    run = _synthetic_run()
    doc = export.run_record(run, include_channels=True)
    assert doc["schema"] == export.RUN_SCHEMA and "wall_s" not in doc["meta"] and doc["timing"] == {"wall_s": 1.25}
    text = export.dumps(doc)
    assert "NaN" not in text and "Infinity" not in text, "canonical JSON has no NaN/inf (non-finite → null)"
    path = export.save_run_json(run, tmp_path / "runs" / "run.json", include_channels=True)
    assert json.loads(path.read_text(encoding="utf-8")) == json.loads(text)
    back = export.run_from_record(export.load_json(path))
    for name, arr in run.channels.items():
        got = back.channels[name]
        if arr.dtype == np.uint8:
            assert _same_bits(got, arr), f"{name}: uint8 state codes must round-trip exactly"
        else:
            finite = np.isfinite(arr)
            assert np.array_equal(np.isnan(got), np.isnan(arr)), f"{name}: NaN positions (null in JSON)"
            assert _same_bits(got[finite], arr[finite]), f"{name}: float64 samples must round-trip bit for bit"
    assert math.copysign(1.0, back.channels["residual_n"][1]) == -1.0, "-0.0 keeps its sign"
    assert _same_bits(back.t_s, run.t_s) and back.rate_hz == run.rate_hz
    assert back.events == run.events and back.kpis == run.kpis and back.meta == run.meta and back.units == run.units
    assert export.run_record(back, include_channels=True) == json.loads(text), "re-export reproduces the document"

    # wall_s never enters the hash; any recorded number does.
    later = export.run_record(_synthetic_run(wall_s=99.0), include_channels=True)
    assert later["sha256"] == doc["sha256"] and later["timing"] != doc["timing"]
    assert export.dumps(export.stable_payload(later)) == export.dumps(export.stable_payload(doc)), "byte-identical payload"
    tweaked = _synthetic_run()
    tweaked.kpis["overshoot_m"] = 1e-9
    assert export.run_record(tweaked, include_channels=True)["sha256"] != doc["sha256"]
    assert export.payload_sha256(json.loads(text)) == doc["sha256"], "the stored hash verifies"
    with pytest.raises(ValueError, match="^schema: "):
        export.run_from_record({"schema": "fabrail.ensemble/1"})

    # Without channels: KPIs, events, meta and the per-channel summary survive.
    slim = export.run_record(run)
    assert "channels" not in slim and slim["summary"]["x_m"]["last"] == float(run.channels["x_m"][-1])
    slim_back = export.run_from_record(json.loads(export.dumps(slim)))
    assert slim_back.kpis == run.kpis and slim_back.events == run.events and slim_back.meta == run.meta

    # Ensemble: per-member KPIs (null where absent), summary, shared params stored once.
    members = [_synthetic_run(seed=i, wall_s=0.5 + i) for i in range(3)]
    members[1].kpis.pop("overshoot_m")
    members[2].kpis["bumper_impacts"] = 1.0
    ens = EnsembleResult.from_members(seed=42, members=members, noise_seeds=[11, 12, 13], meta={"n_workers": 4, "wall_s": 3.0})
    edoc = export.ensemble_record(ens)
    assert edoc["kpis"]["overshoot_m"] == [0.0, None, 0.0] and edoc["summary"]["overshoot_m"]["n"] == 2.0
    etext = export.dumps(edoc)
    eback = export.ensemble_from_record(json.loads(etext))
    assert [m.kpis for m in eback.members] == [m.kpis for m in members]
    assert [m.meta for m in eback.members] == [m.meta for m in members] and eback.meta == ens.meta
    assert eback.noise_seeds == [11, 12, 13] and eback.seed == 42 and eback.scenario == "throw_soft_close"
    assert export.ensemble_record(eback) == json.loads(etext), "ensemble re-export reproduces the document"
    other_wall = EnsembleResult.from_members(
        seed=42, members=[_synthetic_run(seed=i, wall_s=7.0) for i in range(3)], noise_seeds=[11, 12, 13]
    )
    other_wall.members[1].kpis.pop("overshoot_m")
    other_wall.members[2].kpis["bumper_impacts"] = 1.0
    other_wall.meta = {"n_workers": 4, "wall_s": 9.0}
    assert export.ensemble_record(other_wall)["sha256"] == edoc["sha256"], "ensemble hash ignores wall_s"

    # Anchors: values with per-anchor tolerance and source; check_anchors applies abs + rel·|v|.
    adoc = export.anchors_record(
        {"door_hover_gap_mean_m": 0.005, "unseat_time_s": [0.061, 0.072, 0.090]},
        {
            "door_hover_gap_mean_m": {"abs": 2e-6, "source": "FAB-LEV-004 (±2 µm)", "unit": "m"},
            "unseat_time_s": {"rel": 0.05, "source": "core §E.5 (P)", "unit": "s"},
        },
    )
    aback = json.loads(export.dumps(adoc))
    assert aback == adoc and aback["schema"] == export.ANCHORS_SCHEMA
    assert export.check_anchors(aback, {"door_hover_gap_mean_m": 0.0050015, "unseat_time_s": [0.062, 0.070, 0.093]}) == []
    fails = export.check_anchors(aback, {"door_hover_gap_mean_m": 0.0050025, "unseat_time_s": [0.061, 0.072]})
    assert len(fails) == 2 and "FAB-LEV-004" in fails[0] and "shape" in fails[1]
    assert export.check_anchors(aback, {}) == ["door_hover_gap_mean_m: not measured", "unseat_time_s: not measured"]
    for values, tols in (({"a": 1.0}, {}), ({"a": 1.0}, {"a": {"rel": -1.0}}), ({"a": 1.0}, {"a": 0.1, "b": 0.1})):
        with pytest.raises(ValueError):
            export.anchors_record(values, tols)

    # Study: canonical and hashed, with wall_s excluded.
    sdoc = export.study_record("drag-ledger", 1, "How zero is zero friction?", {"mu_eq": [2.0e-3, 3.3e-3], "wall_s": 0.4})
    sdoc2 = export.study_record("drag-ledger", 1, "How zero is zero friction?", {"mu_eq": [2.0e-3, 3.3e-3], "wall_s": 8.0})
    assert sdoc["sha256"] == sdoc2["sha256"] and json.loads(export.dumps(sdoc)) == sdoc


def _house_figure(run: RunResult):
    """A figure in the house style: position (mm), semilog distance-to-go, KPI bars, events and a reference line."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from fabrail import plotstyle

    plotstyle.apply()
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(10, 3))
    ax1.plot(run.t_s, run["x_m"] * 1e3, color=plotstyle.CLASS_COLORS["drawer"], label="x")
    ax1.scatter([e.t_s for e in run.events], [e.a * 1e3 for e in run.events], color=plotstyle.COLORS["star"])
    plotstyle.ref_line(ax1, 0.30 * 1e3, "closed")
    plotstyle.style_ax(ax1, "t [s]", "x [mm]", "terminus")
    d = 0.30 - run["x_m"]
    ax2.semilogy(run.t_s, d, color=plotstyle.COLORS["s2"])
    ax2.axvline(run.events[1].t_s, color=plotstyle.COLORS["muted"])
    plotstyle.style_ax(ax2, "t [s]", "distance to go [m]", "log-damped")
    names = sorted(run.kpis)
    ax3.bar(range(len(names)), [run.kpis[n] for n in names], color=plotstyle.SERIES[: len(names)])
    ax3.fill_between(run.t_s[:50], run["v_mps"][:50], 0.0, alpha=0.12)
    plotstyle.run_stamp(fig, run.meta)
    return fig


@pytest.mark.req("FAB-SW-009")
def test_every_plotted_number():
    """Every number drawn in a house-style figure is in the run's JSON record (m plotted as mm allowed via scale)."""
    import matplotlib.pyplot as plt

    run = _synthetic_run()
    run.meta["closed_x_m"] = 0.30  # the reference level drawn on the figure is recorded, too
    fig = _house_figure(run)
    try:
        doc = json.loads(export.dumps(export.run_record(run, include_channels=True)))
        extra = {"distance_to_go_m": 0.30 - run["x_m"]}  # derived series plotted on ax2, recorded alongside
        record = {"run": doc, "derived": json.loads(export.dumps(extra))}
        plotted = export.plotted_arrays(fig)
        assert sum(a.size for _, a in plotted) > 3 * run.n_samples, "the extractor saw the lines, bars, fills and points"
        missing = export.missing_plotted(fig, record, scales=(1.0, 1e3))
        assert missing == [], f"plotted but not recorded (FAB-SW-009): {missing}"
        # Not vacuous: without channels the time series are missing, and a stray point is reported.
        assert export.missing_plotted(fig, json.loads(export.dumps(export.run_record(run))), scales=(1.0, 1e3))
        fig.axes[0].plot([0.123456789], [9.87654321])
        stray = export.missing_plotted(fig, record, scales=(1.0, 1e3))
        assert len(stray) == 2 and all("lines" in s for s in stray), stray
    finally:
        plt.close(fig)


# ---- native and property variants (no requirement row) -------------------------------------


def test_native_run_export_roundtrip(native):
    """A native run exports and re-imports exactly; two same-seed runs hash identically (wall_s excluded)."""
    from fabrail.results import RunResult as RR

    spec = {
        "schema": native.SCENARIO_SCHEMA,
        "name": "export_smoke",
        "duration_s": 0.05,
        "noise_seed": 42,
        "fidelity": {"model": "travel1d", "coil": "voltage"},
        "initial": {"lev": "levitated", "x_m": 0.5, "v_mps": 0.1},
    }
    params = native.Params.preset("door")
    try:
        runs = [RR.from_native(native.run(params, native.Scenario.from_json(json.dumps(spec)))) for _ in range(2)]
    except NotImplementedError as exc:
        pytest.skip(f"fabrail_sim::run is not implemented yet: {exc}")
    a, b = (export.run_record(r, include_channels=True) for r in runs)
    assert a["sha256"] == b["sha256"], "same seed, same record hash (wall_s excluded; plan §2b item 16)"
    text = export.dumps(a)
    back = export.run_from_record(json.loads(text))
    assert export.run_record(back, include_channels=True) == json.loads(text)
    for name, arr in runs[0].channels.items():
        assert _same_bits(np.nan_to_num(back.channels[name]), np.nan_to_num(arr)), name
    assert back.meta["params"] == json.loads(params.to_json()), "the run records its full parameter set"


@settings(max_examples=60, deadline=None)
@given(st.lists(st.floats(allow_nan=True, allow_infinity=False, width=64), min_size=1, max_size=40))
def test_float_roundtrip_property(values):
    """Any finite float64 (subnormals and -0.0 included) survives run_record → JSON → run_from_record bit for bit."""
    arr = np.asarray(values, dtype=np.float64)
    run = RunResult(rate_hz=10.0, t_s=np.arange(arr.size) / 10.0, channels={"c": arr}, units={"c": "m"}, meta={"wall_s": 0.1})
    back = export.run_from_record(json.loads(export.dumps(export.run_record(run, include_channels=True))))
    got = back.channels["c"]
    finite = np.isfinite(arr)
    assert np.array_equal(np.isnan(got), ~finite)
    assert _same_bits(got[finite], arr[finite])
