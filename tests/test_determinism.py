"""Determinism of runs, ensembles and the golden selftest (FAB-SW-002, FAB-FW-005).

Contract:

- The same seed gives byte-identical canonical JSON (``fabrail.export.run_record``,
  ``wall_s`` excluded from the hashed payload) for two runs in this process and for
  runs in two fresh subprocesses; the SHA-256 of the canonical text, the record's own
  ``sha256`` and a SHA-256 over every raw channel buffer all agree (plan §2b item 16).
- Ensembles (``fabrail.ensemble.run_ensemble``) and native batches
  (``_native.run_batch``) do not depend on the worker count, and a batch member equals
  the same job run alone.
- ``_native.selftest_hash(2000)`` is a pure function (in-process and subprocess), the
  binding exports the ``GOLDEN_SELFTEST_HASH`` that crates/fabrail-rt/src/selftest.rs
  defines, runs record it as ``meta.golden``, the firmware compares against that
  symbol (never a copied literal), and once C1 freezes it (Wave 3) the hash equals it.

Cases: a Carriage6 DOOR ``push_open`` (voltage coil mode, 1 s; SIL ≡ firmware is claimed
in voltage mode only) with every channel sample in the JSON, and the Travel1d WALL
``pwave_m7_r20`` record (numpy P+S synthesis plus Rust sensor noise, 71.5 s).
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import sys
import textwrap

import pytest

import _support

pytestmark = pytest.mark.usefixtures("native")

#: (label, class, scenario, tier, duration_s or None, record channels in the JSON)
CASES = (
    ("door_push_open", "door", "push_open", "carriage6", 1.0, True),
    ("wall_pwave_m7_r20", "wall", "pwave_m7_r20", None, None, False),
)

#: The digest code shared by this process and the subprocesses (one source of truth).
_DIGEST_SRC = textwrap.dedent(
    """
    import hashlib, json
    from fabrail import _native, export, scenarios
    from fabrail.config import RunConfig
    from fabrail.results import RunResult

    def digest(cls, name, fidelity, duration_s, channels, seed=42):
        cfg = RunConfig(rail_class=cls, seed=seed, coil="voltage", duration_s=duration_s)
        params = cfg.params()
        spec_d, series = scenarios.spec(name, params, cfg, fidelity=fidelity)
        data = _native.run(params, _native.Scenario.from_json(scenarios.to_json(spec_d), series))
        rec = export.run_record(RunResult.from_native(data), include_channels=channels)
        text = export.dumps(export.stable_payload(rec), indent=None).encode("utf-8")
        raw = hashlib.sha256()
        for ch in data.channel_names():
            buf, dtype, unit = data.channel(ch)
            raw.update(ch.encode() + b"\\0" + dtype.encode() + b"\\0" + unit.encode() + b"\\0")
            raw.update(buf)
        return {
            "text_sha256": hashlib.sha256(text).hexdigest(),
            "n_bytes": len(text),
            "record_sha256": rec["sha256"],
            "payload_sha256": export.payload_sha256(rec),
            "channels_sha256": raw.hexdigest(),
            "has_wall_s_in_timing": "wall_s" in rec.get("timing", {}),
            "wall_s_in_payload": "wall_s" in text.decode("utf-8"),
        }
    """
)


def _digest_fn():
    ns: dict[str, object] = {}
    exec(compile(_DIGEST_SRC, "<determinism digest>", "exec"), ns)  # noqa: S102 - our own code
    return ns["digest"]


def _subprocess_digests(n: int = 2) -> list[dict[str, dict[str, object]]]:
    """Digests of every case from ``n`` fresh interpreters, run concurrently."""
    code = _DIGEST_SRC + textwrap.dedent(
        f"""
        cases = {[(c[0], c[1], c[2], c[3], c[4], c[5]) for c in CASES]!r}
        print(json.dumps({{c[0]: digest(*c[1:]) for c in cases}}))
        """
    )
    env = {**os.environ, "PYTHONPATH": str(_support.ROOT / "python"), "PYTHONHASHSEED": "random"}
    procs = [
        subprocess.Popen(
            [sys.executable, "-c", code],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            cwd=str(_support.ROOT),
            env=env,
        )
        for _ in range(n)
    ]
    out = []
    for p in procs:
        stdout, stderr = p.communicate(timeout=120)
        assert p.returncode == 0, f"digest subprocess failed:\n{stderr[-3000:]}"
        out.append(json.loads(stdout.strip().splitlines()[-1]))
    return out


@pytest.mark.req("FAB-SW-002")
def test_same_seed_identical_json():
    """Seed 42 gives byte-identical canonical JSON in process (twice) and in two fresh
    subprocesses: SHA-256 of the text, the record's own sha256 and the raw-channel
    SHA-256 all agree, and ``wall_s`` lives only in the unhashed ``timing`` block
    (FAB-SW-002; plan §2b item 16)."""
    digest = _digest_fn()
    local = {c[0]: [digest(*c[1:]) for _ in range(2)] for c in CASES}
    remote = _subprocess_digests(2)
    bad = []
    for label, runs in local.items():
        a, b = runs
        if a != b:
            bad.append(f"{label}: two in-process runs differ: {a} vs {b}")
        for k, other in enumerate(remote, 1):
            if other.get(label) != a:
                bad.append(f"{label}: subprocess {k} differs: {other.get(label)} vs in-process {a}")
        if a["record_sha256"] != a["payload_sha256"]:
            bad.append(f"{label}: record sha256 {a['record_sha256']} != payload_sha256 {a['payload_sha256']}")
        if not a["has_wall_s_in_timing"]:
            bad.append(f"{label}: timing.wall_s missing (the volatile key must move to the unhashed block)")
        if a["wall_s_in_payload"]:
            bad.append(f"{label}: 'wall_s' appears in the hashed payload")
        if a["n_bytes"] < 1000:
            bad.append(f"{label}: suspiciously small record ({a['n_bytes']} bytes)")
    assert not bad, "(E) byte-identical JSON for the same seed (FAB-SW-002):\n" + "\n".join(bad)


def test_different_seed_changes_the_record():
    """Control for the test above: a different seed changes the stochastic record, so an
    identical hash is evidence of determinism, not of a constant output."""
    digest = _digest_fn()
    a = digest("wall", "pwave_m7_r20", None, None, False, seed=42)
    b = digest("wall", "pwave_m7_r20", None, None, False, seed=43)
    assert a["channels_sha256"] != b["channels_sha256"], "seeds 42 and 43 gave identical channels (synthesis unseeded?)"
    assert a["text_sha256"] != b["text_sha256"], "seeds 42 and 43 gave identical records"


def _channels_digest(data) -> str:
    h = hashlib.sha256()
    for ch in data.channel_names():
        buf, dtype, unit = data.channel(ch)
        h.update(f"{ch}\0{dtype}\0{unit}\0".encode())
        h.update(buf)
    h.update(json.dumps(data.events()).encode())
    h.update(json.dumps(sorted(data.kpis().items())).encode())
    return h.hexdigest()


@pytest.mark.req("FAB-SW-002")
def test_ensemble_worker_invariance(native):
    """An ensemble (4 members of the stochastic WALL ``pwave_m5_r10``, Travel1d) and a
    native batch (3 Carriage6 DOOR jobs with distinct seeds) give identical records and
    channel bytes on 1 and on 4 workers, and each batch member equals its job run alone
    (FAB-SW-002; SeedSequence(seed).spawn(n)[i] per member, index order)."""
    from fabrail import export, scenarios
    from fabrail.config import RunConfig
    from fabrail.ensemble import run_ensemble

    bad = []
    # Ensemble: Python synthesis per member + Rust sensor noise, run_batch underneath.
    recs, digests = {}, {}
    for workers in (1, 4):
        cfg = RunConfig(rail_class="wall", seed=42, coil="voltage", workers=workers)
        ens = run_ensemble(cfg.params(), "pwave_m5_r10", cfg, 4)
        recs[workers] = export.ensemble_record(ens)
        digests[workers] = [
            hashlib.sha256(
                b"".join(m.channels[n].tobytes() for n in m.channels) + json.dumps(m.kpis, sort_keys=True).encode()
            ).hexdigest()
            for m in ens.members
        ]
    if recs[1]["sha256"] != recs[4]["sha256"]:
        bad.append(f"ensemble record sha256 differs: 1 worker {recs[1]['sha256']} vs 4 workers {recs[4]['sha256']}")
    if export.dumps(export.stable_payload(recs[1])) != export.dumps(export.stable_payload(recs[4])):
        bad.append("ensemble canonical JSON differs between 1 and 4 workers")
    if digests[1] != digests[4]:
        bad.append(f"ensemble member channels differ between 1 and 4 workers: {digests[1]} vs {digests[4]}")
    if len(set(digests[1])) != len(digests[1]):
        bad.append("ensemble members are not independent draws (identical member digests)")

    # Native batch: order and worker count must not matter.
    jobs = []
    for seed in (1, 2, 3):
        cfg = RunConfig(rail_class="door", seed=seed, coil="voltage", duration_s=0.5)
        p = cfg.params()
        jobs.append((p, scenarios.build("push_open", p, cfg, fidelity="carriage6")))
    one = [_channels_digest(d) for d in native.run_batch(jobs, 1)]
    four = [_channels_digest(d) for d in native.run_batch(jobs, 4)]
    alone = [_channels_digest(native.run(p, s)) for p, s in jobs]
    if one != four:
        bad.append(f"run_batch differs between 1 and 4 workers: {one} vs {four}")
    if one != alone:
        bad.append(f"run_batch members differ from single runs: {one} vs {alone}")
    assert not bad, "(E) worker-count invariance (FAB-SW-002):\n" + "\n".join(bad)


_GOLDEN_RE = re.compile(r"pub\s+const\s+GOLDEN_SELFTEST_HASH\s*:\s*u64\s*=\s*(0x[0-9A-Fa-f_]+|\d[\d_]*)\s*;")


def _golden_from_source() -> int:
    src = (_support.ROOT / "crates" / "fabrail-rt" / "src" / "selftest.rs").read_text(encoding="utf-8")
    m = _GOLDEN_RE.search(src)
    assert m, "crates/fabrail-rt/src/selftest.rs: no `pub const GOLDEN_SELFTEST_HASH: u64 = …;`"
    return int(m.group(1).replace("_", ""), 0)


@pytest.mark.req("FAB-FW-005")
def test_selftest_hash_matches(native):
    """``selftest_hash(2000)`` is pure (two calls and a subprocess agree), the binding's
    ``GOLDEN_SELFTEST_HASH`` is the one fabrail-rt defines, runs record it in
    ``meta.golden``, the firmware compares against that symbol, and — once C1 has frozen
    it (Wave 3) — the hash equals it (FAB-FW-005; TECHNICAL_DESIGN §9.4)."""
    h1 = native.selftest_hash(2000)
    h2 = native.selftest_hash(2000)
    assert h1 == h2, f"(E) selftest_hash(2000) not pure: 0x{h1:016x} vs 0x{h2:016x}"
    assert 0 <= h1 < 2**64 and h1 != 0, f"selftest_hash(2000) = {h1} is not a non-zero u64"
    r = subprocess.run(
        [sys.executable, "-c", "from fabrail import _native; print(_native.selftest_hash(2000))"],
        capture_output=True,
        text=True,
        timeout=60,
        env={**os.environ, "PYTHONPATH": str(_support.ROOT / "python")},
    )
    assert r.returncode == 0, r.stderr[-2000:]
    assert int(r.stdout.strip()) == h1, f"(E) subprocess selftest hash {r.stdout.strip()} != 0x{h1:016x}"

    golden_src = _golden_from_source()
    golden = int(native.GOLDEN_SELFTEST_HASH)
    assert golden == golden_src, (
        f"_native.GOLDEN_SELFTEST_HASH 0x{golden:016x} != fabrail-rt source 0x{golden_src:016x} (stale build?)"
    )
    meta = json.loads(_support.cached_run("door", "hover", fidelity="carriage6", duration_s=0.2).data.meta_json())
    assert int(meta.get("golden", -1)) == golden, f"RunMeta.golden {meta.get('golden')} != GOLDEN_SELFTEST_HASH {golden}"

    main_rs = (_support.ROOT / "firmware" / "src" / "main.rs").read_text(encoding="utf-8")
    assert re.search(r"\bGOLDEN_SELFTEST_HASH\b", main_rs) and "selftest_hash(" in main_rs, (
        "firmware/src/main.rs must compare fabrail_rt::selftest_hash with fabrail_rt::GOLDEN_SELFTEST_HASH"
    )
    literal = f"{h1:x}"
    fw_text = "\n".join(p.read_text(encoding="utf-8") for p in (_support.ROOT / "firmware" / "src").rglob("*.rs"))
    assert literal not in fw_text.lower().replace("_", ""), "the firmware copies the hash as a literal"

    if golden == 0:
        pytest.skip(
            f"GOLDEN_SELFTEST_HASH is not frozen yet (placeholder 0; C1 freezes it in Wave 3); "
            f"current selftest_hash(2000) = 0x{h1:016x}"
        )
    assert h1 == golden, f"(E) selftest_hash(2000) 0x{h1:016x} != GOLDEN_SELFTEST_HASH 0x{golden:016x} (FAB-FW-005)"
