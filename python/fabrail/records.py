"""Recorded ground motion: loaders, the El Centro fetch and a surrogate (core §B.8).

Records are fetched, never vendored (no PEER, PEGS or CESMD data ships with fabrail):
:func:`fetch_elcentro` downloads into ``data/`` (git-ignored and excluded from
publishing) and pins the file's SHA-256 on first use (trust on first use: the hash is
written to ``data/SHA256SUMS`` and every later fetch or load must match it); tests
needing it are marked ``requires_data``. The surrogate :func:`elcentro_like` (medium
soil, 0.34 g, 30 s) keeps every default test offline. Provenance notes:
``python/fabrail/data/README.md``.

Which file to fetch: no reachable digitisation of the 1940 El Centro record is both
federal and unprocessed (physics perspective §0 item 11). The CESMD distribution page
is :data:`ELCENTRO_SOURCE_PAGE`; CESMD's data policy asks for attribution. The OpenSees
"elCentro" example file is the 1968 Borrego Mountain record — do not use it.

Owner: W0-C2 (stub), then A9 (Wave 1).
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import json
import math
import os
import re
import shutil
import tempfile
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .ground import G0_MPS2, resolve_jht, synthesize, trapezoid_integral

__all__ = [
    "ELCENTRO_SOURCE_PAGE",
    "ELCENTRO_URL_ENV",
    "PIN_FILE",
    "Record",
    "elcentro_like",
    "fetch_elcentro",
    "load_at2",
    "load_csv",
    "load_elcentro",
]

#: CESMD Internet Quick Report page of the 18 May 1940 El Centro record (source of the
#: download; not fetched automatically — see :func:`fetch_elcentro`).
ELCENTRO_SOURCE_PAGE = "https://www.strongmotioncenter.org/cgi-bin/CESMD/iqr_dist_DM2.pl?IQRID=ElCentro_18May1940&SFlag=0&Flag=3"

#: Environment variable holding the record file URL when ``url`` is not given.
ELCENTRO_URL_ENV = "FABRAIL_ELCENTRO_URL"

#: Trust-on-first-use pin file in the data directory (``sha256sum`` format).
PIN_FILE = "SHA256SUMS"

#: Provenance file written next to the fetched record.
_ELCENTRO_META = "elcentro.json"

#: Unit conversions to m/s² ("g" is standard gravity; gal = cm/s²).
_UNITS: dict[str, float] = {"g": G0_MPS2, "m/s2": 1.0, "m/s^2": 1.0, "mps2": 1.0, "gal": 0.01, "cm/s2": 0.01, "cm/s^2": 0.01}

_TIME_NAMES = ("t", "time", "t_s", "time_s")
_AXES = ("x", "y", "z")


@dataclass(frozen=True)
class Record:
    """A ground-motion record in SI.

    Attributes:
        name: record name.
        fs_hz: sample rate, Hz.
        a_mps2: acceleration, shape (n,) for one component or (3, n) for x, y, z, m/s².
        source: provenance (URL, file or "surrogate").
        sha256: SHA-256 of the source file ("" for synthetic records).
        meta: loader details (units converted, columns, header fields).
    """

    name: str
    fs_hz: float
    a_mps2: np.ndarray
    source: str = ""
    sha256: str = ""
    meta: dict[str, object] = field(default_factory=dict)

    @property
    def duration_s(self) -> float:
        """Record length, s (samples / fs)."""
        return float(np.shape(self.a_mps2)[-1]) / self.fs_hz


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def _unit_scale(units: str) -> float:
    try:
        return _UNITS[units.strip().lower()]
    except KeyError:
        raise ValueError(f"units: expected one of {', '.join(_UNITS)} (got {units!r})") from None


def _is_number(token: str) -> bool:
    try:
        float(token)
    except ValueError:
        return False
    return True


def load_csv(
    path: str | Path,
    fs_hz: float | None = None,
    columns: str = "x,y,z",
    units: str = "g",
    *,
    name: str | None = None,
) -> Record:
    """Load a CSV record (optional time column; ``units`` "g", "m/s2" or "gal").

    Comma- or whitespace-separated; lines starting with ``#`` are comments. A first row
    with a non-numeric token is a header; a header whose first name is ``t``/``time``
    marks a time column. Without a header, the first column is time exactly when
    ``fs_hz`` is ``None``. A time column must be uniform (1e-6 relative) and, with
    ``fs_hz`` also given, agree with it. ``columns`` names the axes of the data columns
    in order (e.g. ``"z,x,y"``; extra columns are ignored); the result is (n,) for one
    axis, otherwise (3, n) in x, y, z order with unlisted axes zero.
    """
    p = Path(path)
    axes = [c.strip().lower() for c in columns.split(",") if c.strip()]
    if not axes or any(a not in _AXES for a in axes) or len(set(axes)) != len(axes):
        raise ValueError(f"columns: expected distinct axes from x,y,z (got {columns!r})")
    scale = _unit_scale(units)
    rows: list[list[str]] = []
    header: list[str] | None = None
    for line in p.read_text(encoding="utf-8", errors="replace").splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        tokens = [t.strip() for t in (s.split(",") if "," in s else s.split())]
        if header is None and not rows and not all(_is_number(t) for t in tokens):
            header = [t.lower() for t in tokens]
            continue
        rows.append(tokens)
    if not rows:
        raise ValueError(f"{p}: no data rows")
    width = len(rows[0])
    if any(len(r) != width for r in rows):
        raise ValueError(f"{p}: rows have unequal column counts")
    data = np.array([[float(t) for t in r] for r in rows], dtype=np.float64).T
    has_time = header[0] in _TIME_NAMES if header is not None else fs_hz is None
    fs = fs_hz
    if has_time:
        t = data[0]
        data = data[1:]
        dt = np.diff(t)
        if dt.shape[0] < 1 or np.any(dt <= 0.0):
            raise ValueError(f"{p}: the time column must be strictly increasing")
        med = float(np.median(dt))
        if float(np.max(np.abs(dt - med))) > 1e-6 * med:
            raise ValueError(f"{p}: the time column is not uniformly sampled")
        fs_t = 1.0 / med
        if fs_hz is not None and abs(fs_t - fs_hz) > 1e-6 * fs_hz:
            raise ValueError(f"{p}: fs_hz={fs_hz} disagrees with the time column ({fs_t:.9g} Hz)")
        fs = fs_hz if fs_hz is not None else fs_t
    if fs is None or not (math.isfinite(fs) and fs > 0.0):
        raise ValueError(f"{p}: fs_hz is required (no time column)")
    if data.shape[0] < len(axes):
        raise ValueError(f"{p}: {data.shape[0]} data columns for {len(axes)} axes ({columns!r})")
    if len(axes) == 1:
        a = data[0] * scale
    else:
        a = np.zeros((3, data.shape[1]))
        for k, axis in enumerate(axes):
            a[_AXES.index(axis)] = data[k] * scale
    meta: dict[str, object] = {"format": "csv", "units": units, "columns": axes, "time_column": bool(has_time)}
    if header is not None:
        meta["header"] = header
    if len(axes) == 1:
        meta["axis"] = axes[0]
    return Record(name or p.stem, float(fs), a, str(p), _sha256(p), meta)


_NPTS = re.compile(r"NPTS\s*=\s*(\d+)", re.IGNORECASE)
_DT = re.compile(r"DT\s*=\s*([0-9.Ee+-]+)", re.IGNORECASE)


def load_at2(path: str | Path) -> Record:
    """Load a PEER NGA ``.AT2`` record (units g → m/s²).

    Four header lines (title; event, date, station, component; units; ``NPTS= n,
    DT= dt SEC`` — the older ``n dt NPTS, DT`` layout is also read), then free-format
    values. The units line must say G (or CM/S/S). The file is read from disk only;
    fabrail never downloads or ships PEER data.
    """
    p = Path(path)
    lines = p.read_text(encoding="utf-8", errors="replace").splitlines()
    if len(lines) < 5:
        raise ValueError(f"{p}: too short for an AT2 file (4 header lines + data)")
    head = lines[:4]
    m_n, m_dt = _NPTS.search(head[3]), _DT.search(head[3])
    if m_n and m_dt:
        npts, dt = int(m_n.group(1)), float(m_dt.group(1))
    else:
        tok = head[3].replace(",", " ").split()
        if len(tok) < 2 or not (_is_number(tok[0]) and _is_number(tok[1])):
            raise ValueError(f"{p}: cannot read NPTS and DT from header line 4: {head[3]!r}")
        npts, dt = int(float(tok[0])), float(tok[1])
    units_line = head[2].upper()
    if "CM/S" in units_line or "CM/SEC" in units_line:
        scale, units = 0.01, "cm/s2"
    elif re.search(r"\bG\b", units_line):
        scale, units = G0_MPS2, "g"
    else:
        raise ValueError(f"{p}: unrecognised units line {head[2]!r} (expected G or CM/S/S)")
    values = [float(t) for line in lines[4:] for t in line.replace(",", " ").split()]
    if len(values) < npts:
        raise ValueError(f"{p}: header NPTS={npts} but only {len(values)} values")
    if not (dt > 0.0):
        raise ValueError(f"{p}: DT must be > 0 (got {dt})")
    a = np.asarray(values[:npts], dtype=np.float64) * scale
    meta: dict[str, object] = {"format": "at2", "units": units, "npts": npts, "dt_s": dt, "header": head[:3]}
    return Record(p.stem, 1.0 / dt, a, str(p), _sha256(p), meta)


def _read_pins(dest: Path) -> dict[str, str]:
    pins: dict[str, str] = {}
    pin_file = dest / PIN_FILE
    if pin_file.exists():
        for line in pin_file.read_text(encoding="utf-8").splitlines():
            parts = line.strip().split(None, 1)
            if len(parts) == 2:
                pins[parts[1].lstrip("*")] = parts[0].lower()
    return pins


def _write_pin(dest: Path, name: str, digest: str) -> None:
    with open(dest / PIN_FILE, "a", encoding="utf-8") as fh:
        fh.write(f"{digest}  {name}\n")


def fetch_elcentro(
    dest: str | Path = Path("data"),
    *,
    url: str | None = None,
    filename: str | None = None,
    timeout_s: float = 60.0,
    force: bool = False,
) -> Path:
    """Download the 1940 El Centro record into ``dest`` (network; SHA-256 trust on first use).

    ``url`` (or the ``FABRAIL_ELCENTRO_URL`` environment variable) is the record file
    the user is entitled to download — fabrail does not scrape the CESMD page or bypass
    its data policy, so without one this raises ``ValueError`` naming
    :data:`ELCENTRO_SOURCE_PAGE`. ``file://`` URLs work for a local copy. The first
    successful fetch pins the file's SHA-256 in ``dest/SHA256SUMS`` and writes
    ``dest/elcentro.json`` (URL, file, hash, UTC time); a later fetch whose hash differs
    from the pin is rejected and nothing is replaced. An existing pinned file is reused
    without network access unless ``force``.
    """
    d = Path(dest)
    src = url or os.environ.get(ELCENTRO_URL_ENV)
    if not src:
        raise ValueError(
            "fetch_elcentro: no source URL. Records are fetched, never vendored: obtain the 1940 El Centro "
            f"record file from {ELCENTRO_SOURCE_PAGE} (CESMD data policy: attribution required) and pass "
            f"its URL as url= or {ELCENTRO_URL_ENV} (a file:// URL works for a local copy)"
        )
    parsed = urllib.parse.urlparse(src)
    if parsed.scheme not in ("http", "https", "file"):
        raise ValueError(f"fetch_elcentro: unsupported URL scheme {parsed.scheme!r} (http, https or file)")
    name = filename or Path(urllib.parse.unquote(parsed.path)).name or "elcentro_1940.dat"
    if not name.lower().startswith("elcentro"):
        name = f"elcentro_{name}"
    if Path(name).name != name or name in (PIN_FILE, _ELCENTRO_META):
        raise ValueError(f"fetch_elcentro: invalid file name {name!r}")
    d.mkdir(parents=True, exist_ok=True)
    target = d / name
    pins = _read_pins(d)
    if target.exists() and name in pins and not force:
        if _sha256(target) != pins[name]:
            raise ValueError(f"fetch_elcentro: {target} does not match its pinned SHA-256 {pins[name]} (delete it to re-fetch)")
        return target
    fd, tmp_name = tempfile.mkstemp(prefix=".elcentro-", dir=d)
    tmp = Path(tmp_name)
    try:
        with os.fdopen(fd, "wb") as out, urllib.request.urlopen(src, timeout=timeout_s) as resp:  # noqa: S310 - scheme checked above
            shutil.copyfileobj(resp, out)
        digest = _sha256(tmp)
        if name in pins:
            if digest != pins[name]:
                raise ValueError(
                    f"fetch_elcentro: SHA-256 {digest} of {src} does not match the pin {pins[name]} "
                    f"recorded on first use in {d / PIN_FILE}; the file was not replaced"
                )
        else:
            _write_pin(d, name, digest)
        os.replace(tmp, target)
    finally:
        if tmp.exists():
            tmp.unlink()
    meta = {
        "url": src,
        "file": name,
        "sha256": digest,
        "fetched_utc": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"),
        "source_page": ELCENTRO_SOURCE_PAGE,
        "note": "1940 Imperial Valley (El Centro) record; not vendored; cite the provider per its data policy",
    }
    (d / _ELCENTRO_META).write_text(json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return target


def load_elcentro(dest: str | Path = Path("data"), *, fs_hz: float | None = None, units: str = "g", columns: str = "x") -> Record:
    """Load the fetched El Centro record from ``dest`` after checking its pinned SHA-256.

    ``.at2`` files go through :func:`load_at2`; ``.csv``/``.txt``/``.dat`` through
    :func:`load_csv` with ``fs_hz``, ``units`` and ``columns``. Raises
    ``FileNotFoundError`` when nothing was fetched (tests mark this ``requires_data``).
    """
    d = Path(dest)
    meta_path = d / _ELCENTRO_META
    if not meta_path.exists():
        raise FileNotFoundError(f"no fetched El Centro record in {d} (python -m fabrail data fetch elcentro)")
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    path = d / str(meta["file"])
    pins = _read_pins(d)
    digest = _sha256(path)
    if pins.get(path.name) != digest:
        raise ValueError(f"load_elcentro: {path} does not match its pinned SHA-256 in {d / PIN_FILE}")
    suffix = path.suffix.lower()
    rec = load_at2(path) if suffix == ".at2" else load_csv(path, fs_hz=fs_hz, columns=columns, units=units, name="elcentro_1940")
    return Record("elcentro_1940", rec.fs_hz, rec.a_mps2, str(meta.get("url", path)), digest, {**rec.meta, "provenance": meta})


def elcentro_like(rng: np.random.Generator, *, fs_hz: float = 100.0) -> Record:
    """An offline El Centro-like surrogate: medium soil, 0.34 g PGA, 30 s (physics perspective §3).

    :func:`fabrail.ground.synthesize` with the Clough–Penzien medium-soil spectrum, the
    JHT envelope scaled to 30 s and 0.05 Hz baseline correction; one horizontal
    component along x, shape (n,). It matches the 1940 record's PGA, duration and soil
    class, not its waveform: its PGV/PGD are those of the CP model (report them from
    ``meta``). Use it for isolation and end-stop tests only, never for detector tests —
    it has no P onset.
    """
    duration_s, pga_g, soil, model = 30.0, 0.34, "medium", "cp"
    a = synthesize(rng, duration_s, fs_hz, soil=soil, pga_g=pga_g, envelope=None, model=model)
    v = trapezoid_integral(a, fs_hz)
    d = trapezoid_integral(v, fs_hz)
    env = resolve_jht(None, duration_s)
    meta: dict[str, object] = {
        "format": "surrogate",
        "soil": soil,
        "model": model,
        "pga_g": pga_g,
        "duration_s": duration_s,
        "envelope": [env.t_rise_s, env.t_strong_end_s, env.decay_per_s],
        "baseline_hp_hz": 0.05,
        "axis": "x",
        "pgv_mps": float(np.max(np.abs(v))),
        "pgd_m": float(np.max(np.abs(d))),
    }
    return Record("elcentro_like", float(fs_hz), a, "surrogate", "", meta)
