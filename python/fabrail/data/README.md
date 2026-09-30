# fabrail data: provenance only

Nothing in this directory is data. fabrail ships **no** recorded ground motion: no
PEER NGA-West2 records, no CESMD/CGS files, no PEGS or STEAD waveforms. Records are
fetched by the user into the repository-level `data/` directory, which is git-ignored
and excluded from publishing (`up.sh`). Every default test runs offline on synthetic
input.

## What the synthesis modules generate instead

| Module | Output | Model and sources |
|---|---|---|
| `fabrail.ground` | horizontal KT/CP records scaled to a PGA | Kanai 1957 / Tajimi 1960 and Clough–Penzien spectra (soil table of Der Kiureghian & Neuenhofer 1992), Shinozuka & Deodatis 1991 spectral representation, Jennings–Housner–Tsai 1968 envelope, 0.05 Hz zero-phase baseline correction with Boore 2005 pads, ASCE 7-16 §11.4.6 design spectrum, Nigam & Jennings 1969 response spectrum (PHYSICS §19) |
| `fabrail.pwave` | three-axis P+S records and site noise | Boore 2003 stochastic method (Brune source, Hanks & Kanamori 1979 moment, Q(f) = 180 f^0.45, κ), P phase scaled to Wu & Zhao 2006 Pd at the hypocentral distance with τc from Wu & Kanamori 2005; site noise with ADXL355 data-sheet noise density and [UNCERTAIN] footstep, slam, HVAC and self-actuation models (PHYSICS §19, §22) |
| `fabrail.records.elcentro_like` | a 30 s, 0.34 g, medium-soil surrogate | the KT/CP chain above; it matches the 1940 record's PGA, duration and soil class, not its waveform, PGV or PGD. Isolation and end-stop tests only; never detector tests (it has no P onset) |
| `fabrail.rail` | rail irregularity profiles | L/1150 sag, 0.1–0.3 mm joint steps, 20 µm rms roughness over 5–500 mm (PHYSICS §10) |

Every synthetic record is labelled synthetic in the scenario `doc`; every network alert
in a scenario is "synthetic/injected, not a live EEW feed".

## The 1940 El Centro record (optional, user-fetched)

- Source page: CESMD Internet Quick Report, El Centro 18 May 1940
  (<https://www.strongmotioncenter.org/cgi-bin/CESMD/iqr_dist_DM2.pl?IQRID=ElCentro_18May1940&SFlag=0&Flag=3>).
  The record was operated by the U.S. Coast and Geodetic Survey; the currently
  reachable digitisations are processed by non-federal parties (CESMD/CGS) or
  copyright-claimed (PEER), and the NCEI federal set was decommissioned. Follow the
  provider's data policy and cite it (CESMD asks for attribution:
  <https://www.strongmotioncenter.org/NCESMD/reports/CESMD_DataPolicy.htm>).
- Do **not** use the OpenSees example file `elCentro.at2`: it is the 1968 Borrego
  Mountain record.
- Fetch: `fabrail.records.fetch_elcentro(url=...)` (or the `FABRAIL_ELCENTRO_URL`
  environment variable) with the URL of the file you are entitled to download; a
  `file://` URL works for a local copy. fabrail does not scrape the provider's pages.
- Trust on first use: the first fetch writes the file's SHA-256 to `data/SHA256SUMS`
  and its URL, hash and UTC time to `data/elcentro.json`. Every later fetch or
  `load_elcentro()` must match the pin; a mismatch is rejected and nothing is replaced.
  Delete both files to re-pin deliberately.
- Formats read: PEER `.AT2` (`load_at2`, units g or cm/s²) and CSV/whitespace text
  (`load_csv`, optional time column, units g, m/s² or gal).
- Tests that need the real record are marked `requires_data` and skip when it is
  absent.

## Honesty notes

- Synthetic records are models, not measurements; their PGA, Pd and τc follow the
  stated regressions by construction, and their PGV/PGD are those of the chosen
  spectral model.
- fabrail is not an earthquake early-warning provider and ships no live feed.
