//! DetectorReplay: the seismic detector over recorded or synthetic acceleration, streamed in chunks. Owner: A7 (signatures frozen by W0-B3).
//!
//! Runs the same seismic front end as the firmware (ADXL355 quantisation, anti-alias
//! ÷5, band-pass, STA/LTA, Pd/τc, level matrix — the core `SeismicUnit`) over structure
//! acceleration in m/s², fed in chunks (the 24 h noise study streams 10-minute chunks,
//! plan §2b item 13). `feed` returns the events raised within the chunk; `report`
//! summarises everything so far.
//!
//! Signal path (PHYSICS §13, §20):
//!
//! 1. the input stream at `fs_hz` is resampled to the ADXL355 output data rate
//!    (`design.sensors.accel_odr_hz`, 500 Hz) by linear interpolation (a pass-through
//!    when the rates agree);
//! 2. each ADXL355 sample is quantised to its 20-bit code (±2.048 g clip) exactly as
//!    `SimBoard` does; sensor noise is added only after [`DetectorReplay::set_noise_seed`]
//!    (the input is taken to be what the sensor measured);
//! 3. the samples queue in the 32-sample FIFO and the unit runs once per seismic release
//!    (`seismic_decim` base ticks, 100 Hz), draining ≤ 16 samples at tick j·decim —
//!    the firmware's schedule. Release j sees every sample with t ≤ j·T_seis; it runs
//!    once a later sample has arrived, so the last partial release waits for the next
//!    chunk.
//!
//! Events (time from the start of the stream): `Trigger` (a = STA/LTA triggers so far),
//! `SeismicState` (a = new, b = old wire value), `Freeze` (entry into TRIGGERED),
//! `Protect` (a = level, b = seconds to the S-ETA, 0 if unknown).
//!
//! Report keys (frozen): `samples`, `duration_s`, `fs_hz`, `raw_triggers`,
//! `freeze_count`, `protect_count`, `level_max`.

use std::collections::BTreeMap;

use fabrail_core::seismic::SeismicUnit;
use fabrail_params::SystemParams;
use fabrail_types::{ACCEL_DRAIN_MAX, AccelRaw, BASE_TICK_HZ, SeismicState, Tick};

use crate::recorder::EventRec;
use crate::rng::{NormalStream, StreamTag};
use crate::sensors::{Adxl355Fifo, adxl355_code, adxl355_sigma};

/// Streaming seismic detector replay.
#[derive(Clone, Debug)]
pub struct DetectorReplay {
    params: Box<SystemParams>,
    fs_hz: f64,
    samples: u64,
    unit: Option<SeismicUnit>,
    odr_hz: f64,
    lsb: f64,
    sigma: f64,
    noise: Option<[NormalStream; 3]>,
    seis_decim: u64,
    /// Next ADXL355 sample index to produce.
    out_n: u64,
    /// Previous input sample (index, value).
    prev: Option<(u64, [f64; 3])>,
    fifo: Adxl355Fifo,
    /// Next seismic release index.
    release: u64,
    last_state: SeismicState,
    last_protect: bool,
    raw_triggers: u32,
    freeze_count: u64,
    protect_count: u64,
    level_max: u8,
}

impl DetectorReplay {
    /// A detector for parameter set `p` fed at `fs_hz` (the ADXL355 ODR, 500 Hz by
    /// default). A non-finite or non-positive rate makes every `feed` a no-op; a
    /// parameter set the seismic unit rejects only counts samples.
    pub fn new(p: &SystemParams, fs_hz: f64) -> DetectorReplay {
        let cal = &p.design.sensors;
        DetectorReplay {
            params: Box::new(*p),
            fs_hz,
            samples: 0,
            unit: SeismicUnit::new(p).ok(),
            odr_hz: cal.accel_odr_hz,
            lsb: cal.accel_mps2_per_lsb,
            sigma: adxl355_sigma(cal.accel_noise_mps2_per_rthz, cal.accel_odr_hz),
            noise: None,
            seis_decim: p.control.rates.seismic_decim.max(1) as u64,
            out_n: 0,
            prev: None,
            fifo: Adxl355Fifo::new(),
            release: 0,
            last_state: SeismicState::Normal,
            last_protect: false,
            raw_triggers: 0,
            freeze_count: 0,
            protect_count: 0,
            level_max: 0,
        }
    }

    /// Adds the ADXL355 white noise (data-sheet density over the Nyquist band, PHYSICS
    /// §13) from the `Accel` substreams of `seed` (device 0); `None` removes it.
    pub fn set_noise_seed(&mut self, seed: Option<u64>) {
        self.noise = seed.map(|s| {
            core::array::from_fn(|ch| NormalStream::substream(s, StreamTag::Accel, 0, ch as u16))
        });
    }

    /// The input sample rate, Hz.
    pub fn fs_hz(&self) -> f64 {
        self.fs_hz
    }

    /// The parameter set.
    pub fn params(&self) -> &SystemParams {
        &self.params
    }

    fn input_ok(&self) -> bool {
        self.fs_hz.is_finite() && self.fs_hz > 0.0
    }

    fn odr_ok(&self) -> bool {
        self.odr_hz.is_finite() && self.odr_hz > 0.0
    }

    /// Release j's time, s.
    fn release_t(&self, j: u64) -> f64 {
        (j * self.seis_decim) as f64 / BASE_TICK_HZ as f64
    }

    /// Runs every release strictly before `t_next` (the next sample's time).
    fn run_releases_before(&mut self, t_next: f64, out: &mut Vec<EventRec>) {
        while self.release_t(self.release) < t_next {
            let j = self.release;
            self.release += 1;
            let t = self.release_t(j);
            let mut buf = [AccelRaw::default(); ACCEL_DRAIN_MAX];
            let n = self.fifo.drain(&mut buf);
            let Some(unit) = self.unit.as_mut() else {
                continue;
            };
            let r = unit.step(Tick(j * self.seis_decim), &buf[..n], None, false);
            if r.raw_triggers > self.raw_triggers {
                out.push(EventRec {
                    t_s: t,
                    kind: "Trigger".into(),
                    a: r.raw_triggers as f64,
                    b: 0.0,
                });
            }
            self.raw_triggers = r.raw_triggers;
            if r.state != self.last_state {
                out.push(EventRec {
                    t_s: t,
                    kind: "SeismicState".into(),
                    a: r.state.as_u8() as f64,
                    b: self.last_state.as_u8() as f64,
                });
                if r.state == SeismicState::Triggered {
                    self.freeze_count += 1;
                    out.push(EventRec {
                        t_s: t,
                        kind: "Freeze".into(),
                        a: 0.0,
                        b: 0.0,
                    });
                }
                self.last_state = r.state;
            }
            if r.protect && !self.last_protect {
                self.protect_count += 1;
                let to_s = r.s_eta.map_or(0.0, |e| e.seconds_f64() - t);
                out.push(EventRec {
                    t_s: t,
                    kind: "Protect".into(),
                    a: r.level.as_u8() as f64,
                    b: to_s,
                });
            }
            self.last_protect = r.protect;
            self.level_max = self.level_max.max(r.level.as_u8());
        }
    }

    /// Emits ADXL355 sample `out_n` with acceleration `a` (m/s²).
    fn emit(&mut self, a: [f64; 3], out: &mut Vec<EventRec>) {
        let t = self.out_n as f64 / self.odr_hz;
        self.run_releases_before(t, out);
        let mut xyz = [0i32; 3];
        for (ch, (c, v)) in xyz.iter_mut().zip(a).enumerate() {
            let n = match self.noise.as_mut() {
                Some(s) => self.sigma * s[ch].next_normal(),
                None => 0.0,
            };
            *c = adxl355_code(v + n, self.lsb);
        }
        self.fifo.push(xyz);
        self.out_n += 1;
    }

    /// Feeds one chunk of x, y, z acceleration (m/s²; the shortest slice sets the
    /// chunk length) and returns the events raised in it (times from the start of the
    /// stream).
    pub fn feed(&mut self, x: &[f64], y: &[f64], z: &[f64]) -> Vec<EventRec> {
        let mut out = Vec::new();
        if !self.input_ok() {
            return out;
        }
        let n = x.len().min(y.len()).min(z.len());
        if !self.odr_ok() {
            // No usable ADXL355 rate in the parameters: count only.
            self.samples = self.samples.saturating_add(n as u64);
            return out;
        }
        let same_rate = self.fs_hz == self.odr_hz;
        for k in 0..n {
            let m = self.samples;
            let a = [x[k], y[k], z[k]];
            if same_rate {
                self.emit(a, &mut out);
            } else {
                // ADXL355 samples with t_n ≤ t_m, interpolated from the previous input.
                let t_m = m as f64 / self.fs_hz;
                while self.out_n as f64 / self.odr_hz <= t_m {
                    let t_n = self.out_n as f64 / self.odr_hz;
                    let v = match self.prev {
                        Some((pm, pa)) => {
                            let t_p = pm as f64 / self.fs_hz;
                            let u = if t_m > t_p {
                                (t_n - t_p) / (t_m - t_p)
                            } else {
                                1.0
                            };
                            [
                                pa[0] + (a[0] - pa[0]) * u,
                                pa[1] + (a[1] - pa[1]) * u,
                                pa[2] + (a[2] - pa[2]) * u,
                            ]
                        }
                        None => a,
                    };
                    self.emit(v, &mut out);
                }
                self.prev = Some((m, a));
            }
            self.samples += 1;
        }
        out
    }

    /// Summary so far (see the module docs for the keys).
    pub fn report(&self) -> BTreeMap<String, f64> {
        let duration_s = if self.fs_hz > 0.0 {
            self.samples as f64 / self.fs_hz
        } else {
            0.0
        };
        [
            ("samples", self.samples as f64),
            ("duration_s", duration_s),
            ("fs_hz", self.fs_hz),
            ("raw_triggers", self.raw_triggers as f64),
            ("freeze_count", self.freeze_count as f64),
            ("protect_count", self.protect_count as f64),
            ("level_max", self.level_max as f64),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_string(), v))
        .collect()
    }

    /// Seismic releases run so far.
    pub fn releases(&self) -> u64 {
        self.release
    }

    /// ADXL355 samples produced so far.
    pub fn adxl_samples(&self) -> u64 {
        self.out_n
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn report_counts_streamed_samples() {
        let mut d = DetectorReplay::new(&SystemParams::default(), 500.0);
        assert!(d.feed(&[0.0; 500], &[0.0; 500], &[0.0; 400]).is_empty());
        let r = d.report();
        assert_eq!(r["samples"], 400.0);
        assert_eq!(r["duration_s"], 0.8);
        let mut bad = DetectorReplay::new(&SystemParams::default(), 0.0);
        bad.feed(&[0.0], &[0.0], &[0.0]);
        assert_eq!(bad.report()["samples"], 0.0);
        let keys: Vec<_> = r.keys().cloned().collect();
        assert_eq!(
            keys,
            [
                "duration_s",
                "freeze_count",
                "fs_hz",
                "level_max",
                "protect_count",
                "raw_triggers",
                "samples"
            ],
            "frozen report keys"
        );
    }

    #[test]
    fn releases_follow_the_firmware_schedule_across_chunks() {
        let p = fabrail_params::presets::door();
        let mut d = DetectorReplay::new(&p, 500.0);
        // 1 s at 500 Hz in uneven chunks: releases at 0, 10, …, 990 ms run once a later
        // sample exists → 100 releases (the one at 1.0 s waits for more data).
        let x = vec![0.0; 500];
        d.feed(&x[..123], &x[..123], &x[..123]);
        d.feed(&x[123..], &x[123..], &x[123..]);
        assert_eq!(d.adxl_samples(), 500);
        assert_eq!(d.releases(), 100);
        // Chunking does not change anything.
        let mut one = DetectorReplay::new(&p, 500.0);
        one.feed(&x, &x, &x);
        assert_eq!(one.releases(), d.releases());
        assert_eq!(one.report(), d.report());
    }

    #[test]
    fn resampling_from_100_hz_produces_the_odr() {
        let p = fabrail_params::presets::door();
        let mut d = DetectorReplay::new(&p, 100.0);
        let x = vec![0.0; 100];
        d.feed(&x, &x, &x);
        // Inputs at 0 … 0.99 s → ADXL samples at 0 … 0.99 s at 500 Hz: 496.
        assert_eq!(d.adxl_samples(), 496);
        assert_eq!(d.report()["samples"], 100.0);
    }
}
