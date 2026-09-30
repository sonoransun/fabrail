//! Recursive STA/LTA trigger with LTA freeze while triggered (PHYSICS §20; SEI-002, SEI-003). Owner: A3.
//!
//! Frozen API (W0): [`StaLta::new`], [`StaLta::step`] → [`StaLtaOut`]. Recursive
//! averages over |x|² with STA 50 and LTA 3000 samples at 100 Hz, trigger on 4.0 and off
//! 1.5, LTA frozen while triggered. SEI-003 reference: the exact discrete recursion
//! including the LTA term, ±1 sample (plan §2b item 7); step response 0.1116 s (K = 16).
//!
//! **The recursion, exactly as implemented** (Allen 1978; Withers et al. 1998 recursive
//! form; PHYSICS §20). With e_k = x_k² and gains g(N) = 1/min(k, N) (an expanding-window
//! mean until the window has filled, then the fixed recursive gain 1/N):
//!
//! ```text
//! sta_k = sta_{k−1} + g(N_sta)·(e_k − sta_{k−1})
//! lta_k = lta_{k−1} + g(N_lta)·(e_k − lta_{k−1})     (not updated while triggered)
//! ratio_k = sta_k / max(lta_k, floor)
//! on:  not triggered, k ≥ N_lta (warm-up done) and ratio_k ≥ trig_on
//! off: triggered and ratio_k < trig_off   (the LTA resumes on the next sample)
//! ```
//!
//! The ratio uses the LTA *including* sample k when not triggered; the sample that turns
//! the trigger on has therefore already entered the LTA. For a variance step 1 → K from
//! the steady state the continuous estimate is t = τ_sta·ln((K − 1)/(K − θ)) = 0.1116 s at
//! K = 16, θ = 4 (τ_sta = N_sta/f_s = 0.5 s); the exact discrete answer (with the LTA
//! term) is 13 samples (0.13 s), which the unit test reproduces to ±1 sample.
//!
//! `floor` bounds the ratio's denominator: 1 LSB² of the ADXL355 by default, set by
//! `SeismicUnit` (a signal after exact silence, all-zero input, still triggers; no division by zero).
//! Non-finite samples are ignored (the averages are never poisoned).

use fabrail_params::DetectorTuning;

/// STA/LTA output for one sample.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StaLtaOut {
    /// STA/LTA ratio.
    pub ratio: f32,
    /// Currently triggered.
    pub triggered: bool,
    /// A trigger started on this sample (the P pick).
    pub onset: bool,
}

/// Recursive STA/LTA detector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StaLta {
    /// Short-term average.
    pub sta: f32,
    /// Long-term average.
    pub lta: f32,
    /// Trigger-on ratio.
    pub on: f32,
    /// Trigger-off ratio.
    pub off: f32,
    /// Triggered.
    pub triggered: bool,
    /// Samples seen (warm-up: no trigger before the LTA window fills).
    pub n: u32,
    /// STA window, samples.
    pub sta_n: u32,
    /// LTA window, samples.
    pub lta_n: u32,
    /// Ratio denominator floor, (m/s²)².
    pub floor: f32,
    /// Last ratio.
    pub ratio: f32,
}

impl StaLta {
    /// Detector from the detector tuning (windows `sta_n`/`lta_n`, thresholds
    /// `trig_on`/`trig_off`). The ratio floor defaults to the smallest positive f32;
    /// `SeismicUnit` sets it to 1 LSB².
    pub fn new(t: &DetectorTuning) -> StaLta {
        StaLta {
            on: t.trig_on as f32,
            off: t.trig_off as f32,
            sta_n: t.sta_n.max(1),
            lta_n: t.lta_n.max(1),
            floor: f32::MIN_POSITIVE,
            ..StaLta::default()
        }
    }

    /// Recursive gain 1/min(k, N) for sample number k ≥ 1.
    #[inline]
    fn gain(k: u32, n: u32) -> f32 {
        1.0 / (k.min(n).max(1) as f32)
    }

    /// One sample of the (band-passed) characteristic signal.
    pub fn step(&mut self, x: f32) -> StaLtaOut {
        if !x.is_finite() {
            return StaLtaOut {
                ratio: self.ratio,
                triggered: self.triggered,
                onset: false,
            };
        }
        let e = x * x;
        self.n = self.n.saturating_add(1);
        self.sta += Self::gain(self.n, self.sta_n) * (e - self.sta);
        if !self.triggered {
            self.lta += Self::gain(self.n, self.lta_n) * (e - self.lta);
        }
        let den = if self.lta > self.floor {
            self.lta
        } else {
            self.floor
        };
        let ratio = self.sta / den;
        self.ratio = ratio;
        let mut onset = false;
        if self.triggered {
            if ratio < self.off {
                self.triggered = false;
            }
        } else if self.n >= self.lta_n && ratio >= self.on {
            self.triggered = true;
            onset = true;
        }
        StaLtaOut {
            ratio,
            triggered: self.triggered,
            onset,
        }
    }

    /// Ends a trigger without a sample (stale-trigger release): the LTA resumes.
    pub fn release(&mut self) {
        self.triggered = false;
    }

    /// Clears the averages (a new warm-up follows).
    pub fn reset(&mut self) {
        self.sta = 0.0;
        self.lta = 0.0;
        self.triggered = false;
        self.n = 0;
        self.ratio = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    /// The exact discrete recursion in f64 from the steady state (sta = lta = 1) after a
    /// variance step to K, LTA term included: the first sample index (1-based) at which
    /// sta/lta ≥ θ.
    fn exact_trigger_sample(k: f64, theta: f64, n_sta: f64, n_lta: f64) -> u32 {
        let (mut sta, mut lta) = (1.0_f64, 1.0_f64);
        for i in 1..10_000_u32 {
            sta += (k - sta) / n_sta;
            lta += (k - lta) / n_lta;
            if sta / lta >= theta {
                return i;
            }
        }
        u32::MAX
    }

    fn warm(det: &mut StaLta, amp: f32, n: u32) {
        for _ in 0..n {
            let o = det.step(amp);
            assert!(!o.onset, "constant input never triggers");
        }
    }

    #[test]
    fn step_trigger_matches_exact_recursion() {
        let t = presets::door().control.seismic.detector;
        assert_eq!((t.sta_n, t.lta_n), (50, 3000), "core design §C.4");
        let fs = 100.0_f64;
        for (k, cont_s) in [(16.0_f64, 0.1116_f64), (9.0, 0.2350)] {
            let mut det = StaLta::new(&t);
            warm(&mut det, 1.0, t.lta_n + 500);
            assert!(
                (det.sta - 1.0).abs() < 1e-6 && (det.lta - 1.0).abs() < 1e-6,
                "steady state"
            );
            let amp = k.sqrt() as f32;
            let mut got = None;
            for i in 1..400_u32 {
                if det.step(amp).onset {
                    got = Some(i);
                    break;
                }
            }
            let got = got.expect("the step triggers");
            let want = exact_trigger_sample(k, 4.0, 50.0, 3000.0);
            assert!(
                got.abs_diff(want) <= 1,
                "K = {k}: trigger at sample {got}, exact discrete recursion {want} (±1 sample; plan §2b item 7)"
            );
            // Continuous estimate τ_sta·ln((K − 1)/(K − θ)) (reported, not asserted
            // against the discrete answer; SEI-003).
            let cont = 0.5 * ((k - 1.0) / (k - 4.0)).ln();
            assert!(
                (cont - cont_s).abs() < 5e-4,
                "continuous estimate {cont:.4} s (Allen 1978)"
            );
            let lag_s = f64::from(got) / fs;
            assert!(
                lag_s >= cont,
                "the discrete trigger never precedes the continuous estimate"
            );
        }
    }

    #[test]
    fn lta_frozen_while_triggered_and_off_below_1_5() {
        let t = presets::wall().control.seismic.detector;
        let mut det = StaLta::new(&t);
        warm(&mut det, 1.0, t.lta_n);
        let mut on = false;
        for _ in 0..50 {
            on |= det.step(4.0).onset;
        }
        assert!(on && det.triggered, "a ×16 variance step triggers");
        let lta_at = det.lta;
        for _ in 0..200 {
            det.step(4.0);
        }
        assert_eq!(det.lta, lta_at, "LTA frozen while triggered (tolerance 0)");
        let mut off_at = None;
        for i in 0..2000_u32 {
            let o = det.step(1.0);
            if !o.triggered {
                off_at = Some(i);
                break;
            }
        }
        assert!(off_at.is_some(), "returns below trig_off 1.5");
        assert!(det.ratio < 1.5, "ratio {} < 1.5 at release", det.ratio);
        let lta_off = det.lta;
        det.step(1.0);
        assert_ne!(det.lta, lta_off, "LTA resumes after the trigger ends");
    }

    #[test]
    fn no_trigger_during_warm_up_and_nan_ignored() {
        let t = presets::drawer().control.seismic.detector;
        let mut det = StaLta::new(&t);
        for _ in 0..t.lta_n - 1 {
            assert!(!det.step(100.0).onset);
            assert!(!det.step(f32::NAN).onset);
        }
        assert!(
            det.sta.is_finite() && det.lta.is_finite(),
            "NaN never enters the averages"
        );
        // Silence then a signal: the floor keeps the ratio finite and the trigger fires.
        let mut det = StaLta::new(&t);
        det.floor = 1e-9;
        warm(&mut det, 0.0, t.lta_n);
        let o = det.step(1e-2);
        assert!(
            o.onset && o.ratio.is_finite(),
            "trigger after silence, finite ratio"
        );
    }
}
