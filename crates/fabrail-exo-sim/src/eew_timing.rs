//! EEW timing: alert latency chain, warning time, lead-time gain per km, blind-zone radius (FAB-EXO-803, 804). Owner: X6 (API frozen by W0a).
//!
//! EXOTIC §X10 (PHYSICS §20 for the core relations). fabrail is not an EEW provider:
//! every timing here is for synthetic or injected alerts, and every output prints its
//! inputs ([`WarningTiming::inputs`], [`SimWarning::inputs`]).
//!
//! * Alert time from origin: t_alert = R_k/V_p + τ_win + t_pkt + t_tel + t_proc + t_diss
//!   ([`EewLatency`]; default budget τ_win 1 s, t_pkt 1 s, t_tel 0.5 s, t_proc 0.5 s,
//!   t_diss 0.3 s = 3.3 s \[UNCERTAIN: typical-order values\]).
//! * Warning at the site: W = R_site/V_s − t_alert − t_act.
//! * Gain per km of sensor advance: ∂W/∂x = 1/V_p (0.15385 s/km at 6.5 km/s) and 1/V_s
//!   for PLUM-type intensity algorithms (0.27027 s/km at 3.7 km/s)
//!   ([`lead_time_gain_s_per_km`]).
//! * Blind zone r_b = √((V_s·t_alert)² − h²) ([`blind_zone_km`]).
//! * DAS: a fibre measures axial strain ε; for a plane wave crossing the fibre at the
//!   apparent velocity V_app the particle velocity is u̇ = −V_app·ε
//!   ([`das_particle_velocity_mps`]; exotic design §2.8). Broadside arrivals (V_app → ∞)
//!   carry no axial strain, so DAS picks come from channels off the closest point.
//!
//! **Offshore worked case** (FAB-EXO-804; [`OffshoreCase::reference`]): epicentre 100 km
//! offshore at 20 km depth, city 20 km inland (S at 32.9 s), 4th station, 3.3 s latency,
//! V_p 6.5 km/s, V_s 3.7 km/s, t_act 0:
//!
//! | Sensing | k-th sensor (epicentral) | Alert | Warning |
//! |---|---|---|---|
//! | Coastal land network | 110 km | 20.5 s | 12.4 s |
//! | S-net-like seafloor | 34 km | 9.4 s | 23.5 s |
//! | DAS over the source | 8 km | 6.6 s | 26.3 s |
//!
//! [`simulate_offshore`] runs the same chain as a time-stepped simulation (reduced model,
//! 🔶): a P wavefield sampled at 100 Hz at every sensor with ADXL-class noise, STA/LTA
//! picks, per-sensor packetisation phases from the seed, telemetry, the k-th pick at the
//! centre, processing and dissemination, and the S onset at the city on the same sample
//! grid. DAS channels synthesise strain ε = −u̇/V_app and pick on the reconstructed
//! u̇ = −V_app·ε. The simulated warning is compared with the analytic value (±1 s).

use fabrail_sim::rng::{NormalStream, Xoshiro256pp};
use fabrail_types::fmath;

/// Design total alert latency (window + packet + telemetry + processing +
/// dissemination), s (exotic design §2.8 \[UNCERTAIN\]).
pub const DEFAULT_LATENCY_S: f64 = 3.3;

/// P-wave speed of the EXOTIC §X10 worked case, km/s (the core fusion default is 6.0).
pub const V_P_KMPS: f64 = 6.5;

/// S-wave speed of the EXOTIC §X10 worked case and of PLUM-type algorithms, km/s.
pub const V_S_KMPS: f64 = 3.7;

/// The alert latency chain after the first P arrival at the reporting sensor, s.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EewLatency {
    /// Detection window τ_win, s.
    pub window_s: f64,
    /// Packetisation, s.
    pub packet_s: f64,
    /// Telemetry, s.
    pub telemetry_s: f64,
    /// Processing, s.
    pub processing_s: f64,
    /// Dissemination, s.
    pub dissemination_s: f64,
}

impl EewLatency {
    /// Sum of the chain, s.
    pub fn total_s(&self) -> f64 {
        self.window_s + self.packet_s + self.telemetry_s + self.processing_s + self.dissemination_s
    }

    /// The default budget: τ_win 1 s, t_pkt 1 s, t_tel 0.5 s, t_proc 0.5 s, t_diss 0.3 s
    /// (3.3 s; EXOTIC §X10 \[UNCERTAIN: typical-order values\]).
    pub const fn default_budget() -> EewLatency {
        EewLatency {
            window_s: 1.0,
            packet_s: 1.0,
            telemetry_s: 0.5,
            processing_s: 0.5,
            dissemination_s: 0.3,
        }
    }

    /// The budget scaled so its total is `total_s`, keeping the default split (a
    /// non-positive total gives a zero chain).
    pub fn scaled_to(total_s: f64) -> EewLatency {
        let d = EewLatency::default_budget();
        let s = if total_s > 0.0 && total_s.is_finite() {
            total_s / d.total_s()
        } else {
            0.0
        };
        EewLatency {
            window_s: d.window_s * s,
            packet_s: d.packet_s * s,
            telemetry_s: d.telemetry_s * s,
            processing_s: d.processing_s * s,
            dissemination_s: d.dissemination_s * s,
        }
    }
}

/// Lead-time gain per km of sensor advance, 1/v, s/km (FAB-EXO-803).
pub fn lead_time_gain_s_per_km(v_kmps: f64) -> f64 {
    if v_kmps <= 0.0 {
        return 0.0;
    }
    1.0 / v_kmps
}

/// Alert time after origin: R_sensor/V_p + latency, s.
pub fn t_alert_s(r_sensor_km: f64, v_p_kmps: f64, latency: &EewLatency) -> f64 {
    if v_p_kmps <= 0.0 {
        return f64::INFINITY;
    }
    r_sensor_km / v_p_kmps + latency.total_s()
}

/// Warning time at the site: R_site/V_s − t_alert − t_act, s (negative: blind).
pub fn warning_s(r_site_km: f64, v_s_kmps: f64, t_alert_s: f64, t_act_s: f64) -> f64 {
    if v_s_kmps <= 0.0 {
        return f64::NEG_INFINITY;
    }
    r_site_km / v_s_kmps - t_alert_s - t_act_s
}

/// Blind-zone epicentral radius √((V_s·t_alert)² − h²), km (0 when the depth exceeds
/// the S-wave travel distance at alert time).
pub fn blind_zone_km(v_s_kmps: f64, t_alert_s: f64, depth_km: f64) -> f64 {
    let r = v_s_kmps * t_alert_s;
    let d = r * r - depth_km * depth_km;
    if d <= 0.0 { 0.0 } else { fmath::sqrt(d) }
}

/// Hypocentral distance √(r_epi² + h²), km.
pub fn hypocentral_km(r_epi_km: f64, depth_km: f64) -> f64 {
    fmath::hypot(r_epi_km, depth_km)
}

/// DAS particle velocity from axial strain, u̇ = −V_app·ε, m/s (exotic design §2.8;
/// `v_app_mps` is the apparent velocity of the wave along the fibre).
pub fn das_particle_velocity_mps(v_app_mps: f64, strain: f64) -> f64 {
    -v_app_mps * strain
}

/// Axial strain a plane wave of particle velocity `u_dot_mps` produces on a fibre it
/// crosses at apparent velocity `v_app_mps`, ε = −u̇/V_app (0 for a broadside arrival,
/// V_app infinite or non-positive).
pub fn das_strain(v_app_mps: f64, u_dot_mps: f64) -> f64 {
    if !(v_app_mps > 0.0 && v_app_mps.is_finite()) {
        return 0.0;
    }
    -u_dot_mps / v_app_mps
}

/// Apparent velocity along a straight cable at arc position `s_km` from the point of
/// closest approach, for a source at hypocentral distance `r_hyp_km` from that channel:
/// V_app = V·R/|s| (∞ at the closest point), km/s.
pub fn cable_apparent_velocity_kmps(v_kmps: f64, r_hyp_km: f64, s_km: f64) -> f64 {
    let s = fmath::abs(s_km);
    if s <= 0.0 {
        return f64::INFINITY;
    }
    v_kmps * r_hyp_km / s
}

/// P-wave peak displacement from magnitude and hypocentral distance,
/// log10 Pd\[cm\] = −3.463 + 0.729·M − 1.374·log10 R\[km\] (Wu and Zhao 2006, PHYSICS §20),
/// m. Used only to size the synthetic P onset of the timing simulation.
pub fn pd_m(magnitude: f64, r_hyp_km: f64) -> f64 {
    let r = r_hyp_km.max(1.0);
    0.01 * fmath::pow(10.0, -3.463 + 0.729 * magnitude - 1.374 * fmath::log10(r))
}

/// A sensing network of the offshore case.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Sensing {
    /// Coastal land network (the k-th station about 110 km from the epicentre).
    LandNetwork,
    /// S-net-like seafloor cabled stations (about 34 km).
    OffshoreCabled,
    /// A DAS fibre passing over the source (closest approach about 8 km).
    Das,
}

impl Sensing {
    /// Every sensing kind, in table order.
    pub const ALL: [Sensing; 3] = [Sensing::LandNetwork, Sensing::OffshoreCabled, Sensing::Das];

    /// Report name.
    pub const fn name(self) -> &'static str {
        match self {
            Sensing::LandNetwork => "land",
            Sensing::OffshoreCabled => "snet",
            Sensing::Das => "das",
        }
    }
}

/// The offshore worked case (EXOTIC §X10; FAB-EXO-804).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OffshoreCase {
    /// Magnitude (sizes the synthetic P onset only).
    pub magnitude: f64,
    /// Epicentre distance offshore from the coast, km.
    pub epi_offshore_km: f64,
    /// Hypocentral depth, km.
    pub depth_km: f64,
    /// City distance inland from the coast, km.
    pub city_inland_km: f64,
    /// P-wave speed, km/s.
    pub v_p_kmps: f64,
    /// S-wave speed, km/s.
    pub v_s_kmps: f64,
    /// Latency chain.
    pub latency: EewLatency,
    /// Actuation time at the site, s.
    pub t_act_s: f64,
    /// Picks needed for an alert (the k-th sensor; DAS: channels).
    pub k_sensor: usize,
    /// Epicentral distance of the k-th land station, km.
    pub land_km: f64,
    /// Epicentral distance of the k-th seafloor station, km.
    pub snet_km: f64,
    /// Closest approach of the DAS cable to the epicentre, km.
    pub das_offset_km: f64,
}

impl OffshoreCase {
    /// The EXOTIC §X10 worked case: M8, 100 km offshore, 20 km deep, city 20 km inland,
    /// V_p 6.5 / V_s 3.7 km/s, 3.3 s latency, t_act 0, 4th sensor at 110 / 34 / 8 km.
    pub fn reference() -> OffshoreCase {
        OffshoreCase {
            magnitude: 8.0,
            epi_offshore_km: 100.0,
            depth_km: 20.0,
            city_inland_km: 20.0,
            v_p_kmps: V_P_KMPS,
            v_s_kmps: V_S_KMPS,
            latency: EewLatency::default_budget(),
            t_act_s: 0.0,
            k_sensor: 4,
            land_km: 110.0,
            snet_km: 34.0,
            das_offset_km: 8.0,
        }
    }

    /// Epicentral distance of the city, km.
    pub fn city_epi_km(&self) -> f64 {
        self.epi_offshore_km + self.city_inland_km
    }

    /// S-wave arrival at the city after origin, s (32.9 s for the reference case).
    pub fn t_s_city_s(&self) -> f64 {
        if self.v_s_kmps <= 0.0 {
            return f64::INFINITY;
        }
        hypocentral_km(self.city_epi_km(), self.depth_km) / self.v_s_kmps
    }

    /// Epicentral distance of the reporting (k-th) sensor, km.
    pub fn sensor_epi_km(&self, s: Sensing) -> f64 {
        match s {
            Sensing::LandNetwork => self.land_km,
            Sensing::OffshoreCabled => self.snet_km,
            Sensing::Das => self.das_offset_km,
        }
    }

    /// Analytic timing for sensing `s` (EXOTIC §X10 closed forms).
    pub fn analytic(&self, s: Sensing) -> WarningTiming {
        let r_sensor = hypocentral_km(self.sensor_epi_km(s), self.depth_km);
        let t_alert = t_alert_s(r_sensor, self.v_p_kmps, &self.latency);
        let r_city = hypocentral_km(self.city_epi_km(), self.depth_km);
        WarningTiming {
            sensing: s,
            r_sensor_hyp_km: r_sensor,
            r_site_hyp_km: r_city,
            t_alert_s: t_alert,
            warning_s: warning_s(r_city, self.v_s_kmps, t_alert, self.t_act_s),
            blind_zone_km: blind_zone_km(self.v_s_kmps, t_alert, self.depth_km),
            gain_p_s_per_km: lead_time_gain_s_per_km(self.v_p_kmps),
            gain_s_s_per_km: lead_time_gain_s_per_km(self.v_s_kmps),
            case: *self,
        }
    }
}

/// An analytic warning-time result with its inputs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WarningTiming {
    /// Sensing.
    pub sensing: Sensing,
    /// Hypocentral distance of the reporting sensor, km.
    pub r_sensor_hyp_km: f64,
    /// Hypocentral distance of the site, km.
    pub r_site_hyp_km: f64,
    /// Alert time after origin, s.
    pub t_alert_s: f64,
    /// Warning before the S wave at the site, s.
    pub warning_s: f64,
    /// Blind-zone epicentral radius at this alert time, km.
    pub blind_zone_km: f64,
    /// Gain per km of sensor advance for P-based algorithms, s/km.
    pub gain_p_s_per_km: f64,
    /// Gain per km for intensity-based (PLUM) algorithms, s/km.
    pub gain_s_s_per_km: f64,
    /// The inputs.
    pub case: OffshoreCase,
}

impl WarningTiming {
    /// Every input and output as (name, value) pairs (the print-your-inputs rule).
    pub fn inputs(&self) -> Vec<(&'static str, f64)> {
        let c = &self.case;
        vec![
            ("magnitude", c.magnitude),
            ("epi_offshore_km", c.epi_offshore_km),
            ("depth_km", c.depth_km),
            ("city_inland_km", c.city_inland_km),
            ("v_p_kmps", c.v_p_kmps),
            ("v_s_kmps", c.v_s_kmps),
            ("latency_s", c.latency.total_s()),
            ("t_act_s", c.t_act_s),
            ("k_sensor", c.k_sensor as f64),
            ("sensor_epi_km", c.sensor_epi_km(self.sensing)),
            ("r_sensor_hyp_km", self.r_sensor_hyp_km),
            ("r_site_hyp_km", self.r_site_hyp_km),
            ("t_alert_s", self.t_alert_s),
            ("warning_s", self.warning_s),
            ("blind_zone_km", self.blind_zone_km),
        ]
    }
}

/// Settings of the timing simulation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimSettings {
    /// Sample rate, Hz.
    pub fs_hz: f64,
    /// Sensor velocity noise, m/s RMS (ADXL355-class floor integrated to velocity
    /// \[UNCERTAIN\]).
    pub noise_mps: f64,
    /// DAS strain noise, RMS (order 1e-11 per channel at 100 Hz \[UNCERTAIN\]).
    pub das_noise_strain: f64,
    /// STA window, s.
    pub sta_s: f64,
    /// LTA window, s.
    pub lta_s: f64,
    /// STA/LTA trigger ratio.
    pub trigger_ratio: f64,
    /// Dominant frequency of the synthetic P onset, Hz.
    pub f_p_hz: f64,
    /// Rise time of the P onset, s.
    pub rise_s: f64,
    /// DAS channel spacing, m.
    pub das_spacing_m: f64,
    /// DAS cable half-length each side of the closest point, km.
    pub das_half_length_km: f64,
    /// Channels with an apparent velocity above this are unusable (broadside), km/s.
    pub das_v_app_max_kmps: f64,
}

impl Default for SimSettings {
    fn default() -> SimSettings {
        SimSettings {
            fs_hz: 100.0,
            noise_mps: 1.0e-6,
            das_noise_strain: 1.0e-11,
            sta_s: 0.5,
            lta_s: 10.0,
            trigger_ratio: 4.0,
            f_p_hz: 1.0,
            rise_s: 0.1,
            das_spacing_m: 100.0,
            das_half_length_km: 6.0,
            das_v_app_max_kmps: 2000.0,
        }
    }
}

/// The simulated timing of one sensing network.
#[derive(Clone, Debug, PartialEq)]
pub struct SimWarning {
    /// Sensing.
    pub sensing: Sensing,
    /// Sensors (DAS: usable channels) simulated.
    pub n_sensors: usize,
    /// Sensors that picked.
    pub n_picks: usize,
    /// Mean pick delay after the true P onset, s.
    pub pick_delay_mean_s: f64,
    /// k-th pick received at the centre, s after origin.
    pub t_kth_received_s: f64,
    /// Alert issued (after processing and dissemination), s after origin.
    pub t_alert_s: f64,
    /// Simulated S onset at the city (first sample at or after the S arrival), s.
    pub t_s_city_s: f64,
    /// Simulated warning t_S − t_alert − t_act, s.
    pub warning_s: f64,
    /// The analytic counterpart.
    pub analytic: WarningTiming,
}

impl SimWarning {
    /// Every input and output as (name, value) pairs.
    pub fn inputs(&self) -> Vec<(&'static str, f64)> {
        let mut v = self.analytic.inputs();
        v.push(("sim_t_alert_s", self.t_alert_s));
        v.push(("sim_warning_s", self.warning_s));
        v.push(("sim_n_picks", self.n_picks as f64));
        v
    }
}

/// One sensor of the simulation: arrival time of P, peak particle velocity, apparent
/// velocity (DAS only; ∞ otherwise).
#[derive(Clone, Copy, Debug)]
struct Sensor {
    t_p_s: f64,
    a_mps: f64,
    v_app_mps: f64,
}

/// Synthetic P onset u̇(τ) = A·(1 − e^(−τ/τ_r))·sin(2π·f·τ) for τ ≥ 0, m/s.
fn p_onset(a: f64, tau: f64, f_hz: f64, rise_s: f64) -> f64 {
    if tau < 0.0 {
        return 0.0;
    }
    let env = if rise_s > 0.0 {
        1.0 - fmath::exp(-tau / rise_s)
    } else {
        1.0
    };
    a * env * fmath::sin(2.0 * core::f64::consts::PI * f_hz * tau)
}

/// STA/LTA pick on one sensor: samples from `t0` (a full LTA before the arrival) to the
/// arrival plus 5 s; returns the pick time or `None`.
fn pick(sensor: &Sensor, set: &SimSettings, rng: &mut NormalStream, das: bool) -> Option<f64> {
    let fs = set.fs_hz;
    let dt = 1.0 / fs;
    let n_sta = fmath::round(set.sta_s * fs).max(1.0) as usize;
    let n_lta = fmath::round(set.lta_s * fs).max(1.0) as usize;
    // Start on the sample grid a full STA + LTA (plus a margin) before the arrival; the
    // sensors record continuously, so the history may start before the origin time.
    let k0 = fmath::floor(sensor.t_p_s * fs) as i64 - (n_lta + n_sta) as i64 - 20;
    let k_end = fmath::ceil((sensor.t_p_s + 5.0) * fs) as i64;
    let mut buf: Vec<f64> = Vec::with_capacity((k_end - k0).max(0) as usize + 1);
    // Running sums of the STA window (last n_sta samples) and the LTA window (the n_lta
    // samples before it).
    let (mut s_sta, mut s_lta) = (0.0_f64, 0.0_f64);
    for k in k0..=k_end {
        let t = k as f64 * dt;
        let u = p_onset(sensor.a_mps, t - sensor.t_p_s, set.f_p_hz, set.rise_s);
        let x = if das {
            // The fibre measures strain; processing reconstructs u̇ = −V_app·ε.
            let eps = das_strain(sensor.v_app_mps, u) + rng.next_scaled(set.das_noise_strain);
            das_particle_velocity_mps(sensor.v_app_mps, eps)
        } else {
            u + rng.next_scaled(set.noise_mps)
        };
        buf.push(x * x);
        let len = buf.len();
        s_sta += x * x;
        if len > n_sta {
            let moved = buf[len - 1 - n_sta];
            s_sta -= moved;
            s_lta += moved;
        }
        if len > n_sta + n_lta {
            s_lta -= buf[len - 1 - n_sta - n_lta];
            let sta = s_sta / n_sta as f64;
            let lta = s_lta / n_lta as f64;
            if lta > 0.0 && sta / lta >= set.trigger_ratio {
                return Some(t);
            }
        }
    }
    None
}

/// Station epicentral distances around the k-th sensor distance `r_k` (sorted; the k-th
/// is exactly `r_k`): 3 km apart below it and 5 km apart above it (8 stations).
fn station_ring(r_k: f64, k: usize) -> Vec<f64> {
    let k = k.max(1);
    let mut v = Vec::with_capacity(k + 4);
    for i in 0..k {
        v.push((r_k - 3.0 * (k - 1 - i) as f64).max(0.5));
    }
    for i in 1..=4 {
        v.push(r_k + 5.0 * f64::from(i));
    }
    v
}

/// Runs the timing simulation for sensing `s` with seed `seed` (reduced model; EXOTIC
/// §X10). Per-sensor packetisation phases and noise come from the seed.
pub fn simulate_offshore(
    case: &OffshoreCase,
    s: Sensing,
    seed: u64,
    set: &SimSettings,
) -> SimWarning {
    let analytic = case.analytic(s);
    let mut sensors: Vec<Sensor> = Vec::new();
    match s {
        Sensing::LandNetwork | Sensing::OffshoreCabled => {
            for r_epi in station_ring(case.sensor_epi_km(s), case.k_sensor) {
                let r = hypocentral_km(r_epi, case.depth_km);
                sensors.push(Sensor {
                    t_p_s: r / case.v_p_kmps,
                    a_mps: 2.0 * core::f64::consts::PI * set.f_p_hz * pd_m(case.magnitude, r),
                    v_app_mps: f64::INFINITY,
                });
            }
        }
        Sensing::Das => {
            let n_half = fmath::round(set.das_half_length_km * 1000.0 / set.das_spacing_m) as i64;
            for j in -n_half..=n_half {
                let s_km = j as f64 * set.das_spacing_m / 1000.0;
                let r_epi = fmath::hypot(case.das_offset_km, s_km);
                let r = hypocentral_km(r_epi, case.depth_km);
                let v_app = cable_apparent_velocity_kmps(case.v_p_kmps, r, s_km);
                if v_app.is_nan() || v_app > set.das_v_app_max_kmps {
                    continue;
                }
                sensors.push(Sensor {
                    t_p_s: r / case.v_p_kmps,
                    a_mps: 2.0 * core::f64::consts::PI * set.f_p_hz * pd_m(case.magnitude, r),
                    v_app_mps: v_app * 1000.0,
                });
            }
        }
    }
    let lat = case.latency;
    let mut uni = Xoshiro256pp::from_seed(seed ^ 0x4545_5754_494d_4531);
    let mut received: Vec<f64> = Vec::with_capacity(sensors.len());
    let mut delay_sum = 0.0;
    let das = s == Sensing::Das;
    // DAS channels share one interrogator, hence one packet clock; stations have their own.
    let shared_phase = uni.next_f64() * lat.packet_s;
    for (i, sen) in sensors.iter().enumerate() {
        let mut rng = NormalStream::new(Xoshiro256pp::from_seed(
            seed.wrapping_add(0x9e37_79b9_7f4a_7c15u64.wrapping_mul(i as u64 + 1)),
        ));
        let own_phase = uni.next_f64() * lat.packet_s;
        let phase = if das { shared_phase } else { own_phase };
        if let Some(tp) = pick(sen, set, &mut rng, das) {
            delay_sum += tp - sen.t_p_s;
            // The pick is confirmed after the window; its data leave in the next packet
            // (packets close at phase + n·t_pkt), then telemetry.
            let ready = tp + lat.window_s;
            let sent = if lat.packet_s > 0.0 {
                phase + lat.packet_s * fmath::ceil((ready - phase) / lat.packet_s)
            } else {
                ready
            };
            received.push(sent + lat.telemetry_s);
        }
    }
    let n_picks = received.len();
    received.sort_by(f64::total_cmp);
    let k = case.k_sensor.max(1);
    let t_kth = if n_picks >= k {
        received[k - 1]
    } else {
        f64::INFINITY
    };
    let t_alert = t_kth + lat.processing_s + lat.dissemination_s;
    let dt = 1.0 / set.fs_hz;
    let t_s = fmath::ceil(case.t_s_city_s() / dt) * dt;
    SimWarning {
        sensing: s,
        n_sensors: sensors.len(),
        n_picks,
        pick_delay_mean_s: if n_picks > 0 {
            delay_sum / n_picks as f64
        } else {
            f64::NAN
        },
        t_kth_received_s: t_kth,
        t_alert_s: t_alert,
        t_s_city_s: t_s,
        warning_s: t_s - t_alert - case.t_act_s,
        analytic,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rel(a: f64, b: f64) -> f64 {
        ((a - b) / b).abs()
    }

    // req: FAB-EXO-803
    #[test]
    fn lead_time_gain_per_km() {
        // ∂W/∂x = 1/V_p and 1/V_s (rel 1e-9, EXOTIC §X10).
        let gp = lead_time_gain_s_per_km(6.5);
        let gs = lead_time_gain_s_per_km(3.7);
        println!("inputs: V_p 6.5 km/s, V_s 3.7 km/s → gain {gp:.5} / {gs:.5} s/km");
        assert!(
            rel(gp, 0.15385) < 1e-4 && rel(gp, 1.0 / 6.5) < 1e-9,
            "0.15385 s/km at V_p 6.5 (rel 1e-9 to 1/V_p; FAB-EXO-803): {gp}"
        );
        assert!(
            rel(gs, 0.27027) < 1e-4 && rel(gs, 1.0 / 3.7) < 1e-9,
            "0.27027 s/km at V_s 3.7, PLUM (rel 1e-9 to 1/V_s; FAB-EXO-803): {gs}"
        );
        // The gain is the derivative of W with respect to sensor advance: moving the
        // reporting sensor 1 km closer (along a hypocentral ray) buys 1/V_p of warning.
        let lat = EewLatency::default_budget();
        assert!(
            rel(lat.total_s(), DEFAULT_LATENCY_S) < 1e-12,
            "3.3 s budget"
        );
        let w = |r_sensor: f64| warning_s(121.655, 3.7, t_alert_s(r_sensor, 6.5, &lat), 0.0);
        let fd = w(50.0) - w(51.0);
        assert!(
            rel(fd, gp) < 1e-9,
            "finite difference {fd} = 1/V_p (rel 1e-9)"
        );
        // Blind zone r_b = √((V_s·t_alert)² − h²): sign flip exactly at r_b.
        let (h, t_alert) = (10.0, 5.0);
        let rb = blind_zone_km(3.5, t_alert, h);
        assert!(
            rel(rb, (17.5f64 * 17.5 - 100.0).sqrt()) < 1e-12,
            "closed form"
        );
        let w_at = |r_epi: f64| warning_s(hypocentral_km(r_epi, h), 3.5, t_alert, 0.0);
        assert!(
            w_at(rb - 0.01) < 0.0 && w_at(rb + 0.01) > 0.0,
            "W changes sign at r_b"
        );
        assert!(w_at(rb).abs() < 1e-9, "W(r_b) = 0");
        assert_eq!(
            blind_zone_km(3.5, 1.0, 10.0),
            0.0,
            "deeper than V_s·t_alert: 0"
        );
        // The worked case prints its inputs with every output.
        let a = OffshoreCase::reference().analytic(Sensing::OffshoreCabled);
        let names: Vec<&str> = a.inputs().iter().map(|(n, _)| *n).collect();
        for need in [
            "v_p_kmps",
            "v_s_kmps",
            "depth_km",
            "latency_s",
            "sensor_epi_km",
        ] {
            assert!(names.contains(&need), "output prints input {need}");
        }
        // DAS: u̇ = −V_app·ε inverts ε = −u̇/V_app; broadside carries no axial strain.
        let v_app = 140.0e3;
        let eps = das_strain(v_app, 0.02);
        assert!(rel(das_particle_velocity_mps(v_app, eps), 0.02) < 1e-12);
        assert_eq!(das_strain(f64::INFINITY, 0.02), 0.0);
        assert!(cable_apparent_velocity_kmps(6.5, 21.5, 0.0).is_infinite());
    }

    // req: FAB-EXO-804
    #[test]
    fn offshore_m8_warning() {
        // Inputs (EXOTIC §X10): epicentre 100 km offshore, 20 km deep, city 20 km inland,
        // V_p 6.5 km/s, V_s 3.7 km/s, 4th sensor, latency 3.3 s, t_act 0.
        let case = OffshoreCase::reference();
        assert!(
            (case.t_s_city_s() - 32.9).abs() < 0.05,
            "S at the city 32.9 s: {}",
            case.t_s_city_s()
        );
        let table = [
            (Sensing::LandNetwork, 20.5, 12.4),
            (Sensing::OffshoreCabled, 9.4, 23.5),
            (Sensing::Das, 6.6, 26.3),
        ];
        for (s, t_alert, w) in table {
            let a = case.analytic(s);
            println!(
                "{:5} analytic: sensor {:.1} km epi ({:.2} km hyp), alert {:.2} s, W {:.2} s, r_b {:.1} km",
                s.name(),
                case.sensor_epi_km(s),
                a.r_sensor_hyp_km,
                a.t_alert_s,
                a.warning_s,
                a.blind_zone_km
            );
            assert!(
                (a.t_alert_s - t_alert).abs() <= 0.05 && (a.warning_s - w).abs() <= 0.05,
                "{}: analytic alert {:.2} s / W {:.2} s vs the EXOTIC §X10 table {t_alert} / {w} (±0.05 s rounding)",
                s.name(),
                a.t_alert_s,
                a.warning_s
            );
            for seed in [42_u64, 7, 1234] {
                let sim = simulate_offshore(&case, s, seed, &SimSettings::default());
                println!(
                    "      seed {seed}: {} sensors, {} picks, pick delay {:.3} s, alert {:.2} s, W {:.2} s",
                    sim.n_sensors, sim.n_picks, sim.pick_delay_mean_s, sim.t_alert_s, sim.warning_s
                );
                assert!(
                    sim.n_picks >= case.k_sensor,
                    "{}: at least k = {} picks",
                    s.name(),
                    case.k_sensor
                );
                assert!(
                    (sim.warning_s - a.warning_s).abs() <= 1.0,
                    "{} seed {seed}: simulated W {:.2} s within ±1 s of the analytic {:.2} s (FAB-EXO-804)",
                    s.name(),
                    sim.warning_s,
                    a.warning_s
                );
            }
        }
        // Deterministic per seed.
        let a = simulate_offshore(&case, Sensing::Das, 42, &SimSettings::default());
        let b = simulate_offshore(&case, Sensing::Das, 42, &SimSettings::default());
        assert_eq!(a, b);
    }
}
