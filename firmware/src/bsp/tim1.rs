//! TIM1: LSM 3-phase inverter (RM0440 §28 "Advanced-control timers").
//!
//! * Centre-aligned mode 1, ARR = 4250 at 170 MHz → f_PWM = 170 MHz / (2·4250) = 20 kHz
//!   = the base rate (core design §F; `design.power.pwm_hz`, ENV-002).
//! * CH1–CH3 with complementary outputs and hardware dead time; the duty registers are
//!   preloaded, so a duty written in base tick k is applied from the next update event
//!   (k + 1, the ZOH of core design §A.5).
//! * **STO** = MOE cleared with OSSI = 1 (all six switches off at their idle level) plus
//!   the LSM gate enable low: two independent disable paths (IEC 61800-5-2 STO intent;
//!   not a certified function). **ASC** = OC1–3REF forced inactive with MOE = 1, so every
//!   low side conducts after the dead time (back-EMF braking, core design F9).
//! * Break inputs: BKIN2 ← COMP1–3 (phase overcurrent), BKIN ← COMP7 (DC bus). A break
//!   clears MOE in hardware (AOE = 0: it stays off). The break flags latch until the
//!   core commands STO, which clears them; only then can Run/ASC re-arm MOE.
//! * CH4 is internal only: OC4REF (PWM mode 1, CCR4 = ARR − 1) is TRGO, so its rising
//!   edge falls two counts after the counter peak, where the low sides conduct and the
//!   low-side shunts carry the phase currents. TRGO triggers the injected ADC sequences
//!   (JEXTSEL 0 = TIM1_TRGO, RM0440 Tables 163/165) and synchronises the HRTIM timers
//!   (HRTIM_MCR.SYNCIN = internal TIM1_TRGO), so coil and LSM PWM share one period.

use stm32g4xx_hal::rcc::Rcc;
use stm32g4xx_hal::stm32::TIM1;

use super::pins;

/// Auto-reload value (half PWM period in counts).
pub const ARR: u32 = 4250;

/// Dead time, DTG code: DTG[7] = 0 → DT = DTG·t_DTS = 85 × 5.88 ns = 500 ns at
/// t_DTS = 1/170 MHz (RM0440 §28.6.20). A power-stage property (gate driver and switch
/// turn-off time) fixed here until the board exists; see README "open items".
const DTG: u32 = 85;

// CR1 (RM0440 §28.6.1).
const CEN: u32 = 1 << 0;
const CMS_CENTER1: u32 = 0b01 << 5;
const ARPE: u32 = 1 << 7;
// CR2: MMS[2:0] = 111 → TRGO = OC4REF.
const MMS_OC4REF: u32 = 0b111 << 4;
// CCMR OCxM (3-bit encodings; bit 3 of OCxM stays 0).
const OCM_FORCE_INACTIVE: u32 = 0b100;
const OCM_PWM1: u32 = 0b110;
const OCPE: u32 = 1 << 3;
// CCER.
const CC_OUTPUTS: u32 = (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6) | (1 << 8) | (1 << 10);
// BDTR (RM0440 §28.6.20).
const LOCK1: u32 = 0b01 << 8;
const OSSI: u32 = 1 << 10;
const OSSR: u32 = 1 << 11;
const BKE: u32 = 1 << 12;
const BKP_HIGH: u32 = 1 << 13;
const MOE: u32 = 1 << 15;
const BKF_4: u32 = 0b0010 << 16; // f_CK_INT, N = 4 (≈ 24 ns glitch filter)
const BK2F_4: u32 = 0b0010 << 20;
const BK2E: u32 = 1 << 24;
const BK2P_HIGH: u32 = 1 << 25;
// AF1/AF2 (RM0440 §28.6.27–28): comparator sources of BKIN / BKIN2; BKINE/BK2INE (bit 0,
// set at reset) cleared because no external break pin is wired.
const AF1_BKCMP7E: u32 = 1 << 7;
const AF2_BK2CMP123E: u32 = (1 << 1) | (1 << 2) | (1 << 3);
// SR break flags (rc_w0).
const BIF: u32 = 1 << 7;
const B2IF: u32 = 1 << 8;
// EGR.
const UG: u32 = 1 << 0;

const fn ccmr_pair(m_lo: u32, m_hi: u32) -> u32 {
    (m_lo << 4) | OCPE | (m_hi << 12) | (OCPE << 8)
}

/// What the inverter is doing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InverterMode {
    /// Safe Torque Off (MOE = 0, gate enable low).
    Sto,
    /// PWM on the three phases.
    Run,
    /// Active short circuit (low sides on).
    Asc,
}

/// The LSM inverter timer.
pub struct Tim1 {
    tim: TIM1,
    mode: InverterMode,
}

impl Tim1 {
    /// Configures TIM1 (counting, TRGO running) with the outputs in STO.
    pub fn init(tim: TIM1, rcc: &Rcc) -> Tim1 {
        rcc.apb2enr().modify(|_, w| w.tim1en().set_bit());
        let half = ARR / 2;
        // SAFETY: TIM1 is owned here and configured once in init (interrupts disabled).
        // All values are valid encodings of the RM0440 §28.6 fields named above; BDTR is
        // written once including LOCK level 1, as the reference manual requires.
        unsafe {
            tim.cr1().write(|w| w.bits(CMS_CENTER1 | ARPE));
            tim.psc().write(|w| w.bits(0));
            tim.arr().write(|w| w.bits(ARR));
            tim.rcr().write(|w| w.bits(0));
            tim.ccmr1_output()
                .write(|w| w.bits(ccmr_pair(OCM_PWM1, OCM_PWM1)));
            tim.ccmr2_output()
                .write(|w| w.bits(ccmr_pair(OCM_PWM1, OCM_PWM1)));
            tim.ccr1().write(|w| w.bits(half));
            tim.ccr2().write(|w| w.bits(half));
            tim.ccr3().write(|w| w.bits(half));
            tim.ccr4().write(|w| w.bits(ARR - 1));
            tim.ccer().write(|w| w.bits(CC_OUTPUTS));
            tim.cr2().write(|w| w.bits(MMS_OC4REF));
            tim.af1().write(|w| w.bits(AF1_BKCMP7E));
            tim.af2().write(|w| w.bits(AF2_BK2CMP123E));
            tim.bdtr().write(|w| {
                w.bits(
                    DTG | LOCK1 | OSSI | OSSR | BKE | BKP_HIGH | BKF_4 | BK2F_4 | BK2E | BK2P_HIGH,
                )
            });
            tim.egr().write(|w| w.bits(UG));
            tim.sr().write(|w| w.bits(0));
            tim.cr1().modify(|r, w| w.bits(r.bits() | CEN));
        }
        pins::LSM_GATE_EN.set(false);
        Tim1 {
            tim,
            mode: InverterMode::Sto,
        }
    }

    /// Phase duty registers (0..=ARR), applied from the next update event.
    #[inline]
    pub fn set_compare(&mut self, ccr: [u32; 3]) {
        let [a, b, c] = ccr.map(|x| x.min(ARR));
        // SAFETY: CCR1–3 of the owned TIM1 take any value 0..=ARR (clamped above).
        unsafe {
            self.tim.ccr1().write(|w| w.bits(a));
            self.tim.ccr2().write(|w| w.bits(b));
            self.tim.ccr3().write(|w| w.bits(c));
        }
    }

    /// A break (hardware overcurrent) has latched since the last STO.
    #[inline]
    pub fn break_latched(&self) -> bool {
        self.tim.sr().read().bits() & (BIF | B2IF) != 0
    }

    /// Main output enable state (false after STO or a hardware break).
    #[inline]
    pub fn outputs_enabled(&self) -> bool {
        self.tim.bdtr().read().bits() & MOE != 0
    }

    fn set_oc_modes(&mut self, m: u32) {
        // SAFETY: OCxM of the owned TIM1 (not preloaded with CCPC = 0, so the change is
        // immediate); valid 3-bit encodings.
        unsafe {
            self.tim.ccmr1_output().write(|w| w.bits(ccmr_pair(m, m)));
            self.tim
                .ccmr2_output()
                .write(|w| w.bits(ccmr_pair(m, OCM_PWM1)));
        }
    }

    fn arm(&mut self) -> bool {
        if self.break_latched() {
            return false;
        }
        pins::LSM_GATE_EN.set(true);
        // SAFETY: setting MOE on the owned TIM1; LOCK level 1 leaves MOE writable.
        unsafe { self.tim.bdtr().modify(|r, w| w.bits(r.bits() | MOE)) };
        true
    }

    /// PWM on the three phases. Returns false (outputs stay off) while a break is
    /// latched.
    pub fn run(&mut self) -> bool {
        if self.mode != InverterMode::Run {
            self.set_oc_modes(OCM_PWM1);
        }
        self.mode = InverterMode::Run;
        self.arm()
    }

    /// Active short circuit. Returns false (outputs stay off, i.e. STO) while a break
    /// is latched.
    pub fn asc(&mut self) -> bool {
        if self.mode != InverterMode::Asc {
            self.set_oc_modes(OCM_FORCE_INACTIVE);
        }
        self.mode = InverterMode::Asc;
        self.arm()
    }

    /// Safe Torque Off; clears the latched break flags so a later Run/ASC can re-arm.
    pub fn sto(&mut self) {
        // SAFETY: clearing MOE and the rc_w0 break flags of the owned TIM1 (writing 1 to
        // the other rc_w0 flags leaves them unchanged).
        unsafe {
            self.tim.bdtr().modify(|r, w| w.bits(r.bits() & !MOE));
            self.tim.sr().write(|w| w.bits(!(BIF | B2IF)));
        }
        pins::LSM_GATE_EN.set(false);
        self.mode = InverterMode::Sto;
    }
}
