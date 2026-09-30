//! Trace replay (FAB-FW-006; core design §A.4 item 7): the HAL inputs a SIL run's
//! executive received, fed to a fresh executive through a `ReplayBoard`, reproduce
//! every HAL output bit for bit. Claimed in voltage coil mode only (plan §2b item 16),
//! the mode in which the firmware runs the same base-task current loop.

mod common;

use fabrail_sim::replay::{ControlAction, HalOutput, record, replay};

// req: FAB-FW-006
#[test]
fn bit_identical() {
    let (p, base) = common::door_case(7, 0.15);
    assert!(
        base.spec.fidelity.coil.is_voltage() && p.control.rates.coil_mode_voltage,
        "replay equality is claimed in voltage coil mode only (plan §2b item 16)"
    );
    // Add a controller reboot and a transient core-fault injection so the replay also
    // covers the executive-level actions.
    let mut spec = base.spec.clone();
    spec.events.push(fabrail_sim::EventSpec::CoreFault {
        t_s: 0.06,
        what: fabrail_sim::CoreFaultKind::IqGainX3,
        dur_s: Some(0.02),
    });
    spec.events.push(fabrail_sim::EventSpec::Reset {
        t_s: 0.1,
        cause: fabrail_sim::RebootCause::Watchdog,
    });
    let s = fabrail_sim::Scenario::from_spec(spec, &base.series_bytes()).expect("scenario");

    let (data, trace) = record(&p, &s).expect("recorded run");
    assert_eq!(trace.ticks, s.spec.duration_ticks());
    assert!(data.n_samples() > 0);
    assert!(
        trace.outputs.len() as u64 >= trace.ticks,
        "at least one coil command per base tick"
    );
    assert!(trace.outputs.iter().any(|o| matches!(o, HalOutput::Lsm(_))));
    assert!(
        !trace.fast.is_empty(),
        "FastRaw latched at every fast release"
    );
    assert!(
        trace
            .control
            .iter()
            .any(|(_, a)| *a == ControlAction::Reboot)
            && trace
                .control
                .iter()
                .any(|(_, a)| matches!(a, ControlAction::Inject(_)))
            && trace
                .control
                .iter()
                .any(|(_, a)| *a == ControlAction::ClearInjection),
        "reboot and injection recorded"
    );

    let out = replay(&trace).expect("replay");
    assert_eq!(
        out.diverged_at, None,
        "every HAL call matched the recording"
    );
    assert!(!out.exhausted, "the recording covered every HAL call");
    assert_eq!(
        out.consumed,
        trace.inputs.len(),
        "every recorded input consumed"
    );
    assert_eq!(
        out.first_mismatch(&trace.outputs),
        None,
        "(E) outputs bit-identical, FAB-FW-006 / core design §A.4 item 7"
    );
    assert_eq!(out.outputs_digest(), trace.outputs_digest());

    // A second replay of the same trace is identical too (the replay is a pure
    // function of the recording).
    let again = replay(&trace).expect("replay");
    assert_eq!(again.outputs_digest(), out.outputs_digest());
}
