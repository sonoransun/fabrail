//! The golden selftest hash (FAB-FW-005; plan §2b item 15; core design §A.4 item 7).
//!
//! `GOLDEN_SELFTEST_HASH` is a placeholder (0) until Wave 3 (C1) freezes it. Until then
//! this test PRINTS the current hash and asserts equality only when the constant is
//! nonzero. The "identical with and without `fault-injection`" requirement is checked
//! by running this test in both builds against the same value:
//!
//! ```text
//! H=$(cargo test -p fabrail-rt --test golden -- --nocapture 2>&1 | sed -n 's/.*selftest_hash(2000) = \(0x[0-9a-f]*\).*/\1/p')
//! FABRAIL_EXPECT_SELFTEST_HASH=$H cargo test -p fabrail-rt --features fault-injection --test golden
//! ```
//!
//! Once C1 freezes the constant, both builds assert against it directly.

use fabrail_rt::{
    Executive, GOLDEN_SELFTEST_HASH, LcgBoard, SELFTEST_ERROR_HASH, SELFTEST_TICKS, selftest_hash,
    selftest_params,
};

fn parse_hex(s: &str) -> Option<u64> {
    let s = s.trim();
    let s = s.strip_prefix("0x").unwrap_or(s);
    u64::from_str_radix(s, 16).ok()
}

// req: FAB-FW-005
#[test]
fn selftest_hash_frozen() {
    let h = selftest_hash(SELFTEST_TICKS);
    println!(
        "selftest_hash({SELFTEST_TICKS}) = {h:#018x} (fault-injection feature: {})",
        cfg!(feature = "fault-injection")
    );
    assert_ne!(h, SELFTEST_ERROR_HASH, "the selftest executive builds");
    assert_ne!(h, 0, "a real hash");
    assert_eq!(h, selftest_hash(SELFTEST_TICKS), "(E) deterministic");
    if GOLDEN_SELFTEST_HASH != 0 {
        assert_eq!(
            h, GOLDEN_SELFTEST_HASH,
            "(E) SIL ≡ firmware: selftest hash equals the frozen golden value, FAB-FW-005"
        );
    } else {
        println!("GOLDEN_SELFTEST_HASH is the Wave-0 placeholder 0: C1 freezes {h:#018x}");
    }
    if let Some(expect) = std::env::var("FABRAIL_EXPECT_SELFTEST_HASH")
        .ok()
        .as_deref()
        .and_then(parse_hex)
    {
        assert_eq!(
            h, expect,
            "(E) identical with and without the fault-injection feature, FAB-FW-005"
        );
    }
}

/// With the feature compiled in, an injection that was set and cleared before the run
/// leaves the hash untouched, and an active injection does change the outputs (the hook
/// is live and sits before the monitor).
#[cfg(feature = "fault-injection")]
#[test]
fn injection_hook_is_inert_when_clear() {
    use fabrail_rt::CoreFault;

    let run = |setup: &dyn Fn(&mut Executive)| {
        let mut ex = Executive::new(&selftest_params()).unwrap_or_else(|e| panic!("{e}"));
        setup(&mut ex);
        let mut b = LcgBoard::new();
        for _ in 0..SELFTEST_TICKS {
            ex.on_base_tick(&mut b);
            b.advance();
        }
        (b.hash(), ex)
    };
    let (clean, _) = run(&|_| {});
    let (cleared, _) = run(&|ex| {
        ex.inject(CoreFault::Nan);
        ex.clear_injection();
    });
    assert_eq!(clean, selftest_hash(SELFTEST_TICKS));
    assert_eq!(cleared, clean, "(E) a cleared injection is inert");
    let (nan, ex) = run(&|ex| ex.inject(CoreFault::Nan));
    assert_ne!(nan, clean, "an active injection perturbs the outputs");
    assert!(
        ex.snapshot()
            .fast
            .verdict
            .faults
            .contains(fabrail_types::FaultCode::NonFinite),
        "(T) the monitor sees the injected NaN (injection happens before the monitor)"
    );
}

#[cfg(not(feature = "fault-injection"))]
#[test]
fn lcg_board_is_the_selftest_board() {
    let mut ex = Executive::new(&selftest_params()).unwrap_or_else(|e| panic!("{e}"));
    let mut b = LcgBoard::new();
    for _ in 0..SELFTEST_TICKS {
        ex.on_base_tick(&mut b);
        b.advance();
    }
    assert_eq!(b.hash(), selftest_hash(SELFTEST_TICKS));
}
