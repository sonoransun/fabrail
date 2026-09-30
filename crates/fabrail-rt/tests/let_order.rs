//! LET permutation invariance (FAB-FW-004; plan §2b item 9; PHYSICS §24).
//!
//! The base task always runs first in a tick; the groups due at the same tick (fast,
//! supervisor, seismic) then run in each of the 3! = 6 orders. Because a job released at
//! r reads only mailbox values active at ≤ r and publishes values that activate at
//! r + P, every order must issue bit-identical HAL outputs: the FNV-1a hash over every
//! output (the selftest hash definition) is the same for all six.

use fabrail_core::seismic::action_policy::CorePolicy;
use fabrail_params::{Rates, SystemParams, presets};
use fabrail_rt::{Executive, GroupId, LcgBoard, SELFTEST_TICKS, selftest_hash, selftest_params};

/// Hash of `n` ticks of the executive for `p`, with the due groups run in `order`.
fn hash_in_order(p: &SystemParams, n: u32, order: [GroupId; 3]) -> u64 {
    let mut ex = Executive::new(p).unwrap_or_else(|e| panic!("params rejected: {e}"));
    let mut b = LcgBoard::new();
    for _ in 0..n {
        ex.on_base_tick_in_order(&mut b, &mut CorePolicy, order);
        b.advance();
    }
    b.hash()
}

// req: FAB-FW-004
#[test]
fn permutation_invariant() {
    // 4 000 ticks: 2 000 fast, 200 supervisor and 20 seismic releases, every seismic
    // release coinciding with a supervisor and a fast release.
    let n = 4_000;
    let mut cases: Vec<(&str, SystemParams)> = vec![("door (selftest)", selftest_params())];
    let mut explicit = selftest_params();
    explicit.control.rates = Rates {
        base_hz: 20_000,
        fast_decim: 1,
        sup_decim: 20,
        seismic_decim: 200,
        coil_mode_voltage: true,
    };
    cases.push(("door, fast = base", explicit));
    let mut wall = presets::wall();
    wall.control.rates.coil_mode_voltage = true;
    cases.push(("wall (5 kHz fast)", wall));

    for (name, p) in &cases {
        let reference = hash_in_order(p, n, GroupId::DEFAULT_ORDER);
        println!("{name}: LET reference hash = {reference:#018x}");
        for order in GroupId::ALL_ORDERS {
            let h = hash_in_order(p, n, order);
            assert_eq!(
                h, reference,
                "(E) bit-identical outputs for order {order:?} ({name}); FAB-FW-004, PLAN §2b.9"
            );
        }
    }
    // The default order is the selftest executive.
    assert_eq!(
        hash_in_order(&selftest_params(), SELFTEST_TICKS, GroupId::DEFAULT_ORDER),
        selftest_hash(SELFTEST_TICKS),
        "(E) on_base_tick uses the default order"
    );
    // A malformed order still runs each due group exactly once per tick.
    let dup = [GroupId::Seismic, GroupId::Seismic, GroupId::Fast];
    assert_eq!(
        hash_in_order(&selftest_params(), n, dup),
        hash_in_order(&selftest_params(), n, GroupId::DEFAULT_ORDER),
        "(E) duplicates are ignored; the missing group runs afterwards"
    );
}
