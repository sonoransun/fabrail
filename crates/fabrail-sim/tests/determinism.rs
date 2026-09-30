//! Determinism of the SIL harness (FAB-SW-002; core design §A.4 item 5, plan §2b item
//! 16): the same (parameters, scenario, seed) gives byte-identical run data, and batch
//! results are invariant to the worker count. The subprocess half of FAB-SW-002
//! (SHA-256 of the JSON across two processes) is the Python test
//! `py:determinism::same_seed_identical_json`.

mod common;

use fabrail_sim::{RunData, run, run_batch};

/// Every deterministic byte of a run (fingerprint plus the explicit pieces, so a
/// failure names what differs).
fn assert_identical(a: &RunData, b: &RunData, what: &str) {
    assert_eq!(a.channel_names(), b.channel_names(), "{what}: channel set");
    for (ca, cb) in a.channels.iter().zip(&b.channels) {
        assert_eq!(
            ca.data.to_le_bytes(),
            cb.data.to_le_bytes(),
            "{what}: channel {} bytes differ ((E) byte-identical, FAB-SW-002)",
            ca.name
        );
    }
    assert_eq!(a.events.len(), b.events.len(), "{what}: event count");
    for (ea, eb) in a.events.iter().zip(&b.events) {
        assert!(
            ea.kind == eb.kind
                && ea.t_s.to_bits() == eb.t_s.to_bits()
                && ea.a.to_bits() == eb.a.to_bits()
                && ea.b.to_bits() == eb.b.to_bits(),
            "{what}: event {ea:?} vs {eb:?} ((E) bit-identical, FAB-SW-002)"
        );
    }
    let ka: Vec<_> = a
        .kpis
        .iter()
        .map(|(k, v)| (k.clone(), v.to_bits()))
        .collect();
    let kb: Vec<_> = b
        .kpis
        .iter()
        .map(|(k, v)| (k.clone(), v.to_bits()))
        .collect();
    assert_eq!(ka, kb, "{what}: KPIs ((E) bit-identical, FAB-SW-002)");
    assert_eq!(
        a.meta.hashed_json(),
        b.meta.hashed_json(),
        "{what}: provenance without wall_s (plan §2b item 16)"
    );
    assert_eq!(a.fingerprint(), b.fingerprint(), "{what}: fingerprint");
}

// req: FAB-SW-002
#[test]
fn same_seed_identical() {
    for (name, (p, s)) in [
        ("door carriage6 voltage", common::door_case(42, 0.12)),
        ("wall travel1d", common::wall_case(42, 0.4)),
    ] {
        let a = run(&p, &s).expect("first run");
        let b = run(&p, &s).expect("second run");
        assert!(
            a.n_samples() > 10 && !a.channels.is_empty(),
            "{name}: recorded"
        );
        assert!(
            a.events.iter().any(|e| e.kind == "scenario:network_alert"),
            "{name}: events applied"
        );
        assert_identical(&a, &b, name);
        // A cloned scenario and parameter set (fresh allocations) give the same bytes.
        let c = run(&p.clone(), &s.clone()).expect("third run");
        assert_identical(&a, &c, name);
    }
    // The seed is part of the provenance: another seed is another fingerprint.
    let (p, s1) = common::door_case(1, 0.05);
    let (_, s2) = common::door_case(2, 0.05);
    assert_ne!(
        run(&p, &s1).expect("run").fingerprint(),
        run(&p, &s2).expect("run").fingerprint()
    );
}

// req: FAB-SW-002
#[test]
fn batch_worker_invariance() {
    let jobs: Vec<_> = (0..6u64)
        .map(|i| {
            if i % 2 == 0 {
                common::door_case(100 + i, 0.05)
            } else {
                common::wall_case(100 + i, 0.2)
            }
        })
        .collect();
    let reference: Vec<u64> = jobs
        .iter()
        .map(|(p, s)| run(p, s).expect("sequential run").fingerprint())
        .collect();
    for workers in [Some(1), Some(2), Some(4), Some(0), None] {
        let got: Vec<u64> = run_batch(&jobs, workers)
            .into_iter()
            .map(|r| r.expect("batch run").fingerprint())
            .collect();
        assert_eq!(
            got, reference,
            "workers {workers:?}: results in index order and identical to sequential runs \
             ((E) FAB-SW-002, core design §A.4 item 5)"
        );
    }
}
