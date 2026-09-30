//! Contract: every preset round-trips through JSON bit-exactly, unknown fields are
//! rejected, enums are snake_case, and a round-tripped preset still validates and holds
//! its seal (core design §E.2 `tests/serde_roundtrip`; `Params.from_json/to_json`).
//!
//! Needs `--features serde`; without it this file is empty.
#![cfg(feature = "serde")]

use fabrail_params::{SystemParams, crc, meta, presets};

/// True when serde_json parses floats correctly rounded (its `float_roundtrip` feature).
/// Without it, 17-significant-digit decimals can come back 1 ulp off (e.g. the DOOR open
/// cap √0.0338 = 0.18384776310850234).
fn parser_is_exact() -> bool {
    let x = 0.183_847_763_108_502_34_f64;
    let s = serde_json::to_string(&x).expect("serialise");
    serde_json::from_str::<f64>(&s).map(f64::to_bits).ok() == Some(x.to_bits())
}

/// Leaf-by-leaf comparison: bit-exact with an exact parser, else within 1 ulp.
fn assert_same(c: fabrail_types::RailClass, a: &SystemParams, b: &SystemParams, exact: bool) {
    for (i, f) in meta::FIELDS.iter().enumerate() {
        let (x, y) = (
            meta::value_at(a, i).expect("getter"),
            meta::value_at(b, i).expect("getter"),
        );
        let ulps = (x.to_bits() as i64 - y.to_bits() as i64).unsigned_abs();
        let limit = if exact { 0 } else { 1 };
        assert!(
            ulps <= limit,
            "{c:?}: {} = {x:e} came back as {y:e} ({ulps} ulp; limit {limit})",
            f.path
        );
    }
}

#[test]
fn json_roundtrip() {
    let exact = parser_is_exact();
    if !exact {
        eprintln!(
            "note: serde_json lacks `float_roundtrip`; JSON round trips are checked to 1 ulp \
             (enable the feature for bit-exact Params.from_json/to_json)"
        );
    }
    for c in presets::CLASSES {
        let p = presets::by_class(c);
        let s = serde_json::to_string(&p).expect("serialise");
        let back: SystemParams = serde_json::from_str(&s).expect("deserialise");
        assert_same(c, &p, &back, exact);
        if exact {
            assert_eq!(back, p, "{c:?}: JSON round trip is exact");
        }
        assert_eq!(
            back.validate(),
            Ok(()),
            "{c:?}: round-tripped preset validates"
        );
        let pretty = serde_json::to_string_pretty(&p).expect("pretty");
        let back2: SystemParams = serde_json::from_str(&pretty).expect("deserialise pretty");
        assert_same(c, &p, &back2, exact);
        if exact {
            assert_eq!(
                crc::verify_seal(&back),
                Ok(()),
                "{c:?}: seal survives the round trip"
            );
        }
        let v = serde_json::to_value(p).expect("value");
        assert_eq!(
            v.pointer("/design/class").and_then(|x| x.as_str()),
            Some(presets::class_name(c)),
            "{c:?}: class serialises as its lowercase name"
        );
    }
}

#[test]
fn unknown_and_missing_fields_are_rejected() {
    let p = presets::door();
    let s = serde_json::to_string(&p).expect("serialise");
    let typo = s.replacen("\"g_star_m\"", "\"g_star\"", 1);
    assert!(
        serde_json::from_str::<SystemParams>(&typo).is_err(),
        "unknown field rejected"
    );
    let mut v = serde_json::to_value(p).expect("value");
    if let Some(obj) = v
        .pointer_mut("/design/suspension")
        .and_then(|x| x.as_object_mut())
    {
        obj.remove("g_up_m");
    }
    assert!(
        serde_json::from_value::<SystemParams>(v).is_err(),
        "missing field rejected (no silent defaults)"
    );
}

#[test]
fn edited_json_needs_a_reseal() {
    let p = presets::drawer();
    let mut v = serde_json::to_value(p).expect("value");
    *v.pointer_mut("/safety/v_open_max_mps").expect("pointer") = serde_json::json!(0.2);
    let mut q: SystemParams = serde_json::from_value(v).expect("deserialise");
    assert!(
        crc::verify_seal(&q).is_err(),
        "an edited safety block fails the seal until resealed (SAF-015)"
    );
    crc::seal(&mut q);
    assert_eq!(crc::verify_seal(&q), Ok(()));
    assert_eq!(q.validate(), Ok(()), "a lower cap is still valid");
}
