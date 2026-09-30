//! Contract: `meta::FIELDS` describes every parameter leaf of `SystemParams`, in
//! declaration order, and nothing else; each getter returns that leaf's value; every
//! preset lies inside every range (core design §B.2, §E.2 `tests/meta_complete`).
//!
//! The leaf set is recovered from the derived `Debug` output (struct field names in
//! declaration order, arrays indexed), so this runs with default features. With
//! `--features serde` the serde JSON leaves are checked too.

use fabrail_params::{SystemParams, meta, presets};

/// One leaf of the Debug tree: dotted path and its scalar text.
#[derive(Debug, Clone, PartialEq)]
struct Leaf {
    path: String,
    text: String,
}

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Word(String),
    Open,
    Close,
    LBracket,
    RBracket,
    Colon,
    Comma,
}

fn tokenize(s: &str) -> Vec<Tok> {
    let mut out = Vec::new();
    let mut word = String::new();
    let flush = |w: &mut String, out: &mut Vec<Tok>| {
        if !w.is_empty() {
            out.push(Tok::Word(core::mem::take(w)));
        }
    };
    for ch in s.chars() {
        let t = match ch {
            '{' => Some(Tok::Open),
            '}' => Some(Tok::Close),
            '[' => Some(Tok::LBracket),
            ']' => Some(Tok::RBracket),
            ':' => Some(Tok::Colon),
            ',' => Some(Tok::Comma),
            c if c.is_whitespace() => {
                flush(&mut word, &mut out);
                None
            }
            c => {
                word.push(c);
                None
            }
        };
        if let Some(t) = t {
            flush(&mut word, &mut out);
            out.push(t);
        }
    }
    flush(&mut word, &mut out);
    out
}

struct Parser {
    toks: Vec<Tok>,
    pos: usize,
    leaves: Vec<Leaf>,
}

impl Parser {
    fn peek(&self) -> Option<&Tok> {
        self.toks.get(self.pos)
    }

    fn next(&mut self) -> Tok {
        let t = self.toks[self.pos].clone();
        self.pos += 1;
        t
    }

    fn join(prefix: &str, seg: &str) -> String {
        if prefix.is_empty() {
            seg.to_string()
        } else {
            format!("{prefix}.{seg}")
        }
    }

    fn value(&mut self, path: &str) {
        match self.next() {
            Tok::LBracket => {
                let mut k = 0usize;
                while self.peek() != Some(&Tok::RBracket) {
                    self.value(&Self::join(path, &k.to_string()));
                    k += 1;
                    if self.peek() == Some(&Tok::Comma) {
                        self.next();
                    }
                }
                self.next();
            }
            Tok::Word(w) => {
                if self.peek() == Some(&Tok::Open) {
                    // Struct `Name { field: value, ... }`.
                    self.next();
                    while self.peek() != Some(&Tok::Close) {
                        let field = match self.next() {
                            Tok::Word(f) => f,
                            t => panic!("expected a field name, got {t:?}"),
                        };
                        assert_eq!(self.next(), Tok::Colon, "field {field}");
                        self.value(&Self::join(path, &field));
                        if self.peek() == Some(&Tok::Comma) {
                            self.next();
                        }
                    }
                    self.next();
                } else {
                    self.leaves.push(Leaf {
                        path: path.to_string(),
                        text: w,
                    });
                }
            }
            t => panic!("unexpected token {t:?} at {path}"),
        }
    }
}

fn leaves(p: &SystemParams) -> Vec<Leaf> {
    let mut parser = Parser {
        toks: tokenize(&format!("{p:?}")),
        pos: 0,
        leaves: Vec::new(),
    };
    parser.value("");
    assert_eq!(parser.pos, parser.toks.len(), "whole Debug output consumed");
    parser.leaves
}

#[test]
fn every_leaf_has_meta() {
    for c in presets::CLASSES {
        let p = presets::by_class(c);
        let got: Vec<String> = leaves(&p).into_iter().map(|l| l.path).collect();
        let want: Vec<&str> = meta::FIELDS.iter().map(|f| f.path).collect();
        for path in &got {
            assert!(
                want.contains(&path.as_str()),
                "{c:?}: struct leaf {path} has no meta::FIELDS entry"
            );
        }
        for path in &want {
            assert!(
                got.iter().any(|g| g == path),
                "{c:?}: meta::FIELDS entry {path} is not a struct leaf"
            );
        }
        assert_eq!(got.len(), want.len(), "one entry per leaf");
        assert!(
            got.iter().zip(want.iter()).all(|(g, w)| g == w),
            "meta::FIELDS follows declaration order"
        );
    }
}

#[test]
fn getters_return_the_leaf_values() {
    for c in presets::CLASSES {
        let p = presets::by_class(c);
        for (i, leaf) in leaves(&p).iter().enumerate() {
            let v = meta::value_at(&p, i).expect("getter exists");
            match leaf.text.as_str() {
                "true" => assert_eq!(v, 1.0, "{c:?}: {} flag", leaf.path),
                "false" => assert_eq!(v, 0.0, "{c:?}: {} flag", leaf.path),
                t => {
                    if let Ok(x) = t.parse::<f64>() {
                        assert_eq!(v.to_bits(), x.to_bits(), "{c:?}: {} = {t}", leaf.path);
                    } else {
                        // Unit enum variant: the getter returns its wire discriminant.
                        let f = &meta::FIELDS[i];
                        assert!(
                            v >= f.min && v <= f.max && v == v.trunc(),
                            "{c:?}: enum {} = {t} → {v}",
                            leaf.path
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn presets_lie_inside_every_range() {
    for c in presets::CLASSES {
        let p = presets::by_class(c);
        for (i, f) in meta::FIELDS.iter().enumerate() {
            let v = meta::value_at(&p, i).expect("getter exists");
            assert!(
                v.is_finite() && v >= f.min && v <= f.max,
                "{c:?}: {} = {v} outside [{}, {}] ({})",
                f.path,
                f.min,
                f.max,
                f.source
            );
        }
    }
}

#[test]
fn physical_fields_carry_units_and_suffixes() {
    // Every f64 physical field name ends in a unit suffix and carries a non-empty unit;
    // dimensionless ratios use unit "1" (plan §2b item 8; CLAUDE.md naming rule).
    for f in meta::FIELDS {
        assert!(!f.source.trim().is_empty(), "{}: source", f.path);
        let last = f.path.rsplit('.').next().unwrap_or("");
        let is_index = last.chars().all(|c| c.is_ascii_digit());
        if f.unit.is_empty() || f.unit == "1" || is_index {
            continue;
        }
        assert!(
            last.contains('_') || last == "turns" || last == "crc",
            "{}: physical field names carry a unit suffix",
            f.path
        );
    }
}

#[cfg(feature = "serde")]
mod serde_leaves {
    use super::*;

    fn walk(v: &serde_json::Value, prefix: &str, out: &mut Vec<(String, serde_json::Value)>) {
        let join = |seg: &str| {
            if prefix.is_empty() {
                seg.to_string()
            } else {
                format!("{prefix}.{seg}")
            }
        };
        match v {
            serde_json::Value::Object(m) => {
                for (k, x) in m {
                    walk(x, &join(k), out);
                }
            }
            serde_json::Value::Array(a) => {
                for (i, x) in a.iter().enumerate() {
                    walk(x, &join(&i.to_string()), out);
                }
            }
            leaf => out.push((prefix.to_string(), leaf.clone())),
        }
    }

    #[test]
    fn every_serde_leaf_has_meta() {
        for c in presets::CLASSES {
            let p = presets::by_class(c);
            let json = serde_json::to_value(p).expect("serialise");
            let mut got = Vec::new();
            walk(&json, "", &mut got);
            assert_eq!(got.len(), meta::N_FIELDS, "{c:?}: serde leaf count");
            for (path, v) in &got {
                let i = meta::index_of(path)
                    .unwrap_or_else(|| panic!("{c:?}: serde leaf {path} has no meta"));
                let g = meta::value_at(&p, i).expect("getter");
                match v {
                    serde_json::Value::Number(n) => {
                        let x = n.as_f64().expect("number");
                        assert_eq!(g.to_bits(), x.to_bits(), "{c:?}: {path}");
                    }
                    serde_json::Value::Bool(b) => {
                        assert_eq!(g, f64::from(u8::from(*b)), "{c:?}: {path}")
                    }
                    serde_json::Value::String(s) => assert!(
                        s.chars().all(|ch| ch.is_ascii_lowercase() || ch == '_'),
                        "{c:?}: {path} enum serialises snake_case ({s})"
                    ),
                    other => panic!("{c:?}: {path} unexpected leaf {other:?}"),
                }
                let pointer = format!("/{}", path.replace('.', "/"));
                assert!(
                    json.pointer(&pointer).is_some(),
                    "{c:?}: JSON pointer {pointer}"
                );
            }
        }
    }
}
