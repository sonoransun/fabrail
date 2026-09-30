//! Transitive dependency closure of the safety crates (FAB-EXO-116; EXOTIC §S3, plan §2).
//!
//! `cargo metadata --format-version 1 --offline --locked` on the host workspace; from
//! fabrail-safety and fabrail-exo-safety, follow the normal and build edges of the
//! resolve graph (dev edges excluded) and require that the closure never reaches
//! fabrail-core, fabrail-exo, fabrail-plant, fabrail-sim or fabrail-exo-sim; the only
//! fabrail crates allowed are fabrail-types and fabrail-params. This is the Rust twin of
//! scripts/check_nostd.sh tier 5. The manifest is frozen (no serde_json dev-dependency),
//! so a minimal JSON reader lives here.

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;
use std::process::Command;

/// The fabrail crates the safety closure may contain.
const ALLOWED: [&str; 2] = ["fabrail-types", "fabrail-params"];

/// Control, plant and simulation crates the safety closure must never contain.
const FORBIDDEN: [&str; 5] = [
    "fabrail-core",
    "fabrail-exo",
    "fabrail-plant",
    "fabrail-sim",
    "fabrail-exo-sim",
];

/// The safety crates whose closure is checked.
const ROOTS: [&str; 2] = ["fabrail-safety", "fabrail-exo-safety"];

// ---------------------------------------------------------------------------------------
// Minimal JSON reader (RFC 8259 subset sufficient for cargo metadata).
// ---------------------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq)]
enum Json {
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<Json>),
    Obj(Vec<(String, Json)>),
}

impl Json {
    fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Obj(kv) => kv.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }

    fn as_str(&self) -> Option<&str> {
        match self {
            Json::Str(s) => Some(s),
            _ => None,
        }
    }

    fn as_arr(&self) -> &[Json] {
        match self {
            Json::Arr(a) => a,
            _ => &[],
        }
    }
}

struct Parser<'a> {
    b: &'a [u8],
    i: usize,
}

impl Parser<'_> {
    fn parse(text: &str) -> Result<Json, String> {
        let mut p = Parser {
            b: text.as_bytes(),
            i: 0,
        };
        let v = p.value()?;
        p.ws();
        if p.i != p.b.len() {
            return Err(format!("trailing data at byte {}", p.i));
        }
        Ok(v)
    }

    fn ws(&mut self) {
        while self.i < self.b.len() && matches!(self.b[self.i], b' ' | b'\t' | b'\n' | b'\r') {
            self.i += 1;
        }
    }

    fn eat(&mut self, c: u8) -> Result<(), String> {
        self.ws();
        if self.b.get(self.i) == Some(&c) {
            self.i += 1;
            Ok(())
        } else {
            Err(format!("expected '{}' at byte {}", c as char, self.i))
        }
    }

    fn lit(&mut self, s: &str, v: Json) -> Result<Json, String> {
        if self.b[self.i..].starts_with(s.as_bytes()) {
            self.i += s.len();
            Ok(v)
        } else {
            Err(format!("bad literal at byte {}", self.i))
        }
    }

    fn value(&mut self) -> Result<Json, String> {
        self.ws();
        match self.b.get(self.i) {
            Some(b'{') => {
                self.i += 1;
                let mut kv = Vec::new();
                self.ws();
                if self.b.get(self.i) == Some(&b'}') {
                    self.i += 1;
                    return Ok(Json::Obj(kv));
                }
                loop {
                    self.ws();
                    let k = self.string()?;
                    self.eat(b':')?;
                    kv.push((k, self.value()?));
                    self.ws();
                    match self.b.get(self.i) {
                        Some(b',') => self.i += 1,
                        Some(b'}') => {
                            self.i += 1;
                            return Ok(Json::Obj(kv));
                        }
                        _ => return Err(format!("bad object at byte {}", self.i)),
                    }
                }
            }
            Some(b'[') => {
                self.i += 1;
                let mut a = Vec::new();
                self.ws();
                if self.b.get(self.i) == Some(&b']') {
                    self.i += 1;
                    return Ok(Json::Arr(a));
                }
                loop {
                    a.push(self.value()?);
                    self.ws();
                    match self.b.get(self.i) {
                        Some(b',') => self.i += 1,
                        Some(b']') => {
                            self.i += 1;
                            return Ok(Json::Arr(a));
                        }
                        _ => return Err(format!("bad array at byte {}", self.i)),
                    }
                }
            }
            Some(b'"') => Ok(Json::Str(self.string()?)),
            Some(b't') => self.lit("true", Json::Bool(true)),
            Some(b'f') => self.lit("false", Json::Bool(false)),
            Some(b'n') => self.lit("null", Json::Null),
            Some(_) => self.number(),
            None => Err("unexpected end".into()),
        }
    }

    fn string(&mut self) -> Result<String, String> {
        if self.b.get(self.i) != Some(&b'"') {
            return Err(format!("expected string at byte {}", self.i));
        }
        self.i += 1;
        let mut out: Vec<u8> = Vec::new();
        loop {
            let c = *self.b.get(self.i).ok_or("unterminated string")?;
            self.i += 1;
            match c {
                b'"' => break,
                b'\\' => {
                    let e = *self.b.get(self.i).ok_or("bad escape")?;
                    self.i += 1;
                    match e {
                        b'"' | b'\\' | b'/' => out.push(e),
                        b'b' => out.push(8),
                        b'f' => out.push(12),
                        b'n' => out.push(b'\n'),
                        b'r' => out.push(b'\r'),
                        b't' => out.push(b'\t'),
                        b'u' => {
                            let hex = std::str::from_utf8(&self.b[self.i..self.i + 4])
                                .map_err(|e| e.to_string())?;
                            let cp = u32::from_str_radix(hex, 16).map_err(|e| e.to_string())?;
                            self.i += 4;
                            let ch = char::from_u32(cp).unwrap_or('\u{FFFD}');
                            let mut buf = [0u8; 4];
                            out.extend_from_slice(ch.encode_utf8(&mut buf).as_bytes());
                        }
                        _ => return Err(format!("bad escape at byte {}", self.i)),
                    }
                }
                _ => out.push(c),
            }
        }
        String::from_utf8(out).map_err(|e| e.to_string())
    }

    fn number(&mut self) -> Result<Json, String> {
        let start = self.i;
        while self.i < self.b.len()
            && matches!(
                self.b[self.i],
                b'-' | b'+' | b'.' | b'e' | b'E' | b'0'..=b'9'
            )
        {
            self.i += 1;
        }
        let s = std::str::from_utf8(&self.b[start..self.i]).map_err(|e| e.to_string())?;
        s.parse::<f64>()
            .map(Json::Num)
            .map_err(|e| format!("bad number {s:?}: {e}"))
    }
}

// ---------------------------------------------------------------------------------------
// Resolve graph and closure.
// ---------------------------------------------------------------------------------------

/// Package id → (name, normal/build dependency ids).
type Graph = BTreeMap<String, (String, Vec<String>)>;

fn graph_from_metadata(meta: &Json) -> Graph {
    let names: BTreeMap<&str, &str> = meta
        .get("packages")
        .map(Json::as_arr)
        .unwrap_or(&[])
        .iter()
        .filter_map(|p| Some((p.get("id")?.as_str()?, p.get("name")?.as_str()?)))
        .collect();
    let mut g = Graph::new();
    let nodes = meta
        .get("resolve")
        .and_then(|r| r.get("nodes"))
        .map(Json::as_arr)
        .unwrap_or(&[]);
    for n in nodes {
        let Some(id) = n.get("id").and_then(Json::as_str) else {
            continue;
        };
        let deps = n
            .get("deps")
            .map(Json::as_arr)
            .unwrap_or(&[])
            .iter()
            .filter(|d| {
                // A null kind is a normal edge; "build" is a build edge; "dev" is excluded.
                d.get("dep_kinds")
                    .map(Json::as_arr)
                    .unwrap_or(&[])
                    .iter()
                    .any(|k| {
                        matches!(k.get("kind"), Some(Json::Null) | None)
                            || k.get("kind").and_then(Json::as_str) == Some("build")
                    })
            })
            .filter_map(|d| d.get("pkg")?.as_str().map(str::to_string))
            .collect();
        let name = names.get(id).copied().unwrap_or(id).to_string();
        g.insert(id.to_string(), (name, deps));
    }
    g
}

/// Names of every package reachable from the packages named `root` (root excluded).
fn closure(g: &Graph, root: &str) -> Option<BTreeSet<String>> {
    let starts: Vec<&String> = g
        .iter()
        .filter(|(_, (n, _))| n == root)
        .map(|(id, _)| id)
        .collect();
    if starts.is_empty() {
        return None;
    }
    let mut seen: BTreeSet<&str> = BTreeSet::new();
    let mut stack: Vec<&str> = starts.iter().map(|s| s.as_str()).collect();
    while let Some(cur) = stack.pop() {
        for d in g.get(cur).map(|(_, deps)| deps.as_slice()).unwrap_or(&[]) {
            if seen.insert(d.as_str()) {
                stack.push(d.as_str());
            }
        }
    }
    Some(
        seen.iter()
            .filter_map(|id| g.get(*id).map(|(n, _)| n.clone()))
            .collect(),
    )
}

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("workspace root")
}

fn cargo_metadata() -> Json {
    let cargo = std::env::var("CARGO").unwrap_or_else(|_| "cargo".into());
    let root = workspace_root();
    let out = Command::new(cargo)
        .current_dir(&root)
        .args(["metadata", "--format-version", "1", "--offline", "--locked"])
        .arg("--manifest-path")
        .arg(root.join("Cargo.toml"))
        .output()
        .expect("run cargo metadata");
    assert!(
        out.status.success(),
        "cargo metadata failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    Parser::parse(&String::from_utf8(out.stdout).expect("utf-8 metadata")).expect("parse metadata")
}

#[test]
fn closure_logic_on_a_synthetic_graph() {
    // a -> b (normal) -> c (build); a -> d (dev only); c -> e.
    let meta = Parser::parse(
        r#"{"packages":[{"id":"a","name":"fabrail-exo-safety"},{"id":"b","name":"fabrail-types"},
            {"id":"c","name":"cc"},{"id":"d","name":"fabrail-core"},{"id":"e","name":"fabrail-sim"}],
            "resolve":{"nodes":[
              {"id":"a","deps":[{"pkg":"b","dep_kinds":[{"kind":null}]},{"pkg":"d","dep_kinds":[{"kind":"dev"}]}]},
              {"id":"b","deps":[{"pkg":"c","dep_kinds":[{"kind":"build","target":null}]}]},
              {"id":"c","deps":[{"pkg":"e","dep_kinds":[{"kind":null}]}]},
              {"id":"d","deps":[]},{"id":"e","deps":[]}]}}"#,
    )
    .unwrap();
    let g = graph_from_metadata(&meta);
    let c = closure(&g, "fabrail-exo-safety").unwrap();
    assert!(
        c.contains("fabrail-types") && c.contains("cc"),
        "normal + build edges"
    );
    assert!(c.contains("fabrail-sim"), "transitive");
    assert!(!c.contains("fabrail-core"), "dev edges excluded");
    assert!(closure(&g, "absent").is_none());
    assert_eq!(
        Parser::parse(r#"["a\"\\é", -1.5e3, true, null, {}]"#).unwrap(),
        Json::Arr(vec![
            Json::Str("a\"\\é".into()),
            Json::Num(-1500.0),
            Json::Bool(true),
            Json::Null,
            Json::Obj(vec![]),
        ])
    );
}

// req: FAB-EXO-116
#[test]
fn closure_excludes_control_and_plant() {
    let g = graph_from_metadata(&cargo_metadata());
    // The workspace graph is real and contains the crates the rule talks about.
    for name in FORBIDDEN.iter().chain(ROOTS.iter()).chain(ALLOWED.iter()) {
        assert!(
            g.values().any(|(n, _)| n == name),
            "{name} missing from the resolve graph"
        );
    }
    for root in ROOTS {
        let c = closure(&g, root).unwrap_or_else(|| panic!("{root} not in the resolve graph"));
        let fab: BTreeSet<&str> = c
            .iter()
            .map(String::as_str)
            .filter(|n| n.starts_with("fabrail-"))
            .collect();
        for bad in FORBIDDEN {
            assert!(
                !c.contains(bad),
                "{root}: closure reaches {bad} (FAB-EXO-116; closure: {fab:?})"
            );
        }
        for n in &fab {
            assert!(
                ALLOWED.contains(n),
                "{root}: closure reaches {n}; allowed {ALLOWED:?}"
            );
        }
        // Not vacuous: both allowed crates are reached, and dev-only crates are not.
        for n in ALLOWED {
            assert!(c.contains(n), "{root}: closure lacks {n}");
        }
        assert!(
            !c.contains("proptest"),
            "{root}: dev-dependency leaked into the closure"
        );
    }
}
