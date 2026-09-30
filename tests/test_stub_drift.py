"""Contract: ``python/fabrail/_native.pyi`` declares exactly the compiled surface.

FAB-SW-006 (drift guard for the ``_native.pyi`` stub). The stub is parsed with ``ast``
and compared with ``dir(fabrail._native)`` in both directions:

- module level: the same public names (``__version__`` included), with the same kind
  (constant / function / class) and, for ``int``/``str``/``float``/``bool`` constants,
  the annotated type;
- classes: the same public members (methods, static methods, properties, class
  attributes such as ``RailClass.DRAWER``) with the same kind;
- functions, methods and ``__init__``: the same parameter names, in order, and the same
  set of parameters that have defaults (default values are not compared: PyO3 text
  signatures render them as ``...``).

The native half is skipped when ``fabrail._native`` is not built
(``maturin develop --release``); the stub's own well-formedness is always checked.
"""

from __future__ import annotations

import ast
import inspect
from pathlib import Path
from types import ModuleType

import pytest

PYI = Path(__file__).resolve().parents[1] / "python" / "fabrail" / "_native.pyi"

#: Members every Python class inherits; never part of the declared surface.
_OBJECT_MEMBERS = frozenset(dir(object))

#: Annotations checked against the runtime type of module constants.
_SCALARS = {"int": int, "str": str, "float": float, "bool": bool}


def _public(name: str) -> bool:
    return not name.startswith("_") or name == "__version__"


def _decorators(fn: ast.FunctionDef) -> set[str]:
    return {d.id for d in fn.decorator_list if isinstance(d, ast.Name)}


def _params(fn: ast.FunctionDef, *, drop_first: bool) -> list[tuple[str, bool]]:
    """(name, has_default) for every parameter of a stub function."""
    a = fn.args
    positional = [*a.posonlyargs, *a.args]
    first_default = len(positional) - len(a.defaults)
    out = [(arg.arg, i >= first_default) for i, arg in enumerate(positional)]
    out += [(arg.arg, d is not None) for arg, d in zip(a.kwonlyargs, a.kw_defaults)]
    if a.vararg is not None or a.kwarg is not None:
        raise AssertionError(f"{fn.name}: *args/**kwargs are not part of the native surface")
    return out[1:] if drop_first else out


def _native_params(obj: object, *, drop_first: bool) -> list[tuple[str, bool]] | None:
    """(name, has_default) from the PyO3 text signature, or None when unavailable."""
    try:
        sig = inspect.signature(obj)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    ps = list(sig.parameters.values())
    if drop_first and ps and ps[0].name in {"self", "$self", "cls", "$cls"}:
        ps = ps[1:]
    return [(p.name, p.default is not inspect.Parameter.empty) for p in ps]


class _Stub:
    """The parsed stub: module constants, functions and classes."""

    def __init__(self, source: str) -> None:
        tree = ast.parse(source)
        self.consts: dict[str, str] = {}
        self.funcs: dict[str, ast.FunctionDef] = {}
        self.classes: dict[str, ast.ClassDef] = {}
        seen: set[str] = set()
        for node in tree.body:
            if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
                name, kind = node.target.id, "const"
                self.consts[name] = ast.unparse(node.annotation)
            elif isinstance(node, ast.FunctionDef):
                name, kind = node.name, "func"
                self.funcs[name] = node
            elif isinstance(node, ast.ClassDef):
                name, kind = node.name, "class"
                self.classes[name] = node
            else:
                continue
            assert name not in seen, f"_native.pyi declares {name!r} twice ({kind})"
            seen.add(name)
        self.names = {n for n in seen if _public(n)}

    @staticmethod
    def members(cls: ast.ClassDef) -> dict[str, tuple[str, ast.AST]]:
        """Public members of a stub class: name -> (kind, node)."""
        out: dict[str, tuple[str, ast.AST]] = {}
        for node in cls.body:
            if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
                name, kind = node.target.id, "attr"
            elif isinstance(node, ast.FunctionDef):
                decos = _decorators(node)
                kind = "property" if "property" in decos else "static" if "staticmethod" in decos else "method"
                name = node.name
            else:
                continue
            if _public(name):
                assert name not in out, f"{cls.name}.{name} declared twice"
                out[name] = (kind, node)
        return out


@pytest.fixture(scope="module")
def stub() -> _Stub:
    return _Stub(PYI.read_text(encoding="utf-8"))


def test_stub_is_well_formed(stub: _Stub) -> None:
    """The stub parses, declares no name twice and covers the core design §B.7 surface."""
    required = {
        "__version__",
        "BUILD_PROFILE",
        "BASE_TICK_HZ",
        "MAX_MAGNETS",
        "GOLDEN_SELFTEST_HASH",
        "SCENARIO_SCHEMA",
        "RailClass",
        "Params",
        "Scenario",
        "RunData",
        "DetectorReplay",
        "run",
        "run_batch",
        "stability_report",
        "stray_field",
        "selftest_hash",
        "open_loop_pole",
        "zero_power_mass",
        "zero_power_gap",
        "bias_current",
        "stop_ratio",
        "zp_gains",
        "noise_force_gain",
        "v_low_energy",
        "momentum_speed_cap",
        "stop_distance",
        "clamp_force",
        "series_stiffness",
        "iso15066_speed",
        "passivity_k_max",
        "impact_speed",
        "thermal_gap_shift",
        "manual_force",
        "piston_force",
        "den_hartog",
        "warburton",
        "transmissibility",
        "t_min_isolation",
        "pd_wu_zhao",
        "sp_time",
        "exo_version",
        "exo_scene_run",
    }
    missing = required - stub.names
    assert not missing, f"core design §B.7 names missing from _native.pyi: {sorted(missing)}"
    params = _Stub.members(stub.classes["Params"])
    for m in ("preset", "from_json", "describe", "to_json", "get", "set", "with_overrides", "validate", "derived", "rail_class"):
        assert m in params, f"Params.{m} missing from _native.pyi"
    exo = sorted(n for n in stub.funcs if n.startswith("exo"))
    assert exo and all(n.startswith(("exo_", "exo_scene_")) for n in exo), "exotic names are flat exo_*"


@pytest.mark.req("FAB-SW-006")
def test_stub_matches_native(stub: _Stub, native: ModuleType) -> None:
    """Every public name, class member and parameter list of the stub equals the build."""
    errors: list[str] = []

    native_names = {n for n in dir(native) if _public(n)}
    for n in sorted(stub.names - native_names):
        errors.append(f"{n}: declared in _native.pyi but missing from the build")
    for n in sorted(native_names - stub.names):
        errors.append(f"{n}: exported by the build but missing from _native.pyi")

    for name, ann in stub.consts.items():
        if name not in native_names:
            continue
        value = getattr(native, name)
        if callable(value) or isinstance(value, type):
            errors.append(f"{name}: declared as a constant but is {type(value).__name__}")
        elif ann in _SCALARS and not isinstance(value, _SCALARS[ann]):
            errors.append(f"{name}: annotated {ann} but is {type(value).__name__}")

    for name, fn in stub.funcs.items():
        if name not in native_names:
            continue
        obj = getattr(native, name)
        if isinstance(obj, type) or not callable(obj):
            errors.append(f"{name}: declared as a function but is {type(obj).__name__}")
            continue
        got = _native_params(obj, drop_first=False)
        want = _params(fn, drop_first=False)
        if got is not None and got != want:
            errors.append(f"{name}: parameters {got} (build) != {want} (stub)")

    for cname, cnode in stub.classes.items():
        if cname not in native_names:
            continue
        cls = getattr(native, cname)
        if not isinstance(cls, type):
            errors.append(f"{cname}: declared as a class but is {type(cls).__name__}")
            continue
        declared = _Stub.members(cnode)
        built = {n for n in dir(cls) if _public(n) and n not in _OBJECT_MEMBERS}
        for n in sorted(set(declared) - built):
            errors.append(f"{cname}.{n}: declared in _native.pyi but missing from the build")
        for n in sorted(built - set(declared)):
            errors.append(f"{cname}.{n}: exported by the build but missing from _native.pyi")
        for n, (kind, node) in declared.items():
            if n not in built:
                continue
            raw = inspect.getattr_static(cls, n)
            if kind == "property":
                if not inspect.isdatadescriptor(raw):
                    errors.append(f"{cname}.{n}: declared as a property but is {type(raw).__name__}")
            elif kind == "attr":
                if callable(raw) and not isinstance(raw, cls):
                    errors.append(f"{cname}.{n}: declared as a class attribute but is callable")
            else:
                if not callable(getattr(cls, n)):
                    errors.append(f"{cname}.{n}: declared as a method but is not callable")
                    continue
                if kind == "static" and not isinstance(raw, staticmethod):
                    errors.append(f"{cname}.{n}: declared @staticmethod but is {type(raw).__name__}")
                if kind == "method" and isinstance(raw, staticmethod):
                    errors.append(f"{cname}.{n}: is a staticmethod in the build; mark it @staticmethod")
                assert isinstance(node, ast.FunctionDef)
                got = _native_params(getattr(cls, n), drop_first=kind == "method")
                want = _params(node, drop_first=kind == "method")
                if got is not None and got != want:
                    errors.append(f"{cname}.{n}: parameters {got} (build) != {want} (stub)")
        init = next(
            (node for node in cnode.body if isinstance(node, ast.FunctionDef) and node.name == "__init__"),
            None,
        )
        if init is not None:
            got = _native_params(cls, drop_first=False)
            want = _params(init, drop_first=True)
            if got is not None and got != want:
                errors.append(f"{cname}.__init__: parameters {got} (build) != {want} (stub)")

    assert not errors, "_native.pyi drifted from the build:\n  " + "\n  ".join(errors)
