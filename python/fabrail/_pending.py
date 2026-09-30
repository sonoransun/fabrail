"""Marker for Wave-0 stubs whose bodies land in a later wave.

Every stub function in this package keeps its frozen signature and raises
:class:`NotYetImplementedError` naming the wave and work unit that fills it in.
The CLI turns that exception into a one-line "not yet implemented (Wave N, unit X)"
message and exit status 2. This module disappears once no stub remains.
"""

from __future__ import annotations

from typing import NoReturn

__all__ = ["NotYetImplementedError", "pending"]


class NotYetImplementedError(NotImplementedError):
    """A frozen Wave-0 signature whose body is owned by a later work unit."""

    def __init__(self, what: str, wave: str, unit: str) -> None:
        self.what = what
        self.wave = wave
        self.unit = unit
        super().__init__(f"{what}: not yet implemented ({wave}, unit {unit})")


def pending(what: str, wave: str, unit: str) -> NoReturn:
    """Raise :class:`NotYetImplementedError` for ``what`` (e.g. ``"ground.synthesize"``)."""
    raise NotYetImplementedError(what, wave, unit)
