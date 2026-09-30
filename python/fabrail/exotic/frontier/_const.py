"""Physical constants shared by the 🔭 frontier models (FRONTIER §F1; PHYSICS §1).

The values match ``fabrail_params::derive`` where the Rust core defines the same
constant, so a frontier number and a core number never differ by a constant choice.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

from typing import overload

import numpy as np

__all__ = [
    "FLOAT_PAYLOAD_KG_M2",
    "G0_MPS2",
    "G_MARS_MPS2",
    "G_MOON_MPS2",
    "G_NEWTON_M3_PER_KG_S2",
    "MU0_H_PER_M",
    "SIGMA_SB_W_PER_M2_K4",
    "magnetic_pressure_pa",
]

#: Vacuum permeability, H/m (CODATA 2018; equals fabrail_params::derive::MU0_H_PER_M;
#: PHYSICS §2).
MU0_H_PER_M = 1.256_637_062_12e-6

#: Standard gravity, m/s² (CGPM 1901; equals fabrail_params::derive::G0_MPS2; PHYSICS §3).
G0_MPS2 = 9.806_65

#: Lunar surface gravity, m/s² (NASA lunar fact sheet; frontier research §3g;
#: FRONTIER §F7).
G_MOON_MPS2 = 1.62

#: Martian surface gravity, m/s² (NASA Mars fact sheet; frontier research §3g; FRONTIER §F7).
G_MARS_MPS2 = 3.71

#: Newtonian constant of gravitation, m³/(kg·s²) (CODATA 2018; FRONTIER §F4).
G_NEWTON_M3_PER_KG_S2 = 6.674_30e-11

#: Stefan–Boltzmann constant, W/(m²·K⁴) (CODATA 2018, exact; FRONTIER §F7).
SIGMA_SB_W_PER_M2_K4 = 5.670_374_419e-8

#: NASA FLOAT claimed payload, kg/m² (NASA/JPL FLOAT, NIAC; frontier research §3g;
#: exotic critique XCRIT §6: cm-scale prototypes; FRONTIER §F9).
FLOAT_PAYLOAD_KG_M2 = 33.0


@overload
def magnetic_pressure_pa(b_t: float) -> float: ...
@overload
def magnetic_pressure_pa(b_t: np.ndarray) -> np.ndarray: ...
def magnetic_pressure_pa(b_t: float | np.ndarray) -> float | np.ndarray:
    """Magnetic pressure B²/(2μ0), Pa (EXOTIC §X2; PHYSICS §2). Works on numpy arrays too.

    1 T gives 397.9 kPa, the "0.40 MPa at 1 T" of frontier research §3d.
    """
    return b_t * b_t / (2.0 * MU0_H_PER_M)
