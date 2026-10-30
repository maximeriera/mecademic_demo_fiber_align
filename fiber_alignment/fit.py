"""Curve fits for the alignment algorithm (numpy only).

The power two fibers couple falls off as a Gaussian with their lateral offset
and their tilt, so its logarithm is a parabola near the peak: the fits here
work on ln P, weighted by the power so that the readings near the noise floor
count little.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

#: MAD × this estimates the standard deviation of Gaussian noise.
MAD_TO_SIGMA = 1.4826
#: Readings under this fraction of a profile's peak (above its baseline) are
#: left out of a Gaussian fit: there, noise dominates the logarithm.
GAUSS_FLOOR = 0.05
#: Percentile of a profile's readings taken as its baseline.
BASELINE_PERCENTILE = 10


@dataclass(frozen=True)
class Gauss:
    """``baseline + peak * exp(-2 (x - center)² / width²)``: ``width`` is the 1/e² radius."""

    center: float
    width: float
    peak: float
    baseline: float

    @property
    def top(self) -> float:
        """The fitted power at the centre."""
        return self.peak + self.baseline


@dataclass(frozen=True)
class Parabola:
    """``ln P = ln(peak) + curvature * (x - vertex)²``; ``curvature < 0`` for a maximum."""

    vertex: float
    curvature: float
    peak: float

    @property
    def width(self) -> float:
        """Where the fitted power falls to 1/e of the peak, from the vertex (inf if flat)."""
        return 1.0 / math.sqrt(-self.curvature) if self.curvature < 0 else math.inf


def noise_stats(values: Sequence[float]) -> Tuple[float, float]:
    """Median and standard deviation (from the MAD, robust to outliers) of the finite ``values``."""
    v = np.asarray([x for x in values if math.isfinite(x)], dtype=float)
    if not len(v):
        return math.nan, math.nan
    median = float(np.median(v))
    return median, float(MAD_TO_SIGMA * np.median(np.abs(v - median)))


def gauss1d(coords: Sequence[float], powers: Sequence[float], baseline: Optional[float] = None) -> Optional[Gauss]:
    """Fit a Gaussian to a 1-D profile, or None if it shows no peak.

    The baseline is ``baseline``, else the low percentile of the readings. The
    fit is a weighted least-squares parabola through ln(P - baseline) over the
    readings above GAUSS_FLOOR of the peak (Caruana's method, weights P²).
    """
    x = np.asarray(coords, dtype=float)
    p = np.asarray(powers, dtype=float)
    ok = np.isfinite(x) & np.isfinite(p)
    x, p = x[ok], p[ok]
    if len(x) < 4:
        return None
    b = float(np.percentile(p, BASELINE_PERCENTILE)) if baseline is None else float(baseline)
    y = p - b
    top = float(y.max())
    if not top > 0:
        return None
    keep = y > GAUSS_FLOOR * top
    x, y = x[keep], y[keep]
    if len(x) < 4 or np.ptp(x) == 0:
        return None
    # Centred for a well-conditioned fit.
    mid = float(x.mean())
    u = x - mid
    a = np.column_stack([np.ones_like(u), u, u * u])
    (a0, a1, a2), *_ = np.linalg.lstsq(a * y[:, None], np.log(y) * y, rcond=None)
    if not a2 < 0:
        return None
    center = mid - a1 / (2 * a2)
    return Gauss(center=float(center), width=float(math.sqrt(-2.0 / a2)),
                 peak=float(math.exp(a0 - a1 * a1 / (4 * a2))), baseline=b)


def parabola1d(coords: Sequence[float], powers: Sequence[float]) -> Optional[Parabola]:
    """Fit a parabola to ln P against ``coords``, or None with fewer than 3 positive readings."""
    x = np.asarray(coords, dtype=float)
    p = np.asarray(powers, dtype=float)
    ok = np.isfinite(x) & np.isfinite(p) & (p > 0)
    x, p = x[ok], p[ok]
    if len(x) < 3 or np.ptp(x) == 0:
        return None
    mid = float(x.mean())
    u = x - mid
    a = np.column_stack([np.ones_like(u), u, u * u])
    (a0, a1, a2), *_ = np.linalg.lstsq(a, np.log(p), rcond=None)
    if a2 == 0:
        return Parabola(vertex=math.nan, curvature=0.0, peak=math.nan)
    vertex = -a1 / (2 * a2)
    return Parabola(vertex=float(mid + vertex), curvature=float(a2), peak=float(math.exp(a0 - a1 * a1 / (4 * a2))))


def plane_slope(points: Sequence[Tuple[float, float, float]]
                ) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """The slope (d ln P/du, d ln P/dv) of a plane fitted to ln P over ``(u, v, power)`` points, and its
    standard errors (from the readings' scatter about the plane).

    None with fewer than 4 positive readings, or with readings all on a line.
    """
    pts = np.asarray([pt for pt in points if all(math.isfinite(c) for c in pt) and pt[2] > 0], dtype=float)
    if len(pts) < 4:
        return None
    a = np.column_stack([np.ones(len(pts)), pts[:, 0], pts[:, 1]])
    if np.linalg.matrix_rank(a) < 3:
        return None
    y = np.log(pts[:, 2])
    coef, *_ = np.linalg.lstsq(a, y, rcond=None)
    residuals = y - a @ coef
    covariance = np.linalg.inv(a.T @ a) * float(residuals @ residuals) / (len(pts) - 3)
    errors = np.sqrt(np.diag(covariance))
    return (float(coef[1]), float(coef[2])), (float(errors[1]), float(errors[2]))


def quad2d_step(points: Sequence[Tuple[float, float, float]], radius: float) -> Tuple[Tuple[float, float], bool]:
    """The step from the origin towards the maximum of a quadratic fitted to ln P.

    ``points`` are ``(u, v, power)`` around the origin (at least 6 positive
    readings, e.g. a 3×3 or 5×5 pattern). When the fit has a maximum, the
    step goes there (Newton), clipped to ``radius``. Otherwise it goes
    ``radius`` up the fitted slope. Returns ``((du, dv), newton)``.
    """
    pts = np.asarray([pt for pt in points if all(math.isfinite(c) for c in pt) and pt[2] > 0], dtype=float)
    if len(pts) < 6:
        return (0.0, 0.0), False
    u, v, p = pts[:, 0], pts[:, 1], pts[:, 2]
    w = p / p.max()
    a = np.column_stack([np.ones_like(u), u, v, u * u, u * v, v * v])
    (_, gu, gv, huu, huv, hvv), *_ = np.linalg.lstsq(a * w[:, None], np.log(p) * w, rcond=None)
    gradient = np.array([gu, gv])
    hessian = np.array([[2 * huu, huv], [huv, 2 * hvv]])
    newton = bool(np.all(np.linalg.eigvalsh(hessian) < 0))
    if newton:
        step = -np.linalg.solve(hessian, gradient)
    else:
        norm = float(np.linalg.norm(gradient))
        step = gradient / norm * radius if norm > 0 else np.zeros(2)
    length = float(np.linalg.norm(step))
    if length > radius:
        step *= radius / length
    return (float(step[0]), float(step[1])), newton
