import math
import random

import numpy as np
import pytest

from app_logic.fiber_alignment import fit


def gaussian(x, center, width, peak, baseline=0.0):
    return baseline + peak * math.exp(-2 * (x - center) ** 2 / width ** 2)


def test_gauss1d_recovers_a_noisy_profile():
    rng = random.Random(1)
    xs = np.linspace(-25, 25, 81)
    ps = [gaussian(x, 3.2, 7.6, 0.9e-3, 2e-9) * (1 + rng.gauss(0, 1e-3)) + rng.gauss(0, 0.5e-9) for x in xs]
    g = fit.gauss1d(xs, ps)
    assert g is not None
    assert g.center == pytest.approx(3.2, abs=0.05)
    assert g.width == pytest.approx(7.6, rel=0.02)
    assert g.top == pytest.approx(0.9e-3, rel=0.01)


def test_gauss1d_sees_no_peak_in_noise():
    rng = random.Random(2)
    xs = np.linspace(-25, 25, 81)
    ps = [rng.gauss(0, 0.5e-9) for _ in xs]
    g = fit.gauss1d(xs, ps)
    # Noise either gives no fit or a meaningless one; never a confident narrow peak above the noise.
    assert g is None or g.peak < 5e-9


def test_gauss1d_ignores_invalid_readings():
    xs = list(np.linspace(-20, 20, 41))
    ps = [gaussian(x, -1.0, 6.0, 1e-4) for x in xs]
    ps[5], ps[30] = float("nan"), float("inf")
    g = fit.gauss1d(xs, ps)
    assert g.center == pytest.approx(-1.0, abs=1e-6)
    assert g.width == pytest.approx(6.0, rel=1e-6)


def test_parabola1d_finds_the_vertex():
    xs = [-1.5, -0.75, 0.0, 0.75, 1.5]
    ps = [0.8e-3 * math.exp(-((x - 0.4) / 5.5) ** 2) for x in xs]
    p = fit.parabola1d(xs, ps)
    assert p.vertex == pytest.approx(0.4, abs=1e-9)
    assert p.width == pytest.approx(5.5, rel=1e-9)
    assert p.peak == pytest.approx(0.8e-3, rel=1e-9)


def test_quad2d_step_lands_on_a_gaussian_peak():
    r = 3.8
    points = [(i * r, j * r, gaussian(math.hypot(i * r - 1.1, j * r + 0.7), 0, 7.6, 1e-3))
              for i in (-1, 0, 1) for j in (-1, 0, 1)]
    (du, dv), newton = fit.quad2d_step(points, radius=r)
    assert newton
    assert (du, dv) == pytest.approx((1.1, -0.7), abs=1e-6)


def test_quad2d_step_is_clipped_and_climbs_without_a_maximum():
    r = 2.0
    # A plane in log power (no maximum): the step goes r up the slope.
    points = [(i * r, j * r, math.exp(0.3 * i * r - 0.4 * j * r)) for i in (-1, 0, 1) for j in (-1, 0, 1)]
    (du, dv), newton = fit.quad2d_step(points, radius=r)
    assert not newton
    assert math.hypot(du, dv) == pytest.approx(r)
    assert du / dv == pytest.approx(0.3 / -0.4)


def test_noise_stats_resist_outliers():
    values = [1e-9, 1.1e-9, 0.9e-9, 1.05e-9, 0.95e-9, 5e-6, float("nan")]
    median, sigma = fit.noise_stats(values)
    assert median == pytest.approx(1.0e-9, rel=0.06)
    assert sigma < 2e-10
