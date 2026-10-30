"""What the alignment publishes is what the custom view plots (custom_view/__init__.py).

The view draws one card per declared phase, as PHASE_VIEWS says, from the
rows and the payloads below. These checks keep the two in step: a payload the
view would not plot, a phase it would not know how to draw, or rows that are
not where the robot was, fail here.
"""

import math
import re
from pathlib import Path

import numpy as np
import pytest

from app_logic.fiber_alignment import algorithm, alignment, frames

from .sim import START_POSE, Scenario, simulate

VIEW_JS = Path(__file__).resolve().parents[2] / "custom_view" / "view.js"
AXES = {"x", "y", "z", "rx", "ry", "rz"}
KINDS = {"cart", "ang", None}
#: What a phase view may declare (algorithm.PHASE_VIEWS).
VIEW_KEYS = {"type", "axes", "slots", "search_axes", "until_end"}


#: Cold starts: one found by the tilt spiral, one by the lateral spiral after
#: it; and a full search (acquire_full_search) with light at the start.
SCENARIOS = {
    "angle": Scenario(offset=(40, 30, 1.2, -0.4), seed=11),
    "lateral": Scenario(offset=(600, 0, 0.1, 0.0), seed=12, params={"acquire_radius_um": 800.0}),
    "full": Scenario(offset=(50, 0, 0.45, 0.0), seed=13, params={"acquire_full_search": True}),
}


@pytest.fixture(scope="module", params=sorted(SCENARIOS))
def run(request):
    run = simulate(SCENARIOS[request.param])
    assert run.error is None, run.error
    return run


def is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def test_every_phase_declares_a_view_the_page_draws():
    source = VIEW_JS.read_text(encoding="utf-8")
    renderers = set(re.findall(r"^\s*(\w+): render\w+,\s*$", source, re.M))
    assert set(algorithm.PHASE_VIEWS) == set(algorithm.PHASES)
    for name, view in algorithm.PHASE_VIEWS.items():
        assert view["type"] in renderers, f"{name}: no renderer for {view['type']!r} in view.js"
        # Every option a view declares is one the page reads.
        assert set(view) <= VIEW_KEYS, f"{name}: unknown view options {set(view) - VIEW_KEYS}"
        assert all(f"view.{key}" in source for key in view), f"{name}: view.js ignores one of {sorted(view)}"
        assert set(view.get("axes", [])) <= AXES and set(view.get("slots", [])) <= AXES
        for axes in view.get("search_axes", {}).values():
            assert len(axes) == 2 and set(axes) <= AXES
    # The Acquire map goes on through the later phases.
    assert algorithm.PHASE_VIEWS["Acquire"].get("until_end") is True


def test_align_start_announces_the_phases_and_the_row_format(run):
    start = run.context.events("align_start")[-1]
    assert start["phases"] == list(algorithm.PHASES)
    assert start["phase_views"] == algorithm.PHASE_VIEWS
    assert start["sample_format"] == alignment.SAMPLE_FORMAT == 2
    assert start["source_power_w"] == pytest.approx(1e-3)


def test_rows_have_the_documented_shape(run):
    for row in run.rows:
        assert len(row) == 11
        assert all(is_num(v) for v in row[:7])
        assert row[7] in KINDS
        assert isinstance(row[8], int) and -1 <= row[8] < len(algorithm.PHASES)
        assert is_num(row[9]) and isinstance(row[10], bool)


def test_payloads_go_to_phases_whose_card_plots_them(run):
    names = algorithm.PHASES
    for p in run.context.published("align_profile"):
        view = algorithm.PHASE_VIEWS[names[p["phase"]]]
        assert view["type"] == "profiles" and p["axis"] in view["slots"]
        assert p["unit"] == ("°" if p["axis"].startswith("r") else "µm")
        lo, hi = p["span"]
        assert lo < hi
        assert p["passes"] and all(q["dir"] in (1, -1) for q in p["passes"])
        assert all(len(pt) == 2 and is_num(pt[0]) for q in p["passes"] for pt in q["points"])
        if p["fit"] is not None:
            assert p["fit"]["model"] in ("gauss", "parabola")
            assert is_num(p["fit"]["center"])
    for p in run.context.published("align_peak"):
        assert algorithm.PHASE_VIEWS[names[p["phase"]]]["type"] == "peak"
        assert len(p["center"]) == 2 and is_num(p["step_um"]) and is_num(p["radius_um"])
        assert all(len(pt) == 3 for pt in p["points"])
    for p in run.context.published("align_acquire"):
        view = algorithm.PHASE_VIEWS[names[p["phase"]]]
        assert view["type"] == "map"
        assert {"noise_w", "detect_w", "found", "warm", "full", "search"} <= set(p)
        assert isinstance(p["full"], bool)
        if p["search"] is not None:
            assert p["search"] in algorithm.SEARCH_AXES
            assert {"radius", "pitch", "unit"} <= set(p)
            assert p["unit"] == ("°" if p["search"] == "angle" else "µm")


def test_still_readings_and_payloads_survive_into_the_summary(run):
    summary = run.summary
    assert sum(1 for r in summary["points"] if r[10]) == sum(1 for r in run.rows if r[10])
    for name in ("acquire", "profile", "peak"):
        assert len(summary["published"][name]) == len(run.context.published(f"align_{name}"))
    assert summary["phase_views"] == algorithm.PHASE_VIEWS


def spirals(run):
    """For each spiral the run made: its axes' indices in a row, its reading spacing, and its readings."""
    acquire = run.context.published("align_acquire")
    rows = [r for r in run.rows if r[8] == algorithm.PHASES.index("Acquire") and not r[10]]
    for search in dict.fromkeys(a["search"] for a in acquire if a["search"]):
        payloads = [a for a in acquire if a["search"] == search]
        t0 = payloads[0]["t"]
        t1 = min([a["t"] for a in acquire if a["t"] > t0 and a["search"] != search], default=math.inf)
        axes = [algorithm.LOCAL_INDEX[a] for a in algorithm.SEARCH_AXES[search]]
        spacing = payloads[0]["pitch"] / algorithm.SPIRAL_READINGS_PER_PITCH
        readings = np.array([[r[axes[0]], r[axes[1]]] for r in rows if t0 <= r[9] < t1])
        yield search, axes, spacing, readings


def test_plotted_spiral_readings_lie_on_the_commanded_path(run):
    # Rule: what the view plots is where the robot was. Spiral readings are
    # interpolated between poses: they must sit on the spiral the robot ran,
    # in the axes the card plots it over (tilts for the tilt spiral).
    inverse = np.linalg.inv(frames.pose_matrix(START_POSE))
    local = []
    for m in run.robot.moves:
        position, angles = frames.extract_pose_extrinsic(inverse @ frames.pose_matrix(m))
        local.append([*(position * 1000.0), *angles])
    local = np.array(local)
    checked = 0
    for search, axes, spacing, readings in spirals(run):
        if len(readings) < 20:
            continue
        path = local[:, axes]
        a, b = path[:-1], path[1:]
        ab = b - a
        distances = []
        for chunk in np.array_split(readings, max(1, len(readings) // 200)):
            ap = chunk[:, None, :] - a[None, :, :]
            t = np.clip(np.einsum("nmk,mk->nm", ap, ab) / np.maximum(np.einsum("mk,mk->m", ab, ab), 1e-12), 0, 1)
            distances.extend(np.linalg.norm(ap - t[..., None] * ab[None, :, :], axis=2).min(axis=1))
        # A reading's pose lies on the chord between the poses read around
        # it, a reading apart: across a corner of the spiral's polyline, the
        # chord cuts it by up to spacing² / (2 radius).
        radius = np.hypot(readings[:, 0], readings[:, 1])
        allowed = 0.05 * spacing + spacing ** 2 / (2 * np.maximum(radius, spacing))
        excess = np.array(distances) - allowed
        worst = int(np.argmax(excess))
        assert excess[worst] <= 0, (f"a {search} spiral reading {radius[worst]:.3g} from the start is "
                                    f"{distances[worst]:.3g} off the commanded path (allowed {allowed[worst]:.3g})")
        checked += 1
    assert checked, "no spiral to check"


def test_the_result_matches_the_verdict_event(run):
    end = run.context.events("align_end")[-1]
    assert end["power_w"] == run.summary["power_w"] == run.result.power_w
    assert end["passed"] == run.summary["passed"] == run.result.passed
    assert math.isfinite(run.summary["best_still"][6])
