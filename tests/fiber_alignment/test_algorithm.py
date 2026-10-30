"""The staged alignment (algorithm.py) on the simulated cell.

Mostly on optics like the cell's (collimated ends: broad laterally, narrow in
tilt), from its run of 2026-10-27 to cold starts; and on bare single-mode
fibers, to keep the algorithm free of any one scale.
"""

import math

import pytest

from app_logic.fiber_alignment import algorithm
from app_logic.fiber_alignment.alignment import AlignmentError, AlignmentNoSignal

from .sim import BEAM_WIDTH_UM, Scenario, simulate

#: Starts: (x, z) µm and (rx, rz) ° from the optimum, and how the cell behaves.
CASES = {
    # The cell's run of 2026-10-27 (~80 µW at the start), which failed in the Angle phase.
    "cell-run": Scenario(offset=(50, 0, 0.45, 0.0)),
    "few-uW": Scenario(offset=(90, -40, 0.5, -0.35), seed=2),
    # No light at the start, the tilt off: the tilt spiral finds it.
    "angle-off": Scenario(offset=(40, 30, 1.2, -0.4), seed=3),
    # No light at the start, the tilt right but 600 µm off: the lateral spiral finds it.
    "lateral-off": Scenario(offset=(600, 0, 0.1, 0.0), seed=4, params={"acquire_radius_um": 800.0}),
    "slow-io": Scenario(offset=(80, 40, 0.3, 0.2), latency_s=0.004, pose_query_s=0.003, seed=7),
    # Bare single-mode fibers: the same algorithm, other widths.
    "smf-near": Scenario(optics="smf", offset=(120, -80, 0.8, -0.5)),
    "smf-far": Scenario(optics="smf", offset=(300, 250, 1.5, 1.0), seed=3),
    "smf-tcp-behind-tip": Scenario(optics="smf", offset=(80, 60, 1.0, -1.0), pivot_um=(0, 500, 0), seed=5),
}


@pytest.fixture(scope="module", params=sorted(CASES))
def run(request):
    return simulate(CASES[request.param])


def test_it_reaches_the_peak(run):
    assert run.error is None, run.error
    assert run.result.passed
    assert run.result.power_w >= 0.97 * run.optics.best_power_w()
    assert run.result.duration_s < 120


def test_it_never_moves_along_the_fiber(run):
    # Rows are the TCP's offsets in the start's tool frame: y is the fiber axis.
    assert max(abs(row[1]) for row in run.rows) < 1e-3


def test_it_ends_on_the_best_reading_at_rest(run):
    best, final = run.summary["best_still"], run.summary["final_local"]
    assert abs(best[0] - final[0]) <= 1.0 and abs(best[2] - final[2]) <= 1.0


def test_it_runs_every_phase_in_order(run):
    names = [p["name"] for p in run.summary["phases"] if "t0" in p]
    assert names == list(algorithm.PHASES)
    assert all(p["status"] == "done" for p in run.summary["phases"])


def searches(run):
    return [a["search"] for a in run.context.published("align_acquire")]


def test_light_at_the_start_skips_the_searches():
    for name in ("cell-run", "few-uW"):
        run = simulate(CASES[name])
        assert run.rows[0][6] >= 1e-6, f"{name}: no light at the start"
        assert searches(run) == [None]
        readings = [r for r in run.rows if r[8] == algorithm.PHASES.index("Acquire")]
        assert len(readings) == algorithm.NOISE_READS


def test_no_light_tilts_first_then_moves():
    run = simulate(CASES["angle-off"])
    assert run.rows[0][6] < 1e-6
    assert searches(run) == ["angle", "angle"]
    run = simulate(CASES["lateral-off"])
    assert searches(run) == ["angle", "lateral", "lateral"]


FULL = {"acquire_full_search": True}


def spiral_reach(run):
    """Each spiral's search: (how far its readings went from the start, its radius, its reading spacing)."""
    acquire = run.context.published("align_acquire")
    rows = [r for r in run.rows if r[8] == algorithm.PHASES.index("Acquire") and not r[10]]
    reach = {}
    for search in dict.fromkeys(a["search"] for a in acquire if a["search"]):
        payloads = [a for a in acquire if a["search"] == search]
        u, v = (algorithm.LOCAL_INDEX[a] for a in algorithm.SEARCH_AXES[search])
        far = max(math.hypot(r[u], r[v]) for r in rows if payloads[0]["t"] <= r[9] <= payloads[-1]["t"])
        reach[search] = (far, payloads[0]["radius"], payloads[0]["pitch"] / algorithm.SPIRAL_READINGS_PER_PITCH)
    return reach


def test_a_full_search_runs_both_spirals_to_their_radius_even_with_light_at_the_start():
    run = simulate(Scenario(offset=CASES["cell-run"].offset, params=FULL))
    assert run.error is None, run.error
    assert run.rows[0][6] >= 1e-6, "no light at the start"
    assert searches(run) == ["angle", "angle", "lateral", "lateral"]
    assert all(a["full"] and a["warm"] for a in run.context.published("align_acquire"))
    for search, (far, radius, spacing) in spiral_reach(run).items():
        assert far >= radius - spacing, f"the {search} spiral stopped at {far:.3g} of {radius:g}"
    assert run.result.passed
    assert run.result.power_w >= 0.97 * run.optics.best_power_w()


def test_a_full_search_from_no_light_aligns_too():
    run = simulate(Scenario(offset=CASES["angle-off"].offset, seed=3, params=FULL))
    assert run.error is None, run.error
    assert run.rows[0][6] < 1e-6
    assert searches(run) == ["angle", "angle", "lateral", "lateral"]
    assert all(far >= radius - spacing for far, radius, spacing in spiral_reach(run).values())
    assert run.result.power_w >= 0.97 * run.optics.best_power_w()
    # Off, the tilt spiral stops once past the light, well inside its radius.
    far, radius, _ = spiral_reach(simulate(CASES["angle-off"]))["angle"]
    assert far < 0.6 * radius
    # In the dark, it still stops with the error.
    run = simulate(Scenario(dark=True, params=FULL))
    assert isinstance(run.error, AlignmentNoSignal)
    assert searches(run) == ["angle", "lateral"]


def test_a_full_search_goes_back_to_the_light_the_lateral_spiral_missed(monkeypatch):
    original = algorithm.spiral

    def blind_lateral(io, state, search, info, full=False):
        if search != "lateral":
            return original(io, state, search, info, full)
        detect, state.detect_w = state.detect_w, math.inf  # sees no light
        try:
            return original(io, state, search, info, full)
        finally:
            state.detect_w = detect

    monkeypatch.setattr(algorithm, "spiral", blind_lateral)
    run = simulate(Scenario(offset=CASES["cell-run"].offset, params=FULL))
    assert run.error is None, run.error
    assert searches(run) == ["angle", "angle", "lateral"]
    rows = [r for r in run.rows if r[8] == algorithm.PHASES.index("Acquire")]
    still = [r for r in rows if r[10]]
    # The tilt spiral's best, then the same pose again once the lateral spiral is over.
    assert len(still) == 2 and still[-1][9] > max(r[9] for r in rows if not r[10])
    tilt = run.context.published("align_acquire")[1]["at"]
    for r in still:
        assert [r[3], r[5]] == pytest.approx(tilt, abs=1e-3) and math.hypot(r[0], r[2]) < 1e-3
    assert run.result.power_w >= 0.97 * run.optics.best_power_w()


def test_the_peak_pattern_spans_a_fifth_of_the_width():
    run = simulate(CASES["cell-run"])
    widths = {}
    for p in run.context.published("align_profile"):
        if p["axis"] in ("x", "z") and p["chosen"] is not None:
            widths[p["axis"]] = p["fit"]["width"]  # the latest width measured: the one the Peak phase uses
    mean = 0.5 * (widths["x"] + widths["z"])
    assert mean == pytest.approx(BEAM_WIDTH_UM, rel=0.1)
    radii = [p["radius_um"] for p in run.context.published("align_peak")]
    assert radii and all(r == pytest.approx(0.2 * mean, rel=1e-3) for r in radii)  # ±48 µm on the cell's ends


@pytest.mark.parametrize("grid", [3, 5])
def test_the_peak_pattern_is_a_square_of_peak_grid_readings_taken_without_long_moves(grid):
    run = simulate(Scenario(offset=CASES["cell-run"].offset, params={"peak_grid": grid}))
    assert run.error is None, run.error
    half = grid // 2
    for p in run.context.published("align_peak"):
        (cx, cz), spacing = p["center"], 2 * p["radius_um"] / (grid - 1)
        expected = sorted((cx + i * spacing, cz + j * spacing) for i in range(-half, half + 1) for j in range(-half, half + 1))
        got = sorted((x, z) for x, z, _ in p["points"])
        assert len(got) == grid * grid
        assert [c for pt in got for c in pt] == pytest.approx([c for pt in expected for c in pt], abs=2e-3)
        # In reading order, each point is next to the one before.
        steps = [math.dist(a[:2], b[:2]) for a, b in zip(p["points"], p["points"][1:])]
        assert max(steps) <= spacing + 2e-3


@pytest.mark.parametrize("grid", [1, 4, 11])
def test_a_peak_grid_without_a_centre_or_out_of_bounds_is_refused_before_moving(grid):
    run = simulate(Scenario(params={"peak_grid": grid}))
    assert isinstance(run.error, AlignmentError) and "peak_grid" in str(run.error)
    assert run.robot.moves == []


def test_the_walk_shows_the_tcp_off_the_tip():
    run = simulate(CASES["smf-tcp-behind-tip"])
    walks = [p["walk_um_per_deg"] for p in run.context.published("align_profile") if p["axis"] in ("rx", "rz")]
    # 500 µm behind the tip, plus the 20 µm gap the beam crosses, per degree.
    expected = 520 * math.pi / 180
    assert walks and all(w is not None and abs(abs(w) - expected) < 0.5 for w in walks)


def test_no_light_stops_with_a_clear_error():
    run = simulate(Scenario(dark=True))
    assert isinstance(run.error, AlignmentNoSignal)
    assert "tilting" in str(run.error) and "moving" in str(run.error) and "check the laser" in str(run.error)
    assert searches(run) == ["angle", "lateral"]
    end = run.context.events("align_end")[-1]
    assert end["passed"] is False and "AlignmentNoSignal" in end["error"]
    assert run.summary["error"]


@pytest.mark.parametrize("params, name", [
    ({"acquire_radius_um": 1950.0}, "acquire_radius_um"),
    ({"acquire_radius_deg": 7.8}, "acquire_radius_deg"),
])
def test_a_window_too_small_for_a_search_is_refused_before_moving(params, name):
    run = simulate(Scenario(params=params))
    assert isinstance(run.error, AlignmentError)
    assert name in str(run.error)
    assert run.robot.moves == []


# --- One cross scan --------------------------------------------------------------------------

def cross_scan_once(scenario, width_um, start_x):
    """One cross_scan along x from ``start_x`` with a width guess of ``width_um``."""
    out = {}

    def probe(io, settings):
        with io.phase("Probe"):
            state = algorithm._State(settings, width={"x": width_um, "z": width_um}, angle_width=0.3,
                                     scan_range_w=1.2e-3, detect_w=1e-6)
            state.pos["x"] = start_x
            out["lit"] = algorithm.cross_scan(io, state, "x", 0)
            out["state"] = state

    run = simulate(scenario, align=probe, phases=("Probe",), phase_views={})
    assert run.error is None, run.error
    return run, out["lit"], out["state"]


def scanned_span(run):
    spans = [p["span"] for p in run.context.published("align_profile")]
    return min(s[0] for s in spans), max(s[1] for s in spans)


def test_a_window_narrower_than_the_beam_still_finds_its_centre():
    # A ±45 µm window on a beam of 240 µm: the fit over the noise floor still centres it.
    run, lit, state = cross_scan_once(Scenario(), width_um=30.0, start_x=20.0)
    assert lit
    assert abs(state.pos["x"]) < 2.0
    assert 30.0 < state.width["x"] <= 90.0  # grows towards 240 µm, ×3 at most per fit


def test_a_centre_out_of_reach_is_never_extrapolated():
    # The centre 400 µm away: the windows (±45 µm, moving by a half-span) cannot reach it.
    run, lit, state = cross_scan_once(Scenario(offset=(400, 0, 0, 0)), width_um=30.0, start_x=0.0)
    lo, hi = scanned_span(run)
    assert lit
    assert lo <= state.pos["x"] <= hi < 400.0
    best = max((pt for p in run.context.published("align_profile") for q in p["passes"] for pt in q["points"]),
               key=lambda pt: pt[1])
    assert state.pos["x"] == pytest.approx(best[0], abs=1e-3)  # published to the nm


def test_no_light_along_the_line_leaves_the_fiber_where_it_is():
    run, lit, state = cross_scan_once(Scenario(dark=True), width_um=30.0, start_x=12.0)
    assert not lit
    assert state.pos["x"] == 12.0
    # The window doubled before giving up.
    spans = [p["span"][1] - p["span"][0] for p in run.context.published("align_profile")]
    assert len(spans) == algorithm.DARK_WIDENINGS + 1 and spans[-1] > spans[0] * 7
