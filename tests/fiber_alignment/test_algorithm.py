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
    # No light at the start, the tilt off: a rotation scan finds it.
    "angle-off": Scenario(offset=(40, 30, 1.2, -0.4), seed=3),
    # No light at the start, the tilt right but 600 µm off: a translation scan finds it.
    "lateral-off": Scenario(offset=(600, 0, 0.1, 0.0), seed=4, params={"acquire_radius_um": 800.0}),
    "slow-io": Scenario(offset=(80, 40, 0.3, 0.2), latency_s=0.004, pose_query_s=0.003, seed=7),
    # Bare single-mode fibers: the same algorithm, other widths.
    "smf-near": Scenario(optics="smf", offset=(120, -80, 0.8, -0.5)),
    # 390 µm off, 26 capture radii of these fibers: only the last, 500 µm disc
    # of the search reaches it, a Lissajous ~155 mm long (it covers its whole
    # disc evenly, where a spiral would stop on its way out: 71 s in all), at
    # 0.2 mm/s (a 30 µm capture diameter in MIN_CROSSING_S): ~455 s.
    "smf-far": Scenario(optics="smf", offset=(300, 250, 1.5, 1.0), seed=3, params={"align_timeout_s": 600.0}),
    "smf-tcp-behind-tip": Scenario(optics="smf", offset=(80, 60, 1.0, -1.0), pivot_um=(0, 500, 0), seed=5),
}


@pytest.fixture(scope="module", params=sorted(CASES))
def run(request):
    return simulate(CASES[request.param])


#: Cases slower than 120 s, and how slow (s).
SLOW = {"smf-far": 500.0}


def test_it_reaches_the_peak(run, request):
    assert run.error is None, run.error
    assert run.result.passed
    assert run.result.power_w >= 0.97 * run.optics.best_power_w()
    assert run.result.duration_s < SLOW.get(request.node.callspec.params["run"], 120.0)


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


def test_no_light_moves_first_then_tilts():
    # Translation first: a scan in x/z, then one in Rx/Rz, around the start.
    # Off in tilt, the rotation scan finds the light, or a hint of it that
    # the scans around it then turn into light; off sideways, the first
    # translation scan.
    run = simulate(CASES["angle-off"])
    assert run.rows[0][6] < 1e-6
    acquire = run.context.published("align_acquire")
    assert searches(run)[:2] == ["lateral", "angle"]
    assert acquire[-1]["found"]
    assert acquire[-1]["search"] == "angle" or acquire[2]["around"] == "hint 1"
    run = simulate(CASES["lateral-off"])
    assert searches(run) == ["lateral", "lateral"]


FULL = {"acquire_full_search": True}


def scans(run):
    """Each scan of the first-light search: (its search, its payload, its readings on the move)."""
    acquire = run.context.published("align_acquire")
    rows = [r for r in run.rows if r[8] == algorithm.PHASES.index("Acquire") and not r[10]]
    out = []
    for k, a in enumerate(acquire):
        if a["search"] and not a["found"]:
            t1 = acquire[k + 1]["t"] if k + 1 < len(acquire) else math.inf
            out.append((a["search"], a, [r for r in rows if a["t"] <= r[9] < t1]))
    return out


def reach(payload, readings):
    """How far a scan's readings went from the start along the nearer of its two axes, and its spacing."""
    u, v = (algorithm.LOCAL_INDEX[a] for a in algorithm.SEARCH_AXES[payload["search"]])
    return min(max(abs(r[u]) for r in readings), max(abs(r[v]) for r in readings)), payload["spacing"]


def test_a_full_search_runs_both_scans_to_their_end_even_with_light_at_the_start():
    run = simulate(Scenario(offset=CASES["cell-run"].offset, params=FULL))
    assert run.error is None, run.error
    assert run.rows[0][6] >= 1e-6, "no light at the start"
    acquire = run.context.published("align_acquire")
    assert searches(run)[:2] == ["lateral", "angle"] and sum(a["found"] for a in acquire) == 1
    assert all(a["full"] and a["warm"] for a in acquire)
    for search, payload, readings in scans(run):
        far, spacing = reach(payload, readings)
        assert far >= payload["radius"] - spacing, f"the {search} scan stopped at {far:.3g} of {payload['radius']:g}"
    assert run.result.passed
    assert run.result.power_w >= 0.97 * run.optics.best_power_w()


def test_a_full_search_ends_on_the_brightest_light_of_its_two_scans():
    run = simulate(Scenario(offset=CASES["cell-run"].offset, params=FULL))
    brightest = max(r[6] for _, _, readings in scans(run) for r in readings)
    found = [a for a in run.context.published("align_acquire") if a["found"]][-1]
    # At rest, where it was read (or found again close by).
    assert found["power_w"] >= algorithm.CONFIRM_FRACTION * brightest


def test_a_full_search_from_no_light_aligns_too():
    run = simulate(Scenario(offset=CASES["angle-off"].offset, seed=3, params=FULL))
    assert run.error is None, run.error
    assert run.rows[0][6] < 1e-6
    assert searches(run)[:2] == ["lateral", "angle"]
    for search, payload, readings in scans(run):
        far, spacing = reach(payload, readings)
        assert far >= payload["radius"] - spacing, f"the {search} scan stopped at {far:.3g} of {payload['radius']:g}"
    assert run.result.power_w >= 0.97 * run.optics.best_power_w()
    # Not full, the rotation scan stops at the light, well before the end of
    # its path: 0.9° off, well inside its first disc (± 0.67° and what it sees).
    near = simulate(Scenario(offset=(40, 30, 0.9, -0.3), seed=3))
    assert near.rows[0][6] < 1e-6
    payload, readings = [(p, r) for search, p, r in scans(near) if search == "angle"][-1]
    capture = payload["spacing"] / algorithm.LISSAJOUS_READING
    n = max(2, math.ceil(algorithm.LISSAJOUS_GAP * payload["radius"] / (algorithm.LISSAJOUS_COVER * capture)))
    path = algorithm.lissajous_path((0.0, 0.0), payload["radius"], n, payload["spacing"])
    u, v = (algorithm.LOCAL_INDEX[a] for a in algorithm.SEARCH_AXES["angle"])
    travelled = sum(math.hypot(b[u] - a[u], b[v] - a[v]) for a, b in zip(readings, readings[1:]))
    assert travelled < 0.6 * sum(math.hypot(*(b - a)) for a, b in zip(path, path[1:]))
    # In the dark, the full scans, then every size of the rounds, then the error.
    run = simulate(Scenario(dark=True, params=FULL))
    assert isinstance(run.error, AlignmentNoSignal)
    assert searches(run) == ["lateral", "angle"] * (1 + len(algorithm.ACQUIRE_STAGES))


#: The cell in November 2026: a narrow core with faint wide wings (fitted to its
#: 466 runs of Nov 5-6), a 90 µm/° walk, its params, 36 nW of stray light.
CELL_2026_11 = dict(width_um=75.0, angle_deg=0.075, wing=(1e-2, 300.0, 0.3), gap_um=90.0 / math.radians(1.0),
                    background_w=3.6e-8)
CELL_2026_11_PARAMS = {"lateral_width_um": 75.0, "angle_width_deg": 0.075, "signal_detect_uw": 0.5,
                       "acquire_capture_um": 250.0, "acquire_capture_deg": 0.3, "power_threshold_mw": 0.9}


def acquire_only(scenario):
    """The Acquire phase alone in ``scenario``: (the run, its state after Acquire)."""
    out = {}

    def probe(io, settings):
        with io.phase("Acquire"):
            lateral = settings.lateral_width_mm * 1e3
            state = algorithm._State(settings, width={"x": lateral, "z": lateral}, angle_width=settings.angle_width_deg,
                                     scan_range_w=algorithm.SCAN_RANGE * settings.source_power_w)
            algorithm.acquire(io, state)
            out["state"] = state

    run = simulate(scenario, align=probe, phases=("Acquire",), phase_views={})
    assert run.error is None, run.error
    return run, out["state"]


def test_a_hint_of_light_restarts_the_search_around_it():
    # 1.2° off in tilt: the first rotation scan (± 0.67°) only reaches the
    # faint edge of the light, under the detection level. Confirmed at rest,
    # that hint becomes the centre, and the scans around it find the light.
    run, state = acquire_only(Scenario(offset=(0.0, 0.0, 1.2, 0.0), params=CELL_2026_11_PARAMS, **CELL_2026_11))
    acquire = run.context.published("align_acquire")
    assert any(a["around"] == "hint 1" for a in acquire), [(a["search"], a["around"]) for a in acquire]
    assert acquire[-1]["found"] and state.peak_w >= state.detect_w
    assert acquire[-1]["restarts"] >= 1


def test_back_beside_the_light_finds_it_again(monkeypatch):
    # Poses reported late on the move: a reading is plotted at a pose the
    # fiber had passed when the light reached the meter, so going back to it
    # lands beside the light. Going back over the path around it, slower,
    # finds it again: Acquire ends on light.
    slow = []
    original = algorithm._back_along

    def watched(io, state, name, *args):
        slow.append(name)
        return original(io, state, name, *args)

    monkeypatch.setattr(algorithm, "_back_along", watched)
    run, state = acquire_only(Scenario(offset=(-60.0, 50.0, 0.55, -0.2), params=CELL_2026_11_PARAMS, pose_delay_s=0.06,
                                       **CELL_2026_11))
    assert slow, "the fiber never had to look again"
    found = run.context.published("align_acquire")[-1]
    assert found["found"] and found["power_w"] >= state.detect_w
    # Where Acquire left the fiber, light reaches the meter at rest.
    still = [r for r in run.rows if r[10]]
    assert still[-1][6] >= state.detect_w


def slow_meter(offset, range_up_s):
    """The cell on 2026-11-06: ~0.1 nW of stray light, so the meter sits in its
    lowest range (120 nW here) and stays there ``range_up_s`` after light
    exceeds it, reading the range's top meanwhile."""
    return Scenario(offset=offset, params=CELL_2026_11_PARAMS, meter_range_up_s=range_up_s, meter_lowest_range_w=1.2e-7,
                    **{**CELL_2026_11, "background_w": 1e-10})


def test_light_read_far_under_its_power_on_the_move_is_a_hint():
    # On the cell, scans crossing µW read 50-100 nW: a fiftieth of the
    # detection level is a hint, a quarter would have ignored them all.
    run, state = acquire_only(slow_meter((-131.29, -42.02, -2.0401, 0.3002), 0.2))
    acquire = run.context.published("align_acquire")
    restart = next(a["t"] for a in acquire if a["around"] == "hint 1")
    before = [r[6] for r in run.rows if not r[10] and r[9] < restart]
    assert max(before) < 0.25 * state.detect_w, max(before)
    assert acquire[-1]["found"] and state.peak_w >= state.detect_w


def test_light_under_a_faintly_lit_centre_is_still_light():
    # 180 nW at the start: a hint must beat 3× that, more than the detection
    # level, but light at the detection level counts all the same.
    run, state = acquire_only(slow_meter((98.41, -146.97, -0.3572, -0.3573), 0.3))
    assert 0.1 * state.detect_w < run.rows[0][6] < state.detect_w
    assert run.context.published("align_acquire")[-1]["found"] and state.peak_w >= state.detect_w


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
    assert searches(run) == ["lateral", "angle"] * len(algorithm.ACQUIRE_STAGES)
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
