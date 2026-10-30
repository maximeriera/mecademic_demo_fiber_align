"""The contract layer (alignment.py), on the simulated cell."""

import math

import pytest

from app_logic.fiber_alignment import alignment, frames
from app_logic.fiber_alignment.alignment import _interpolate, _Stamped

from .sim import START_POSE, Scenario, simulate


def test_local_and_pose_at_are_inverse_in_the_start_tool_frame():
    run = simulate(Scenario(), align=lambda io, settings: _check_frame(io), phases=("Check",), phase_views={})
    assert run.error is None


def _check_frame(io):
    offsets = (12.5, -3.0, 40.0, 0.7, -0.2, 1.3)
    pose = io.pose_at(*offsets)
    assert io.local(pose) == pytest.approx(offsets, abs=1e-6)
    # A pure lateral offset moves along the start's tool x axis, not the WRF's.
    T = frames.pose_matrix(START_POSE)
    expected = T[:3, 3] + T[:3, 0] * 0.1
    assert list(io.pose_at(x=100.0)[:3]) == pytest.approx(expected, abs=1e-9)


def test_poses_are_read_as_the_cell_answers_them():
    # On the cell, GetRtTargetCartPos times out ("timeout waiting for
    # interruptable event"): every pose is a GetRtCartPos, and none is asked
    # while the robot's motion is paused.
    run = simulate(Scenario(offset=(120, -80, 0.8, -0.5)))
    assert run.error is None, run.error
    assert run.robot.pose_queries
    assert {name for name, _ in run.robot.pose_queries} == {"GetRtCartPos"}
    assert not any(paused for _, paused in run.robot.pose_queries)


def test_interpolate_takes_angles_the_short_way():
    before = _Stamped(0.0, (0.0, 0.0, 0.0, 179.0, 0.0, -10.0))
    after = _Stamped(1.0, (1.0, 2.0, 3.0, -179.0, 0.0, 10.0))
    mid = _interpolate(before, after, 0.5)
    assert mid[:3] == pytest.approx((0.5, 1.0, 1.5))
    assert alignment._wrap_deg(mid[3]) == pytest.approx(-180.0)
    assert mid[5] == pytest.approx(0.0)
    assert _interpolate(before, after, 7.0) == pytest.approx(after.pose)


def test_scan_readings_sit_on_the_path_despite_the_reading_time():
    # Slow meter and robot replies: without interpolation, readings land late along the scan.
    def two_way_lags(patch):
        saved = alignment._interpolate
        if patch:
            alignment._interpolate = lambda before, after, t: after.pose
        try:
            run = simulate(Scenario(offset=(150, 0, 0.5, 0.5), latency_s=0.004, pose_query_s=0.003))
        finally:
            alignment._interpolate = saved
        assert run.error is None
        return [p["fit"]["lag"] for p in run.context.published("align_profile")
                if p["fit"] and p["fit"].get("lag") is not None]

    # Interpolated, what remains is a twentieth of the lag at most.
    assert max(abs(lag) for lag in two_way_lags(patch=False)) < 0.05 * min(abs(lag) for lag in two_way_lags(patch=True))


def test_the_verdict_goes_back_to_the_best_reading_at_rest():
    def wander(io, settings):
        with io.phase("Wander"):
            io.read_still("cart")                   # at the optimum
            io.move_to(io.pose_at(x=150.0))         # far off it (the beam is ~240 µm wide)
    run = simulate(Scenario(), align=wander, phases=("Wander",), phase_views={})
    assert run.error is None
    assert run.result.power_w > 0.97 * run.optics.best_power_w()
    assert run.summary["final_local"][0] == pytest.approx(0.0, abs=1e-6)


def test_a_timeout_returns_to_the_best_reading_at_rest():
    def stall(io, settings):
        with io.phase("Stall"):
            io.read_still("cart")
            io.move_to(io.pose_at(z=25.0))
            while True:
                io.read_power("cart")
    run = simulate(Scenario(params={"align_timeout_s": 2.0}), align=stall, phases=("Stall",), phase_views={})
    assert run.error is None
    assert run.result.timed_out
    assert run.result.power_w > 0.97 * run.optics.best_power_w()


def test_rows_are_local_offsets_and_still_readings_survive_the_summary():
    def few(io, settings):
        with io.phase("Few"):
            for x in (0.0, 5.0, -5.0):
                io.move_to(io.pose_at(x=x))
                io.read_still("cart")
            for _ in range(10):
                io.read_power("cart")
    run = simulate(Scenario(), align=few, phases=("Few",), phase_views={"Few": {"type": "map", "axes": ["x", "z"]}})
    rows = run.rows
    assert all(len(r) == 11 for r in rows)
    still = [r for r in rows if r[10]]
    assert [round(r[0], 3) for r in still] == [0.0, 5.0, -5.0]
    assert run.summary["sample_format"] == alignment.SAMPLE_FORMAT
    assert sum(1 for r in run.summary["points"] if r[10]) == len(still)
    assert run.summary["phase_views"] == {"Few": {"type": "map", "axes": ["x", "z"]}}


def test_an_overrange_never_counts_as_the_best():
    def overrange(io, settings):
        with io.phase("Range"):
            io.set_power_range(1e-6)               # far below the light at the optimum
            assert math.isinf(io.read_power("cart"))
            io.set_power_range(None)
            io.read_still("cart")
    run = simulate(Scenario(), align=overrange, phases=("Range",), phase_views={})
    assert run.error is None
    assert math.isfinite(run.summary["best_still"][6])
    # The range the algorithm fixed is undone afterwards.
    assert run.meter.auto_range is True
