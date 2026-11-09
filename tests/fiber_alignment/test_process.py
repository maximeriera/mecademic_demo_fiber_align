"""The process steps that need no robot program: holding the final position."""

import logging
from types import SimpleNamespace

import pytest

from app_logic.fiber_alignment import process
from app_logic.fiber_alignment.cell import POWER_METER, ROBOT, ProcessError

from .sim import FakeContext, SimClock


class Context(FakeContext):
    """FakeContext with the variables the steps read."""

    def __init__(self, clock, params, variables):
        super().__init__(clock, params)
        self.variables = dict(variables)

    def get_variable(self, name, default=None):
        return self.variables.get(name, default)


class Meter:
    """A meter whose power falls a little at each reading; ``on_read`` runs before each."""

    def __init__(self, on_read=None):
        self.reads = 0
        self.on_read = on_read

    def read_power(self):
        if self.on_read:
            self.on_read(self.reads)
        self.reads += 1
        return 1e-3 * (0.92 - 0.001 * self.reads)


def hold(monkeypatch, hold_s, meter, at_alignment=True):
    clock = SimClock()
    monkeypatch.setattr(process, "time", SimpleNamespace(monotonic=clock.monotonic, sleep=clock.sleep))
    context = Context(clock, {"hold_final_s": hold_s}, {"at_alignment": at_alignment})
    devices = {ROBOT: SimpleNamespace(logger=logging.getLogger("test")), POWER_METER: SimpleNamespace(api=meter)}
    return context, clock, lambda: process.hold_final_position(devices, context)


def test_the_fiber_holds_its_final_position_for_hold_final_s_reading_the_power(monkeypatch):
    meter = Meter()
    context, clock, run = hold(monkeypatch, 5.0, meter)
    first, last, lowest = run()
    assert context.steps == ["Hold final position"]
    assert clock.now == pytest.approx(5.0)
    assert meter.reads == 1 + round(5.0 / process.HOLD_READ_S)  # one at the start, then every HOLD_READ_S
    assert first == pytest.approx(0.919e-3) and last == lowest < first


def test_an_abort_ends_the_hold_at_once(monkeypatch):
    # The controller marks the abort before stopping the task: the next check sees it.
    context = None

    def abort_on_third(reads):
        if reads == 2:
            context.mark("abort")

    meter = Meter(on_read=abort_on_third)
    context, clock, run = hold(monkeypatch, 60.0, meter)
    with pytest.raises(ProcessError, match="interrupted"):
        run()
    assert clock.now < 2.0


def test_no_hold_at_zero_and_none_away_from_the_station(monkeypatch):
    meter = Meter()
    context, clock, run = hold(monkeypatch, 0.0, meter)
    assert run() is None and meter.reads == 0 and clock.now == 0.0
    assert context.steps == ["Hold final position"]
    context, clock, run = hold(monkeypatch, 5.0, Meter(), at_alignment=False)
    with pytest.raises(ProcessError, match="alignment station"):
        run()
