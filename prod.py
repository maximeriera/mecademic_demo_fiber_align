"""PROD task: one fiber per cycle. Pick it, align it, hold it there, retract, put it back.

With the ``prod_align_only`` param on, a cycle only aligns the fiber already in
the gripper, from a random start: Move to alignment start, Random offset,
Align, Hold, Retract (process.run_align_only_cycle). Nothing is picked or placed,
``next_fiber`` does not advance, and a search that finds no light counts in
``failed_count`` instead of faulting the cell.

The full cycle works through the rack in order. ``next_fiber`` (context.yaml) says
which slot comes next, and advances 1 → N → 1 after each cycle, recovered ones
included. A cycle that faults leaves it unchanged, so production resumes on the same fiber
once the cell is recovered.

A fiber that does not reach the threshold is still put back and counted in
``failed_count``, and the loop goes on with the next fiber. So does a cycle
that hits an error at the alignment station (a software error, a lost reply):
the fiber is retracted and put back, the error counted in ``error_count`` and
kept in ``last_error`` (process.recover). An ABORT, a device fault, or an
error at the rack still faults the cell.

The steps themselves live in fiber_alignment/process.py, shared with the
Manual tab.
"""

import time
from typing import Dict

from core import AppContext
from devices import Device

# Imported both ways on purpose: core/Task.py resolves this module as flat
# "prod" when --workspace points at a folder holding the task modules
# directly, and as "app_logic.prod" when it holds an app_logic/ folder. The
# relative form has no parent package in the first case.
try:
    from .fiber_alignment import poses, process
except ImportError as e:  # pragma: no cover - depends on how the workspace is loaded
    if e.name is not None:
        raise  # a module is really missing (e.g. a dependency): report that, not the fallback
    from fiber_alignment import poses, process


def prod_cycle(devices: Dict[str, Device], context: AppContext):
    """Run one production cycle on fiber ``next_fiber``, or an alignment-only cycle (``prod_align_only``).

    An error at the alignment station retracts the fiber, puts it back (in a
    full cycle) and goes on with the next one (process.recover); it counts in
    ``error_count``. Any other error faults the cell, as before.
    """
    align_only = bool(context.get_param("prod_align_only", False))
    # Declared every cycle: the other mode's sequence would otherwise stay in force.
    context.declare_sequence(process.ALIGN_ONLY_STEPS if align_only else process.CYCLE_STEPS)
    fiber = None if align_only else context.get_variable("next_fiber", 1)
    started = context.data.now()
    try:
        if align_only:
            result = process.run_align_only_cycle(devices, context)
        else:
            result = process.run_cycle(devices, context, fiber)
    except Exception as e:
        process.recover(devices, context, e, place=not align_only, since=started)  # re-raises when it cannot
        result = None
        counter = None
    else:
        counter = "aligned_count" if result is not None and result.passed else "failed_count"
    if fiber is not None:
        context.set_variable("next_fiber", poses.next_slot_number(fiber))
    if counter:
        context.set_variable(counter, context.get_variable(counter, 0) + 1)

    # Demo pacing, deliberately outside any step: it shows as idle time on the
    # Production tab. Remove it, or set cycle_pause_s to 0, to run flat out.
    time.sleep(context.get_param("cycle_pause_s", 1.0))
