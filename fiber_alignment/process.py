"""The steps of the fiber alignment process.

Pick, Move to alignment start, Align, Retract and Place make the PROD cycle;
with ``prod_align_only``, PROD runs Move to alignment start, Random offset,
Align and Retract instead (``run_align_only_cycle``). Shared by the PROD cycle
(prod.py) and the Manual tab (manual_actions.py), so a step tested by hand is
exactly the step production runs. Each function:

* reports itself as one step, whose name matches ``sequences:`` in config.yaml;
* checks the cell state first and raises ProcessError, before anything moves,
  when the step would be unsafe or meaningless (e.g. Pick with a fiber already
  in the gripper);
* ends in a safe pose, so the Manual tab can run the steps in any order;
* keeps the ``held_fiber`` and ``at_alignment`` variables (context.yaml) up to
  date. They are how the cell knows what it holds and where it is, across steps
  and app restarts.

Absolute moves are robot programs (see poses.py); only the alignment search
moves the robot from Python.
"""

from __future__ import annotations

import math
import random
from typing import Any, Callable, Dict, Optional, Tuple

from core import AppContext
from devices import Device

from . import algorithm, gripper, memory, poses, runlog
from .alignment import AlignmentNoSignal, AlignmentResult, AlignmentSettings, run_alignment
from .cell import ROBOT, ProcessError, power_meter, prepare_robot, run_program

#: The steps of a PROD cycle, in order (the same as ``sequences: prod_cycle`` in
#: config.yaml), and those of the alignment-only cycle. prod.py declares the
#: one it runs, for "step 2 of 4" on the Production tab.
CYCLE_STEPS = ["Pick fiber", "Move to alignment start", "Align fiber", "Retract from alignment", "Place fiber"]
ALIGN_ONLY_STEPS = ["Move to alignment start", "Random offset", "Align fiber", "Retract from alignment"]


def pick_fiber(devices: Dict[str, Device], context: AppContext, number: int) -> None:
    """Take fiber ``number`` from its slot: ``fiber<N>_in``, close the gripper, ``fiber<N>_out``."""
    with context.step("Pick fiber"):
        held = context.get_variable("held_fiber", 0)
        if held:
            raise ProcessError(
                f"The gripper already holds fiber {held}: put it back first (Manual: Place Fiber)."
            )
        if context.get_variable("at_alignment", False):
            raise ProcessError("The robot is at the alignment station: Retract first.")
        slot = poses.fiber_slot(number)
        robot = prepare_robot(devices, context)

        gripper.open_gripper(devices, context)
        run_program(devices, robot, slot.program_in)
        with context.step("Close gripper"):
            gripper.close_gripper(devices, context)
        context.set_variable("held_fiber", slot.number)
        run_program(devices, robot, slot.program_out)


def move_to_alignment_start(devices: Dict[str, Device], context: AppContext) -> None:
    """Bring the held fiber in front of the fixed fiber: the ``align_start`` program."""
    with context.step("Move to alignment start"):
        if context.get_variable("at_alignment", False):
            raise ProcessError("The robot is already at the alignment station: Retract first.")
        if not context.get_variable("held_fiber", 0):
            devices[ROBOT].logger.warning("Moving to the alignment start with no fiber in the gripper.")
        robot = prepare_robot(devices, context)

        # Set before the program: if it is interrupted near the fixed fiber,
        # HOME still backs out with align_retract first.
        context.set_variable("at_alignment", True)
        run_program(devices, robot, poses.ALIGN_START_PROGRAM)


def align_fiber(devices: Dict[str, Device], context: AppContext,
                on_summary: Optional[Callable[[Dict[str, Any]], None]] = None) -> AlignmentResult:
    """Run the alignment algorithm from the current pose, then judge it against the threshold.

    The search stays within ``align_range_um`` / ``align_range_deg`` of where it
    starts. It only makes sense at the alignment station, with a fiber. It
    starts the search where this fiber aligned last (see memory.py), unless
    ``use_remembered_pose`` is off, and remembers where it ends, whenever that
    is in the light. ``on_summary`` gets the run's summary (run_alignment).
    """
    with context.step("Align fiber"):
        logger = devices[ROBOT].logger
        if not context.get_variable("at_alignment", False):
            logger.warning("Aligning away from the alignment station (at_alignment is false).")
        fiber = context.get_variable("held_fiber", 0)
        robot = prepare_robot(devices, context)

        if context.get_param("use_remembered_pose", True):
            seed = memory.remembered_pose(context, fiber, logger)
        else:
            seed = None
            logger.info("use_remembered_pose is off: the search starts from the current pose.")
        result = run_alignment(robot, power_meter(devices), context, fiber=fiber, align=algorithm.align,
                               logger=logger, phases=algorithm.PHASES, phase_views=algorithm.PHASE_VIEWS,
                               seed=seed, on_summary=on_summary)

        # In the light, even under the threshold: a better start for this fiber than align_start.
        if math.isfinite(result.power_w) and result.power_w >= AlignmentSettings.from_context(context).signal_detect_w:
            memory.remember(context, fiber, result.pose, result.power_w, logger)

        power_mw = result.power_w * 1e3
        # Overrange reads +inf, which the Variables tab could not display.
        context.set_variable("last_align_power_mw", power_mw if math.isfinite(power_mw) else 0.0)
        log = logger.info if result.passed else logger.warning
        log(f"Fiber {fiber}: alignment {'passed' if result.passed else 'FAILED'} at {power_mw:.4g} mW "
            f"({result.reads} readings, {result.duration_s:.1f} s{', timed out' if result.timed_out else ''}).")
        return result


def random_offset(devices: Dict[str, Device], context: AppContext) -> Tuple[float, float, float, float]:
    """Move the fiber by a random offset in its tool frame (``MoveLinRelTrf``), at the alignment station.

    Laterally (X/Z) up to ``random_offset_um``, in tilt (Rx/Rz) up to
    ``random_offset_deg``, each drawn evenly over a disc of that radius.
    Never along or about the fiber axis (Y): the gap stays the one
    ``align_start`` taught. Returns (x µm, z µm, rx °, rz °).
    """
    with context.step("Random offset"):
        if not context.get_variable("at_alignment", False):
            raise ProcessError("The robot is not at the alignment station: Move to Alignment Start first.")
        x, z = _in_disc(max(0.0, float(context.get_param("random_offset_um", 300.0))))
        rx, rz = _in_disc(max(0.0, float(context.get_param("random_offset_deg", 1.0))))
        robot = prepare_robot(devices, context)

        devices[ROBOT].logger.info(f"Random offset (tool frame): x {x:+.1f} µm, z {z:+.1f} µm, "
                                   f"rx {rx:+.3f}°, rz {rz:+.3f}°")
        robot.MoveLinRelTrf(x * 1e-3, 0.0, z * 1e-3, rx, 0.0, rz)
        robot.WaitIdle()
        return x, z, rx, rz


def _in_disc(radius: float) -> Tuple[float, float]:
    """A point drawn evenly over a disc of ``radius`` around 0."""
    r = radius * math.sqrt(random.random())
    angle = 2.0 * math.pi * random.random()
    return r * math.cos(angle), r * math.sin(angle)


def retract_from_alignment(devices: Dict[str, Device], context: AppContext) -> None:
    """Back the fiber out of the alignment station: the ``align_retract`` program."""
    with context.step("Retract from alignment"):
        if not context.get_variable("at_alignment", False):
            raise ProcessError("The robot is not at the alignment station: nothing to retract from.")
        robot = prepare_robot(devices, context)

        run_program(devices, robot, poses.ALIGN_RETRACT_PROGRAM)
        context.set_variable("at_alignment", False)


def place_fiber(devices: Dict[str, Device], context: AppContext) -> None:
    """Put the held fiber back into its own slot: ``fiber<N>_in``, open the gripper, ``fiber<N>_out``."""
    with context.step("Place fiber"):
        held = context.get_variable("held_fiber", 0)
        if not held:
            raise ProcessError("The gripper holds no fiber (held_fiber is 0): nothing to place.")
        if context.get_variable("at_alignment", False):
            raise ProcessError("The robot is at the alignment station: Retract first.")
        slot = poses.fiber_slot(held)
        robot = prepare_robot(devices, context)

        run_program(devices, robot, slot.program_in)
        with context.step("Open gripper"):
            gripper.open_gripper(devices, context)
        context.set_variable("held_fiber", 0)
        run_program(devices, robot, slot.program_out)


def run_cycle(devices: Dict[str, Device], context: AppContext, number: int) -> AlignmentResult:
    """Pick fiber ``number``, align it, retract and put it back.

    A fiber that does not reach the threshold is still put back. The caller
    decides what a failed alignment means (prod.py counts it and moves on).
    """
    pick_fiber(devices, context, number)
    move_to_alignment_start(devices, context)
    result = align_fiber(devices, context)
    retract_from_alignment(devices, context)
    place_fiber(devices, context)
    return result


def run_align_only_cycle(devices: Dict[str, Device], context: AppContext) -> Optional[AlignmentResult]:
    """Move the held fiber to the alignment start, off by a random offset, align it, and retract.

    For running the alignment over and over from random starts: nothing is
    picked or placed. A search that finds no light is not a fault here: it is
    logged, the fiber still retracts, and the result is None (prod.py counts
    it as failed). Any other error stops the cycle as usual. Every alignment,
    whatever its end, adds a row to the ``align_only_csv`` file (runlog.py).
    """
    logger = devices[ROBOT].logger
    move_to_alignment_start(devices, context)
    offset = random_offset(devices, context)
    summary: Dict[str, Any] = {}
    result: Optional[AlignmentResult] = None
    try:
        result = align_fiber(devices, context, on_summary=summary.update)
    except AlignmentNoSignal as e:
        logger.warning(f"Alignment-only cycle: no light found ({e}). Retracting for the next cycle.")
    finally:
        runlog.record(context, logger, offset, summary)
    retract_from_alignment(devices, context)
    return result
