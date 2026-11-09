"""The steps of the fiber alignment process.

Pick, Move to alignment start, Align, Hold, Retract and Place make the PROD
cycle; with ``prod_align_only``, PROD runs Move to alignment start, Random
offset, Align, Hold and Retract instead (``run_align_only_cycle``). Shared by the PROD cycle
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
import time
from datetime import datetime
from typing import Any, Callable, Dict, Optional, Tuple

from core import AppContext
from devices import Device

from . import algorithm, gripper, memory, poses, runlog
from .alignment import AlignmentNoSignal, AlignmentResult, AlignmentSettings, run_alignment
from .cell import ROBOT, ProcessError, power_meter, prepare_robot, run_program

#: The steps of a PROD cycle, in order (the same as ``sequences: prod_cycle`` in
#: config.yaml), and those of the alignment-only cycle. prod.py declares the
#: one it runs, for "step 2 of 4" on the Production tab.
CYCLE_STEPS = ["Pick fiber", "Move to alignment start", "Align fiber", "Hold final position", "Retract from alignment",
               "Place fiber"]
ALIGN_ONLY_STEPS = ["Move to alignment start", "Random offset", "Align fiber", "Hold final position",
                    "Retract from alignment"]
#: How often the power is read while the fiber holds its final position (s).
HOLD_READ_S = 0.5


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
        # The walk the last run measured: the Angle phase reads each tilt at rest with it.
        walk = memory.remembered_walk(context, logger)
        result = run_alignment(robot, power_meter(devices), context, fiber=fiber, align=algorithm.align,
                               logger=logger, phases=algorithm.PHASES, phase_views=algorithm.PHASE_VIEWS,
                               seed=seed, on_summary=on_summary, walk=walk)
        memory.remember_walk(context, result.walk, logger)

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


def hold_final_position(devices: Dict[str, Device], context: AppContext) -> Optional[Tuple[float, float, float]]:
    """Keep the fiber where the alignment left it for ``hold_final_s``, reading the power meanwhile.

    The robot does not move. An ABORT or a device fault ends the hold at once
    (the controller marks it on the events channel before stopping the task;
    there is no robot command here for it to interrupt). Logs the power at the
    start and at the end of the hold, its lowest, and the drift in dB: how
    steady the alignment stays. Returns (first, last, lowest) in W, or None
    when there is no hold (``hold_final_s`` 0) or no reading.
    """
    with context.step("Hold final position"):
        hold_s = max(0.0, float(context.get_param("hold_final_s", 5.0)))
        if hold_s <= 0:
            return None
        if not context.get_variable("at_alignment", False):
            raise ProcessError("The robot is not at the alignment station: no final position to hold.")
        meter = power_meter(devices)
        since = context.data.now()
        end = time.monotonic() + hold_s
        readings = []
        while True:
            stop = [e for _, e in context.data.window("events", since) if (e or {}).get("name") in ("abort", "fault")]
            if stop:
                raise ProcessError(f"Hold of the final position interrupted ({stop[-1]['name']}).")
            watts = meter.read_power()
            # Overrange reads +inf and an invalid reading NaN: not a power to report.
            if isinstance(watts, float) and math.isfinite(watts):
                readings.append(watts)
            left = end - time.monotonic()
            if left <= 0:
                break
            time.sleep(min(HOLD_READ_S, left))
        logger = devices[ROBOT].logger
        if not readings:
            logger.warning(f"Held the final position {hold_s:g} s: no valid power reading.")
            return None
        first, last, lowest = readings[0], readings[-1], min(readings)
        drift = f"{10 * math.log10(last / first):+.3f} dB" if first > 0 and last > 0 else "n/a"
        logger.info(f"Held the final position {hold_s:g} s: {first * 1e3:.4g} → {last * 1e3:.4g} mW "
                    f"(lowest {lowest * 1e3:.4g} mW, drift {drift}, {len(readings)} readings).")
        return first, last, lowest


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
    """Pick fiber ``number``, align it, hold it at its final position, retract and put it back.

    A fiber that does not reach the threshold is still put back. The caller
    decides what a failed alignment means (prod.py counts it and moves on).
    """
    pick_fiber(devices, context, number)
    move_to_alignment_start(devices, context)
    result = align_fiber(devices, context)
    hold_final_position(devices, context)
    retract_from_alignment(devices, context)
    place_fiber(devices, context)
    return result


def recover(devices: Dict[str, Device], context: AppContext, error: Exception, place: bool, since: float) -> None:
    """After an error in a PROD cycle at the alignment station: retract, put the fiber back (``place``), go on.

    For errors the cell can step back from: a software error, a lost reply, a
    search that went wrong. Re-raises ``error`` (the cell faults, as before)
    when it should not move:
    * after an ABORT or a device fault since ``since`` (the hub time when the
      cycle started, ``context.data.now()``): the controller marks either
      before stopping the task. Any one since then counts, not only the
      latest event: a failing alignment marks align_end after it;
    * when the robot is not at the alignment station: an error at the rack,
      where the robot may be stopped inside a slot. The operator must look.
    If the recovery itself fails, the cell faults with both errors. Records
    the error in ``error_count`` and ``last_error``.
    """
    stopped = any((event or {}).get("name") in ("abort", "fault") for _, event in context.data.window("events", since))
    if stopped or not context.get_variable("at_alignment", False):
        raise error
    logger = devices[ROBOT].logger
    message = f"{type(error).__name__}: {error}"
    logger.error(f"PROD cycle failed at the alignment station ({message}). Retracting"
                 f"{' and putting the fiber back' if place else ''}, then on with the next cycle.")
    try:
        # Whatever the error left queued: stop it, and accept the retract.
        robot = devices[ROBOT].api
        robot.ClearMotion()
        robot.ResumeMotion()
        retract_from_alignment(devices, context)
        if place and context.get_variable("held_fiber", 0):
            place_fiber(devices, context)
    except Exception as recovery_error:
        raise ProcessError(f"{message}; then the recovery failed: {type(recovery_error).__name__}: "
                           f"{recovery_error}. Check where the robot and the fiber are.") from recovery_error
    context.set_variable("error_count", context.get_variable("error_count", 0) + 1)
    context.set_variable("last_error", f"{datetime.now():%Y-%m-%d %H:%M:%S} {message}"[:300])


def run_align_only_cycle(devices: Dict[str, Device], context: AppContext) -> Optional[AlignmentResult]:
    """Move the held fiber to the alignment start, off by a random offset, align it, hold it there, and retract.

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
    if result is not None:
        hold_final_position(devices, context)
    retract_from_alignment(devices, context)
    return result
