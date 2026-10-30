"""Manual actions: one button per process step, on the Manual tab.

Each button runs the exact function production uses (fiber_alignment/process.py),
so the process can be tested one step at a time, in order:

    Pick Fiber → Move to Alignment Start → Run Alignment → Retract → Place Fiber

Random Offset, between Move to Alignment Start and Run Alignment, gives the
alignment a random start, as the alignment-only production does.

Pick Fiber and Full Cycle use the fiber set by the ``manual_fiber`` param
(Variables tab). Place Fiber always returns the held fiber to its own slot. A
step asked for in the wrong state (e.g. Place Fiber with nothing held) faults
the cell, before anything moves, with a message saying what to do.

Any callable registered in :func:`get_manual_actions` becomes a button, and
must also be listed under ``manual_actions:`` in config.yaml (same key). A
manual action runs as an exclusive task, like HOME or PROD, so the controller
must be READY to start one.
"""

from __future__ import annotations

from typing import Dict

from core import AppContext
from devices import Device

try:
    from .fiber_alignment import gripper, process
except ImportError as e:  # pragma: no cover - depends on how the workspace is loaded
    if e.name is not None:
        raise  # a module is really missing (e.g. a dependency): report that, not the fallback
    from fiber_alignment import gripper, process


def pick_fiber(devices: Dict[str, Device], context: AppContext):
    """Take the ``manual_fiber`` fiber from its slot."""
    process.pick_fiber(devices, context, context.get_param("manual_fiber", 1))


def move_to_alignment(devices: Dict[str, Device], context: AppContext):
    process.move_to_alignment_start(devices, context)


def random_offset(devices: Dict[str, Device], context: AppContext):
    process.random_offset(devices, context)


def align_fiber(devices: Dict[str, Device], context: AppContext):
    process.align_fiber(devices, context)


def retract_fiber(devices: Dict[str, Device], context: AppContext):
    process.retract_from_alignment(devices, context)


def place_fiber(devices: Dict[str, Device], context: AppContext):
    process.place_fiber(devices, context)


def full_cycle(devices: Dict[str, Device], context: AppContext):
    """Pick, align, retract and place the ``manual_fiber`` fiber once, outside the PROD loop."""
    process.run_cycle(devices, context, context.get_param("manual_fiber", 1))


def open_gripper(devices: Dict[str, Device], context: AppContext):
    """Open the gripper wherever the robot is. A held fiber is released on the spot."""
    with context.step("Open gripper"):
        gripper.open_gripper(devices, context)
    context.set_variable("held_fiber", 0)


def close_gripper(devices: Dict[str, Device], context: AppContext):
    """Close the gripper.

    ``held_fiber`` is left unchanged: the cell cannot know what, if anything,
    is now held. Set it in the Variables tab if needed.
    """
    with context.step("Close gripper"):
        gripper.close_gripper(devices, context)


def get_manual_actions():
    """Return the manual actions shown on the Manual tab.

    Keys must match the ``key:`` entries under ``manual_actions:`` in
    config.yaml. The controller refuses to start if one is missing.
    """
    return {
        "pick_fiber": pick_fiber,
        "move_to_alignment": move_to_alignment,
        "random_offset": random_offset,
        "align_fiber": align_fiber,
        "retract_fiber": retract_fiber,
        "place_fiber": place_fiber,
        "full_cycle": full_cycle,
        "open_gripper": open_gripper,
        "close_gripper": close_gripper,
    }
