"""HOME task: bring the robot to its home position.

Runs on demand from the Control tab, and automatically before and after every
production run. If the robot was left at the alignment station (e.g. an
aborted alignment), it first backs out along the fiber axis.

The gripper is not touched, so a held fiber stays held. Put it back with
Manual: Place Fiber.
"""

from typing import Dict

from core import AppContext
from devices import Device

try:
    from .fiber_alignment import cell, poses, process
except ImportError as e:  # pragma: no cover - depends on how the workspace is loaded
    if e.name is not None:
        raise  # a module is really missing (e.g. a dependency): report that, not the fallback
    from fiber_alignment import cell, poses, process


def home(devices: Dict[str, Device], context: AppContext):
    """Back out of the alignment station if needed, then move to HOME_JOINTS."""
    if context.get_variable("at_alignment", False):
        process.retract_from_alignment(devices, context)

    with context.step("Move to home position"):
        robot = cell.prepare_robot(devices, context)
        robot.MoveJoints(*poses.HOME_JOINTS)
        robot.WaitIdle()
