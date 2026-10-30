"""SHIPMENT task: fold the robot into its transport/storage pose.

Like HOME, it first backs out of the alignment station if needed, and it
never opens the gripper.
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


def shipment(devices: Dict[str, Device], context: AppContext):
    """Back out of the alignment station if needed, then move to SHIPMENT_JOINTS."""
    if context.get_variable("at_alignment", False):
        process.retract_from_alignment(devices, context)

    with context.step("Move to shipment position"):
        robot = cell.prepare_robot(devices, context)
        robot.MoveJoints(*poses.SHIPMENT_JOINTS)
        robot.WaitIdle()
