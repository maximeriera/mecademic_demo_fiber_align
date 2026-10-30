"""The cell's devices, as the process code reaches them.

The names below must match the keys under ``devices:`` in config.yaml. If you
rename a device there, rename it here too.
"""

from __future__ import annotations

from typing import Any, Dict

from core import AppContext
from devices import Device

from . import poses

ROBOT = "meca_robot"
GRIPPER = "fiber_gripper"
POWER_METER = "power_meter"

#: Cartesian acceleration (%) and angular velocity (°/s) restored before every
#: step: the alignment algorithm changes both. Mecademic's defaults.
CART_ACC_PCT = 50.0
CART_ANG_VEL_DEG_S = 45.0


class ProcessError(RuntimeError):
    """A process step was asked for in a state where it is unsafe or meaningless.

    Raised before anything moves, with a message saying how to recover. Like any
    exception in a task it faults the controller: read the message in the
    ApplicationController log, fix the state (Manual tab, or the variables in the
    Variables tab), then Clear Faults.
    """


def prepare_robot(devices: Dict[str, Device], context: AppContext) -> Any:
    """Return the robot's mecademicpy API with the tool frame, speeds and blending applied.

    Every process function starts with this. Speeds edited in the Variables tab
    therefore apply from the next step on, and the steps can run in any order
    from the Manual tab. It returns once the robot has applied them and run the
    ``init`` program: the alignment reads its start pose right after.
    """
    if not poses.POSES_TAUGHT:
        raise ProcessError(
            "fiber_alignment/poses.py still holds placeholder poses: teach them on the robot, "
            "then set POSES_TAUGHT = True."
        )
    robot = devices[ROBOT].api
    robot.SetTrf(*poses.TOOL_TRF)
    robot.SetJointVel(context.get_param("joint_vel_pct", 20.0))
    robot.SetCartLinVel(context.get_param("lin_vel_mm_s", 10.0))
    robot.SetCartAcc(CART_ACC_PCT)
    robot.SetCartAngVel(CART_ANG_VEL_DEG_S)
    # No corner cutting: every move must reach its pose exactly, e.g. a
    # straight descent onto a slot.
    robot.SetBlending(0)
    robot.StartProgram(poses.INIT_PROGRAM)
    # These are queued: wait until they apply. ABORT interrupts the wait.
    robot.WaitIdle()
    return robot


def run_program(devices: Dict[str, Device], robot: Any, name: str) -> None:
    """Run a program stored on the robot and wait until it has finished.

    ``robot`` is the API returned by :func:`prepare_robot`. ABORT interrupts
    the wait, like any WaitIdle().
    """
    devices[ROBOT].logger.info(f"[{ROBOT}] Running robot program '{name}'")
    robot.StartProgram(name)
    robot.WaitIdle()


def power_meter(devices: Dict[str, Device]) -> Any:
    """Return the power meter's driver (``devices.api.thorlabs_pm.PowerMeter``)."""
    return devices[POWER_METER].api
