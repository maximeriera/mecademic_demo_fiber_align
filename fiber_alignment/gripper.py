"""SmarAct gripper: its jaw is a positioner on a SmarAct MCS2 controller.

Opening and closing are closed-loop moves of that channel to the positions set
by the ``gripper_open_mm`` / ``gripper_closed_mm`` params (Variables tab),
followed by a wait until the move has ended. The channel is the first one listed
under ``channels:`` for ``fiber_gripper`` in config.yaml.

* Teach ``gripper_closed_mm`` as the position where the jaws hold the fiber,
  not beyond it. A jaw blocked short of its target ends the move with
  MCS2MotionError (failed move, or still moving at GRIPPER_TIMEOUT_S), which
  faults the task.
* ABORT interrupts the wait (MCS2Aborted), like a robot WaitIdle().
* With ``type: "simulated"`` in config.yaml (no gripper connected), both
  moves are logged no-ops.

Every open and close is logged to ``logs/devices/fiber_gripper.log``: the
start position and target, then the end position and duration, or on failure
the error with the jaw's position and channel state flags.
"""

import time
from typing import Any, Dict

from core import AppContext
from devices import Device
from devices.api.smaract_mcs2 import MCS2Aborted

from .cell import GRIPPER

#: Longest an open or close move may take before it counts as failed (s).
GRIPPER_TIMEOUT_S = 10.0


def open_gripper(devices: Dict[str, Device], context: AppContext) -> None:
    _move_jaw(devices, "Open", context.get_param("gripper_open_mm", 0.0))


def close_gripper(devices: Dict[str, Device], context: AppContext) -> None:
    _move_jaw(devices, "Close", context.get_param("gripper_closed_mm", 1.0))


def _move_jaw(devices: Dict[str, Device], action: str, position_mm: float) -> None:
    gripper = devices[GRIPPER]
    api, log = gripper.api, gripper.logger
    # A simulated stand-in has no channels; its api ignores the number anyway.
    channel = (getattr(gripper, "channels", None) or [0])[0]
    tag = f"[{gripper.device_id}] {action} (channel {channel})"

    log.info(f"{tag}: from {_position(api, channel)} to {position_mm:.6f} mm")
    t0 = time.monotonic()
    try:
        api.move_to_mm(channel, position_mm)
        api.wait(channel, timeout=GRIPPER_TIMEOUT_S)
    except MCS2Aborted:
        log.warning(f"{tag}: interrupted by ABORT after {time.monotonic() - t0:.2f} s, "
                    f"at {_position(api, channel)}")
        raise
    except Exception as exc:
        log.error(f"{tag}: failed after {time.monotonic() - t0:.2f} s: {type(exc).__name__}: {exc} | "
                  f"at {_position(api, channel)}, state: {_state(api, channel)}")
        raise
    log.info(f"{tag}: done in {time.monotonic() - t0:.2f} s, at {_position(api, channel)}")


# The two helpers below only describe the jaw for a log line. They never raise,
# so a diagnostic cannot hide the error being logged (the link may be down).

def _position(api: Any, channel: int) -> str:
    try:
        mm = api.position_mm(channel)
    except Exception as exc:
        return f"? mm ({type(exc).__name__})"
    # A simulated stand-in returns None.
    return f"{mm:.6f} mm" if isinstance(mm, (int, float)) else "? mm"


def _state(api: Any, channel: int) -> str:
    try:
        flags = api.state_flags(channel)
    except Exception as exc:
        return f"? ({type(exc).__name__})"
    return (", ".join(flags) or "no flags") if isinstance(flags, list) else "?"
