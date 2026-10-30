"""Where each fiber aligned last, so that its next alignment starts there.

Each fiber of the rack sits a little differently in the gripper, and points a
little differently: from ``align_start``, a fiber just picked is often off in
tilt and sideways at once, the slowest case for the search for first light.
Where the same fiber aligned last time is a much better start, as long as it
goes back into the same slot.

The poses are kept in the ``fiber_poses`` variable (context.yaml, persisted),
as JSON::

    {"<fiber>": {"pose": [x, y, z, alpha, beta, gamma], "power_mw": ..., "trf": [...]}}

A pose is the robot's (mm in the WRF, then degrees, with ``TOOL_TRF``), so it
stays valid if ``align_start`` is taught again. One remembered with another
``TOOL_TRF`` is not used. Reset the variable in the Variables tab (or set it to
``{}``) to forget them all.
"""

from __future__ import annotations

import json
import logging
import math
from typing import Any, Dict, Optional, Sequence

from core import AppContext

from . import poses
from .frames import Pose6

#: The context variable (context.yaml) holding the poses.
VARIABLE = "fiber_poses"
#: Decimals kept: 0.1 µm and 0.0001°, under the robot's resolution.
DECIMALS = 4
#: How far a remembered TOOL_TRF may be from the current one and still count as the same (mm or °).
TRF_TOLERANCE = 1e-3


def remembered_pose(context: AppContext, fiber: int, logger: logging.Logger) -> Optional[Pose6]:
    """The pose where ``fiber`` aligned last, or None (none yet, or remembered with another TOOL_TRF)."""
    entry = _load(context, logger).get(str(fiber))
    if not isinstance(entry, dict):
        return None
    trf = _six(entry.get("trf"))
    if trf is None or any(abs(a - b) > TRF_TOLERANCE for a, b in zip(trf, poses.TOOL_TRF)):
        logger.info(f"Fiber {fiber}: its remembered pose was taken with another TOOL_TRF, not used.")
        return None
    return _six(entry.get("pose"))


def remember(context: AppContext, fiber: int, pose: Sequence[float], power_w: float,
             logger: logging.Logger) -> None:
    """Keep ``pose`` as where ``fiber`` aligned, read ``power_w`` there."""
    memory = _load(context, logger)
    memory[str(fiber)] = {
        "pose": [round(float(v), DECIMALS) for v in pose],
        "power_mw": round(power_w * 1e3, DECIMALS),
        "trf": [round(float(v), DECIMALS) for v in poses.TOOL_TRF],
    }
    context.set_variable(VARIABLE, json.dumps(memory, sort_keys=True))


def _load(context: AppContext, logger: logging.Logger) -> Dict[str, Any]:
    """The remembered poses by fiber; {} if there are none or the variable is not valid JSON."""
    text = context.get_variable(VARIABLE, "{}")
    if not isinstance(text, str) or not text.strip():
        return {}
    try:
        memory = json.loads(text)
    except ValueError:
        logger.warning(f"{VARIABLE} is not valid JSON: no fiber's pose is remembered "
                       f"(the next alignment replaces it).")
        return {}
    return memory if isinstance(memory, dict) else {}


def _six(values: Any) -> Optional[Pose6]:
    """``values`` as six finite floats, or None."""
    try:
        pose = tuple(float(v) for v in values)
    except (TypeError, ValueError):
        return None
    return pose if len(pose) == 6 and all(math.isfinite(v) for v in pose) else None
