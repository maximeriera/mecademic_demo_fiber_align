"""CALIBRATION task: nothing to calibrate in this demo yet.

Candidates, if the demo needs one:

* zero the power meter (``cell.power_meter(devices).zero()``) with no fiber in
  front of the fixed one, so that only the dark offset is nulled;
* re-teach the ``align_start`` robot program from the pose of a successful alignment.
"""

from typing import Dict

from core import AppContext
from devices import Device

try:
    from .fiber_alignment import cell
except ImportError as e:  # pragma: no cover - depends on how the workspace is loaded
    if e.name is not None:
        raise  # a module is really missing (e.g. a dependency): report that, not the fallback
    from fiber_alignment import cell


def calib(devices: Dict[str, Device], context: AppContext):
    """No-op for now: logs that there is nothing to calibrate."""
    devices[cell.ROBOT].logger.info("CALIBRATION: nothing to calibrate in the fiber alignment demo.")
