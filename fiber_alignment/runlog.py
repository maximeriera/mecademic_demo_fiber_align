"""One CSV row per alignment-only cycle (``prod_align_only``): its random start, each phase, the outcome.

For long unattended runs. The file is the ``align_only_csv`` param: a path
under the app's ``logs/`` folder (mounted out of the container, not in git), or
an absolute one; empty turns the log off. Each row is appended and flushed at
once, so a night's data survives a crash. A file whose header no longer
matches the columns (after an update) is renamed aside, with the time, and a
new one started.

Columns:

* ``time``, ``run`` (the row number in this file), ``fiber``,
  ``use_remembered_pose``, ``threshold_mw``.
* ``offset_*``: the random offset commanded from ``align_start`` (tool frame:
  x, z in µm, rx, rz in °).
* ``start_power_w``: the power where the search started (median of its first
  readings); ``acquire_found``; ``acquire_how``: the search that found the
  light and where it was centred ("warm @ start", "angle @ ring 3/6", ...);
  ``acquire_trail``: every search run, "+" on the one that found it.
* Per phase (``acquire``, ``center``, ``angle``, ``peak``): ``_status`` (done,
  failed, timed out, not run, ...), ``_s`` (duration) and ``_best_mw`` (the
  best reading so far when the phase ended); ``peak_circles`` and
  ``peak_iterations``: the gradient circles and the pattern iterations.
* ``passed``, ``power_mw`` (the verdict's reading), ``duration_s``, ``reads``,
  ``timed_out``.
* ``final_*``: where the fiber ended, as offsets from where the alignment
  started (the offset pose: x, z µm, rx, rz °), and as the robot's pose
  (``final_pose_*``: mm and °, WRF), e.g. to follow a drift over the night.
* ``error``: why the run stopped, if it raised (e.g. no light found).
"""

from __future__ import annotations

import csv
import logging
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from core import AppContext
from core.LogSetup import log_root_dir

from .algorithm import PHASES

#: The CSV file when the param is not set, under logs/.
DEFAULT_PATH = "alignment/align_only_runs.csv"

COLUMNS: List[str] = (
    ["time", "run", "fiber", "use_remembered_pose", "threshold_mw",
     "offset_x_um", "offset_z_um", "offset_rx_deg", "offset_rz_deg",
     "start_power_w", "acquire_found", "acquire_how", "acquire_trail"]
    + [f"{p.lower()}_{k}" for p in PHASES for k in ("status", "s", "best_mw")]
    + ["peak_circles", "peak_iterations",
       "passed", "power_mw", "duration_s", "reads", "timed_out",
       "final_x_um", "final_z_um", "final_rx_deg", "final_rz_deg",
       "final_pose_x_mm", "final_pose_y_mm", "final_pose_z_mm",
       "final_pose_alpha_deg", "final_pose_beta_deg", "final_pose_gamma_deg",
       "error"]
)


def record(context: AppContext, logger: logging.Logger, offset: Optional[Sequence[float]],
           summary: Dict[str, Any]) -> None:
    """Append the row of one alignment-only cycle. Never raises: a failed write is a warning."""
    try:
        path = csv_path(context)
        if path is None:
            return
        row = make_row(context, offset, summary)
        run = _append(path, row)
        logger.info(f"Alignment-only run {run} logged to {path}")
    except Exception as e:  # the log must never stop production, nor hide the alignment's own error
        logger.warning(f"Could not log the alignment-only run to CSV: {type(e).__name__}: {e}")


def csv_path(context: AppContext) -> Optional[Path]:
    """The CSV file (``align_only_csv``, relative to logs/), or None when the param is empty."""
    name = str(context.get_param("align_only_csv", DEFAULT_PATH)).strip()
    if not name:
        return None
    path = Path(name)
    return path if path.is_absolute() else Path(log_root_dir()) / "logs" / path


def make_row(context: AppContext, offset: Optional[Sequence[float]], summary: Dict[str, Any]) -> Dict[str, Any]:
    """The CSV row (by column) of one cycle: its offset (x, z µm, rx, rz °) and its alignment summary."""
    published = summary.get("published") or {}
    acquire = published.get("acquire") or []
    found = [a for a in acquire if a.get("found")]
    row: Dict[str, Any] = {
        "time": datetime.now().astimezone().isoformat(timespec="seconds"),
        "fiber": summary.get("fiber", context.get_variable("held_fiber", 0)),
        "use_remembered_pose": bool(context.get_param("use_remembered_pose", True)),
        "threshold_mw": _num(_mw(summary.get("threshold_w")), 4),
        "start_power_w": _num(acquire[0].get("noise_w") if acquire else None, 4, sig=True),
        "acquire_found": bool(found),
        "acquire_how": _how(found[0]) if found else "",
        "acquire_trail": " ".join(_how(a) + ("+" if a.get("found") else "") for a in acquire),
        "peak_circles": len(published.get("gradient") or []),
        "peak_iterations": len(published.get("peak") or []),
        "passed": bool(summary.get("passed")),
        "power_mw": _num(_mw(summary.get("power_w")), 5, sig=True),
        "duration_s": _num(summary.get("duration_s"), 2),
        "reads": summary.get("reads", ""),
        "timed_out": bool(summary.get("timed_out")),
        "error": summary.get("error", "") if summary else "no alignment summary: the step failed before the alignment ran",
    }
    for key, value in zip(("offset_x_um", "offset_z_um", "offset_rx_deg", "offset_rz_deg"), offset or ()):
        row[key] = _num(value, 2 if key.endswith("_um") else 4)
    phases = {p.get("name"): p for p in summary.get("phases") or []}
    for name in PHASES:
        phase = phases.get(name, {})
        key = name.lower()
        row[f"{key}_status"] = phase.get("status", "")
        if "t0" in phase and "t1" in phase:
            row[f"{key}_s"] = _num(phase["t1"] - phase["t0"], 2)
        row[f"{key}_best_mw"] = _num(_mw(phase.get("best_w")), 5, sig=True)
    local = summary.get("final_local")
    if local:
        x, _, z, rx, _, rz = local
        row.update(final_x_um=_num(x, 2), final_z_um=_num(z, 2), final_rx_deg=_num(rx, 4), final_rz_deg=_num(rz, 4))
    pose = summary.get("final")
    if pose:
        for key, value in zip(("x_mm", "y_mm", "z_mm", "alpha_deg", "beta_deg", "gamma_deg"), pose):
            row[f"final_pose_{key}"] = _num(value, 4)
    return row


def _append(path: Path, row: Dict[str, Any]) -> int:
    """Append ``row`` to ``path`` (header first, in a new file); returns its run number."""
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = 0
    if path.exists():
        with path.open(newline="", encoding="utf-8") as f:
            lines = list(csv.reader(f))
        if lines and lines[0] != COLUMNS:
            # Other columns (an older version): keep that file, start a new one.
            path.rename(path.with_name(f"{path.stem}.{datetime.now():%Y%m%d-%H%M%S}{path.suffix}"))
        else:
            rows = max(0, len(lines) - 1)
    new = not path.exists()
    row["run"] = rows + 1
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        if new:
            writer.writeheader()
        writer.writerow(row)
        f.flush()
        os.fsync(f.fileno())
    return row["run"]


def _how(payload: Dict[str, Any]) -> str:
    """'angle @ ring 3/6', 'warm @ start': the search of an "acquire" payload, and where it was centred."""
    return f"{payload.get('search') or 'warm'} @ {payload.get('around') or 'start'}"


def _mw(watts: Any) -> Optional[float]:
    return watts * 1e3 if isinstance(watts, (int, float)) else None


def _num(value: Any, digits: int, sig: bool = False) -> Any:
    """``value`` rounded (to ``digits`` significant digits with ``sig``), or "" if not a finite number."""
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        return ""
    return float(f"{value:.{digits}g}") if sig else round(float(value), digits)
