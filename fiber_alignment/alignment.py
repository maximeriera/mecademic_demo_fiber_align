"""The contract between the cell and the alignment algorithm.

The algorithm (algorithm.py) never touches a device. It receives an
:class:`AlignmentIO`, which moves the held fiber anywhere in 3D (position and
orientation) and reads the power meter, plus the :class:`AlignmentSettings`. It
must leave the fiber at the best position it found. Everything else is handled
here:

* Every reading is recorded with the robot's pose at the middle of the meter's
  reading (interpolated during a scan, see :meth:`AlignmentIO.scan`), and
  published to the custom view (``align_sample``) as offsets from the start in
  the start's tool frame, the frame the algorithm moves in. The graph
  therefore shows exactly what the algorithm measured, and moves only while an
  alignment runs.
* The algorithm names its phases with ``io.phase()``: they show in the top bar,
  are timed on the Production tab and are plotted by the custom view, each as
  declared in the algorithm's ``PHASE_VIEWS``. Extra data for the view (scan
  profiles, peak steps) goes through ``io.publish()``.
* Every target pose is checked **before anything is sent**: each coordinate
  must stay within ``align_range_um`` / ``align_range_deg`` of the start pose,
  and be a finite number.
* A pose remembered for the fiber (``seed``, where it aligned last) reaches
  the algorithm as ``io.seed``, offsets from the start. The window stays
  centred on the start: a remembered pose never moves it.
* ABORT, a device fault and ``align_timeout_s`` are checked before every
  command and every reading. A scan interrupted for any reason clears the
  robot's motion queue, so queued moves never run on after it.
* The verdict is a fresh reading at the final position, compared with
  ``power_threshold_mw``. The algorithm's own claim is not used. If that
  reading falls clearly below the best reading taken at rest, the fiber goes
  back there first.

A pose is ``(x, y, z, alpha, beta, gamma)``: mm in the robot's WRF, then Euler
angles in degrees (XYZ mobile), exactly as the robot reports it. Power is in W.
"""

from __future__ import annotations

import logging
import math
import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterator, List, NamedTuple, Optional, Sequence

import numpy as np

from core import AppContext

from . import frames
from .frames import Pose6

#: Version of the ``align_sample`` rows (see :meth:`AlignmentIO.row`).
SAMPLE_FORMAT = 2
#: Most points kept in the align_result summary for the custom view, besides
#: the readings taken at rest, which are all kept.
RESULT_POINTS_MAX = 5000
#: Most entries kept per io.publish() name in the align_result summary.
RESULT_PUBLISHED_MAX = 500
#: Pause between two readings during a scan (s). The meter reading and the
#: pose query take a few ms more.
SCAN_SAMPLE_PERIOD_S = 0.002
#: Meter reading plus pose query (s), until measured: about 3 ms per averaged
#: sample on a PM100x, plus USB and the robot's reply.
READ_S_GUESS = 0.008
#: Checkpoint ids used by scans (the robot allows 1-8000).
CHECKPOINT_IDS = range(7000, 8000)
#: Motion for the return to the best pose: slow, no blending.
RETURN_MOTION = {"acc": 25, "lin_vel": 1, "ang_vel": 5, "blending": 0}
#: A final reading this far (dB) under the best reading taken at rest sends the
#: fiber back there before the verdict.
FINAL_TOLERANCE_DB = 0.2
#: A scan whose robot pose has not changed for this long (s) has stalled: the
#: robot is not executing it (paused, refused the moves, ...).
SCAN_STALL_S = 3.0
#: Pose change (mm or °) that counts as motion for the stall check.
SCAN_MOTION_EPS = 1e-3
#: Longest a point-to-point alignment move may take (s).
MOVE_TIMEOUT_S = 30.0


class AlignmentError(RuntimeError):
    """The alignment cannot run, or the algorithm broke the contract."""


class AlignmentRangeError(AlignmentError):
    """The algorithm asked for a pose outside the search window (or not a number)."""


class AlignmentInterrupted(AlignmentError):
    """ABORT was pressed, or a device faulted, while the alignment was running."""


class AlignmentNoSignal(AlignmentError):
    """No light anywhere in the search: the laser, the fibers or the start pose need checking."""


class AlignmentTimeout(Exception):
    """Raised by every AlignmentIO call once ``timeout_s`` has elapsed.

    Not an error: run_alignment() catches it, moves back to the best pose read
    so far and judges that. An algorithm may catch it to clean up, but it must
    not move again.
    """


@dataclass(frozen=True)
class Sample:
    """One power reading and the robot's actual pose at the middle of it."""

    pose: Pose6
    power_w: float
    #: "cart" (lateral moves), "ang" (rotations) or None.
    kind: Optional[str] = None
    #: Index of the phase it was read in (see AlignmentIO.phase), or -1.
    phase: int = -1
    #: Seconds since the alignment started.
    t: float = 0.0
    #: Read with the fiber at rest (see AlignmentIO.read_still).
    still: bool = False


class _Stamped(NamedTuple):
    """A pose and when it was read (time.monotonic(), s)."""

    t: float
    pose: Pose6


@dataclass(frozen=True)
class AlignmentSettings:
    #: The alignment passes once the final reading reaches this (W).
    threshold_w: float
    #: How far each coordinate may go from the start pose (mm, degrees).
    range_mm: float
    range_deg: float
    timeout_s: float
    #: Pause after a point-to-point move, before anything else (s).
    settle_s: float
    #: Power of the source (W): what a perfect coupling would read.
    source_power_w: float
    #: Starting guesses of how wide the coupling is, re-measured as the
    #: alignment goes: the 1/e² radius of the power against a lateral offset
    #: (mm), and its 1/e half-width against a tilt (degrees).
    lateral_width_mm: float
    angle_width_deg: float
    #: How far the searches for first light go from the start: tilt (degrees), lateral (mm).
    acquire_radius_deg: float
    acquire_radius_mm: float
    #: Still no light, the tilt search runs again at points this far around the start (mm); 0: not.
    acquire_ring_mm: float
    #: Lowest power counted as light (W). The algorithm raises it above the noise it measures.
    signal_detect_w: float
    #: Run both searches for first light to their full radius, even with light
    #: at the start: shows the whole search (demo).
    full_search: bool
    #: Readings per side of the Peak phase's square pattern (odd).
    peak_grid: int
    #: Rounds of gradient circles (a lateral one, then a tilt one) that start the Peak phase; 0: none.
    gradient_rounds: int

    @classmethod
    def from_context(cls, context: AppContext) -> "AlignmentSettings":
        """Read the params from context.yaml, converted to W, mm and degrees."""
        def mw(name: str, default: float) -> float:
            return float(context.get_param(name, default)) * 1e-3

        return cls(
            threshold_w=mw("power_threshold_mw", 0.9),
            range_mm=float(context.get_param("align_range_um", 2000.0)) * 1e-3,
            range_deg=float(context.get_param("align_range_deg", 8.0)),
            timeout_s=float(context.get_param("align_timeout_s", 300.0)),
            settle_s=float(context.get_param("align_settle_s", 0.05)),
            source_power_w=mw("source_power_mw", 1.0),
            lateral_width_mm=float(context.get_param("lateral_width_um", 200.0)) * 1e-3,
            angle_width_deg=float(context.get_param("angle_width_deg", 0.3)),
            acquire_radius_deg=float(context.get_param("acquire_radius_deg", 2.0)),
            acquire_radius_mm=float(context.get_param("acquire_radius_um", 500.0)) * 1e-3,
            # Off unless context.yaml sets it (750 there).
            acquire_ring_mm=float(context.get_param("acquire_ring_um", 0.0)) * 1e-3,
            signal_detect_w=float(context.get_param("signal_detect_uw", 1.0)) * 1e-6,
            full_search=bool(context.get_param("acquire_full_search", False)),
            peak_grid=int(context.get_param("peak_grid", 5)),
            gradient_rounds=int(context.get_param("gradient_rounds", 8)),
        )


@dataclass(frozen=True)
class AlignmentResult:
    fiber: int
    #: The final reading reached the threshold.
    passed: bool
    #: That final reading (W).
    power_w: float
    #: Final pose.
    pose: Pose6
    #: Power readings taken, the final one included.
    reads: int
    duration_s: float
    timed_out: bool


class AlignmentIO:
    """What the algorithm may do: move the fiber, scan, and read the power."""

    def __init__(self, robot: Any, meter: Any, context: AppContext, settings: AlignmentSettings,
                 logger: Optional[logging.Logger] = None, phases: Sequence[str] = ()):
        self._robot = robot
        self._meter = meter
        self._context = context
        self._settings = settings
        self._log = logger or logging.getLogger(__name__)
        #: The algorithm's declared phases, in order (the view's phase list).
        self.phases: List[str] = list(phases)
        #: Where the search started: the centre of the window, and the origin
        #: of the alignment frame (see :meth:`local`).
        self.start: Pose6 = self._target_pose()
        self._start_matrix = frames.pose_matrix(self.start)
        self._start_inverse = np.linalg.inv(self._start_matrix)
        #: Where this fiber aligned last, as :meth:`local` offsets from the
        #: start, or None. Set by run_alignment; never checked against the window.
        self.seed: Optional[Pose6] = None
        self._t0 = time.monotonic()
        self._deadline = self._t0 + settings.timeout_s
        #: Every reading so far.
        self.samples: List[Sample] = []
        #: The highest valid reading so far, or None before the first.
        self.best: Optional[Sample] = None
        #: The highest valid reading taken at rest (read_still), or None.
        self.best_still: Optional[Sample] = None
        #: One entry per phase run, for the summary.
        self.phase_log: List[Dict[str, Any]] = []
        #: What the algorithm published for the view, by name.
        self.published: Dict[str, List[Any]] = {}
        self._phase = -1
        self._checkpoints = iter(())
        # Power meter settings changed by the algorithm, with their original values.
        self._meter_originals: Dict[str, Any] = {}
        # Measured time of a reading plus its pose query (s), outside read_still.
        self._read_s: Optional[float] = None
        # The pose read after the latest reading: the "before" of a scan's next one.
        self._last_pose: Optional[_Stamped] = None

    # --- For the algorithm ---------------------------------------------------

    @contextmanager
    def phase(self, name: str) -> Iterator[None]:
        """Run one phase of the algorithm (``with io.phase("Acquire"): ...``).

        The phase is shown in the top bar under "Align fiber" and timed on the
        Production tab (a nested step), listed and plotted by the custom view,
        and summarised in align_result. Readings taken inside it are
        tagged with it. Name the phases as declared in ``PHASES`` (algorithm.py).
        """
        index = self.phases.index(name) if name in self.phases else -1
        entry: Dict[str, Any] = {
            "name": name,
            "index": index,
            "t0": round(self.elapsed_s(), 3),
            "best_before_w": self.best.power_w if self.best else None,
            "status": "running",
        }
        reads_before = len(self.samples)
        previous, self._phase = self._phase, index
        self.phase_log.append(entry)
        self._context.mark("align_phase_start", phase=name, index=index, t=entry["t0"])
        self._log.info(f"Alignment phase '{name}' started")
        status = "done"
        try:
            with self._context.step(name):
                yield
        except AlignmentTimeout:
            status = "timed out"
            raise
        except BaseException:
            status = "failed"
            raise
        finally:
            self._phase = previous
            entry.update(t1=round(self.elapsed_s(), 3), status=status, reads=len(self.samples) - reads_before,
                         best_w=self.best.power_w if self.best else None)
            self._context.mark("align_phase_end", phase=name, index=index, t=entry["t1"], status=status,
                               reads=entry["reads"], best_w=entry["best_w"], best_before_w=entry["best_before_w"])
            self._log.info(f"Alignment phase '{name}' {status}: {entry['reads']} reading(s) in "
                           f"{entry['t1'] - entry['t0']:.2f} s")

    def publish(self, name: str, value: Any) -> None:
        """Share extra data with the custom view, as channel ``align_<name>``.

        Dict values get the current phase index and time added. The view uses
        "acquire", "profile", "gradient" and "peak" (see custom_view/__init__.py); anything
        else is kept in align_result for later use.
        """
        if isinstance(value, dict):
            value = {**value, "phase": self._phase, "t": round(self.elapsed_s(), 3)}
        self.published.setdefault(name, []).append(value)
        self._context.publish(f"align_{name}", value)

    def pose(self) -> Pose6:
        """The robot's actual pose now."""
        self._check()
        return self._actual_pose()

    def local(self, pose: Sequence[float]) -> Pose6:
        """``pose`` as offsets from the start, in the start's tool frame.

        ``(x, y, z)`` in µm, then ``(rx, ry, rz)``, rotations in degrees about
        the start's tool axes (extrinsic). The fibers face each other along y.
        """
        T = self._start_inverse @ frames.pose_matrix(pose)
        position, angles = frames.extract_pose_extrinsic(T)
        return tuple(float(v) for v in (*(position * 1000.0), *angles))

    def pose_at(self, x: float = 0.0, y: float = 0.0, z: float = 0.0,
                rx: float = 0.0, ry: float = 0.0, rz: float = 0.0) -> Pose6:
        """The robot pose at offsets from the start, in the start's tool frame (µm, degrees, as :meth:`local`)."""
        offset = frames.make_homogeneous_matrix_extrinsic([x * 1e-3, y * 1e-3, z * 1e-3], [rx, ry, rz])
        return frames.matrix_pose(self._start_matrix @ offset)

    def elapsed_s(self) -> float:
        return time.monotonic() - self._t0

    @property
    def sample_period_s(self) -> float:
        """Time between two readings of a scan (s), measured: reading, pose query and pause.

        Set scan speeds from it: speed = wanted spacing between readings / period.
        """
        return (self._read_s if self._read_s is not None else READ_S_GUESS) + SCAN_SAMPLE_PERIOD_S

    def move_to(self, pose: Sequence[float]) -> None:
        """Move in a straight line to ``pose`` and return once there and settled."""
        self._check()
        self._move(self._checked(pose))

    def move_rel_trf(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                     drx: float = 0.0, dry: float = 0.0, drz: float = 0.0) -> None:
        """Move relative to the tool frame, like ``MoveLinRelTrf``.

        The target is computed here and sent as an absolute ``MoveLin``, so the
        pose checked against the window is exactly the pose the robot receives.
        """
        self._check()
        current = frames.pose_matrix(self._target_pose())
        relative = frames.make_homogeneous_matrix([dx, dy, dz], [drx, dry, drz])
        self._move(self._checked(frames.matrix_pose(current @ relative)))

    def read_power(self, kind: Optional[str] = None) -> float:
        """Power (W) now. Recorded with the pose, and drawn on the custom view."""
        self._check()
        return self._read(kind).power_w

    def read_still(self, kind: Optional[str] = None) -> float:
        """Power (W) with the fiber at rest, e.g. right after :meth:`move_to`.

        Recorded as taken at rest: the best of these is where run_alignment
        returns if the algorithm leaves the fiber somewhere clearly worse.
        Set the averaging such readings deserve with set_power_averaging().
        """
        self._check()
        return self._read(kind, still=True).power_w

    def scan(self, poses: Sequence[Sequence[float]], kind: Optional[str] = None,
             stop: Optional[Callable[[List[Sample]], bool]] = None) -> List[Sample]:
        """Move through ``poses`` in one continuous motion, reading the power all along.

        The moves are queued while the robot is paused, then run with the
        current blending, and readings are taken until the last pose is
        reached. Each reading's pose is interpolated between the poses read
        just before and just after it, at the middle of the reading, so a fast
        scan places its readings where they were taken. If ``stop(samples)``
        returns True (called after every reading, with this scan's samples),
        the motion is cleared and the scan ends there. Returns this scan's
        samples. An overrange reads +inf: read it as "more than the range".
        """
        self._check()
        targets = [self._checked(pose) for pose in poses]
        if not targets:
            return []
        robot = self._robot
        scanned: List[Sample] = []
        t0 = time.monotonic()
        self._log.info(f"Alignment scan ({kind or '-'}): {len(targets)} pose(s) queued")
        try:
            # Where the robot rests before the scan: the first reading's
            # "before". Read before the pause: the robot is never queried
            # while its motion is paused.
            before = self._stamped_pose()
            robot.PauseMotion()
            for target in targets:
                robot.MoveLin(*target)
            done = robot.SetCheckpoint(self._next_checkpoint())
            robot.ResumeMotion()
            stopped = False
            moved_from, moved_at = None, time.monotonic()
            while not done.is_set():
                self._check()
                # mecademicpy never "sets" a checkpoint the robot discarded
                # (motion cleared, robot in error or not activated): without
                # this, the loop would wait for it until the timeout.
                try:
                    done.check_interrupted()
                except Exception as e:
                    raise AlignmentError(f"The robot dropped the scan ({e}). Robot: {self._robot_state()}") from e
                sample = self._read(kind, before=before)
                before = self._last_pose
                scanned.append(sample)
                if moved_from is None or _moved(sample.pose, moved_from):
                    moved_from, moved_at = sample.pose, time.monotonic()
                elif time.monotonic() - moved_at > SCAN_STALL_S:
                    raise AlignmentError(
                        f"The robot is not executing the scan: no motion for {SCAN_STALL_S:g} s. "
                        f"Robot: {self._robot_state()}"
                    )
                if stop is not None and stop(scanned):
                    self._clear_motion()
                    stopped = True
                    break
                time.sleep(SCAN_SAMPLE_PERIOD_S)
        except BaseException as e:
            self._clear_motion()
            self._log.warning(f"Alignment scan ({kind or '-'}) ended by {type(e).__name__} after "
                              f"{time.monotonic() - t0:.2f} s, {len(scanned)} reading(s)")
            raise
        best = max((s.power_w for s in scanned if math.isfinite(s.power_w)), default=float("nan"))
        self._log.info(f"Alignment scan ({kind or '-'}) {'stopped by the algorithm' if stopped else 'done'}: "
                       f"{len(scanned)} reading(s) in {time.monotonic() - t0:.2f} s, best {best:.4g} W")
        return scanned

    def set_motion(self, acc: Optional[float] = None, lin_vel: Optional[float] = None,
                   ang_vel: Optional[float] = None, blending: Optional[float] = None) -> None:
        """Cartesian acceleration (%), linear (mm/s) and angular (°/s) velocity, blending (%)."""
        self._check()
        self._apply_motion(acc, lin_vel, ang_vel, blending)

    def set_power_averaging(self, count: int) -> None:
        """Samples averaged per reading (about 3 ms each on a PM100x). Restored afterwards."""
        self._check()
        self._set_meter("averaging", int(count))

    def set_power_auto_range(self, enabled: bool) -> None:
        """Power meter auto range. Restored afterwards."""
        self._check()
        self._set_meter("auto_range", bool(enabled))

    def set_power_range(self, watts: Optional[float]) -> None:
        """A fixed meter range that reads up to ``watts`` (W), or auto range with None. Restored afterwards.

        A fixed range keeps a scan free of range switches. Above it, readings are +inf.
        """
        self._check()
        if watts is None:
            self._set_meter("auto_range", True)
        else:
            self._set_meter("auto_range", False)
            self._set_meter("power_range", float(watts))

    # --- For run_alignment ---------------------------------------------------------

    def _check(self) -> None:
        self._check_interrupted()
        if time.monotonic() >= self._deadline:
            raise AlignmentTimeout(f"Alignment timed out after {self._settings.timeout_s:g} s.")

    def _check_interrupted(self) -> None:
        # ABORT clears the robot's motion queue, which makes a WaitIdle() in
        # progress raise. Between two moves (settling, reading the meter, or a
        # scan sampling) there is none to interrupt, and the robot accepts the
        # next move again. So the alignment also stops on the abort/fault event
        # the controller marks first. run_alignment marks align_start before the
        # algorithm runs, so an older abort can never be the latest event here.
        event = self._context.latest("events") or {}
        if event.get("name") in ("abort", "fault"):
            raise AlignmentInterrupted(f"Alignment interrupted ({event['name']}).")

    def _checked(self, pose: Sequence[float]) -> Pose6:
        """``pose`` as six floats, or AlignmentRangeError if it leaves the window."""
        try:
            target = tuple(float(v) for v in pose)
        except (TypeError, ValueError):
            raise AlignmentRangeError(f"Not a pose: {pose!r}") from None
        if len(target) != 6 or not all(math.isfinite(v) for v in target):
            raise AlignmentRangeError(f"Not a valid pose: {pose!r}")
        names = ("X", "Y", "Z", "alpha", "beta", "gamma")
        for i, name in enumerate(names):
            if i < 3:
                offset, limit, unit = target[i] - self.start[i], self._settings.range_mm, "mm"
            else:
                offset, limit, unit = _wrap_deg(target[i] - self.start[i]), self._settings.range_deg, "°"
            # The epsilon absorbs float noise on poses computed from the start.
            if abs(offset) > limit + 1e-9:
                raise AlignmentRangeError(
                    f"{name} {target[i]:.4f} is {offset:+.4f} {unit} from the start: "
                    f"outside the ±{limit:g} {unit} window. Pose: {target}"
                )
        return target

    def _move(self, target: Pose6) -> None:
        self._log.debug(f"Alignment move to {_fmt_pose(target)}")
        self._robot.MoveLin(*target)
        try:
            # Bounded: a robot that does not execute the move (paused, refused
            # it, ...) must not hold the task forever.
            self._robot.WaitIdle(timeout=MOVE_TIMEOUT_S)
        except Exception as e:
            self._clear_motion()
            self._check_interrupted()  # an ABORT reads as one, not as a failed move
            raise AlignmentError(f"The robot did not complete a move to {_fmt_pose(target)} "
                                 f"({type(e).__name__}: {e}). Robot: {self._robot_state()}") from e
        time.sleep(self._settings.settle_s)

    def _read(self, kind: Optional[str], before: Optional[_Stamped] = None, still: bool = False) -> Sample:
        # A direct reading on this (the task) thread. The custom view shows
        # these readings while they flow, instead of reading the meter itself.
        t_a = time.monotonic()
        value = self._meter.read_power()
        t_b = time.monotonic()
        if value is None:
            raise AlignmentError("The power meter returned no reading: alignment needs the real power meter.")
        after = self._stamped_pose()
        self._last_pose = after
        if not still:
            # Plain readings use the scan's averaging: they time a scan's readings.
            took = time.monotonic() - t_a
            self._read_s = took if self._read_s is None else 0.8 * self._read_s + 0.2 * took
        # The meter averages over the time between the query and its reply:
        # the pose at its middle, between the poses read around it.
        pose = after.pose if before is None else _interpolate(before, after, 0.5 * (t_a + t_b))
        sample = Sample(pose, float(value), kind, self._phase, round(self.elapsed_s(), 4), still)
        self.samples.append(sample)
        self._context.publish("align_sample", self.row(sample))
        # Only a finite reading can be the best: NaN is invalid, +inf an overrange.
        if math.isfinite(sample.power_w):
            if self.best is None or sample.power_w > self.best.power_w:
                self.best = sample
            if still and (self.best_still is None or sample.power_w > self.best_still.power_w):
                self.best_still = sample
        return sample

    def row(self, sample: Sample) -> List[Any]:
        """A sample as the custom view reads it (format SAMPLE_FORMAT):
        ``[x, y, z, rx, ry, rz, power_w, kind, phase, t, still]``, offsets from
        the start in its tool frame as :meth:`local` gives them (µm, degrees)."""
        x, y, z, rx, ry, rz = self.local(sample.pose)
        return [round(x, 3), round(y, 3), round(z, 3), round(rx, 5), round(ry, 5), round(rz, 5),
                sample.power_w, sample.kind, sample.phase, sample.t, sample.still]

    def _actual_pose(self) -> Pose6:
        return self._query_pose("GetRtCartPos")

    def _target_pose(self) -> Pose6:
        return self._query_pose("GetRtTargetCartPos")

    def _stamped_pose(self) -> _Stamped:
        """The actual pose, stamped at the middle of its query."""
        t = time.monotonic()
        pose = self._actual_pose()
        return _Stamped(0.5 * (t + time.monotonic()), pose)

    def _query_pose(self, command: str) -> Pose6:
        try:
            # value = getattr(self._robot, command)(synchronous_update=True)
            # GetRtCartPos for every pose: the synchronous target query
            # times out on the cell ("timeout waiting for interruptable event").
            value = self._robot.GetRtCartPos()
        except Exception as e:
            raise AlignmentError(f"{command} failed ({type(e).__name__}: {e}). Robot: {self._robot_state()}") from e
        return self._pose_from(value)

    def _robot_state(self) -> str:
        """The robot's status for an error message (activated, homed, sim, error, paused, ...). Never raises."""
        try:
            return str(self._robot.GetStatusRobot())
        except Exception as e:
            return f"status unavailable ({type(e).__name__})"

    @staticmethod
    def _pose_from(value: Any) -> Pose6:
        try:
            pose = tuple(float(v) for v in value[:6])
        except (TypeError, ValueError):
            pose = ()
        if len(pose) != 6:
            raise AlignmentError(f"The robot reported no position ({value!r}): alignment needs the real robot.")
        return pose

    def _apply_motion(self, acc, lin_vel, ang_vel, blending) -> None:
        robot = self._robot
        if acc is not None:
            robot.SetCartAcc(acc)
        if lin_vel is not None:
            robot.SetCartLinVel(lin_vel)
        if ang_vel is not None:
            robot.SetCartAngVel(ang_vel)
        if blending is not None:
            robot.SetBlending(blending)

    def _clear_motion(self) -> None:
        """Stop and empty the motion queue, then accept new moves. Never raises."""
        try:
            self._robot.ClearMotion()
            self._robot.ResumeMotion()
        except Exception:
            pass

    def _next_checkpoint(self) -> int:
        checkpoint = next(self._checkpoints, None)
        if checkpoint is None:
            self._checkpoints = iter(CHECKPOINT_IDS)
            checkpoint = next(self._checkpoints)
        return checkpoint

    def _set_meter(self, name: str, value: Any) -> None:
        if name not in self._meter_originals:
            self._meter_originals[name] = getattr(self._meter, name)
        setattr(self._meter, name, value)

    def _restore_meter(self) -> None:
        """Put back the meter settings the algorithm changed. Never raises."""
        # Auto range last: setting a fixed range turns it off.
        for name in sorted(self._meter_originals, key=lambda n: n == "auto_range"):
            try:
                setattr(self._meter, name, self._meter_originals[name])
            except Exception:
                pass

    def _best_rest(self) -> Optional[Sample]:
        """Where to go back to: the best reading taken at rest, else the best of all."""
        return self.best_still or self.best

    def _return_to(self, sample: Sample) -> None:
        """Slowly back to where ``sample`` was read (not window-checked: the robot was there)."""
        self._apply_motion(**RETURN_MOTION)
        self._move(self._pose_from(sample.pose))


#: The algorithm's signature, see algorithm.py.
Algorithm = Callable[[AlignmentIO, AlignmentSettings], None]


def run_alignment(robot: Any, meter: Any, context: AppContext, fiber: int,
                  align: Algorithm, logger: Optional[logging.Logger] = None,
                  phases: Sequence[str] = (), phase_views: Optional[Dict[str, Any]] = None,
                  seed: Optional[Sequence[float]] = None,
                  on_summary: Optional[Callable[[Dict[str, Any]], None]] = None) -> AlignmentResult:
    """Run ``align`` from the robot's current pose, then judge the result.

    Brackets the run with ``align_start`` / ``align_end`` events and publishes
    an ``align_result`` summary (points, phases, published data) for the
    custom view. Each phase is logged to ``logger`` (the robot's device log).
    ``phases`` are the algorithm's phase names, in order (``algorithm.PHASES``),
    and ``phase_views`` how the custom view plots each (``algorithm.PHASE_VIEWS``).
    ``seed`` is a robot pose where this fiber aligned before: the algorithm
    gets it as ``io.seed`` and may start there. ``on_summary`` is called with
    the ``align_result`` summary as it is published, whether the run passed,
    failed, or raised (e.g. to log each run).
    """
    log = logger or logging.getLogger(__name__)
    settings = AlignmentSettings.from_context(context)
    phase_views = dict(phase_views or {})
    io = AlignmentIO(robot, meter, context, settings, logger=log, phases=phases)
    if seed is not None:
        io.seed = io.local(seed)
    remembered = "" if io.seed is None else (
        f", remembered pose at x {io.seed[0]:.1f} µm, z {io.seed[2]:.1f} µm, "
        f"rx {io.seed[3]:.3f}°, rz {io.seed[5]:.3f}° from it")
    log.info(f"Alignment of fiber {fiber}: start {_fmt_pose(io.start)}{remembered}, window ±{settings.range_mm:g} mm "
             f"/ ±{settings.range_deg:g}°, threshold {settings.threshold_w:.4g} W, timeout {settings.timeout_s:g} s. "
             f"Robot: {io._robot_state()}")
    context.mark("align_start", fiber=fiber, threshold_w=settings.threshold_w, start=list(io.start),
                 range_mm=settings.range_mm, range_deg=settings.range_deg, phases=list(io.phases),
                 phase_views=phase_views, timeout_s=settings.timeout_s, source_power_w=settings.source_power_w,
                 sample_format=SAMPLE_FORMAT)

    timed_out = False
    try:
        try:
            align(io, settings)
        except AlignmentTimeout:
            timed_out = True
            io._check_interrupted()
            best = io._best_rest()
            log.warning(f"Alignment timed out after {settings.timeout_s:g} s: back to the best pose read "
                        f"({best.power_w if best else float('nan'):.4g} W)")
            if best is not None:
                io._return_to(best)
        # The verdict: a fresh reading where the algorithm left the fiber, in
        # auto range so that it is valid whatever range a scan last fixed.
        io._check_interrupted()
        io._set_meter("auto_range", True)
        final = io._read(None)
        best = io.best_still
        if best is not None and not final.power_w >= best.power_w * 10 ** (-FINAL_TOLERANCE_DB / 10):
            log.warning(f"Alignment left the fiber at {final.power_w:.4g} W, under the best reading at rest "
                        f"({best.power_w:.4g} W): back there before the verdict")
            io._return_to(best)
            io._check_interrupted()
            final = io._read(None)
    except Exception as e:
        # An abort lands here too, from WaitIdle() or as AlignmentInterrupted.
        # Close the run on the view, then let the task runner handle it.
        error = f"{type(e).__name__}: {e}"
        log.error(f"Alignment of fiber {fiber} failed after {io.elapsed_s():.1f} s: {error}")
        context.mark("align_end", fiber=fiber, passed=False, error=error)
        try:
            summary = _summary(io, settings, fiber, phase_views, passed=False, power_w=None, error=error)
            context.publish("align_result", summary)
            if on_summary is not None:
                on_summary(summary)
        except Exception as summary_error:  # never hide the alignment's own error
            log.warning(f"Could not publish the alignment summary: {summary_error}")
        raise
    finally:
        io._restore_meter()

    result = AlignmentResult(
        fiber=fiber,
        passed=final.power_w >= settings.threshold_w,
        power_w=final.power_w,
        pose=final.pose,
        reads=len(io.samples),
        duration_s=io.elapsed_s(),
        timed_out=timed_out,
    )
    context.mark("align_end", fiber=fiber, passed=result.passed, power_w=final.power_w)
    summary = _summary(io, settings, fiber, phase_views, passed=result.passed, power_w=final.power_w,
                       final=list(final.pose), final_local=list(io.row(final)[:6]), timed_out=timed_out)
    context.publish("align_result", summary)
    if on_summary is not None:
        on_summary(summary)
    return result


def _summary(io: AlignmentIO, settings: AlignmentSettings, fiber: int, phase_views: Dict[str, Any],
             **outcome: Any) -> Dict[str, Any]:
    """The align_result payload: enough for the custom view to redraw the run after a page reload."""
    # Every n-th reading, every reading at rest, the last one, and the best
    # one of the run and of each phase (the view shows them, and must show the
    # same after a reload).
    n = len(io.samples)
    keep = set(range(0, n, max(1, math.ceil(n / RESULT_POINTS_MAX))))
    best_of_phase: Dict[int, int] = {}
    for i, s in enumerate(io.samples):
        if s.still or s is io.best or s is io.best_still:
            keep.add(i)
        if math.isfinite(s.power_w) and (s.phase not in best_of_phase
                                         or s.power_w > io.samples[best_of_phase[s.phase]].power_w):
            best_of_phase[s.phase] = i
    keep.update(best_of_phase.values())
    keep.add(n - 1)
    kept = [io.samples[i] for i in sorted(keep) if i >= 0]
    ran = {entry["index"] for entry in io.phase_log}
    phases = [dict(entry) for entry in io.phase_log]
    for index, name in enumerate(io.phases):
        if index not in ran:
            # Skipped if a later phase ran, else the run ended before reaching it.
            later = any(i > index for i in ran)
            phases.append({"name": name, "index": index, "status": "skipped" if later else "not run"})
    phases.sort(key=lambda p: (p["index"] if p["index"] >= 0 else len(io.phases), p.get("t0", 0.0)))
    return {
        "fiber": fiber,
        "threshold_w": settings.threshold_w,
        "source_power_w": settings.source_power_w,
        "start": list(io.start),
        "sample_format": SAMPLE_FORMAT,
        "timeout_s": settings.timeout_s,
        "phase_names": list(io.phases),
        "phase_views": phase_views,
        "reads": len(io.samples),
        "duration_s": round(io.elapsed_s(), 2),
        **outcome,
        "best_still": io.row(io.best_still) if io.best_still else None,
        "phases": phases,
        "published": {name: values[-RESULT_PUBLISHED_MAX:] for name, values in io.published.items()},
        "points": [io.row(s) for s in kept],
    }


def _interpolate(before: _Stamped, after: _Stamped, t: float) -> Pose6:
    """The pose at time ``t``, linearly between two stamped poses (angles the short way round)."""
    span = after.t - before.t
    f = min(1.0, max(0.0, (t - before.t) / span)) if span > 0 else 1.0
    return tuple(
        _wrap_deg(b + f * _wrap_deg(a - b)) if i >= 3 else b + f * (a - b)
        for i, (b, a) in enumerate(zip(before.pose, after.pose))
    )


def _wrap_deg(angle: float) -> float:
    """``angle`` brought into [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def _moved(pose: Pose6, reference: Pose6) -> bool:
    """True once ``pose`` is more than SCAN_MOTION_EPS (mm or °) away from ``reference``."""
    return any(abs(_wrap_deg(a - b) if i >= 3 else a - b) > SCAN_MOTION_EPS
               for i, (a, b) in enumerate(zip(pose, reference)))


def _fmt_pose(pose: Sequence[float]) -> str:
    return "(" + ", ".join(f"{v:.4f}" for v in pose) + ")"
