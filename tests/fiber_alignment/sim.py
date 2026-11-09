"""A simulated alignment cell: robot, power meter and context, on a simulated clock.

Enough of mecademicpy and of the Thorlabs driver for alignment.py and
algorithm.py to run unchanged, fast and deterministically:

* :class:`SimClock` stands in for ``alignment.time``: it moves only when the
  code sleeps, reads the meter or queries the robot.
* :class:`FakeRobot` executes queued ``MoveLin`` moves at constant speed, with
  pause/resume, checkpoints and clear, like the real motion queue. It answers
  pose queries as the cell does: ``GetRtCartPos`` only (the synchronous
  ``GetRtTargetCartPos`` times out there), and never while motion is paused.
* :class:`FakeMeter` reads the power the fiber ends would couple at the
  robot's pose (:class:`Optics`: collimated ends like the cell's, or bare
  single-mode fibers), averaged over the time a reading takes while the robot
  moves: a reading taken during a scan is smeared and late, as on the cell.
* :class:`FakeContext` publishes to a real :class:`core.DataHub` and records the
  timeline in simulated time, for the replay tool (app_logic/tools/view_replay.py).

:func:`simulate` runs one alignment in a :class:`Scenario` and returns
everything a test needs.
"""

from __future__ import annotations

import bisect
import math
import random
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from core.DataHub import DataHub
from app_logic.fiber_alignment import alignment, algorithm, frames

#: A generic start pose: tool axes not aligned with the WRF, far from gimbal lock.
START_POSE = (180.0, -20.0, 140.0, 10.0, 20.0, -30.0)
#: Single-mode fiber at 1557 nm: mode field radius (µm) and wavelength (µm).
MODE_RADIUS_UM = 5.2
WAVELENGTH_UM = 1.557
#: Two glass/air faces, about 3.5 % reflected each.
FRESNEL = 0.93
#: The cell's collimated ends, as its alignment of 2026-10-27 showed them: the
#: 1/e² radius of the power against a lateral offset (µm), its 1/e half-width
#: against a tilt (°), and the distance between the ends (µm), over which a
#: tilt walks the beam sideways.
BEAM_WIDTH_UM = 240.0
BEAM_ANGLE_DEG = 0.3
BEAM_GAP_UM = 10000.0
#: The meter's response time in its higher ranges (s), with FakeMeter.tau_s set.
FAST_TAU_S = 0.002


class SimClock:
    """Simulated time: what ``alignment.time`` provides, plus :meth:`advance`."""

    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(0.0, seconds)

    def advance(self, seconds: float) -> None:
        self.now += max(0.0, seconds)


class FakeCheckpoint:
    """Like mecademicpy's InterruptableEvent for a checkpoint."""

    def __init__(self, robot: "FakeRobot", n: int) -> None:
        self._robot = robot
        self.n = n
        self._set = False
        self._interrupted = False

    def is_set(self) -> bool:
        self._robot._advance()
        return self._set

    def check_interrupted(self) -> None:
        if self._interrupted:
            raise RuntimeError(f"checkpoint {self.n} discarded")


class FakeRobot:
    """The mecademicpy calls alignment.py makes, on a simulated clock.

    Moves run in a straight line at the linear and angular speeds in force when
    they start (speed changes are queued, as on the robot). Blending and
    accelerations are ignored.
    """

    def __init__(self, clock: SimClock, start: Sequence[float] = START_POSE, pose_query_s: float = 0.001,
                 pose_delay_s: float = 0.0):
        self.clock = clock
        self.pose_query_s = pose_query_s
        #: How old the poses it reports are (s): its real-time data, late.
        self.pose_delay_s = pose_delay_s
        self._times: List[float] = []
        self._poses: List[Tuple[float, ...]] = []
        self._pose = tuple(float(v) for v in start)
        self._queue: deque = deque()
        self._paused = False
        self._t = clock.now
        self._segment: Optional[Tuple[Tuple[float, ...], Tuple[float, ...], float, float]] = None
        self.lin_vel = 10.0
        self.ang_vel = 45.0
        self.moves: List[Tuple[float, ...]] = []
        # Every pose query: (command, motion paused at the time).
        self.pose_queries: List[Tuple[str, bool]] = []
        self.on_pose = None  # called with every pose the robot passes through (collision check)

    # --- Motion queue -------------------------------------------------------------

    def MoveLin(self, *pose: float) -> None:
        target = tuple(float(v) for v in pose)
        self.moves.append(target)
        self._queue.append(("move", target))

    def SetCheckpoint(self, n: int) -> FakeCheckpoint:
        checkpoint = FakeCheckpoint(self, n)
        self._queue.append(("checkpoint", checkpoint))
        return checkpoint

    def SetCartLinVel(self, v: float) -> None:
        self._queue.append(("lin_vel", float(v)))

    def SetCartAngVel(self, v: float) -> None:
        self._queue.append(("ang_vel", float(v)))

    def SetCartAcc(self, v: float) -> None:
        pass

    def SetBlending(self, v: float) -> None:
        pass

    def PauseMotion(self) -> None:
        self._advance()
        self._paused = True

    def ResumeMotion(self) -> None:
        self._advance()
        self._paused = False

    def ClearMotion(self) -> None:
        self._advance()
        for kind, item in self._queue:
            if kind == "checkpoint":
                item._interrupted = True
        self._queue.clear()
        # Stops where it is (the real robot decelerates over a few µm).
        self._segment = None

    def WaitIdle(self, timeout: Optional[float] = None) -> None:
        self._advance()
        while self._segment is not None or self._queue:
            if self._paused:
                self.clock.advance(timeout or 0.0)
                raise TimeoutError("robot paused")
            if self._segment is None:
                self._start_next()
                continue
            _, _, t0, duration = self._segment
            self.clock.now = max(self.clock.now, t0 + duration)
            self._advance()

    # --- Real-time data -----------------------------------------------------------

    def GetRtCartPos(self) -> List[float]:
        self.pose_queries.append(("GetRtCartPos", self._paused))
        if self._paused:
            # Not something the cell was ever asked: the alignment must not do it.
            raise TimeoutError("Timeout waiting for interruptable event (pose queried while motion is paused)")
        # The robot answers halfway through the query.
        self.clock.advance(self.pose_query_s / 2)
        self._advance()
        pose = list(self._reported())
        self.clock.advance(self.pose_query_s / 2)
        return pose

    def GetRtTargetCartPos(self) -> List[float]:
        # As on the cell: the synchronous target query never gets its answer.
        self.pose_queries.append(("GetRtTargetCartPos", self._paused))
        raise TimeoutError("Timeout waiting for interruptable event")

    def GetStatusRobot(self) -> str:
        return "FakeRobot(activated, homed)"

    # --- Internals ------------------------------------------------------------------

    @property
    def pose(self) -> Tuple[float, ...]:
        self._advance()
        return self._pose

    def _duration(self, a: Sequence[float], b: Sequence[float]) -> float:
        lin = math.dist(a[:3], b[:3]) / max(self.lin_vel, 1e-6)
        ang = max(abs(alignment._wrap_deg(y - x)) for x, y in zip(a[3:], b[3:])) / max(self.ang_vel, 1e-6)
        return max(lin, ang, 1e-6)

    def _start_next(self) -> None:
        """Take queued items until a move starts (or the queue is empty), at time self._t."""
        while self._queue and self._segment is None:
            kind, item = self._queue.popleft()
            if kind == "move":
                self._segment = (self._pose, item, self._t, self._duration(self._pose, item))
            elif kind == "checkpoint":
                item._set = True
            elif kind == "lin_vel":
                self.lin_vel = item
            elif kind == "ang_vel":
                self.ang_vel = item

    def _advance(self) -> None:
        """Bring the motion up to the clock."""
        now = self.clock.now
        while True:
            if self._paused:
                self._t = now
                return
            if self._segment is None:
                self._start_next()
                if self._segment is None:
                    self._t = now
                    return
            a, b, t0, duration = self._segment
            if now >= t0 + duration:
                self._pose = b
                self._t = t0 + duration
                self._segment = None
                self._passed()
                continue
            f = (now - t0) / duration
            self._pose = tuple(x + f * (alignment._wrap_deg(y - x) if i >= 3 else y - x)
                               for i, (x, y) in enumerate(zip(a, b)))
            self._t = now
            self._passed()
            return

    def _passed(self) -> None:
        if self.pose_delay_s > 0:
            self._times.append(self.clock.now)
            self._poses.append(self._pose)
            if len(self._times) > 20000:
                del self._times[:10000], self._poses[:10000]
        if self.on_pose is not None:
            self.on_pose(self._pose)

    def _reported(self) -> Tuple[float, ...]:
        """The pose it reports: its pose pose_delay_s ago (interpolated between those it passed), else now."""
        if self.pose_delay_s <= 0 or not self._times:
            return self._pose
        t = self.clock.now - self.pose_delay_s
        k = bisect.bisect_right(self._times, t)
        if k == 0:
            return self._poses[0]
        if k == len(self._times):
            return self._pose if t >= self._times[-1] and self._segment is None else self._poses[-1]
        (t0, p0), (t1, p1) = (self._times[k - 1], self._poses[k - 1]), (self._times[k], self._poses[k])
        f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return tuple(x + f * (alignment._wrap_deg(y - x) if i >= 3 else y - x) for i, (x, y) in enumerate(zip(p0, p1)))


@dataclass
class Optics:
    """Where the fibers couple best, and how.

    The optimum is the start pose moved by ``offset`` (x, z in µm, rx, rz in
    degrees, in the start's tool frame): what the alignment has to find. The
    held fiber is the receiver (the fixed end emits): it points along its tool
    +y, at ``gap_um`` from the fixed end at the optimum. The coupling is the
    same either way round (reciprocity), so it is computed as if the held fiber
    sent its own beam to the fixed end: a tilt walks that beam sideways over
    the gap. ``pivot_um`` is how far the real fiber tip is from the taught
    TCP, in the tool frame: rotations then also move the tip.

    ``model`` "beam": collimated ends, a Gaussian in the lateral offset where
    the beam lands (``width_um``) times a Gaussian in the tilt (``angle_deg``).
    "smf": bare single-mode fibers, both widths from the mode and the gap.
    """

    offset: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    model: str = "beam"
    gap_um: float = BEAM_GAP_UM
    width_um: float = BEAM_WIDTH_UM
    angle_deg: float = BEAM_ANGLE_DEG
    source_w: float = 1e-3
    pivot_um: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    start: Tuple[float, ...] = START_POSE
    #: "beam" optics only: faint wide wings around the core, as on the cell in
    #: November 2026: (amplitude, against the core's peak; 1/e² radius, µm;
    #: 1/e tilt half-width, °). None: a pure Gaussian.
    wing: Optional[Tuple[float, float, float]] = None

    def __post_init__(self) -> None:
        dx, dz, drx, drz = self.offset
        # The real tip is the TCP moved by the pivot error, in the tool frame.
        self._pivot = frames.make_homogeneous_matrix([v * 1e-3 for v in self.pivot_um], [0, 0, 0])
        self._pivot_inverse = np.linalg.inv(self._pivot)
        # The optimum, as a tip pose: the start's tip moved by `offset` in its own frame.
        T_start_tip = frames.pose_matrix(self.start) @ self._pivot
        self._optimum = T_start_tip @ frames.make_homogeneous_matrix_extrinsic(
            [dx * 1e-3, 0, dz * 1e-3], [drx, 0, drz])
        self._optimum_inverse = np.linalg.inv(self._optimum)

    def tip_offset(self, pose: Sequence[float]) -> Tuple[float, float, float, float, float]:
        """The tip, with the robot at ``pose``, relative to the optimum: x, y, z (µm), rx, rz (degrees)."""
        T = self._optimum_inverse @ frames.pose_matrix(pose) @ self._pivot
        position, angles = frames.extract_pose_extrinsic(T)
        x, y, z = (float(v) * 1000.0 for v in position)
        return x, y, z, float(angles[0]), float(angles[2])

    def gap(self, pose: Sequence[float]) -> float:
        """Gap between the fiber facets (µm) with the robot at ``pose``."""
        return self.gap_um - self.tip_offset(pose)[1]

    def coupling(self, pose: Sequence[float]) -> float:
        """Fraction of the source coupled with the robot at ``pose``."""
        x, y, z, rx, rz = self.tip_offset(pose)
        gap = max(self.gap_um - y, 0.0)
        # Where the held fiber's own beam (by reciprocity) lands on the fixed end: a tilt walks it sideways.
        x_land = x - gap * math.sin(math.radians(rz))
        z_land = z + gap * math.sin(math.radians(rx))
        if self.model == "beam":
            r2, t2 = x_land ** 2 + z_land ** 2, rx ** 2 + rz ** 2
            core = math.exp(-2.0 * r2 / self.width_um ** 2 - t2 / self.angle_deg ** 2)
            if self.wing is None:
                return FRESNEL * core
            amplitude, width, angle = self.wing
            return FRESNEL * (core + amplitude * math.exp(-2.0 * r2 / width ** 2 - t2 / angle ** 2))
        z_r = math.pi * MODE_RADIUS_UM ** 2 / WAVELENGTH_UM
        width_sq = MODE_RADIUS_UM ** 2 * (2.0 + (gap / z_r) ** 2)
        gap_loss = 1.0 / (1.0 + (gap / (2.0 * z_r)) ** 2)
        tilt_rad = math.radians(math.hypot(rx, rz))
        tilt_limit = WAVELENGTH_UM / (math.pi * MODE_RADIUS_UM)
        return (FRESNEL * gap_loss * math.exp(-2.0 * (x_land ** 2 + z_land ** 2) / width_sq)
                * math.exp(-(tilt_rad / tilt_limit) ** 2))

    def optimum_pose(self) -> Tuple[float, ...]:
        """The robot (TCP) pose that couples best."""
        return frames.matrix_pose(self._optimum @ self._pivot_inverse)

    def best_power_w(self) -> float:
        """The most power the algorithm's moves can couple (W): x, z, rx, rz in the start's tool frame, y kept."""
        from scipy.optimize import minimize

        T_start = frames.pose_matrix(self.start)

        def pose_of(v):
            return frames.matrix_pose(T_start @ frames.make_homogeneous_matrix_extrinsic(
                [v[0] * 1e-3, 0, v[1] * 1e-3], [v[2], 0, v[3]]))

        dx, dz, drx, drz = self.offset
        res = minimize(lambda v: -self.coupling(pose_of(v)), [dx, dz, drx, drz], method="Nelder-Mead",
                       options={"xatol": 1e-4, "fatol": 1e-12, "maxiter": 4000})
        return -res.fun * self.source_w


class FakeMeter:
    """The Thorlabs driver calls alignment.py makes: ``read_power`` and its settings.

    A reading takes ``latency_s`` plus ``sample_s`` per averaged sample. It
    averages the power at the robot's pose over that time, so it lags and
    smears during a scan. Noise: ``noise_w`` plus ``noise_rel`` of the power.
    A fixed range reads +inf above it; auto range switches with a delay.

    ``tau_s``: the meter's own response time (first order) while it shows
    less than ``tau_below_w``, as in its low-power ranges on the cell (above
    it, ``FAST_TAU_S``). A reading taken on the move then shows the power of
    a pose already left behind: on a narrow beam, going back to the pose of
    the best reading can land beside the light. ``background_w``: stray
    light, read everywhere.

    ``range_up_s``: in auto range, how long the power must stay over the
    range before the meter switches up; until then it reads the range's
    top. Its lowest range is ``lowest_range_w``. On the cell (2026-11-06,
    ~0.1 nW of stray light, so the meter sat in its lowest range), scans
    crossing µW of light read 50-100 nW on the move.
    """

    def __init__(self, clock: SimClock, robot: FakeRobot, optics: Optics, *, dark: bool = False,
                 latency_s: float = 0.0015, sample_s: float = 0.003, noise_w: float = 0.5e-9,
                 noise_rel: float = 1e-3, seed: int = 0, tau_s: float = 0.0, tau_below_w: float = math.inf,
                 background_w: float = 0.0, range_up_s: float = 0.0, lowest_range_w: float = 1.2e-8):
        self.clock = clock
        self.robot = robot
        self.optics = optics
        self.dark = dark
        self.latency_s = latency_s
        self.sample_s = sample_s
        self.noise_w = noise_w
        self.noise_rel = noise_rel
        self.averaging = 1
        self.auto_range = True
        self.power_range = 1.2e-3
        self.power_unit = "W"
        self.reads = 0
        self.tau_s = tau_s
        self.tau_below_w = tau_below_w
        self.background_w = background_w
        self.range_up_s = range_up_s
        self.lowest_range_w = lowest_range_w
        self._over_since: Optional[float] = None  # since when the power is over the auto range, with range_up_s
        self._rng = random.Random(seed)
        self._auto_top = 1.2e-3
        self._seen: Optional[Tuple[float, float]] = None  # (time, power the meter shows), with tau_s

    def read_power(self) -> float:
        self.reads += 1
        steps = max(1, int(self.averaging))
        self.clock.advance(self.latency_s / 2)
        total = 0.0
        for _ in range(steps):
            self.clock.advance(self.sample_s / 2)
            total += self._power_now()
            self.clock.advance(self.sample_s / 2)
        self.clock.advance(self.latency_s / 2)
        power = total / steps
        power += self._rng.gauss(0.0, self.noise_w + self.noise_rel * power)
        if self.auto_range:
            # Up once the power exceeds the range (for range_up_s), down only
            # under a hundredth of it, like the meter; each switch costs a few ms.
            top = self._auto_top
            if power > top and self.range_up_s > 0:
                if self._over_since is None:
                    self._over_since = self.clock.now
                if self.clock.now - self._over_since < self.range_up_s:
                    return top
            self._over_since = None
            while power > top:
                top *= 10.0
            while power < top / 100.0 and top > self.lowest_range_w:
                top /= 10.0
            if top != self._auto_top:
                self.clock.advance(0.005)
                self._auto_top = top
            return power
        if power > self.power_range:
            return float("inf")
        # 14 bits of resolution over the range.
        return round(power / self.power_range * 16384) / 16384 * self.power_range

    def _power_now(self) -> float:
        power = self.background_w + (0.0 if self.dark else self.optics.source_w * self.optics.coupling(self.robot.pose))
        if self.tau_s <= 0:
            return power
        # First order: the shown power moves towards the true one, slowly in the low ranges.
        now = self.clock.now
        if self._seen is not None:
            t, shown = self._seen
            tau = self.tau_s if shown < self.tau_below_w else FAST_TAU_S
            power = shown + (power - shown) * (1.0 - math.exp(-max(0.0, now - t) / tau))
        self._seen = (now, power)
        return power


class FakeContext:
    """The AppContext calls alignment.py makes, on a real DataHub.

    Also records every publish and mark with its simulated time in ``timeline``,
    for app_logic/tools/view_replay.py.
    """

    def __init__(self, clock: SimClock, params: Optional[Dict[str, Any]] = None):
        self.clock = clock
        self.data = DataHub()
        self.params: Dict[str, Any] = dict(params or {})
        self.timeline: List[Tuple[float, str, str, Any]] = []
        self.steps: List[str] = []

    def get_param(self, name: str, default: Any = None) -> Any:
        return self.params.get(name, default)

    def publish(self, channel: str, value: Any) -> None:
        self.timeline.append((self.clock.now, "publish", channel, value))
        self.data.publish(channel, value)

    def mark(self, name: str, **data: Any) -> None:
        self.timeline.append((self.clock.now, "mark", name, data))
        self.data.mark(name, **data)

    def latest(self, channel: str, default: Any = None) -> Any:
        return self.data.latest(channel, default)

    @contextmanager
    def step(self, name: str):
        self.steps.append(name)
        yield

    def published(self, channel: str) -> List[Any]:
        return [value for _, kind, name, value in self.timeline if kind == "publish" and name == channel]

    def events(self, name: Optional[str] = None) -> List[Dict[str, Any]]:
        return [dict(data, name=n) for _, kind, n, data in self.timeline
                if kind == "mark" and (name is None or n == name)]


#: Params of a simulated run: the defaults of context.yaml, with a threshold the model can reach.
DEFAULT_PARAMS = {
    "power_threshold_db": -0.97,  # 0.80 mW of the 1 mW source
    "align_range_um": 2000.0,
    "align_range_deg": 8.0,
    "align_timeout_s": 300.0,
    "align_settle_s": 0.05,
    "source_power_mw": 1.0,
    "lateral_width_um": 200.0,
    "angle_width_deg": 0.3,
    "acquire_radius_deg": 2.0,
    "acquire_radius_um": 500.0,
    "signal_detect_uw": 1.0,
    "acquire_full_search": False,
    "peak_grid": 5,
}
#: Bare single-mode fibers 20 µm apart: what changes for them.
SMF = {"gap_um": 20.0, "params": {"lateral_width_um": 10.0, "angle_width_deg": 5.0}}


@dataclass
class Scenario:
    """One simulated alignment: where the optimum is, and how the cell behaves."""

    offset: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    #: "beam" (the cell's collimated ends) or "smf" (bare single-mode fibers, see SMF).
    optics: str = "beam"
    gap_um: Optional[float] = None
    #: The beam's 1/e² radius (µm) and 1/e tilt half-width (°), "beam" optics; None: the module's defaults.
    width_um: Optional[float] = None
    angle_deg: Optional[float] = None
    #: Faint wide wings around the beam's core: see Optics.wing.
    wing: Optional[Tuple[float, float, float]] = None
    #: How old the robot's reported poses are (s): see FakeRobot.pose_delay_s.
    pose_delay_s: float = 0.0
    #: The meter's response time (s) under meter_tau_below_w, and stray light (W): see FakeMeter.
    meter_tau_s: float = 0.0
    meter_tau_below_w: float = math.inf
    background_w: float = 0.0
    #: How long the meter takes to switch up out of a range, and its lowest range (W): see FakeMeter.
    meter_range_up_s: float = 0.0
    meter_lowest_range_w: float = 1.2e-8
    pivot_um: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    dark: bool = False
    latency_s: float = 0.0015
    sample_s: float = 0.003
    pose_query_s: float = 0.001
    noise: bool = True
    seed: int = 0
    params: Dict[str, Any] = field(default_factory=dict)
    #: The walk a previous run left (µm/° by tilt axis: process.py passes memory.remembered_walk), or None.
    walk: Optional[Dict[str, float]] = None


@dataclass
class Run:
    """What one simulated alignment left behind."""

    result: Optional[alignment.AlignmentResult]
    error: Optional[BaseException]
    context: FakeContext
    robot: FakeRobot
    meter: FakeMeter
    optics: Optics
    clock: SimClock

    @property
    def summary(self) -> Dict[str, Any]:
        return self.context.published("align_result")[-1]

    @property
    def rows(self) -> List[List[Any]]:
        return self.context.published("align_sample")


def simulate(scenario: Scenario, align=None, phases=None, phase_views=None) -> Run:
    """Run one alignment in ``scenario`` on a simulated clock. Never raises: see ``Run.error``."""
    clock = SimClock()
    smf = scenario.optics == "smf"
    gap = scenario.gap_um if scenario.gap_um is not None else (SMF["gap_um"] if smf else BEAM_GAP_UM)
    shape = {k: v for k, v in (("width_um", scenario.width_um), ("angle_deg", scenario.angle_deg),
                                ("wing", scenario.wing)) if v is not None}
    optics = Optics(offset=scenario.offset, model=scenario.optics, gap_um=gap, pivot_um=scenario.pivot_um, **shape)
    robot = FakeRobot(clock, pose_query_s=scenario.pose_query_s, pose_delay_s=scenario.pose_delay_s)
    meter = FakeMeter(clock, robot, optics, dark=scenario.dark, latency_s=scenario.latency_s,
                      sample_s=scenario.sample_s, noise_w=0.5e-9 if scenario.noise else 0.0,
                      noise_rel=1e-3 if scenario.noise else 0.0, seed=scenario.seed,
                      tau_s=scenario.meter_tau_s, tau_below_w=scenario.meter_tau_below_w,
                      background_w=scenario.background_w, range_up_s=scenario.meter_range_up_s,
                      lowest_range_w=scenario.meter_lowest_range_w)
    context = FakeContext(clock, {**DEFAULT_PARAMS, **(SMF["params"] if smf else {}), **scenario.params})

    def check_gap(pose):
        if optics.gap(pose) <= 0.0:
            raise AssertionError(f"The fibers touched: gap {optics.gap(pose):.2f} µm at {pose}")

    robot.on_pose = check_gap
    saved = alignment.time
    alignment.time = SimpleNamespace(monotonic=clock.monotonic, sleep=clock.sleep)
    try:
        result = alignment.run_alignment(
            robot, meter, context, fiber=1, align=align or algorithm.align,
            phases=algorithm.PHASES if phases is None else phases,
            phase_views=algorithm.PHASE_VIEWS if phase_views is None else phase_views,
            walk=scenario.walk)
        error = None
    except Exception as e:  # the test decides whether it was expected
        result, error = None, e
    finally:
        alignment.time = saved
    return Run(result, error, context, robot, meter, optics, clock)
