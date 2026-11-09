"""Fiber alignment algorithm: find the light, centre on it, square the angle, climb to the peak.

Built for a demo cell that may start cold, with collimated (or lensed) fiber
ends: the coupling tolerates a lateral offset of a few hundred µm but a tilt
of a few tenths of a degree only, so first light is mostly a matter of angle.
It never moves the fiber along its axis: the gap stays the one the
``align_start`` program taught.

How wide the coupling is, against a lateral offset and against a tilt, sets
every scale of the later phases (scan spans, steps). The params give starting
guesses (``lateral_width_um``, ``angle_width_deg``); each fit re-measures them.
The first-light scans are sized from how far light still shows from the
optimum instead (``acquire_capture_um``, ``acquire_capture_deg``): a beam's
faint wings can carry light much further than its core.

``align`` runs four phases, each inside ``with io.phase(name):`` and plotted by
the custom view as PHASE_VIEWS declares:

1. Acquire: the noise floor where the search starts: the pose remembered for
   this fiber (``io.seed``) if there is one, else the start. Unless light
   already reaches the meter, Lissajous scans of growing discs around it,
   a translation (x/z) then a rotation (Rx/Rz) at each size. Any hint of
   light, a loss ``hint_gain_db`` better than the centre's but under the
   detection level, confirmed at rest, becomes the new centre: the scans
   start again from
   the smallest disc around it. Light at the detection level, confirmed at
   rest, ends the search; back at the pose of a reading taken on the move,
   a fiber that misses the light (the meter lagging, or poses read late)
   finds it again going back over that stretch of the scan, slower. Dark at
   the remembered pose, the same around the start;
   still dark, rotation scans at ``acquire_ring_um`` around the start (6
   points). The scans are sized from how far light shows from the optimum
   (``acquire_capture_um``, ``acquire_capture_deg``), not from the beam's
   core. With ``acquire_full_search`` (for demos), a translation then a
   rotation scan over the full radius run to their end first.
2. Center: a two-way scan across the light along x, then z, each fitted
   with a Gaussian over the noise floor; the fiber moves to the fitted
   centre. The two directions also measure what is left of the
   reading/pose lag.
3. Angle: for rx then rz, tilts stepping away from the current one while the
   power rises. With the walk known (how far the beam moves sideways per
   degree of tilt: remembered from the last run, ``io.walk``), each tilt is
   read at rest where the walk puts the beam; else each is re-centred with a
   cross scan along the lateral axis the tilt walks the beam on. A parabola
   through the log of the power at the best tilt and its two neighbours
   gives the tilt to keep, re-centred there along both lateral axes. A second,
   finer round only when the first was not clean.
4. Peak: first the gradient circles of the cell's first script, in rounds: a
   circle in x/z, then one in Rx/Rz (carrying x/z along by the walk the
   Angle phase measured), each read on the move, a plane fitted to the log
   power over it, and a step up its slope (halved after an overshoot), until
   no step stands out of the noise (``gradient_rounds`` at most). Then a square pattern of readings at rest
   around the centre (``peak_grid`` per side, 5 by default), a quadratic fit
   of the log power and a Newton step to its top, until the steps are small;
   then to the best reading taken at rest.

Positions are offsets from the start in the start's tool frame (see
``io.local`` / ``io.pose_at``): x and z lateral (µm), y along the fiber (never
moved), rx and rz rotations about the tool axes (degrees). Scan speeds come
from the measured time between readings (``io.sample_period_s``), so readings
are as dense as intended whatever the meter and the network do.

The contract with the cell (see alignment.py): move only through ``io``, never
catch exceptions broadly, leave the fiber at the best pose found. The custom
view plots the readings and what is published here with ``io.publish()``:
"acquire", "profile", "gradient" and "peak" (their fields: custom_view/__init__.py).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import fit
from .alignment import AlignmentError, AlignmentIO, AlignmentNoSignal, AlignmentSettings, Sample
from .frames import Pose6

#: The phases, in order. The custom view lists them and plots each as PHASE_VIEWS says.
PHASES = ("Acquire", "Center", "Angle", "Peak")

#: How the custom view plots each phase: "map" is a 3D landscape of the power
#: over two axes (Acquire: X/Z, whichever search ran; a map may instead follow
#: each search's own axes with "search_axes", e.g. {"angle": ["rx", "rz"]}),
#: "profiles" the latest 1-D scan per axis (one plot per slot), "peak" the
#: lateral circles' and the pattern's readings, and the steps of the peak search. Axes are io.local's:
#: x, y, z (µm), rx, ry, rz (°). A map "until_end" goes on plotting the
#: readings of the later phases, to the end of the run, with the best power
#: seen at each spot: the landscape rises as the alignment climbs.
PHASE_VIEWS = {
    # Over the axes of the search running: a rotation scan's Lissajous shows
    # over Rx/Rz. Once Center starts, over X/Z, where the later phases climb.
    "Acquire": {"type": "map", "axes": ["x", "z"], "search_axes": {"lateral": ["x", "z"], "angle": ["rx", "rz"]},
                "until_end": True},
    "Center": {"type": "profiles", "slots": ["x", "z"]},
    "Angle": {"type": "profiles", "slots": ["rx", "rz", "x", "z"]},
    "Peak": {"type": "peak", "axes": ["x", "z"]},
}

# --- Meter ---------------------------------------------------------------------------
#: Samples averaged per reading (about 3 ms each on a PM100x): while scanning, and at rest.
SCAN_AVERAGING = 1
STILL_AVERAGING = 10
#: Fixed meter range of the scans (no range switch mid-scan): at most this × the source power.
SCAN_RANGE = 1.2
#: A scan's range starts at this × the power expected (at least × the
#: detection level), so faint light keeps the meter's resolution: at 1.2 mW,
#: a reading of a few tens of nW rounds to nothing. An overrange raises it ×10.
SCAN_HEADROOM = 10.0
#: Highest range tried when light overranges it (W).
MAX_RANGE_W = 1.0

# --- Motion ----------------------------------------------------------------------------
#: Point-to-point moves: to a scan's start, to a pattern point, to a tilt.
#: Under a mm inside the window: at 2 mm/s they took ~0.2 s of each cross scan.
MOVE_MOTION = {"acc": 50, "lin_vel": 10.0, "ang_vel": 5.0, "blending": 0}
#: Slowest scan speeds (mm/s, °/s).
MIN_SCAN_SPEED = 0.01
MIN_SCAN_ANG_SPEED = 0.01

# --- Widths ------------------------------------------------------------------------------
#: A fitted width changes at most by this factor per fit, and stays within these bounds.
WIDTH_CHANGE = 3.0
LATERAL_WIDTH_BOUNDS_UM = (1.0, 2000.0)
ANGLE_WIDTH_BOUNDS_DEG = (0.005, 10.0)

# --- Acquire -----------------------------------------------------------------------------
#: Readings at the start that give the noise floor.
NOISE_READS = 20
#: Light is a reading this many noise standard deviations above the floor, and
#: at least signal_detect_uw.
NOISE_SIGMAS = 8.0
#: The search scans growing discs around its centre: these fractions of
#: acquire_radius_um (translation) and acquire_radius_deg (rotation).
ACQUIRE_STAGES = (1 / 3, 2 / 3, 1.0)
#: The searches of each stage, in turn: translation, then rotation.
SEARCH_ORDER = ("lateral", "angle")
#: A Lissajous scan, u = A sin(n t), v = A sin((n + 1) t), leaves no point of
#: its square ± A farther than about LISSAJOUS_GAP · A / n from its path
#: (measured; its path is about 6.2 · A · n long). Its points beyond A from
#: the centre are brought back onto that circle: no scan reaches farther
#: than A, and no point of its disc is left farther from its path.
LISSAJOUS_GAP = 1.05
#: Each scan passes within LISSAJOUS_COVER capture radii (acquire_capture_um,
#: acquire_capture_deg) of every point of its disc, and reads every
#: LISSAJOUS_READING capture radii along its path: light it crosses shows in
#: several readings.
LISSAJOUS_COVER = 0.7
LISSAJOUS_READING = 0.35
#: Moves queued per scan: a long path runs as consecutive scans.
SCAN_CHUNK = 400
#: A hint of light: HINT_READINGS readings in a row whose loss against the
#: source is hint_gain_db (context.yaml) better than at the search's centre,
#: at rest; at least noise + HINT_SIGMAS σ, and at most HINT_UNDER_DETECT_DB
#: under the detection level (never more than the detection level itself:
#: light is light, however bright the centre already is). Confirmed at rest,
#: it becomes the new centre and the search starts again around it, from the
#: smallest disc.
#: Read on the move, light can show far below what it is: on the cell
#: (2026-11-06, ~0.1 nW of stray light), scans crossing 1-70 µW read only
#: 50-100 nW (the meter slow to leave its lowest range). A floor of a quarter
#: of the 0.5 µW detection level (6 dB under it) ignored them, and 9 runs of
#: 45 never found the light; 17 dB under it (1/50) takes them, and at rest
#: they show the light.
HINT_SIGMAS = 5.0
HINT_UNDER_DETECT_DB = 17.0
HINT_READINGS = 2
#: Once light shows, a scan goes on while it grows, and stops once it falls
#: under PAST_PEAK of its best: the top of the crossing, not its edge.
PAST_PEAK = 0.5
#: A scan crosses a capture diameter in at least this long (s): a meter slow
#: at low power still shows light it passes. The cell's meter takes a while
#: to leave its lowest range: crossing light faster, it reads µW as 50-100 nW
#: (2026-11-06). In simulation with such a meter (up after 0.2 s over its
#: range), the cell's starts found their light 40 times in 40 at 0.15 s,
#: twice at 0.05 s.
MIN_CROSSING_S = 0.15
#: Back at the pose of a reading taken on the move, a power at rest under
#: CONFIRM_FRACTION of that reading means the reading and its pose were out
#: of step: the meter lags (the light lies behind the pose along the path),
#: or the pose reported was old (ahead). The scan goes over BACK_CAPTURES
#: capture radii of its path around that pose, and at least what it covers
#: in BACK_LAG_S either side, backwards, BACK_SLOWDOWN times slower: out of
#: step the other way round, the backward reading and the forward one
#: bracket the light, and their weighted mean along the path is where it
#: is, whatever the delay.
CONFIRM_FRACTION = 0.5
#: At rest there, a power under the detection level is read again for up to
#: this long (s), the highest kept: a meter still in its lowest range reads
#: the top of that range until it switches up (on the cell, µW read as
#: 50-100 nW), and the light would pass for a mere hint.
RANGE_SETTLE_S = 0.3
BACK_CAPTURES = 4.0
BACK_LAG_S = 0.1
BACK_SLOWDOWN = 3.0
#: Path points a scan's reading may lie ahead of the one before: at the
#: scan's speed it reads every point or more often, a few slow readings
#: aside. Where it lies along the path, even where the path crosses itself.
TRACK_AHEAD = 12
#: Room the window must leave around the searches (mm, degrees).
WINDOW_MARGIN_MM = 0.1
ANGLE_MARGIN_DEG = 0.5
#: Points of the ring of rotation scans (acquire_ring_um), evenly around the start.
RING_POINTS = 6
#: The two axes each search moves along.
SEARCH_AXES = {"angle": ("rx", "rz"), "lateral": ("x", "z")}

# --- Center --------------------------------------------------------------------------------
#: A cross scan spans ± SCAN_HALF_WIDTHS widths, with SCAN_DENSITY readings per width.
SCAN_HALF_WIDTHS = 1.5
SCAN_DENSITY = 10
SCAN_SPACING_MIN_UM = 0.25
#: A fitted centre within this fraction of the span from its ends (or past
#: them) is at the edge: the window moves towards it, by at most a half-span,
#: at most EDGE_RESCANS times. A window without light doubles, at most
#: DARK_WIDENINGS times (TILT_DARK_WIDENINGS when re-centring after a tilt).
EDGE_FRACTION = 0.15
EDGE_RESCANS = 3
DARK_WIDENINGS = 3
TILT_DARK_WIDENINGS = 2
#: Centring stops once the centre moves less than max(CENTER_TOL_WIDTHS widths,
#: CENTER_TOL_MIN_UM), after CENTER_ITERATIONS at most. One: a second only
#: confirmed the first (2-4 µm on the cell, 71 runs), and Angle re-centres
#: both axes at the tilts it keeps.
CENTER_TOL_WIDTHS = 0.05
CENTER_TOL_MIN_UM = 0.5
CENTER_ITERATIONS = 1

# --- Angle -----------------------------------------------------------------------------------
#: The lateral axis a tilt walks the beam along.
TILT_WALK = {"rx": "z", "rz": "x"}
#: First tilt step, in angle widths (halved each round), and its growth while the power rises.
TILT_STEP_WIDTHS = 0.5
TILT_GROWTH = 1.5
#: Most tilts tried per line search.
TILT_MAX = 10
#: Rounds of rx then rz. A second round only if a search of the first was not
#: clean (no bracketed parabola, or the beam re-centred off the walk's
#: prediction): on the cell it gained 0.18 dB in 16 s, which Peak's circles
#: polish. Then rounds while one gains at least ANGLE_MIN_GAIN_DB.
ANGLE_ROUNDS = 3
ANGLE_MIN_GAIN_DB = 0.1
#: A tilt search is clean if the beam, re-centred at the tilt kept, is within
#: TILT_WALK_TOL_WIDTHS lateral widths of where the walk put it. The walk is
#: updated from that re-centring when the tilt moved at least
#: TILT_WALK_UPDATE_WIDTHS angle widths.
TILT_WALK_TOL_WIDTHS = 0.25
TILT_WALK_UPDATE_WIDTHS = 0.5

# --- Peak --------------------------------------------------------------------------------------
#: Gradient circles: radius in widths (lateral, or tilt): 36 µm and 0.03° on
#: the cell's ends, as the circles of its first script. Readings near the top
#: fall ~4 % (lateral) or ~2 % (tilt) around it: a clear slope, little loss.
GRADIENT_RADIUS_WIDTHS = 0.15
#: Moves queued per lap (blended into a circle), and readings per lap.
GRADIENT_POINTS = 36
GRADIENT_READINGS = 24
#: Lit readings a circle needs to fit its plane.
GRADIENT_MIN_READINGS = 8
#: A step goes at most this many widths: the slope gives the distance to the
#: top of a Gaussian at any distance, but the widths are measured, not exact.
GRADIENT_MAX_STEP_WIDTHS = 0.5
#: A step the other way from the previous one (of the same search) went past
#: the top: the steps shrink by this factor from then on. On the cell's faint
#: light of 2026-10-28, the top was ~2.5× sharper than its fitted width.
GRADIENT_OVERSHOOT_GAIN = 0.5
#: A step under this many times its noise (from the readings' scatter about
#: the plane: 1.5-2.7 % on the cell) is negligible, and not taken.
GRADIENT_NOISE_SIGMAS = 2.0
#: The rounds stop once both steps are under GRADIENT_MIN_STEP_WIDTHS widths
#: (and under PEAK_MIN_STEP_UM, or GRADIENT_MIN_STEP_DEG for a tilt).
GRADIENT_MIN_STEP_WIDTHS = 0.01
GRADIENT_MIN_STEP_DEG = 0.001
#: Pattern radius: PEAK_RADIUS_WIDTHS × the mean width, at least PEAK_RADIUS_MIN_UM.
#: Each step is clipped to it. At 0.2 width the edge points still read ~8 %
#: under the centre: plenty for the fit.
PEAK_RADIUS_WIDTHS = 0.2
PEAK_RADIUS_MIN_UM = 1.0
#: The search stops on a step under max(PEAK_MIN_STEP_UM, PEAK_MIN_STEP_WIDTHS widths)
#: (about the robot's resolution, or nothing that matters), a gain under PEAK_MIN_GAIN_DB,
#: or after PEAK_ITERATIONS.
PEAK_MIN_STEP_UM = 0.5
PEAK_MIN_STEP_WIDTHS = 0.01
PEAK_MIN_GAIN_DB = 0.05
PEAK_ITERATIONS = 6
#: Readings per side of the pattern (the peak_grid param): odd, so that the
#: centre is one of them, and within these bounds.
PEAK_GRID_BOUNDS = (3, 9)

#: Index of each axis in io.local's offsets.
LOCAL_INDEX = {"x": 0, "z": 2, "rx": 3, "rz": 5}


@dataclass
class _State:
    """What the phases learn and hand on, in io.local's coordinates."""

    settings: AlignmentSettings
    #: The best estimate of the optimum so far: x, z (µm), rx, rz (°).
    pos: Dict[str, float] = field(default_factory=lambda: {"x": 0.0, "z": 0.0, "rx": 0.0, "rz": 0.0})
    #: 1/e² radius (µm) of the power against x and z.
    width: Dict[str, float] = field(default_factory=dict)
    #: 1/e half-width (°) of the power against a tilt.
    angle_width: float = 0.0
    #: How far the beam walks along the lateral axis per degree of rx / rz
    #: (µm/°): remembered from the last run (io.walk), or once measured.
    walk: Dict[str, float] = field(default_factory=dict)
    #: The latest cross scan that found light: (axis, rx, rz it ran at, power, centre).
    last_cross: Optional[Tuple[str, float, float, float, float]] = None
    #: The power expected at pos (W).
    peak_w: float = 0.0
    #: The meter's reading without light (W), and the lowest reading counted as light.
    baseline_w: float = 0.0
    detect_w: float = 0.0
    #: Highest fixed range of the scans (W); raised if light overranges it.
    scan_range_w: float = 0.0
    # Meter and motion settings in force, so that only changes are sent.
    averaging: Optional[int] = None
    range_w: Optional[float] = -1.0
    motion: Dict[str, float] = field(default_factory=dict)

    def pose(self, io: AlignmentIO, **override: float) -> Pose6:
        p = {**self.pos, **override}
        return io.pose_at(x=p["x"], z=p["z"], rx=p["rx"], rz=p["rz"])


def align(io: AlignmentIO, settings: AlignmentSettings) -> None:
    if settings.acquire_radius_mm + WINDOW_MARGIN_MM > settings.range_mm:
        raise AlignmentError(
            f"acquire_radius_um ({settings.acquire_radius_mm * 1e3:g}) needs {WINDOW_MARGIN_MM * 1e3:g} µm of room "
            f"inside align_range_um ({settings.range_mm * 1e3:g}): lower it, or widen the window.")
    if settings.acquire_radius_deg + ANGLE_MARGIN_DEG > settings.range_deg:
        raise AlignmentError(
            f"acquire_radius_deg ({settings.acquire_radius_deg:g}) needs {ANGLE_MARGIN_DEG:g}° of room inside "
            f"align_range_deg ({settings.range_deg:g}): lower it, or widen the window.")
    if settings.acquire_ring_mm + WINDOW_MARGIN_MM > settings.range_mm:
        raise AlignmentError(
            f"acquire_ring_um ({settings.acquire_ring_mm * 1e3:g}) needs {WINDOW_MARGIN_MM * 1e3:g} µm of room "
            f"inside align_range_um ({settings.range_mm * 1e3:g}): lower it, or widen the window.")
    lo, hi = PEAK_GRID_BOUNDS
    if not (lo <= settings.peak_grid <= hi and settings.peak_grid % 2 == 1):
        raise AlignmentError(f"peak_grid ({settings.peak_grid}) must be odd, from {lo} to {hi}: the centre of "
                             f"the Peak pattern is one of its readings.")
    lateral = settings.lateral_width_mm * 1e3
    state = _State(settings, width={"x": lateral, "z": lateral}, angle_width=settings.angle_width_deg,
                   scan_range_w=SCAN_RANGE * settings.source_power_w)
    state.walk.update({axis: float(v) for axis, v in (io.walk or {}).items()
                       if axis in TILT_WALK and isinstance(v, (int, float)) and math.isfinite(v)})
    with io.phase("Acquire"):
        acquire(io, state)
    with io.phase("Center"):
        center(io, state)
    with io.phase("Angle"):
        angle(io, state)
    io.walk = dict(state.walk)
    with io.phase("Peak"):
        peak(io, state)


# --- Acquire -------------------------------------------------------------------------------

@dataclass
class _Search:
    """The first-light search in progress: what it heeds, and what it found so far."""

    #: The fields every "acquire" payload carries (noise, detection level, ...).
    info: Dict
    #: The power at rest at the search's centre (W): a hint's loss beats its own by gain_db.
    centre_w: float
    #: The lowest power a hint may have (W).
    hint_floor_w: float
    #: The detection level (W): a hint never needs more, light is light.
    detect_w: float
    #: How much better a hint's loss against the source is than the centre's (dB): hint_gain_db.
    gain_db: float = 3.0
    restarts: int = 0
    scans: int = 0

    def hint_w(self) -> float:
        """The power (W) a hint needs: a loss gain_db better than the centre's (the source cancels out), floored, capped."""
        return min(self.detect_w, max(self.hint_floor_w, self.centre_w * 10.0 ** (self.gain_db / 10.0)))


def acquire(io: AlignmentIO, state: _State) -> None:
    """The noise floor; unless light shows already, the search for it; ends on light, confirmed at rest.

    It starts at the pose remembered for this fiber (``io.seed``) when the
    scans around it fit in the window, else at the start. Dark there: rounds
    of Lissajous scans of growing discs (``search_rounds``), translation
    then rotation, restarting around any hint of light; dark at the
    remembered pose, the same around the start; still dark, rotation scans at
    the ring's points around the start. With ``full_search`` (demos), a
    translation then a rotation scan over the full radius run to their end
    first, light at the start or not.

    Every "acquire" payload says where its scan is centred ("around"):
    "remembered pose", "start", "hint k" or "ring k/n".
    """
    s = state.settings
    seed = _seed(io, state)
    if seed is not None:
        state.pos.update(seed)
        _goto(io, state)
    _averaging(io, state, SCAN_AVERAGING)
    _range(io, state, None)
    noise, sigma = fit.noise_stats([io.read_power("cart") for _ in range(NOISE_READS)])
    warm = math.isfinite(noise) and noise >= s.signal_detect_w
    if warm:
        # Light already: the noise floor is unknown, and negligible against it.
        state.baseline_w, state.detect_w = 0.0, s.signal_detect_w
    else:
        state.baseline_w = noise if math.isfinite(noise) else 0.0
        state.detect_w = max(s.signal_detect_w, noise + NOISE_SIGMAS * sigma) if math.isfinite(noise) else s.signal_detect_w
    info = {"noise_w": noise, "sigma_w": sigma, "detect_w": state.detect_w, "warm": warm, "full": s.full_search,
            "pattern": "lissajous", "hint_gain_db": s.hint_gain_db,
            "around": "start" if seed is None else "remembered pose"}
    if warm and not s.full_search:
        state.peak_w = noise
        io.publish("acquire", {**info, "search": None, "found": True,
                               "at": [round(state.pos["x"], 5), round(state.pos["z"], 5)], "power_w": noise})
        return
    floor = noise + HINT_SIGMAS * sigma if math.isfinite(noise) else 0.0
    search = _Search(info, centre_w=max(0.0, state.baseline_w),
                     hint_floor_w=max(state.detect_w * 10.0 ** (-HINT_UNDER_DETECT_DB / 10.0), floor),
                     detect_w=state.detect_w, gain_db=s.hint_gain_db)
    if s.full_search and full_search(io, state, search):
        return
    if search_rounds(io, state, search, info["around"]):
        return
    where = (f"tilting ±{s.acquire_radius_deg:g}° nor moving ±{s.acquire_radius_mm * 1e3:g} µm around the start"
             + ("" if seed is None else " and around the fiber's remembered pose"))
    if seed is not None:
        # The fiber is no longer where it aligned last: the same search around the start.
        state.pos.update(x=0.0, z=0.0, rx=0.0, rz=0.0)
        _goto(io, state)
        if search_rounds(io, state, search, "start"):
            return
    ring = _ring(state)
    for k, (x, z) in enumerate(ring, start=1):
        # Off sideways too far for the rotation scans at the start to see the light: the same, from here.
        state.pos.update(x=x, z=z, rx=0.0, rz=0.0)
        _goto(io, state)
        if search_rounds(io, state, search, f"ring {k}/{len(ring)}", searches=("angle",)):
            return
    if ring:
        where += f", nor tilting at {len(ring)} points {s.acquire_ring_mm * 1e3:g} µm around it"
    raise AlignmentNoSignal(
        f"No light above {state.detect_w:.3g} W (noise {noise:.3g} W) {where}: check the laser, the fibers "
        f"and the align_start pose.")


def search_rounds(io: AlignmentIO, state: _State, search: _Search, around: str,
                  searches: Tuple[str, ...] = SEARCH_ORDER) -> bool:
    """Scans of growing discs around pos (ACQUIRE_STAGES), the searches in turn, until light shows.

    Light, confirmed at rest: True, the fiber there. A hint, confirmed at
    rest: the new centre, and the scans start again from the smallest
    disc around it. Dark everywhere: False, the fiber back at its centre.
    """
    centre = dict(state.pos)
    stage = 0
    while stage < len(ACQUIRE_STAGES):
        restarted = False
        for name in searches:
            found = acquire_scan(io, state, search, name, ACQUIRE_STAGES[stage], around)
            if found == "light":
                return True
            if found == "hint":
                # Better than the centre: start again from here.
                search.restarts += 1
                centre, around = dict(state.pos), f"hint {search.restarts}"
                stage, restarted = 0, True
                break
            if state.pos != centre:
                state.pos.update(centre)
                _goto(io, state)
        if not restarted:
            stage += 1
    return False


def full_search(io: AlignmentIO, state: _State, search: _Search) -> bool:
    """For demos: a translation then a rotation scan over the full radius, each to its end; then the light they saw.

    True once that light is confirmed at rest. False otherwise: the fiber at
    the hint they saw, if any (a new centre), else back where it started.
    """
    best: Optional[Tuple[str, _Scan, Dict]] = None
    for name in SEARCH_ORDER:
        radius, capture, _ = _scale(state, name)
        amplitude = _within_window(state, name, radius)
        if amplitude <= 0:
            continue
        payload = _scan_payload(search, state, name, amplitude, search.info["around"])
        io.publish("acquire", {**payload, "found": False})
        scan = lissajous_scan(io, state, name, amplitude, capture, None)
        if scan.best is not None and (best is None or scan.best.power_w > best[1].best.power_w):
            best = (name, scan, payload)
    if best is None:
        return False
    name, scan, payload = best
    return _take(io, state, search, name, scan.best, scan.best_s, scan, payload) == "light"


def acquire_scan(io: AlignmentIO, state: _State, search: _Search, name: str, fraction: float,
                 around: str) -> Optional[str]:
    """One Lissajous scan of the search ``name`` around pos, over ``fraction`` of its radius, until a hint of light.

    Then the best it saw is confirmed at rest (``_take``). Returns "light",
    "hint" or None, as ``_take``.
    """
    radius, capture, _ = _scale(state, name)
    amplitude = _within_window(state, name, fraction * radius)
    if amplitude <= 0:
        return None
    payload = _scan_payload(search, state, name, amplitude, around)
    io.publish("acquire", {**payload, "found": False})
    scan = lissajous_scan(io, state, name, amplitude, capture, search.hint_w())
    if scan.trigger is not None:
        return _take(io, state, search, name, scan.trigger, scan.trigger_s, scan, payload)
    if scan.best is None:
        return None
    return _take(io, state, search, name, scan.best, scan.best_s, scan, payload)


def _take(io: AlignmentIO, state: _State, search: _Search, name: str, sample: Sample, s_out: float, scan: _Scan,
          payload: Dict) -> Optional[str]:
    """What a scan's ``sample`` (read ``s_out`` along its path) is worth, at rest (``_confirm``).

    "light": at least the detection level, the fiber there. "hint": at least
    a hint (``search.hint_w``), the fiber there, the search's new centre.
    None: less, or nothing to confirm; the fiber back where the scan was centred.
    """
    hint_w = search.hint_w()
    if not sample.power_w >= hint_w:
        return None
    axes = SEARCH_AXES[name]
    centre = dict(state.pos)
    power = _confirm(io, state, name, sample, s_out, scan, _scale(state, name)[1])
    if power >= state.detect_w:
        state.peak_w = power
        io.publish("acquire", {**payload, "found": True, "power_w": power,
                               "at": [round(state.pos[axes[0]], 5), round(state.pos[axes[1]], 5)]})
        return "light"
    if power >= hint_w:
        state.peak_w = search.centre_w = power
        return "hint"
    state.pos.update(centre)
    _goto(io, state)
    return None


def _scale(state: _State, name: str) -> Tuple[float, float, str]:
    """The search's radius, capture radius and unit: degrees for "angle", µm for "lateral"."""
    s = state.settings
    if name == "angle":
        return s.acquire_radius_deg, s.acquire_capture_deg, "°"
    return s.acquire_radius_mm * 1e3, s.acquire_capture_mm * 1e3, "µm"


def _scan_payload(search: _Search, state: _State, name: str, amplitude: float, around: str) -> Dict:
    """The "acquire" payload of a scan as it starts (counted in ``search.scans``)."""
    _, capture, unit = _scale(state, name)
    search.scans += 1
    return {**search.info, "search": name, "radius": round(amplitude, 5), "unit": unit, "around": around,
            "pitch": round(2 * LISSAJOUS_COVER * capture, 5), "spacing": round(LISSAJOUS_READING * capture, 5),
            "restarts": search.restarts}


def lissajous_path(centre: Tuple[float, float], amplitude: float, n: int, spacing: float) -> np.ndarray:
    """Points (u, v) of u = A sin(n t), v = A sin((n + 1) t) around ``centre``, t from 0 to 2π, ``spacing`` apart along it.

    It starts at the centre, crosses it again halfway (t = π) and ends there.
    The first half covers the square ± A already, about twice as coarsely.
    Points beyond A from the centre are brought back onto that circle (the
    corners of the square): the path stays in the disc of radius A, at most
    ``spacing`` between points, and covers it as well.
    """
    t = np.linspace(0.0, 2.0 * np.pi, max(2000, 400 * n))
    u, v = amplitude * np.sin(n * t), amplitude * np.sin((n + 1) * t)
    along = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(u), np.diff(v)))])
    at = np.interp(np.arange(0.0, along[-1], spacing).tolist() + [along[-1]], along, t)
    points = np.column_stack((amplitude * np.sin(n * at), amplitude * np.sin((n + 1) * at)))
    r = np.hypot(points[:, 0], points[:, 1])
    points *= np.minimum(1.0, amplitude / np.maximum(r, 1e-300))[:, None]
    return points + np.asarray(centre, dtype=float)


@dataclass
class _Scan:
    """A scan made: its path (in the search's axes), its speed, and its readings that count, with where they lie on it."""

    path: np.ndarray
    #: In the search's units per second.
    speed: float
    #: The best reading of the run that stopped it (None: it ran to its end), and its best reading.
    trigger: Optional[Sample] = None
    best: Optional[Sample] = None
    #: How far along the path they were read (the search's units).
    trigger_s: float = 0.0
    best_s: float = 0.0


def lissajous_scan(io: AlignmentIO, state: _State, name: str, amplitude: float, capture: float,
                   trigger_w: Optional[float]) -> _Scan:
    """A Lissajous over ± ``amplitude`` around pos, in the search's axes, read on the move.

    Sized from ``capture`` (how far from the optimum light shows): it passes
    within LISSAJOUS_COVER of it from every point, reads every
    LISSAJOUS_READING of it, and crosses it in at least MIN_CROSSING_S. It
    stops once HINT_READINGS readings in a row reach ``trigger_w`` (None:
    never). A full scan ends back at pos.
    """
    axes = SEARCH_AXES[name]
    n = max(2, math.ceil(LISSAJOUS_GAP * amplitude / (LISSAJOUS_COVER * capture)))
    spacing = LISSAJOUS_READING * capture
    path = lissajous_path((state.pos[axes[0]], state.pos[axes[1]]), amplitude, n, spacing)
    return _run_path(io, state, name, path, _search_speed(io, name, spacing, capture), trigger_w)


def _search_speed(io: AlignmentIO, name: str, spacing: float, capture: float) -> float:
    """The scan speed (the search's units per second): a reading every ``spacing``, a capture diameter in MIN_CROSSING_S at least."""
    return min(spacing / io.sample_period_s, 2.0 * capture / MIN_CROSSING_S)


def _run_path(io: AlignmentIO, state: _State, name: str, path: np.ndarray, speed: float,
              trigger_w: Optional[float]) -> _Scan:
    """Move along ``path`` (from pos, in the search's axes) at ``speed``, reading all along.

    Once HINT_READINGS readings in a row reach ``trigger_w`` (None: never),
    it goes on while the light grows, and stops once it falls under
    PAST_PEAK of its best. The scan's trigger is the best reading of that
    run (None if it ran to its end).
    """
    axes = SEARCH_AXES[name]
    kind = "ang" if name == "angle" else "cart"
    # Scan readings, in auto range: the light may be anywhere from nothing to the peak.
    _averaging(io, state, SCAN_AVERAGING)
    _range(io, state, None)
    if name == "angle":
        _motion(io, state, acc=MOVE_MOTION["acc"], ang_vel=max(MIN_SCAN_ANG_SPEED, speed), blending=100)
    else:
        _motion(io, state, acc=MOVE_MOTION["acc"], lin_vel=max(MIN_SCAN_SPEED, speed * 1e-3), blending=100)
    seen: Dict[str, Optional[Tuple[Sample, float]]] = {"best": None, "run": None, "trigger": None}
    count = {"run": 0}
    along = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(path, axis=0).T))])
    track = {"i": 0}

    def locate(sample: Sample) -> float:
        # How far along the path it was read: on the segments just ahead of
        # the reading before, not on another pass of a path that crosses itself.
        offsets = io.local(sample.pose)
        point = np.array([offsets[LOCAL_INDEX[axes[0]]], offsets[LOCAL_INDEX[axes[1]]]])
        lo = max(0, track["i"] - 2)
        a, b = path[lo:lo + TRACK_AHEAD + 3][:-1], path[lo + 1:lo + TRACK_AHEAD + 3]
        ab = b - a
        t = np.clip(np.einsum("ij,ij->i", point - a, ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-18), 0, 1)
        k = int(np.argmin(np.hypot(*(a + t[:, None] * ab - point).T)))
        track["i"] = lo + k
        return float(along[lo + k] + t[k] * (along[lo + k + 1] - along[lo + k]))

    def watch(samples: List[Sample]) -> bool:
        sample = samples[-1]
        here = (sample, locate(sample))
        if math.isnan(sample.power_w):
            return False
        if seen["best"] is None or sample.power_w > seen["best"][0].power_w:
            seen["best"] = here
        if trigger_w is None:
            return False
        # An overrange (+inf) is light too.
        if sample.power_w >= trigger_w:
            count["run"] += 1
            if seen["run"] is None or sample.power_w > seen["run"][0].power_w:
                seen["run"] = here
        elif count["run"] < HINT_READINGS:
            count["run"], seen["run"] = 0, None  # a blip: forget it
        if count["run"] >= HINT_READINGS and sample.power_w < PAST_PEAK * seen["run"][0].power_w:
            seen["trigger"] = seen["run"]
            return True
        return False

    # The path starts where the fiber is.
    for first in range(1, len(path), SCAN_CHUNK):
        targets = [state.pose(io, **{axes[0]: u, axes[1]: v}) for u, v in path[first:first + SCAN_CHUNK]]
        io.scan(targets, kind=kind, stop=watch)
        if seen["trigger"] is not None:
            break
    if seen["trigger"] is None and count["run"] >= HINT_READINGS:
        seen["trigger"] = seen["run"]  # light still growing where the path ended
    scan = _Scan(path, speed)
    if seen["trigger"] is not None:
        scan.trigger, scan.trigger_s = seen["trigger"]
    if seen["best"] is not None:
        scan.best, scan.best_s = seen["best"]
    return scan


def _confirm(io: AlignmentIO, state: _State, name: str, sample: Sample, s_out: float, scan: _Scan,
             capture: float) -> float:
    """Back to where ``sample`` was read (``s_out`` along ``scan``'s path), the power at rest there; pos follows.

    Read on the move, light may show out of step with its pose (the meter
    lags, or the pose was read late): back at the pose of the reading, the
    fiber may sit beside the light, which lies on the path behind or ahead.
    Under CONFIRM_FRACTION of the reading, the scan goes back over that
    stretch of its path (``_back_along``) to where the light is, if that
    reads more at rest.
    """
    axes = SEARCH_AXES[name]
    kind = "ang" if name == "angle" else "cart"
    offsets = io.local(sample.pose)
    state.pos.update({axis: offsets[LOCAL_INDEX[axis]] for axis in axes})
    _goto(io, state)
    power = _settled_still(io, state, kind)
    if power >= CONFIRM_FRACTION * sample.power_w:
        return power
    here = dict(state.pos)
    found = _back_along(io, state, name, s_out, scan, capture)
    if found is not None:
        state.pos.update({axes[0]: found[0], axes[1]: found[1]})
        _goto(io, state)
        again = _settled_still(io, state, kind)
        if again > power:
            return again
    state.pos.update(here)
    _goto(io, state)
    return power


def _back_along(io: AlignmentIO, state: _State, name: str, s_out: float, scan: _Scan,
                capture: float) -> Optional[Tuple[float, float]]:
    """Where along ``scan``'s path the light that a reading ``s_out`` along it showed out of step really is, or None.

    Over BACK_CAPTURES capture radii of the path centred on the reading (at
    least BACK_LAG_S of the scan either side), backwards, BACK_SLOWDOWN
    times slower. A reading out of step by a time τ
    (late: τ > 0, its pose reported old: τ < 0) lands a speed × τ along the
    way it was taken: s_out = s + v τ on the way out, s_back = s - v τ / k on
    the way back. Hence s = (s_out + k · s_back) / (1 + k), whatever τ.
    """
    axes = SEARCH_AXES[name]
    path = scan.path
    along = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(path, axis=0).T))])
    half = max(0.5 * BACK_CAPTURES * capture, BACK_LAG_S * scan.speed)
    first = max(0, int(np.searchsorted(along, s_out - half)) - 1)
    last = min(len(path), int(np.searchsorted(along, s_out + half)) + 1)
    stretch = path[first:last][::-1]
    if len(stretch) < 2:
        return None
    state.pos.update({axes[0]: float(stretch[0][0]), axes[1]: float(stretch[0][1])})
    _goto(io, state)
    back = _run_path(io, state, name, stretch, scan.speed / BACK_SLOWDOWN, None)
    if back.best is None:
        return None
    s_back = along[last - 1] - back.best_s  # the stretch runs the path backwards from its point last - 1
    s = (s_out + BACK_SLOWDOWN * s_back) / (1.0 + BACK_SLOWDOWN)
    return float(np.interp(s, along, path[:, 0])), float(np.interp(s, along, path[:, 1]))


def _within_window(state: _State, name: str, amplitude: float) -> float:
    """``amplitude``, reduced so that a scan's disc around pos stays inside the window, its margins left."""
    s = state.settings
    if name == "angle":
        room = s.range_deg - ANGLE_MARGIN_DEG - math.hypot(state.pos["rx"], state.pos["rz"])
    else:
        room = (s.range_mm - WINDOW_MARGIN_MM) * 1e3 - math.hypot(state.pos["x"], state.pos["z"])
    return max(0.0, min(amplitude, room))


def _seed(io: AlignmentIO, state: _State) -> Optional[Dict[str, float]]:
    """The pose remembered for this fiber, as pos (x, z, rx, rz), or None.

    None too when the scans around it would leave the window (align_start
    taught again far from it). Its offset along the fiber (y) is not used:
    the gap stays the one align_start taught.
    """
    if io.seed is None:
        return None
    s = state.settings
    x, _, z, rx, _, rz = io.seed
    lateral_room = (s.range_mm - WINDOW_MARGIN_MM - s.acquire_radius_mm) * 1e3
    angle_room = s.range_deg - ANGLE_MARGIN_DEG - s.acquire_radius_deg
    if math.hypot(x, z) > lateral_room or math.hypot(rx, rz) > angle_room:
        return None
    return {"x": x, "z": z, "rx": rx, "rz": rz}


def _ring(state: _State) -> List[Tuple[float, float]]:
    """The lateral points (x, z in µm) of the ring of rotation scans, around the start: none if acquire_ring_um is 0."""
    radius = state.settings.acquire_ring_mm * 1e3
    if radius <= 0:
        return []
    return [(radius * math.cos(2 * math.pi * k / RING_POINTS), radius * math.sin(2 * math.pi * k / RING_POINTS))
            for k in range(RING_POINTS)]


# --- Center --------------------------------------------------------------------------------

def center(io: AlignmentIO, state: _State) -> None:
    """Cross scans along x and z until the fitted centre stops moving; ends there, at rest."""
    for iteration in range(CENTER_ITERATIONS):
        x0, z0 = state.pos["x"], state.pos["z"]
        for axis in ("x", "z"):
            if not cross_scan(io, state, axis, iteration):
                raise AlignmentNoSignal(f"Lost the light while centring along {axis}: no reading above "
                                        f"{state.detect_w:.3g} W. Check the laser and the fibers.")
        tolerance = max(CENTER_TOL_MIN_UM, CENTER_TOL_WIDTHS * min(state.width.values()))
        if math.hypot(state.pos["x"] - x0, state.pos["z"] - z0) < tolerance:
            break
    _goto(io, state)
    state.peak_w = _still(io, state, "cart")


def cross_scan(io: AlignmentIO, state: _State, axis: str, iteration: int,
               widenings: int = DARK_WIDENINGS) -> bool:
    """Scan across the light along ``axis`` ("x" or "z") both ways, and move pos[axis] to its centre.

    Each direction's readings are fitted with a Gaussian over the noise floor
    (exact in log power, so the centre and the width come out right even from
    a window narrower than the beam). Their centres differ by twice what
    remains of the reading/pose lag ("lag" in the profile), and their mean is
    free of it. A centre at or past the window's edge moves the window towards
    it by at most a half-span, and scans again; if no window catches the
    centre, the fiber goes to the best reading, never to an extrapolated
    centre. The width is updated from a centre caught inside a window only.
    A window without light doubles around the same middle, ``widenings``
    times at most.

    Returns False when no reading reaches the detection level (pos unchanged):
    no light along this line.
    """
    width = state.width[axis]
    half = SCAN_HALF_WIDTHS * width
    spacing = max(width / SCAN_DENSITY, SCAN_SPACING_MIN_UM)
    limit = _lateral_limit(state, axis)
    middle = state.pos[axis]
    best: Optional[Tuple[float, float]] = None  # (coordinate, power) of the best reading of all windows
    caught: Optional[fit.Gauss] = None
    dark = 0
    for attempt in range(EDGE_RESCANS + widenings + 1):
        lo, hi = max(middle - half, -limit), min(middle + half, limit)
        passes = _scan_both_ways(io, state, axis, lo, hi, spacing)
        points = [pt for _, pts in passes for pt in pts if math.isfinite(pt[1])]
        top = max(points, key=lambda pt: pt[1]) if points else None
        if top is not None and (best is None or top[1] > best[1]):
            best = top
        light = top is not None and top[1] >= state.detect_w
        found, lag = _fit_passes(passes, state.baseline_w) if light else (None, None)
        inside = found is not None and lo + EDGE_FRACTION * (hi - lo) <= found.center <= hi - EDGE_FRACTION * (hi - lo)
        io.publish("profile", {
            "axis": axis,
            "iteration": iteration,
            "rescan": attempt > 0,
            "unit": "µm",
            "span": [round(lo, 3), round(hi, 3)],
            "passes": [{"dir": d, "points": [[round(c, 3), p] for c, p in pts]} for d, pts in passes],
            "fit": None if found is None else {
                "model": "gauss", "center": round(found.center, 3), "width": round(found.width, 3),
                "peak": found.top, "baseline": found.baseline, "lag": None if lag is None else round(lag, 3),
            },
            "chosen": round(found.center, 3) if inside else None,
        })
        if inside:
            caught = found
            break
        if not light:
            if best is not None and best[1] >= state.detect_w:
                break  # an earlier window had the light: settle there
            dark += 1
            if dark > widenings:
                break
            half *= 2  # no light here: a wider window around the same middle
            continue
        if attempt - dark >= EDGE_RESCANS:
            break
        # Towards the fitted centre (else the best reading), by at most a half-span.
        target = found.center if found is not None else top[0]
        middle += max(-half, min(half, target - middle))
    state.last_cross = None
    if caught is not None:
        state.pos[axis] = caught.center
        state.width[axis] = _new_width(state.width[axis], caught.width, LATERAL_WIDTH_BOUNDS_UM)
        state.peak_w = caught.top
    elif best is not None and best[1] >= state.detect_w:
        state.pos[axis] = best[0]
        state.peak_w = best[1]
    else:
        state.peak_w = best[1] if best is not None else 0.0
        return False
    state.last_cross = (axis, state.pos["rx"], state.pos["rz"], state.peak_w, state.pos[axis])
    return True


def _scan_both_ways(io: AlignmentIO, state: _State, axis: str, lo: float, hi: float,
                    spacing: float) -> List[Tuple[int, List[Tuple[float, float]]]]:
    """Readings (coordinate, power) from lo to hi, then back, at a fixed meter range (_scan_range); raised on an overrange.

    One motion, lo → hi → lo (it stops at hi: no blending), split where the
    coordinate turns: one scan to queue instead of two.
    """
    index = LOCAL_INDEX[axis]
    range_w = _scan_range(state)
    while True:
        _averaging(io, state, SCAN_AVERAGING)
        _range(io, state, range_w)
        _goto(io, state, **{axis: lo})
        _motion(io, state, lin_vel=_scan_speed(io, spacing), blending=0)
        samples = io.scan([state.pose(io, **{axis: hi}), state.pose(io, **{axis: lo})], kind="cart")
        points = [(io.local(s.pose)[index], s.power_w) for s in samples]
        turn = int(np.argmax([c for c, _ in points])) + 1 if points else 0
        passes = [(1, points[:turn]), (-1, points[turn:])]
        if not any(math.isinf(p) for _, pts in passes for _, p in pts):
            return passes
        range_w = _raise_range(state, range_w)


def _fit_passes(passes, baseline: float) -> Tuple[Optional[fit.Gauss], Optional[float]]:
    """A Gaussian per direction and their mean (with the lag between them), else one through all readings."""
    fits = [fit.gauss1d(*zip(*pts), baseline=baseline) if len(pts) >= 4 else None for _, pts in passes]
    if len(fits) == 2 and all(fits):
        forward, backward = fits
        return fit.Gauss(center=0.5 * (forward.center + backward.center),
                         width=0.5 * (forward.width + backward.width),
                         peak=0.5 * (forward.peak + backward.peak),
                         baseline=baseline), 0.5 * (forward.center - backward.center)
    everything = [pt for _, pts in passes for pt in pts]
    return (fit.gauss1d(*zip(*everything), baseline=baseline) if len(everything) >= 4 else None), None


# --- Angle ----------------------------------------------------------------------------------

def angle(io: AlignmentIO, state: _State) -> None:
    """Tilt line searches in rx then rz, re-centred as the tilt walks the beam; again, finer, if the first round was not clean, and while it gains."""
    for round_ in range(ANGLE_ROUNDS):
        before = state.peak_w
        clean = [tilt_search(io, state, axis, round_) for axis in ("rx", "rz")]
        if round_ == 0 and all(clean):
            break  # Peak's gradient circles polish what is left
        if _db(state.peak_w, before) < ANGLE_MIN_GAIN_DB:
            break
    _goto(io, state)
    state.peak_w = _still(io, state, "cart")


def tilt_search(io: AlignmentIO, state: _State, axis: str, round_: int) -> bool:
    """Step the tilt away from pos[axis] while the power rises; keep the tilt a parabola through the best picks.

    With the walk known (state.walk: remembered, or measured earlier in the
    run), each tilt is read at rest where the walk puts the beam, the
    lateral position carried from where the search starts. Else, at each
    tilt, the fiber is re-centred along the lateral axis the tilt walks the
    beam on (the latest cross scan stands for the starting tilt if it was
    along that axis, there), and the power there counts; the walk is
    measured from those centres. A tilt where no light shows counts as no
    power: it ends that direction. At the tilt kept, the fiber is re-centred
    along both lateral axes, and a known walk is corrected from where the
    beam really was.

    Returns whether the search was clean: the tilt kept from a parabola with
    a top between its points, and the beam re-centred within
    TILT_WALK_TOL_WIDTHS widths of where the walk put it.
    """
    walk = TILT_WALK[axis]
    other = "x" if walk == "z" else "z"
    step = TILT_STEP_WIDTHS * state.angle_width * 0.5 ** round_
    limit = _tilt_limit(state, axis)
    tried: Dict[float, Tuple[float, float]] = {}  # tilt -> (power, centre along `walk`)
    known = axis in state.walk
    start = round(state.pos[axis], 6)
    centre0 = state.pos[walk]

    def lit() -> List[float]:
        return sorted(t for t, (p, _) in tried.items() if p >= state.detect_w)

    def walk_slope() -> float:
        # µm per degree: from this search's lit tilts, else an earlier search, else none yet.
        tilts = lit()
        if len(tilts) >= 2:
            return float(np.polyfit(tilts, [tried[t][1] for t in tilts], 1)[0])
        return state.walk.get(axis, 0.0)

    def measure(tilt: float) -> float:
        tilt = round(min(max(tilt, -limit), limit), 6)
        if tilt in tried:
            return tried[tilt][0]
        if known:
            # At rest, where the walk puts the beam at this tilt.
            state.pos[axis] = tilt
            state.pos[walk] = centre0 + state.walk[axis] * (tilt - start)
            _goto(io, state)
            tried[tilt] = (_still(io, state, "ang"), state.pos[walk])
            return tried[tilt][0]
        last = state.last_cross
        if (tilt == start and last is not None and last[0] == walk
                and abs(last[1] - state.pos["rx"]) < 1e-9 and abs(last[2] - state.pos["rz"]) < 1e-9):
            tried[tilt] = (last[3], last[4])  # just scanned there: no need to again
            return tried[tilt][0]
        # Where the beam should land at this tilt: from the nearest lit tilt, along the walk.
        tilts = lit()
        if tilts:
            near = min(tilts, key=lambda t: abs(t - tilt))
            state.pos[walk] = tried[near][1] + walk_slope() * (tilt - near)
        state.pos[axis] = tilt
        cross_scan(io, state, walk, round_, widenings=TILT_DARK_WIDENINGS)
        tried[tilt] = (state.peak_w, state.pos[walk])
        return tried[tilt][0]

    current = measure(start)
    for direction in (1, -1):
        tilt, power, size = start, current, step
        while len(tried) < TILT_MAX:
            nxt = round(min(max(tilt + direction * size, -limit), limit), 6)
            if nxt == tilt:
                break
            p = measure(nxt)
            if not p > power:
                break
            tilt, power, size = nxt, p, size * TILT_GROWTH
        if direction == 1 and len(tried) > 1 and max(tried.values())[0] > current:
            break  # the power rose this way: the other side cannot hold the peak
    tilts = sorted(tried)
    powers = [tried[t][0] for t in tilts]
    i = int(np.argmax(powers))
    parabola = None
    if 0 < i < len(tilts) - 1 and all(p >= state.detect_w for p in powers[i - 1:i + 2]):
        parabola = fit.parabola1d(tilts[i - 1:i + 2], powers[i - 1:i + 2])
    bracketed = parabola is not None and parabola.curvature < 0 and math.isfinite(parabola.vertex)
    if bracketed:
        chosen = min(max(parabola.vertex, tilts[i - 1]), tilts[i + 1])
        state.angle_width = _new_width(state.angle_width, parabola.width, ANGLE_WIDTH_BOUNDS_DEG)
    else:
        chosen = tilts[i]
    # Start the re-centring where the walk puts the centre at the tilt kept, then scan both lateral axes.
    if known:
        predicted = centre0 + state.walk[axis] * (chosen - start)
    else:
        lit_tilts = lit() or tilts
        slope = walk_slope() if len(lit()) >= 2 else None
        if slope is not None:
            state.walk[axis] = slope
        predicted = float(np.interp(chosen, lit_tilts, [tried[t][1] for t in lit_tilts]))
    state.pos[axis] = chosen
    state.pos[walk] = predicted
    centred = all([cross_scan(io, state, lateral, round_) for lateral in (walk, other)])
    off = abs(state.pos[walk] - predicted)
    if known and centred and abs(chosen - start) >= TILT_WALK_UPDATE_WIDTHS * state.angle_width:
        # Where the beam really was at the tilt kept: the walk, corrected.
        state.walk[axis] = (state.pos[walk] - centre0) / (chosen - start)
    clean = bracketed and centred and off <= TILT_WALK_TOL_WIDTHS * state.width[walk]
    # The walk (µm/°) is in effect the distance between the fiber ends (or from the TCP to the tip).
    io.publish("profile", {
        "axis": axis,
        "iteration": round_,
        "rescan": False,
        "unit": "°",
        "span": [tilts[0], tilts[-1]] if len(tilts) > 1 else [tilts[0] - step, tilts[0] + step],
        "passes": [{"dir": 1, "points": [[t, p] for t, p in zip(tilts, powers)]}],
        "fit": None if not bracketed else {
            "model": "parabola", "center": round(parabola.vertex, 5), "width": round(parabola.width, 5),
            "peak": parabola.peak,
        },
        "chosen": round(chosen, 5),
        "walk_um_per_deg": round(state.walk[axis], 1) if axis in state.walk else None,
        "at_rest": known,
        "clean": clean,
    })
    return clean


# --- Peak -----------------------------------------------------------------------------------

def pattern(n: int) -> List[Tuple[float, float]]:
    """An n×n grid in pattern radii (-1 to 1), row by row, each row the other way round: no long moves."""
    steps = [-1.0 + 2.0 * i / (n - 1) for i in range(n)]
    return [(u, v) for j, v in enumerate(steps) for u in (steps if j % 2 == 0 else steps[::-1])]


def peak(io: AlignmentIO, state: _State) -> None:
    """Gradient circles (``gradient``), then Newton steps on a quadratic fit of log power over a square pattern
    at rest; ends on the best reading at rest.

    The pattern has peak_grid readings per side and spans ± the radius.
    """
    gradient(io, state)
    mean_width = 0.5 * (state.width["x"] + state.width["z"])
    radius = max(PEAK_RADIUS_MIN_UM, PEAK_RADIUS_WIDTHS * mean_width)
    min_step = max(PEAK_MIN_STEP_UM, PEAK_MIN_STEP_WIDTHS * mean_width)
    grid = pattern(state.settings.peak_grid)
    _range(io, state, None)
    previous = None
    for iteration in range(PEAK_ITERATIONS):
        cx, cz = state.pos["x"], state.pos["z"]
        readings = []
        for u, v in grid:
            _goto(io, state, x=cx + u * radius, z=cz + v * radius)
            readings.append((u * radius, v * radius, _still(io, state, "cart")))
        middle = readings[grid.index((0.0, 0.0))][2]
        (du, dz), newton = fit.quad2d_step(readings, radius)
        step = math.hypot(du, dz)
        io.publish("peak", {
            "iter": iteration,
            "center": [round(cx, 3), round(cz, 3)],
            "radius_um": round(radius, 3),
            "step": [round(du, 3), round(dz, 3)],
            "step_um": round(step, 3),
            "step_widths": round(step / mean_width, 5),
            "newton": newton,
            "power_w": middle,
            "points": [[round(cx + u, 3), round(cz + v, 3), p] for u, v, p in readings],
        })
        state.pos.update(x=cx + du, z=cz + dz)
        gain = _db(middle, previous) if previous else math.inf
        previous = middle
        if step < min_step or gain < PEAK_MIN_GAIN_DB:
            break
    _goto(io, state)
    state.peak_w = _still(io, state, "cart")
    best = io.best_still
    if best is not None and best.power_w > state.peak_w:
        _motion(io, state, **MOVE_MOTION)
        io.move_to(best.pose)


def gradient(io: AlignmentIO, state: _State) -> None:
    """Rounds of a lateral circle then a tilt circle around pos (``circle``), each stepping pos up its slope.

    The circles of the cell's first script (looping_circles). The rounds stop
    once neither circle takes a step, after gradient_rounds, or on a circle
    that cannot run. If the power at rest is then lower than before them, pos
    goes back to where they started.
    """
    rounds = state.settings.gradient_rounds
    if rounds <= 0:
        return
    start, before = dict(state.pos), state.peak_w
    climbs = [_Climb("lateral"), _Climb("angle")]
    for round_ in range(rounds):
        moved = []
        for climb in climbs:
            moved.append(circle(io, state, climb, round_))
            if moved[-1] is None:
                break
        if None in moved or not any(moved):
            break
    _range(io, state, None)
    _goto(io, state)
    power = _still(io, state, "cart")
    if power >= before:
        state.peak_w = power
    else:
        # A slope misread (noise, a drift of the source): the pattern starts where the circles did.
        state.pos.update(start)


@dataclass
class _Climb:
    """One search of the gradient circles ("lateral" or "angle"), from round to round."""

    search: str
    #: The fraction of the Newton step taken: GRADIENT_OVERSHOOT_GAIN less after each overshoot.
    gain: float = 1.0
    #: The latest step taken, in the circle's two axes.
    step: Tuple[float, float] = (0.0, 0.0)


def circle(io: AlignmentIO, state: _State, climb: _Climb, round_: int) -> Optional[bool]:
    """A circle around pos in the lateral plane ("lateral": x/z) or in tilt ("angle": rx/rz), read on the move.

    It runs once each way, so that the reading/pose lag, and a drift of the
    source, cancel in the fit. A plane fitted to ln P over its readings gives
    the slope; on a Gaussian, the top is the slope × width² / 4 away (1/e²
    radius, lateral) or × width² / 2 (1/e half-width, tilt). pos steps that
    far times ``climb.gain``, halved whenever a step goes back on the
    previous one, at most GRADIENT_MAX_STEP_WIDTHS. A step under
    max(the minimum step, GRADIENT_NOISE_SIGMAS × its noise) is not taken. A
    tilt circle carries the lateral position with the tilt, by the walk the
    Angle phase measured: it follows the ridge where the beam stays centred,
    as tilt_search does, instead of crossing it.

    True if pos moved, False if the step was negligible, None if the circle
    cannot run (it would leave the window, or has too little light). pos is
    unchanged unless it moved.
    """
    search = climb.search
    if search == "lateral":
        axes, unit, kind = ("x", "z"), "µm", "cart"
        widths = (state.width["x"], state.width["z"])
        newton = tuple(w * w / 4 for w in widths)
        min_step = max(PEAK_MIN_STEP_UM, GRADIENT_MIN_STEP_WIDTHS * min(widths))
    else:
        axes, unit, kind = ("rx", "rz"), "°", "ang"
        widths = (state.angle_width, state.angle_width)
        newton = tuple(w * w / 2 for w in widths)
        min_step = max(GRADIENT_MIN_STEP_DEG, GRADIENT_MIN_STEP_WIDTHS * state.angle_width)
    radius = GRADIENT_RADIUS_WIDTHS * 0.5 * (widths[0] + widths[1])
    centre = dict(state.pos)

    def moved_by(du: float, dv: float) -> Dict[str, float]:
        p = {axes[0]: centre[axes[0]] + du, axes[1]: centre[axes[1]] + dv}
        for axis, d in zip(axes, (du, dv)):
            if axis in TILT_WALK:
                p[TILT_WALK[axis]] = centre[TILT_WALK[axis]] + state.walk.get(axis, 0.0) * d
        return p

    path = [moved_by(radius * math.cos(a), radius * math.sin(a))
            for a in (2 * math.pi * k / GRADIENT_POINTS for k in range(GRADIENT_POINTS + 1))]
    if not all(_inside(state, {**centre, **p}) for p in path):
        return None
    spacing = 2 * math.pi * radius / GRADIENT_READINGS
    range_w = _scan_range(state)
    while True:
        _averaging(io, state, SCAN_AVERAGING)
        _range(io, state, range_w)
        _goto(io, state, **path[0])
        if search == "lateral":
            _motion(io, state, lin_vel=_scan_speed(io, spacing), blending=100)
        else:
            _motion(io, state, ang_vel=_scan_ang_speed(io, spacing), blending=100)
        samples = []
        for lap in (path[1:], path[-2::-1]):
            samples += io.scan([state.pose(io, **p) for p in lap], kind=kind)
        if not any(math.isinf(s.power_w) for s in samples):
            break
        range_w = _raise_range(state, range_w)
    readings = []
    for s in samples:
        if math.isfinite(s.power_w) and s.power_w >= state.detect_w:
            offsets = io.local(s.pose)
            readings.append((offsets[LOCAL_INDEX[axes[0]]] - centre[axes[0]],
                             offsets[LOCAL_INDEX[axes[1]]] - centre[axes[1]], s.power_w))
    fitted = fit.plane_slope(readings) if len(readings) >= GRADIENT_MIN_READINGS else None
    step, noise, noise_widths, taken = (0.0, 0.0), 0.0, 0.0, False
    if fitted is not None:
        (gu, gv), (eu, ev) = fitted
        full = (gu * newton[0], gv * newton[1])
        if full[0] * climb.step[0] + full[1] * climb.step[1] < 0:
            climb.gain *= GRADIENT_OVERSHOOT_GAIN  # the previous step went past the top
        step = (climb.gain * full[0], climb.gain * full[1])
        noise = climb.gain * math.hypot(eu * newton[0], ev * newton[1])
        noise_widths = climb.gain * math.hypot(eu * newton[0] / widths[0], ev * newton[1] / widths[1])
        in_widths = math.hypot(step[0] / widths[0], step[1] / widths[1])
        if in_widths > GRADIENT_MAX_STEP_WIDTHS:
            step = (step[0] * GRADIENT_MAX_STEP_WIDTHS / in_widths, step[1] * GRADIENT_MAX_STEP_WIDTHS / in_widths)
        taken = math.hypot(*step) >= max(min_step, GRADIENT_NOISE_SIGMAS * noise)
    if not taken:
        step = (0.0, 0.0)
    io.publish("gradient", {
        "round": round_,
        "search": search,
        "unit": unit,
        "center": [round(centre[axes[0]], 5), round(centre[axes[1]], 5)],
        "radius": round(radius, 5),
        "readings": len(readings),
        "power_w": float(np.mean([p for _, _, p in readings])) if readings else None,
        "step": [round(step[0], 5), round(step[1], 5)],
        "step_widths": round(math.hypot(step[0] / widths[0], step[1] / widths[1]), 5),
        "noise_widths": round(noise_widths, 5),
        "gain": climb.gain,
    })
    if fitted is None:
        return None
    if taken:
        state.pos.update(moved_by(*step))
        climb.step = step
    return taken


# --- Helpers ---------------------------------------------------------------------------------

def _goto(io: AlignmentIO, state: _State, **override: float) -> None:
    """Point-to-point to pos, with ``override`` coordinates."""
    _motion(io, state, **MOVE_MOTION)
    io.move_to(state.pose(io, **override))


def _still(io: AlignmentIO, state: _State, kind: str) -> float:
    """A reading at rest, averaged. A scan's fixed range may be too low for it: then in auto range."""
    _averaging(io, state, STILL_AVERAGING)
    power = io.read_still(kind)
    if math.isinf(power):
        _range(io, state, None)
        power = io.read_still(kind)
    return power


def _settled_still(io: AlignmentIO, state: _State, kind: str) -> float:
    """A reading at rest (``_still``); under the detection level, read again for up to RANGE_SETTLE_S, the highest kept."""
    power = _still(io, state, kind)
    until = io.elapsed_s() + RANGE_SETTLE_S
    while power < state.detect_w and io.elapsed_s() < until:
        power = max(power, _still(io, state, kind))
    return power


def _new_width(old: float, fitted: float, bounds: Tuple[float, float]) -> float:
    """A fitted width, changed from ``old`` by at most WIDTH_CHANGE either way, within ``bounds``."""
    if not math.isfinite(fitted) or fitted <= 0:
        return old
    changed = min(max(fitted, old / WIDTH_CHANGE), old * WIDTH_CHANGE)
    return min(max(changed, bounds[0]), bounds[1])


def _lateral_limit(state: _State, axis: str) -> float:
    """How far along ``axis`` (µm) the window lets the fiber go, the other lateral axis where it is."""
    other = state.pos["z" if axis == "x" else "x"]
    room = state.settings.range_mm * 1e3 - WINDOW_MARGIN_MM * 1e3
    return math.sqrt(max(room * room - other * other, 0.0))


def _tilt_limit(state: _State, axis: str) -> float:
    """How far in ``axis`` (°) the window lets the fiber tilt, the other tilt where it is."""
    other = state.pos["rz" if axis == "rx" else "rx"]
    room = state.settings.range_deg - ANGLE_MARGIN_DEG
    return math.sqrt(max(room * room - other * other, 0.0))


def _scan_speed(io: AlignmentIO, spacing_um: float) -> float:
    """Scan speed (mm/s) that leaves ``spacing_um`` between two readings."""
    return max(MIN_SCAN_SPEED, spacing_um * 1e-3 / io.sample_period_s)


def _scan_ang_speed(io: AlignmentIO, spacing_deg: float) -> float:
    """Angular scan speed (°/s) that leaves ``spacing_deg`` between two readings."""
    return max(MIN_SCAN_ANG_SPEED, spacing_deg / io.sample_period_s)


def _motion(io: AlignmentIO, state: _State, **motion: float) -> None:
    changed = {k: v for k, v in motion.items() if state.motion.get(k) != v}
    if changed:
        io.set_motion(**changed)
        state.motion.update(changed)


def _averaging(io: AlignmentIO, state: _State, count: int) -> None:
    if state.averaging != count:
        io.set_power_averaging(count)
        state.averaging = count


def _range(io: AlignmentIO, state: _State, watts: Optional[float]) -> None:
    """A fixed meter range (W), or auto range with None."""
    if state.range_w != watts:
        io.set_power_range(watts)
        state.range_w = watts


def _scan_range(state: _State) -> float:
    """The fixed meter range a scan starts at: SCAN_HEADROOM × the power expected
    (at least × the detection level), at most state.scan_range_w."""
    expected = max((p for p in (state.peak_w, state.detect_w) if math.isfinite(p)), default=0.0)
    return min(state.scan_range_w, SCAN_HEADROOM * expected) if expected > 0 else state.scan_range_w


def _raise_range(state: _State, range_w: float) -> float:
    """The range after an overrange at ``range_w``: ten times higher (and state.scan_range_w with it)."""
    range_w *= 10.0
    state.scan_range_w = max(state.scan_range_w, range_w)
    if range_w > MAX_RANGE_W:
        raise AlignmentError(f"The power meter overranges at {range_w / 10:.3g} W: "
                             f"check source_power_mw and the meter.")
    return range_w


def _inside(state: _State, pos: Dict[str, float]) -> bool:
    """Whether ``pos`` (x, z in µm, rx, rz in °) is inside the window, its margins left."""
    s = state.settings
    return (math.hypot(pos["x"], pos["z"]) <= (s.range_mm - WINDOW_MARGIN_MM) * 1e3
            and math.hypot(pos["rx"], pos["rz"]) <= s.range_deg - ANGLE_MARGIN_DEG)


def _db(power: float, reference: Optional[float]) -> float:
    """``power`` against ``reference`` in dB (-inf if either is not positive)."""
    if not reference or not power or power <= 0 or reference <= 0:
        return -math.inf
    return 10 * math.log10(power / reference)
