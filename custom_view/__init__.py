"""Custom view: live power against the threshold, and the alignment phase by phase, on /view/.

Optional. When this package exists next to ``config.yaml``, the controller
calls :func:`setup` at every Initialize, then starts the hub. Delete the folder
and the standard app runs exactly as before.

What the page reads (the contract with fiber_alignment/alignment.py and
algorithm.py; app_logic/tests/fiber_alignment/test_contract.py checks it):

* ``power``: a source sampling the power meter at POWER_RATE_HZ, for the live
  readout. It runs on its own thread. While an alignment streams readings, it
  repeats the latest of them instead of reading the meter: the tile then shows
  what the algorithm measured, and the alignment's readings never wait for it.
* ``align_phases``: published here at setup, the algorithm's phases and how
  each is plotted (``algorithm.PHASES`` / ``PHASE_VIEWS``), so that the page
  lays out its cards before the first alignment.
* Published by fiber_alignment/alignment.py during the "Align fiber" step
  only, which is why the plots move only then:

  - ``align_sample``: one per reading, ``[x, y, z, rx, ry, rz, power_w, kind,
    phase, t, still]``. x, y, z (µm) and rx, ry, rz (°) are the robot's
    offsets from the start in the start's tool frame, the frame the algorithm
    moves in (y runs along the fiber). ``kind`` is "cart" (lateral moves),
    "ang" (tilts) or None for the verdict. ``phase`` is the index in the
    run's phases (-1 outside any), ``t`` the time since the start (s),
    ``still`` true for a reading taken at rest.
  - ``align_acquire``: the search for light: ``noise_w``, ``detect_w`` (the
    level counted as light), ``warm`` (light at the start: no search, unless
    ``full``), ``full`` (acquire_full_search: both spirals run to their full
    radius), ``found``, and ``search``: null after a warm start, else "angle"
    (a spiral in rx/rz) or "lateral" (a spiral in x/z), published as each
    spiral starts and as it ends on the light, with its ``radius``, ``pitch``
    and ``unit`` (° or µm), and once found ``at`` (the spiral's two axes) and
    ``power_w``. ``around``: where the search is centred, "remembered pose"
    (where this fiber aligned last), "start", or "ring k/n" (the tilt spirals
    around the start, when both spirals found nothing). A search may run
    several times: the map shows its latest run.
  - ``align_profile``: one per 1-D scan. ``axis`` (x, z, rx or rz), ``unit``
    (µm or °), ``iteration``, ``rescan``, ``span`` [lo, hi], ``passes``
    [{``dir`` 1 or -1, ``points`` [[coord, power_w], ...]}], ``fit`` (null,
    or ``model`` "gauss" with ``center``, ``width`` (1/e²), ``peak`` (the
    fitted power at the centre), ``baseline`` and ``lag`` (µm, what remains
    of the reading/pose lag), or ``model`` "parabola" with ``center``,
    ``width`` and ``peak``), ``chosen`` (where the fiber went, or null), and
    for tilts ``walk_um_per_deg`` (how far the beam walks along the lateral
    axis per degree of tilt: in effect the distance between the fiber ends).
  - ``align_gradient``: one per gradient circle, at the start of the Peak
    phase: ``round``, ``search`` ("lateral": a circle in x/z, "angle": in
    rx/rz), ``unit`` (µm or °), ``center`` and ``step`` [du, dv] in the
    circle's two axes (the step taken: [0, 0] when negligible), ``radius``,
    ``step_widths`` (the step in coupling widths), ``noise_widths`` (the
    step's noise, from the readings' scatter), ``gain`` (the fraction of
    the full step, halved after each overshoot), ``readings`` (lit readings
    fitted) and ``power_w`` (their mean, or null). Its readings are the
    samples of kind "cart" (lateral) or "ang" (tilt) taken meanwhile.
  - ``align_peak``: one per peak-search iteration: ``iter``, ``center`` [x, z],
    ``radius_um``, ``step`` [dx, dz], ``step_um``, ``step_widths``,
    ``newton``, ``power_w`` (at the centre) and ``points`` [[x, z, power_w], ...].
  - ``align_result``: the summary at the end (phases, phase views, published
    data, decimated readings with every reading at rest, ``best_still``, the
    final pose), from which a page loaded after the run redraws it. The hub
    keeps only the latest 2000 readings.
  - Events: ``align_start`` (phases, ``phase_views``, ``sample_format``,
    timeout, start pose, threshold, ``source_power_w``) and ``align_end``
    around the run, ``align_phase_start`` / ``align_phase_end`` around each
    phase. Each publish also carries ``phase`` and ``t``.
* The threshold is not on the hub. The page reads ``power_threshold_mw`` from
  /api/context, so an edit in the Variables tab shows at once, even before
  Initialize.

The page knows no phase by name: it draws one card per phase, as its view
says ("map", "profiles" or "peak"). It must not load anything from a CDN:
show-floor networks are not reliable. Its plots are drawn by plot3d.js,
chart2d.js and landscape.js, in this folder.
"""

import math

from devices.SimulatedDevice import SimulatedDevice

try:
    from ..fiber_alignment.algorithm import PHASES, PHASE_VIEWS
    from ..fiber_alignment.cell import POWER_METER
except ImportError as e:  # pragma: no cover - depends on how the workspace is loaded
    if e.name is not None:
        raise  # a module is really missing (e.g. a dependency): report that, not the fallback
    from fiber_alignment.algorithm import PHASES, PHASE_VIEWS
    from fiber_alignment.cell import POWER_METER

#: Live readout rate. A PM100x with ``averaging: 10`` takes ~30-35 ms per
#: reading: 20 Hz is about as fast as it goes with room to spare. The driver's
#: lock makes a step reading the meter wait at most one reading of it, and
#: during an alignment this source does not read the meter at all.
POWER_RATE_HZ = 20
#: An alignment reading younger than this (s) stands in for the meter.
ALIGN_FRESH_S = 0.5


def setup(hub, devices):
    """Declare the data sources for this workspace."""
    meter = devices.get(POWER_METER)
    if meter is None:
        raise LookupError(f"No device '{POWER_METER}' in config.yaml: this view reads its power.")
    if isinstance(meter, SimulatedDevice):
        raise LookupError(f"Device '{POWER_METER}' is simulated: there is no power to show.")

    # The page compares readings with a threshold in mW and draws them on a
    # linear axis; dBm would also be negative below 1 mW.
    unit = meter.api.power_unit
    if unit != "W":
        raise ValueError(
            f"Power meter '{POWER_METER}' reads in {unit}; the custom view needs watts. "
            f"Set power_unit: \"W\" on it in config.yaml and Initialize again."
        )

    hub.publish("align_phases", {"phases": list(PHASES), "phase_views": PHASE_VIEWS})

    def read_power():
        # During an alignment, its own latest reading: no second reader on the meter.
        recent = hub.window("align_sample", hub.now() - ALIGN_FRESH_S)
        watts = recent[-1][1][6] if recent else meter.api.read_power()
        # Overrange reads +inf and an invalid reading NaN: record nothing
        # rather than a value the page cannot use.
        return watts if isinstance(watts, float) and math.isfinite(watts) else None

    hub.add_source("power", read_power, rate_hz=POWER_RATE_HZ)
