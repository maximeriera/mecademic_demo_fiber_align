# Fiber alignment: brief

Part 1 is for whoever presents the demo: what the robot does, why it is
hard, and how to read the screen. Part 2 is for whoever runs, tunes or
changes it. The code is in `fiber_alignment/algorithm.py` (the algorithm)
and `fiber_alignment/alignment.py` (moves, readings, safety, verdict).

---

## Part 1: for presenters

### What the robot does

The robot holds the receiving fiber, connected to a power meter. Facing it,
across a small gap, a fixed fiber emits infrared laser light (1557 nm, about
1 mW). The laser stays fixed for safety: its beam always points the same way,
and the robot only ever moves the passive, receiving end. Both fiber ends have
collimating optics: the light crosses the gap as a beam about half a
millimetre wide. The robot moves the receiver, sideways and in angle, until it
collects as much of that light as possible.

The light is invisible and there is no camera. The robot only has one number
to go by: the power the meter reads, up to about a hundred times a second.

A perfect alignment reads about 93 % of the source power. The rest is
reflected at the two glass/air faces. On screen, that is −0.31 dB vs source.

### Why it is hard

- **Four settings at once.** The robot can shift the receiver sideways (X, Z)
  and tilt it (Rx, Rz). It never moves it along its axis: the gap stays as
  taught, so the two ends can never touch.
- **Lopsided tolerances.** Sideways, the beam forgives a few hundred µm. In
  angle, 0.2° already loses two thirds of the light. More than about 0.7° off,
  nothing reaches the meter: it reads noise, and there is no slope to climb.
  So finding the light is mostly a matter of angle.
- **A different fiber every cycle.** Each fiber sits a little differently in
  the gripper and points a little differently. A fiber just picked is often
  off in angle and sideways at once, the hardest case to find. So the robot
  remembers where each fiber aligned last time, and starts there.
- **Angle and position are linked.** Tilting the receiver about its tip also
  moves the sideways position where it collects best, by about 175 µm per
  degree for ends 10 mm apart. Each tilt has to be re-centred before it can be
  judged.
- **A flat top.** Near the best pose, being 20 µm or 0.05° off costs only 1 to
  3 %. The last phase finds the top from the shape of the power around it,
  not by feel.

### The four phases

The screen has one card per phase, left to right. The card of the running phase
is outlined in yellow.

| Phase | What the robot does | What the card shows |
|---|---|---|
| **Acquire**: find the light | It goes to where this fiber aligned last time, if it has before, and measures the meter's noise. If light already reaches the meter, it goes straight to Center (unless the full search is on: see below). Otherwise it sweeps Lissajous curves around the start: sideways first, then in angle, over a small disc, then larger ones (up to 500 µm and 3°). A faint hint of light makes it start again from there. It stops at the first light, goes back to it and checks it at rest, looking around slowly if the light was not quite there. Still dark, it sweeps in angle again at 6 points around the start. | A 3D landscape of the power along the latest scan, always over X/Z: a tilt scan turns the fiber without moving it sideways, so it shows as a column of readings at one spot. The line under it says where the scan is centred: "remembered pose", "hint 1", "ring 3/6", or nothing for the start; "light at the remembered pose" when the fiber found light where it aligned last time. The "light" mark on the colour bar is the level that counts as light. It keeps filling during the later phases: their readings join the map, which shows the best power seen at each spot, so the landscape rises to the final peak as the robot climbs. |
| **Center**: centre on the beam | It sweeps across the beam along X, then along Z, both ways. It fits a bell curve to each sweep and moves to its top, until the top stops moving. | The latest X and Z sweeps (green forward, blue backward), the fitted bell curve, and the chosen centre in yellow, in dB against the source on a log scale of the loss (the last dB under the source as tall as the 40 under it). |
| **Angle**: square the angle | For Rx, then Rz, it tries tilts stepping away from the current one while the power rises, re-centring at each tilt. A curve through the best tilts picks the angle. It repeats with finer steps while that still gains. | Top: the power against each tilt, with the fitted curve and the "walk": how far the best position moves per degree of tilt. Bottom: the sideways re-centring sweeps. All in dB against the source on a log scale of the loss, the tilts zoomed on the latest round. |
| **Peak**: the last few µm, and the last hundredths of a degree | First, small circles read on the move, as in the cell's first script: one sideways (about 36 µm across), then one in angle (about 0.03°). The power around each circle shows which way is uphill, and the robot steps that way; shorter steps after one goes too far. It repeats until the steps are lost in the noise, usually 1 to 4 rounds of about 2.5 s on the cell. Then it takes readings at rest on a small square grid, 5×5 by default (±0.2 beam width, about ±50 µm). It fits a dome to them and steps to its top, until the steps are tiny. It ends on the best reading taken at rest. | The grid readings and the sideways circles as a 3D landscape in dB against the source (on a log scale of the loss, so tenths of a dB show), the centres on its floor, and the size of every step, circles then grid: lateral ones in µm, tilt ones in m°. |

When the start is tilted, Angle brings by far the largest gain (+6 to
+18 dB in simulation); when it is only off sideways, Center does. Peak
mostly polishes, but it is the only phase after Angle that corrects the
angle. The whole alignment takes about 19 to 27 s on the simulated cell,
30 to 46 s when the start is off in angle and sideways at once, and about
12 s from a fiber's remembered pose. On the real cell, it scales with how fast the meter and the
robot answer.

### The top row

- **Power, live.** In dB against the source (`source_power_mw`): 0 dB is
  all of it. Under it, the reading itself and the source, then the threshold
  (`power_threshold_db`) in dB and in µW. Green at or above the threshold,
  red below, with the margin to it.
- **Power over time.** In dB against the source on a log scale of the loss
  (the last dB under the source as tall as the 40 under it, as on the Center
  and Angle charts), the threshold dashed, and the best so far drawn in
  green. Each phase has a band.
- **Alignment.** The verdict, the time spent against the time budget, and one
  line per phase with its gain in dB and its duration.

### The verdict

`✓ Aligned · −0.31 dB` or `✗ Below threshold · …`: the final power against
the source. The line under it gives that power and the source in µW and mW,
the threshold in dB, and the number of readings. A fiber below the threshold is still put back in its slot and
counted as failed, and the cycle moves on to the next fiber. If the alignment
stops on an error (or ABORT), it shows `✗ Interrupted` with the message under
it.

### Showing the whole search

When light already reaches the meter at the start, Acquire skips the
search, so the audience never sees it. To show it, set
`acquire_full_search` to true in the Variables tab while the cell is idle.

Every alignment then runs a sideways Lissajous scan over its full disc
(500 µm), then a tilt one (3°), even with light at the start. Both are
centred where the fiber starts: its remembered pose, else the start. The
fiber then goes to the brightest point of the two. The Acquire card reads
"full search · …". It costs a few seconds per fiber; set it back to false
for production.

---

## Part 2: technical details

### Frame and axes

Every position is an offset from the alignment start, the pose `align_start`
leaves the receiver in. Offsets are in the start's tool frame (`TOOL_TRF`,
whose TCP must be the tip of the receiving fiber the robot holds):

- X and Z: lateral, in µm.
- Y: along the fiber, never moved.
- Rx and Rz: tilts about the tool axes, in degrees.

`io.local(pose)` and `io.pose_at(...)` convert between robot poses and these
offsets. The rows published on `align_sample` use them too.

### The phases in detail

**Acquire** (`acquire`, `search_rounds`, `lissajous_scan`, `_confirm`)

- **Where it starts.** At the pose where this fiber aligned last
  (`io.seed`, from the `fiber_poses` variable: see "Remembered poses"
  below), else at the start. A remembered pose is not used if the scans
  around it would leave the window (`align_start` taught again far from it).
  Its offset along the fiber is never used: the gap stays the taught one.
- **Noise.** 20 readings there give the noise floor (their median) and its
  spread σ. Light is a reading of at least `signal_detect_uw`, and at least
  median + 8σ.
- **Light at the start** (median ≥ `signal_detect_uw`). Straight to Center,
  with no baseline under the fits (unless `acquire_full_search`, below).
- **Lissajous scans.** Each scan follows u = A·sin(n·t), v = A·sin((n+1)·t)
  around its centre, in X/Z (translation) or Rx/Rz (rotation).
  - It starts at the centre, crosses it again halfway and ends there.
  - It covers the square ± A, the first half already, coarsely. Its
    points beyond A from the centre (the square's corners) are brought
    back onto that circle: it never reaches farther than A, as the window
    and fibers close together need, and still covers the disc.
  - Its lines are sized from how far light shows from the optimum
    (`acquire_capture_um`, `acquire_capture_deg`), not from the beam's core.
    It passes within 0.7 of that from every point (n = ⌈1.05 A / (0.7 ×
    capture)⌉, measured), and reads every 0.35 of it along its path.
  - Speeds come from the measured time per reading, but a scan takes at
    least 0.15 s to cross a capture diameter, so that a meter slow at low
    power still shows light it passes: the cell's meter, slow to leave its
    lowest range, reads µW crossed faster as 50-100 nW. Moves are queued in chunks of 400,
    with blending.
  - Each reading is placed along the path as it comes, on the stretch just
    ahead of the one before, so a path that crosses itself does not mix
    up its passes.
- **Translation, then rotation, over growing discs.** Around the centre:
  a translation scan, then a rotation scan, over 1/3, then 2/3, then all of
  `acquire_radius_um` / `acquire_radius_deg`. Near first, then wider.
- **A hint of light restarts the search.** A hint is two readings in a row
  at least 3× the power at rest at the centre, at least noise + 5σ and a
  fiftieth of the detection level, and never more than the detection level
  itself. It stops the scan.
  - So low because, read on the move, light can show far below what it
    is: on the cell on 2026-11-06, scans crossing 1-70 µW read 50-100 nW
    (the meter slow to leave its lowest range). At rest it shows.
  - Confirmed at rest, the hint becomes the new centre, and the scans start
    again from the smallest disc around it.
  - Each restart needs 3× more power than the last, so there are only a few.
  - Payloads then say `around: "hint k"`.
- **Light, confirmed at rest, ends Acquire.** Readings taken on the move may
  show the light out of step with their pose: late if the meter lags,
  early if the pose was read late. Back at the pose of such a reading, the
  fiber may sit beside the light: on the cell's narrow beam, 22 runs of
  2026-11-05 went back to a dark spot this way, and Center then lost the
  light.
  - So the fiber goes back to the reading's pose and reads at rest
    (`_confirm`). Under the detection level, it reads again for up to
    0.3 s and keeps the highest: a meter still in its lowest range reads
    that range's top until it switches up.
  - Under half of what the scan read, it goes back over the stretch of the
    path around that reading, backwards and 3× slower (`_back_along`):
    ± 2 capture radii, and at least what the scan covers in 0.1 s.
  - Out of step by a time τ, the light shows v·τ along the way it is
    crossed: on the way out at s + v·τ, on the way back at s − v·τ/3. So
    the light is at (s_out + 3·s_back) / 4, whatever τ, and the fiber goes
    there, if it reads more at rest.
  - Center always starts on light.
- **Remembered pose dark.** The fiber is no longer where it aligned last:
  the same scans again around the start.
- **The ring.** Still dark: rotation scans at 6 points `acquire_ring_um`
  around the start, at the start's tilt.
  - They cover a fiber off in tilt and sideways at once, beyond what the
    scans at the start can see.
  - About 1.5 × `acquire_capture_um` (400 µm). With 0, no ring.
- **Still dark.** `AlignmentNoSignal`.
- **`acquire_full_search`.** A translation scan, then a rotation scan,
  over the full radius, each to its end whatever the start reads. Then the
  brightest reading of the two, confirmed at rest.
- **Each payload** on `align_acquire` says where its scan is centred
  (`around`: "remembered pose", "start", "hint k", "ring k/6"), with
  `pattern: "lissajous"`, how far it reaches (`radius`), line spacing (`pitch`) and
  reading spacing (`spacing`). The Acquire card maps the latest scan, over X/Z.

**Center** (`center`, `cross_scan`)

- **Iterations.** Up to 3, each an X scan then a Z scan. It stops when the
  centre moves less than max(0.5 µm, 0.05 width).
- **A scan.** It spans ±1.5 widths around the current estimate, with a
  reading every width/10 (at least 0.25 µm). Each scan runs both ways, with
  1 sample per reading and the meter at a fixed range: 10 × the power
  expected (at least 10 × the light level), at most 1.2 × source, ×10 on an
  overrange. At 1.2 × source, faint first light (tens of nW) would read 0.
- **The fit.** Each direction is fitted with a Gaussian over the noise floor:
  log(power − floor) is a parabola, fitted with weights.
  - The centre is the mean of the two directions.
  - `lag` is half their difference: what remains of the reading/pose delay.
- **Window rules.**
  - A centre within 15 % of the window's ends, or past them: the window moves
    towards it by at most half its span, and scans again (3 times at most).
  - No light in the window: it doubles around the same middle (3 times at
    most).
  - If no window catches the centre, the fiber goes to the best reading. It
    never goes to an extrapolated centre.
- **The width.** Updated only from a centre caught inside a window, by ×3 or
  ÷3 at most per fit.
- **No light along a line.** `AlignmentNoSignal` ("Lost the light while
  centring along x").

**Angle** (`angle`, `tilt_search`)

- **Rounds.** Rx then Rz, repeated while a round gains at least 0.1 dB, 3
  rounds at most.
- **Tilt steps.** From the current tilt, steps of 0.5 angle width (halved
  each round), growing ×1.5 while the power rises. It tries one direction,
  then the other if the first did not help. At most 10 tilts per search.
- **At each tilt.** The fiber is re-centred along the lateral axis that tilt
  walks the beam on: Rx walks it along Z, Rz along X. The re-centring starts
  where the walk measured so far predicts the beam. The power after
  re-centring is the tilt's power. A tilt with no light counts as no power,
  and ends that direction.
- **The tilt kept.** A parabola in log power through the best tilt and its
  two neighbours gives the tilt, kept between those two neighbours. It also
  gives a new angle width.
- **The walk.** The slope of the re-centred position against the tilt
  (µm/°) is published as "walk". It is in effect the distance from the
  rotation centre (the TCP, at the receiver's tip) to the emitter's end, per
  radian: ~175 µm/° for ends 10 mm apart.
- **Then.** Both lateral axes are re-centred.

**Peak** (`peak`): the gradient circles (`gradient`, `circle`), then the pattern

- **The circles.** These are the circles of the cell's first script
  (`looping_circles`). Each round runs a circle in X/Z, then one in Rx/Rz,
  around the current pose.
  - Their radius is 0.15 width (`GRADIENT_RADIUS_WIDTHS`), about 36 µm and
    0.03° on the cell's ends.
  - Each circle runs once each way, on the move, at a fixed meter range.
    Going both ways cancels the reading/pose lag, and a slow drift of the
    source, in the fit.
  - Readings: 24 a lap in simulation (`GRADIENT_READINGS`). On the cell, a
    lap of 36 blended moves takes ~0.42 s whatever the speed, so it gets
    ~50 readings, and a round takes ~2.5 s.
  - The robot rounds the circle off: on the cell, it traced ~23 µm and
    ~0.031° for 28 µm and 0.043°. The fit uses the poses read, so this does
    no harm.
- **The step.** A plane fitted to the log power over the circle's readings
  gives the slope, and its noise (from the readings' scatter about it).
  - On a Gaussian, the top is the slope × width² / 4 away (lateral, 1/e²
    radius), or × width² / 2 (tilt, 1/e half-width). The step goes that
    far, times a gain, and at most 0.5 width (`GRADIENT_MAX_STEP_WIDTHS`).
  - The gain starts at 1. It halves each time a step goes back against the
    previous one of its kind, i.e. the previous step overshot
    (`GRADIENT_OVERSHOOT_GAIN`). On the cell's faint light of 2026-10-28,
    the top was ~2.5× sharper than its fitted width. Full steps then swung
    −12, +14, −11 µm from round to round.
  - A step under twice its noise (`GRADIENT_NOISE_SIGMAS`) is not taken: the
    readings cannot tell it from noise. The cell's readings scattered 1.5–
    2.7 % about the plane, about 1 µm of noise on a lateral step.
  - The first script stepped a fixed 30 µm / 0.03°, shrunk ×0.8 above a
    threshold. Here the steps shrink near the top by themselves, and after
    an overshoot, with no threshold.
- **The tilt circle follows the ridge.** It carries X/Z along with the
  tilt, by the walk the Angle phase measured. The beam then stays centred
  as the circle tilts, as in Angle's re-centred tilts.
  - Without this, a TCP away from the tip (or a long gap) makes the circle
    cross the ridge: its slope is mostly the sideways offset, and its steps
    throw the beam off. In simulation, with the TCP 500 µm behind the tip,
    the circles then gained nothing.
- **When they stop.** When neither circle of a round takes a step: each is
  under 0.01 width (and 0.5 µm / 0.001°) or under its noise. Otherwise
  after `gradient_rounds` (8). A circle stops them early if it would leave
  the window, or sees fewer than 8 lit readings.
- **Kept only if better.** A reading at rest follows. If it is lower than
  before the circles, the pattern starts where the circles did.
- **What they gain** (simulation).
  - Started 0.1–0.15° off in tilt (power 0.56–0.78 of the best) on
    cell-like optics, Peak ended at 0.9997–1.001 with the circles, in 2–3
    rounds. Without them it ended at 0.57–0.78: the pattern moves only X/Z.
  - With a top sharper than its fitted width and 2 % noise per sample, as
    on the cell, they stop after 1–4 rounds, ending at 0.998–1.000. Without
    the gain and the noise test, they ran all 8 rounds with steps swinging
    ~0.4 width.
  - When Angle has already found the top, one round finds nothing to do.
- **The pattern.** A square grid of `peak_grid` × `peak_grid` readings at
  rest (5×5 by default, 10 samples averaged each), taken row by row, each row
  the other way round so that no move is long.
  - It spans ±0.2 × the mean lateral width (`PEAK_RADIUS_WIDTHS`): about
    ±48 µm on the cell's ends, so a 5×5 grid has a reading every ~24 µm.
  - Its edge points read ~8 % under its centre, plenty for the fit.
  - A 5×5 iteration takes about 2.5 s, a 3×3 one about 1 s; one iteration is
    usually enough.
- **The step.** A quadratic fit of log power gives a Newton step to the top
  (a gradient step if the fit has no maximum), clipped to the radius.
- **When it stops.** On a step under max(0.5 µm, 0.01 width), a gain under
  0.05 dB, or after 6 iterations. The fiber then goes to the centre, or to
  the best reading at rest if that one is better.

**The verdict** (`run_alignment`)

- A fresh reading in auto range, where the algorithm left the fiber. If it is
  more than 0.2 dB under the best reading at rest, the fiber slowly goes back
  there first.
- Passed if the reading is at least the threshold: `power_threshold_db`
  under `source_power_mw` (its sign aside: −0.4 dB is 0.912 mW from 1 mW).
- The result is published on `align_result`, with everything the page needs
  to redraw the run after a reload.

**Remembered poses** (`memory.py`, called by `process.align_fiber`)

- After every alignment whose final reading is light (≥ `signal_detect_uw`),
  passed or not, the final pose is kept for that fiber in the `fiber_poses`
  variable (JSON, persisted). The next alignment of the same fiber starts
  there.
- The poses are the robot's, in the WRF: they stay valid if `align_start` is
  taught again. One kept with another `TOOL_TRF` is not used.
- To forget them (a fiber moved in its slot, a new rack), press Reset on
  `fiber_poses` in the Variables tab, or set it to `{}`.
- To ignore them without forgetting them, turn `use_remembered_pose` off:
  every alignment then searches from where it starts, and the poses are
  still kept up to date.

### Params (`context.yaml`, Variables tab)

The defaults are `context.yaml`'s. Every param persists: an edit in the
Variables tab survives a restart, but then a new default in `context.yaml`
does not apply to it until you press Reset on it.

| Param | Default | What it sets |
|---|---|---|
| `power_threshold_db` | −0.4 | Pass or fail: how far under the source the final power may be (dB, its sign aside). The cell's passed runs of 2026-11-06 ended 0.23–0.39 dB under 1 mW. |
| `source_power_mw` | 1.0 | The source's output. The threshold and the view's dB are against it; the scans' fixed meter range is at most 1.2 × this. |
| `lateral_width_um` | 200 | Starting guess of the lateral 1/e² radius. Cross scans span ±1.5 widths with a reading every width/10, and the Peak pattern spans ±0.2 width. Re-measured by every fit. |
| `angle_width_deg` | 0.2 | Starting guess of the tilt 1/e half-width (0.19° measured on 2026-10-28). The first tilt step is 0.5 width. Re-measured by the tilt parabolas. |
| `acquire_radius_deg` | 3.0 | How far the rotation scans reach, in their largest disc (1/3, 2/3, then all of it). |
| `acquire_radius_um` | 500 | How far the translation scans reach, in their largest disc. |
| `acquire_capture_um` | 250 | How far sideways from the optimum light still shows above `signal_detect_uw`. Sizes the translation scans' lines. Fitted to the cell's runs of Nov 5–6 (~360 µm: the beam's faint wings), with a margin. |
| `acquire_capture_deg` | 0.3 | The same in tilt (~0.45° on the cell). Sizes the rotation scans' lines. |
| `acquire_ring_um` | 400 | How far around the start the ring of rotation scans runs, when all the scans found nothing: about 1.5 × `acquire_capture_um`. 0: no ring. |
| `signal_detect_uw` | 0.005 | Lowest power counted as light, raised to noise + 8σ. |
| `acquire_full_search` | true | Run a translation then a rotation scan over the full radius first, even with light at the start (demo). |
| `use_remembered_pose` | true | Start where this fiber aligned last (`fiber_poses`). Off: always search from where the alignment starts. |
| `peak_grid` | 5 | Readings per side of the Peak pattern: odd, 3 to 9 (anything else is refused before the robot moves). More readings give a finer grid and a better fit, but a slower phase. |
| `gradient_rounds` | 8 | Most rounds of gradient circles (one in X/Z, one in Rx/Rz: ~2.5 s together on the cell, ~1 s simulated) at the start of Peak. They stop sooner once no step stands out of the noise. 0: no circles, and then nothing after Angle corrects the tilt. |
| `align_range_um` | 2000 | Window: how far X, Y, Z may go from the start. It must leave 100 µm beyond `acquire_radius_um` and `acquire_ring_um`. |
| `align_range_deg` | 8.0 | Window: how far the three angles may turn. It must leave 0.5° beyond `acquire_radius_deg`. |
| `align_timeout_s` | 150 | After this, back to the best reading at rest, and judged there. |
| `align_settle_s` | 0.05 | Pause after each point-to-point move. |

The finer constants (spans, densities, steps, stop criteria) are at the top
of `algorithm.py`, each with its reason.

How far the search reaches, with the cell's optics of November 2026 (a
75 µm, 0.075° core with faint wings, fitted to its 466 runs of Nov 5–6) and
0.5 µW to detect:
- light shows within about ±0.45° of the best tilt (at the best position),
  and within about ±360 µm of the best position (at the best tilt): the
  `acquire_capture_*` params, with a margin;
- so the rotation scans find a start up to about 3.4° off in tilt, and the
  translation scans one up to about 0.9 mm off sideways;
- a start off in both at once is the ring's.

### Readings, poses and timing

- **Robot queries.** Every pose is read with `GetRtCartPos()`. On the cell,
  `GetRtTargetCartPos` times out ("timeout waiting for interruptable
  event"), and no pose is ever asked while the robot's motion is paused.
- **A scan.** The moves are queued while the motion is paused, with a
  checkpoint after the last one, then the motion resumes. Readings go on
  until the checkpoint is reached; each is one meter query followed by one
  pose query.
- **Where a reading was taken.** A reading's pose is interpolated at the
  middle of the meter's reading, between the poses read just before and
  just after it. Readings taken on the move therefore land where they were
  taken.
- **Speeds.** Scan speeds are the wanted spacing divided by the measured
  time per reading (a moving average, plus a 2 ms pause). A slower meter or
  network gives slower scans, not sparser readings.
- **What remains of the delay** (6.8 µm on the cell on 2026-10-27) is
  cancelled by scanning both ways. It shows as `lag` on the Center card.
- **The meter.**
  - 1 sample per reading while scanning, 10 at rest.
  - A fixed range during cross scans and gradient circles: 10 × the power
    expected, at most 1.2 × source, ×10 on an overrange, up to 1 W. Auto
    range in the first-light scans, for the readings at rest of the Peak phase and for
    the verdict.
  - The meter's settings are restored after the run.

### Safety

- **The window.** Every pose is checked against `align_range_um` /
  `align_range_deg` before it is sent. One outside is refused, and the
  alignment faults. The search's radii and the ring are checked against the
  window before anything moves, and each scan's disc shrinks to stay in
  it. The window stays centred on `align_start`:
  a remembered pose never moves it.
- **No move along the fiber.** Tool Y is never commanded, so the gap stays
  as taught. The simulator fails any run where the ends touch.
- **Timeout.** After `align_timeout_s`, the fiber goes back slowly to the
  best reading at rest and is judged there.
- **ABORT or a device fault.** The robot's motion is cleared at once, and the
  view shows "Interrupted".
- **A stalled scan.** No robot motion for 3 s during a scan ends the run,
  with the robot's state in the message.

### Failure messages

| Message | Meaning | What to do |
|---|---|---|
| `No light above … tilting ±3° nor moving ±500 µm around the start (and around the fiber's remembered pose), nor tilting at 6 points 750 µm around it: check the laser, the fibers and the align_start pose.` | No search saw light. | Check the laser is on and both fibers are connected. Check `align_start` lands within ~3° and ~1 mm of the best pose. Or raise the radii, within the window. |
| `Lost the light while centring along x: …` | The light vanished during Center. | The laser, the fibers, or the fiber slipping in the gripper. |
| `acquire_radius_um (…) needs 100 µm of room inside align_range_um (…)` (or `acquire_radius_deg`, `acquire_ring_um`) | The search would leave the window. Refused before anything moves. | Lower the radius, or widen the window. |
| `X … is … mm from the start: outside the ±… mm window` | The algorithm asked for a pose beyond the window. | Widen the window, or start closer to the best pose. |
| `The power meter overranges at … W: check source_power_mw and the meter.` | More light than the meter's top range. | Check `source_power_mw` and the meter. |
| `The robot dropped the scan …`, `The robot is not executing the scan …`, `The robot did not complete a move …` | The robot is in error, paused or not activated. The message gives its state. | Clear the robot's error, then Clear Faults. |
| `✗ Below threshold` (not an error) | The alignment ran, but the best power is under the threshold. | Compare the threshold with the peak measured on the setup. Check that the connectors are clean. |
| `timed out` on the verdict line (not an error) | The time budget ran out; the fiber was judged at its best reading. | See the tuning below, or raise `align_timeout_s`. |

### Tuning on the cell

1. **Widths.** Run one alignment. Read the 1/e² width fitted on the Center
   card's profiles, and the tilt width from the Angle card's parabolas. Set
   `lateral_width_um` and `angle_width_deg` close to them.
2. **Walk.** The "walk" under the tilt plots should match the distance
   between the fiber ends. If it is much larger, the TCP is not at the
   receiver's tip: tilts swing the tip around. Teach `TOOL_TRF` again.
3. **Lag.** The Center card's `lag` shows how late readings land along a
   scan. Two-way scans cancel it, but a large value means slow readings.
4. **Threshold.** Set `source_power_mw` to the source's output, note the
   loss of a good alignment against it on the live tile, then set
   `power_threshold_db` a little under it (−0.4 dB on the cell).
5. **Cold starts.** Turn `use_remembered_pose` off first, or the remembered
   pose skips the search. Run 5 alignments from no light (tilted ~1° from
   the Manual tab) and 5 from a few µW. Note their time and final power.
   For many in a row, run PROD with `prod_align_only` on. Each cycle moves
   to `align_start`, adds a random offset (`random_offset_um`,
   `random_offset_deg`), aligns and retracts; `aligned_count` /
   `failed_count` give the success rate.
6. **Remembered poses.** Align each fiber once from the rack. The next time
   it is picked, the Acquire card should read "light at the remembered pose".
7. **Gradient circles.** On the Peak card, the circles' steps should fall
   to 0 (not taken) within 1–4 rounds. If they keep going for all 8 rounds,
   read `noise_widths` and `gain` in the `align_gradient` payloads.
   Repeated steps despite a small gain point to a wrong walk (see 2), or to
   a source that drifts within a lap. Set `gradient_rounds` to 0 to compare.

### Testing without the cell

`app_logic/tests/fiber_alignment/sim.py` simulates the cell: a robot running the moves
and answering pose queries as the cell does, and a meter reading what the ends
couple at the robot's pose (late and smeared on the move), on a simulated
clock. From the repository root:

```
pip install -r requirements-dev.txt
python -m pytest app_logic/tests -q
```

The tests cover:
- the cell's run of 2026-10-27, a few µW at the start, and no light (tilt
  or lateral search);
- the full search, and the Peak grid (the gradient circles run before it in
  every test, but no test checks what they gain);
- single-mode fibers, with a TCP off the tip;
- the dark;
- that what the alignment publishes is what the view plots.

They run without `acquire_ring_um` (it is 0 when `context.yaml` does not set
it) and with no remembered pose, so they check the searches as they were.
The ring, the 5 nW detection and the remembered poses were checked with
separate simulated runs on 2026-10-28, with the `context.yaml` params. So
were the gradient circles, by moving the pose off in tilt and sideways just
before Peak.

To watch a simulated run in the real page:

```
python app_logic/tools/view_replay.py                                   # http://127.0.0.1:8765/view/
python app_logic/tools/view_replay.py --full-search --offset 50 0 0.45 0  # the whole search, light at the start
python app_logic/tools/view_replay.py --step & node app_logic/tools/view_check.mjs  # browser check, screenshots in test-results/view/
```

`--offset X Z RX RZ` puts the best pose that far from the start (µm, µm, °,
°). `--dark` runs with no light, and `--optics smf` uses bare single-mode
fibers.
