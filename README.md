# Fiber alignment demo

A Mecademic robot takes one of N receiving fibers from its rack (the light they
collect goes to a Thorlabs power meter). It brings the fiber in front of a
fixed fiber that emits the laser light, and aligns it until the power reaches
a threshold. Then it backs out and puts the fiber back where it was. The laser
stays fixed for safety: its beam always points the same way, and the robot
only ever moves the receiving end. Each PROD cycle handles one fiber and works
through the rack in order.

| # | Step (as shown in the UI) | How |
|---|---|---|
| 1 | Pick fiber | robot program `fiber<N>_in`, gripper closes, `fiber<N>_out` |
| 2 | Move to alignment start | robot program `align_start` |
| 3 | Align fiber | alignment algorithm (see below): robot moves from Python + power meter |
| 4 | Retract from alignment | robot program `align_retract` |
| 5 | Place fiber (into the slot it was picked from) | robot program `fiber<N>_in`, gripper opens, `fiber<N>_out` |

A fiber that does not reach the threshold is still put back and counted in
`failed_count`, and the loop moves on to the next fiber.

**Alignment-only production** (`prod_align_only` on, Variables tab): PROD
only aligns the fiber already in the gripper, over and over, each time from
a random start. Nothing is picked or placed, and `next_fiber` stays as it is.

| # | Step | How |
|---|---|---|
| 1 | Move to alignment start | robot program `align_start` |
| 2 | Random offset | `MoveLinRelTrf`: X/Z up to `random_offset_um` (300 µm), Rx/Rz up to `random_offset_deg` (1°), drawn evenly over a disc; never along or about the fiber (Y) |
| 3 | Align fiber | the alignment, from the offset pose |
| 4 | Retract from alignment | robot program `align_retract` |

`aligned_count` / `failed_count` then give the success rate. A search that
finds no light counts as failed and the fiber still retracts; it does not
fault the cell.

Each run adds a row to `logs/alignment/align_only_runs.csv` (the
`align_only_csv` param; empty turns it off), flushed at once so an
overnight run keeps its data. A row holds:
- the time, the run number and the fiber;
- the random offset (x, z µm, rx, rz °);
- the power at the start, whether Acquire found light, and how (e.g.
  `angle @ ring 3/6`);
- for each phase, its status, duration and best power so far;
- the gradient circles and pattern iterations;
- the verdict, power, duration and readings;
- where the fiber ended, both from the offset start and as the robot pose;
- the error, if the run stopped on one.

The columns are listed in `fiber_alignment/runlog.py`. An update that
changes them renames the old file aside instead of mixing them. Turn `use_remembered_pose` off for these runs, or each
alignment goes straight back to where the last one ended. The Manual tab
has the same Random Offset step as a button.

## Robot programs

Every absolute move is a program stored on the robot (Mecademic web portal),
started with `StartProgram`. Their names are in `fiber_alignment/poses.py`.

| Program | From → to |
|---|---|
| `fiber1_in` … `fiber4_in` | any safe pose → the slot's grip pose. Never drives the gripper |
| `fiber1_out` … `fiber4_out` | the slot's grip pose → a safe pose |
| `align_start` | a safe pose → the alignment start: fibers facing, small gap |
| `align_retract` | the aligned pose (anywhere inside the alignment window) → straight back out along the fiber axis → a safe pose |

Teach them with `TOOL_TRF` (`poses.py`) active: Python sets it before every
step, and the alignment rotates about it, so its TCP must be the fiber tip.
They run at the speeds set in the Variables tab unless they set their own.

## Files

| File | What it holds |
|---|---|
| `config.yaml` | Devices (`meca_robot`, `fiber_gripper`, `power_meter`), Manual tab buttons, step sequences |
| `context.yaml` | Params (threshold, alignment settings, speeds, `manual_fiber`; all persisted) and variables (cell state, each fiber's remembered pose, results) |
| `prod.py` | PROD cycle: `run_cycle(next_fiber)`, or `run_align_only_cycle()` with `prod_align_only`; counters, next fiber |
| `home.py`, `shipment.py` | Back out of the alignment station if needed, then a joint position |
| `calib.py` | Nothing to calibrate yet |
| `manual_actions.py` | One Manual tab button per step, plus gripper open/close and a full cycle |
| `fiber_alignment/poses.py` | **Data**: robot program names, tool frame, home/shipment joints |
| `fiber_alignment/cell.py` | Device names, `ProcessError`, robot setup (frame, speeds), `run_program()` |
| `fiber_alignment/gripper.py` | SmarAct gripper: open/close are moves of its MCS2 jaw channel |
| `fiber_alignment/process.py` | The process steps (and Random offset), shared by PROD and the Manual tab |
| `fiber_alignment/alignment.py` | Contract with the algorithm: 3D moves, scans, readings, window, timeout, verdict |
| `ALIGNMENT.md` | A brief on the alignment: for presenters, then the technical details |
| `fiber_alignment/algorithm.py` | The alignment algorithm (see below), and how the custom view plots each of its phases (`PHASE_VIEWS`) |
| `fiber_alignment/memory.py` | Where each fiber aligned last (the `fiber_poses` variable), so that its next alignment starts there |
| `fiber_alignment/runlog.py` | A CSV row per alignment-only run: its random offset, how the light was found, each phase, the result |
| `fiber_alignment/fit.py` | Its curve fits: Gaussian profiles, parabolas and quadratics in log power |
| `fiber_alignment/frames.py` | Frame math from the colleague's script (poses ↔ 4×4 matrices) |
| `custom_view/` | `/view/`: live power vs threshold (green/red), the verdict, and a plot per alignment phase (see below) |
| `tests/fiber_alignment/`, `tools/` | A simulated cell, the tests run on it, and a replay of a run into the custom view (see below). Development only: `.dockerignore` keeps them out of the image |

## To do before running on the cell

- [ ] Write the robot programs above in the web portal, taught with `TOOL_TRF` active.
- [ ] Teach `TOOL_TRF` and the home/shipment joints in `fiber_alignment/poses.py`, then set `POSES_TAUGHT = True`. Until then, every robot move is refused.
- [ ] `config.yaml`: the robot's IP address. The power meter's `wavelength` (1557 nm, from the colleague's script) should match the source.
- [ ] `context.yaml`: `source_power_mw`, the laser's output measured fiber to meter, and `power_threshold_mw`, 0.5–1 dB under the peak power measured on the setup (0.65 mW: the cell peaked at 0.76 mW on 2026-10-28). With an air gap a perfect alignment loses ~7 % to the two glass/air faces: about 0.93 mW from 1 mW.
- [ ] Check `align_range_um` / `align_range_deg`: any pose the algorithm asks for beyond them is refused, which faults the alignment. The alignment refuses to start when `acquire_radius_um`, `acquire_ring_um` or `acquire_radius_deg` does not fit in the window.
- [ ] `lateral_width_um` / `angle_width_deg`: how wide the coupling is, as starting guesses (every alignment re-measures them). The defaults come from the cell's collimated ends (alignment of 2026-10-27: ~240 µm; 2026-10-28: ~0.19° in tilt).
- [ ] Every param persists: a value edited in the Variables tab survives a restart, but then a new `default` in `context.yaml` does not apply to it until you press Reset on it (Variables tab). After pulling new defaults, Reset the params you have not tuned on purpose.
- [ ] Gripper (SmarAct MCS2 at 192.168.0.20): check its jaw channel and `velocity_mm_s` in `config.yaml`, then teach `gripper_open_mm` / `gripper_closed_mm` (Variables tab). The closed position is where the jaws hold the fiber, not beyond it.

## Testing step by step

With the controller READY, run the Manual tab buttons in process order, at low
speed first (`joint_vel_pct`, `lin_vel_mm_s`):

Pick Fiber → Move to Alignment Start → Run Alignment → Retract → Place Fiber

- `manual_fiber` (Variables tab) selects the fiber for Pick Fiber and Full Cycle.
- Each step checks the cell state before moving, using `held_fiber` and
  `at_alignment`. Asked for in the wrong state, it faults the cell with a
  message saying what to do. The message is in the ApplicationController log;
  fix the state, then Clear Faults.
- After a manual intervention (e.g. a fiber removed by hand), correct
  `held_fiber` / `at_alignment` in the Variables tab.
- Each fiber's alignment starts where that fiber aligned last
  (`fiber_poses`, written after every alignment that ends in light). Reset
  it in the Variables tab after moving a fiber in its slot or changing the
  rack; a remembered pose taken with another `TOOL_TRF` is ignored anyway.
  With `use_remembered_pose` off, every alignment searches from where it
  starts; the poses are still kept.
- HOME backs out of the alignment station first. It never opens the gripper.
  Recovering from an abort at a rack slot is not automated: jog the robot clear.

Open `/view/` during Run Alignment to watch it.

Then, on the cell:

1. Run one alignment and note what the fits measured: the 1/e² width on the
   Center card's profiles, the tilt width from the Angle card's parabolas
   (`angle_width_deg` is where the power falls to 1/e), and the `walk` under
   the tilt profiles: how far the best position moves sideways per degree of
   tilt, in effect the distance between the fiber ends (175 µm/° for 10 mm). Set
   `lateral_width_um` / `angle_width_deg` close to the widths measured.
2. On the Center card, `lag` (half the gap between the two directions of a
   scan) shows how late the readings land along a scan. Every centring scan
   runs both ways, which cancels it.
3. Start 5 alignments from a pose with no light (tilted ~1°, Manual tab) and
   5 from one with a few µW, and note their time and final power. Turn
   `use_remembered_pose` off first, or the remembered pose skips the search.
   For many such runs, start PROD with `prod_align_only` on: each cycle
   starts from `align_start` moved by a random offset.
4. Align each fiber once from the rack. The next time it is picked, the
   Acquire card should read "light at the remembered pose".

## Alignment

[`ALIGNMENT.md`](ALIGNMENT.md) explains it at more length: what to tell an
audience and how to read the screen, then each phase, param and failure
message in detail.

`fiber_alignment/algorithm.py` aligns from where this fiber aligned last
(`fiber_poses`, unless `use_remembered_pose` is off), else from wherever `align_start` left the fiber, even with
no light reaching the meter, as long as the optimum is within
`acquire_radius_deg` in tilt and about `acquire_ring_um` laterally. It never
moves the fiber along its axis: the gap stays the taught one.

Every scale (scan spans, steps, spiral pitches) follows how wide the coupling
is: `lateral_width_um` (the 1/e² radius against a lateral offset) and
`angle_width_deg` (the 1/e half-width against a tilt) are starting guesses,
and each fit re-measures them. The cell's collimated ends tolerate a few
hundred µm laterally but a few tenths of a degree in tilt: first light is
mostly a matter of angle. Four phases:

| Phase | What it does |
|---|---|
| Acquire | Starts at the fiber's remembered pose, if any. Measures the noise floor. Any light there (above `signal_detect_uw`, 5 nW, and the noise) goes straight to Center. Otherwise a spiral in tilt (Rx/Rz, out to `acquire_radius_deg`), then, still dark, a lateral spiral (out to `acquire_radius_um`), each stopping once past the light; dark at the remembered pose, the same around the start. Still dark, tilt spirals at 6 points `acquire_ring_um` around the start: a picked fiber is often off in tilt and sideways at once, which neither spiral alone finds. Then to the best reading. Still no light stops the alignment with "check the laser, the fibers and the align_start pose". With `acquire_full_search` (for demos), the first two spirals run to their full radius, even with light at the start. |
| Center | Scans across the light along X then Z, both ways, fits each scan with a Gaussian over the noise floor and moves to its centre, until the centre stops moving. A window that misses the centre moves towards it by half its span at most; failing that, the fiber goes to the best reading, never to an extrapolated centre. A window without light widens. The meter's range is fixed during a scan, at 10× the power expected, so faint first light stays visible. |
| Angle | For Rx then Rz: tilts stepping away from the current one (half an angle width at first) while the power rises, each re-centred along the lateral axis the tilt walks the beam on; a parabola through the best tilt and its neighbours picks the tilt. A tilt without light is a reading of no power, not an error. Again with half the step while it gains. |
| Peak | First the gradient circles of the cell's first script: rounds of a circle in X/Z, then one in Rx/Rz (0.15 width radius: ~36 µm, ~0.03°), read on the move both ways. A plane fitted to the log power over each gives the slope, and a step up it, sized from the slope and the width (at most 0.5 width). The steps shrink by half after one overshoots, and a step under twice its noise is not taken. The tilt circle carries X/Z along by the walk Angle measured, so the beam stays centred. They stop once no step is taken, or after `gradient_rounds` (8); usually 1 to 4 rounds of ~2.5 s on the cell (~1 s simulated). Kept only if the power at rest is not lower. Then readings at rest on a square pattern of ±0.2 lateral width around the centre (~±50 µm on the cell's ends), `peak_grid` per side (5 by default: 25 readings, ~2.5 s), a quadratic fit of the log power and a step to its top (at most the pattern's radius), until the steps are small. Ends on the best reading taken at rest. |

Each reading is paired with the robot's pose at the middle of the meter's
reading (interpolated between the poses read before and after it), so fast
scans place their readings where they were taken. Scan speeds come from the
measured time per reading. In simulation, with optics like the cell's, the
alignment of 2026-10-27 (~80 µW at the start) takes about 20 s, and a cold
start needing the tilt or the lateral spiral about 20–22 s. A full search
adds about 2 s; a 3×3 Peak pattern (`peak_grid` 3) saves about 1.5 s. With
0.2° / 240 µm optics and the `context.yaml` defaults, light is found from
starts up to 900 µm and 2.5° off at once (48 of 48 tested); those needing
the ring take 30–46 s in all. From the remembered pose, about 12 s. The
gradient circles add ~1 s when Angle has already found the top (one round:
~2.5 s on the cell); started
0.1–0.15° off in tilt, Peak still ends within 0.1 % of the best (0.57–0.78
without them, as the pattern moves only X/Z).

## Custom view (`/view/`)

Laid out for a 1920×1080 screen, read from a distance. It stacks on narrower
screens.

- **Top row** (about a fifth of the screen's height):
  - the live meter reading, green at or above `power_threshold_mw`, red below;
  - the power over time in dB vs the threshold (best so far in green), with
    a band per phase;
  - the alignment: the verdict, the time spent against `align_timeout_s`,
    and one line per phase with its gain in dB and its duration.
- **Plots, side by side**: one card per phase of the algorithm, in order,
  which stays on screen after its phase. The cards with a 3D landscape
  (Acquire, Peak) are half again as wide as a column of charts
  (`LANDSCAPE_WIDTH` in `custom_view/view.js`). The running phase's card is
  outlined in yellow. Positions are offsets from the alignment start in the
  tool frame, the axes the algorithm moves along (X/Z lateral in µm, Rx/Rz
  in °).
  - Acquire: a 3D landscape of the power along the latest search spiral,
    over X/Z whichever spiral ran (the readings at the start when light was
    already there). A tilt spiral stays at one X/Z spot, so it shows as a
    column of readings there until the later phases spread out. Until light is found it is
    scaled to the level counted as light, marked "light" on the colour bar.
    Its line says where the spiral is centred unless at the start
    ("remembered pose", "ring 3/6"), and reads "full search · …" when
    `acquire_full_search` is on. It keeps filling
    through the later phases, over the same axes, with the best power seen at
    each spot: the landscape rises to the final peak.
  - Center: the latest X and Z scans, both directions (green forward, blue
    backward), the fitted Gaussian and the centre chosen, in yellow.
  - Angle: the power against each tilt with its parabola and the beam's
    walk per degree, and the lateral re-centring scans.
  - Peak: the readings at rest and those of the lateral gradient circles as
    a 3D landscape, with the centre of each lateral circle and pattern
    iteration on its floor; and every step in widths, in order (tilt
    circles blue, lateral circles pale green, pattern green). Its line
    counts the pattern's iterations and the circles, and gives the best
    reading at rest.
  - Colours span the dB range of the plot. A colour bar gives the values and
    marks the threshold, so small differences near the peak still show.

Everything comes from the hub. A page opened or reloaded after a run redraws it
from `align_result`. The page knows no phase by name: the algorithm declares
its phases and how to plot each (`PHASES`, `PHASE_VIEWS` in
`fiber_alignment/algorithm.py`), and publishes what the plots need with
`io.publish()`; `custom_view/__init__.py` documents the data.

## Testing without the cell

`app_logic/tests/fiber_alignment/sim.py` simulates the cell: a robot executing the
moves (answering pose queries the way the cell does), a power meter reading
what the fiber ends couple at the robot's pose (collimated ends like the
cell's, or bare single-mode fibers; late and smeared while the robot moves,
as a real reading is), on a simulated clock. From the repository root:

```
pip install -r requirements-dev.txt
python -m pytest app_logic/tests -q
```

checks the alignment from the cell's run of 2026-10-27, from a few µW, from
no light (tilt or lateral search), with the full search, on single-mode
fibers with a TCP off the tip, in the dark, and that what it publishes is
what the custom view plots. To watch a simulated run in the real custom view:

```
python app_logic/tools/view_replay.py            # then open http://127.0.0.1:8765/view/
python app_logic/tools/view_replay.py --step &   # or let a browser check it:
node app_logic/tools/view_check.mjs              # screenshots in test-results/view/
```

`--full-search` replays a full search (with `--offset 50 0 0.45 0`, from
light at the start), `--dark` a run with no light.

`view_check.mjs` needs Playwright and its Chromium (`npm i -g playwright`,
`npx playwright install chromium`).
