// Fiber alignment view: live power against the threshold, the power over
// time in dB, the alignment's progress and verdict, and one plot card per
// phase, side by side, laid out as the algorithm declares them
// (algorithm.PHASE_VIEWS, carried by align_start and align_result, and by
// align_phases before any run): "map" a 3D power landscape over two axes,
// "profiles" the latest 1-D scans of each axis, "peak" the peak search's
// readings at rest and its steps. The view knows no phase by name.
// Reads the hub through DataHubClient, the threshold from /api/context.
// Dependency-free canvas drawing (plot3d.js, chart2d.js, landscape.js): no CDN
// on a show floor.
(function () {
    'use strict';

    const THRESHOLD_PARAM = 'power_threshold_mw';
    const CONTEXT_POLL_MS = 1000;
    // The hub is polled every POLL_MS: the live tile shows each new reading
    // (custom_view.POWER_RATE_HZ). The run's cards and charts are rebuilt at
    // most every RUN_RENDER_MS, and only when the hub brought something new for them.
    const POLL_MS = 50;
    const RUN_RENDER_MS = 100;
    // What the page keeps of the hub's channels (s): a run lasts at most
    // align_timeout_s, and a reloaded page redraws the latest one from align_result.
    const HISTORY_S = 600;
    // A power reading older than this is stale: the hub stopped sampling.
    const STALE_S = 2;
    const EVENTS_SHOWN = 8;
    // Read from a distance on a 27" screen.
    const FONT_PX = 14;
    const AXES_ANIM_MS = 300;
    const SURFACE_REBUILD_MS = 250;
    // Earlier scans kept on a profile, fading.
    const PROFILES_SHOWN = 6;
    // Time slices of the power trace, about one per pixel column.
    const TRACE_SLICES = 1000;
    // Points drawn along a fitted curve.
    const CURVE_POINTS = 120;

    // A sample is [x, y, z, rx, ry, rz, power_w, kind, phase, t, still]: offsets
    // from the start in its tool frame, µm and degrees (AlignmentIO.row).
    const POWER = 6;
    const KIND = 7;
    const PHASE = 8;
    const T = 9;
    const STILL = 10;
    // The axes PHASE_VIEWS names, as offsets in a sample.
    const AXES = {
        x: { index: 0, name: 'X', unit: 'µm' },
        y: { index: 1, name: 'Y', unit: 'µm' },
        z: { index: 2, name: 'Z', unit: 'µm' },
        rx: { index: 3, name: 'Rx', unit: '°' },
        ry: { index: 4, name: 'Ry', unit: '°' },
        rz: { index: 5, name: 'Rz', unit: '°' },
    };
    // What the algorithm publishes besides the samples (io.publish names).
    const PUBLISHED = ['acquire', 'profile', 'peak', 'gradient'];
    // How a phase the algorithm declares no view for is plotted.
    const DEFAULT_VIEW = { type: 'map', axes: ['x', 'z'] };
    // Width of a card holding a 3D landscape (map, peak), against 1 for a column of charts.
    const LANDSCAPE_WIDTH = 1.5;

    const $ = id => document.getElementById(id);
    const css = getComputedStyle(document.documentElement);
    const token = (name, fallback) => css.getPropertyValue(name).trim() || fallback;
    const ACCENT = token('--color-accent', '#5DEFBF');
    const SOFT = token('--color-accent-soft', '#9DF5D7');
    const YELLOW = token('--color-yellow', '#ffd447');
    const BLUE = token('--color-blue', '#3bb8ff');
    // Power colours of the landscapes: a heat ramp whose lightness rises evenly
    // (OKLCH L 0.40 → 0.96), dim violet → magenta → coral → amber → pale
    // yellow. Each plot spreads it over the dB range it shows, so small
    // differences stay visible, and a colour bar gives the values.
    const HEAT = ['#3f3399', '#6f2fb7', '#a62cb8', '#d83a95', '#f36069', '#fd8b4a', '#feb251', '#fed756', '#fbf6a6'];
    // The ramp reaches at most this far under a plot's top reading, and spans at least this (dB).
    const HEAT_SPAN_DB = 40;
    const HEAT_MIN_SPAN_DB = 0.2;

    const isNum = v => typeof v === 'number' && Number.isFinite(v);
    const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
    const dbOf = (watts, ref) => (watts > 0 && ref > 0 ? 10 * Math.log10(watts / ref) : -60);
    const signed = (v, digits = 1) => `${v >= 0 ? '+' : '−'}${Math.abs(v).toFixed(digits)}`;
    const plural = (n, word) => `${n} ${word}${n === 1 ? '' : 's'}`;

    // HEAT, interpolated into 256 CSS colours.
    const HEAT_LUT = (() => {
        const rgb = HEAT.map(hex => [1, 3, 5].map(i => parseInt(hex.slice(i, i + 2), 16)));
        return Array.from({ length: 256 }, (_, k) => {
            const x = (k / 255) * (rgb.length - 1);
            const i = Math.min(rgb.length - 2, Math.floor(x));
            const c = rgb[i].map((v, j) => Math.round(v + (x - i) * (rgb[i + 1][j] - v)));
            return `rgb(${c[0]}, ${c[1]}, ${c[2]})`;
        });
    })();

    function maxOf(values, fallback = -Infinity) {
        let m = fallback;
        for (const v of values) if (isNum(v) && v > m) m = v;
        return m;
    }

    function minOf(values, fallback = Infinity) {
        let m = fallback;
        for (const v of values) if (isNum(v) && v < m) m = v;
        return m;
    }

    // [low, high] watts a plot's colours span: the dB range of `watts`, at
    // most HEAT_SPAN_DB deep and at least HEAT_MIN_SPAN_DB wide.
    function heatDomain(watts) {
        const hi = maxOf(watts);
        if (!(hi > 0)) return null;
        const lo = Math.max(hi * 10 ** (-HEAT_SPAN_DB / 10), minOf(watts.filter(w => w > 0), hi));
        const missing = HEAT_MIN_SPAN_DB - 10 * Math.log10(hi / lo);
        const pad = missing > 0 ? 10 ** (missing / 20) : 1;
        return [lo / pad, hi * pad];
    }

    // Where `watts` falls on the ramp of `domain`: 0 at its low end, 1 at its top.
    const heatFraction = (domain, watts) => (watts > 0 ? Math.log10(watts / domain[0]) / Math.log10(domain[1] / domain[0]) : 0);
    const heatColor = (domain, watts) => HEAT_LUT[Math.round(clamp(domain ? heatFraction(domain, watts) : 0, 0, 1) * 255)];

    // --- Power units --------------------------------------------------------------

    // The hub and the alignment carry watts; the page shows them with an SI
    // prefix picked from the value.
    const UNITS = [
        { scale: 1, name: 'W' },
        { scale: 1e-3, name: 'mW' },
        { scale: 1e-6, name: 'µW' },
        { scale: 1e-9, name: 'nW' },
    ];
    const unitFor = watts => UNITS.find(u => Math.abs(watts) >= u.scale) || UNITS[UNITS.length - 1];

    function fmt(watts, unit) {
        const v = watts / unit.scale;
        const a = Math.abs(v);
        return v.toFixed(a >= 100 ? 1 : a >= 10 ? 2 : 3);
    }

    function withUnit(watts) {
        if (!isNum(watts)) return '—';
        const unit = unitFor(watts);
        return `${fmt(watts, unit)} ${unit.name}`;
    }

    // --- Threshold, from /api/context ---------------------------------------------------

    let thresholdW = null;  // null until known (no context file, or no such param)

    function refreshContext() {
        fetch('/api/context')
            .then(r => r.json())
            .then(ctx => {
                const param = ctx.enabled && ctx.params ? ctx.params[THRESHOLD_PARAM] : null;
                thresholdW = param && isNum(param.value) ? param.value * 1e-3 : null;
            })
            .catch(() => { /* the hub poll reports a lost connection */ });
    }

    // --- Live power tile ------------------------------------------------------------------

    function renderPower(state) {
        const samples = state.channels.power;
        const last = samples.length ? samples[samples.length - 1] : null;
        const fresh = last && state.now !== null && state.now - last[0] <= STALE_S;
        const watts = fresh && isNum(last[1]) ? last[1] : null;
        const known = watts !== null && thresholdW !== null;

        if (watts !== null) {
            const unit = unitFor(watts);
            $('power-now').textContent = fmt(watts, unit);
            $('power-unit').textContent = unit.name;
        } else {
            $('power-now').textContent = '—';
            $('power-unit').textContent = '';
        }
        $('threshold').textContent = thresholdW !== null ? withUnit(thresholdW) : 'not set';
        $('margin').textContent = known && watts > 0 && thresholdW > 0 ? `(${signed(dbOf(watts, thresholdW))} dB)` : '';

        const card = $('power-card');
        card.classList.toggle('above', known && watts >= thresholdW);
        card.classList.toggle('below', known && watts < thresholdW);
        $('power-state').textContent = watts === null ? 'Waiting for the power meter…'
            : thresholdW === null ? `No threshold: add ${THRESHOLD_PARAM} to context.yaml`
            : watts >= thresholdW ? 'Above threshold' : 'Below threshold';
    }

    // --- The alignment being shown ------------------------------------------------------------

    const publishedOf = source => Object.fromEntries(PUBLISHED.map(name => [name, (source || {})[name] || []]));

    // The best reading taken at rest among `samples`, as a sample row.
    function bestStillOf(samples) {
        let best = null;
        samples.forEach(s => { if (s[STILL] && isNum(s[POWER]) && (!best || s[POWER] > best[POWER])) best = s; });
        return best;
    }

    function fromResult(result, t) {
        return {
            key: `result:${t}`,
            status: result.error ? 'error' : 'done',
            fiber: result.fiber,
            threshold_w: result.threshold_w,
            source_w: result.source_power_w,
            timeout_s: result.timeout_s,
            phases: result.phase_names || [],
            views: result.phase_views || {},
            samples: result.points || [],
            reads: result.reads,
            phaseLog: (result.phases || []).filter(p => p.t0 !== undefined),
            phaseSummary: result.phases || [],
            published: publishedOf(result.published),
            bestStill: result.best_still || null,
            elapsed: result.duration_s || 0,
            error: result.error,
            result: result.error ? null : result,
        };
    }

    // The alignment in progress, else the latest one: built from the hub's
    // events and samples, completed by its align_result summary once it ends.
    // The hub keeps only its latest samples, so a page (re)loaded during or
    // after a run lacks the start of it, which the summary holds (decimated,
    // but every reading taken at rest).
    function buildRun(state) {
        const events = state.events;
        let start = -1;
        for (let i = events.length - 1; i >= 0; i--) {
            if (events[i].name === 'align_start') { start = i; break; }
        }
        const results = state.channels.align_result;
        const lastResult = results.length ? results[results.length - 1] : null;
        if (start < 0) return lastResult ? fromResult(lastResult[1], lastResult[0]) : null;

        const begin = events[start];
        const after = events.slice(start + 1);
        const end = after.find(e => e.name === 'align_end');
        const t1 = end ? end.t : Infinity;
        const within = ([t]) => t >= begin.t && t <= t1;
        const values = channel => (state.channels[channel] || []).filter(within).map(([, v]) => v);

        const phaseLog = [];
        after.forEach(e => {
            if (e.t > t1) return;
            if (e.name === 'align_phase_start') {
                phaseLog.push({ name: e.data.phase, index: e.data.index, t0: e.data.t, status: 'running' });
            } else if (e.name === 'align_phase_end') {
                const open = phaseLog.filter(p => p.name === e.data.phase && p.status === 'running').pop();
                if (open) {
                    Object.assign(open, {
                        t1: e.data.t, status: e.data.status, reads: e.data.reads,
                        best_w: e.data.best_w, best_before_w: e.data.best_before_w,
                    });
                }
            }
        });

        const d = begin.data || {};
        const samples = values('align_sample');
        const run = {
            key: `run:${begin.t}`,
            status: end ? (end.data.error ? 'error' : 'done') : 'running',
            fiber: d.fiber,
            threshold_w: d.threshold_w,
            source_w: d.source_power_w,
            timeout_s: d.timeout_s,
            phases: d.phases || [],
            views: d.phase_views || {},
            samples,
            reads: null,
            phaseLog,
            phaseSummary: null,
            published: Object.fromEntries(PUBLISHED.map(name => [name, values(`align_${name}`)])),
            bestStill: bestStillOf(samples),
            elapsed: end ? end.t - begin.t : (state.now || begin.t) - begin.t,
            error: end ? end.data.error : null,
            result: null,
        };
        if (end) {
            // align_result is published right after align_end: until it
            // arrives, the end event's own data stands in for it.
            const summary = lastResult && lastResult[0] >= end.t ? lastResult[1] : null;
            if (summary) {
                run.phaseSummary = summary.phases;
                run.elapsed = summary.duration_s;
                run.reads = summary.reads;
                if (run.samples.length < summary.reads) run.samples = summary.points || [];
                const published = publishedOf(summary.published);
                PUBLISHED.forEach(name => {
                    if (published[name].length > run.published[name].length) run.published[name] = published[name];
                });
                run.bestStill = summary.best_still || run.bestStill;
            }
            if (!end.data.error) run.result = summary || { passed: end.data.passed, power_w: end.data.power_w };
        }
        return run;
    }

    // Each declared phase with its state: pending, running, done, timed out,
    // failed, skipped or not run.
    function phaseStates(run) {
        const names = run.phases.length ? run.phases : [...new Set(run.phaseLog.map(p => p.name))];
        const started = run.phaseLog.map(p => names.indexOf(p.name));
        return names.map((name, i) => {
            const log = run.phaseLog.filter(p => p.name === name).pop();
            if (log) return Object.assign({ name, index: i }, log);
            const summary = (run.phaseSummary || []).find(p => p.name === name);
            if (summary) return Object.assign({ name, index: i }, summary);
            const later = started.some(j => j > i);
            return { name, index: i, status: later ? 'skipped' : run.status === 'running' ? 'pending' : 'not run' };
        });
    }

    // Phases that took readings (or are taking them).
    const ran = phase => !!phase && ['running', 'done', 'timed out', 'failed'].includes(phase.status);

    const viewOf = (run, name) => run.views[name] || DEFAULT_VIEW;
    const inPhase = (list, phase) => list.filter(p => p.phase === phase.index);

    function gainDb(phase) {
        return phase.best_before_w > 0 && phase.best_w > 0 ? dbOf(phase.best_w, phase.best_before_w) : null;
    }

    // Readings taken: the summary's count once the run is over, as its points are decimated.
    const readsOf = run => (isNum(run.reads) ? run.reads : run.samples.length);

    // --- The alignment card: verdict, time budget, phases ---------------------------------------

    const PHASE_ICONS = { running: '●', done: '✓', 'timed out': '✓', failed: '✗', skipped: '–', pending: '○', 'not run': '·' };
    let phasesKey = '';

    function renderRun(run, states) {
        $('run-fiber').textContent = run.fiber ? `— fiber #${run.fiber}` : '— no fiber held';
        const verdict = $('verdict');
        const sub = $('verdict-sub');
        const r = run.result || {};
        if (run.status === 'running') {
            const best = maxOf(run.samples.map(s => s[POWER]));
            verdict.className = 'verdict live';
            verdict.textContent = 'Aligning…';
            sub.textContent = `${run.samples.length} readings${isNum(best) ? ` · best so far ${withUnit(best)}` : ''}`;
        } else if (run.status === 'error') {
            verdict.className = 'verdict ng';
            verdict.textContent = '✗ Interrupted';
            sub.textContent = run.error || '';
        } else {
            verdict.className = `verdict ${r.passed ? 'ok' : 'ng'}`;
            verdict.textContent = `${r.passed ? '✓ Aligned' : '✗ Below threshold'} · ${withUnit(r.power_w)}`;
            const coupling = r.power_w > 0 && run.source_w > 0 ? ` · ${signed(dbOf(r.power_w, run.source_w), 2)} dB vs source` : '';
            sub.textContent = `threshold ${withUnit(run.threshold_w)}${coupling} · ${readsOf(run)} readings`
                + (r.timed_out ? ' · timed out' : '');
        }
        sub.title = sub.textContent;

        const timeout = run.timeout_s || 0;
        const frac = timeout ? clamp(run.elapsed / timeout, 0, 1) : 0;
        $('budget-fill').style.width = `${(frac * 100).toFixed(1)}%`;
        $('budget-fill').classList.toggle('warn', frac > 0.8);
        $('budget-text').textContent = timeout ? `${run.elapsed.toFixed(1)} s / ${timeout} s` : `${run.elapsed.toFixed(1)} s`;

        renderPhaseList(run, states);
    }

    // What a running phase is doing, from what its card plots.
    function progress(run, phase) {
        const type = viewOf(run, phase.name).type;
        if (type === 'profiles') return `scan ${inPhase(run.published.profile, phase).length}`;
        if (type === 'peak') {
            // Its gradient circles come first, then the pattern's iterations.
            const iterations = inPhase(run.published.peak, phase).length;
            const circles = inPhase(run.published.gradient, phase).length;
            return iterations || !circles ? `iter ${iterations}` : `circle ${circles}`;
        }
        const acquire = inPhase(run.published.acquire, phase);
        const last = acquire[acquire.length - 1];
        return last && last.found ? 'light found' : `${run.samples.filter(s => s[PHASE] === phase.index).length} readings`;
    }

    // One row per phase: its state, what it gained (or is doing) and how long it took.
    function renderPhaseList(run, states) {
        const rows = states.map(p => {
            let metric = '';
            let time = '';
            if (p.status === 'running') {
                metric = progress(run, p);
                time = `${Math.max(0, run.elapsed - (p.t0 || 0)).toFixed(1)} s`;
            } else if (ran(p)) {
                const gain = gainDb(p);
                const found = p.status === 'failed' ? 'best' : 'found';
                metric = gain !== null ? `${signed(gain)} dB` : isNum(p.best_w) ? `${found} ${withUnit(p.best_w)}` : '';
                if (p.status === 'timed out') metric += `${metric ? ' · ' : ''}timed out`;
                time = `${((p.t1 || 0) - (p.t0 || 0)).toFixed(1)} s`;
            } else if (p.status !== 'pending') {
                metric = p.status;
            }
            return { cls: p.status.replace(' ', '-'), icon: PHASE_ICONS[p.status] || '·', name: p.name, metric, time };
        });
        const key = JSON.stringify(rows);
        if (key === phasesKey) return;
        phasesKey = key;
        $('phase-list').replaceChildren(...rows.map(row => {
            const li = document.createElement('li');
            li.className = row.cls;
            ['icon', 'name', 'metric', 'time'].forEach(field => {
                const span = document.createElement('span');
                span.className = `ph-${field}`;
                span.textContent = row[field];
                li.append(span);
            });
            return li;
        }));
    }

    // --- 3D power landscapes ----------------------------------------------------------------------

    // A Plot3D panel showing power over two pose axes: the readings' path, the
    // surface interpolated from them (height = linear power, colour = HEAT
    // over the surface's dB range, with a colour bar), the best reading, and
    // optionally a trail on the floor. Axes fit the data, only grow within
    // one `key`, and animate. Until the first reading, it shows only a message.
    function landscape(canvas, cam, message) {
        const plot = Plot3D.create(canvas, cam, { fontPx: FONT_PX });
        let unit = UNITS[1];
        let threshold = null;
        let key = null;
        let ranges = null;
        let built = { key: null, shape: '', at: 0 };
        let domain = null;  // [low, high] watts the colours span
        plot.setColor(z => heatColor(domain, z * unit.scale));
        plot.setMessage(message);

        function extent(values, minSpan) {
            let lo = minOf(values);
            let hi = maxOf(values);
            if (!isNum(lo) || !isNum(hi)) return [-minSpan / 2, minSpan / 2];
            const pad = Math.max((hi - lo) * 0.15, (minSpan - (hi - lo)) / 2, 0);
            return [lo - pad, hi + pad];
        }
        const union = (a, b) => [Math.min(a[0], b[0]), Math.max(a[1], b[1])];

        // Back to a message only.
        function clear(text) {
            key = null;
            ranges = null;
            built = { key: null, shape: '', at: 0 };
            plot.setAxes(null);
            plot.setPoints([]);
            plot.setSurface(null);
            plot.setMarkers([]);
            plot.setFloorPath([]);
            plot.setLegend(null);
            plot.setMessage(text);
        }

        return {
            plot,
            clear,
            update(o) {
                const pts = o.points;
                if (!pts.length) {
                    clear(o.message || 'Waiting for readings…');
                    return;
                }
                threshold = o.threshold;
                const r = { x: extent(pts.map(p => p[0]), o.minSpan), y: extent(pts.map(p => p[1]), o.minSpan) };
                ranges = key === o.key && ranges ? { x: union(ranges.x, r.x), y: union(ranges.y, r.y) } : r;
                key = o.key;
                // Up to the threshold once the readings come within a tenth of it:
                // before that (no light yet), the readings' own scale, at least
                // o.minTop (the level counted as light, so that noise stays flat).
                const peak = maxOf(pts.map(p => p[2]), 0);
                const top = Math.max(threshold > 0 && peak >= 0.1 * threshold ? Math.max(threshold, peak) : peak, o.minTop || 0);
                unit = unitFor(top || 1e-3);
                const zTop = ((top || 1e-3) / unit.scale) * 1.1;
                plot.setLabels({ x: `${o.names[0]} (${o.unit})`, y: `${o.names[1]} (${o.unit})`, z: `Power (${unit.name})` });
                plot.setAxes({ x: ranges.x, y: ranges.y, z: [0, zTop] }, AXES_ANIM_MS);
                // A reading under zero is the meter's noise: drawn on the floor.
                const pts3 = pts.map(([u, v, w]) => [u, v, Math.max(0, w) / unit.scale]);
                plot.setPoints(pts3, { tail: o.tail || 150 });

                // Rebuilt at once for a new key, else at most every
                // SURFACE_REBUILD_MS, when the readings or the axes changed.
                const shape = `${ranges.x}|${ranges.y}|${unit.name}|${pts3.length}`;
                const now = performance.now();
                if (o.key !== built.key || (shape !== built.shape && now - built.at > SURFACE_REBUILD_MS)) {
                    const surface = pts3.length >= 3
                        ? Landscape.build(pts3, ranges.x, ranges.y, { n: o.n || 40, sigmaCells: o.sigmaCells || 1.6, combine: o.combine || 'mean' })
                        : null;
                    plot.setSurface(surface);
                    // The colours span what the surface shows, else the readings
                    // (and up to o.minTop, as the height).
                    const shown = surface ? surface.z.flat().filter(isNum) : pts3.map(p => p[2]);
                    domain = heatDomain(shown.map(z => z * unit.scale).concat(o.minTop ? [o.minTop] : []));
                    // Marks on the colour bar: the threshold, and any level the phase uses (e.g. "light").
                    const marks = [{ watts: threshold, text: 'threshold' }].concat(o.marks || [])
                        .filter(m => domain && m.watts > domain[0] && m.watts < domain[1])
                        .map(m => ({ f: heatFraction(domain, m.watts), text: m.text }));
                    plot.setLegend(domain && { stops: HEAT, low: withUnit(domain[0]), high: withUnit(domain[1]), marks });
                    built = { key: o.key, shape, at: now };
                }
                let best = null;
                pts3.forEach(p => { if (!best || p[2] > best[2]) best = p; });
                plot.setMarkers(best ? [{ x: best[0], y: best[1], z: best[2], kind: 'best' }] : []);
                plot.setFloorPath(o.floorPath || []);
            },
        };
    }

    // [u, v, power] of `samples` along two axes (offsets in µm or degrees, as sampled).
    function project(samples, axes) {
        return samples.filter(s => isNum(s[POWER])).map(s => [s[axes[0].index], s[axes[1].index], s[POWER]]);
    }

    // All the landscapes turn together, when dragged. No auto-rotation: a
    // landscape redraws only when its data changes, it is dragged or resized
    // (a speed above 0, e.g. 0.15 rad/s, turns them all the time instead).
    const cam = Plot3D.camera({ speed: 0 });

    // --- 2D charts ---------------------------------------------------------------------------------

    const trace = Chart2D.create($('chart-trace'), { fontPx: FONT_PX, xLabel: 'time (s)', yLabel: 'dB vs threshold', message: 'Waiting for an alignment…' });

    // Everything drawn each frame: the trace, then the plot cards' plots (refilled when the cards change).
    const drawn = [trace];
    Plot3D.animate(drawn, cam);

    // The lowest and highest point of each of `buckets` time slices: a long
    // trace drawn fast, with its envelope kept.
    function envelope(points, buckets) {
        if (points.length <= buckets * 2) return points;
        const t0 = points[0][0];
        const span = points[points.length - 1][0] - t0 || 1;
        const out = [];
        let slice = -1, lo = null, hi = null;
        const flush = () => {
            if (!lo) return;
            if (lo === hi) out.push(lo);
            else out.push(...(lo[0] < hi[0] ? [lo, hi] : [hi, lo]));
        };
        points.forEach(p => {
            const k = Math.floor(((p[0] - t0) / span) * buckets);
            if (k !== slice) {
                flush();
                slice = k;
                lo = hi = p;
            } else if (p[1] < lo[1]) {
                lo = p;
            } else if (p[1] > hi[1]) {
                hi = p;
            }
        });
        flush();
        return out;
    }

    function renderTrace(run) {
        const thr = run.threshold_w;
        const points = run.samples.filter(s => isNum(s[POWER]) && isNum(s[T])).map(s => [s[T], dbOf(s[POWER], thr)]);
        // Best so far, as steps.
        const bestLine = [];
        let best = -Infinity;
        points.forEach(([t, db]) => {
            if (db <= best) return;
            if (bestLine.length) bestLine.push([t, best]);
            best = db;
            bestLine.push([t, best]);
        });
        if (points.length) bestLine.push([points[points.length - 1][0], best]);
        const dbs = points.map(p => p[1]);
        const lo = Math.max(-45, Math.min(-10, Math.floor((minOf(dbs, 0) - 2) / 5) * 5));
        // 10 dB over the threshold at least: room for the phase names and the threshold's label above its line.
        const hi = Math.max(10, Math.ceil((maxOf(dbs, 0) + 2) / 5) * 5);
        const tMax = Math.max(10, run.elapsed, maxOf(points.map(p => p[0]), 0)) * 1.05;
        const bands = run.phaseLog.map((p, i) => ({
            x0: p.t0,
            x1: isNum(p.t1) ? p.t1 : run.elapsed,
            label: p.name,
            color: i % 2 ? 'rgba(59, 184, 255, 0.07)' : 'rgba(93, 239, 191, 0.07)',
        }));
        const last = points[points.length - 1];
        trace.setRange({ x: [0, tMax], y: [lo, hi] }, AXES_ANIM_MS);
        trace.setData({
            series: [
                { points: envelope(points, TRACE_SLICES), color: SOFT, width: 1, alpha: 0.55 },
                { points: bestLine, color: ACCENT, width: 2.4 },
            ],
            bands,
            hlines: [{ y: 0, color: ACCENT, dash: [6, 4], label: 'threshold' }],
            markers: last && run.status === 'running' ? [{ x: last[0], y: last[1], color: YELLOW }] : [],
        });
    }

    // --- Plot cards: one per declared phase, side by side ------------------------------------------

    // What an empty plot says, from its phase's state.
    function idleMessage(phase, waiting) {
        if (!phase || phase.status === 'pending') return waiting;
        return phase.status === 'skipped' ? 'Skipped' : 'Not run';
    }

    // "312 readings · best 94.48 µW" for a card's title line. Once the phase
    // is over, its own count: the points of a reloaded run are decimated.
    // `still`: the best of the readings at rest only.
    function phaseNumbers(run, phase, count, still = false) {
        const samples = run.samples.filter(s => s[PHASE] === phase.index);
        const reads = phase.status !== 'running' && isNum(phase.reads) ? phase.reads : samples.length;
        const best = maxOf(samples.filter(s => !still || s[STILL]).map(s => s[POWER]));
        return `${count || `${reads} readings`}${isNum(best) ? ` · best ${withUnit(best)}` : ''}`;
    }

    // A "map" card: the phase's readings as a 3D landscape over two axes.
    // When the phase searches ("acquire" payloads with a "search"), the
    // landscape follows the latest search, over the axes the view declares
    // for it ("search_axes"), with the readings of its latest run only (a
    // search may run several times, centred elsewhere: "around"). The light
    // detection level is marked on the colour bar. A map "until_end" goes on
    // with the readings of the later phases, to the end of the run, and its
    // surface shows the best power seen at each spot.
    const SEARCH_NAMES = { angle: 'tilt search', lateral: 'lateral search' };

    function renderMap(view, name) {
        const canvas = document.createElement('canvas');
        const waiting = `Waiting for ${name}…`;
        const plot = landscape(canvas, cam, waiting);
        return {
            width: LANDSCAPE_WIDTH,
            nodes: [canvas],
            drawn: [plot.plot],
            update(run, phase) {
                if (!ran(phase)) {
                    plot.clear(idleMessage(phase, waiting));
                    return '';
                }
                const acquire = inPhase(run.published.acquire, phase);
                const last = acquire[acquire.length - 1];
                const search = last && last.search;
                const keys = (search && view.search_axes && view.search_axes[search]) || view.axes || DEFAULT_VIEW.axes;
                const axes = keys.map(a => AXES[a] || AXES.x);
                // The test reads which axes are shown.
                canvas.dataset.axes = keys.join(',');
                // Each run of a search starts with a payload not "found" yet.
                const runs = acquire.filter(a => a.search === search);
                const begun = runs.filter(a => !a.found);
                const since = !search ? -Infinity : begun.length ? begun[begun.length - 1].t : minOf(runs.map(a => a.t));
                // "until_end": the later phases' readings (and the verdict's) too.
                const shown = s => s[PHASE] === phase.index || (view.until_end && (s[PHASE] > phase.index || s[PHASE] < 0));
                const samples = run.samples.filter(s => s[T] >= since && shown(s));
                canvas.dataset.points = String(samples.length);
                plot.update({
                    key: `${run.key}:${name}:${keys.join(',')}:${since}`,
                    points: project(samples, axes),
                    names: axes.map(a => a.name), unit: axes[0].unit, minSpan: axes[0].unit === '°' ? 0.05 : 20,
                    threshold: run.threshold_w,
                    marks: last && isNum(last.detect_w) ? [{ watts: last.detect_w, text: 'light' }] : [],
                    minTop: last && isNum(last.detect_w) ? last.detect_w : 0,
                    // Readings at the same spot from different phases: the best power seen there.
                    combine: view.until_end ? 'max' : 'mean',
                });
                // Where the search is centred, unless at the start: "remembered pose", "ring 3/6".
                const around = a => (a.around && a.around !== 'start' ? ` · ${a.around}` : '');
                const how = (SEARCH_NAMES[search] || 'search') + (last ? around(last) : '');
                const atStart = last && last.around === 'remembered pose' ? 'light at the remembered pose' : 'light at the start';
                let light = !last ? null : last.found ? `light found · ${how}` : how;
                if (last && last.full) {
                    // A full search runs both spirals to the end, light at the start or not.
                    const first = acquire.find(a => a.found);
                    light = `full search · ${!last.found ? how : last.warm ? atStart
                        : `light found · ${SEARCH_NAMES[first.search] || 'search'}${around(first)}`}`;
                } else if (last && last.warm) {
                    light = atStart;
                }
                return phaseNumbers(run, phase, light && phase.status === 'running' && !last.found
                    ? `${light} · ${samples.length} readings` : light);
            },
        };
    }

    // The fitted curve of a profile, sampled across its span, in `unit`.
    function fitCurve(profile, unit) {
        const f = profile.fit;
        if (!f || !isNum(f.center) || !isNum(f.width) || !(f.width > 0) || !isNum(f.peak)) return null;
        const [lo, hi] = profile.span;
        const at = f.model === 'parabola'
            ? x => f.peak * Math.exp(-(((x - f.center) / f.width) ** 2))
            : x => (f.baseline || 0) + (f.peak - (f.baseline || 0)) * Math.exp((-2 * (x - f.center) ** 2) / f.width ** 2);
        return Array.from({ length: CURVE_POINTS + 1 }, (_, k) => {
            const x = lo + ((hi - lo) * k) / CURVE_POINTS;
            return [x, at(x) / unit.scale];
        });
    }

    function profileLabel(spec, p) {
        let text = `${spec.name} (${spec.unit}) · ${spec.unit === '°' ? 'round' : 'scan'} ${p.iteration + 1}`;
        if (p.rescan) text += ' · rescan';
        if (p.fit && isNum(p.fit.lag)) text += ` · lag ${Math.abs(p.fit.lag) < 0.005 ? '0.00' : p.fit.lag.toFixed(2)} µm`;
        if (isNum(p.walk_um_per_deg)) text += ` · walk ${Math.round(p.walk_um_per_deg)} µm/°`;
        return text;
    }

    // One profile chart: the latest scans along `axis` (newest bright: forward
    // green, backward blue), the fitted curve and the chosen point in yellow.
    function drawProfile(chart, axis, list, run, phase) {
        const spec = AXES[axis];
        if (!list.length) {
            chart.setRange(null);
            chart.setData({});
            chart.setMessage(ran(phase) ? `No ${spec.name} scan yet` : idleMessage(phase, `Waiting for ${spec.name} scans…`));
            return;
        }
        const latest = list[list.length - 1];
        const shown = list.slice(-PROFILES_SHOWN);
        const powers = shown.flatMap(p => p.passes.flatMap(q => q.points.map(pt => pt[1])));
        // A tilt changes the power by a few percent only: zoom on the readings.
        // A lateral scan crosses the whole peak: from 0, up to the threshold.
        const zoom = spec.unit === '°';
        const top = (zoom ? maxOf(powers, 0) : Math.max(run.threshold_w || 0, maxOf(powers, 0))) || 1e-3;
        const unit = unitFor(top);
        const low = zoom ? minOf(powers, top) : 0;
        const margin = zoom ? Math.max((top - low) * 0.2, top * 0.005) : top * 0.1;
        const series = [];
        shown.forEach((p, i) => {
            const newest = i === shown.length - 1;
            p.passes.forEach(q => series.push({
                points: q.points.filter(pt => isNum(pt[0]) && isNum(pt[1])).map(([c, w]) => [c, w / unit.scale]),
                color: newest ? (q.dir < 0 ? BLUE : ACCENT) : SOFT,
                alpha: newest ? 1 : 0.12 + (0.45 * (i + 1)) / shown.length,
                width: newest ? 2 : 1.2,
                dots: newest,
            }));
        });
        const curve = fitCurve(latest, unit);
        if (curve) series.push({ points: curve, color: YELLOW, width: 1.6, alpha: 0.9 });
        const [lo, hi] = latest.span;
        const pad = (hi - lo) * 0.04 || 1;
        chart.setLabels({ x: profileLabel(spec, latest), y: `Power (${unit.name})` });
        chart.setRange({ x: [lo - pad, hi + pad], y: [Math.max(0, low - margin) / unit.scale, (top + margin) / unit.scale] }, AXES_ANIM_MS);
        chart.setData({
            series,
            vlines: isNum(latest.chosen) ? [{ x: latest.chosen, color: YELLOW, dash: [4, 4] }] : [],
            hlines: run.threshold_w ? [{ y: run.threshold_w / unit.scale, color: ACCENT, dash: [6, 4], label: 'threshold' }] : [],
        });
    }

    // A "profiles" card: one chart per declared slot (axis), each with the
    // phase's latest scans along it. More than two slots take two columns.
    function renderProfiles(view) {
        const slots = (view.slots || []).filter(axis => AXES[axis]);
        const grid = document.createElement('div');
        grid.className = 'panel panel-profiles';
        grid.dataset.slots = String(slots.length);
        const charts = slots.map(axis => {
            const canvas = document.createElement('canvas');
            grid.append(canvas);
            return { axis, chart: Chart2D.create(canvas, { fontPx: FONT_PX - 1, message: `Waiting for ${AXES[axis].name} scans…` }) };
        });
        return {
            width: slots.length > 2 ? 2 : 1,
            nodes: [grid],
            drawn: charts.map(c => c.chart),
            update(run, phase) {
                const scans = ran(phase) ? inPhase(run.published.profile, phase) : [];
                charts.forEach(({ axis, chart }) => drawProfile(chart, axis, scans.filter(p => p.axis === axis), run, phase));
                if (!ran(phase)) return '';
                return phaseNumbers(run, phase, phase.status === 'running' ? `scan ${scans.length}` : plural(scans.length, 'scan'));
            },
        };
    }

    // A "peak" card: the phase's readings at rest, with those of its lateral
    // gradient circles ("gradient" payloads, kind "cart"), as a 3D landscape;
    // the centre of each lateral circle then of each pattern iteration as a
    // trail on its floor; and every step, in widths (lateral or tilt), in
    // the order taken: the circles' first, then the pattern's.
    const STEP_SERIES = [
        { key: 'angle', text: 'tilt circles', color: BLUE },
        { key: 'lateral', text: 'lateral circles', color: SOFT },
        { key: 'pattern', text: 'pattern', color: ACCENT },
    ];

    function renderPeak(view, name) {
        const axes = (view.axes || DEFAULT_VIEW.axes).map(a => AXES[a] || AXES.x);
        const body = document.createElement('div');
        body.className = 'panel peak-body';
        const canvas = document.createElement('canvas');
        canvas.className = 'peak-map';
        const title = document.createElement('div');
        title.className = 'sub-title';
        title.append('Steps, in widths: ');
        STEP_SERIES.forEach((s, i) => {
            const key = document.createElement('span');
            key.style.color = s.color;
            key.textContent = `${i ? ' · ' : ''}${s.text}`;
            title.append(key);
        });
        const stepsCanvas = document.createElement('canvas');
        stepsCanvas.className = 'peak-steps';
        body.append(canvas, title, stepsCanvas);
        const waiting = `Waiting for ${name}…`;
        const plot = landscape(canvas, cam, waiting);
        const chart = Chart2D.create(stepsCanvas, {
            fontPx: FONT_PX - 1, xLabel: 'step', yLabel: 'step (widths)', message: 'No step yet',
        });
        return {
            width: LANDSCAPE_WIDTH,
            nodes: [body],
            drawn: [plot.plot, chart],
            update(run, phase) {
                if (!ran(phase)) {
                    plot.clear(idleMessage(phase, waiting));
                    chart.setRange(null);
                    chart.setData({});
                    chart.setMessage(idleMessage(phase, 'No step yet'));
                    return '';
                }
                chart.setMessage('No step yet');
                const circles = inPhase(run.published.gradient, phase);
                const steps = inPhase(run.published.peak, phase);
                // Lateral moves, in x/z: the lateral circles', then the pattern's.
                const moves = circles.filter(c => c.search === 'lateral').concat(steps);
                const trail = moves.map(p => p.center);
                const last = moves[moves.length - 1];
                if (last && Array.isArray(last.step)) trail.push([last.center[0] + last.step[0], last.center[1] + last.step[1]]);
                const lastStep = steps[steps.length - 1];
                const lastCircle = circles.filter(c => c.search === 'lateral').pop();
                plot.update({
                    key: `${run.key}:${name}`,
                    points: project(run.samples.filter(s => s[PHASE] === phase.index && (s[STILL] || s[KIND] === 'cart')), axes),
                    names: axes.map(a => a.name), unit: axes[0].unit, minSpan: 4,
                    // The latest pattern (or circle) stands out, whatever its size.
                    threshold: run.threshold_w, n: 24, sigmaCells: 2.2,
                    tail: lastStep && lastStep.points ? lastStep.points.length : lastCircle ? lastCircle.readings : 9,
                    floorPath: trail,
                });
                const series = Object.fromEntries(STEP_SERIES.map(s => [s.key, []]));
                circles.concat(steps).forEach((p, i) => {
                    const list = series[p.search || 'pattern'];
                    if (list && isNum(p.step_widths)) list.push([i + 1, p.step_widths]);
                });
                const count = circles.length + steps.length;
                const top = maxOf(STEP_SERIES.flatMap(s => series[s.key].map(p => p[1])), 0);
                chart.setRange({ x: [0.5, Math.max(2, count) + 0.5], y: [0, Math.max(0.05, top) * 1.2] }, AXES_ANIM_MS);
                chart.setData({ series: STEP_SERIES.map(s => ({ points: series[s.key], color: s.color, width: 2, dots: true })) });
                const iterations = phase.status === 'running' ? `iter ${steps.length}` : plural(steps.length, 'iteration');
                return phaseNumbers(run, phase, `${iterations}${circles.length ? ` · ${plural(circles.length, 'circle')}` : ''}`, true);
            },
        };
    }

    // Plot cards by view type (algorithm.PHASE_VIEWS); app_logic/tests/fiber_alignment/test_contract.py reads this table.
    const RENDERERS = {
        map: renderMap,
        profiles: renderProfiles,
        peak: renderPeak,
    };

    let cards = [];
    let layoutKey = '';

    // One card per phase, in order, each built by its view's renderer.
    function layout(phases, views) {
        const key = JSON.stringify([phases, views]);
        if (key === layoutKey) return;
        layoutKey = key;
        cards = phases.map(name => {
            const view = views[name] || DEFAULT_VIEW;
            const body = (RENDERERS[view.type] || RENDERERS[DEFAULT_VIEW.type])(view, name);
            const el = document.createElement('section');
            el.className = 'card plot-card';
            el.dataset.phase = name;
            el.dataset.view = view.type;
            // Its share of the row; two columns' worth stacks as a wide card on narrow screens.
            const width = body.width || 1;
            if (width >= 2) el.classList.add('wide');
            const head = document.createElement('div');
            head.className = 'panel-head';
            const title = document.createElement('span');
            title.className = 'panel-title';
            title.textContent = name;
            const sub = document.createElement('span');
            sub.className = 'panel-sub';
            head.append(title, sub);
            el.append(head, ...body.nodes);
            return { name, el, sub, width, drawn: body.drawn, update: body.update };
        });
        const row = $('plots');
        row.replaceChildren(...cards.map(c => c.el));
        row.style.setProperty('--plot-columns', cards.map(c => `minmax(0, ${c.width}fr)`).join(' ') || 'minmax(0, 1fr)');
        drawn.splice(1, drawn.length - 1, ...cards.flatMap(c => c.drawn));
    }

    function renderPlots(run, states) {
        layout(run.phases, run.views);
        cards.forEach(card => {
            const phase = states.find(p => p.name === card.name) || null;
            card.el.classList.toggle('active', !!phase && phase.status === 'running');
            card.sub.textContent = card.update(run, phase) || '';
        });
    }

    // --- Events and hub status -------------------------------------------------------------------------

    let epochMs = null;

    function describe(event) {
        const data = event.data || {};
        let text = event.name;
        if (data.phase !== undefined) text += ` · ${data.phase}`;
        if (data.cycle !== undefined) text += ` #${data.cycle}`;
        if (data.fiber !== undefined) text += data.fiber ? ` · fiber #${data.fiber}` : ' · no fiber';
        if (data.error) text += ` — ${data.error}`;
        else if (data.passed !== undefined) text += data.passed ? ' · passed' : ' · below threshold';
        return text;
    }

    function renderEvents(state) {
        const events = state.events.slice(-EVENTS_SHOWN).reverse();
        if (!events.length) return;
        $('events').replaceChildren(...events.map(e => {
            const li = document.createElement('li');
            const name = document.createElement('span');
            name.textContent = describe(e);
            const time = document.createElement('span');
            time.className = 't';
            time.textContent = epochMs === null
                ? `${e.t.toFixed(2)} s`
                : new Date(epochMs + e.t * 1000).toLocaleTimeString([], { hour12: false });
            li.append(name, time);
            return li;
        }));
    }

    function refreshManifest() {
        fetch('/api/data')
            .then(r => r.json())
            .then(info => {
                if (info.epoch_utc) epochMs = Date.parse(info.epoch_utc);
                $('setup-error').hidden = !info.setup_error;
                $('setup-error').textContent = info.setup_error ? `Custom view setup failed: ${info.setup_error}` : '';
                const sources = Object.values(info.channels || {}).filter(c => c.rate_hz);
                const errors = sources.reduce((n, c) => n + (c.errors || 0), 0);
                $('view-status').textContent = info.active
                    ? `Live · ${sources.length} source(s) sampled` + (errors ? ` · ${errors} read error(s)` : '')
                    : 'Hub not running — Initialize the controller.';
            })
            .catch(() => { $('view-status').textContent = 'Connection lost — retrying…'; });
    }

    // --- Main loop ------------------------------------------------------------------------------------------

    // The channels the run's cards are drawn from (the live tile reads `power`).
    const RUN_CHANNELS = ['events', 'align_phases', 'align_sample', 'align_result', ...PUBLISHED.map(name => `align_${name}`)];
    let runStamp = null;
    let runRenderedAt = -Infinity;

    // Whether an alignment is running: the latest align_start has no align_end after it.
    function aligning(state) {
        const events = state.channels.events;
        for (let i = events.length - 1; i >= 0; i--) {
            const name = events[i][1] && events[i][1].name;
            if (name === 'align_end') return false;
            if (name === 'align_start') return true;
        }
        return false;
    }

    // What the run's cards depend on: each channel's latest sample, the clock's
    // origin (event times), and the clock itself while an alignment runs (its
    // time and budget move even between readings).
    function runStampOf(state) {
        const ends = RUN_CHANNELS.map(name => {
            const samples = state.channels[name] || [];
            return samples.length ? samples[samples.length - 1][0] : null;
        });
        return JSON.stringify([ends, epochMs, aligning(state) ? state.now : null]);
    }

    function render(state) {
        renderPower(state);
        // The rest only when it changed, and not more often than RUN_RENDER_MS:
        // a change skipped here is drawn at a later poll.
        const now = performance.now();
        if (now - runRenderedAt < RUN_RENDER_MS) return;
        const stamp = runStampOf(state);
        if (stamp === runStamp) return;
        runStamp = stamp;
        runRenderedAt = now;
        renderEvents(state);
        const run = buildRun(state);
        if (!run) {
            // No alignment yet: the cards of the phases the algorithm declares, waiting.
            const declared = state.latest('align_phases');
            if (declared) layout(declared.phases || [], declared.phase_views || {});
            return;
        }
        const states = phaseStates(run);
        renderRun(run, states);
        renderTrace(run);
        renderPlots(run, states);
    }

    refreshManifest();
    setInterval(refreshManifest, 5000);
    refreshContext();
    setInterval(refreshContext, CONTEXT_POLL_MS);

    DataHubClient.poll({
        channels: ['power', ...RUN_CHANNELS.filter(name => name !== 'events')],
        intervalMs: POLL_MS,
        historyS: HISTORY_S,
        onData: render,
        onError: () => { $('view-status').textContent = 'Connection lost — retrying…'; },
    });
})();
