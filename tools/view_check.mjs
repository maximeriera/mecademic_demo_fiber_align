// Drive a browser through app_logic/tools/view_replay.py and check that the custom view
// shows the alignment as it ran:
//
//   python app_logic/tools/view_replay.py --step &
//   node app_logic/tools/view_check.mjs [--url http://127.0.0.1:8765] [--out test-results/view]
//
// 1. One card per declared phase, in order, of the declared type, before the
//    first alignment already; the 3D landscape cards wider than a column of
//    charts, and every phase row inside the alignment card.
// 2. While a phase runs, its card is the active one and draws its readings,
//    and a map declared "until_end" (Acquire) keeps taking new readings.
// 3. The verdict shows the alignment's final power as a loss against the
//    source (dB), pass or fail, then the power, the source and the threshold
//    in dB; every phase is listed as done. The live tile shows the power in
//    dB against the source, the source and the threshold in dB.
// 4. Each card's line shows what the phase did (light found, a full search,
//    scans, iterations), the Acquire map is over the axes of the search that
//    found the light, and the peak card's best is the best reading at rest.
// 5. A page loaded after the run shows the same cards and numbers.
// 6. No error in the console.
// Saves a screenshot halfway through each phase and one of the end, for review.
// Needs Playwright (require('playwright')) with its Chromium.
import { createRequire } from 'module';
import fs from 'fs';
import path from 'path';

const require = createRequire(import.meta.url);
const { chromium } = require('playwright');

const args = process.argv.slice(2);
const option = (name, fallback) => {
    const i = args.indexOf(name);
    return i >= 0 && args[i + 1] ? args[i + 1] : fallback;
};
const URL_BASE = option('--url', 'http://127.0.0.1:8765');
const OUT = option('--out', 'test-results/view');
// Share of a canvas's pixels a drawn plot inks at least (a message alone inks far less).
const INKED = 0.03;

const failures = [];
function check(ok, message) {
    console.log(`${ok ? 'ok  ' : 'FAIL'} ${message}`);
    if (!ok) failures.push(message);
}

const api = async route => (await fetch(`${URL_BASE}${route}`)).json();
const plural = (n, word) => `${n} ${word}${n === 1 ? '' : 's'}`;
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));

async function until(predicate, timeoutMs, what) {
    const end = Date.now() + timeoutMs;
    for (;;) {
        const value = await predicate();
        if (value) return value;
        if (Date.now() > end) throw new Error(`Timed out waiting for ${what}`);
        await sleep(100);
    }
}

// As view.js formats power.
const UNITS = [{ scale: 1, name: 'W' }, { scale: 1e-3, name: 'mW' }, { scale: 1e-6, name: 'µW' }, { scale: 1e-9, name: 'nW' }];
function withUnit(watts) {
    const unit = UNITS.find(u => Math.abs(watts) >= u.scale) || UNITS[UNITS.length - 1];
    const v = watts / unit.scale;
    const a = Math.abs(v);
    return `${v.toFixed(a >= 100 ? 1 : a >= 10 ? 2 : 3)} ${unit.name}`;
}
// As view.js formats a loss against the source.
const signed = (v, digits) => `${v >= 0 ? '+' : '−'}${Math.abs(v).toFixed(digits)}`;
const dbText = db => `${signed(db, Math.abs(db) < 10 ? 2 : 1)} dB`;
const lossDb = (watts, source) => Math.max(-90, 10 * Math.log10(watts / source));

// What the page shows: the cards (phase, type, state, line, ink per canvas), the verdict and the phase list.
async function snapshot(page) {
    return page.evaluate(() => {
        const inked = canvas => {
            if (!canvas.width || !canvas.height) return 0;
            const data = canvas.getContext('2d').getImageData(0, 0, canvas.width, canvas.height).data;
            let n = 0;
            for (let i = 3; i < data.length; i += 16) if (data[i] > 0) n++;
            return n / (data.length / 16);
        };
        // The alignment card clips what overflows it: its last phase row must end inside it.
        const runCard = document.querySelector('.run-card');
        const rows = runCard.querySelectorAll('#phase-list li');
        const inside = runCard.getBoundingClientRect().bottom - parseFloat(getComputedStyle(runCard).borderBottomWidth);
        return {
            phasesFit: !rows.length || rows[rows.length - 1].getBoundingClientRect().bottom <= inside + 0.5,
            cards: [...document.querySelectorAll('#plots .plot-card')].map(card => ({
                phase: card.dataset.phase,
                view: card.dataset.view,
                width: card.getBoundingClientRect().width,
                active: card.classList.contains('active'),
                sub: card.querySelector('.panel-sub').textContent,
                axes: card.querySelector('canvas').dataset.axes || null,
                points: card.querySelector('canvas').dataset.points || null,
                ink: [...card.querySelectorAll('canvas')].map(inked),
            })),
            verdict: document.getElementById('verdict').textContent,
            verdictSub: document.getElementById('verdict-sub').textContent,
            tile: {
                value: document.getElementById('power-now').textContent,
                unit: document.getElementById('power-unit').textContent,
                watts: document.getElementById('power-watts').textContent,
                threshold: document.getElementById('threshold').textContent,
                thresholdW: document.getElementById('threshold-w').textContent,
            },
            phases: [...document.querySelectorAll('#phase-list li')].map(li => ({
                state: li.className, text: li.textContent,
            })),
        };
    });
}

async function main() {
    fs.mkdirSync(OUT, { recursive: true });
    const declared = await api('/api/replay');
    await api('/api/replay/restart');

    const browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 1920, height: 1080 } });
    const errors = [];
    page.on('pageerror', e => errors.push(String(e)));
    page.on('console', m => { if (m.type() === 'error') errors.push(m.text()); });
    await page.goto(`${URL_BASE}/view/`);

    // 1. The declared cards, before the run.
    await page.waitForFunction(n => document.querySelectorAll('#plots .plot-card').length === n,
        declared.phases.length, { timeout: 15000 });
    let shown = await snapshot(page);
    check(JSON.stringify(shown.cards.map(c => c.phase)) === JSON.stringify(declared.phases),
        `one card per phase, in order: ${shown.cards.map(c => c.phase).join(', ')}`);
    check(shown.cards.every(c => c.view === (declared.phase_views[c.phase] || { type: 'map' }).type),
        `each card of its declared type: ${shown.cards.map(c => `${c.phase}=${c.view}`).join(', ')}`);
    // The 3D landscapes get more room than a column of charts.
    const column = Math.min(...shown.cards.filter(c => c.view === 'profiles').map(c => c.width));
    shown.cards.filter(c => c.view === 'map' || c.view === 'peak').forEach(c => {
        check(c.width >= 1.3 * column, `${c.phase}: its landscape card is wider than a column of charts (${Math.round(c.width)} px vs ${Math.round(column)} px)`);
    });

    // 2. Halfway through each phase (the replay waits there with --step).
    const stepped = [];
    const mapCounts = {};  // readings on each "until_end" map, at the latest pause
    for (const [i, name] of declared.phases.entries()) {
        const status = await until(async () => {
            const s = await api('/api/replay');
            return (s.paused && s.phase === name) || s.done ? s : null;
        }, 120000, `phase ${name}`);
        if (status.done) break;
        stepped.push(name);
        await sleep(1200);  // a few polls and frames
        shown = await snapshot(page);
        const card = shown.cards[i];
        check(card.active && shown.cards.filter(c => c.active).length === 1, `${name}: its card is the active one`);
        check(Math.max(...card.ink) > INKED, `${name}: its card draws readings (ink ${card.ink.map(v => v.toFixed(3)).join('/')})`);
        check(shown.phasesFit, `${name}: every phase row fits in the alignment card`);
        // A map "until_end" keeps taking the readings of the later phases,
        // over its own axes (not a search's) once its phase is over.
        shown.cards.forEach((c, k) => {
            const view = declared.phase_views[c.phase] || {};
            if (!view.until_end || k > i) return;
            const count = Number(c.points);
            if (k < i) {
                check(count > mapCounts[c.phase], `${c.phase}: its map keeps updating during ${name} (${mapCounts[c.phase]} → ${count} readings)`);
                const axes = (view.axes || ['x', 'z']).join(',');
                check(c.axes === axes, `${c.phase}: during ${name}, its map is over ${c.axes} (its own axes: ${axes})`);
            }
            mapCounts[c.phase] = count;
        });
        await page.screenshot({ path: path.join(OUT, `${i + 1}-${name.toLowerCase()}.png`) });
        await api('/api/replay/resume');
    }
    check(stepped.length === declared.phases.length || declared.error,
        `the replay paused in every phase (${stepped.join(', ')}); start the replay with --step`);

    // 3. The verdict.
    const final = await until(async () => {
        const s = await api('/api/replay');
        return s.done ? s : null;
    }, 120000, 'the end of the run');
    await page.waitForFunction(() => /Aligned|Below threshold|Interrupted/.test(document.getElementById('verdict').textContent),
        null, { timeout: 15000 });
    await sleep(1500);
    shown = await snapshot(page);
    if (final.result) {
        const power = withUnit(final.result.power_w);
        const db = dbText(lossDb(final.result.power_w, final.source_power_w));
        check(shown.verdict.includes(db) && shown.verdict.includes(final.result.passed ? '✓ Aligned' : '✗ Below threshold'),
            `verdict "${shown.verdict}" matches the result (${final.result.passed ? 'passed' : 'failed'} at ${db})`);
        const threshold = `threshold ${dbText(final.threshold_db)}`;
        check(shown.verdictSub.includes(`${power} of ${withUnit(final.source_power_w)} source`) && shown.verdictSub.includes(threshold),
            `the power, the source and the ${threshold} shown: "${shown.verdictSub}"`);
        // The live tile: the latest reading against the source, and the threshold, in dB.
        const tile = shown.tile;
        check(tile.unit === 'dB' && /^[+−]\d+\.\d+$/.test(tile.value) && tile.watts.endsWith(`of ${withUnit(final.source_power_w)} source`),
            `live tile in dB against the source: "${tile.value} ${tile.unit}", "${tile.watts}"`);
        check(tile.threshold === dbText(final.threshold_db) && tile.thresholdW === `(${withUnit(final.threshold_w)})`,
            `live tile threshold "${tile.threshold} ${tile.thresholdW}"`);
        check(shown.phases.length === declared.phases.length && shown.phases.every(p => p.state === 'done'),
            `every phase listed as done: ${shown.phases.map(p => p.text).join(' | ')}`);
        check(shown.phasesFit, 'every phase row fits in the alignment card');
        check(shown.cards.every(c => !c.active), 'no card active after the run');
        check(shown.cards.every(c => Math.max(...c.ink) > INKED), `every card keeps its plot (ink ${shown.cards.map(c => Math.max(...c.ink).toFixed(3)).join('/')})`);

        // 4. Each card's line.
        shown.cards.forEach(card => {
            const counts = final.counts[card.phase] || {};
            if (card.view === 'map') {
                check(/light (found|at the start)/.test(card.sub), `${card.phase}: "${card.sub}" says where the light was found`);
                if (final.full_search) check(card.sub.startsWith('full search · '), `${card.phase}: "${card.sub}" says the search ran in full`);
                // After the run, over the map's own axes, where the later
                // phases climbed (a search's only while its phase ran).
                const view = declared.phase_views[card.phase] || {};
                const axes = (view.axes || ['x', 'z']).join(',');
                check(card.axes === axes, `${card.phase}: after the run, the map is over ${card.axes} (its own axes: ${axes})`);
                if (view.until_end) {
                    check(Number(card.points) === final.acquire_map_points,
                        `${card.phase}: the map plots every reading to the end of the run (${card.points} of ${final.acquire_map_points})`);
                }
            } else if (card.view === 'profiles') {
                const scans = plural(counts.profile || 0, 'scan');
                check(card.sub.startsWith(scans), `${card.phase}: "${card.sub}" counts its ${scans}`);
            } else if (card.view === 'peak') {
                const iterations = plural(counts.peak || 0, 'iteration');
                check(card.sub.startsWith(iterations), `${card.phase}: "${card.sub}" counts its ${iterations}`);
                const best = final.best_still;
                if (best && declared.phases[best[8]] === card.phase) {
                    const shown = dbText(lossDb(best[6], final.source_power_w));
                    check(card.sub.includes(`best ${shown}`), `${card.phase}: its best is the best reading at rest (${shown})`);
                }
            }
        });
    } else {
        check(shown.verdict.includes('Interrupted') && shown.verdictSub.includes(final.error.split(':')[0]),
            `verdict "${shown.verdict}" shows the error (${final.error})`);
    }
    await page.screenshot({ path: path.join(OUT, 'end.png') });

    // 5. The same after a reload, from align_result alone.
    await page.reload();
    await page.waitForFunction(() => /Aligned|Below threshold|Interrupted/.test(document.getElementById('verdict').textContent),
        null, { timeout: 15000 });
    await sleep(1500);
    const reloaded = await snapshot(page);
    const numbers = s => JSON.stringify({ cards: s.cards.map(c => [c.phase, c.view, c.sub]), verdict: s.verdict, sub: s.verdictSub, phases: s.phases });
    check(numbers(reloaded) === numbers(shown), 'a reloaded page shows the same cards and numbers');
    if (numbers(reloaded) !== numbers(shown)) console.log(`  live:     ${numbers(shown)}\n  reloaded: ${numbers(reloaded)}`);
    await page.screenshot({ path: path.join(OUT, 'reloaded.png') });

    // 6. The console.
    check(errors.length === 0, `no console error${errors.length ? `: ${errors.join(' | ')}` : ''}`);
    await browser.close();

    console.log(failures.length ? `\n${failures.length} check(s) failed` : `\nAll checks passed. Screenshots in ${OUT}/`);
    process.exit(failures.length ? 1 : 0);
}

main().catch(e => {
    console.error(e);
    process.exit(2);
});
