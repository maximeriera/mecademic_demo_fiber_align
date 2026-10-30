// plot3d.js — dependency-free 3D scatter + surface on a 2D <canvas>.
//
//   const camera = Plot3D.camera();                    // share it to rotate panels together
//   const plot = Plot3D.create(canvas, camera, {labels: {x: 'X', y: 'Y', z: 'Z'}, fontPx: 14});
//   plot.setAxes({x: [x0, x1], y: [y0, y1], z: [z0, z1]}, 300); // optional ms: animate to it
//   plot.setAxes(null);                                // no box: only the message
//   plot.setMessage('Waiting…');                       // shown while there is nothing to draw
//   plot.setLabels({x: 'X', y: 'Y', z: 'Z'});         // e.g. when the z unit changes
//   plot.setColor(z => 'rgb(...)');                    // surface and point colour, from height
//   plot.setPoints([[x, y, z], ...], {tail: 200});     // path; the last `tail` points stand out
//   plot.setSurface({x: [...], y: [...], z: [[...]]}); // mesh on grid nodes, null = hole
//   plot.setMarkers([{x, y, z, kind: 'best'}]);        // a star on the best point
//   plot.setFloorPath([[x, y], ...]);                  // a trail drawn on the floor
//   plot.setLegend({stops: [css, ...], low: 'text', high: 'text',
//                   marks: [{f: 0.6, text: 'threshold'}]});   // colour bar, top right; f in 0..1 left to right
//   Plot3D.animate([plot, ...], camera);              // auto-rotates (camera speed > 0); drag to orbit
//
// Redraws at most 1000 / FRAME_MS times a second, and a plot (or a chart handed
// to animate) only when it changed: new data or axes, the camera turned, a resize.
//
// Orthographic projection, painter's ordering: fine for a few thousand points.
(function (global) {
    'use strict';

    const AUTO_ROTATE_RAD_S = 0.25;
    const IDLE_AFTER_DRAG_MS = 4000;
    // The camera's pitch until dragged, which a tall box is fitted for.
    const DEFAULT_PITCH = 0.6;
    // Height of the drawn box: at least half its floor, up to a cube when the canvas has room.
    const BOX_Z_MIN = 0.5;
    const BOX_Z_MAX = 1;
    // Most points of a path drawn per frame before its tail.
    const PATH_POINTS_MAX = 3000;
    // Thickness of the colour bar.
    const LEGEND_BAR_PX = 10;
    // Shortest time between two redraws (ms): 30 a second keeps a slow turn smooth,
    // at half the cost of drawing every display frame.
    const FRAME_MS = 1000 / 30;

    function niceTicks(min, max, count) {
        const span = max - min;
        if (!(span > 0)) return { values: [min], step: 1 };
        const raw = span / count;
        const mag = Math.pow(10, Math.floor(Math.log10(raw)));
        const norm = raw / mag;
        const step = (norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10) * mag;
        const values = [];
        for (let v = Math.ceil(min / step) * step; v <= max + step * 1e-9; v += step) {
            values.push(+v.toFixed(10));
        }
        return { values, step };
    }

    function decimalsFor(step) {
        return Math.max(0, Math.min(4, -Math.floor(Math.log10(step))));
    }

    function camera(options) {
        return Object.assign({ yaw: -0.65, pitch: DEFAULT_PITCH, dragging: false, idleUntil: 0, speed: AUTO_ROTATE_RAD_S }, options);
    }

    function attachDrag(canvas, cam) {
        canvas.style.touchAction = 'none';
        canvas.style.cursor = 'grab';
        let last = null;
        canvas.addEventListener('pointerdown', e => {
            last = [e.clientX, e.clientY];
            cam.dragging = true;
            canvas.style.cursor = 'grabbing';
            canvas.setPointerCapture(e.pointerId);
        });
        canvas.addEventListener('pointermove', e => {
            if (!last) return;
            cam.yaw += (e.clientX - last[0]) * 0.01;
            // Below ~15° the floor collapses and its tick labels pile up.
            cam.pitch = Math.max(0.25, Math.min(1.5, cam.pitch + (e.clientY - last[1]) * 0.01));
            last = [e.clientX, e.clientY];
        });
        const end = () => {
            last = null;
            cam.dragging = false;
            cam.idleUntil = performance.now() + IDLE_AFTER_DRAG_MS;
            canvas.style.cursor = 'grab';
        };
        canvas.addEventListener('pointerup', end);
        canvas.addEventListener('pointercancel', end);
    }

    const easeOut = t => 1 - Math.pow(1 - t, 3);
    const sameRange = (a, b) => a && b && a[0] === b[0] && a[1] === b[1];

    function create(canvas, cam, options) {
        options = options || {};
        let labels = options.labels || { x: 'X', y: 'Y', z: 'Z' };
        const fontPx = options.fontPx || 11;
        const style = Object.assign({
            text: '#9aa39f',
            grid: 'rgba(255, 255, 255, 0.08)',
            edge: 'rgba(255, 255, 255, 0.22)',
            path: 'rgba(252, 253, 252, 0.14)',
            tail: '252, 253, 252',
            marker: '#FCFDFC',
            best: '#5DEFBF',
            floor: 'rgba(157, 245, 215, 0.85)',
        }, options.style);
        let axes = null;
        let anim = null;           // {from, to, start, ms}: an axes transition in progress
        let color = () => '#5DEFBF';
        let points = [];
        let tail = Infinity;
        let surface = null;
        let markers = [];
        let floorPath = [];
        let legend = null;
        let message = 'Waiting for data…';
        // Something to draw since the last render, and the camera that render used.
        let dirty = true;
        let drawnFrom = null;
        // Each surface quad's colour, computed at its first draw: the surface and
        // its colours do not change while the camera turns.
        let surfaceColors = null;

        attachDrag(canvas, cam);

        function setAxes(value, ms) {
            if (!value) {
                dirty = dirty || axes !== null;
                axes = null;
                anim = null;
                return;
            }
            const target = anim ? anim.to : axes;
            if (target && ['x', 'y', 'z'].every(k => sameRange(target[k], value[k]))) return;
            dirty = true;
            if (!axes || !ms) {
                axes = value;
                anim = null;
                return;
            }
            anim = { from: currentAxes(), to: value, start: performance.now(), ms };
        }

        function currentAxes() {
            if (!anim) return axes;
            const t = Math.min(1, (performance.now() - anim.start) / anim.ms);
            const e = easeOut(t);
            const lerp = k => anim.from[k].map((v, i) => v + e * (anim.to[k][i] - v));
            axes = t >= 1 ? anim.to : { x: lerp('x'), y: lerp('y'), z: lerp('z') };
            if (t >= 1) anim = null;
            return axes;
        }

        function render() {
            const dpr = window.devicePixelRatio || 1;
            const w = canvas.clientWidth;
            const h = canvas.clientHeight;
            if (!w || !h) return;  // hidden panel
            if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
                // Resizing clears the canvas.
                canvas.width = Math.round(w * dpr);
                canvas.height = Math.round(h * dpr);
                dirty = true;
            }
            const turned = !drawnFrom || drawnFrom[0] !== cam.yaw || drawnFrom[1] !== cam.pitch;
            if (!dirty && !anim && !(axes && turned)) return;  // the canvas still shows it
            dirty = false;
            drawnFrom = [cam.yaw, cam.pitch];
            const ctx = canvas.getContext('2d');
            ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
            ctx.clearRect(0, 0, w, h);
            ctx.font = `${fontPx}px Archivo, sans-serif`;
            currentAxes();

            if (!axes || (!points.length && !surface)) {
                ctx.fillStyle = style.text;
                ctx.textAlign = 'center';
                ctx.textBaseline = 'middle';
                ctx.fillText(message, w / 2, h / 2);
                if (!axes) return;
            }

            // Half-extents of the drawn box: X fixed, Y keeps the real aspect,
            // Z whatever the z range is: as tall as the canvas allows once the
            // floor fills its width, within BOX_Z_MIN..BOX_Z_MAX.
            const aspect = (axes.y[1] - axes.y[0]) / (axes.x[1] - axes.x[0]);
            const box = { x: 1, y: Math.max(0.3, Math.min(1.5, aspect)), z: BOX_Z_MIN };
            const cosY = Math.cos(cam.yaw), sinY = Math.sin(cam.yaw);
            const cosP = Math.cos(cam.pitch), sinP = Math.sin(cam.pitch);
            const reach = Math.hypot(box.x, box.y);
            const roomW = w - fontPx * 8;
            const roomH = h - fontPx * 5;
            const fitW = roomW / (2 * reach);
            const tallest = (roomH / (2 * fitW) - reach * Math.sin(DEFAULT_PITCH)) / Math.cos(DEFAULT_PITCH);
            box.z = Math.max(BOX_Z_MIN, Math.min(BOX_Z_MAX, tallest));
            const scale = Math.min(fitW, roomH / (2 * (reach * sinP + box.z * cosP)));
            const cx = w / 2;
            const cy = h / 2;

            // World → [screen x, screen y, depth]; larger depth = farther away.
            const P = (x, y, z) => {
                const X = ((x - axes.x[0]) / (axes.x[1] - axes.x[0]) * 2 - 1) * box.x;
                const Y = ((y - axes.y[0]) / (axes.y[1] - axes.y[0]) * 2 - 1) * box.y;
                const Z = ((z - axes.z[0]) / (axes.z[1] - axes.z[0]) * 2 - 1) * box.z;
                const xr = X * cosY - Y * sinY;
                const yr = X * sinY + Y * cosY;
                return [cx + xr * scale, cy - (Z * cosP + yr * sinP) * scale, yr * cosP - Z * sinP];
            };
            const line = (a, b) => {
                ctx.beginPath();
                ctx.moveTo(a[0], a[1]);
                ctx.lineTo(b[0], b[1]);
                ctx.stroke();
            };

            const [x0, x1] = axes.x, [y0, y1] = axes.y, [z0, z1] = axes.z;
            const xTicks = niceTicks(x0, x1, 5);
            const yTicks = niceTicks(y0, y1, 4);
            const zTicks = niceTicks(z0, z1, 4);

            // Floor grid and box edges.
            ctx.lineWidth = 1;
            ctx.strokeStyle = style.grid;
            xTicks.values.forEach(v => line(P(v, y0, z0), P(v, y1, z0)));
            yTicks.values.forEach(v => line(P(x0, v, z0), P(x1, v, z0)));
            ctx.strokeStyle = style.edge;
            [y0, y1].forEach(y => [z0, z1].forEach(z => line(P(x0, y, z), P(x1, y, z))));
            [x0, x1].forEach(x => [z0, z1].forEach(z => line(P(x, y0, z), P(x, y1, z))));
            [x0, x1].forEach(x => [y0, y1].forEach(y => line(P(x, y, z0), P(x, y, z1))));

            // Tick labels on the floor edges nearest the viewer, pushed outward.
            const centre = P((x0 + x1) / 2, (y0 + y1) / 2, z0);
            const label = (pt, text, push) => {
                const dx = pt[0] - centre[0], dy = pt[1] - centre[1];
                const len = Math.hypot(dx, dy) || 1;
                ctx.fillText(text, pt[0] + (dx / len) * push, pt[1] + (dy / len) * push);
            };
            ctx.fillStyle = style.text;
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            const yEdge = P(x0, y0, z0)[2] + P(x1, y0, z0)[2] < P(x0, y1, z0)[2] + P(x1, y1, z0)[2] ? y0 : y1;
            const xEdge = P(x0, y0, z0)[2] + P(x0, y1, z0)[2] < P(x1, y0, z0)[2] + P(x1, y1, z0)[2] ? x0 : x1;
            xTicks.values.forEach(v => label(P(v, yEdge, z0), v.toFixed(decimalsFor(xTicks.step)), fontPx * 1.3));
            yTicks.values.forEach(v => label(P(xEdge, v, z0), v.toFixed(decimalsFor(yTicks.step)), fontPx * 1.3));
            // Titles sit past the tick labels, clear of the middle tick.
            label(P((x0 + x1) / 2, yEdge, z0), labels.x, fontPx * 4);
            label(P(xEdge, (y0 + y1) / 2, z0), labels.y, fontPx * 4);

            // Z ticks on the vertical edge that is leftmost on screen. Its foot is
            // a floor corner that already carries tick labels, so skip low ticks.
            const corners = [[x0, y0], [x1, y0], [x0, y1], [x1, y1]];
            const [zx, zy] = corners.reduce((a, b) => (P(a[0], a[1], z0)[0] <= P(b[0], b[1], z0)[0] ? a : b));
            ctx.textAlign = 'right';
            zTicks.values
                .filter(v => v >= z0 + 0.15 * (z1 - z0))
                .forEach(v => {
                    const p = P(zx, zy, v);
                    ctx.fillText(v.toFixed(decimalsFor(zTicks.step)), p[0] - 10, p[1]);
                });
            const zTop = P(zx, zy, z1);
            ctx.textAlign = 'left';
            ctx.fillText(labels.z, zTop[0] - 6, zTop[1] - fontPx * 1.3);

            if (floorPath.length) drawFloorPath(ctx, P, z0);
            if (surface) drawSurface(ctx, P);
            if (points.length) drawPoints(ctx, P, z0);
            if (markers.length) drawMarkers(ctx, P);
            if (legend) drawLegend(ctx, w);
        }

        // A horizontal colour bar in the top right corner, above where the box
        // can reach: `low` and `high` label its ends, each mark is a tick at
        // f (0 = left end, 1 = right end) with its text under it.
        function drawLegend(ctx, w) {
            const y = fontPx * 0.4;
            const barW = Math.min(180, w * 0.3);
            ctx.textBaseline = 'middle';
            const x1 = w - ctx.measureText(legend.high || '').width - 10;
            const x0 = x1 - barW;
            const stops = legend.stops || [];
            const ramp = ctx.createLinearGradient(x0, 0, x1, 0);
            stops.forEach((c, i) => ramp.addColorStop(stops.length > 1 ? i / (stops.length - 1) : 0, c));
            ctx.fillStyle = ramp;
            ctx.fillRect(x0, y, barW, LEGEND_BAR_PX);
            ctx.strokeStyle = style.edge;
            ctx.lineWidth = 1;
            ctx.strokeRect(x0 + 0.5, y + 0.5, barW - 1, LEGEND_BAR_PX - 1);

            ctx.fillStyle = style.text;
            ctx.textAlign = 'left';
            ctx.fillText(legend.high || '', x1 + 6, y + LEGEND_BAR_PX / 2);
            ctx.textAlign = 'right';
            ctx.fillText(legend.low || '', x0 - 6, y + LEGEND_BAR_PX / 2);
            ctx.textAlign = 'center';
            ctx.textBaseline = 'top';
            (legend.marks || []).forEach(m => {
                const mx = x0 + m.f * barW;
                ctx.strokeStyle = style.marker;
                ctx.lineWidth = 2;
                line2(ctx, [mx, y - 3], [mx, y + LEGEND_BAR_PX + 3]);
                ctx.fillText(m.text, Math.max(x0, Math.min(x1, mx)), y + LEGEND_BAR_PX + 5);
            });
        }

        function drawSurface(ctx, P) {
            const xs = surface.x, ys = surface.y, zs = surface.z;
            if (!surfaceColors) surfaceColors = [];
            const quads = [];
            for (let j = 0; j + 1 < ys.length; j++) {
                for (let i = 0; i + 1 < xs.length; i++) {
                    const z = [zs[j][i], zs[j][i + 1], zs[j + 1][i + 1], zs[j + 1][i]];
                    if (z.some(v => v === null || v === undefined)) continue;
                    const p = [
                        P(xs[i], ys[j], z[0]), P(xs[i + 1], ys[j], z[1]),
                        P(xs[i + 1], ys[j + 1], z[2]), P(xs[i], ys[j + 1], z[3]),
                    ];
                    const k = j * xs.length + i;
                    const fill = surfaceColors[k] || (surfaceColors[k] = color((z[0] + z[1] + z[2] + z[3]) / 4));
                    quads.push({ p, depth: (p[0][2] + p[1][2] + p[2][2] + p[3][2]) / 4, fill });
                }
            }
            quads.sort((a, b) => b.depth - a.depth);
            ctx.lineWidth = 0.6;
            ctx.strokeStyle = 'rgba(12, 26, 15, 0.45)';
            quads.forEach(q => {
                ctx.fillStyle = q.fill;
                ctx.beginPath();
                ctx.moveTo(q.p[0][0], q.p[0][1]);
                for (let k = 1; k < 4; k++) ctx.lineTo(q.p[k][0], q.p[k][1]);
                ctx.closePath();
                ctx.fill();
                ctx.stroke();
            });
        }

        function drawPoints(ctx, P, zFloor) {
            const first = Math.max(0, points.length - tail);

            // The whole path, faint (every n-th point of a long one); then its
            // tail, brighter towards the latest point.
            const stride = Math.max(1, Math.ceil(first / PATH_POINTS_MAX));
            ctx.strokeStyle = style.path;
            ctx.lineWidth = 1;
            ctx.beginPath();
            for (let i = 0; i <= first; i = i < first && i + stride > first ? first : i + stride) {
                const s = P(points[i][0], points[i][1], points[i][2]);
                i ? ctx.lineTo(s[0], s[1]) : ctx.moveTo(s[0], s[1]);
            }
            ctx.stroke();
            const projected = points.slice(first).map(([x, y, z]) => ({ s: P(x, y, z), z }));
            for (let i = 1; i < projected.length; i++) {
                const a = projected[i - 1].s, b = projected[i].s;
                ctx.strokeStyle = `rgba(${style.tail}, ${(0.15 + 0.75 * i / projected.length).toFixed(3)})`;
                ctx.lineWidth = 1.5;
                line2(ctx, a, b);
            }

            [...projected]
                .sort((a, b) => b.s[2] - a.s[2])
                .forEach(({ s, z }) => {
                    ctx.fillStyle = color(z);
                    ctx.beginPath();
                    ctx.arc(s[0], s[1], 2.8, 0, Math.PI * 2);
                    ctx.fill();
                });

            // The current spot: a probe line down to the floor and a ring.
            const last = points[points.length - 1];
            const tip = projected[projected.length - 1].s;
            const foot = P(last[0], last[1], zFloor);
            ctx.strokeStyle = style.marker;
            ctx.lineWidth = 1;
            ctx.setLineDash([3, 3]);
            line2(ctx, foot, tip);
            ctx.setLineDash([]);
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(tip[0], tip[1], 7, 0, Math.PI * 2);
            ctx.stroke();
        }

        function drawMarkers(ctx, P) {
            markers.forEach(m => {
                const s = P(m.x, m.y, m.z);
                if (m.kind === 'best') {
                    star(ctx, s[0], s[1], 9, 4);
                    ctx.fillStyle = style.best;
                    ctx.fill();
                    ctx.strokeStyle = 'rgba(12, 26, 15, 0.9)';
                    ctx.lineWidth = 1.5;
                    ctx.stroke();
                }
            });
        }

        function drawFloorPath(ctx, P, zFloor) {
            const s = floorPath.map(([x, y]) => P(x, y, zFloor));
            ctx.strokeStyle = style.floor;
            ctx.fillStyle = style.floor;
            ctx.lineWidth = 2;
            ctx.beginPath();
            s.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
            ctx.stroke();
            s.forEach(p => {
                ctx.beginPath();
                ctx.arc(p[0], p[1], 3.5, 0, Math.PI * 2);
                ctx.fill();
            });
        }

        return {
            setAxes,
            setLabels(value) { labels = value; dirty = true; },
            setColor(fn) { color = fn; surfaceColors = null; dirty = true; },
            setPoints(value, opts) {
                points = value || [];
                tail = opts && opts.tail ? opts.tail : Infinity;
                dirty = true;
            },
            setSurface(value) { surface = value || null; surfaceColors = null; dirty = true; },
            setMarkers(value) { markers = value || []; dirty = true; },
            setFloorPath(value) { floorPath = value || []; dirty = true; },
            setLegend(value) { legend = value || null; dirty = true; },
            setMessage(text) { dirty = dirty || text !== message; message = text; },
            render,
        };
    }

    function line2(ctx, a, b) {
        ctx.beginPath();
        ctx.moveTo(a[0], a[1]);
        ctx.lineTo(b[0], b[1]);
        ctx.stroke();
    }

    function star(ctx, x, y, outer, inner) {
        ctx.beginPath();
        for (let k = 0; k < 10; k++) {
            const r = k % 2 ? inner : outer;
            const a = -Math.PI / 2 + (k * Math.PI) / 5;
            const px = x + r * Math.cos(a), py = y + r * Math.sin(a);
            k ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
        }
        ctx.closePath();
    }

    function animate(plots, cam) {
        const still = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
        let previous = performance.now();
        function frame(now) {
            requestAnimationFrame(frame);
            // A 2 ms margin: on a 60 Hz display, every other frame.
            if (now - previous < FRAME_MS - 2) return;
            const dt = Math.min(0.1, (now - previous) / 1000);
            previous = now;
            if (!still && !cam.dragging && now > cam.idleUntil) cam.yaw += dt * cam.speed;
            plots.forEach(plot => plot.render());
        }
        requestAnimationFrame(frame);
    }

    global.Plot3D = { camera, create, animate, niceTicks, decimalsFor };
})(window);
