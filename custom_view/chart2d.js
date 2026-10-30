// chart2d.js — dependency-free 2D line chart on a <canvas>, for the custom view.
//
//   const chart = Chart2D.create(canvas, {fontPx: 14, xLabel: 't (s)', yLabel: 'dB'});
//   chart.setRange({x: [x0, x1], y: [y0, y1]}, 300);   // optional ms: animate to it
//   chart.setRange(null);                               // no axes: only the message
//   chart.setData({
//       series:  [{points: [[x, y], ...], color, width, dots}],
//       bands:   [{x0, x1, label, color}],               // shaded x ranges, e.g. phases
//       hlines:  [{y, color, dash, label}],              // e.g. the threshold
//       vlines:  [{x, color, dash}],
//       markers: [{x, y, color}],                        // rings, e.g. the latest point
//   });
//   chart.render();   // draws only what changed since the last call (data, range, size);
//                     // or hand it to Plot3D.animate() with the 3D plots
(function (global) {
    'use strict';

    const easeOut = t => 1 - Math.pow(1 - t, 3);

    function niceTicks(min, max, count) {
        const span = max - min;
        if (!(span > 0)) return { values: [min], step: 1 };
        const raw = span / count;
        const mag = Math.pow(10, Math.floor(Math.log10(raw)));
        const norm = raw / mag;
        const step = (norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10) * mag;
        const values = [];
        for (let v = Math.ceil(min / step) * step; v <= max + step * 1e-9; v += step) values.push(+v.toFixed(10));
        return { values, step };
    }

    const decimalsFor = step => Math.max(0, Math.min(4, -Math.floor(Math.log10(step))));

    function create(canvas, options) {
        options = options || {};
        const fontPx = options.fontPx || 13;
        const style = Object.assign({
            text: '#9aa39f',
            grid: 'rgba(255, 255, 255, 0.07)',
            axis: 'rgba(255, 255, 255, 0.22)',
        }, options.style);
        let labels = { x: options.xLabel || '', y: options.yLabel || '' };
        let message = options.message || 'Waiting for data…';
        let data = { series: [], bands: [], hlines: [], vlines: [], markers: [] };
        let range = null;
        let anim = null;
        // Something to draw since the last render (a range animation draws every frame).
        let dirty = true;

        function setRange(value, ms) {
            if (!value) {
                dirty = dirty || range !== null;
                range = null;
                anim = null;
                return;
            }
            const target = anim ? anim.to : range;
            const same = target && ['x', 'y'].every(k => target[k][0] === value[k][0] && target[k][1] === value[k][1]);
            if (same) return;
            dirty = true;
            if (!range || !ms) {
                range = value;
                anim = null;
                return;
            }
            anim = { from: current(), to: value, start: performance.now(), ms };
        }

        function current() {
            if (!anim) return range;
            const t = Math.min(1, (performance.now() - anim.start) / anim.ms);
            const e = easeOut(t);
            const lerp = k => anim.from[k].map((v, i) => v + e * (anim.to[k][i] - v));
            range = t >= 1 ? anim.to : { x: lerp('x'), y: lerp('y') };
            if (t >= 1) anim = null;
            return range;
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
            if (!dirty && !anim) return;  // nothing changed: the canvas still shows it
            dirty = false;
            const ctx = canvas.getContext('2d');
            ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
            ctx.clearRect(0, 0, w, h);
            ctx.font = `${fontPx}px Archivo, sans-serif`;
            current();
            const hasData = data.series.some(s => s.points && s.points.length);
            if (!range || !hasData) {
                ctx.fillStyle = style.text;
                ctx.textAlign = 'center';
                ctx.textBaseline = 'middle';
                ctx.fillText(message, w / 2, h / 2);
                if (!range) return;
            }

            const pad = { l: fontPx * 4.2, r: fontPx * 1.2, t: fontPx * 1.6, b: fontPx * 3 };
            const pw = w - pad.l - pad.r;
            const ph = h - pad.t - pad.b;
            if (pw <= 0 || ph <= 0) return;
            const [x0, x1] = range.x;
            const [y0, y1] = range.y;
            const sx = x => pad.l + ((x - x0) / (x1 - x0)) * pw;
            const sy = y => pad.t + (1 - (y - y0) / (y1 - y0)) * ph;

            // Bands (behind everything), labelled at the top.
            ctx.save();
            ctx.beginPath();
            ctx.rect(pad.l, pad.t, pw, ph);
            ctx.clip();
            (data.bands || []).forEach(b => {
                const a = Math.max(pad.l, sx(b.x0));
                const z = Math.min(pad.l + pw, sx(b.x1));
                if (z <= a) return;
                ctx.fillStyle = b.color || 'rgba(255, 255, 255, 0.04)';
                ctx.fillRect(a, pad.t, z - a, ph);
                if (b.label && z - a > fontPx * 2) {
                    ctx.fillStyle = style.text;
                    ctx.textAlign = 'left';
                    ctx.textBaseline = 'top';
                    ctx.fillText(b.label, a + 4, pad.t + 3, z - a - 6);
                }
            });
            ctx.restore();

            // Grid and tick labels.
            ctx.strokeStyle = style.grid;
            ctx.fillStyle = style.text;
            ctx.lineWidth = 1;
            const yt = niceTicks(y0, y1, 4);
            ctx.textAlign = 'right';
            ctx.textBaseline = 'middle';
            yt.values.forEach(v => {
                const y = Math.round(sy(v)) + 0.5;
                ctx.beginPath();
                ctx.moveTo(pad.l, y);
                ctx.lineTo(pad.l + pw, y);
                ctx.stroke();
                ctx.fillText(v.toFixed(decimalsFor(yt.step)), pad.l - 6, y);
            });
            const xt = niceTicks(x0, x1, Math.max(2, Math.floor(pw / (fontPx * 6))));
            ctx.textAlign = 'center';
            ctx.textBaseline = 'top';
            xt.values.forEach(v => ctx.fillText(v.toFixed(decimalsFor(xt.step)), sx(v), pad.t + ph + 5));
            ctx.strokeStyle = style.axis;
            ctx.strokeRect(pad.l + 0.5, pad.t + 0.5, pw, ph);
            if (labels.x) ctx.fillText(labels.x, pad.l + pw / 2, pad.t + ph + fontPx * 1.6);
            if (labels.y) {
                ctx.save();
                ctx.translate(fontPx * 0.9, pad.t + ph / 2);
                ctx.rotate(-Math.PI / 2);
                ctx.textBaseline = 'middle';
                ctx.fillText(labels.y, 0, 0);
                ctx.restore();
            }

            ctx.save();
            ctx.beginPath();
            ctx.rect(pad.l, pad.t, pw, ph);
            ctx.clip();
            (data.hlines || []).forEach(l => {
                ctx.strokeStyle = l.color || style.axis;
                ctx.lineWidth = l.width || 1.5;
                ctx.setLineDash(l.dash || []);
                const y = sy(l.y);
                ctx.beginPath();
                ctx.moveTo(pad.l, y);
                ctx.lineTo(pad.l + pw, y);
                ctx.stroke();
                ctx.setLineDash([]);
                if (l.label) {
                    // Above the line, or under it when the line runs too close to the top.
                    const above = y - pad.t >= fontPx + 2;
                    ctx.fillStyle = l.color || style.text;
                    ctx.textAlign = 'right';
                    ctx.textBaseline = above ? 'bottom' : 'top';
                    ctx.fillText(l.label, pad.l + pw - 4, above ? y - 3 : y + 3);
                }
            });
            (data.vlines || []).forEach(l => {
                ctx.strokeStyle = l.color || style.axis;
                ctx.lineWidth = l.width || 1;
                ctx.setLineDash(l.dash || []);
                const x = sx(l.x);
                ctx.beginPath();
                ctx.moveTo(x, pad.t);
                ctx.lineTo(x, pad.t + ph);
                ctx.stroke();
                ctx.setLineDash([]);
            });
            (data.series || []).forEach(s => {
                const pts = s.points || [];
                if (!pts.length) return;
                ctx.globalAlpha = s.alpha === undefined ? 1 : s.alpha;
                ctx.strokeStyle = s.color || '#5DEFBF';
                ctx.lineWidth = s.width || 1.6;
                ctx.beginPath();
                pts.forEach(([x, y], i) => (i ? ctx.lineTo(sx(x), sy(y)) : ctx.moveTo(sx(x), sy(y))));
                ctx.stroke();
                if (s.dots) {
                    ctx.fillStyle = s.color || '#5DEFBF';
                    pts.forEach(([x, y]) => {
                        ctx.beginPath();
                        ctx.arc(sx(x), sy(y), s.dotRadius || 2.2, 0, Math.PI * 2);
                        ctx.fill();
                    });
                }
                ctx.globalAlpha = 1;
            });
            (data.markers || []).forEach(m => {
                ctx.strokeStyle = m.color || '#FCFDFC';
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.arc(sx(m.x), sy(m.y), m.radius || 6, 0, Math.PI * 2);
                ctx.stroke();
            });
            ctx.restore();
        }

        return {
            setRange,
            setData(value) {
                data = Object.assign({ series: [], bands: [], hlines: [], vlines: [], markers: [] }, value);
                dirty = true;
            },
            setLabels(value) {
                labels = Object.assign(labels, value);
                dirty = true;
            },
            setMessage(text) {
                dirty = dirty || text !== message;
                message = text;
            },
            render,
        };
    }

    global.Chart2D = { create };
})(window);
