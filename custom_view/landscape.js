// landscape.js — scattered power readings → a regular grid, for Plot3D.setSurface.
//
//   const surface = Landscape.build(points, [x0, x1], [y0, y1], {n: 40, sigmaCells: 1.6});
//   plot.setSurface(surface);   // {x: [...], y: [...], z: [[...]]}, null where nothing was read
//
// Each node is the Gaussian-weighted average of the readings around it
// (kernel width in grid cells), so the surface follows the measurements and
// stays empty where the scan never went. Readings are first binned on a grid
// BIN times finer than the surface, so a long run costs little more than a
// short one. A bin holds the mean of its readings, or with {combine: 'max'}
// the best of them: readings taken at the same spot at different times (or
// tilts) then show the best power seen there.
(function (global) {
    'use strict';

    const BIN = 2;

    function build(points, xRange, yRange, options) {
        const opts = Object.assign({ n: 40, sigmaCells: 1.6, minWeight: 0.15, combine: 'mean' }, options);
        const best = opts.combine === 'max';
        const n = opts.n;
        const [x0, x1] = xRange;
        const [y0, y1] = yRange;
        const xs = [];
        const ys = [];
        for (let i = 0; i < n; i++) {
            xs.push(x0 + ((x1 - x0) * i) / (n - 1));
            ys.push(y0 + ((y1 - y0) * i) / (n - 1));
        }
        const fx = (n - 1) / (x1 - x0);
        const fy = (n - 1) / (y1 - y0);

        // Readings → bins: sum of the powers and count, per fine cell.
        const m = (n - 1) * BIN + 1;
        const binSum = new Float64Array(m * m);
        const binCount = new Uint32Array(m * m);
        for (const [x, y, v] of points) {
            if (!Number.isFinite(v)) continue;
            const bi = Math.round((x - x0) * fx * BIN);
            const bj = Math.round((y - y0) * fy * BIN);
            if (bi < 0 || bi >= m || bj < 0 || bj >= m) continue;
            const k = bj * m + bi;
            if (best) {
                if (!binCount[k] || v > binSum[k]) binSum[k] = v;
                binCount[k] = 1;
            } else {
                binSum[k] += v;
                binCount[k] += 1;
            }
        }

        // Bins → nodes, through the Gaussian kernel.
        const sum = new Float64Array(n * n);
        const weight = new Float64Array(n * n);
        const sigma = opts.sigmaCells;
        const reach = Math.ceil(sigma * 2.5);
        const k2 = 1 / (2 * sigma * sigma);
        for (let bj = 0; bj < m; bj++) {
            for (let bi = 0; bi < m; bi++) {
                const count = binCount[bj * m + bi];
                if (!count) continue;
                const v = binSum[bj * m + bi];
                const gx = bi / BIN;
                const gy = bj / BIN;
                const i0 = Math.max(0, Math.floor(gx - reach));
                const i1 = Math.min(n - 1, Math.ceil(gx + reach));
                const j0 = Math.max(0, Math.floor(gy - reach));
                const j1 = Math.min(n - 1, Math.ceil(gy + reach));
                for (let j = j0; j <= j1; j++) {
                    const dy = j - gy;
                    for (let i = i0; i <= i1; i++) {
                        const dx = i - gx;
                        const d2 = dx * dx + dy * dy;
                        if (d2 > reach * reach) continue;
                        const k = Math.exp(-d2 * k2);
                        sum[j * n + i] += k * v;
                        weight[j * n + i] += k * count;
                    }
                }
            }
        }

        const z = [];
        for (let j = 0; j < n; j++) {
            const row = [];
            for (let i = 0; i < n; i++) {
                const w = weight[j * n + i];
                row.push(w >= opts.minWeight ? sum[j * n + i] / w : null);
            }
            z.push(row);
        }
        return { x: xs, y: ys, z };
    }

    global.Landscape = { build };
})(window);
