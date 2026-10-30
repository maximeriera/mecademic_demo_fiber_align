"""Replay a simulated alignment into the real custom view, for a browser to watch.

    python app_logic/tools/view_replay.py [--offset X Z RX RZ] [--optics beam|smf] [--speed 4] [--port 8765]
                                          [--dark] [--full-search] [--step]

Runs one alignment on the simulated cell (app_logic/tests/fiber_alignment/sim.py),
then serves the page in app_logic/custom_view/ and its /static/ files with the
hub endpoints it polls, replaying the run's publishes and events into a real
core.DataHub at ``--speed`` times real time. app_logic/tools/view_check.mjs
drives a browser through it; open http://localhost:8765/view/ to watch by hand.

Besides the page's own endpoints:

* ``/api/replay``: where the replay is (current phase, paused, done) and the
  run's outcome, for the checker to compare with what the page shows.
* ``/api/replay/restart``: replay the same run again from the start.
* ``/api/replay/resume``: with ``--step``, the replay pauses halfway through
  each phase until this is called.

Development only: nothing here is part of the app.
"""

from __future__ import annotations

import argparse
import json
import math
import mimetypes
import sys
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from core.DataHub import DataHub  # noqa: E402
from app_logic.custom_view import POWER_RATE_HZ  # noqa: E402  (the live tile's sampling rate)
from app_logic.fiber_alignment import algorithm  # noqa: E402
from app_logic.tests.fiber_alignment.sim import Scenario, simulate  # noqa: E402

VIEW_DIR = ROOT / "app_logic" / "custom_view"
STATIC_DIR = ROOT / "static"


def jsonable(value):
    """Strict JSON, as app.py's _jsonable makes it: NaN and infinities become null."""
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if hasattr(value, "tolist"):
        return jsonable(value.tolist())
    try:
        return jsonable(float(value))
    except (TypeError, ValueError):
        return str(value)


class Replay:
    """The recorded run, replayed into a fresh DataHub at ``speed`` times real time."""

    def __init__(self, run, speed: float, threshold_mw: float, step: bool = False):
        self.run = run
        self.speed = speed
        self.threshold_mw = threshold_mw
        self.timeline = run.context.timeline
        self.hub = DataHub()
        self.epoch_utc = datetime.now(timezone.utc).isoformat()
        self.phase = None
        self.done = False
        self.paused = False
        # With step, the timeline entries halfway through each phase, where the replay waits.
        self._pauses = self._midpoints() if step else set()
        self._resume = threading.Event()
        self._lock = threading.Lock()
        self._generation = 0
        self.restart()

    def _midpoints(self) -> set:
        starts = [i for i, e in enumerate(self.timeline) if e[1] == "mark" and e[2] == "align_phase_start"]
        ends = [i for i, e in enumerate(self.timeline) if e[1] == "mark" and e[2] == "align_phase_end"]
        return {(a + b) // 2 for a, b in zip(starts, ends)}

    def restart(self) -> None:
        with self._lock:
            self._generation += 1
            generation = self._generation
            self.hub = DataHub()
            self.phase = None
            self.done = False
            self.paused = False
            self._resume.set()
        threading.Thread(target=self._play, args=(generation,), daemon=True).start()

    def resume(self) -> None:
        self._resume.set()

    def _play(self, generation: int) -> None:
        hub = self.hub
        # What custom_view.setup() publishes at Initialize.
        hub.publish("align_phases", {"phases": list(algorithm.PHASES), "phase_views": algorithm.PHASE_VIEWS})
        time.sleep(1.0)
        t0_sim = self.timeline[0][0] if self.timeline else 0.0
        t0 = time.monotonic()
        last_power, next_power = None, 0.0
        for index, (t_sim, kind, name, value) in enumerate(self.timeline):
            if generation != self._generation:
                return
            if index in self._pauses:
                self._resume.clear()
                self.paused = True
                paused_at = time.monotonic()
                self._resume.wait()
                self.paused = False
                # The replay clock stops while paused.
                t0 += time.monotonic() - paused_at
                if generation != self._generation:
                    return
            wait = t0 + (t_sim - t0_sim) / self.speed - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            if kind == "mark":
                hub.mark(name, **value)
                if name == "align_phase_start":
                    self.phase = value.get("phase")
                elif name == "align_end":
                    self.phase = None
            else:
                hub.publish(name, value)
                if name == "align_sample":
                    last_power = value[6]
            # The live tile: during a run, custom_view repeats the latest alignment reading.
            if last_power is not None and t_sim >= next_power:
                hub.publish("power", last_power)
                next_power = t_sim + 1.0 / POWER_RATE_HZ
        self.done = True
        while generation == self._generation:
            if last_power is not None:
                hub.publish("power", self.run.result.power_w if self.run.result else last_power)
            time.sleep(1.0 / POWER_RATE_HZ)

    def _acquire_map_points(self) -> int:
        acquire = self.run.context.published("align_acquire")
        search = acquire[-1].get("search") if acquire else None
        since = min(a["t"] for a in acquire if a.get("search") == search) if search else -math.inf
        return sum(1 for row in self.run.rows if row[9] >= since)

    def status(self) -> dict:
        run = self.run
        summary = run.context.published("align_result")[-1] if run.context.published("align_result") else None
        per_phase: dict = {}
        for name in ("acquire", "profile", "peak"):
            for value in run.context.published(f"align_{name}"):
                phase = algorithm.PHASES[value["phase"]] if 0 <= value.get("phase", -1) < len(algorithm.PHASES) else None
                per_phase.setdefault(phase, {}).setdefault(name, 0)
                per_phase[phase][name] += 1
        return {
            "phase": self.phase,
            "paused": self.paused,
            "done": self.done,
            "phases": list(algorithm.PHASES),
            "phase_views": algorithm.PHASE_VIEWS,
            "threshold_w": self.threshold_mw * 1e-3,
            "error": None if run.error is None else f"{type(run.error).__name__}: {run.error}",
            "result": None if run.result is None else {
                "passed": run.result.passed, "power_w": run.result.power_w, "reads": run.result.reads,
                "duration_s": run.result.duration_s,
            },
            "best_still": summary.get("best_still") if summary else None,
            # The search the Acquire card ends on (None: light at the start), and whether it ran in full.
            "acquire_search": ([a.get("search") for a in run.context.published("align_acquire")] or [None])[-1],
            "full_search": bool(run.context.params.get("acquire_full_search")),
            # What the Acquire map plots by the end ("until_end"): every reading from its last search's start.
            "acquire_map_points": self._acquire_map_points(),
            "source_power_w": summary.get("source_power_w") if summary else None,
            "counts": per_phase,
        }


def make_handler(replay: Replay):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):  # quiet
            pass

        def _json(self, payload, status=200):
            body = json.dumps(jsonable(payload)).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _file(self, base: Path, rel: str):
            path = (base / rel).resolve()
            if base.resolve() not in path.parents or not path.is_file() or path.suffix in (".py", ".pyc"):
                self.send_error(404)
                return
            body = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", mimetypes.guess_type(path.name)[0] or "application/octet-stream")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):  # noqa: N802 - http.server's naming
            url = urlparse(self.path)
            query = parse_qs(url.query)
            path = url.path
            hub = replay.hub
            if path in ("/view", "/view/"):
                return self._file(VIEW_DIR, "index.html")
            if path.startswith("/view/"):
                return self._file(VIEW_DIR, path[len("/view/"):])
            if path.startswith("/static/"):
                return self._file(STATIC_DIR, path[len("/static/"):])
            if path == "/api/data":
                return self._json({"active": True, "setup_error": None, "epoch_utc": replay.epoch_utc,
                                   "channels": {"power": {"rate_hz": POWER_RATE_HZ, "errors": 0}},
                                   "view_available": True})
            if path == "/api/data/read":
                after = query.get("after", [None])[0]
                cursor, data = hub.read(query.get("ch", []), int(after) if after is not None else None)
                return self._json({"now": hub.now(), "cursor": cursor, "data": data})
            if path == "/api/context":
                return self._json({"enabled": True, "params": {"power_threshold_mw": {"value": replay.threshold_mw}},
                                   "variables": {}})
            if path == "/api/status":
                phase = replay.phase
                return self._json({"status": "BUSY" if phase else "READY", "task": "align_fiber" if phase else None,
                                   "step": None if not phase else {
                                       "name": phase, "path": ["Align fiber", phase], "index": None, "total": None,
                                       "depth": 1, "elapsed_s": 0, "progress": 0}})
            if path == "/api/replay":
                return self._json(replay.status())
            if path == "/api/replay/restart":
                replay.restart()
                return self._json({"restarted": True})
            if path == "/api/replay/resume":
                replay.resume()
                return self._json({"resumed": True})
            self.send_error(404)

    return Handler


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    # Default: the cell's optics, no light at the start until the tilt spiral finds it.
    parser.add_argument("--offset", type=float, nargs=4, default=(40.0, 30.0, 1.2, -0.4),
                        metavar=("X", "Z", "RX", "RZ"), help="optimum from the start: µm, µm, °, °")
    parser.add_argument("--pivot", type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=("X", "Y", "Z"),
                        help="fiber tip from the taught TCP, in the tool frame (µm)")
    parser.add_argument("--optics", choices=("beam", "smf"), default="beam",
                        help="beam: collimated ends like the cell's; smf: bare single-mode fibers")
    parser.add_argument("--dark", action="store_true", help="no light: the search fails")
    parser.add_argument("--full-search", action="store_true",
                        help="acquire_full_search: both searches run to their full radius, even with light at the start")
    parser.add_argument("--speed", type=float, default=4.0, help="replay speed, × real time")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--step", action="store_true", help="pause halfway through each phase until resumed")
    args = parser.parse_args(argv)

    scenario = Scenario(offset=tuple(args.offset), optics=args.optics, pivot_um=tuple(args.pivot), dark=args.dark,
                        seed=args.seed, params={"acquire_full_search": args.full_search})
    run = simulate(scenario)
    threshold_mw = run.context.params["power_threshold_mw"]
    replay = Replay(run, args.speed, threshold_mw, step=args.step)
    outcome = replay.status()
    print(f"Simulated: {outcome['result'] or outcome['error']}", flush=True)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(replay))
    print(f"Serving http://127.0.0.1:{args.port}/view/ at {args.speed:g}× real time", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
