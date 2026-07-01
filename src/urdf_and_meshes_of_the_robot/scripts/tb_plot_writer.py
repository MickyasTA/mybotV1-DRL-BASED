#!/usr/bin/env python3
"""
TBPlotWriter / PngPlotWriter — live PNG graphs of every scalar metric, one file per tag
plus a combined _overview.png, refreshed by a background thread. raw = #9ecae1,
EMA-smoothed = #08519c.

TBPlotWriter ALSO writes real TensorBoard event files (needs torch) — used by the
PyTorch trainers (ppo_balance.py, ppo_parallel.py). PngPlotWriter has the identical
add_scalar/render/close API but no torch dependency — used by the JAX/MJX trainer
(mjx_ppo.py), which runs in an isolated conda env that doesn't have torch installed.
Both write into the SAME run-dir layout:

    metrics/runs/<session>/    <- TensorBoard event files (TBPlotWriter only)
    metrics/graphs/            <- one PNG per scalar + _overview.png (both)

Usage:
    writer = TBPlotWriter(log_dir=<metrics>/runs/<session>, graph_dir=<metrics>/graphs)
    # or, torch-free: writer = PngPlotWriter(graph_dir=<metrics>/graphs)
    writer.add_scalar("episode/score", score, episode)
    ...
    writer.close()
"""

import os
import re
import threading
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")           # headless: render to files, no display needed
import matplotlib.pyplot as plt


def _ema(values, alpha=0.1):
    """Exponential moving average for a readable smoothed trend line."""
    if not values:
        return []
    out = [values[0]]
    for v in values[1:]:
        out.append(alpha * v + (1 - alpha) * out[-1])
    return out


def _safe(tag):
    return re.sub(r"[^0-9a-zA-Z._-]+", "__", tag).strip("_") or "metric"


class _PlotMixin:
    """Shared live-PNG-rendering behavior for TBPlotWriter and PngPlotWriter."""

    def _init_plotting(self, graph_dir, refresh_sec):
        self.graph_dir = graph_dir
        os.makedirs(self.graph_dir, exist_ok=True)
        self._series = defaultdict(lambda: ([], []))   # tag -> (steps, values)
        # CONTINUITY across processes: reload the persisted series (written by render()),
        # so successive curriculum stages EXTEND the same curves instead of starting new
        # graphs -- one continuous training record per run dir.
        self._series_path = os.path.join(self.graph_dir, "_series.json")
        try:
            import json
            with open(self._series_path) as f:
                for tag, (steps, vals) in json.load(f).items():
                    self._series[tag] = (list(steps), list(vals))
        except Exception:
            pass
        self._lock = threading.Lock()
        self._dirty = False
        self._stop = threading.Event()
        self._refresh = max(2.0, float(refresh_sec))
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _record(self, tag, scalar_value, global_step):
        try:
            v = float(scalar_value)
            with self._lock:
                steps, vals = self._series[tag]
                steps.append(int(global_step) if global_step is not None else len(vals))
                vals.append(v)
                self._dirty = True
        except (TypeError, ValueError):
            pass

    def _loop(self):
        while not self._stop.wait(self._refresh):
            if self._dirty:
                try:
                    self.render()
                except Exception:
                    pass   # never let plotting kill training

    def _snapshot(self):
        with self._lock:
            self._dirty = False
            return {t: (list(s), list(v)) for t, (s, v) in self._series.items() if v}

    def render(self):
        """Render one PNG per metric + a combined overview grid into graph_dir."""
        data = self._snapshot()
        if not data:
            return
        try:                        # persist for cross-stage continuity (atomic replace)
            import json
            tmp = self._series_path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(data, f)
            os.replace(tmp, self._series_path)
        except Exception:
            pass
        for tag, (steps, vals) in data.items():
            fig, ax = plt.subplots(figsize=(6.4, 3.6), dpi=110)
            ax.plot(steps, vals, color="#9ecae1", lw=1.0, label="raw")
            if len(vals) >= 3:
                ax.plot(steps, _ema(vals), color="#08519c", lw=2.0, label="smoothed")
            ax.set_title(tag, fontsize=10)
            ax.set_xlabel("episode/step"); ax.set_ylabel("value")
            ax.grid(True, alpha=0.3); ax.legend(fontsize=7, loc="best")
            last = vals[-1]
            ax.annotate(f"last={last:.4g}", xy=(0.98, 0.02), xycoords="axes fraction",
                        ha="right", va="bottom", fontsize=8,
                        bbox=dict(boxstyle="round", fc="#fff7bc", ec="none", alpha=0.8))
            fig.tight_layout()
            fig.savefig(os.path.join(self.graph_dir, _safe(tag) + ".png"))
            plt.close(fig)
        tags = sorted(data.keys())
        n = len(tags)
        cols = min(3, n)
        rows = (n + cols - 1) // cols
        fig, axes = plt.subplots(rows, cols, figsize=(cols * 5.0, rows * 3.0), dpi=100,
                                 squeeze=False)
        for i, tag in enumerate(tags):
            steps, vals = data[tag]
            ax = axes[i // cols][i % cols]
            ax.plot(steps, vals, color="#9ecae1", lw=0.9)
            if len(vals) >= 3:
                ax.plot(steps, _ema(vals), color="#08519c", lw=1.8)
            ax.set_title(tag, fontsize=8)
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=6)
        for j in range(n, rows * cols):
            axes[j // cols][j % cols].axis("off")
        fig.suptitle("mybot — training metrics (live)", fontsize=12)
        fig.tight_layout(rect=(0, 0, 1, 0.97))
        fig.savefig(os.path.join(self.graph_dir, "_overview.png"))
        plt.close(fig)


class TBPlotWriter(_PlotMixin):
    """A drop-in SummaryWriter (real TensorBoard event files) that ALSO exports live
    PNG graphs. Needs torch; imported lazily so this module stays importable without
    torch (see PngPlotWriter, used by the torch-free JAX/MJX trainer)."""

    def __init__(self, log_dir=None, graph_dir=None, refresh_sec=20.0, **kwargs):
        from torch.utils.tensorboard import SummaryWriter
        self._tb = SummaryWriter(log_dir=log_dir, **kwargs)
        self._init_plotting(graph_dir or os.path.join(log_dir or ".", "graphs"), refresh_sec)

    def add_scalar(self, tag, scalar_value, global_step=None, *args, **kwargs):
        self._tb.add_scalar(tag, scalar_value, global_step, *args, **kwargs)
        self._record(tag, scalar_value, global_step)

    def close(self):
        self._stop.set()
        try:
            self.render()
        except Exception:
            pass
        self._tb.close()


class PngPlotWriter(_PlotMixin):
    """Same add_scalar/render/close API as TBPlotWriter, PNG graphs only, no torch —
    for trainers running in an env without torch installed (mjx_ppo.py / mybot_mjx)."""

    def __init__(self, graph_dir, refresh_sec=20.0):
        self._init_plotting(graph_dir, refresh_sec)

    def add_scalar(self, tag, scalar_value, global_step=None, *args, **kwargs):
        self._record(tag, scalar_value, global_step)

    def close(self):
        self._stop.set()
        try:
            self.render()
        except Exception:
            pass
