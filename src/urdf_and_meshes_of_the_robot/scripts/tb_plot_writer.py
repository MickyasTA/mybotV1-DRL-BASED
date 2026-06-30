#!/usr/bin/env python3
"""
TBPlotWriter — a drop-in SummaryWriter that ALSO renders every scalar metric to a
readable PNG graph, live, into a single folder.

Mirrors the collision_avoidance_system results/eval style: TensorBoard event files
under runs/<ts>/ PLUS one PNG per metric + a combined _overview.png under graphs/,
refreshed live by a background thread. raw = #9ecae1, EMA-smoothed = #08519c.

Usage:
    writer = TBPlotWriter(log_dir=<metrics>/runs/<ts>, graph_dir=<metrics>/graphs)
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

from torch.utils.tensorboard import SummaryWriter


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


class TBPlotWriter(SummaryWriter):
    """SummaryWriter that also exports live PNG graphs of every scalar."""

    def __init__(self, log_dir=None, graph_dir=None, refresh_sec=20.0, **kwargs):
        super().__init__(log_dir=log_dir, **kwargs)
        self.graph_dir = graph_dir or os.path.join(log_dir or ".", "graphs")
        os.makedirs(self.graph_dir, exist_ok=True)
        self._series = defaultdict(lambda: ([], []))   # tag -> (steps, values)
        self._lock = threading.Lock()
        self._dirty = False
        self._stop = threading.Event()
        self._refresh = max(2.0, float(refresh_sec))
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def add_scalar(self, tag, scalar_value, global_step=None, *args, **kwargs):
        super().add_scalar(tag, scalar_value, global_step, *args, **kwargs)
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
        fig.suptitle("mybot balance — training metrics (live)", fontsize=12)
        fig.tight_layout(rect=(0, 0, 1, 0.97))
        fig.savefig(os.path.join(self.graph_dir, "_overview.png"))
        plt.close(fig)

    def close(self):
        self._stop.set()
        try:
            self.render()
        except Exception:
            pass
        super().close()
