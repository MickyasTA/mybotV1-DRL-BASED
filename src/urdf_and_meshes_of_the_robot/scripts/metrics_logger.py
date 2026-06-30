#!/usr/bin/env python3
"""
metrics_logger.py — per-episode CSV + summary JSON, in the SAME layout as
collision_avoidance_system/models/detour_runs/run1/metrics/:

    metrics/
      episodes_<session>.csv     # one row per episode
      summary_<session>.json     # session_info + performance_metrics + recent
      runs/<session>/            # TensorBoard event files (TBPlotWriter)
      graphs/                    # one PNG per scalar + _overview.png (TBPlotWriter)

Columns are adapted to the BALANCE task (the detour task logged collision/goal
fields; ours logs tilt/fall/effort), but the structure/format is identical.
"""

import os
import csv
import json
import time


EPISODE_FIELDS = [
    "episode", "score", "steps", "duration", "fell",
    "mean_tilt_deg", "max_tilt_deg", "mean_effort",
    "global_step", "timestamp",
]


class MetricsLogger:
    def __init__(self, metrics_dir, session_id):
        self.metrics_dir = metrics_dir
        self.session_id = session_id
        os.makedirs(metrics_dir, exist_ok=True)
        self.csv_path = os.path.join(metrics_dir, f"episodes_{session_id}.csv")
        self.summary_path = os.path.join(metrics_dir, f"summary_{session_id}.json")
        self.start_time = None            # stamped by caller (Date is unavailable in some envs)
        self._rows = []
        # (re)create the CSV with a header if it doesn't exist yet (resume appends)
        if not os.path.isfile(self.csv_path) or os.stat(self.csv_path).st_size == 0:
            with open(self.csv_path, "w", newline="") as f:
                csv.writer(f).writerow(EPISODE_FIELDS)

    def set_start_time(self, t):
        self.start_time = t

    def log_episode(self, episode, score, steps, duration, fell,
                    mean_tilt_deg, max_tilt_deg, mean_effort, global_step, timestamp):
        row = [episode, score, steps, duration, bool(fell),
               mean_tilt_deg, max_tilt_deg, mean_effort, global_step, timestamp]
        self._rows.append(dict(zip(EPISODE_FIELDS, row)))
        with open(self.csv_path, "a", newline="") as f:
            csv.writer(f).writerow(row)

    def write_summary(self, now, total_episodes, recent_n=100):
        if not self._rows:
            return
        scores = [r["score"] for r in self._rows]
        lens = [r["steps"] for r in self._rows]
        durs = [r["duration"] for r in self._rows]
        falls = [1 for r in self._rows if r["fell"]]
        recent = self._rows[-recent_n:]
        summary = {
            "session_info": {
                "session_id": self.session_id,
                "start_time": self.start_time,
                "duration": (now - self.start_time) if self.start_time else None,
                "total_episodes": total_episodes,
            },
            "performance_metrics": {
                "avg_score": sum(scores) / len(scores),
                "max_score": max(scores),
                "min_score": min(scores),
                "avg_episode_length": sum(lens) / len(lens),
                "max_episode_length": max(lens),
                "avg_episode_duration": sum(durs) / len(durs),
                "fall_rate": len(falls) / len(self._rows),
                "balance_rate": 1.0 - len(falls) / len(self._rows),
            },
            "recent_performance": {
                "recent_avg_score": sum(r["score"] for r in recent) / len(recent),
                "recent_avg_length": sum(r["steps"] for r in recent) / len(recent),
                "recent_balance_rate": 1.0 - sum(1 for r in recent if r["fell"]) / len(recent),
            },
        }
        with open(self.summary_path, "w") as f:
            json.dump(summary, f, indent=2)
