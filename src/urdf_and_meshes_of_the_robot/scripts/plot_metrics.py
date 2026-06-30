#!/usr/bin/env python3
"""
plot_metrics.py  --  training-metrics visualization for the mybot PPO trainer.

Renders, from a `progress.csv` produced by ppo_balance.py:
  * one PNG per metric         (graphs/<metric>.png)
  * one multi-panel overview   (graphs/_overview.png)

Style is deliberately modelled on the `collision_avoidance_system` overview
plots: every panel shows the *raw* series as a thin light-blue line plus an
EMA-*smoothed* dark-blue line on top, laid out in a 3-column grid.

Two ways to use it
------------------
1. Live, from inside training (ppo_balance.py imports `render_overview` and
   calls it every few updates so you get a self-refreshing dashboard PNG).

2. Standalone, any time:
       python plot_metrics.py ~/mybot_train/logs/progress.csv
       python plot_metrics.py <csv> <out_graph_dir> [--x global_step]

It only depends on matplotlib (Agg backend) + the std lib, so it runs headless.
"""

import os
import csv
import sys
import argparse

import matplotlib
matplotlib.use("Agg")          # headless: no display needed
import matplotlib.pyplot as plt


# colours lifted from the collision_avoidance overview look
RAW_COLOR    = "#9ecae1"       # thin, light blue  -> the noisy raw series
SMOOTH_COLOR = "#08519c"       # thick, dark blue  -> the EMA-smoothed trend
LAST_BOX     = dict(boxstyle="round", fc="#fff7bc", ec="none", alpha=0.85)

# human-friendly titles / hints for the columns ppo_balance.py logs
METRIC_LABELS = {
    "ep_rew_mean":  "Episode reward (mean of last 20)",
    "ep_len_mean":  "Episode length (mean of last 20)",
    "policy_loss":  "Policy (surrogate) loss",
    "value_loss":   "Value-function loss",
    "entropy":      "Policy entropy",
    "approx_kl":    "Approx KL (policy step size)",
    "sps":          "Throughput (env steps / sec)",
    "eval_return":  "Eval return (deterministic)",
}
# columns that are axes/identifiers, never plotted as their own metric
X_CANDIDATES = ("global_step", "update", "timestamp", "step")


def ema(values, alpha=0.1):
    """Exponential moving average; ignores NaNs so gaps don't poison the trend."""
    out, m = [], None
    for v in values:
        if v != v:                       # NaN
            out.append(m if m is not None else float("nan"))
            continue
        m = v if m is None else (alpha * v + (1.0 - alpha) * m)
        out.append(m)
    return out


def _safe(name):
    return name.replace("/", "__").replace(" ", "_")


def load_csv(path):
    """Return (column_names, list_of_row_dicts). Tolerates a partly-written file."""
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        cols = reader.fieldnames or []
        rows = [r for r in reader if any((v or "").strip() for v in r.values())]
    return cols, rows


def _to_float(s):
    try:
        if s is None or s == "" or s.lower() == "nan":
            return float("nan")
        return float(s)
    except (ValueError, AttributeError):
        return float("nan")


def _extract(cols, rows, xcol):
    """Pick the x-axis and build {metric: [floats]} for every numeric column."""
    if xcol not in cols:
        xcol = next((c for c in X_CANDIDATES if c in cols), cols[0])
    x = [_to_float(r.get(xcol)) for r in rows]

    metrics = {}
    for c in cols:
        if c in X_CANDIDATES:
            continue
        vals = [_to_float(r.get(c)) for r in rows]
        if all(v != v for v in vals):    # skip all-NaN columns
            continue
        metrics[c] = vals
    return xcol, x, metrics


def _draw_panel(ax, x, vals, title, compact=False):
    ax.plot(x, vals, color=RAW_COLOR, lw=(0.9 if compact else 1.1),
            label=None if compact else "raw", zorder=1)
    finite = [v for v in vals if v == v]
    if len(finite) >= 3:
        ax.plot(x, ema(vals), color=SMOOTH_COLOR, lw=(1.8 if compact else 2.0),
                label=None if compact else "smoothed", zorder=2)
    ax.set_title(title, fontsize=(8 if compact else 10))
    ax.grid(True, alpha=0.3)
    if compact:
        ax.tick_params(labelsize=6)
    # annotate the latest value
    if finite:
        ax.annotate(f"last={finite[-1]:.4g}", xy=(0.98, 0.04),
                    xycoords="axes fraction", ha="right", va="bottom",
                    fontsize=(6.5 if compact else 8), bbox=LAST_BOX)


def render_overview(csv_path, graph_dir, xcol="global_step", title="mybot PPO — training metrics"):
    """Render per-metric PNGs + a single _overview.png. Safe to call repeatedly."""
    if not csv_path or not os.path.isfile(csv_path):
        return None
    cols, rows = load_csv(csv_path)
    if not rows:
        return None
    os.makedirs(graph_dir, exist_ok=True)
    xcol, x, metrics = _extract(cols, rows, xcol)
    if not metrics:
        return None

    # ---- per-metric figures ----
    for name, vals in metrics.items():
        fig, ax = plt.subplots(figsize=(6.4, 3.6), dpi=110)
        _draw_panel(ax, x, vals, METRIC_LABELS.get(name, name), compact=False)
        ax.set_xlabel(xcol)
        ax.set_ylabel("value")
        ax.legend(fontsize=7, loc="best")
        fig.tight_layout()
        fig.savefig(os.path.join(graph_dir, _safe(name) + ".png"))
        plt.close(fig)

    # ---- combined overview grid (3 columns, like the reference) ----
    names = list(metrics.keys())
    n = len(names)
    ncols = min(3, n)
    nrows = (n + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(ncols * 5.0, nrows * 3.0),
                             dpi=100, squeeze=False)
    for i, name in enumerate(names):
        _draw_panel(axes[i // ncols][i % ncols], x, metrics[name],
                    METRIC_LABELS.get(name, name), compact=True)
    for j in range(n, nrows * ncols):           # blank the unused cells
        axes[j // ncols][j % ncols].axis("off")
    subtitle = f"{n} metrics · {len(rows)} updates · x = {xcol}"
    fig.suptitle(f"{title}\n{subtitle}", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out = os.path.join(graph_dir, "_overview.png")
    fig.savefig(out)
    plt.close(fig)
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description="Render mybot PPO training graphs from progress.csv")
    p.add_argument("csv", nargs="?", default="./logs/progress.csv",
                   help="path to progress.csv")
    p.add_argument("graph_dir", nargs="?", default=None,
                   help="output dir (default: <csv_dir>/graphs)")
    p.add_argument("--x", default="global_step", help="x-axis column")
    args = p.parse_args(argv)

    graph_dir = args.graph_dir or os.path.join(os.path.dirname(os.path.abspath(args.csv)), "graphs")
    out = render_overview(args.csv, graph_dir, xcol=args.x)
    if out:
        print(f"[plot_metrics] wrote {out}")
    else:
        print(f"[plot_metrics] nothing to plot from {args.csv}")


if __name__ == "__main__":
    main()
