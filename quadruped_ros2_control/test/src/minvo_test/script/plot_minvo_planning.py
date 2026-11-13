#!/usr/bin/env python3
"""Plot the CSV emitted by the production minvo_planning_test executable."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

NAMES = ["Discrete", "Bezier", "MINVO"]
COLORS = ["#c75b4b", "#e0a643", "#278c9e"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", nargs="?", type=Path, default=Path(__file__).resolve().parents[1] / "data/benchmark")
    parser.add_argument("--result", type=Path, default=Path(__file__).resolve().parents[1] / "result/benchmark")
    args = parser.parse_args()
    out = args.directory
    args.result.mkdir(parents=True, exist_ok=True)
    data = pd.read_csv(out / "metrics.csv")
    curves = pd.read_csv(out / "trajectories.csv")
    first = data[data["repeat"] == 0]
    summary = {}
    for mode, name in enumerate(NAMES):
        group = data[data["mode"] == mode]
        cases = first[first["mode"] == mode]
        summary[name] = {
            "cases": len(cases),
            "solved": int(cases.solved.sum()),
            "independently_certified": int(cases.certified.sum()),
            "sampled_collisions": int((cases.min_clearance < -1e-6).sum()),
            "worst_clearance_m": float(cases.min_clearance.min()),
            "median_ms": float(group.planning_ms.median()),
            "p95_ms": float(group.planning_ms.quantile(.95)),
            "p99_ms": float(group.planning_ms.quantile(.99)),
            "max_ms": float(group.planning_ms.max()),
        }
    paired = first.pivot(index="case_id", columns="mode", values="correction_rms")
    improvement = (paired[1] - paired[2]) / paired[1]
    objective = first.pivot(index="case_id", columns="mode", values="objective")
    objective_reduction = (objective[1]-objective[2])/objective[1]
    summary["paired_MINVO_vs_Bezier"] = {
        "median_objective_reduction_percent": float(100*objective_reduction.median()),
        "max_objective_reduction_percent": float(100*objective_reduction.max()),
        "strictly_lower_objective_cases": int((objective_reduction>1e-6).sum()),
        "common_solved_cases": int(improvement.notna().sum()),
        "median_rms_reduction_percent": float(100 * improvement.median()),
        "max_rms_reduction_percent": float(100 * improvement.max()),
        "strictly_lower_rms_cases": int((improvement > 1e-6).sum()),
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                         "figure.dpi": 150, "savefig.dpi": 180})
    fig, axes = plt.subplots(1, 3, figsize=(14, 4), layout="constrained")
    for mode, name in enumerate(NAMES):
        group = first[first["mode"] == mode]
        axes[0].plot(group.case_id, group.min_clearance * 1000, "o-", ms=3,
                     label=name, color=COLORS[mode], lw=1)
        axes[1].plot(group.case_id, group.objective, "o-", ms=3,
                     label=name, color=COLORS[mode], lw=1)
    axes[0].axhline(0, color="black", lw=.7)
    axes[0].set(title="Independent dense sphere clearance", xlabel="Paired case", ylabel="Minimum clearance (mm)")
    axes[1].set(title="Same quadratic correction objective", xlabel="Paired case", ylabel="QP correction objective")
    axes[1].legend(frameon=False)
    axes[2].boxplot([data[data["mode"] == i].planning_ms for i in range(3)], labels=NAMES, showfliers=True)
    axes[2].set(title="Total planner latency (5 repeats/case)", ylabel="Milliseconds")
    fig.suptitle("27 synthetic static terrains | fixed XY, endpoints, weights and solver budget")
    fig.savefig(args.result / "planning_comparison.png")
    plt.close(fig)

    # Pick the largest independently measured discrete collision, without dropping failed cases.
    candidate = first[(first["mode"] == 0) & (first.solved == 1)].sort_values("min_clearance").iloc[0]
    case_id = int(candidate.case_id)
    fig, ax = plt.subplots(figsize=(9, 4.5), layout="constrained")
    case_curves = curves[curves.case_id == case_id]
    shown = case_curves[case_curves["mode"] == 0]
    ax.fill_between(shown.x, 0, shown.terrain, color="#4b5563", alpha=.4, label="Obstacle")
    ax.plot(shown.x, shown.required, "--", color="black", lw=1, label="Sphere clearance envelope")
    ax.plot(shown.x, shown.nominal_z, ":", color="#64748b", label="Nominal height")
    for mode, name in enumerate(NAMES):
        group = case_curves[case_curves["mode"] == mode]
        if not group.empty:
            ax.plot(group.x, group.z, color=COLORS[mode], label=name, lw=1.8)
    ax.set(title=f"Case {case_id}: thin obstacle between collocation samples",
           xlabel="World x (m)", ylabel="Foot centre height minus radius (m)")
    ax.legend(ncol=3, frameon=False)
    fig.savefig(args.result / "narrow_obstacle.png")
    plt.close(fig)

    hull = pd.read_csv(out / "hull_bounds.csv")
    fig, ax = plt.subplots(figsize=(8, 4.5), layout="constrained")
    ax.plot(hull.true_min * 1000, hull.true_min * 1000, "--", color="#64748b", label="Actual curve minimum")
    ax.plot(hull.true_min * 1000, hull.bezier_lower * 1000, color=COLORS[1], label="Bezier hull lower bound")
    ax.plot(hull.true_min * 1000, hull.minvo_lower * 1000, color=COLORS[2], label="MINVO hull lower bound")
    ax.axhline(0, color="black", lw=.7)
    ax.set(title="Same quintic representation of z(u) = (u - 0.5)^2 + minimum",
           xlabel="Actual minimum clearance (mm)", ylabel="Certified hull lower bound (mm)")
    ax.legend(frameon=False)
    fig.savefig(args.result / "hull_tightness.png")
    plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
