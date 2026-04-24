#!/usr/bin/env python3
"""Plot the relationship among alpha, m_raw, m_relaxed, and s_alpha.

Reads logs/data/footplacement_two_pahse.csv by default and saves figures to
logs/plot_fig. The script separates swing segments by alpha and only uses
swing-phase samples (alpha >= 0) for phase-domain plots.
"""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
LOGS_DIR = SCRIPT_DIR.parent
DATA_PATH = LOGS_DIR / "data" / "foot_placement_constraint_final_leg_0.csv"
OUTPUT_DIR = LOGS_DIR / "plot_fig"

REQUIRED_COLUMNS = ("time", "alpha", "s_alpha", "m_raw", "m_relaxed")


def load_csv(csv_path: Path) -> dict[str, np.ndarray]:
	with csv_path.open("r", newline="") as file:
		reader = csv.DictReader(file)
		if reader.fieldnames is None:
			raise ValueError(f"CSV file is empty: {csv_path}")

		missing_columns = [column for column in REQUIRED_COLUMNS if column not in reader.fieldnames]
		if missing_columns:
			raise ValueError(
				f"Missing required columns {missing_columns} in {csv_path}. "
				f"Found columns: {reader.fieldnames}"
			)

		rows = list(reader)

	if not rows:
		raise ValueError(f"CSV file has no data rows: {csv_path}")

	data = {
		column: np.array([float(row[column]) for row in rows], dtype=float)
		for column in REQUIRED_COLUMNS
	}
	return data


def split_swing_segments(alpha: np.ndarray) -> list[np.ndarray]:
	valid_indices = np.flatnonzero(alpha >= 0.0)
	if valid_indices.size == 0:
		return []

	segments: list[np.ndarray] = []
	start = 0
	for index in range(1, valid_indices.size):
		previous_idx = valid_indices[index - 1]
		current_idx = valid_indices[index]
		is_discontinuous = current_idx != previous_idx + 1
		alpha_reset = alpha[current_idx] < alpha[previous_idx]
		if is_discontinuous or alpha_reset:
			segments.append(valid_indices[start:index])
			start = index

	segments.append(valid_indices[start:])
	return segments


def plot_time_series(data: dict[str, np.ndarray], output_path: Path) -> None:
	time = data["time"]
	alpha = data["alpha"]
	s_alpha = data["s_alpha"]
	m_raw = data["m_raw"]
	m_relaxed = data["m_relaxed"]

	figure, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)

	axes[0].plot(time, alpha, color="tab:blue", linewidth=1.8, label="alpha")
	axes[0].set_ylabel("alpha")
	axes[0].set_title("Foot Placement Relaxation Over Time")
	axes[0].grid(True, alpha=0.3)
	axes[0].legend(loc="best")

	axes[1].plot(time, m_raw, color="tab:red", linewidth=1.8, label="m_raw")
	axes[1].plot(time, m_relaxed, color="tab:green", linewidth=1.8, label="m_relaxed")
	axes[1].plot(time, s_alpha, color="tab:orange", linewidth=1.8, linestyle="--", label="s_alpha")
	axes[1].set_xlabel("time [s]")
	axes[1].set_ylabel("value")
	axes[1].grid(True, alpha=0.3)
	axes[1].legend(loc="best")

	figure.tight_layout()
	figure.savefig(output_path, dpi=180, bbox_inches="tight")
	plt.close(figure)


def plot_phase_relationships(data: dict[str, np.ndarray], output_path: Path) -> None:
	alpha = data["alpha"]
	s_alpha = data["s_alpha"]
	m_raw = data["m_raw"]
	m_relaxed = data["m_relaxed"]

	swing_segments = split_swing_segments(alpha)
	if not swing_segments:
		raise ValueError("No swing-phase samples found. Expected alpha >= 0 in at least one segment.")

	figure, axes = plt.subplots(3, 1, figsize=(11, 11), sharex=True)
	colors = plt.cm.tab10(np.linspace(0.0, 1.0, max(len(swing_segments), 1)))

	for segment_id, segment_indices in enumerate(swing_segments, start=1):
		color = colors[segment_id - 1]
		label = f"swing #{segment_id}"
		segment_alpha = alpha[segment_indices]

		axes[0].plot(segment_alpha, m_raw[segment_indices], color=color, marker="o", markersize=3.5,
					 linewidth=1.4, label=label)
		axes[1].plot(segment_alpha, m_relaxed[segment_indices], color=color, marker="o", markersize=3.5,
					 linewidth=1.4, label=label)
		axes[2].plot(segment_alpha, s_alpha[segment_indices], color=color, marker="o", markersize=3.5,
					 linewidth=1.4, label=label)

	axes[0].set_ylabel("m_raw")
	axes[0].set_title("Relationship Under Different Swing Phases")
	axes[0].grid(True, alpha=0.3)
	axes[0].legend(loc="best", ncol=2)

	axes[1].set_ylabel("m_relaxed")
	axes[1].grid(True, alpha=0.3)

	axes[2].set_xlabel("alpha")
	axes[2].set_ylabel("s_alpha")
	axes[2].grid(True, alpha=0.3)

	figure.tight_layout()
	figure.savefig(output_path, dpi=180, bbox_inches="tight")
	plt.close(figure)


def plot_constraint_gap(data: dict[str, np.ndarray], output_path: Path) -> None:
	alpha = data["alpha"]
	s_alpha = data["s_alpha"]
	m_raw = data["m_raw"]
	m_relaxed = data["m_relaxed"]

	swing_mask = alpha >= 0.0
	swing_alpha = alpha[swing_mask]
	swing_raw = m_raw[swing_mask]
	swing_relaxed = m_relaxed[swing_mask]
	swing_relaxation = s_alpha[swing_mask]

	figure, axes = plt.subplots(1, 2, figsize=(12, 4.8))

	axes[0].scatter(swing_alpha, swing_raw, s=22, color="tab:red", alpha=0.8, label="m_raw")
	axes[0].scatter(swing_alpha, swing_relaxed, s=22, color="tab:green", alpha=0.8, label="m_relaxed")
	axes[0].set_xlabel("alpha")
	axes[0].set_ylabel("constraint value")
	axes[0].set_title("Constraint Value vs Swing Phase")
	axes[0].grid(True, alpha=0.3)
	axes[0].legend(loc="best")

	axes[1].scatter(swing_relaxation, swing_relaxed - swing_raw, s=22, color="tab:purple", alpha=0.8)
	axes[1].plot(
		[swing_relaxation.min(), swing_relaxation.max()],
		[swing_relaxation.min(), swing_relaxation.max()],
		color="black",
		linestyle="--",
		linewidth=1.2,
		label="y = x",
	)
	axes[1].set_xlabel("s_alpha")
	axes[1].set_ylabel("m_relaxed - m_raw")
	axes[1].set_title("Relaxation Consistency Check")
	axes[1].grid(True, alpha=0.3)
	axes[1].legend(loc="best")

	figure.tight_layout()
	figure.savefig(output_path, dpi=180, bbox_inches="tight")
	plt.close(figure)


def main() -> None:
	OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

	data = load_csv(DATA_PATH)

	plot_time_series(data, OUTPUT_DIR / "footplacement_time_series.png")
	plot_phase_relationships(data, OUTPUT_DIR / "footplacement_phase_relationships.png")
	plot_constraint_gap(data, OUTPUT_DIR / "footplacement_relaxation_check.png")

	print(f"Loaded data from: {DATA_PATH}")
	print(f"Saved figures to: {OUTPUT_DIR}")
	print("Generated:")
	print("  - footplacement_time_series.png")
	print("  - footplacement_phase_relationships.png")
	print("  - footplacement_relaxation_check.png")


if __name__ == "__main__":
	main()
