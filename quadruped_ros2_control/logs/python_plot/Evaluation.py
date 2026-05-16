#!/usr/bin/env python3
"""
Compute trajectory evaluation metrics from logs/data CSV files.

Metrics follow the ESKF-style evaluation used in the paper:
  - ATEatt / ATEpos: absolute trajectory error (RMSE in rad / m)
  - RPEatt / RPEpos: relative pose error at a fixed 1 m segment (RMSE in rad / m)
  - For rotation-dominant logs with less than 1 m translation, RPE falls back to a
    fixed angular span (default pi/2 rad).

Usage:
  python3 Evaluation.py
"""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass
from typing import Sequence

import numpy as np

# ---------------------------------------------------------------------------
# Path configuration (relative to this script)
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR = os.path.dirname(SCRIPT_DIR)
DATA_DIR = os.path.join(LOGS_DIR, "data")
OUT_DIR = os.path.join(LOGS_DIR, "evaluation")

# Position: estimated vs ground truth (columns: est_*, gt_*)
POS_EST_TRUE_X = os.path.join(DATA_DIR, "pos_est_true_x.csv")
POS_EST_TRUE_Y = os.path.join(DATA_DIR, "pos_est_true_y.csv")
POS_EST_TRUE_Z = os.path.join(DATA_DIR, "pos_est_true_z.csv")

# Orientation (ZYX Euler: yaw, pitch, roll in radians)
ROT_EST_TRUE_YAW = os.path.join(DATA_DIR, "rot_est_true_yaw.csv")
ROT_EST_TRUE_PITCH = os.path.join(DATA_DIR, "rot_est_true_pitch.csv")
ROT_EST_TRUE_ROLL = os.path.join(DATA_DIR, "rot_est_true_roll.csv")

# RPE settings
RPE_DELTA_FRAMES: Sequence[int] = ()
RPE_DELTA_METERS: Sequence[float] = (1.0,)
RPE_DELTA_RADIANS: Sequence[float] = (math.pi / 2,)
DISTANCE_TOLERANCE_RATIO = 0.1
ANGLE_TOLERANCE_RATIO = 0.1

REPORT_TXT = os.path.join(OUT_DIR, "evaluation_report.txt")


@dataclass(frozen=True)
class MetricStats:
    rmse: float
    mean: float
    median: float
    std: float
    max_value: float
    count: int


@dataclass(frozen=True)
class RPEResult:
    label: str
    att: MetricStats
    pos: MetricStats
    pair_count: int


@dataclass(frozen=True)
class EstGtSeries:
    time: np.ndarray
    est: np.ndarray
    gt: np.ndarray


def read_csv(path: str) -> tuple[list[str], np.ndarray]:
    with open(path, "r", newline="") as file:
        reader = csv.reader(file)
        try:
            header = [cell.strip() for cell in next(reader)]
        except StopIteration:
            return [], np.empty((0, 0))

        rows: list[list[float]] = []
        for row in reader:
            if not row or all(cell.strip() == "" for cell in row):
                continue
            row = [cell.strip() for cell in row]
            if len(row) < len(header):
                row.extend([""] * (len(header) - len(row)))
            elif len(row) > len(header):
                row = row[: len(header)]
            try:
                rows.append([float(cell) if cell != "" else math.nan for cell in row])
            except ValueError:
                continue

    if not rows:
        return header, np.empty((0, len(header)))
    return header, np.asarray(rows, dtype=float)


def require_columns(header: Sequence[str], names: Sequence[str], path: str) -> None:
    missing = [name for name in names if name not in header]
    if missing:
        raise ValueError(f"{path} missing columns: {', '.join(missing)}")


def sanitize_time_series(
    time: np.ndarray,
    values: Sequence[np.ndarray],
    path: str,
) -> tuple[np.ndarray, list[np.ndarray]]:
    arrays = [np.asarray(time, dtype=float), *[np.asarray(value, dtype=float) for value in values]]
    mask = np.ones(arrays[0].shape[0], dtype=bool)
    for array in arrays:
        mask &= np.isfinite(array)

    if not np.any(mask):
        raise ValueError(f"No finite samples in {path}")

    clean_time = arrays[0][mask]
    clean_values = [array[mask] for array in arrays[1:]]
    order = np.argsort(clean_time)
    clean_time = clean_time[order]
    clean_values = [value[order] for value in clean_values]

    unique_time, unique_indices = np.unique(clean_time, return_index=True)
    if unique_time.size < 2:
        raise ValueError(f"Need at least two unique timestamps in {path}")

    return unique_time, [value[unique_indices] for value in clean_values]


def load_est_gt_pair(
    path: str,
    est_col: str,
    gt_col: str,
    *,
    unwrap_angles: bool = False,
) -> EstGtSeries:
    header, data = read_csv(path)
    if data.size == 0:
        raise ValueError(f"No valid samples in {path}")
    require_columns(header, (est_col, gt_col), path)
    time = data[:, header.index("time")] if "time" in header else np.arange(data.shape[0], dtype=float)
    est_idx = header.index(est_col)
    gt_idx = header.index(gt_col)
    time, (est, gt) = sanitize_time_series(time, (data[:, est_idx], data[:, gt_idx]), path)
    if unwrap_angles:
        est = np.unwrap(est)
        gt = np.unwrap(gt)
    return EstGtSeries(time=time, est=est, gt=gt)


def align_series_by_time(series: Sequence[EstGtSeries]) -> list[tuple[np.ndarray, np.ndarray]]:
    if not series:
        raise ValueError("No series to align")

    start_time = max(item.time[0] for item in series)
    end_time = min(item.time[-1] for item in series)
    if end_time <= start_time:
        raise ValueError("No overlapping timestamp range across evaluation CSV files")

    reference_time = series[0].time
    time_grid = reference_time[(reference_time >= start_time) & (reference_time <= end_time)]
    if time_grid.size < 2:
        sample_count = min(item.time.size for item in series)
        time_grid = np.linspace(start_time, end_time, sample_count)

    if time_grid.size < 2:
        raise ValueError("No overlapping samples after timestamp alignment")

    return [
        (
            np.interp(time_grid, item.time, item.est),
            np.interp(time_grid, item.time, item.gt),
        )
        for item in series
    ]


def rotation_matrix_zyx(yaw: float, pitch: float, roll: float) -> np.ndarray:
    cz, sz = math.cos(yaw), math.sin(yaw)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cx, sx = math.cos(roll), math.sin(roll)

    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    return rz @ ry @ rx


def pose_from_zyx(position: np.ndarray, yaw: float, pitch: float, roll: float) -> np.ndarray:
    pose = np.eye(4, dtype=float)
    pose[:3, :3] = rotation_matrix_zyx(yaw, pitch, roll)
    pose[:3, 3] = position
    return pose


def invert_se3(pose: np.ndarray) -> np.ndarray:
    rotation = pose[:3, :3]
    translation = pose[:3, 3]
    inverse = np.eye(4, dtype=float)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def rotation_angle_rad(rotation: np.ndarray) -> float:
    trace = float(np.trace(rotation))
    cosine = np.clip((trace - 1.0) * 0.5, -1.0, 1.0)
    return float(math.acos(cosine))


def metric_stats(values: np.ndarray) -> MetricStats:
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        raise ValueError("Cannot compute statistics on an empty array")
    return MetricStats(
        rmse=float(np.sqrt(np.mean(values**2))),
        mean=float(np.mean(values)),
        median=float(np.median(values)),
        std=float(np.std(values)),
        max_value=float(np.max(values)),
        count=int(values.size),
    )


def cumulative_arc_length(positions: np.ndarray) -> np.ndarray:
    arc_length = np.zeros(positions.shape[0], dtype=float)
    if positions.shape[0] < 2:
        return arc_length
    deltas = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    arc_length[1:] = np.cumsum(deltas)
    return arc_length


def cumulative_rotation_angle(poses: np.ndarray) -> np.ndarray:
    rotation_arc = np.zeros(poses.shape[0], dtype=float)
    if poses.shape[0] < 2:
        return rotation_arc

    for index in range(1, poses.shape[0]):
        delta_rotation = poses[index - 1, :3, :3].T @ poses[index, :3, :3]
        rotation_arc[index] = rotation_arc[index - 1] + rotation_angle_rad(delta_rotation)
    return rotation_arc


def compute_ate_pos(pos_est: np.ndarray, pos_gt: np.ndarray) -> MetricStats:
    if pos_est.shape != pos_gt.shape or pos_est.shape[1] != 3:
        raise ValueError("Position arrays must both be Nx3 with matching shapes")
    errors = np.linalg.norm(pos_est - pos_gt, axis=1)
    return metric_stats(errors)


def compute_ate_att(poses_est: np.ndarray, poses_gt: np.ndarray) -> MetricStats:
    errors = np.empty(poses_est.shape[0], dtype=float)
    for index in range(poses_est.shape[0]):
        rotation_error = poses_gt[index, :3, :3].T @ poses_est[index, :3, :3]
        errors[index] = rotation_angle_rad(rotation_error)
    return metric_stats(errors)


def build_poses(
    positions: np.ndarray,
    yaws: np.ndarray,
    pitches: np.ndarray,
    rolls: np.ndarray,
) -> np.ndarray:
    poses = np.zeros((positions.shape[0], 4, 4), dtype=float)
    for index in range(positions.shape[0]):
        poses[index] = pose_from_zyx(
            positions[index],
            float(yaws[index]),
            float(pitches[index]),
            float(rolls[index]),
        )
    return poses


def compute_rpe_errors(
    poses_est: np.ndarray,
    poses_gt: np.ndarray,
    pairs: Sequence[tuple[int, int]],
) -> tuple[np.ndarray, np.ndarray]:
    pos_errors: list[float] = []
    att_errors: list[float] = []

    for start, end in pairs:
        gt_rel = invert_se3(poses_gt[start]) @ poses_gt[end]
        est_rel = invert_se3(poses_est[start]) @ poses_est[end]
        error_pose = invert_se3(gt_rel) @ est_rel
        pos_errors.append(float(np.linalg.norm(error_pose[:3, 3])))
        att_errors.append(rotation_angle_rad(error_pose[:3, :3]))

    return np.asarray(pos_errors, dtype=float), np.asarray(att_errors, dtype=float)


def frame_pairs(num_samples: int, delta_frames: int) -> list[tuple[int, int]]:
    if delta_frames <= 0:
        return []
    return [(index, index + delta_frames) for index in range(0, num_samples - delta_frames)]


def arc_delta_pairs(
    arc_length: np.ndarray,
    delta: float,
    tolerance_ratio: float,
) -> list[tuple[int, int]]:
    if delta <= 0.0 or arc_length.shape[0] < 2:
        return []

    tolerance = delta * tolerance_ratio
    pairs: list[tuple[int, int]] = []

    for start in range(arc_length.shape[0] - 1):
        target_distance = arc_length[start] + delta
        if target_distance > arc_length[-1]:
            continue

        candidate_indices = np.arange(start + 1, arc_length.shape[0])
        segment_lengths = arc_length[candidate_indices] - arc_length[start]
        distance_error = np.abs(segment_lengths - delta)
        end = int(candidate_indices[int(np.argmin(distance_error))])
        if abs((arc_length[end] - arc_length[start]) - delta) <= tolerance:
            pairs.append((start, end))

    return pairs


def distance_pairs(
    pos_gt: np.ndarray,
    delta_meters: float,
    tolerance_ratio: float = DISTANCE_TOLERANCE_RATIO,
) -> list[tuple[int, int]]:
    return arc_delta_pairs(cumulative_arc_length(pos_gt), delta_meters, tolerance_ratio)


def rotation_pairs(
    poses_gt: np.ndarray,
    delta_radians: float,
    tolerance_ratio: float = ANGLE_TOLERANCE_RATIO,
) -> list[tuple[int, int]]:
    return arc_delta_pairs(
        cumulative_rotation_angle(poses_gt),
        delta_radians,
        tolerance_ratio,
    )


def compute_rpe(
    poses_est: np.ndarray,
    poses_gt: np.ndarray,
    pos_gt: np.ndarray,
    label: str,
    pairs: Sequence[tuple[int, int]],
) -> RPEResult | None:
    if not pairs:
        return None

    pos_errors, att_errors = compute_rpe_errors(poses_est, poses_gt, pairs)
    return RPEResult(
        label=label,
        pos=metric_stats(pos_errors),
        att=metric_stats(att_errors),
        pair_count=pos_errors.size,
    )


def load_trajectory() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    aligned = align_series_by_time(
        [
            load_est_gt_pair(POS_EST_TRUE_X, "est_x", "gt_x"),
            load_est_gt_pair(POS_EST_TRUE_Y, "est_y", "gt_y"),
            load_est_gt_pair(POS_EST_TRUE_Z, "est_z", "gt_z"),
            load_est_gt_pair(ROT_EST_TRUE_YAW, "est_yaw", "gt_yaw", unwrap_angles=True),
            load_est_gt_pair(ROT_EST_TRUE_PITCH, "est_pitch", "gt_pitch", unwrap_angles=True),
            load_est_gt_pair(ROT_EST_TRUE_ROLL, "est_roll", "gt_roll", unwrap_angles=True),
        ]
    )
    (
        (est_x, gt_x),
        (est_y, gt_y),
        (est_z, gt_z),
        (est_yaw, gt_yaw),
        (est_pitch, gt_pitch),
        (est_roll, gt_roll),
    ) = aligned

    pos_est = np.column_stack((est_x, est_y, est_z))
    pos_gt = np.column_stack((gt_x, gt_y, gt_z))
    poses_est = build_poses(pos_est, est_yaw, est_pitch, est_roll)
    poses_gt = build_poses(pos_gt, gt_yaw, gt_pitch, gt_roll)
    return pos_est, pos_gt, poses_est, poses_gt


def format_metric_block(name: str, stats: MetricStats, unit: str) -> list[str]:
    return [
        f"{name}:",
        f"  mean   : {stats.mean:.6f} {unit}",
        f"  median : {stats.median:.6f} {unit}",
        f"  std    : {stats.std:.6f} {unit}",
        f"  max    : {stats.max_value:.6f} {unit}",
        f"  count  : {stats.count}",
    ]


def format_rpe_block(result: RPEResult) -> list[str]:
    suffix = result.label.removeprefix("RPE ").strip()
    lines = [f"RPEatt {suffix}:"]
    lines.extend(
        [
            f"  mean   : {result.att.mean:.6f} rad",
            f"  median : {result.att.median:.6f} rad",
            f"  std    : {result.att.std:.6f} rad",
            f"  max    : {result.att.max_value:.6f} rad",
            f"  count  : {result.pair_count}",
        ]
    )
    lines.append(f"RPEpos {suffix}:")
    lines.extend(
        [
            f"  mean   : {result.pos.mean:.6f} m",
            f"  median : {result.pos.median:.6f} m",
            f"  std    : {result.pos.std:.6f} m",
            f"  max    : {result.pos.max_value:.6f} m",
            f"  count  : {result.pair_count}",
        ]
    )
    return lines


def is_one_meter_rpe(result: RPEResult) -> bool:
    label = result.label.removeprefix("RPE ").strip()
    if not label.endswith("m"):
        return False
    try:
        return math.isclose(float(label.removesuffix("m")), 1.0)
    except ValueError:
        return False


def format_rotation_delta_label(delta_radians: float) -> str:
    degrees = math.degrees(delta_radians)
    if math.isclose(degrees, round(degrees), abs_tol=0.05):
        return f"RPE {round(degrees):g}deg"
    return f"RPE {delta_radians:g} rad"


def pick_summary_rpe(results: Sequence[RPEResult]) -> RPEResult | None:
    for result in results:
        if is_one_meter_rpe(result):
            return result
    return results[0] if results else None


def build_report(
    ate_att: MetricStats,
    ate_pos: MetricStats,
    rpe_results: Sequence[RPEResult],
) -> str:
    lines = [
        "Trajectory Evaluation Report",
        f"Data directory: {DATA_DIR}",
        "",
        "Summary (RMSE):",
        "  Metric    Value      Unit",
        f"  ATEatt    {ate_att.rmse:.6f}   rad",
        f"  ATEpos    {ate_pos.rmse:.6f}   m",
    ]

    summary_rpe = pick_summary_rpe(rpe_results)
    if summary_rpe is not None:
        lines.append(f"  RPEatt    {summary_rpe.att.rmse:.6f}   rad")
        lines.append(f"  RPEpos    {summary_rpe.pos.rmse:.6f}   m")

    lines.extend(["", "Detailed statistics:"])
    lines.extend(format_metric_block("ATEatt", ate_att, "rad"))
    lines.append("")
    lines.extend(format_metric_block("ATEpos", ate_pos, "m"))

    for result in rpe_results:
        lines.append("")
        lines.extend(format_rpe_block(result))

    if not rpe_results:
        lines.extend(["", "RPE: no valid pose pairs for the configured deltas."])

    return "\n".join(lines).rstrip() + "\n"


def main() -> None:
    required_files = (
        POS_EST_TRUE_X,
        POS_EST_TRUE_Y,
        POS_EST_TRUE_Z,
        ROT_EST_TRUE_YAW,
        ROT_EST_TRUE_PITCH,
        ROT_EST_TRUE_ROLL,
    )
    missing = [path for path in required_files if not os.path.isfile(path)]
    if missing:
        raise FileNotFoundError("Missing required CSV files:\n" + "\n".join(missing))

    pos_est, pos_gt, poses_est, poses_gt = load_trajectory()
    ate_att = compute_ate_att(poses_est, poses_gt)
    ate_pos = compute_ate_pos(pos_est, pos_gt)

    rpe_results: list[RPEResult] = []
    for delta_frames in RPE_DELTA_FRAMES:
        result = compute_rpe(
            poses_est,
            poses_gt,
            pos_gt,
            f"RPE delta={delta_frames} frames",
            frame_pairs(pos_est.shape[0], delta_frames),
        )
        if result is not None:
            rpe_results.append(result)

    for delta_meters in RPE_DELTA_METERS:
        result = compute_rpe(
            poses_est,
            poses_gt,
            pos_gt,
            f"RPE {delta_meters:g}m",
            distance_pairs(pos_gt, delta_meters),
        )
        if result is not None:
            rpe_results.append(result)

    if not rpe_results:
        for delta_radians in RPE_DELTA_RADIANS:
            result = compute_rpe(
                poses_est,
                poses_gt,
                pos_gt,
                format_rotation_delta_label(delta_radians),
                rotation_pairs(poses_gt, delta_radians),
            )
            if result is not None:
                rpe_results.append(result)

    os.makedirs(OUT_DIR, exist_ok=True)
    report = build_report(ate_att, ate_pos, rpe_results)
    with open(REPORT_TXT, "w", encoding="utf-8") as file:
        file.write(report)

    print(report)
    print(f"Saved report to {REPORT_TXT}")


if __name__ == "__main__":
    main()
