"""Comparison reports for Smarpod/hexapod calibration assessments.

This module turns the old standalone ``testing.py`` workflow into a reusable
report generator for this package.  After each assessment it can compare:

* raw ball measurement positions between compatible measurement files
* solved pivot-calibration values between compatible result files
* per-set fitted sphere centers and sphere-fit quality

Files are grouped by their exact pose-id or pivot-id set before comparison so
we do not compare unrelated calibration runs.
"""
from __future__ import annotations

import argparse
import csv
import json
import re
from collections import defaultdict
from itertools import combinations
from pathlib import Path
from typing import Iterable, Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


AXES = ("x", "y", "z")


def natural_key(text: str) -> list[object]:
    """Sort names such as ``Ball_2`` before ``Ball_10``."""
    return [
        int(part) if part.isdigit() else part.lower()
        for part in re.split(r"(\d+)", text)
    ]


def write_csv(rows: list[dict[str, object]], path: Path) -> None:
    """Write rows to CSV, creating a small marker file for empty tables."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("no_data\n", encoding="utf-8")
        return

    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _label_from_stem(stem: str, prefix: str) -> str:
    if stem.startswith(prefix):
        return stem[len(prefix):]
    return stem


def _result_label(path: Path) -> str:
    return _label_from_stem(path.stem, "results_calibrate_smarpod_measurement_")


def _measurement_label(path: Path) -> str:
    return _label_from_stem(path.stem, "calibrate_smarpod_measurement_")


def _short_label(label: str) -> str:
    return label[-6:] if len(label) > 6 else label


def read_measurements(path: Path) -> tuple[dict, dict[tuple[str, str], np.ndarray]]:
    """Return raw ball positions in micrometres indexed by ``(pose_id, ball)``."""
    with path.open(encoding="utf-8") as file:
        document = json.load(file)

    measurements: dict[tuple[str, str], np.ndarray] = {}
    for pose in document.get("calibration_data", []):
        pose_id = str(pose["pose_id"])
        for result in pose.get("results_list", []):
            for ball_name, ball_data in result.items():
                translation = ball_data["translation"]
                key = (pose_id, str(ball_name))
                if key in measurements:
                    raise ValueError(
                        f"Duplicate measurement for pose {pose_id!r}, "
                        f"ball {ball_name!r} in {path.name}"
                    )
                measurements[key] = np.array(
                    [float(translation[axis]) for axis in AXES],
                    dtype=float,
                )

    if not measurements:
        raise ValueError(f"No ball measurements found in {path}")
    return document, measurements


def match_measurements(
    first: dict[tuple[str, str], np.ndarray],
    second: dict[tuple[str, str], np.ndarray],
) -> tuple[list[dict[str, object]], set[tuple[str, str]], set[tuple[str, str]]]:
    """Match records by pose and ball and calculate ``second - first``."""
    first_keys = set(first)
    second_keys = set(second)
    common_keys = first_keys & second_keys
    if not common_keys:
        raise ValueError("The files have no matching pose/ball measurements.")

    rows: list[dict[str, object]] = []
    for pose_id, ball_name in sorted(
        common_keys,
        key=lambda item: (natural_key(item[0]), natural_key(item[1])),
    ):
        position_1 = first[(pose_id, ball_name)]
        position_2 = second[(pose_id, ball_name)]
        difference = position_2 - position_1
        rows.append({
            "pose_id": pose_id,
            "ball": ball_name,
            "x_1_um": position_1[0],
            "y_1_um": position_1[1],
            "z_1_um": position_1[2],
            "x_2_um": position_2[0],
            "y_2_um": position_2[1],
            "z_2_um": position_2[2],
            "dx_um": difference[0],
            "dy_um": difference[1],
            "dz_um": difference[2],
            "distance_um": float(np.linalg.norm(difference)),
        })
    return rows, first_keys - second_keys, second_keys - first_keys


def read_pivot_calibration(path: Path) -> dict[str, object]:
    """Read solved ``B_T_P`` and ``J_t_P`` data from an assessment result."""
    with path.open(encoding="utf-8") as file:
        document = json.load(file)

    try:
        pivot = document["pivot_calibration"]
        b_t_p = pivot["B_T_P"]
        translation = b_t_p["translation_mm"]
        rotation_matrix = np.asarray(b_t_p["rotation_matrix"], dtype=float)
        euler = b_t_p.get("rotation_euler_zyx_deg") or b_t_p["rotation_euler_deg"]
        j_t_p = pivot["J_t_P_mm"]
    except (KeyError, TypeError) as error:
        raise ValueError(f"Missing B_T_P or J_t_P_mm data in {path}") from error

    if rotation_matrix.shape != (3, 3):
        raise ValueError(f"B_T_P rotation matrix in {path} is not 3 x 3.")

    return {
        "metadata": document.get("metadata", {}) or {},
        "B_T_P translation": np.array(
            [float(translation[axis]) for axis in AXES],
            dtype=float,
        ),
        "B_T_P Euler ZYX": np.array(
            [float(euler[angle]) for angle in ("yaw", "pitch", "roll")],
            dtype=float,
        ),
        "B_T_P rotation matrix": rotation_matrix,
        "J_t_P_mm": np.array(
            [float(j_t_p[axis]) for axis in AXES],
            dtype=float,
        ),
    }


def _calibration_id_from_mapping(data: dict) -> str:
    """Return the best available calibration id from a result/log mapping."""
    metadata = data.get("metadata", {}) if isinstance(data, dict) else {}
    candidates = (
        metadata.get("cal_id") if isinstance(metadata, dict) else None,
        metadata.get("id") if isinstance(metadata, dict) else None,
        data.get("cal_id") if isinstance(data, dict) else None,
        data.get("id") if isinstance(data, dict) else None,
        data.get("calibration_id") if isinstance(data, dict) else None,
    )
    for candidate in candidates:
        if candidate is None:
            continue
        value = str(candidate).strip()
        if value:
            return value
    return ""


def _measurement_path_from_mapping(data: dict) -> str:
    metadata = data.get("metadata", {}) if isinstance(data, dict) else {}
    candidates = (
        metadata.get("measurement_file_path") if isinstance(metadata, dict) else None,
        data.get("measurement_file_path") if isinstance(data, dict) else None,
    )
    for candidate in candidates:
        if candidate is None:
            continue
        value = str(candidate).strip()
        if value:
            return value
    return ""


def _timestamp_from_mapping(data: dict) -> str:
    candidate = data.get("timestamp") if isinstance(data, dict) else None
    return str(candidate).strip() if candidate is not None else ""


def _vector_from_nested_mapping(
    data: dict,
    outer_key: str,
    components: tuple[str, ...],
) -> np.ndarray:
    values = data.get(outer_key, {}) if isinstance(data, dict) else {}
    if not isinstance(values, dict):
        values = {}
    return np.array([float(values.get(component, 0.0)) for component in components])


def discover_smarpod_station_log(current_measurement_path: Path) -> Optional[Path]:
    """Find the Smarpod station calibration-history JSON near measurements."""
    measurement_dir = current_measurement_path.parent
    candidates = (
        measurement_dir.parent / "Smarpod_Station.json",
        measurement_dir / "Smarpod_Station.json",
        measurement_dir.parent / "calibration_logs" / "Smarpod_Station.json",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return None


def read_smarpod_station_history(path: Path) -> list[dict[str, object]]:
    """Read calibration history entries from ``Smarpod_Station.json``."""
    with path.open(encoding="utf-8") as file:
        document = json.load(file)
    history = document.get("calibration_history", [])
    if not isinstance(history, list):
        raise ValueError(f"Invalid calibration_history in {path}")

    entries: list[dict[str, object]] = []
    for index, entry in enumerate(history, start=1):
        if not isinstance(entry, dict):
            continue
        update = entry.get("joint_value_update", {})
        if not isinstance(update, dict):
            update = {}
        change = update.get("joint_value_change", {})
        previous = update.get("previous_joint_value", {})
        new = update.get("new_joint_value", {})
        if not isinstance(change, dict):
            change = {}
        if not isinstance(previous, dict):
            previous = {}
        if not isinstance(new, dict):
            new = {}

        entries.append({
            "sequence": index,
            "cal_id": _calibration_id_from_mapping(entry),
            "measurement_file_path": _measurement_path_from_mapping(entry),
            "timestamp": _timestamp_from_mapping(entry),
            "joint_name": update.get("joint_name", "Smarpod_Station"),
            "update_mode": update.get("update_mode", ""),
            "translation_change_um": _vector_from_nested_mapping(
                change,
                "translation",
                AXES,
            ),
            "rotation_change_deg": _vector_from_nested_mapping(
                change,
                "rotation",
                ("rx", "ry", "rz"),
            ),
            "previous_translation_um": _vector_from_nested_mapping(
                previous,
                "translation",
                AXES,
            ),
            "new_translation_um": _vector_from_nested_mapping(
                new,
                "translation",
                AXES,
            ),
            "previous_rotation_deg": _vector_from_nested_mapping(
                previous,
                "rotation",
                ("rx", "ry", "rz"),
            ),
            "new_rotation_deg": _vector_from_nested_mapping(
                new,
                "rotation",
                ("rx", "ry", "rz"),
            ),
        })
    return entries


def _measurement_path_matches(left: str, right: str) -> bool:
    if not left or not right:
        return False
    left_path = Path(left)
    right_path = Path(right)
    return (
        left == right
        or left_path.name == right_path.name
        or str(left_path.resolve()) == str(right_path.resolve())
    )


def match_station_entries_to_results(
    station_entries: list[dict[str, object]],
    calibration_results: list[tuple[Path, str, dict[str, object]]],
) -> list[dict[str, object]]:
    """Join ``Smarpod_Station.json`` entries to assessed result files."""
    rows: list[dict[str, object]] = []
    unmatched_entries = list(station_entries)

    for result_path, label, result in calibration_results:
        metadata = result.get("metadata", {}) if isinstance(result, dict) else {}
        cal_id = _calibration_id_from_mapping({"metadata": metadata})
        measurement_file_path = _measurement_path_from_mapping({"metadata": metadata})
        match: Optional[dict[str, object]] = None

        if cal_id:
            for entry in unmatched_entries:
                if str(entry.get("cal_id", "")) == cal_id:
                    match = entry
                    break

        if match is None and measurement_file_path:
            for entry in unmatched_entries:
                if _measurement_path_matches(
                    str(entry.get("measurement_file_path", "")),
                    measurement_file_path,
                ):
                    match = entry
                    break

        if match is None:
            continue

        unmatched_entries.remove(match)
        translation_change = np.asarray(match["translation_change_um"], dtype=float)
        rotation_change = np.asarray(match["rotation_change_deg"], dtype=float)
        previous_translation = np.asarray(match["previous_translation_um"], dtype=float)
        new_translation = np.asarray(match["new_translation_um"], dtype=float)
        previous_rotation = np.asarray(match["previous_rotation_deg"], dtype=float)
        new_rotation = np.asarray(match["new_rotation_deg"], dtype=float)
        rows.append({
            "result": label,
            "result_file": str(result_path),
            "cal_id": cal_id or str(match.get("cal_id", "")),
            "measurement_file_path": measurement_file_path
            or str(match.get("measurement_file_path", "")),
            "station_log_sequence": int(match["sequence"]),
            "station_log_timestamp": str(match.get("timestamp", "")),
            "update_mode": str(match.get("update_mode", "")),
            "dx_um": translation_change[0],
            "dy_um": translation_change[1],
            "dz_um": translation_change[2],
            "translation_change_norm_um": float(np.linalg.norm(translation_change)),
            "drx_deg": rotation_change[0],
            "dry_deg": rotation_change[1],
            "drz_deg": rotation_change[2],
            "rotation_change_norm_deg": float(np.linalg.norm(rotation_change)),
            "previous_x_um": previous_translation[0],
            "previous_y_um": previous_translation[1],
            "previous_z_um": previous_translation[2],
            "new_x_um": new_translation[0],
            "new_y_um": new_translation[1],
            "new_z_um": new_translation[2],
            "previous_rx_deg": previous_rotation[0],
            "previous_ry_deg": previous_rotation[1],
            "previous_rz_deg": previous_rotation[2],
            "new_rx_deg": new_rotation[0],
            "new_ry_deg": new_rotation[1],
            "new_rz_deg": new_rotation[2],
        })
    rows.sort(key=lambda row: natural_key(str(row["result"])))
    return rows


def relative_rotation_angle_deg(first: np.ndarray, second: np.ndarray) -> float:
    """Return the smallest angle of the rotation taking ``first`` to ``second``."""
    relative = second @ first.T
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def discover_measurement_files(
    current_measurement_path: Path,
    extra_paths: Optional[Iterable[Path]] = None,
) -> list[Path]:
    """Find raw calibration measurement files near the current run."""
    measurement_dir = current_measurement_path.parent
    paths = {
        current_measurement_path.resolve(),
        *(path.resolve() for path in measurement_dir.glob("calibrate_smarpod_measurement_*.json")),
    }
    if extra_paths:
        paths.update(path.resolve() for path in extra_paths)
    return sorted(
        (path for path in paths if path.is_file()),
        key=lambda path: natural_key(path.name),
    )


def discover_result_files(
    current_measurement_path: Path,
    current_result_path: Optional[Path] = None,
) -> list[Path]:
    """Find processed assessment result files near the current run."""
    measurement_dir = current_measurement_path.parent
    result_root = measurement_dir / "calibrate_smarpod_results"
    patterns = (
        result_root.glob("*/results_calibrate_smarpod_measurement_*.json"),
        result_root.glob("results_calibrate_smarpod_measurement_*.json"),
        measurement_dir.glob("results_calibrate_smarpod_measurement_*.json"),
        (measurement_dir / "results_calibration").glob(
            "results_calibrate_smarpod_measurement_*.json"
        ),
    )
    paths: set[Path] = set()
    for matches in patterns:
        paths.update(path.resolve() for path in matches)
    if current_result_path is not None:
        paths.add(current_result_path.resolve())
    return sorted(
        (path for path in paths if path.is_file()),
        key=lambda path: natural_key(str(path)),
    )


def group_measurements_by_pose_ids(
    paths: list[Path],
) -> list[tuple[tuple[str, ...], list[tuple[Path, str]]]]:
    """Group raw measurement files by exact pose-id set."""
    grouped: dict[tuple[str, ...], list[tuple[Path, str]]] = defaultdict(list)
    for path in paths:
        with path.open(encoding="utf-8") as file:
            document = json.load(file)
        pose_ids = tuple(sorted(
            {str(pose["pose_id"]) for pose in document.get("calibration_data", [])},
            key=natural_key,
        ))
        if pose_ids:
            grouped[pose_ids].append((path, _measurement_label(path)))

    groups = sorted(grouped.items(), key=lambda item: natural_key(item[1][0][1]))
    for _, files in groups:
        files.sort(key=lambda item: natural_key(item[1]))
    return groups


def read_pivots(path: Path) -> dict[str, dict[str, object]]:
    """Read the per-set sphere fit results from an assessment result file."""
    with path.open(encoding="utf-8") as file:
        document = json.load(file)
    pivots = document.get("sphere_fits_by_set") or document.get("pivots")
    if not isinstance(pivots, dict) or not pivots:
        raise ValueError(f"No sphere fit results found in {path}")
    return pivots


def group_results_by_pivot_ids(
    result_paths: list[Path],
) -> list[tuple[tuple[str, ...], list[tuple[Path, str, dict[str, dict[str, object]]]]]]:
    """Group result files by exact fitted-pivot id set."""
    grouped: dict[
        tuple[str, ...],
        list[tuple[Path, str, dict[str, dict[str, object]]]],
    ] = defaultdict(list)
    for path in result_paths:
        pivots = read_pivots(path)
        pivot_ids = tuple(sorted(pivots, key=natural_key))
        grouped[pivot_ids].append((path, _result_label(path), pivots))

    groups = sorted(grouped.items(), key=lambda item: natural_key(item[1][0][1]))
    for _, results in groups:
        results.sort(key=lambda item: natural_key(item[1]))
    return groups


def write_measurement_id_set_manifest(
    groups: list[tuple[tuple[str, ...], list[tuple[Path, str]]]],
    path: Path,
) -> None:
    rows: list[dict[str, object]] = []
    for group_index, (pose_ids, files) in enumerate(groups, start=1):
        for measurement_path, label in files:
            rows.append({
                "id_set_group": group_index,
                "pose_id_count": len(pose_ids),
                "measurement": label,
                "file": str(measurement_path),
                "first_pose_id": pose_ids[0],
                "last_pose_id": pose_ids[-1],
            })
    write_csv(rows, path)


def write_pivot_id_set_manifest(
    groups: list[tuple[tuple[str, ...], list[tuple[Path, str, dict[str, dict[str, object]]]]]],
    path: Path,
) -> None:
    rows: list[dict[str, object]] = []
    for group_index, (pivot_ids, results) in enumerate(groups, start=1):
        for result_path, label, _ in results:
            rows.append({
                "id_set_group": group_index,
                "pivot_count": len(pivot_ids),
                "result": label,
                "file": str(result_path),
                "first_pivot_id": pivot_ids[0],
                "last_pivot_id": pivot_ids[-1],
            })
    write_csv(rows, path)


def all_pivot_comparison_rows(
    results: list[tuple[Path, str, dict[str, object]]],
) -> list[dict[str, object]]:
    """Create calibration value rows relative to the first compatible result."""
    baseline = results[0][2]
    groups = (
        ("B_T_P translation", ("x", "y", "z"), "um", 1000.0),
        ("B_T_P Euler ZYX", ("yaw", "pitch", "roll"), "deg", 1.0),
        ("J_t_P_mm", ("x", "y", "z"), "um", 1000.0),
    )
    rows: list[dict[str, object]] = []
    for path, label, result in results:
        for group, components, unit, scale in groups:
            values = np.asarray(result[group], dtype=float) * scale
            baseline_values = np.asarray(baseline[group], dtype=float) * scale
            for component, value, baseline_value in zip(components, values, baseline_values):
                rows.append({
                    "result": label,
                    "file": str(path),
                    "parameter": group,
                    "component": component,
                    "unit": unit,
                    "value": float(value),
                    "difference_from_first": float(value - baseline_value),
                })

        angle = relative_rotation_angle_deg(
            np.asarray(baseline["B_T_P rotation matrix"], dtype=float),
            np.asarray(result["B_T_P rotation matrix"], dtype=float),
        )
        rows.append({
            "result": label,
            "file": str(path),
            "parameter": "B_T_P relative rotation",
            "component": "angle",
            "unit": "deg",
            "value": angle,
            "difference_from_first": angle,
        })
    return rows


def pairwise_pivot_rows(
    results: list[tuple[Path, str, dict[str, object]]],
) -> list[dict[str, object]]:
    """Calculate vector/rotation distances between every pair of results."""
    rows: list[dict[str, object]] = []
    for index_1 in range(len(results)):
        for index_2 in range(index_1 + 1, len(results)):
            _, label_1, result_1 = results[index_1]
            _, label_2, result_2 = results[index_2]
            b_distance = float(np.linalg.norm(
                np.asarray(result_2["B_T_P translation"])
                - np.asarray(result_1["B_T_P translation"])
            ))
            j_distance = float(np.linalg.norm(
                np.asarray(result_2["J_t_P_mm"])
                - np.asarray(result_1["J_t_P_mm"])
            ))
            rotation_angle = relative_rotation_angle_deg(
                np.asarray(result_1["B_T_P rotation matrix"], dtype=float),
                np.asarray(result_2["B_T_P rotation matrix"], dtype=float),
            )
            rows.append({
                "result_1": label_1,
                "result_2": label_2,
                "B_T_P_translation_distance_um": b_distance * 1000.0,
                "B_T_P_relative_rotation_deg": rotation_angle,
                "J_t_P_distance_um": j_distance * 1000.0,
            })
    return rows


def pivot_set_comparison_rows(
    pivot_ids: tuple[str, ...],
    results: list[tuple[Path, str, dict[str, dict[str, object]]]],
) -> list[dict[str, object]]:
    """Create per-sphere-center values and differences from the first run."""
    baseline_pivots = results[0][2]
    rows: list[dict[str, object]] = []
    for path, label, pivots in results:
        for sequence, pivot_id in enumerate(pivot_ids, start=1):
            pivot = pivots[pivot_id]
            baseline = baseline_pivots[pivot_id]
            center = np.array(
                [float(pivot["center_mm"][axis]) for axis in AXES],
                dtype=float,
            ) * 1000.0
            baseline_center = np.array(
                [float(baseline["center_mm"][axis]) for axis in AXES],
                dtype=float,
            ) * 1000.0
            difference = center - baseline_center
            radius_um = float(pivot["radius_mm"]) * 1000.0
            baseline_radius_um = float(baseline["radius_mm"]) * 1000.0
            rows.append({
                "result": label,
                "file": str(path),
                "pivot_sequence": sequence,
                "pivot_id": pivot_id,
                "center_x_um": center[0],
                "center_y_um": center[1],
                "center_z_um": center[2],
                "delta_center_x_um": difference[0],
                "delta_center_y_um": difference[1],
                "delta_center_z_um": difference[2],
                "center_distance_from_first_um": float(np.linalg.norm(difference)),
                "radius_um": radius_um,
                "radius_difference_from_first_um": radius_um - baseline_radius_um,
                "rms_error_um": float(pivot["rms_error_um"]),
                "max_abs_error_um": float(pivot["max_abs_error_um"]),
                "point_count": int(pivot["point_count"]),
            })
    return rows


def plot_raw_axis_differences(rows: list[dict[str, object]], output_path: Path) -> None:
    by_ball: dict[str, list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        by_ball[str(row["ball"])].append(row)

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    colors = plt.cm.tab10(np.linspace(0, 1, max(1, len(by_ball))))
    for color, ball in zip(colors, sorted(by_ball, key=natural_key)):
        ball_rows = by_ball[ball]
        samples = np.arange(1, len(ball_rows) + 1)
        for axis, delta_name in zip(axes, ("dx_um", "dy_um", "dz_um")):
            axis.plot(
                samples,
                [float(row[delta_name]) for row in ball_rows],
                marker=".",
                linewidth=1.0,
                markersize=3,
                color=color,
                label=ball.replace("CAL_Smarpod_", ""),
            )

    for axis, label in zip(axes, ("Delta X", "Delta Y", "Delta Z")):
        axis.axhline(0, color="black", linewidth=0.8)
        axis.set_ylabel(f"{label} (um)")
        axis.grid(True, alpha=0.25)
    axes[0].set_title("Raw ball coordinate differences (measurement 2 - measurement 1)")
    axes[-1].set_xlabel("Pose sequence")
    axes[0].legend(ncol=3, fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_raw_distance_heatmap(rows: list[dict[str, object]], output_path: Path) -> None:
    balls = sorted({str(row["ball"]) for row in rows}, key=natural_key)
    poses = sorted({str(row["pose_id"]) for row in rows}, key=natural_key)
    lookup = {
        (str(row["ball"]), str(row["pose_id"])): float(row["distance_um"])
        for row in rows
    }
    matrix = np.array(
        [[lookup.get((ball, pose), np.nan) for pose in poses] for ball in balls],
        dtype=float,
    )

    fig, axis = plt.subplots(figsize=(15, 5))
    image = axis.imshow(matrix, aspect="auto", interpolation="nearest", cmap="viridis")
    axis.set_title("Raw 3-D difference magnitude for every matched ball measurement")
    axis.set_xlabel("Pose sequence")
    axis.set_ylabel("Ball")
    axis.set_yticks(range(len(balls)))
    axis.set_yticklabels([ball.replace("CAL_Smarpod_Ball_", "Ball ") for ball in balls])
    tick_indices = np.linspace(0, len(poses) - 1, min(12, len(poses)), dtype=int)
    axis.set_xticks(tick_indices)
    axis.set_xticklabels(tick_indices + 1)
    colorbar = fig.colorbar(image, ax=axis)
    colorbar.set_label("Distance (um)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_all_pivot_series(
    results: list[tuple[Path, str, dict[str, object]]],
    output_path: Path,
    differences: bool,
) -> None:
    labels = [_short_label(label) for _, label, _ in results]
    groups = (
        ("B_T_P translation", ("X", "Y", "Z"), "um", 1000.0),
        ("B_T_P Euler ZYX", ("Yaw", "Pitch", "Roll"), "deg", 1.0),
        ("J_t_P_mm", ("X", "Y", "Z"), "um", 1000.0),
    )
    baseline = results[0][2]
    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True)
    positions = np.arange(len(results))

    for axis, (key, components, unit, scale) in zip(axes, groups):
        values = np.asarray([result[key] for _, _, result in results], dtype=float) * scale
        if differences:
            values = values - np.asarray(baseline[key], dtype=float) * scale
        for component_index, component in enumerate(components):
            axis.plot(
                positions,
                values[:, component_index],
                marker="o",
                linewidth=1.8,
                label=component,
            )
        axis.axhline(0, color="black", linewidth=0.8)
        axis.set_title(key)
        axis.set_ylabel(f"{'Difference from first' if differences else 'Value'} ({unit})")
        axis.grid(True, alpha=0.25)
        axis.legend(ncol=3)

    axes[-1].set_xticks(positions, labels, rotation=30, ha="right")
    axes[-1].set_xlabel("Assessment result")
    fig.suptitle(
        "Pivot-calibration differences within one pivot-ID set"
        if differences
        else "Pivot-calibration values within one pivot-ID set"
    )
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_pairwise_pivot_heatmaps(
    results: list[tuple[Path, str, dict[str, object]]],
    pairwise_rows: list[dict[str, object]],
    output_path: Path,
) -> None:
    labels = [_short_label(label) for _, label, _ in results]
    full_labels = [label for _, label, _ in results]
    label_indices = {label: index for index, label in enumerate(full_labels)}
    metrics = (
        ("B_T_P_translation_distance_um", "B_T_P translation", "um"),
        ("B_T_P_relative_rotation_deg", "B_T_P rotation", "deg"),
        ("J_t_P_distance_um", "J_t_P", "um"),
    )
    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    for axis, (metric, title, unit) in zip(axes, metrics):
        matrix = np.zeros((len(results), len(results)), dtype=float)
        for row in pairwise_rows:
            index_1 = label_indices[str(row["result_1"])]
            index_2 = label_indices[str(row["result_2"])]
            matrix[index_1, index_2] = float(row[metric])
            matrix[index_2, index_1] = float(row[metric])
        image = axis.imshow(matrix, cmap="viridis", interpolation="nearest")
        axis.set_title(f"{title} distance ({unit})")
        axis.set_xticks(range(len(labels)), labels, rotation=45, ha="right")
        axis.set_yticks(range(len(labels)), labels)
        threshold = float(np.nanmax(matrix)) * 0.55 if matrix.size else 0.0
        for row_index in range(len(labels)):
            for column_index in range(len(labels)):
                value = matrix[row_index, column_index]
                text_color = "white" if value > threshold else "black"
                axis.text(
                    column_index,
                    row_index,
                    f"{value:.4f}",
                    ha="center",
                    va="center",
                    fontsize=7,
                    color=text_color,
                )
        fig.colorbar(image, ax=axis, fraction=0.046, pad=0.04)

    fig.suptitle("Pairwise calibration comparison within one pivot-ID set")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_station_value_changes(
    rows: list[dict[str, object]],
    output_path: Path,
) -> None:
    """Plot value changes logged in ``Smarpod_Station.json``."""
    labels = [_short_label(str(row["result"])) for row in rows]
    positions = np.arange(len(rows))
    translation_values = np.asarray(
        [[row["dx_um"], row["dy_um"], row["dz_um"]] for row in rows],
        dtype=float,
    )
    rotation_values = np.asarray(
        [[row["drx_deg"], row["dry_deg"], row["drz_deg"]] for row in rows],
        dtype=float,
    )

    fig, axes = plt.subplots(2, 1, figsize=(13, 9), sharex=True)
    for component_index, component in enumerate(("X", "Y", "Z")):
        axes[0].plot(
            positions,
            translation_values[:, component_index],
            marker="o",
            linewidth=1.8,
            label=f"d{component}",
        )
    for component_index, component in enumerate(("RX", "RY", "RZ")):
        axes[1].plot(
            positions,
            rotation_values[:, component_index],
            marker="o",
            linewidth=1.8,
            label=f"d{component}",
        )

    axes[0].set_title("Smarpod_Station.json value changes")
    axes[0].set_ylabel("Translation change (um)")
    axes[1].set_ylabel("Rotation change (deg)")
    axes[1].set_xlabel("Assessment result")
    for axis in axes:
        axis.axhline(0, color="black", linewidth=0.8)
        axis.grid(True, alpha=0.25)
        axis.legend(ncol=3)
    axes[-1].set_xticks(positions, labels, rotation=30, ha="right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_station_change_norms(
    rows: list[dict[str, object]],
    output_path: Path,
) -> None:
    """Plot translation and rotation change magnitudes from station logs."""
    labels = [_short_label(str(row["result"])) for row in rows]
    positions = np.arange(len(rows))
    translation_norms = np.asarray(
        [float(row["translation_change_norm_um"]) for row in rows],
        dtype=float,
    )
    rotation_norms = np.asarray(
        [float(row["rotation_change_norm_deg"]) for row in rows],
        dtype=float,
    )

    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    axes[0].bar(positions, translation_norms, color="#4C78A8", alpha=0.85)
    axes[1].bar(positions, rotation_norms, color="#F58518", alpha=0.85)
    axes[0].set_title("Smarpod_Station.json change magnitudes")
    axes[0].set_ylabel("Translation norm (um)")
    axes[1].set_ylabel("Rotation norm (deg)")
    axes[1].set_xlabel("Assessment result")
    for axis in axes:
        axis.grid(True, axis="y", alpha=0.25)
    axes[-1].set_xticks(positions, labels, rotation=30, ha="right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_pivot_center_differences(
    pivot_ids: tuple[str, ...],
    results: list[tuple[Path, str, dict[str, dict[str, object]]]],
    output_path: Path,
) -> None:
    baseline = results[0][2]
    positions = np.arange(1, len(pivot_ids) + 1)
    fig, axes = plt.subplots(3, 1, figsize=(15, 10), sharex=True)

    for _, label, pivots in results:
        differences = np.array([
            [
                (
                    float(pivots[pivot_id]["center_mm"][axis])
                    - float(baseline[pivot_id]["center_mm"][axis])
                ) * 1000.0
                for axis in AXES
            ]
            for pivot_id in pivot_ids
        ])
        for axis_index, axis in enumerate(axes):
            axis.plot(
                positions,
                differences[:, axis_index],
                marker=".",
                linewidth=1.2,
                label=_short_label(label),
            )

    for axis, component in zip(axes, ("X", "Y", "Z")):
        axis.axhline(0, color="black", linewidth=0.8)
        axis.set_ylabel(f"Delta {component} (um)")
        axis.grid(True, alpha=0.25)
    axes[0].set_title(
        f"Sphere-center differences from {results[0][1]} "
        f"({len(pivot_ids)} matching IDs)"
    )
    axes[0].legend(ncol=min(4, len(results)), fontsize=8)
    axes[-1].set_xlabel("Sphere-set sequence (see sphere_center_comparison.csv)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_pivot_center_distance_heatmap(
    pivot_ids: tuple[str, ...],
    results: list[tuple[Path, str, dict[str, dict[str, object]]]],
    output_path: Path,
) -> None:
    baseline = results[0][2]
    matrix = np.array([
        [
            np.linalg.norm(np.array([
                float(pivots[pivot_id]["center_mm"][axis])
                - float(baseline[pivot_id]["center_mm"][axis])
                for axis in AXES
            ]) * 1000.0)
            for pivot_id in pivot_ids
        ]
        for _, _, pivots in results
    ])

    fig, axis = plt.subplots(figsize=(15, max(4, len(results) * 0.8)))
    image = axis.imshow(matrix, aspect="auto", interpolation="nearest", cmap="magma")
    axis.set_title("Sphere-center distance from the first compatible result")
    axis.set_xlabel("Sphere-set sequence")
    axis.set_ylabel("Assessment result")
    axis.set_yticks(range(len(results)), [_short_label(label) for _, label, _ in results])
    tick_indices = np.linspace(0, len(pivot_ids) - 1, min(12, len(pivot_ids)), dtype=int)
    axis.set_xticks(tick_indices, tick_indices + 1)
    colorbar = fig.colorbar(image, ax=axis)
    colorbar.set_label("Center distance (um)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_pivot_fit_quality(
    pivot_ids: tuple[str, ...],
    results: list[tuple[Path, str, dict[str, dict[str, object]]]],
    output_path: Path,
) -> None:
    positions = np.arange(1, len(pivot_ids) + 1)
    fig, axes = plt.subplots(2, 1, figsize=(15, 8), sharex=True)
    for _, label, pivots in results:
        rms_values = [float(pivots[pivot_id]["rms_error_um"]) for pivot_id in pivot_ids]
        maximum_values = [float(pivots[pivot_id]["max_abs_error_um"]) for pivot_id in pivot_ids]
        axes[0].plot(positions, rms_values, marker=".", linewidth=1.2, label=_short_label(label))
        axes[1].plot(
            positions,
            maximum_values,
            marker=".",
            linewidth=1.2,
            label=_short_label(label),
        )

    axes[0].set_title("Sphere-fit quality")
    axes[0].set_ylabel("RMS error (um)")
    axes[1].set_ylabel("Maximum absolute error (um)")
    axes[1].set_xlabel("Sphere-set sequence (see sphere_center_comparison.csv)")
    for axis in axes:
        axis.grid(True, alpha=0.25)
    axes[0].legend(ncol=min(4, len(results)), fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def _write_raw_measurement_reports(
    measurement_files: list[Path],
    output_dir: Path,
) -> int:
    groups = group_measurements_by_pose_ids(measurement_files)
    write_measurement_id_set_manifest(groups, output_dir / "id_set_manifest.csv")
    comparison_count = 0

    for group_index, (pose_ids, group_files) in enumerate(groups, start=1):
        group_dir = output_dir / f"id_set_{group_index:02d}_{len(pose_ids)}_poses"
        group_dir.mkdir(parents=True, exist_ok=True)
        pair_summary_rows: list[dict[str, object]] = []
        for (file_1, label_1), (file_2, label_2) in combinations(group_files, 2):
            metadata_1, measurements_1 = read_measurements(file_1)
            metadata_2, measurements_2 = read_measurements(file_2)
            rows, only_in_first, only_in_second = match_measurements(measurements_1, measurements_2)

            pair_dir = group_dir / f"{_short_label(label_1)}_vs_{_short_label(label_2)}"
            pair_dir.mkdir(parents=True, exist_ok=True)
            write_csv(rows, pair_dir / "raw_ball_measurement_differences.csv")
            plot_raw_axis_differences(rows, pair_dir / "raw_coordinate_differences.png")
            plot_raw_distance_heatmap(rows, pair_dir / "raw_distance_heatmap.png")

            differences = np.asarray(
                [[row["dx_um"], row["dy_um"], row["dz_um"]] for row in rows],
                dtype=float,
            )
            distances = np.asarray([row["distance_um"] for row in rows], dtype=float)
            pair_summary_rows.append({
                "measurement_1": label_1,
                "measurement_2": label_2,
                "timestamp_1": str(metadata_1.get("timestamp", "")),
                "timestamp_2": str(metadata_2.get("timestamp", "")),
                "matched_measurements": len(rows),
                "only_in_first": len(only_in_first),
                "only_in_second": len(only_in_second),
                "mean_dx_um": float(differences[:, 0].mean()),
                "mean_dy_um": float(differences[:, 1].mean()),
                "mean_dz_um": float(differences[:, 2].mean()),
                "mean_3d_distance_um": float(distances.mean()),
                "max_3d_distance_um": float(distances.max()),
            })
            comparison_count += 1
        write_csv(pair_summary_rows, group_dir / "pairwise_summary.csv")
    return comparison_count


def _write_pivot_result_reports(
    result_files: list[Path],
    output_dir: Path,
    station_log_path: Optional[Path] = None,
) -> int:
    pivot_groups = group_results_by_pivot_ids(result_files)
    write_pivot_id_set_manifest(pivot_groups, output_dir / "id_set_manifest.csv")
    comparison_count = 0

    for group_index, (pivot_ids, group_pivot_results) in enumerate(pivot_groups, start=1):
        group_dir = output_dir / f"id_set_{group_index:02d}_{len(pivot_ids)}_pivots"
        calibration_dir = group_dir / "pivot_calibration"
        centers_dir = group_dir / "sphere_centers"
        calibration_dir.mkdir(parents=True, exist_ok=True)
        centers_dir.mkdir(parents=True, exist_ok=True)

        calibration_results = [
            (path, label, read_pivot_calibration(path))
            for path, label, _ in group_pivot_results
        ]
        calibration_rows = all_pivot_comparison_rows(calibration_results)
        write_csv(calibration_rows, calibration_dir / "pivot_calibration_comparison.csv")
        plot_all_pivot_series(
            calibration_results,
            calibration_dir / "pivot_calibration_values.png",
            differences=False,
        )
        plot_all_pivot_series(
            calibration_results,
            calibration_dir / "pivot_calibration_differences.png",
            differences=True,
        )

        if len(calibration_results) >= 2:
            pairwise_rows = pairwise_pivot_rows(calibration_results)
            write_csv(pairwise_rows, calibration_dir / "pivot_calibration_pairwise.csv")
            plot_pairwise_pivot_heatmaps(
                calibration_results,
                pairwise_rows,
                calibration_dir / "pivot_calibration_pairwise.png",
            )
            comparison_count += len(pairwise_rows)

        if station_log_path is not None:
            station_entries = read_smarpod_station_history(station_log_path)
            station_rows = match_station_entries_to_results(
                station_entries,
                calibration_results,
            )
            station_dir = group_dir / "smarpod_station_joint_config"
            station_dir.mkdir(parents=True, exist_ok=True)
            write_csv(
                station_rows,
                station_dir / "smarpod_station_value_changes.csv",
            )
            if station_rows:
                plot_station_value_changes(
                    station_rows,
                    station_dir / "smarpod_station_value_changes.png",
                )
                plot_station_change_norms(
                    station_rows,
                    station_dir / "smarpod_station_change_norms.png",
                )

        center_rows = pivot_set_comparison_rows(pivot_ids, group_pivot_results)
        write_csv(center_rows, centers_dir / "sphere_center_comparison.csv")
        plot_pivot_center_differences(
            pivot_ids,
            group_pivot_results,
            centers_dir / "sphere_center_coordinate_differences.png",
        )
        plot_pivot_center_distance_heatmap(
            pivot_ids,
            group_pivot_results,
            centers_dir / "sphere_center_distance_heatmap.png",
        )
        plot_pivot_fit_quality(
            pivot_ids,
            group_pivot_results,
            centers_dir / "sphere_fit_quality.png",
        )
    return comparison_count


def generate_hexapod_calibration_comparison_report(
    measurement_file_path: str | Path,
    result_file_path: Optional[str | Path] = None,
    output_dir: Optional[str | Path] = None,
    station_log_path: Optional[str | Path] = None,
) -> dict[str, object]:
    """Generate comparison CSVs and plots for compatible calibration runs.

    Parameters
    ----------
    measurement_file_path:
        Raw ``calibrate_smarpod_measurement_*.json`` file for the current run.
    result_file_path:
        Processed assessment result JSON for the current run.  If supplied, it
        is included even before filesystem discovery sees it.
    output_dir:
        Optional report root.  Defaults to
        ``<measurement_dir>/calibrate_smarpod_comparison_results``.
    station_log_path:
        Optional explicit path to ``Smarpod_Station.json``.  If omitted, the
        report looks next to the measurement directory.

    Returns
    -------
    dict
        Counts and paths useful for logging.
    """
    measurement_path = Path(measurement_file_path).resolve()
    if not measurement_path.is_file():
        raise FileNotFoundError(f"Measurement file does not exist: {measurement_path}")

    result_path = Path(result_file_path).resolve() if result_file_path else None
    if result_path is not None and not result_path.is_file():
        raise FileNotFoundError(f"Result file does not exist: {result_path}")

    station_path = (
        Path(station_log_path).resolve()
        if station_log_path is not None
        else discover_smarpod_station_log(measurement_path)
    )
    if station_path is not None and not station_path.is_file():
        raise FileNotFoundError(f"Smarpod station log does not exist: {station_path}")

    report_root = (
        Path(output_dir).resolve()
        if output_dir is not None
        else measurement_path.parent / "calibrate_smarpod_comparison_results"
    )
    report_root.mkdir(parents=True, exist_ok=True)

    raw_output_dir = report_root / "raw_ball_measurements"
    result_output_dir = report_root / "assessed_calibration_results"
    raw_output_dir.mkdir(parents=True, exist_ok=True)
    result_output_dir.mkdir(parents=True, exist_ok=True)

    measurement_files = discover_measurement_files(measurement_path)
    result_files = discover_result_files(measurement_path, result_path)

    raw_comparisons = _write_raw_measurement_reports(measurement_files, raw_output_dir)
    result_comparisons = 0
    if result_files:
        result_comparisons = _write_pivot_result_reports(
            result_files,
            result_output_dir,
            station_log_path=station_path,
        )
    else:
        write_csv([], result_output_dir / "id_set_manifest.csv")

    summary = {
        "measurement_file": str(measurement_path),
        "result_file": str(result_path) if result_path else "",
        "report_dir": str(report_root),
        "station_log_file": str(station_path) if station_path else "",
        "raw_measurement_file_count": len(measurement_files),
        "assessment_result_file_count": len(result_files),
        "raw_pairwise_comparison_count": raw_comparisons,
        "result_pairwise_comparison_count": result_comparisons,
    }
    (report_root / "summary.json").write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate Smarpod/hexapod calibration comparison reports."
    )
    parser.add_argument(
        "measurement_file",
        type=Path,
        help="current calibrate_smarpod_measurement_*.json file",
    )
    parser.add_argument(
        "--result-file",
        type=Path,
        help="current processed results_calibrate_smarpod_measurement_*.json file",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="optional output directory for comparison CSVs and plots",
    )
    parser.add_argument(
        "--station-log",
        type=Path,
        help="optional Smarpod_Station.json calibration-history file",
    )
    args = parser.parse_args()

    summary = generate_hexapod_calibration_comparison_report(
        measurement_file_path=args.measurement_file,
        result_file_path=args.result_file,
        output_dir=args.output_dir,
        station_log_path=args.station_log,
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
