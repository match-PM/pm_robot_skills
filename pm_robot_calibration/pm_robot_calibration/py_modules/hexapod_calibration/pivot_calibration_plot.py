"""
Diagnostic plots for the pivot calibration result.

Each diagnostic is exported as a **separate** PNG so the user can pick
what they want.  The previous multi-panel summary is also still
available via :func:`plot_pivot_calibration_errors`.

All residuals are plotted in **micrometres**.

Plots produced (default filenames shown):

1. ``<stem>_01_3d_centers.png`` -- measured vs predicted sphere centres
   in 3D, coloured by position set.
2. ``<stem>_02_residual_vs_rx.png`` -- per-axis residual vs commanded rx.
3. ``<stem>_03_residual_vs_x.png`` -- per-axis residual vs commanded x
   (with a y_cmd background band so the position sets separate).
4. ``<stem>_04_residual_norms.png`` -- residual-norm bar chart, coloured
   by position set.
5. ``<stem>_05_sphere_fit_error_<pos_id>.png`` -- for every position set
   (x_cmd, y_cmd constant), the radial fit residual of every measured
   point to the fitted sphere, grouped by sphere id (so the 9 balls
   share one plot per position).
"""
from __future__ import annotations

import os
from typing import Optional, Sequence

import numpy as np
from numpy.typing import NDArray

import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt


# ----------------------------------------------------------------- helpers
def _extract_set_command(
    sphere_sets: Sequence,
    sphere_set_ids: Sequence[str],
) -> dict[str, dict[str, float]]:
    """Return a mapping ``sphere_set_id -> {x_cmd, y_cmd, rx_cmd, ...}``."""
    out: dict[str, dict[str, float]] = {}
    for sphere_set in sphere_sets:
        out[sphere_set.sphere_set_id] = {
            "x_cmd": float(sphere_set.x_cmd),
            "y_cmd": float(sphere_set.y_cmd),
            "rx_cmd": float(sphere_set.rx_cmd),
            "ry_cmd": float(sphere_set.ry_cmd),
            "rz_cmd": float(sphere_set.rz_cmd),
        }
    return out


def _position_set_id(x: float, y: float) -> str:
    """Format the (x_cmd, y_cmd) pair as ``x{n}_y{n}`` like the rest of the codebase."""
    def f(value: float) -> str:
        if float(value).is_integer():
            return str(int(value))
        return str(value)
    return f"x{f(x)}_y{f(y)}"


def _make_position_colours(position_ids: list[str]) -> tuple[dict, list]:
    """Build a stable colour map for a list of position ids."""
    unique = sorted(set(position_ids))
    cmap = plt.get_cmap("tab10", max(len(unique), 1))
    return ({pid: cmap(i) for i, pid in enumerate(unique)}, unique)


def _format_short_label(set_id: str) -> str:
    """Make a set id like ``x0_y0_rx1.5_ry0.0`` readable on a tight x-axis."""
    return (
        set_id
        .replace("rx", "r")
        .replace("ry", "p")
        .replace("_x", " x")
        .replace("_y", " y")
    )


def _build_path(base_path: str, suffix: str, new_ext: Optional[str] = None) -> str:
    """Return ``<stem><suffix><.ext>`` next to ``base_path``.

    If ``new_ext`` is given it overrides the extension (e.g. ``".png"``);
    otherwise the extension of ``base_path`` is preserved.
    """
    stem, ext = os.path.splitext(base_path)
    if new_ext is not None:
        ext = new_ext
    return f"{stem}{suffix}{ext}"


# ----------------------------------------------------- individual plot helpers
def _plot_3d_centers(
    measured: NDArray[np.float64],
    predicted: NDArray[np.float64],
    colours: list,
    position_ids: list[str],
    unique_position_ids: list[str],
    colour_by_id: dict,
    title_prefix: str,
) -> plt.Figure:
    """3D scatter of measured vs predicted sphere centres."""
    fig = plt.figure(figsize=(9, 7), constrained_layout=True)
    ax = fig.add_subplot(1, 1, 1, projection="3d")
    for pid in unique_position_ids:
        mask = np.array([pid == p for p in position_ids])
        ax.scatter(
            measured[mask, 0], measured[mask, 1], measured[mask, 2],
            color=colour_by_id[pid], marker="o", s=30,
            label=f"measured {pid}",
        )
        ax.scatter(
            predicted[mask, 0], predicted[mask, 1], predicted[mask, 2],
            color=colour_by_id[pid], marker="x", s=40,
        )
    for n in range(measured.shape[0]):
        ax.plot(
            [measured[n, 0], predicted[n, 0]],
            [measured[n, 1], predicted[n, 1]],
            [measured[n, 2], predicted[n, 2]],
            color=colours[n], alpha=0.4, linewidth=0.8,
        )
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title(f"{title_prefix}\nmeasured (●) vs predicted (×) sphere centres\n"
                 "lines: per-set residual")
    ax.legend(loc="upper left", fontsize=8, framealpha=0.8)
    return fig


def _plot_residual_vs_rx(
    rx_cmd: NDArray[np.float64],
    residuals_um: NDArray[np.float64],
    title_prefix: str,
) -> plt.Figure:
    """Per-axis residual vs commanded rx."""
    fig, ax = plt.subplots(figsize=(9, 6), constrained_layout=True)
    for axis_index, axis_label in enumerate(["X", "Y", "Z"]):
        ax.scatter(
            rx_cmd, residuals_um[:, axis_index],
            color="C0" if axis_index == 0 else ("C1" if axis_index == 1 else "C2"),
            marker="o", s=24, alpha=0.7, label=f"residual {axis_label}",
        )
    ax.axhline(0.0, color="k", linewidth=0.6, linestyle="--")
    ax.set_xlabel("rx_cmd (deg)")
    ax.set_ylabel("residual (um)")
    ax.set_title(f"{title_prefix}\nResidual per axis vs commanded rx")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)
    return fig


def _plot_residual_vs_x(
    x_cmd: NDArray[np.float64],
    y_cmd: NDArray[np.float64],
    ry_cmd: NDArray[np.float64],
    residuals_um: NDArray[np.float64],
    unique_position_ids: list[str],
    colour_by_id: dict,
    position_ids: list[str],
    title_prefix: str,
) -> plt.Figure:
    """Per-axis residual vs commanded x (with position-set background bands)."""
    fig, ax = plt.subplots(figsize=(9, 6), constrained_layout=True)
    for axis_index, axis_label in enumerate(["X", "Y", "Z"]):
        ax.scatter(
            x_cmd + 0.1 * (ry_cmd - ry_cmd.mean()),
            residuals_um[:, axis_index],
            color="C0" if axis_index == 0 else ("C1" if axis_index == 1 else "C2"),
            marker="o", s=24, alpha=0.7, label=f"residual {axis_label}",
        )
    ax.axhline(0.0, color="k", linewidth=0.6, linestyle="--")
    ax.set_xlabel("x_cmd (mm)  (jittered by ry_cmd / 10)")
    ax.set_ylabel("residual (um)")
    ax.set_title(f"{title_prefix}\nResidual per axis vs commanded x (y_cmd shown by colour)")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)
    for pid in unique_position_ids:
        mask = np.array([pid == p for p in position_ids])
        y_vals = y_cmd[mask]
        if y_vals.size == 0:
            continue
        mean_y = float(np.mean(y_vals))
        ax.axhspan(
            mean_y - 1.0, mean_y + 1.0,
            xmin=0.0, xmax=1.0,
            facecolor=colour_by_id[pid], alpha=0.05,
        )
    return fig


def _plot_residual_norms(
    set_ids: list[str],
    cmds: dict,
    residual_norms_um: NDArray[np.float64],
    colours: list,
    unique_position_ids: list[str],
    colour_by_id: dict,
    title_prefix: str,
) -> plt.Figure:
    """Residual-norm bar chart, sorted by position set, then rx, ry, rz."""
    fig, ax = plt.subplots(figsize=(13, 6), constrained_layout=True)
    composite = sorted(
        range(len(set_ids)),
        key=lambda n: (cmds[set_ids[n]]["x_cmd"],
                       cmds[set_ids[n]]["y_cmd"],
                       cmds[set_ids[n]]["rx_cmd"],
                       cmds[set_ids[n]]["ry_cmd"],
                       cmds[set_ids[n]]["rz_cmd"]),
    )
    sorted_residuals = residual_norms_um[composite]
    sorted_colours = [colours[i] for i in composite]
    sorted_labels = [set_ids[i] for i in composite]
    ax.bar(range(len(sorted_residuals)), sorted_residuals, color=sorted_colours)
    ax.set_xticks(range(len(sorted_labels)))
    ax.set_xticklabels(
        [_format_short_label(lbl) for lbl in sorted_labels],
        rotation=90, fontsize=6,
    )
    ax.set_ylabel("|residual| (um)")
    ax.set_title(f"{title_prefix}\nPer-set residual norm "
                 "(sorted by position set, then rx, ry, rz)")
    ax.grid(True, axis="y", alpha=0.3)
    handles = [
        plt.Rectangle((0, 0), 1, 1, color=colour_by_id[pid])
        for pid in unique_position_ids
    ]
    ax.legend(handles, unique_position_ids, fontsize=8, loc="upper right",
              title="position set", title_fontsize=8)
    return fig


def _plot_sphere_fit_errors_for_position(
    position_id: str,
    sphere_sets_in_position: list,
    title_prefix: str,
    y_limits_um: Optional[tuple[float, float]] = None,
    pad_factor: float = 0.05,
) -> plt.Figure:
    """For one position set, plot the radial fit residual of every
    measurement to the fitted sphere.

    Layout
    ------
    * x-axis: ordered list of sphere ids (e.g. Ball_1 ... Ball_9).
    * For every sphere, a grouped bar chart of the radial residuals
      ``||measurement - centre|| - R`` (in micrometres) is drawn, one
      bar per commanded configuration in this position set.
    * The bar grouping is **(rx_cmd, ry_cmd)**.  Configurations that
      share the same (rx, ry) are drawn in the same group with
      slightly different colours and listed in the legend as
      ``rx=X.X ry=Y.Y rz=Z`` so all 16 sets in the position are visible.

    Parameters
    ----------
    position_id : str
        Identifier like ``"x0_y0"``.
    sphere_sets_in_position : list
        All ``SphereSet`` records belonging to this position.
    title_prefix : str
        Title prefix for the figure.
    y_limits_um : tuple[float, float], optional
        If given, force the y-axis to ``(ymin, ymax)`` (micrometres).
        Use a value computed across **all** position sets so every
        figure is directly comparable.
    pad_factor : float
        Fractional padding added around the residual range when
        ``y_limits_um`` is ``None``.
    """
    # Build (sphere_id, rx, ry, rz, set_id, residual_um) tuples.
    sorted_sets = sorted(
        sphere_sets_in_position,
        key=lambda s: (s.rx_cmd, s.ry_cmd, s.rz_cmd),
    )
    rows: list[tuple[str, float, float, float, str, float]] = []
    config_labels: list[str] = []
    for sphere_set in sorted_sets:
        config_labels.append(
            f"rx{sphere_set.rx_cmd:+.1f} "
            f"ry{sphere_set.ry_cmd:+.1f} "
            f"rz{sphere_set.rz_cmd:+.0f}"
        )
        for measurement in sphere_set.sphere_measurements.get_all_sphere_measurements():
            err = measurement.sphere_fit_error_mm
            if err is None:
                continue
            rows.append((
                measurement.sphere_id,
                sphere_set.rx_cmd,
                sphere_set.ry_cmd,
                sphere_set.rz_cmd,
                config_labels[-1],
                float(err) * 1000.0,
            ))

    sphere_ids = sorted({r[0] for r in rows}, key=lambda s: int(s.rsplit("_", 1)[-1]))

    fig, ax = plt.subplots(figsize=(13, 6), constrained_layout=True)

    if not rows:
        ax.text(
            0.5, 0.5,
            f"No sphere-fit residuals available for position set {position_id}",
            ha="center", va="center", transform=ax.transAxes,
        )
        ax.set_axis_off()
        return fig

    n_configs = len(config_labels)
    width = 0.85 / max(n_configs, 1)
    x_base = np.arange(len(sphere_ids))

    # Use a continuous colormap so 16 entries stay distinguishable.
    cmap = plt.get_cmap("tab20", max(n_configs, 1))
    for i, set_label in enumerate(config_labels):
        heights = []
        for sid in sphere_ids:
            val = next(
                (r[5] for r in rows if r[0] == sid and r[4] == set_label),
                np.nan,
            )
            heights.append(val)
        offsets = x_base + (i - (n_configs - 1) / 2.0) * width
        ax.bar(
            offsets, heights, width=width,
            color=cmap(i), label=set_label,
            edgecolor="black", linewidth=0.3,
        )

    ax.axhline(0.0, color="k", linewidth=0.6, linestyle="--")
    ax.set_xticks(x_base)
    # Make sphere ids short: "Ball_3" instead of "CAL_Smarpod_Ball_3"
    ax.set_xticklabels(
        [sid.replace("CAL_Smarpod_", "") for sid in sphere_ids],
        rotation=0,
    )
    ax.set_xlabel("calibration sphere (id)")
    ax.set_ylabel("radial fit residual (um)\n  = ||measurement - centre|| - R")
    ax.set_title(
        f"{title_prefix}\n"
        f"Sphere-fit residuals -- position set {position_id}  "
        f"(one bar per commanded rx/ry/rz)"
    )

    # ------------------------------------------------------------------
    # Y-axis: enforce a shared range across all position sets so the
    # three sphere-fit-error figures are directly comparable.
    # ------------------------------------------------------------------
    if y_limits_um is None:
        finite = np.array([h for h in np.concatenate(heights).ravel()
                           if np.isfinite(h)], dtype=np.float64)
        if finite.size == 0:
            y_limits_um = (-1.0, 1.0)
        else:
            y_max = float(np.max(np.abs(finite)))
            pad = max(y_max * pad_factor, 0.5)
            y_limits_um = (-y_max - pad, y_max + pad)
    ax.set_ylim(*y_limits_um)
    # Two-column legend below the plot, font kept small because we may
    # have 16 entries.
    ncol = 2 if n_configs <= 8 else 4
    ax.legend(
        title="commanded configuration",
        title_fontsize=8, fontsize=7,
        loc="upper center", bbox_to_anchor=(0.5, -0.18),
        ncol=ncol, frameon=False,
    )
    ax.grid(True, axis="y", alpha=0.3)
    return fig


# ---------------------------------------------------------- public entry point
def plot_pivot_calibration_errors(
    result,
    sphere_sets: Sequence,
    output_path: str,
    title: Optional[str] = None,
) -> str:
    """Render the combined 2x2 multi-panel diagnostic figure.

    Kept for backwards compatibility / quick inspection.  See
    :func:`plot_pivot_calibration_individual` for the per-figure
    versions.
    """
    measured = np.asarray(result.measured_centers_mm, dtype=np.float64)
    predicted = np.asarray(result.predicted_centers_mm, dtype=np.float64)
    residuals = np.asarray(result.residuals_mm, dtype=np.float64)
    residual_norms = np.linalg.norm(residuals, axis=1)
    set_ids = list(result.sphere_set_ids)
    cmds = _extract_set_command(sphere_sets, set_ids)

    position_ids = [
        _position_set_id(cmds[sid]["x_cmd"], cmds[sid]["y_cmd"])
        for sid in set_ids
    ]
    colour_by_id, unique_position_ids = _make_position_colours(position_ids)
    colours = [colour_by_id[pid] for pid in position_ids]
    rx_cmd = np.array([cmds[sid]["rx_cmd"] for sid in set_ids])
    ry_cmd = np.array([cmds[sid]["ry_cmd"] for sid in set_ids])
    x_cmd = np.array([cmds[sid]["x_cmd"] for sid in set_ids])
    y_cmd = np.array([cmds[sid]["y_cmd"] for sid in set_ids])
    residuals_um = residuals * 1000.0

    fig = plt.figure(figsize=(15, 11), constrained_layout=True)
    fig.suptitle(
        title or "Pivot calibration: measured vs predicted sphere centres",
        fontsize=14,
    )

    # Panel 1: 3D scatter of measured vs predicted, connected by lines.
    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    for pid in unique_position_ids:
        mask = np.array([pid == p for p in position_ids])
        ax3d.scatter(
            measured[mask, 0], measured[mask, 1], measured[mask, 2],
            color=colour_by_id[pid], marker="o", s=30,
            label=f"measured {pid}",
        )
        ax3d.scatter(
            predicted[mask, 0], predicted[mask, 1], predicted[mask, 2],
            color=colour_by_id[pid], marker="x", s=40,
        )
    for n in range(measured.shape[0]):
        ax3d.plot(
            [measured[n, 0], predicted[n, 0]],
            [measured[n, 1], predicted[n, 1]],
            [measured[n, 2], predicted[n, 2]],
            color=colours[n], alpha=0.4, linewidth=0.8,
        )
    ax3d.set_xlabel("X (mm)")
    ax3d.set_ylabel("Y (mm)")
    ax3d.set_zlabel("Z (mm)")
    ax3d.set_title("Measured (●) vs predicted (×) sphere centres\n"
                   "lines: per-set residual")
    ax3d.legend(loc="upper left", fontsize=8, framealpha=0.8)

    # Panel 2: residual per axis vs commanded rx.
    ax2 = fig.add_subplot(2, 2, 2)
    for axis_index, axis_label in enumerate(["X", "Y", "Z"]):
        ax2.scatter(
            rx_cmd, residuals[:, axis_index] * 1000.0,
            color="C0" if axis_index == 0 else ("C1" if axis_index == 1 else "C2"),
            marker="o", s=24, alpha=0.7, label=f"residual {axis_label}",
        )
    ax2.axhline(0.0, color="k", linewidth=0.6, linestyle="--")
    ax2.set_xlabel("rx_cmd (deg)")
    ax2.set_ylabel("residual (um)")
    ax2.set_title("Residual per axis vs commanded rx")
    ax2.legend(fontsize=8, loc="upper right")
    ax2.grid(True, alpha=0.3)

    # Panel 3: residual per axis vs commanded (x, y).
    ax3 = fig.add_subplot(2, 2, 3)
    for axis_index, axis_label in enumerate(["X", "Y", "Z"]):
        ax3.scatter(
            x_cmd + 0.1 * (ry_cmd - ry_cmd.mean()),
            residuals[:, axis_index] * 1000.0,
            color="C0" if axis_index == 0 else ("C1" if axis_index == 1 else "C2"),
            marker="o", s=24, alpha=0.7, label=f"residual {axis_label}",
        )
    ax3.axhline(0.0, color="k", linewidth=0.6, linestyle="--")
    ax3.set_xlabel("x_cmd (mm)  (jittered by ry_cmd / 10)")
    ax3.set_ylabel("residual (um)")
    ax3.set_title("Residual per axis vs commanded x (y_cmd shown by colour)")
    ax3.legend(fontsize=8, loc="upper right")
    ax3.grid(True, alpha=0.3)

    for pid in unique_position_ids:
        mask = np.array([pid == p for p in position_ids])
        y_vals = y_cmd[mask]
        if y_vals.size == 0:
            continue
        mean_y = float(np.mean(y_vals))
        ax3.axhspan(
            mean_y - 1.0, mean_y + 1.0,
            xmin=0.0, xmax=1.0,
            facecolor=colour_by_id[pid], alpha=0.05,
        )

    # Panel 4: residual-norm bar chart, coloured by position set.
    ax4 = fig.add_subplot(2, 2, 4)
    composite = sorted(
        range(len(set_ids)),
        key=lambda n: (position_ids[n], cmds[set_ids[n]]["rx_cmd"],
                       cmds[set_ids[n]]["ry_cmd"],
                       cmds[set_ids[n]]["rz_cmd"]),
    )
    sorted_residuals = residual_norms[composite] * 1000.0
    sorted_colours = [colours[i] for i in composite]
    sorted_labels = [set_ids[i] for i in composite]
    ax4.bar(range(len(sorted_residuals)), sorted_residuals, color=sorted_colours)
    ax4.set_xticks(range(len(sorted_labels)))
    ax4.set_xticklabels(
        [_format_short_label(lbl) for lbl in sorted_labels],
        rotation=90, fontsize=6,
    )
    ax4.set_ylabel("|residual| (um)")
    ax4.set_title("Per-set residual norm (sorted by position set, then rx, ry, rz)")
    ax4.grid(True, axis="y", alpha=0.3)
    handles = [
        plt.Rectangle((0, 0), 1, 1, color=colour_by_id[pid])
        for pid in unique_position_ids
    ]
    ax4.legend(handles, unique_position_ids, fontsize=8, loc="upper right",
               title="position set", title_fontsize=8)

    os.makedirs(os.path.dirname(os.path.abspath(output_path)) or ".", exist_ok=True)
    fig.savefig(output_path, dpi=140)
    plt.close(fig)
    return output_path


def plot_pivot_calibration_individual(
    result,
    sphere_sets: Sequence,
    base_path: str,
    title: Optional[str] = None,
) -> list[str]:
    """Render every diagnostic as a **separate** PNG and return the list of paths.

    Parameters
    ----------
    result : PivotCalibrationResult
        The solved calibration result.
    sphere_sets : Sequence[SphereSet]
        The original sphere sets, used for colour / labelling and to
        build the per-position sphere-fit error figures.
    base_path : str
        The base output path (e.g. ``results/<stem>.png``).  Each
        individual figure is written next to this file with a numeric
        suffix on the stem, e.g. ``results/<stem>_01_3d_centers.png``.
    title : str, optional
        Optional title prefix used for every individual figure.

    Returns
    -------
    list[str]
        The output paths in the order they were written.
    """
    measured = np.asarray(result.measured_centers_mm, dtype=np.float64)
    predicted = np.asarray(result.predicted_centers_mm, dtype=np.float64)
    residuals = np.asarray(result.residuals_mm, dtype=np.float64)
    residual_norms = np.linalg.norm(residuals, axis=1)
    set_ids = list(result.sphere_set_ids)
    cmds = _extract_set_command(sphere_sets, set_ids)

    position_ids = [
        _position_set_id(cmds[sid]["x_cmd"], cmds[sid]["y_cmd"])
        for sid in set_ids
    ]
    colour_by_id, unique_position_ids = _make_position_colours(position_ids)
    colours = [colour_by_id[pid] for pid in position_ids]

    rx_cmd = np.array([cmds[sid]["rx_cmd"] for sid in set_ids])
    ry_cmd = np.array([cmds[sid]["ry_cmd"] for sid in set_ids])
    x_cmd = np.array([cmds[sid]["x_cmd"] for sid in set_ids])
    y_cmd = np.array([cmds[sid]["y_cmd"] for sid in set_ids])
    residuals_um = residuals * 1000.0
    residual_norms_um = residual_norms * 1000.0

    title_prefix = title or "Pivot calibration"

    # Group sphere sets by position id so we can render one
    # sphere-fit-error figure per position set.
    sets_by_position: dict[str, list] = {pid: [] for pid in unique_position_ids}
    for sphere_set in sphere_sets:
        pid = _position_set_id(sphere_set.x_cmd, sphere_set.y_cmd)
        sets_by_position.setdefault(pid, []).append(sphere_set)

    # ------------------------------------------------------------------
    # Compute the SHARED y-axis range for all sphere-fit-error figures
    # so the per-position plots are directly comparable.  We take the
    # worst-case absolute residual across **every** position set and
    # add a small symmetric padding.
    # ------------------------------------------------------------------
    shared_y_limits_um: Optional[tuple[float, float]] = None
    all_sphere_fit_residuals: list[float] = []
    for sets_in_position in sets_by_position.values():
        for sphere_set in sets_in_position:
            for measurement in sphere_set.sphere_measurements.get_all_sphere_measurements():
                err = measurement.sphere_fit_error_mm
                if err is None:
                    continue
                all_sphere_fit_residuals.append(float(err) * 1000.0)
    if all_sphere_fit_residuals:
        y_max = float(np.max(np.abs(all_sphere_fit_residuals)))
        pad = max(y_max * 0.05, 0.5)  # at least 0.5 µm padding
        shared_y_limits_um = (-y_max - pad, y_max + pad)

    parent = os.path.dirname(os.path.abspath(base_path))
    if parent:
        os.makedirs(parent, exist_ok=True)

    out_paths: list[str] = []

    # 1. 3D measured vs predicted centres.
    fig = _plot_3d_centers(
        measured, predicted, colours, position_ids,
        unique_position_ids, colour_by_id, title_prefix,
    )
    p = _build_path(base_path, "_01_3d_centers")
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out_paths.append(p)

    # 2. Residual per axis vs rx.
    fig = _plot_residual_vs_rx(rx_cmd, residuals_um, title_prefix)
    p = _build_path(base_path, "_02_residual_vs_rx")
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out_paths.append(p)

    # 3. Residual per axis vs x (with position-set background bands).
    fig = _plot_residual_vs_x(
        x_cmd, y_cmd, ry_cmd, residuals_um,
        unique_position_ids, colour_by_id, position_ids, title_prefix,
    )
    p = _build_path(base_path, "_03_residual_vs_x")
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out_paths.append(p)

    # 4. Residual-norm bar chart.
    fig = _plot_residual_norms(
        set_ids, cmds, residual_norms_um, colours,
        unique_position_ids, colour_by_id, title_prefix,
    )
    p = _build_path(base_path, "_04_residual_norms")
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out_paths.append(p)

    # 5. Sphere-fit-error figures -- one per position set, all sharing
    # the same y-axis so the figures can be compared side-by-side.
    for pid in sorted(sets_by_position):
        fig = _plot_sphere_fit_errors_for_position(
            pid, sets_by_position[pid], title_prefix,
            y_limits_um=shared_y_limits_um,
        )
        p = _build_path(base_path, f"_05_sphere_fit_error_{pid}")
        fig.savefig(p, dpi=140)
        plt.close(fig)
        out_paths.append(p)

    return out_paths


def plot_smarpod_calibration_test_measurements(
    measurement_data: Sequence[dict],
    output_path: str,
    title: Optional[str] = None,
) -> str:
    """Plot scalar test measurements from ``test_hexapod_calibration``.

    The test action stores one measurement per commanded pose.  This plot
    keeps the raw order visible and also groups measurements by commanded
    x/y position so a drift or position-dependent offset is easy to spot.
    """
    if not measurement_data:
        raise ValueError("No test measurements available for plotting.")

    ordered = sorted(
        measurement_data,
        key=lambda entry: int(entry.get("current_iteration", 0)),
    )

    iterations = np.asarray(
        [float(entry.get("current_iteration", index + 1)) for index, entry in enumerate(ordered)],
        dtype=np.float64,
    )
    measurements_um = np.asarray(
        [float(entry["measurement_um"]) for entry in ordered],
        dtype=np.float64,
    )
    x_cmd_um = np.asarray([float(entry.get("x_cmd_um", 0.0)) for entry in ordered], dtype=np.float64)
    y_cmd_um = np.asarray([float(entry.get("y_cmd_um", 0.0)) for entry in ordered], dtype=np.float64)
    rx_cmd = np.asarray([float(entry.get("rx_cmd", 0.0)) for entry in ordered], dtype=np.float64)
    ry_cmd = np.asarray([float(entry.get("ry_cmd", 0.0)) for entry in ordered], dtype=np.float64)
    rz_cmd = np.asarray([float(entry.get("rz_cmd", 0.0)) for entry in ordered], dtype=np.float64)
    pose_ids = [str(entry.get("pose_id", f"pose_{int(iteration)}")) for entry, iteration in zip(ordered, iterations)]

    baseline_um = float(measurements_um[0])
    delta_um = measurements_um - baseline_um
    position_ids = [
        _position_set_id(x / 1000.0, y / 1000.0)
        for x, y in zip(x_cmd_um, y_cmd_um)
    ]
    colour_by_id, unique_position_ids = _make_position_colours(position_ids)
    colours = [colour_by_id[pid] for pid in position_ids]

    fig, axes = plt.subplots(3, 1, figsize=(13, 11), constrained_layout=True)
    fig.suptitle(title or "Smarpod calibration test measurements", fontsize=14)

    axes[0].plot(iterations, measurements_um, color="0.35", linewidth=1.0, alpha=0.7)
    axes[0].scatter(iterations, measurements_um, c=colours, s=34, zorder=3)
    axes[0].axhline(baseline_um, color="k", linewidth=0.7, linestyle="--", alpha=0.6)
    axes[0].set_ylabel("measurement (um)")
    axes[0].set_title("Measured sensor value per test pose")
    axes[0].grid(True, alpha=0.3)

    axes[1].bar(iterations, delta_um, color=colours)
    axes[1].axhline(0.0, color="k", linewidth=0.7, linestyle="--")
    axes[1].set_ylabel("delta from first pose (um)")
    axes[1].set_title("Measurement change relative to first pose")
    axes[1].grid(True, axis="y", alpha=0.3)

    for pid in unique_position_ids:
        mask = np.array([pid == position_id for position_id in position_ids])
        axes[2].scatter(
            rx_cmd[mask],
            delta_um[mask],
            color=colour_by_id[pid],
            s=34,
            label=pid,
        )
    axes[2].axhline(0.0, color="k", linewidth=0.7, linestyle="--")
    axes[2].set_xlabel("rx_cmd (deg)")
    axes[2].set_ylabel("delta from first pose (um)")
    axes[2].set_title("Measurement change vs commanded rx")
    axes[2].grid(True, alpha=0.3)

    handles = [
        plt.Rectangle((0, 0), 1, 1, color=colour_by_id[pid])
        for pid in unique_position_ids
    ]
    axes[0].legend(
        handles,
        unique_position_ids,
        fontsize=8,
        loc="best",
        title="position set",
        title_fontsize=8,
    )
    axes[2].legend(fontsize=8, loc="best", title="position set", title_fontsize=8)

    tick_step = max(1, int(np.ceil(len(pose_ids) / 18)))
    tick_indices = list(range(0, len(pose_ids), tick_step))
    for ax in axes[:2]:
        ax.set_xticks(iterations[tick_indices])
        ax.set_xticklabels(
            [_format_short_label(pose_ids[index]) for index in tick_indices],
            rotation=70,
            ha="right",
            fontsize=7,
        )

    summary = (
        f"n={len(measurements_um)}  "
        f"mean={np.mean(measurements_um):.2f} um  "
        f"std={np.std(measurements_um):.2f} um  "
        f"range={np.ptp(measurements_um):.2f} um"
    )
    axes[0].text(
        0.01,
        0.98,
        summary,
        transform=axes[0].transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "0.8"},
    )

    command_summary = (
        f"ry range: {np.min(ry_cmd):.2f}..{np.max(ry_cmd):.2f} deg, "
        f"rz range: {np.min(rz_cmd):.2f}..{np.max(rz_cmd):.2f} deg"
    )
    axes[2].text(
        0.01,
        0.98,
        command_summary,
        transform=axes[2].transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "0.8"},
    )

    os.makedirs(os.path.dirname(os.path.abspath(output_path)) or ".", exist_ok=True)
    fig.savefig(output_path, dpi=140)
    plt.close(fig)
    return output_path
