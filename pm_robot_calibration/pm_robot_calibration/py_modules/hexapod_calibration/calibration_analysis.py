"""
CalibrationAnalysis: high-level orchestrator for a Smarpod calibration.

A :class:`CalibrationAnalysis` is created from a
:class:`sphere_calibration.SphereCalibration` and bundles together:

* the **fitted pivots** (one per sphere set) from
  :meth:`fit_pivot_per_set`
* the **pivot calibration result** (B__T__P and J__t__P) from
  :meth:`solve_pivot_calibration`

All errors and residuals are reported in **micrometres (µm)** in the
human-readable outputs and JSON, because the relevant numbers are
single- to double-digit microns and that's the natural unit for
machine-tool calibration.

Typical usage ::

    sc = SphereCalibration.load_file("measurements/calibration_xxx.json")
    analysis = sc.run_calibration()
    analysis.print_results()             # to stdout
    analysis.save_results()              # writes to results/<stem>.json
    analysis.plot_results()              # writes to results/<stem>.png
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from typing import Optional

import numpy as np
from geometry_msgs.msg import Transform
from scipy.spatial.transform import Rotation as SciPyRotation

from pm_robot_calibration.py_modules.hexapod_calibration.data_classes import(
    CALIBRATION_SPHERE_DIAMETER_MM,
    CALIBRATION_SPHERE_RADIUS_MM,
)

from pm_robot_calibration.py_modules.hexapod_calibration.geometry_utils import rotation_matrix_to_euler_zyx

from pm_robot_calibration.py_modules.hexapod_calibration.sphere_calibration import (
    PivotCalibrationResult,
    SphereCalibration,
    SphereFitResult,
)


# --------------------------------------------------------- result container
@dataclass
class CalibrationAnalysis:
    """Bundle of all results computed from a single calibration file."""

    sphere_calibration: SphereCalibration
    pivots: dict[str, SphereFitResult]
    pivot_calibration: PivotCalibrationResult
    diameter_mm: float = CALIBRATION_SPHERE_DIAMETER_MM
    radius_mm: float = CALIBRATION_SPHERE_RADIUS_MM

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------
    @classmethod
    def run(
        cls,
        sphere_calibration: SphereCalibration,
        diameter_mm: Optional[float] = None,
        radius_mm: Optional[float] = None,
    ) -> "CalibrationAnalysis":
        """Run the full analysis: pivot fit per set + pivot calibration.

        Pass ``diameter_mm`` (e.g. 6.35) or ``radius_mm`` (e.g. 3.175)
        to override the module-level default.  ``diameter_mm`` takes
        precedence if both are given; pass ``None`` for both to use
        :data:`data_classes.CALIBRATION_SPHERE_DIAMETER_MM`.
        """
        if diameter_mm is None and radius_mm is None:
            diameter_mm = CALIBRATION_SPHERE_DIAMETER_MM
        if diameter_mm is None:
            diameter_mm = 2.0 * float(radius_mm)
        radius_mm = 0.5 * float(diameter_mm)

        pivots = sphere_calibration.fit_pivot_per_set(radius_mm=radius_mm)
        pivot_calibration = sphere_calibration.solve_pivot_calibration(
            pivots=pivots,
        )
        return cls(
            sphere_calibration=sphere_calibration,
            pivots=pivots,
            pivot_calibration=pivot_calibration,
            diameter_mm=float(diameter_mm),
            radius_mm=float(radius_mm),
        )

    # ------------------------------------------------------------------
    # Convenience views
    # ------------------------------------------------------------------
    @property
    def n_sets(self) -> int:
        return len(self.pivots)

    @property
    def n_position_sets(self) -> int:
        return len(self.pivot_calibration.position_set_residuals)

    @property
    def sphere_ids(self) -> list[str]:
        return self.sphere_calibration.get_all_sphere_ids()

    @property
    def reference_frame(self) -> Optional[str]:
        return self.sphere_calibration.calibration_reference_frame

    @property
    def fixed_reference_frame(self) -> Optional[str]:
        return self.sphere_calibration.calibration_fixed_reference_frame

    @property
    def timestamp(self) -> Optional[str]:
        return self.sphere_calibration.timestamp

    @property
    def filename(self) -> Optional[str]:
        return self.sphere_calibration.filename

    @property
    def stem(self) -> str:
        """Default file stem (the calibration file basename, or 'calibration')."""
        return self.sphere_calibration.filename or "calibration"

    @property
    def default_json_path(self) -> str:
        return os.path.join("results", f"results_{self.stem}.json")

    @property
    def default_plot_path(self) -> str:
        return os.path.join("results", f"{self.stem}.png")

    def _resolve_output_path(self, file_path: Optional[str], default_path: str) -> str:
        """Return a writable file path, accepting either a file or directory."""
        if file_path is None:
            return default_path

        if os.path.isdir(file_path) or file_path.endswith(os.sep):
            return os.path.join(file_path, os.path.basename(default_path))

        return file_path

    # ------------------------------------------------------------------
    # Convenience accessors for the **main results**
    # ------------------------------------------------------------------
    #
    # These helpers are what you reach for when you want to feed the
    # solved calibration into another piece of code:
    #
    # * ``get_B_T_P_translation(unit=...)`` -- 3-vector
    #   translation of :math:`B \\to P`.
    # * ``get_J_t_P_translation(unit=...)`` -- 3-vector
    #   translation of :math:`J \\to P` (sphere offset from the pivot
    #   point to the sphere centre).
    # * ``get_B_T_P_euler(unit=...)`` -- ZYX intrinsic Euler
    #   ``(yaw, pitch, roll)`` of :math:`B \\to P`.
    # * ``get_B_T_P_rotation_matrix()`` -- 3x3 rotation of
    #   :math:`B \\to P` (no unit).
    # * ``get_B_T_P_homogeneous(unit=...)`` -- the full 4x4 matrix
    #   with translation in the requested unit.
    # * ``get_B_T_P_ros_transform()`` -- ROS2 ``geometry_msgs/Transform``
    #   with translation in metres.
    #
    # Supported length units: ``"mm"`` (default), ``"um"``,
    # ``"micron"``, ``"microns"``, ``"m"``, ``"cm"``.
    # Supported angle units: ``"deg"`` (default) and ``"rad"``.
    #
    # Examples
    # --------
    # >>> analysis.get_B_T_P_translation()                    # mm, numpy array
    # >>> analysis.get_B_T_P_translation("um")                # micrometres
    # >>> analysis.get_J_t_P_translation("m")                  # metres
    # >>> analysis.get_B_T_P_euler("rad")                     # radians
    # >>> analysis.get_B_T_P_translation("mm").tolist()       # as plain list
    _LENGTH_UNIT_FACTORS_MM = {
        "mm": 1.0,
        "um": 1_000.0,
        "micron": 1_000.0,
        "microns": 1_000.0,
        "cm": 0.1,
        "m": 1e-3,
    }
    _ANGLE_UNIT_FACTORS_RAD = {
        "rad": 1.0,
        "deg": 180.0 / math.pi,
    }

    @staticmethod
    def _resolve_length_unit(unit: str) -> tuple[str, float]:
        """Return ``(canonical_name, factor_from_mm)`` or raise."""
        if not isinstance(unit, str):
            raise TypeError(f"unit must be a string, got {type(unit).__name__}")
        key = unit.strip().lower()
        if key not in CalibrationAnalysis._LENGTH_UNIT_FACTORS_MM:
            valid = ", ".join(sorted(CalibrationAnalysis._LENGTH_UNIT_FACTORS_MM))
            raise ValueError(
                f"Unknown length unit {unit!r}; choose one of: {valid}"
            )
        return key, CalibrationAnalysis._LENGTH_UNIT_FACTORS_MM[key]

    @staticmethod
    def _resolve_angle_unit(unit: str) -> tuple[str, float]:
        """Return ``(canonical_name, factor_from_rad)`` or raise."""
        if not isinstance(unit, str):
            raise TypeError(f"unit must be a string, got {type(unit).__name__}")
        key = unit.strip().lower()
        if key not in CalibrationAnalysis._ANGLE_UNIT_FACTORS_RAD:
            valid = ", ".join(sorted(CalibrationAnalysis._ANGLE_UNIT_FACTORS_RAD))
            raise ValueError(
                f"Unknown angle unit {unit!r}; choose one of: {valid}"
            )
        return key, CalibrationAnalysis._ANGLE_UNIT_FACTORS_RAD[key]

    def get_B_T_P_translation(
        self,
        unit: str = "mm",
        as_list: bool = False,
    ) -> NDArray[np.float64]:
        """Return the translation vector of ``B_T_P`` (3 components).

        Parameters
        ----------
        unit : str
            One of ``"mm"`` (default), ``"um"``/``"micron"``/``"microns"``,
            ``"cm"``, ``"m"``.
        as_list : bool
            If ``True``, return a plain Python list instead of a numpy
            array (handy when handing the result to JSON / ROS / etc.).

        Returns
        -------
        numpy.ndarray or list
            ``(x, y, z)`` translation of :math:`B \\to P` in the chosen
            unit.  Internally the result is always stored in mm.
        """
        _, factor = self._resolve_length_unit(unit)
        vec = np.asarray(self.pivot_calibration.B_T_P[:3, 3],
                         dtype=np.float64).reshape(3) * factor
        if as_list:
            return [float(v) for v in vec]
        return vec

    def get_J_t_P_translation(
        self,
        unit: str = "mm",
        as_list: bool = False,
    ) -> NDArray[np.float64]:
        """Return the translation vector of ``J_t_P`` (sphere offset).

        Parameters
        ----------
        unit : str
            Same options as :meth:`get_B_T_P_translation`.
        as_list : bool
            If ``True``, return a plain Python list instead of a numpy
            array.

        Returns
        -------
        numpy.ndarray or list
            ``(x, y, z)`` translation of :math:`J \\to P` (pivot point
            to sphere centre) in the chosen unit.
        """
        _, factor = self._resolve_length_unit(unit)
        vec = np.asarray(self.pivot_calibration.J_t_P,
                         dtype=np.float64).reshape(3) * factor
        if as_list:
            return [float(v) for v in vec]
        return vec

    def get_B_T_P_euler(
        self,
        unit: str = "deg",
        as_list: bool = False,
    ) -> NDArray[np.float64]:
        """Return the ZYX intrinsic Euler angles ``(yaw, pitch, roll)``
        of ``B_T_P``.

        Parameters
        ----------
        unit : str
            ``"deg"`` (default) or ``"rad"``.
        as_list : bool
            If ``True``, return a plain Python list instead of a numpy
            array.

        Returns
        -------
        numpy.ndarray or list
            ``(yaw, pitch, roll)`` of the rotation of
            :math:`B \\to P` in the chosen unit.
        """
        _, factor = self._resolve_angle_unit(unit)
        R = np.asarray(self.pivot_calibration.B_T_P[:3, :3],
                       dtype=np.float64)
        euler_rad = rotation_matrix_to_euler_zyx(R)
        vec = np.asarray(euler_rad, dtype=np.float64).reshape(3) * factor
        if as_list:
            return [float(v) for v in vec]
        return vec

    def get_B_T_P_rotation_matrix(self) -> NDArray[np.float64]:
        """Return the 3x3 rotation matrix of ``B_T_P`` (unitless)."""
        return np.asarray(self.pivot_calibration.B_T_P[:3, :3],
                          dtype=np.float64)

    def get_B_T_P_homogeneous(
        self,
        unit: str = "mm",
    ) -> NDArray[np.float64]:
        """Return the full 4x4 homogeneous transform of ``B_T_P``.

        The translation is converted from the internal mm to ``unit``;
        the rotation block is unitless.
        """
        _, factor = self._resolve_length_unit(unit)
        H = np.asarray(self.pivot_calibration.B_T_P,
                       dtype=np.float64).copy()
        H[:3, 3] *= factor
        return H

    def get_B_T_P_ros_transform(self) -> Transform:
        """Return ``B_T_P`` as a ROS2 ``geometry_msgs.msg.Transform``.

        ROS transforms use metres for translation.  The calibration solver
        stores ``B_T_P`` in millimetres internally, so the translation is
        converted to metres here.
        """
        transform = Transform()

        translation_m = self.get_B_T_P_translation(unit="m")
        transform.translation.x = float(translation_m[0])
        transform.translation.y = float(translation_m[1])
        transform.translation.z = float(translation_m[2])

        rotation_matrix = self.get_B_T_P_rotation_matrix()
        quat = SciPyRotation.from_matrix(rotation_matrix).as_quat()
        transform.rotation.x = float(quat[0])
        transform.rotation.y = float(quat[1])
        transform.rotation.z = float(quat[2])
        transform.rotation.w = float(quat[3])

        return transform

    # ------------------------------------------------------------------
    # print_results
    # ------------------------------------------------------------------
    def print_results(self) -> None:
        """Print a human-readable summary of every result to stdout.

        All errors / residuals / distances are reported in micrometres.
        """
        sc = self.sphere_calibration
        cal_result = self.pivot_calibration

        print()
        print("=" * 72)
        print(f"Calibration analysis: {sc.filename}")
        print("=" * 72)
        print(f"  timestamp:                  {sc.timestamp}")
        print(f"  reference frame:            {sc.calibration_reference_frame}")
        print(f"  fixed reference frame:      {sc.calibration_fixed_reference_frame}")
        position_sets = sorted(cal_result.position_set_residuals.keys())
        print(f"  position sets:              {position_sets}")
        print(f"  sphere ids:                 {self.sphere_ids}")
        print(
            f"  calibration sphere radius:  "
            f"{self.radius_mm:.4f} mm "
            f"(diameter {self.diameter_mm:.4f} mm)"
        )

        # ----- Pivot per set -----
        print()
        print(f"Pivot centres ({self.n_sets} fitted, one per sphere set):")
        for sphere_set_id, result in self.pivots.items():
            sphere_set = next(
                (s for s in sc.sphere_sets if s.sphere_set_id == sphere_set_id),
                None,
            )
            if sphere_set is not None:
                cmd = (
                    f"rx={sphere_set.rx_cmd:+.2f} ry={sphere_set.ry_cmd:+.2f} "
                    f"x={sphere_set.x_cmd:+.2f} y={sphere_set.y_cmd:+.2f}"
                )
            else:
                cmd = "cmd=?"
            print(
                f"  {sphere_set_id:60s}  "
                f"center=({result.center.x:+.4f}, {result.center.y:+.4f}, "
                f"{result.center.z:+.4f}) mm  "
                f"r={result.radius_mm:.4f} mm  "
                f"rms={result.rms_error_mm * 1000:+.3f} um  "
                f"max={result.max_abs_error_mm * 1000:+.3f} um  "
                f"[{cmd}]"
            )

        print()
        print(f"Total pivot centres: {self.n_sets}")

        # ----- Per-set sphere-fit error max -----
        max_sphere_id, max_error = sc.get_max_sphere_fit_error()
        if max_sphere_id is not None:
            print(
                f"Max sphere-fit error across all points: "
                f"{max_error * 1000:.3f} um  (worst ball: {max_sphere_id})"
            )

        # ----- First set detailed view -----
        if sc.sphere_sets:
            first_set = sc.sphere_sets[0]
            first_result = self.pivots[first_set.sphere_set_id]
            print()
            print(f"First set: {first_set.sphere_set_id}")
            print(
                f"  pivot centre: ({first_result.center.x:+.4f}, "
                f"{first_result.center.y:+.4f}, {first_result.center.z:+.4f}) mm"
            )
            print(f"  residuals by ball (radius {first_result.radius_mm:.4f} mm):")
            for sid in sorted(first_result.residuals_by_sphere_id):
                residual_um = first_result.residuals_by_sphere_id[sid] * 1000.0
                measurement = first_set.sphere_measurements.get_sphere_measurement(sid)
                position = measurement.measurement_position
                print(
                    f"    {sid}: pos=({position.x:+.4f}, {position.y:+.4f}, "
                    f"{position.z:+.4f}) mm  err={residual_um:+.3f} um"
                )

        # ----- Pivot calibration -----
        print()
        print("=" * 72)
        print("Pivot calibration: B__T__P and J__t__P")
        print("=" * 72)
        print(
            f"  Converged: {cal_result.converged}  in "
            f"{cal_result.iterations} iterations"
        )
        print(f"  RMS error:   {cal_result.rms_error_mm * 1000:.3f} um")
        print(f"  Max error:   {cal_result.max_abs_error_mm * 1000:.3f} um")
        # Per-axis residual statistics in um.
        res_um = cal_result.residuals_mm * 1000.0
        print(
            f"  Mean error:  "
            f"x={res_um[:, 0].mean():+.3f} um  "
            f"y={res_um[:, 1].mean():+.3f} um  "
            f"z={res_um[:, 2].mean():+.3f} um"
        )
        print(
            f"  Std error:   "
            f"x={res_um[:, 0].std(ddof=0):.3f} um  "
            f"y={res_um[:, 1].std(ddof=0):.3f} um  "
            f"z={res_um[:, 2].std(ddof=0):.3f} um"
        )
        print()
        print("  B__T__P (base -> pivot origin):")
        print(
            f"    translation (mm): "
            f"({cal_result.B_T_P[0, 3]:+.4f}, "
            f"{cal_result.B_T_P[1, 3]:+.4f}, "
            f"{cal_result.B_T_P[2, 3]:+.4f})"
        )
        print("    rotation matrix:")
        for i in range(3):
            print(
                "      ["
                f"{cal_result.B_T_P[i, 0]:+.6f}  "
                f"{cal_result.B_T_P[i, 1]:+.6f}  "
                f"{cal_result.B_T_P[i, 2]:+.6f}]"
            )
        euler_rad = rotation_matrix_to_euler_zyx(cal_result.B_T_P[:3, :3])
        euler_deg = tuple(math.degrees(float(v)) for v in euler_rad)
        print("    ZYX intrinsic Euler (yaw, pitch, roll):")
        print(
            f"      radians: ({euler_rad[0]:+.6f}, {euler_rad[1]:+.6f}, "
            f"{euler_rad[2]:+.6f})"
        )
        print(
            f"      degrees: ({euler_deg[0]:+.4f}, {euler_deg[1]:+.4f}, "
            f"{euler_deg[2]:+.4f})"
        )
        print()
        print("  J__t__P (pivot point -> sphere centre, mm):")
        print(
            f"    ({cal_result.J_t_P[0]:+.4f}, "
            f"{cal_result.J_t_P[1]:+.4f}, "
            f"{cal_result.J_t_P[2]:+.4f})"
        )
        print()
        print("  Per-position-set residual stats:")
        for pos_id, stats in cal_result.position_set_residuals.items():
            print(
                f"    {pos_id}: "
                f"rms={stats['rms'] * 1000:.3f} um  "
                f"max={stats['max_abs'] * 1000:.3f} um"
            )

    # ------------------------------------------------------------------
    # save_results
    # ------------------------------------------------------------------
    def save_results(self, file_path: Optional[str] = None) -> str:
        """Write a JSON-serializable summary of every result.

        If ``file_path`` is ``None`` the path
        :attr:`default_json_path` is used (``results/<stem>.json``).

        All errors / residuals in the output are in **micrometres**.

        Returns the path that was written to.
        """
        file_path = self._resolve_output_path(file_path, self.default_json_path)

        cal = self.pivot_calibration
        res_um = cal.residuals_mm * 1000.0
        euler_rad = rotation_matrix_to_euler_zyx(cal.B_T_P[:3, :3])
        euler_deg = tuple(math.degrees(float(v)) for v in euler_rad)
        data = {
            "metadata": {
                "filename": self.filename,
                "timestamp": self.timestamp,
                "calibration_reference_frame": self.reference_frame,
                "calibration_fixed_reference_frame": self.fixed_reference_frame,
                "calibration_sphere_diameter_mm": self.diameter_mm,
                "calibration_sphere_radius_mm": self.radius_mm,
                "n_sphere_sets": self.n_sets,
                "n_position_sets": self.n_position_sets,
                "sphere_ids": self.sphere_ids,
            },
            "pivots": {
                sphere_set_id: {
                    "sphere_set_id": sphere_set_id,
                    "center_mm": {
                        "x": float(result.center.x),
                        "y": float(result.center.y),
                        "z": float(result.center.z),
                    },
                    "radius_mm": float(result.radius_mm),
                    "rms_error_um": float(result.rms_error_mm * 1000.0),
                    "max_abs_error_um": float(result.max_abs_error_mm * 1000.0),
                    "point_count": int(result.point_count),
                    "residuals_by_sphere_id_um": {
                        sid: float(value) * 1000.0
                        for sid, value in result.residuals_by_sphere_id.items()
                    },
                }
                for sphere_set_id, result in self.pivots.items()
            },
            "pivot_calibration": {
                "B_T_P": {
                    "translation_mm": {
                        "x": float(cal.B_T_P[0, 3]),
                        "y": float(cal.B_T_P[1, 3]),
                        "z": float(cal.B_T_P[2, 3]),
                    },
                    "rotation_matrix": [
                        [float(cal.B_T_P[i, j]) for j in range(3)]
                        for i in range(3)
                    ],
                    "rotation_euler_zyx_rad": {
                        "yaw":   float(euler_rad[0]),
                        "pitch": float(euler_rad[1]),
                        "roll":  float(euler_rad[2]),
                    },
                    "rotation_euler_zyx_deg": {
                        "yaw":   float(euler_deg[0]),
                        "pitch": float(euler_deg[1]),
                        "roll":  float(euler_deg[2]),
                    },
                },
                "J_t_P_mm": {
                    "x": float(cal.J_t_P[0]),
                    "y": float(cal.J_t_P[1]),
                    "z": float(cal.J_t_P[2]),
                },
                "solution": {
                    "rms_error_um": float(cal.rms_error_mm * 1000.0),
                    "max_abs_error_um": float(cal.max_abs_error_mm * 1000.0),
                    "mean_residual_per_axis_um": {
                        "x": float(res_um[:, 0].mean()),
                        "y": float(res_um[:, 1].mean()),
                        "z": float(res_um[:, 2].mean()),
                    },
                    "std_residual_per_axis_um": {
                        "x": float(res_um[:, 0].std(ddof=0)),
                        "y": float(res_um[:, 1].std(ddof=0)),
                        "z": float(res_um[:, 2].std(ddof=0)),
                    },
                    "iterations": int(cal.iterations),
                    "converged": bool(cal.converged),
                },
                "per_set_residuals_um": {
                    sphere_set_id: {
                        "x": float(res_um[n, 0]),
                        "y": float(res_um[n, 1]),
                        "z": float(res_um[n, 2]),
                        "norm": float(np.linalg.norm(cal.residuals_mm[n]) * 1000.0),
                    }
                    for n, sphere_set_id in enumerate(cal.sphere_set_ids)
                },
                "position_set_residuals_um": {
                    pos_id: {
                        "rms_um": float(stats["rms"]) * 1000.0,
                        "max_abs_um": float(stats["max_abs"]) * 1000.0,
                    }
                    for pos_id, stats in cal.position_set_residuals.items()
                },
            },
        }

        parent = os.path.dirname(os.path.abspath(file_path))
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, sort_keys=False)
        return file_path

    # ------------------------------------------------------------------
    # plot_results
    # ------------------------------------------------------------------
    def plot_results(
        self,
        file_path: Optional[str] = None,
        individual: bool = True,
        summary: bool = False,
    ) -> list[str]:
        """Save the diagnostic plot(s) to disk.

        Parameters
        ----------
        file_path : str, optional
            Base output path.  Defaults to
            :attr:`default_plot_path` (``results/<stem>.png``).
            When ``individual=True`` each per-figure file is written
            with a numeric suffix on this stem (e.g.
            ``results/<stem>_01_3d_centers.png``).
        individual : bool
            If ``True`` (the default), also write every diagnostic as a
            separate PNG via
            :func:`pivot_calibration_plot.plot_pivot_calibration_individual`.
        summary : bool
            If ``True``, also write the combined 2x2 multi-panel
            figure as ``results/<stem>.png``.  Default is ``False`` --
            the user typically only wants the individual figures.

        Returns
        -------
        list[str]
            All the output paths that were written (combined figure
            first if ``summary=True``, then the individual ones).
        """
        from pm_robot_calibration.py_modules.hexapod_calibration.pivot_calibration_plot import (
            plot_pivot_calibration_errors,
            plot_pivot_calibration_individual,
        )

        file_path = self._resolve_output_path(file_path, self.default_plot_path)

        title = (
            f"Pivot calibration -- {self.filename}  "
            f"(D = {self.diameter_mm:.4f} mm, "
            f"RMS = {self.pivot_calibration.rms_error_mm * 1000:.2f} um, "
            f"max = {self.pivot_calibration.max_abs_error_mm * 1000:.2f} um)"
        )

        written: list[str] = []

        # 1. The combined 2x2 figure (opt-in now).
        if summary:
            combined = plot_pivot_calibration_errors(
                self.pivot_calibration,
                self.sphere_calibration.sphere_sets,
                file_path,
                title=title,
            )
            written.append(combined)

        # 2. One figure per diagnostic.
        if individual:
            extra = plot_pivot_calibration_individual(
                self.pivot_calibration,
                self.sphere_calibration.sphere_sets,
                file_path,
                title=title,
            )
            written.extend(extra)

        return written

    # ------------------------------------------------------------------
    # sweep_diameter
    # ------------------------------------------------------------------
    def sweep_diameter(
        self,
        diameters_mm: list[float],
    ) -> list[dict]:
        """Run the calibration for each diameter in ``diameters_mm``.

        For each diameter, fits a pivot per sphere set, solves
        B__T__P / J__t__P, and returns a dict with the diameter,
        the resulting B__T__P translation, J__t__P, and the
        pivot-calibration RMS / max errors.  Handy for inspecting
        the sensitivity of the calibration to the assumed sphere
        diameter.

        Returns a list of dicts, one per diameter, with keys:

        ``diameter_mm``, ``radius_mm``,
        ``B_T_P_translation_mm`` (3-vector),
        ``J_t_P_mm`` (3-vector),
        ``rms_error_um``, ``max_abs_error_um``,
        ``converged`` (bool), ``iterations`` (int).
        """
        results: list[dict] = []
        for d in diameters_mm:
            d = float(d)
            r = 0.5 * d
            pivots = self.sphere_calibration.fit_pivot_per_set(radius_mm=r)
            cal = self.sphere_calibration.solve_pivot_calibration(pivots=pivots)
            results.append({
                "diameter_mm": d,
                "radius_mm": r,
                "B_T_P_translation_mm": np.asarray(
                    cal.B_T_P[:3, 3], dtype=np.float64,
                ),
                "J_t_P_mm": np.asarray(cal.J_t_P, dtype=np.float64),
                "rms_error_um": float(cal.rms_error_mm) * 1000.0,
                "max_abs_error_um": float(cal.max_abs_error_mm) * 1000.0,
                "converged": bool(cal.converged),
                "iterations": int(cal.iterations),
            })
        return results

    def print_diameter_sweep(
        self,
        diameters_mm: list[float],
    ) -> list[dict]:
        """Run :meth:`sweep_diameter` and print a compact table.

        Convenience for interactive use — the return value is the same
        as :meth:`sweep_diameter`, in case the caller wants to plot it.
        """
        results = self.sweep_diameter(diameters_mm)
        print()
        print("Diameter sweep:")
        print(
            f"  {'D (mm)':>10s}  {'R (mm)':>10s}  "
            f"{'rms (um)':>10s}  {'max (um)':>10s}  "
            f"{'B__T__P x (mm)':>16s}  {'B__T__P y (mm)':>16s}  {'B__T__P z (mm)':>16s}  "
            f"{'J__t__P x (mm)':>16s}  {'J__t__P y (mm)':>16s}  {'J__t__P z (mm)':>16s}"
        )
        print("  " + "-" * 150)
        for row in results:
            b = row["B_T_P_translation_mm"]
            j = row["J_t_P_mm"]
            print(
                f"  {row['diameter_mm']:>10.4f}  {row['radius_mm']:>10.4f}  "
                f"{row['rms_error_um']:>10.3f}  {row['max_abs_error_um']:>10.3f}  "
                f"{b[0]:>+16.4f}  {b[1]:>+16.4f}  {b[2]:>+16.4f}  "
                f"{j[0]:>+16.4f}  {j[1]:>+16.4f}  {j[2]:>+16.4f}"
            )
        return results
