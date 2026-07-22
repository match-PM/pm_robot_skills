"""
SphereCalibration: the data model for a Smarpod calibration run.

A ``SphereCalibration`` is loaded from one calibration JSON file
(see ``measurements/calibration_data_20260716_153250.json``).  It
contains:

* a list of ``SphereSet`` (one per commanded configuration) — the new
  format packs 9 ``SphereMeasurement`` records into each set
* a list of unique sphere ids (the persistent ``CAL_Smarpod_Ball_*``
  identifiers)
* methods to fit a sphere to every calibration sphere's trajectory
  across all sphere sets, with the per-point signed radial residual
  written back to the underlying ``SphereMeasurement`` objects

Backwards-compat: the previous public class name was ``MeasurementSet``;
it remains importable under that name as an alias.
"""
import os
import json
from dataclasses import dataclass, field
from typing import Optional, Callable

import numpy as np
from numpy.typing import NDArray

from pm_robot_calibration.py_modules.hexapod_calibration.data_classes  import (
    Point,
    SphereSet,
    SphereFitResult,
    SphereMeasurements,
    SPHERE_IDS,
)

from pm_robot_calibration.py_modules.hexapod_calibration.geometry_utils import (
    normal_to_angles,
    rotation_matrix_to_euler_zyx,
)


# --------------------------------------------------------------------- helpers
def _read_calibration_payload(filepath: str) -> dict:
    """Read a calibration JSON file and return the raw top-level dict."""
    with open(filepath) as f:
        raw = json.load(f)
    if isinstance(raw, dict):
        return raw
    if isinstance(raw, list):
        return {"calibration_data": raw, "format": "legacy_list"}
    raise ValueError(
        f"Unexpected calibration JSON root type {type(raw).__name__} in {filepath!r}"
    )


def _format_float(value) -> str:
    """Format a float in a stable, readable way for ids."""
    if isinstance(value, float) and value.is_integer():
        return str(int(value))
    return str(value)


# ------------------------------------------------------ rigid-transform helpers
def _rotation_matrix_zyx(rx: float, ry: float, rz: float) -> NDArray[np.float64]:
    """Build a 3x3 rotation matrix from ZYX intrinsic Euler angles (rad).

    The rotation is ``R = Rz @ Ry @ Rx`` — i.e. roll about X is applied
    first, then pitch about Y, then yaw about Z.  The hexapod's
    ``P__T__Jn`` transform uses this convention, matching
    ``SphereSet.sphere_set_transform``.
    """
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0,  cx, -sx],
                   [0.0,  sx,  cx]])
    Ry = np.array([[ cy, 0.0,  sy],
                   [0.0, 1.0, 0.0],
                   [-sy, 0.0,  cy]])
    Rz = np.array([[ cz, -sz, 0.0],
                   [ sz,  cz, 0.0],
                   [0.0, 0.0, 1.0]])
    return Rz @ Ry @ Rx


def _rigid_from_axis_angle(axis: NDArray[np.float64], angle_rad: float) -> NDArray[np.float64]:
    """Rodrigues' formula: 3x3 rotation from a (possibly unnormalised) axis.

    The rotation is by ``angle_rad`` radians about the (normalised) axis.
    The norm of ``axis`` is ignored.
    """
    n = float(np.linalg.norm(axis))
    if n < 1e-15:
        return np.eye(3)
    kx, ky, kz = axis / n
    c = float(np.cos(angle_rad))
    s = float(np.sin(angle_rad))
    one_minus_c = 1.0 - c
    return np.array([
        [c + kx * kx * one_minus_c,        kx * ky * one_minus_c - kz * s,  kx * kz * one_minus_c + ky * s],
        [ky * kx * one_minus_c + kz * s,   c + ky * ky * one_minus_c,        ky * kz * one_minus_c - kx * s],
        [kz * kx * one_minus_c - ky * s,   kz * ky * one_minus_c + kx * s,    c + kz * kz * one_minus_c],
    ], dtype=np.float64)


def _exp_skew(delta_w: NDArray[np.float64]) -> NDArray[np.float64]:
    """Closed-form matrix exponential of a 3x3 skew-symmetric matrix.

    Given a 3-vector ``delta_w``, returns ``exp([delta_w]_x)``.
    This is the right-multiplied rotation perturbation used by the
    pivot calibration solver.  The norm of ``delta_w`` is the
    rotation angle in radians.
    """
    angle = float(np.linalg.norm(delta_w))
    if angle < 1e-15:
        # First-order expansion for very small angles: I + [delta_w]_x.
        return np.eye(3) + _skew(delta_w)
    axis = np.asarray(delta_w, dtype=np.float64).reshape(3) / angle
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    one_minus_c = 1.0 - c
    kx, ky, kz = float(axis[0]), float(axis[1]), float(axis[2])
    return np.array([
        [c + kx * kx * one_minus_c,        kx * ky * one_minus_c - kz * s,  kx * kz * one_minus_c + ky * s],
        [ky * kx * one_minus_c + kz * s,   c + ky * ky * one_minus_c,        ky * kz * one_minus_c - kx * s],
        [kz * kx * one_minus_c - ky * s,   kz * ky * one_minus_c + kx * s,    c + kz * kz * one_minus_c],
    ], dtype=np.float64)


def _skew(v: NDArray[np.float64]) -> NDArray[np.float64]:
    """3x3 skew-symmetric (cross-product) matrix of a 3-vector."""
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    return np.array([[ 0.0, -z,    y],
                     [   z, 0.0, -x],
                     [  -y,   x, 0.0]], dtype=np.float64)


# ---------------------------------------------------------- results container
@dataclass
class SphereCalibrationResults:
    sphere_calibration: object
    main_normal_direction: NDArray[np.float64]
    main_normal_direction_deg: NDArray[np.float64]
    average_slope_deviation_deg: float
    rotation_center_error_deg: Point
    rotation_center_max_error_mm: float

    def get_as_dict(self):
        return {
            "main_normal_direction": {
                "x": float(self.main_normal_direction[0]),
                "y": float(self.main_normal_direction[1]),
                "z": float(self.main_normal_direction[2]),
            },
            "main_normal_direction_deg": {
                "tilt_x_deg": float(self.main_normal_direction_deg[0]),
                "tilt_y_deg": float(self.main_normal_direction_deg[1]),
                "angle_from_z_deg": float(self.main_normal_direction_deg[2]),
            },
            "average_slope_deviation_deg": self.average_slope_deviation_deg,
            "rotation_center_error_deg": {
                "x": self.rotation_center_error_deg.x,
                "y": self.rotation_center_error_deg.y,
                "z": self.rotation_center_error_deg.z,
            },
            "rotation_center_max_error_mm": self.rotation_center_max_error_mm,
        }


# ----------------------------------------------------------- pivot calibration
@dataclass
class PivotCalibrationResult:
    """Result of the pivot calibration: the unknown ``B__T__P`` and ``J__t__P``.

    The chain equation is ::

        B__T__Pn = B__T__P @ P__T__Jn @ T(J__t__P)

    where ``P__T__Jn`` is the ideal transform commanded by the
    hexapod and ``T(J__t__P)`` is a pure translation by the (static)
    sphere offset.  ``B__T__Pn`` is the measured sphere centre for
    set ``n``, recovered by :meth:`SphereCalibration.fit_pivot_per_set`.
    """

    # The two unknowns we are solving for.
    B_T_P: NDArray[np.float64]            # 4x4 base -> true pivot origin
    J_t_P: NDArray[np.float64]            # 3-vector, Jn -> sphere centre (mm)

    # Per-set diagnostics.
    sphere_set_ids: list[str]
    measured_centers_mm: NDArray[np.float64]   # (N, 3) input: B__T__Pn
    predicted_centers_mm: NDArray[np.float64]  # (N, 3) output: B__T__P @ P__T__Jn @ T(J__t__P)
    residuals_mm: NDArray[np.float64]          # (N, 3) per-set translation error

    # Solution quality.
    rms_error_mm: float
    max_abs_error_mm: float
    iterations: int
    converged: bool

    # Optional grouping of the residual stats by (x_cmd, y_cmd) position set.
    position_set_residuals: dict[str, dict[str, float]] = field(default_factory=dict)

    # ---- convenience views ----------------------------------------------
    @property
    def sphere_offset_mm(self) -> NDArray[np.float64]:
        """Alias for :attr:`J_t_P` (mm)."""
        return self.J_t_P

    @property
    def base_to_pivot_mm(self) -> NDArray[np.float64]:
        """3-vector translation of :attr:`B_T_P` (mm)."""
        return np.asarray(self.B_T_P[:3, 3], dtype=np.float64).reshape(3)

    @property
    def base_to_pivot_rotation(self) -> NDArray[np.float64]:
        """3x3 rotation of :attr:`B_T_P`."""
        return np.asarray(self.B_T_P[:3, :3], dtype=np.float64)

    def get_as_dict(self) -> dict:
        """Return a JSON-serializable representation of the result."""
        return {
            "B_T_P": {
                "translation_mm": {
                    "x": float(self.B_T_P[0, 3]),
                    "y": float(self.B_T_P[1, 3]),
                    "z": float(self.B_T_P[2, 3]),
                },
                "rotation_matrix": [
                    [float(self.B_T_P[i, j]) for j in range(3)]
                    for i in range(3)
                ],
            },
            "J_t_P_mm": {
                "x": float(self.J_t_P[0]),
                "y": float(self.J_t_P[1]),
                "z": float(self.J_t_P[2]),
            },
            "rms_error_mm": float(self.rms_error_mm),
            "max_abs_error_mm": float(self.max_abs_error_mm),
            "iterations": int(self.iterations),
            "converged": bool(self.converged),
            "per_set_residual_norm_mm": [
                float(np.linalg.norm(row))
                for row in self.residuals_mm
            ],
            "position_set_residuals": self.position_set_residuals,
        }


# ----------------------------------------------------------------- main class
@dataclass
class SphereCalibration:
    """A collection of ``SphereSet`` loaded from a single calibration file."""

    sphere_sets: list[SphereSet] = field(default_factory=list)
    filename: Optional[str] = None
    source_payload: Optional[dict] = field(default=None, repr=False, compare=False)

    # ---- top-level metadata from the new calibration JSON format ----
    timestamp: Optional[str] = None
    calibration_fixed_reference_frame: Optional[str] = None
    calibration_reference_frame: Optional[str] = None
    current_calibration_transformation: Optional[dict] = None
    goal_handle: Optional[dict] = None

    # ----- backwards-compat: ``poses`` and ``Pose`` alias attributes
    @property
    def poses(self) -> list[SphereSet]:
        return self.sphere_sets

    @poses.setter
    def poses(self, value: list[SphereSet]) -> None:
        self.sphere_sets = value

    @staticmethod
    def list_position_sets_in_file(filepath: str) -> list[str]:
        """Return the (x, y) position ids available in the file."""
        payload = _read_calibration_payload(filepath)
        keys: set[tuple[float, float]] = set()
        for s in payload.get("calibration_data", []):
            keys.add((
                float(s.get("x_cmd", 0.0)),
                float(s.get("y_cmd", 0.0)),
            ))
        return [f"x{_format_float(x)}_y{_format_float(y)}" for (x, y) in sorted(keys)]

    @staticmethod
    def list_sets_in_file(filepath: str) -> list[str]:
        """Return the unique (x, y, rx, ry) set ids in the file."""
        payload = _read_calibration_payload(filepath)
        keys: set[tuple[float, float, float, float]] = set()
        for s in payload.get("calibration_data", []):
            keys.add((
                float(s.get("x_cmd", 0.0)),
                float(s.get("y_cmd", 0.0)),
                float(s.get("rx_cmd", 0.0)),
                float(s.get("ry_cmd", 0.0)),
            ))
        return [
            f"x{_format_float(x)}_y{_format_float(y)}_"
            f"rx{_format_float(rx)}_ry{_format_float(ry)}"
            for (x, y, rx, ry) in sorted(keys)
        ]

    @classmethod
    def load_file(cls, filepath: str) -> "SphereCalibration":
        """Load **all** sphere sets from a calibration JSON file."""
        payload = _read_calibration_payload(filepath)
        sets = [SphereSet.from_pose_dict(s) for s in payload.get("calibration_data", [])]
        return cls.from_payload(payload, sets, filepath)

    @classmethod
    def load_position_set(
        cls,
        filepath: str,
        position_set_id: str,
    ) -> "SphereCalibration":
        """Load only the sphere sets that share the given (x, y) position."""
        try:
            parts = position_set_id.split("_")
            x_cmd = float(parts[0][1:])   # strip leading "x"
            y_cmd = float(parts[1][1:])   # strip leading "y"
        except (IndexError, ValueError) as exc:
            raise ValueError(
                f"Malformed position_set_id {position_set_id!r}; expected "
                f"'x{{}}_y{{}}' e.g. 'x0.0_y0.0'"
            ) from exc

        target_xy = (x_cmd, y_cmd)
        payload = _read_calibration_payload(filepath)
        sets_all = payload.get("calibration_data", [])
        matching = [
            s for s in sets_all
            if (float(s.get("x_cmd", 0.0)),
                float(s.get("y_cmd", 0.0))) == target_xy
        ]

        if not matching:
            available = sorted({
                (float(s.get("x_cmd", 0.0)),
                 float(s.get("y_cmd", 0.0)))
                for s in sets_all
            })
            raise ValueError(
                f"No sphere sets found for position_set_id={position_set_id!r} "
                f"(target xy={target_xy}) in {filepath!r}. "
                f"Available positions: {available}"
            )

        sphere_sets = [SphereSet.from_pose_dict(s) for s in matching]
        return cls.from_payload(payload, sphere_sets, filepath)

    @classmethod
    def load_all_positions(cls, filepath: str) -> dict[str, "SphereCalibration"]:
        """Load every (x, y) position and return a dict keyed by position id."""
        return {
            pid: cls.load_position_set(filepath, pid)
            for pid in cls.list_position_sets_in_file(filepath)
        }

    @classmethod
    def load(cls, filename: str, position_set_id: str) -> "SphereCalibration":
        """Backwards-compat: ``SphereCalibration.load(file, "x0_y0")``."""
        if not position_set_id:
            return cls.load_file(filename)
        return cls.load_position_set(filename, position_set_id)

    @classmethod
    def from_payload(
        cls,
        payload: dict,
        sphere_sets: list[SphereSet],
        filepath: Optional[str] = None,
    ) -> "SphereCalibration":
        """Build a ``SphereCalibration`` from a raw payload and set list."""
        base_name = (
            os.path.splitext(os.path.basename(filepath))[0]
            if filepath else None
        )
        return cls(
            sphere_sets=sphere_sets,
            filename=base_name,
            source_payload=payload,
            timestamp=payload.get("timestamp"),
            calibration_fixed_reference_frame=payload.get("calibration_fixed_reference_frame"),
            calibration_reference_frame=payload.get("calibration_reference_frame"),
            current_calibration_transformation=payload.get("current_calibration_transformation"),
            goal_handle=payload.get("goal_handle"),
        )

    def get_max_sphere_fit_error(self) -> tuple[Optional[str], Optional[float]]:
        """Return the (sphere_id, max_abs_error_mm) across all sphere sets.

        Looks at the ``sphere_fit_error_mm`` field on every
        ``SphereMeasurement``.  The value is a *signed* radial residual
        (``distance(point, center) - radius``), so we compare absolute
        values.  Returns ``(None, None)`` if no fit has been run yet.
        """
        max_abs_error = 0.0
        max_sphere_id: Optional[str] = None
        for sphere_set in self.sphere_sets:
            for measurement in sphere_set.sphere_measurements.get_all_sphere_measurements():
                if measurement.sphere_fit_error_mm is None:
                    continue
                abs_err = abs(measurement.sphere_fit_error_mm)
                if abs_err > max_abs_error:
                    max_abs_error = abs_err
                    max_sphere_id = measurement.sphere_id
        if max_sphere_id is None:
            return None, None
        return max_sphere_id, float(max_abs_error)
    # ------------------------------------------------------------ iteration
    def __iter__(self):
        return iter(self.sphere_sets)

    def __len__(self):
        return len(self.sphere_sets)

    # -------------------------------------------------------------- lookup
    def get_sphere_positions(self, sphere_id: str) -> NDArray[np.float64]:
        """Return an ``(N, 3)`` array of positions of one sphere (mm)."""
        pts: list[list[float]] = []
        for sphere_set in self.sphere_sets:
            measurement = sphere_set.sphere_measurements.get_sphere_measurement(sphere_id)
            if measurement is None:
                continue
            pts.append([
                measurement.measurement_position.x,
                measurement.measurement_position.y,
                measurement.measurement_position.z,
            ])
        return np.array(pts, dtype=np.float64)

    def get_all_sphere_ids(self) -> list[str]:
        """Sorted, unique list of sphere ids present in this calibration."""
        ids: set[str] = set()
        for sphere_set in self.sphere_sets:
            ids.update(sphere_set.sphere_measurements.sphere_ids)
        return sorted(ids)

    def fit_pivot_per_set(
        self,
        radius_mm: Optional[float] = None,
    ) -> dict[str, SphereFitResult]:
        """Fit **one** sphere with known radius per sphere set.

        For every ``SphereSet`` in this calibration, fit a sphere of
        radius ``radius_mm`` (default ``CALIBRATION_SPHERE_RADIUS_MM``,
        i.e. 6.35 mm / 2) to the 9 measurements in the set.  The
        resulting ``SphereFitResult.sphere_id`` is the sphere set id and
        the per-point residuals are stored in
        ``residuals_by_sphere_id`` keyed by the original ball id.

        Returns
        -------
        dict[str, SphereFitResult]
            Mapping ``sphere_set_id -> fit result``.  There is one entry
            per sphere set (e.g. 48 entries for this calibration).
        """
        return {
            sphere_set.sphere_set_id: SphereSet.fit_pivot(sphere_set, radius_mm)
            for sphere_set in self.sphere_sets
        }

    def get_sphere_set_by_iteration(
        self,
        iteration: int,
    ) -> Optional[SphereSet]:
        for s in self.sphere_sets:
            if s.current_iteration == iteration:
                return s
        return None

    def run_calibration(
        self,
        diameter_mm: Optional[float] = None,
        radius_mm: Optional[float] = None,
    ) -> "CalibrationAnalysis":
        """Run the full pivot analysis: fit per set + solve B__T__P / J__t__P.

        You can specify the calibration-sphere diameter either as a
        ``diameter_mm`` (the value printed on the ball, e.g. 6.35) or
        as a ``radius_mm`` (3.175 for the same ball).  ``diameter_mm``
        takes precedence if both are given.  Pass ``None`` for either
        to use the module default from
        :data:`data_classes.CALIBRATION_SPHERE_DIAMETER_MM`.

        Returns a :class:`calibration_analysis.CalibrationAnalysis`
        that bundles every result and exposes ``print_results``,
        ``save_results``, ``plot_results`` and ``sweep_diameter``
        helpers.
        """
        from pm_robot_calibration.py_modules.hexapod_calibration.calibration_analysis import (
            CalibrationAnalysis,
        )

        return CalibrationAnalysis.run(
            self,
            diameter_mm=diameter_mm,
            radius_mm=radius_mm,
        )

    def solve_pivot_calibration(
        self,
        pivots: Optional[dict[str, "SphereFitResult"]] = None,
        max_iterations: int = 200,
        position_tolerance_mm: float = 1e-12,
        angle_tolerance_rad: float = 1e-12,
    ) -> "PivotCalibrationResult":
        """Solve for ``B__T__P`` and ``J__t__P`` from the fitted pivots.

        The chain equation is ::

            B__T__Pn = B__T__P @ P__T__Jn @ T(J__t__P)

        where ``P__T__Jn`` is the ideal hexapod transform commanded
        for set ``n`` (built from ``rx_cmd``, ``ry_cmd``, ``rz_cmd``,
        ``x_cmd``, ``y_cmd``) and ``T(J__t__P)`` is a pure translation
        by the (static) sphere offset.  ``B__T__Pn`` is the measured
        sphere centre for set ``n`` (a 3-vector).

        The unknowns are the 6 elements of ``B__T__P`` (a rigid 4x4
        transform) and the 3 elements of ``J__t__P`` — 9 parameters
        total.  We solve the over-determined nonlinear least-squares
        problem with a Gauss-Newton iteration that respects the rigid
        constraint on ``B__T__P`` (translation + axis-angle rotation).

        Parameters
        ----------
        pivots : dict[str, SphereFitResult], optional
            Mapping ``sphere_set_id -> SphereFitResult`` as returned by
            :meth:`fit_pivot_per_set`.  Defaults to running
            ``fit_pivot_per_set()`` on this calibration.
        max_iterations : int
            Maximum number of Gauss-Newton iterations.
        position_tolerance_mm, angle_tolerance_rad
            Stop iterating when the correction step is below these.

        Returns
        -------
        PivotCalibrationResult
            ``B_T_P`` (4x4), ``J_t_P`` (3-vector), per-set residuals,
            overall RMS / max error, and per-position-set residual
            stats (grouped by ``(x_cmd, y_cmd)``).
        """
        if pivots is None:
            pivots = self.fit_pivot_per_set()
        if not pivots:
            raise ValueError("No pivots supplied; cannot solve calibration.")

        # ------------------------------------------------------------------
        # Build the per-set commanded transforms and measured centres, in
        # the order implied by ``pivots``.
        # ------------------------------------------------------------------
        sphere_set_ids: list[str] = []
        P_T_J_list: list[NDArray[np.float64]] = []
        measured_centers: list[NDArray[np.float64]] = []
        position_keys: list[tuple[float, float]] = []

        for sphere_set_id, result in pivots.items():
            sphere_set = next(
                (s for s in self.sphere_sets if s.sphere_set_id == sphere_set_id),
                None,
            )
            if sphere_set is None:
                raise ValueError(
                    f"Unknown sphere set id {sphere_set_id!r} in pivots; "
                    f"not present in this calibration"
                )
            sphere_set_ids.append(sphere_set_id)
            P_T_J_list.append(np.asarray(sphere_set.sphere_set_transform, dtype=np.float64))
            measured_centers.append(np.asarray(
                [result.center.x, result.center.y, result.center.z],
                dtype=np.float64,
            ))
            position_keys.append((float(sphere_set.x_cmd), float(sphere_set.y_cmd)))

        # Pre-compute the 3x3 rotations and 3-vectors of the per-set commanded
        # transforms so the inner loop is just a few multiplies.
        P_R_J = np.stack([P[:3, :3] for P in P_T_J_list], axis=0)  # (N, 3, 3)
        P_t_J = np.stack([P[:3,  3] for P in P_T_J_list], axis=0)  # (N, 3)
        measured_centers_arr = np.stack(measured_centers, axis=0)  # (N, 3)

        # ------------------------------------------------------------------
        # Initial guess.
        #
        # Take ``B__T__P`` as a pure translation by the centroid of the
        # measured centres (after subtracting the mean commanded pivot
        # point), and ``J__t__P = 0``.  The Gauss-Newton converges from
        # this in well under 20 iterations.
        # ------------------------------------------------------------------
        mean_measured = measured_centers_arr.mean(axis=0)
        mean_commanded = P_t_J.mean(axis=0)
        initial_translation = mean_measured - mean_commanded
        B_T_P = np.eye(4, dtype=np.float64)
        B_T_P[:3, 3] = initial_translation
        J_t_P = np.zeros(3, dtype=np.float64)

        # ------------------------------------------------------------------
        # Gauss-Newton over (delta_t (3), delta_w (3) for axis-angle,
        # delta_s (3) for the offset vector).  The rotation of B__T__P is
        # updated as R <- R @ exp([delta_w]_x) (right-multiplied, so the
        # 3-vector lives in the B frame, which keeps the math stable).
        # ------------------------------------------------------------------
        iterations = 0
        converged = False
        prev_rms: Optional[float] = None
        for it in range(1, max_iterations + 1):
            iterations = it
            B_R_P = B_T_P[:3, :3]
            B_t_P = B_T_P[:3, 3]

            # Predicted centres: B_R_P @ P_R_J_n @ J_t_P + (B_t_P + B_R_P @ P_t_J_n)
            R_j_offset = np.einsum("nij,j->ni", P_R_J, J_t_P)             # (N, 3)
            predicted_centers = (
                np.einsum("ij,nj->ni", B_R_P, R_j_offset)
                + B_t_P
                + np.einsum("ij,nj->ni", B_R_P, P_t_J)
            )
            residuals = predicted_centers - measured_centers_arr          # (N, 3)

            # Build the 3N x 9 Jacobian analytically.
            #
            # Let delta_t, delta_w, delta_s be the (small) corrections to
            # the translation of B__T__P, the right-multiplied axis-angle
            # of B_R_P, and J_t_P respectively.  Then to first order
            #   B_R_P  <-  B_R_P @ (I + [delta_w]_x)
            #   B_t_P  <-  B_t_P + delta_t
            #   J_t_P  <-  J_t_P + delta_s
            #
            # The predicted centre for set n is
            #   c_n = B_t_P + B_R_P @ P_t_J_n + B_R_P @ P_R_J_n @ J_t_P
            # Define t_n = P_t_J_n + P_R_J_n @ J_t_P and
            #       t_world_n = B_R_P @ t_n (the "rotated" offset in B).
            # Then
            #   c_n  =  B_t_P + B_R_P @ t_n
            #        =  B_t_P + B_R_P @ (I + [delta_w]_x) @ t_n
            #        =  B_t_P + B_R_P @ t_n  +  B_R_P @ [delta_w]_x @ t_n
            # Using [a]_x b = -[b]_x a and writing the perturbation in
            # the **world** frame, B_R_P @ delta_w = w,
            #   B_R_P @ [delta_w]_x @ t_n  =  -[B_R_P @ t_n]_x @ (B_R_P @ delta_w)
            #                                =  -[t_world_n]_x @ (B_R_P @ delta_w)
            # Therefore
            #   dc_n / d delta_t = I
            #   dc_n / d delta_w = -[t_world_n]_x @ B_R_P
            #   dc_n / d delta_s =  B_R_P @ P_R_J_n
            N = P_R_J.shape[0]
            t_rot = P_t_J + np.einsum("nij,j->ni", P_R_J, J_t_P)         # (N, 3)
            t_rot_world = np.einsum("ij,nj->ni", B_R_P, t_rot)           # (N, 3)

            J_t = np.zeros((3 * N, 3), dtype=np.float64)
            J_w = np.zeros((3 * N, 3), dtype=np.float64)
            J_s = np.zeros((3 * N, 3), dtype=np.float64)
            for n in range(N):
                row = slice(3 * n, 3 * n + 3)
                J_t[row] = np.eye(3)
                J_w[row] = -_skew(t_rot_world[n]) @ B_R_P
                J_s[row] = B_R_P @ P_R_J[n]
            J = np.hstack([J_t, J_w, J_s])                                # (3N, 9)

            # Solve J @ delta = -residuals in the least-squares sense.
            rhs = -residuals.reshape(3 * N)
            try:
                delta, _, _, _ = np.linalg.lstsq(J, rhs, rcond=None)
            except np.linalg.LinAlgError:
                break

            delta_t = delta[0:3]
            delta_w = delta[3:6]
            delta_s = delta[6:9]

            B_t_P = B_t_P + delta_t
            B_R_P = B_R_P @ _exp_skew(delta_w)
            J_t_P = J_t_P + delta_s

            # Compute the current RMS for the convergence criterion.
            R_j_offset = np.einsum("nij,j->ni", P_R_J, J_t_P)
            predicted_now = (
                np.einsum("ij,nj->ni", B_R_P, R_j_offset)
                + B_t_P
                + np.einsum("ij,nj->ni", B_R_P, P_t_J)
            )
            current_rms = float(np.sqrt(np.mean(
                np.linalg.norm(predicted_now - measured_centers_arr, axis=1) ** 2,
            )))

            step_translation = float(np.linalg.norm(delta_t))
            step_angle = float(np.linalg.norm(delta_w))
            step_offset = float(np.linalg.norm(delta_s))
            hard_converged = (
                step_translation < position_tolerance_mm
                and step_angle < angle_tolerance_rad
                and step_offset < position_tolerance_mm
            )
            # Soft convergence: the relative improvement in RMS is below
            # 1e-6 — i.e. we are at the noise floor.
            soft_converged = (
                prev_rms is not None
                and prev_rms > 0
                and abs(prev_rms - current_rms) / max(prev_rms, 1e-30) < 1e-6
            )
            prev_rms = current_rms
            if hard_converged or soft_converged:
                converged = True
                break

        # Write the final B_T_P back together.
        B_T_P = np.eye(4, dtype=np.float64)
        B_T_P[:3, :3] = B_R_P
        B_T_P[:3,  3] = B_t_P

        # Final residual statistics.
        # (Reuse the prediction loop above with the converged params.)
        R_j_offset = np.einsum("nij,j->ni", P_R_J, J_t_P)
        predicted_centers = (
            np.einsum("ij,nj->ni", B_R_P, R_j_offset)
            + B_t_P
            + np.einsum("ij,nj->ni", B_R_P, P_t_J)
        )
        residuals = predicted_centers - measured_centers_arr
        residual_norms = np.linalg.norm(residuals, axis=1)
        rms_error = float(np.sqrt(np.mean(residual_norms ** 2)))
        max_abs_error = float(np.max(residual_norms))

        # Per-position-set residual stats, keyed by ``x{}_y{}``.
        position_set_residuals: dict[str, dict[str, float]] = {}
        for n, key in enumerate(position_keys):
            pos_id = f"x{_format_float(key[0])}_y{_format_float(key[1])}"
            entry = position_set_residuals.setdefault(
                pos_id,
                {"count": 0, "sum_sq": 0.0, "max_abs": 0.0},
            )
            entry["count"] += 1
            entry["sum_sq"] += float(residual_norms[n] ** 2)
            entry["max_abs"] = max(entry["max_abs"], float(residual_norms[n]))
        for entry in position_set_residuals.values():
            count = max(1, int(entry["count"]))
            entry["rms"] = float(np.sqrt(entry["sum_sq"] / count))
            entry["max_abs"] = float(entry["max_abs"])
            del entry["sum_sq"]
            del entry["count"]

        return PivotCalibrationResult(
            B_T_P=B_T_P,
            J_t_P=J_t_P,
            sphere_set_ids=sphere_set_ids,
            measured_centers_mm=measured_centers_arr,
            predicted_centers_mm=predicted_centers,
            residuals_mm=residuals,
            rms_error_mm=rms_error,
            max_abs_error_mm=max_abs_error,
            iterations=iterations,
            converged=converged,
            position_set_residuals=position_set_residuals,
        )

    def get_sphere_fit_errors(
        self,
        sphere_id: str,
    ) -> dict[str, Optional[float]]:
        """Return the stored sphere-fit residuals by sphere set id."""
        return {
            sphere_set.sphere_set_id: (
                float(measurement.sphere_fit_error_mm)
                if measurement is not None
                and measurement.sphere_fit_error_mm is not None
                else None
            )
            for sphere_set in self.sphere_sets
            for measurement in [
                sphere_set.sphere_measurements.get_sphere_measurement(sphere_id)
            ]
        }

    def list_all_sets(self) -> list[str]:
        """Sorted list of unique (x, y, rx, ry, rz) set ids."""
        keys: set[tuple[float, float, float, float, float]] = set()
        for s in self.sphere_sets:
            keys.add((
                float(s.x_cmd),
                float(s.y_cmd),
                float(s.rx_cmd),
                float(s.ry_cmd),
                float(s.rz_cmd),
            ))
        return [
            f"x{_format_float(x)}_y{_format_float(y)}_"
            f"rx{_format_float(rx)}_ry{_format_float(ry)}_rz{_format_float(rz)}"
            for (x, y, rx, ry, rz) in sorted(keys)
        ]

    @property
    def number_of_sets(self) -> int:
        return len(self.list_all_sets())

    def filter_sphere_sets(self, predicate) -> list[SphereSet]:
        """Return all sphere sets for which ``predicate(sphere_set)`` is True."""
        if isinstance(predicate, str):
            shortcut = _SHORTCUT_PREDICATES.get(predicate)
            if shortcut is None:
                raise ValueError(
                    f"Unknown predicate shortcut {predicate!r}; "
                    f"known shortcuts: {sorted(_SHORTCUT_PREDICATES)}"
                )
            pred = shortcut
        else:
            pred = predicate
        return [s for s in self.sphere_sets if pred(s)]

    # Backwards-compat helpers -------------------------------------------------
    def get_ball_positions(self, ball_id: str) -> NDArray[np.float64]:
        return self.get_sphere_positions(ball_id)

    def get_all_ball_ids(self) -> list[str]:
        return self.get_all_sphere_ids()

    def get_sphere_fit_errors_for_ball(
        self,
        ball_id: str,
    ) -> dict[str, Optional[float]]:
        return self.get_sphere_fit_errors(ball_id)

    def get_pose_by_iteration(self, iteration: int) -> Optional[SphereSet]:
        return self.get_sphere_set_by_iteration(iteration)

    def filter_poses(self, predicate) -> list[SphereSet]:
        return self.filter_sphere_sets(predicate)


# ---------------------------------------------------------- predicate helpers
_SIGN = {
    "==": lambda a, b: a == b,
    "!=": lambda a, b: a != b,
    "<":  lambda a, b: a <  b,
    "<=": lambda a, b: a <= b,
    ">":  lambda a, b: a >  b,
    ">=": lambda a, b: a >= b,
}


_SHORTCUT_PREDICATES: dict[str, Callable] = {
    "rx_zero":     lambda s: abs(s.rx_cmd) <= 1e-9,
    "ry_zero":     lambda s: abs(s.ry_cmd) <= 1e-9,
    "rz_zero":     lambda s: abs(s.rz_cmd) <= 1e-9,
    "rx_pos":      lambda s: s.rx_cmd >  1e-9,
    "ry_pos":      lambda s: s.ry_cmd >  1e-9,
    "rx_neg":      lambda s: s.rx_cmd < -1e-9,
    "ry_neg":      lambda s: s.ry_cmd < -1e-9,
    "rx_nonzero":  lambda s: abs(s.rx_cmd) >  1e-9,
    "ry_nonzero":  lambda s: abs(s.ry_cmd) >  1e-9,
    "rz_nonzero":  lambda s: abs(s.rz_cmd) >  1e-9,
    "rx_eq_ry":    lambda s: abs(s.rx_cmd - s.ry_cmd) <= 1e-9,
    "rx_eq_neg_ry": lambda s: abs(s.rx_cmd + s.ry_cmd) <= 1e-9,
}


# Backwards-compat alias for the previous public class name.
MeasurementSet = SphereCalibration

