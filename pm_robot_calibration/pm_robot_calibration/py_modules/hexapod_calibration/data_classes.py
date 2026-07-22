"""
Small data classes used by the calibration pipeline:
Point, SphereMeasurement(s), SphereFitResult, SphereSet.
"""
import numpy as np
from numpy.typing import NDArray
from dataclasses import dataclass, field
from typing import Optional

# ---------------------------------------------------------------- sphere IDs
# The source JSON uses CAL_Smarpod_Ball_* as the persistent identifiers.
# The Python API refers to them as sphere ids without changing the raw ids.
SPHERE_IDS: tuple[str, ...] = (
    "CAL_Smarpod_Ball_1",
    "CAL_Smarpod_Ball_2",
    "CAL_Smarpod_Ball_3",
    "CAL_Smarpod_Ball_4",
    "CAL_Smarpod_Ball_5",
    "CAL_Smarpod_Ball_6",
    "CAL_Smarpod_Ball_7",
    "CAL_Smarpod_Ball_8",
    "CAL_Smarpod_Ball_9",
)

# Diameter of the physical Smarpod calibration sphere, in millimetres.
# The radius is used by the fixed-radius pivot fit (one sphere per
# sphere_set, fitted to the 9 measured surface points).
CALIBRATION_SPHERE_DIAMETER_MM: float = 6.35
CALIBRATION_SPHERE_RADIUS_MM: float = CALIBRATION_SPHERE_DIAMETER_MM / 2.0


@dataclass
class Point:
    x: float
    y: float
    z: float

    @classmethod
    def from_um(cls, d: dict) -> "Point":
        return cls(
            0.001 * d["x"],
            0.001 * d["y"],
            0.001 * d["z"]
        )

    @classmethod
    def from_mm(cls, d: dict) -> "Point":
        return cls(
            d["x"],
            d["y"],
            d["z"]
        )

    @classmethod
    def from_array(cls, arr: NDArray[np.float64]) -> "Point":
        """
        Unit: millimeters (mm)
        """
        return cls(
            float(arr[0]),
            float(arr[1]),
            float(arr[2])
        )

    def as_array(self) -> NDArray[np.float64]:
        return np.array([self.x, self.y, self.z], dtype=np.float64)



@dataclass
class SphereMeasurement:
    """
    One calibration sphere measurement at a commanded (rx, ry, rz, x, y)
    sphere_set.  The sphere is identified by its ``sphere_id``.

    The measured 3D position in the calibration reference frame is stored
    as a ``Point`` in mm.  The ball's rotation (rx, ry, rz) is stored in
    degrees (the new JSON format reports rotations in degrees).

    ``sphere_fit_error_mm`` is the signed radial residual assigned by a
    sphere fit: ``distance(measurement_position, center) - radius``.
    Positive values mean that the point is outside the fitted sphere.
    """
    sphere_id: str
    measurement_position: Point
    rotation_deg: Optional[NDArray[np.float64]] = None
    sphere_fit_error_mm: Optional[float] = None

    @property
    def measurement_mm(self) -> float:
        """Backwards-compatible alias (no z-correction in the new format)."""
        return 0.0

    @property
    def endeffector_position(self) -> Point:
        """Backwards-compatible alias for the measured point."""
        return self.measurement_position

    @property
    def frame_id(self) -> str:
        """Backwards-compatible alias for ``sphere_id``."""
        return self.sphere_id

    @property
    def sphere_fit_error_abs_mm(self) -> Optional[float]:
        """Absolute radial residual, or ``None`` before fitting."""
        if self.sphere_fit_error_mm is None:
            return None
        return abs(self.sphere_fit_error_mm)

    @property
    def fit_error_mm(self) -> Optional[float]:
        """Short alias for ``sphere_fit_error_mm``."""
        return self.sphere_fit_error_mm

    def get_as_dict(self) -> dict:
        """Return the measurement and its optional fit error as a dict."""
        result = {
            "frame_id": self.frame_id,
            "measurement_position_mm": {
                "x": float(self.measurement_position.x),
                "y": float(self.measurement_position.y),
                "z": float(self.measurement_position.z),
            },
            "rotation_deg": (
                [float(value) for value in self.rotation_deg]
                if self.rotation_deg is not None else None
            ),
            "sphere_fit_error_mm": (
                float(self.sphere_fit_error_mm)
                if self.sphere_fit_error_mm is not None else None
            ),
        }
        return result

    @classmethod
    def from_measurement_dict(cls, pose_data: dict) -> "FrameMeasurement":
        """
        Backwards-compatible loader for the old format (single ball per
        pose, with ``measurement_mm`` + ``transform_endeffector``).
        """
        t = pose_data["transform_endeffector"]["translation"]  # um
        r = pose_data.get("transform_endeffector", {}).get("rotation", {}) or {}
        return cls(
            frame_id="0",
            measurement_position=Point.from_um(t),
            rotation_deg=np.array(
                [float(r.get("rx", 0.0)),
                 float(r.get("ry", 0.0)),
                 float(r.get("rz", 0.0))],
                dtype=np.float64,
            ),
        )

    @classmethod
    def from_sphere_dict(cls, sphere_id: str, sphere_data: dict) -> "SphereMeasurement":
        """
        Build a ``SphereMeasurement`` from a new-format sphere entry.

        The translation is converted from um to mm.  The rotation is
        stored in degrees.
        """
        t = sphere_data["translation"]
        r = sphere_data.get("rotation", {}) or {}
        return cls(
            sphere_id=sphere_id,
            measurement_position=Point.from_um(t),
            rotation_deg=np.array(
                [float(r.get("rx", 0.0)),
                 float(r.get("ry", 0.0)),
                 float(r.get("rz", 0.0))],
                dtype=np.float64,
            ),
        )

    @classmethod
    def from_ball_dict(cls, ball_id: str, ball_data: dict) -> "SphereMeasurement":
        """Backwards-compatible alias for :meth:`from_sphere_dict`."""
        return cls.from_sphere_dict(ball_id, ball_data)


@dataclass
class SphereFitResult:
    """Result of fitting a sphere to one or more sphere measurements."""
    sphere_id: Optional[str]
    center: Point
    radius_mm: float
    rms_error_mm: float
    max_abs_error_mm: float
    point_count: int
    residuals_mm: NDArray[np.float64] = field(default_factory=lambda: np.empty(0))
    residuals_by_sphere_id: dict[str, float] = field(default_factory=dict)
    residuals_by_sphere_set_id: dict[str, float] = field(default_factory=dict)

    def get_as_dict(self) -> dict:
        """Return a JSON-serializable representation of the fit."""
        return {
            "sphere_id": self.sphere_id,
            "center_mm": {
                "x": float(self.center.x),
                "y": float(self.center.y),
                "z": float(self.center.z),
            },
            "radius_mm": float(self.radius_mm),
            "rms_error_mm": float(self.rms_error_mm),
            "max_abs_error_mm": float(self.max_abs_error_mm),
            "point_count": int(self.point_count),
            "residuals_mm": [float(value) for value in self.residuals_mm],
            "residuals_by_sphere_id": {
                sphere_id: float(value)
                for sphere_id, value in self.residuals_by_sphere_id.items()
            },
            "residuals_by_sphere_set_id": {
                sphere_set_id: float(value)
                for sphere_set_id, value in self.residuals_by_sphere_set_id.items()
            },
        }


@dataclass
class SphereMeasurements:
    """Measurements for the spheres observed in one ``SphereSet``."""
    measurements: dict[str, SphereMeasurement] = field(default_factory=dict)

    def add_sphere_measurement(
        self,
        sphere_id: str,
        measurement: SphereMeasurement,
    ) -> None:
        self.measurements[sphere_id] = measurement

    def get_sphere_measurement(
        self,
        sphere_id: str,
    ) -> Optional[SphereMeasurement]:
        return self.measurements.get(sphere_id)

    def get_all_sphere_measurements(self) -> list[SphereMeasurement]:
        return list(self.measurements.values())

    @property
    def sphere_ids(self) -> list[str]:
        return list(self.measurements.keys())

    # ------------------------- compatibility with the previous API
    def add_frame_measurement(
        self,
        frame_id: str,
        measurement: SphereMeasurement,
    ) -> None:
        self.add_sphere_measurement(frame_id, measurement)

    def get_frame_measurement(
        self,
        frame_id: str,
    ) -> Optional[SphereMeasurement]:
        return self.get_sphere_measurement(frame_id)

    def get_all_frame_measurements(self) -> list[SphereMeasurement]:
        return self.get_all_sphere_measurements()

    @property
    def frame_ids(self) -> list[str]:
        return self.sphere_ids

    @property
    def frames(self) -> list[SphereMeasurement]:
        return self.get_all_sphere_measurements()


# Compatibility aliases for code written before the sphere-oriented rename.
FrameMeasurement = SphereMeasurement
BallMeasurements = SphereMeasurements
BallMeasurments = SphereMeasurements


@dataclass
class SphereSet:
    """One commanded configuration and the 9 sphere measurements for it."""

    sphere_set_id: str
    rx_cmd: float
    ry_cmd: float
    rz_cmd: float = 0.0
    x_cmd: float = 0.0
    y_cmd: float = 0.0
    angle: float = 0.0
    current_iteration: int = 0
    sphere_measurements: SphereMeasurements = field(
        default_factory=SphereMeasurements,
    )

    # Backwards-compat: ``pose_id`` / ``frame_measurments`` aliases.
    @property
    def pose_id(self) -> str:
        return self.sphere_set_id

    @property
    def frame_measurments(self) -> SphereMeasurements:
        return self.sphere_measurements

    @pose_id.setter
    def pose_id(self, value: str) -> None:
        self.sphere_set_id = value

    @frame_measurments.setter
    def frame_measurments(self, value: SphereMeasurements) -> None:
        self.sphere_measurements = value

    @property
    def sphere_set_transform(self) -> Optional[NDArray[np.float64]]:
        """4x4 commanded transform for this sphere set."""
        rx_rad = np.deg2rad(self.rx_cmd)
        ry_rad = np.deg2rad(self.ry_cmd)
        rz_rad = np.deg2rad(self.rz_cmd)

        Rx = np.array([[1, 0, 0],
                       [0, np.cos(rx_rad), -np.sin(rx_rad)],
                       [0, np.sin(rx_rad), np.cos(rx_rad)]])

        Ry = np.array([[np.cos(ry_rad), 0, np.sin(ry_rad)],
                       [0, 1, 0],
                       [-np.sin(ry_rad), 0, np.cos(ry_rad)]])

        Rz = np.array([[np.cos(rz_rad), -np.sin(rz_rad), 0],
                       [np.sin(rz_rad), np.cos(rz_rad), 0],
                       [0, 0, 1]])

        R = Rz @ Ry @ Rx
        t = np.array([self.x_cmd, self.y_cmd, 0.0])

        transform = np.eye(4)
        transform[:3, :3] = R
        transform[:3, 3] = t
        return transform

    @property
    def sphere_set_rotation_matrix(self) -> Optional[NDArray[np.float64]]:
        """3x3 commanded rotation matrix for this sphere set."""
        rx_rad = np.deg2rad(self.rx_cmd)
        ry_rad = np.deg2rad(self.ry_cmd)
        rz_rad = np.deg2rad(self.rz_cmd)

        Rx = np.array([[1, 0, 0],
                       [0, np.cos(rx_rad), -np.sin(rx_rad)],
                       [0, np.sin(rx_rad), np.cos(rx_rad)]])

        Ry = np.array([[np.cos(ry_rad), 0, np.sin(ry_rad)],
                       [0, 1, 0],
                       [-np.sin(ry_rad), 0, np.cos(ry_rad)]])

        Rz = np.array([[np.cos(rz_rad), -np.sin(rz_rad), 0],
                       [np.sin(rz_rad), np.cos(rz_cmd := rz_rad), 0],
                       [0, 0, 1]])

        return Rz @ Ry @ Rx

    @property
    def sphere_set_translation_vector(self) -> Optional[NDArray[np.float64]]:
        """3-vector translation component of the commanded transform."""
        return np.array([self.x_cmd, self.y_cmd, 0.0], dtype=np.float64)

    # Backwards-compat: previous property names.
    pose_transform = sphere_set_transform
    pose_rotation_matrix = sphere_set_rotation_matrix
    pose_translation_vector = sphere_set_translation_vector

    @classmethod
    def fit_pivot(
        cls,
        sphere_set: "SphereSet",
        radius_mm: Optional[float] = None,
    ) -> "SphereFitResult":
        """Fit **one** sphere with a known radius to the 9 measurements.

        The pivot of a sphere set is the centre of the calibration sphere
        in the calibration reference frame, recovered from the 9 surface
        points measured by the Smarpod at the commanded configuration.

        Parameters
        ----------
        sphere_set : SphereSet
            The commanded configuration holding the 9 measurements.
        radius_mm : float, optional
            Known sphere radius (mm).  Defaults to
            ``CALIBRATION_SPHERE_RADIUS_MM`` (diameter 6.35 mm).

        Returns
        -------
        SphereFitResult
            ``sphere_id`` is the sphere_set id; ``radius_mm`` is the
            supplied radius; the radial residuals are stored on each
            ``SphereMeasurement.sphere_fit_error_mm`` and indexed by
            ``sphere_id`` (the ball id) in
            ``residuals_by_sphere_id``.
        """
        from pm_robot_calibration.py_modules.hexapod_calibration.geometry_utils import (
            fit_sphere_fixed_radius,
        )

        if radius_mm is None:
            radius_mm = CALIBRATION_SPHERE_RADIUS_MM

        measurements = sphere_set.sphere_measurements.get_all_sphere_measurements()
        if len(measurements) < 3:
            raise ValueError(
                "At least 3 sphere measurements are required to fit a "
                f"sphere with a known radius; got {len(measurements)}"
            )

        points = np.asarray(
            [m.measurement_position.as_array() for m in measurements],
            dtype=np.float64,
        )
        center_array, rms_error, _ = fit_sphere_fixed_radius(points, radius_mm)
        center_array = np.asarray(center_array, dtype=np.float64).reshape(3)

        distances = np.linalg.norm(points - center_array, axis=1)
        residuals = np.asarray(distances - radius_mm, dtype=np.float64)
        for measurement, residual in zip(measurements, residuals):
            measurement.sphere_fit_error_mm = float(residual)

        return SphereFitResult(
            sphere_id=sphere_set.sphere_set_id,
            center=Point.from_array(center_array),
            radius_mm=float(radius_mm),
            rms_error_mm=float(rms_error),
            max_abs_error_mm=float(np.max(np.abs(residuals))),
            point_count=len(measurements),
            residuals_mm=residuals,
            residuals_by_sphere_id={
                measurement.sphere_id: float(residual)
                for measurement, residual in zip(measurements, residuals)
            },
        )

    @classmethod
    def from_measurement_dict(
        cls,
        pose_data: dict,
        frame_id: str = "0",
        pose_id: Optional[str] = None,
    ) -> "SphereSet":
        """Backwards-compatible loader for the *old* per-pose schema."""
        sphere_measurements = SphereMeasurements()
        sphere_measurements.add_sphere_measurement(
            frame_id,
            SphereMeasurement.from_measurement_dict(pose_data),
        )

        rx = float(pose_data["rx_cmd"])
        ry = float(pose_data["ry_cmd"])
        x = float(pose_data.get("x_cmd", 0.0))
        y = float(pose_data.get("y_cmd", 0.0))
        rz = float(pose_data.get("rz_cmd", 0.0))
        angle = float(pose_data.get("angle", abs(rx) if rx != 0 else abs(ry)))
        current_iteration = int(pose_data.get("current_iteration", 0))

        if pose_id is None:
            pose_id = pose_data.get("pose_id") or f"x{x}_y{y}_rx{rx}_ry{ry}"

        return cls(
            sphere_set_id=pose_id,
            rx_cmd=rx,
            ry_cmd=ry,
            rz_cmd=rz,
            x_cmd=x,
            y_cmd=y,
            angle=angle,
            current_iteration=current_iteration,
            sphere_measurements=sphere_measurements,
        )

    @classmethod
    def from_pose_dict(
        cls,
        pose_data: dict,
        pose_id: Optional[str] = None,
    ) -> "SphereSet":
        """Build a ``SphereSet`` from a *new-format* per-sphere-set dict."""
        results_list = pose_data.get("results_list", []) or []
        sphere_measurements = SphereMeasurements()
        for entry in results_list:
            if not isinstance(entry, dict):
                continue
            for sphere_id, sphere_data in entry.items():
                if isinstance(sphere_data, dict):
                    sphere_measurements.add_sphere_measurement(
                        sphere_id,
                        SphereMeasurement.from_sphere_dict(sphere_id, sphere_data),
                    )

        rx = float(pose_data.get("rx_cmd", 0.0))
        ry = float(pose_data.get("ry_cmd", 0.0))
        rz = float(pose_data.get("rz_cmd", 0.0))
        x = float(pose_data.get("x_cmd", 0.0))
        y = float(pose_data.get("y_cmd", 0.0))
        angle = float(pose_data.get("angle", abs(rx) if rx != 0 else abs(ry)))
        current_iteration = int(pose_data.get("current_iteration", 0))

        if pose_id is None:
            pose_id = pose_data.get("pose_id") or f"x{x}_y{y}_rx{rx}_ry{ry}"

        return cls(
            sphere_set_id=pose_id,
            rx_cmd=rx,
            ry_cmd=ry,
            rz_cmd=rz,
            x_cmd=x,
            y_cmd=y,
            angle=angle,
            current_iteration=current_iteration,
            sphere_measurements=sphere_measurements,
        )


