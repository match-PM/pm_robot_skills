"""
Free helper functions used across the calibration data classes:
angle conversions between 3D normals and (tilt_x, tilt_y, angle_from_z),
2D line fits (PCA), line-intersection combinators, and small
geometric utilities.
"""
import numpy as np
from numpy.typing import NDArray
from itertools import combinations
from typing import Optional, Iterable, Mapping, Hashable


# ---------------------------------------------------------------- angle utils
def normal_to_angles(
    n: NDArray[np.float64],
    in_deg: bool = True,
) -> tuple[float, float, float]:
    """
    Convert a 3D normal to ``(tilt_x, tilt_y, angle_from_z)``.

    The input is normalized internally.  The returned values are in
    degrees by default, matching the calibration analysis conventions.
    """
    n = np.asarray(n, dtype=np.float64).reshape(3)
    norm = np.linalg.norm(n)
    if norm == 0.0:
        raise ValueError("A zero vector has no normal direction")
    n = n / norm

    tilt_x = np.arctan2(n[1], n[2])
    tilt_y = np.arctan2(n[0], n[2])
    angle_from_z = np.arccos(np.clip(n[2], -1.0, 1.0))

    if in_deg:
        return (
            float(np.degrees(tilt_x)),
            float(np.degrees(tilt_y)),
            float(np.degrees(angle_from_z)),
        )
    return float(tilt_x), float(tilt_y), float(angle_from_z)


def rotation_matrix_to_euler_zyx(R: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Convert a 3D rotation matrix to ZYX intrinsic Euler angles (yaw, pitch, roll).

    The rotation is decomposed as R = R_z(yaw) @ R_y(pitch) @ R_x(roll)
    where rotations are applied in the order: first roll (X), then pitch (Y), then yaw (Z).

    Returns:
        np.array shape (3,) with [yaw, pitch, roll] in radians
    """
    # Check for gimbal lock (pitch = +/- 90 degrees)
    if abs(R[2, 0]) < 1.0 - 1e-10:
        # Normal case
        yaw = np.arctan2(R[1, 0], R[0, 0])
        pitch = np.arcsin(-R[2, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])
    else:
        # Gimbal lock case
        # When pitch = +90 deg, R[2,0] = -1
        # When pitch = -90 deg, R[2,0] = +1
        if R[2, 0] < 0:
            # pitch = +90 deg
            pitch = np.pi / 2
            yaw = np.arctan2(-R[0, 1], R[1, 1])
            roll = 0.0
        else:
            # pitch = -90 deg
            pitch = -np.pi / 2
            yaw = np.arctan2(R[0, 1], -R[1, 1])
            roll = 0.0

    return np.array([yaw, pitch, roll], dtype=np.float64)


def fit_sphere(points):
    """
    Fits a sphere to a set of 3D points.

    Parameters
    ----------
    points : (N,3) array_like
        List or array of [x, y, z] coordinates.

    Returns
    -------
    center : ndarray, shape (3,)
        Estimated sphere center.
    radius : float
        Estimated sphere radius.
    rms_error : float
        RMS fitting error.
    """
    points = np.asarray(points)

    if points.shape[1] != 3:
        raise ValueError("Points must have shape (N,3).")

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # Linear least-squares system
    A = np.column_stack((2*x, 2*y, 2*z, np.ones(len(points))))
    b = x**2 + y**2 + z**2

    coeffs, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

    cx, cy, cz = coeffs[:3]
    c = coeffs[3]

    radius = np.sqrt(c + cx**2 + cy**2 + cz**2)

    center = np.array([cx, cy, cz])

    # Compute RMS residual
    distances = np.linalg.norm(points - center, axis=1)
    rms_error = np.sqrt(np.mean((distances - radius) ** 2))

    return center, radius, rms_error


def fit_sphere_fixed_radius(
    points,
    radius: float,
    max_iterations: int = 200,
    tolerance_mm: float = 1e-9,
):
    """
    Fit a sphere with a known radius to a set of 3D points.

    The returned center is the point that minimises the sum of squared
    radial residuals ``(||p_i - c|| - radius)`` using a Gauss-Newton
    iteration seeded by the algebraic ``fit_sphere`` solution.

    This is the fit you want when the sphere geometry is known (e.g. a
    calibration sphere with diameter 6.35 mm mounted on the robot, and
    you want to recover its center from the 9 measured surface points).

    Parameters
    ----------
    points : (N, 3) array_like
        Points on the sphere surface (mm).
    radius : float
        Known sphere radius (mm).
    max_iterations : int
        Maximum number of Gauss-Newton iterations.
    tolerance_mm : float
        Stop iterating when the centre moves by less than this (mm).

    Returns
    -------
    center : ndarray, shape (3,)
        Fitted sphere center (mm).
    rms_error : float
        RMS of the radial residuals (mm).
    iterations : int
        Number of iterations actually performed.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if points.shape[0] < 3:
        raise ValueError(
            "At least 3 non-collinear points are required to fit a "
            f"sphere with a known radius; got {points.shape[0]}"
        )
    if radius <= 0:
        raise ValueError(f"radius must be positive; got {radius}")

    # Seed the iteration with the algebraic fit, then enforce the
    # known radius by snapping the centre to the desired distance from
    # the centroid of the points.
    center_array, _, _ = fit_sphere(points)
    center_array = np.asarray(center_array, dtype=np.float64).reshape(3)

    # Snap seed to the constraint surface.
    direction = center_array - points.mean(axis=0)
    distance = np.linalg.norm(direction)
    if distance > 1e-12:
        center_array = points.mean(axis=0) + radius * direction / distance

    iterations = 0
    for iterations in range(1, max_iterations + 1):
        diffs = points - center_array
        distances = np.linalg.norm(diffs, axis=1)
        # Skip rows where the centre coincides with a point (division by
        # zero).  In practice the seed keeps us well away from that.
        with np.errstate(divide="ignore", invalid="ignore"):
            unit = diffs / distances[:, None]
        residuals = distances - radius  # signed radial residuals

        # Gauss-Newton normal equations: J = -unit, so
        # (J^T J) delta = -J^T residuals
        J = -unit
        JT = J.T
        JtJ = JT @ J
        Jtr = JT @ residuals
        try:
            delta = np.linalg.solve(JtJ, Jtr)
        except np.linalg.LinAlgError:
            # Fall back to least squares if the system is singular.
            delta, _, _, _ = np.linalg.lstsq(JtJ, Jtr, rcond=None)

        new_center = center_array - delta
        step = float(np.linalg.norm(new_center - center_array))
        center_array = new_center
        if step < tolerance_mm:
            break

    distances = np.linalg.norm(points - center_array, axis=1)
    residuals = distances - radius
    rms_error = float(np.sqrt(np.mean(residuals ** 2)))
    return center_array, rms_error, iterations