
import numpy as np
import matplotlib.pyplot as plt
import json
from numpy.typing import NDArray
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import matplotlib.pyplot as plt
from numpy.typing import NDArray
from itertools import combinations
import os




def normal_to_angles(n: NDArray[np.float64]) -> tuple[float, float, float]:
    n = n / np.linalg.norm(n)

    tilt_x = np.rad2deg(np.arctan2(n[1], n[2]))
    tilt_y = np.rad2deg(np.arctan2(n[0], n[2]))
    angle_z = np.rad2deg(np.arccos(np.clip(np.dot(n, [0, 0, 1]), -1, 1)))

    return tilt_x, tilt_y, angle_z

# def fit_line(x: NDArray[np.float64], y: NDArray[np.float64]) -> tuple[float, float]:
#     A = np.vstack([x, np.ones(len(x))]).T
#     m, c = np.linalg.lstsq(A, y, rcond=None)[0]
#     return m, c

def fit_line(x: NDArray[np.float64], y: NDArray[np.float64]):
    pts = np.column_stack([x, y])

    centroid = pts.mean(axis=0)
    centered = pts - centroid

    # PCA (total least squares)
    _, _, vh = np.linalg.svd(centered)

    direction = vh[0]  # (dx, dy), normalized direction
    direction = canonical_direction(direction)

    return centroid, direction

def intersect(p1, d1, p2, d2):
    A = np.column_stack([d1, -d2])
    b = p2 - p1

    t, _ = np.linalg.lstsq(A, b, rcond=None)[0]

    return p1 + t * d1

def direction_angle(direction: np.ndarray) -> float:
    dx, dy = direction
    return np.rad2deg(np.arctan2(dy, dx))

def plot_line(center, direction, t_range=50):
    t = np.linspace(-t_range, t_range, 100)
    pts = center + t[:, None] * direction

    plt.plot(pts[:, 0], pts[:, 1], linewidth=2)

def rotate(points, angle_deg):
    theta = np.deg2rad(angle_deg)

    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

    return points @ R.T

def apply_correction(tilt_x, tilt_y, center, angle_deg):
    pts = np.column_stack([tilt_x, tilt_y])

    # 1. shift to center
    pts_centered = pts - np.array([center.x, center.y])

    # 2. rotate (inverse correction!)
    theta = np.deg2rad(angle_deg)

    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

    corrected = pts_centered @ R.T

    return corrected[:, 0], corrected[:, 1]

def canonical_direction(d: np.ndarray) -> np.ndarray:
    d = d / np.linalg.norm(d)

    # force x >= 0 (or y >= 0 if x is small)
    if d[0] < 0 or (abs(d[0]) < 1e-12 and d[1] < 0):
        d = -d

    return d


def line_angle(direction):
    dx, dy = direction
    angle = np.rad2deg(np.arctan2(dy, dx)) % 180
    return angle

@dataclass
class Point:
    x: float
    y: float
    z: float

    @classmethod
    def from_um(cls, d):
        return cls(
            0.001 * d["x"],
            0.001 * d["y"],
            0.001 * d["z"]
        )

    def as_array(self) -> NDArray[np.float64]:
        return np.array([self.x, self.y, self.z], dtype=np.float64)

@dataclass
class RotationCenterResult:
    center: NDArray[np.float64]
    residual_mm: float

@dataclass
class FrameMeasurement:
    frame_id: str
    correction_z_mm: float
    position: Point


@dataclass
class Plane:
    center: NDArray[np.float64]
    normal: NDArray[np.float64]

    tilt_x_deg: float
    tilt_y_deg: float
    angle_from_z_deg: float

    residual_mm: Optional[float] = None
    rotation_center_error_mm: Optional[float] = None

    @classmethod
    def fit(cls, points: NDArray[np.float64]) -> "Plane":

        center = points.mean(axis=0)

        _, _, vh = np.linalg.svd(points - center)
        normal = vh[-1]
        normal /= np.linalg.norm(normal)

        if normal[2] < 0:
            normal = -normal

        tilt_x, tilt_y, angle_z = normal_to_angles(normal)

        return cls(
            center=center,
            normal=normal,
            tilt_x_deg=tilt_x,
            tilt_y_deg=tilt_y,
            angle_from_z_deg=angle_z
        )

    def compute_center_error(self, global_center: np.ndarray):
        if self.center is None or self.normal is None:
            return None

        self.rotation_center_error_mm = float(
            np.dot(self.normal, global_center - self.center)
        )
        return self.rotation_center_error_mm

@dataclass
class Pose:
    pose_id: str
    rx_cmd: float
    ry_cmd: float

    frames: dict[str, FrameMeasurement]

    plane: Optional[Plane] = None

    @property
    def points(self) -> NDArray[np.float64]:
        return np.array(
            [frame.position.as_array() for frame in self.frames.values()],
            dtype=np.float64
        )

    def fit_plane(self):
        self.plane = Plane.fit(self.points)

    def set_residual(self, residual_mm: float):
        if self.plane is None:
            raise RuntimeError("Plane has not been fitted")

        self.plane.residual_mm = residual_mm

@dataclass
class MeasurementSetResults:
    rotation_center_mm: Point
    rotation_center_avg_residual_mm: float
    average_slope_deviation_deg: float
    rotation_center_error_deg: Point
    rotation_center_max_error_mm: float

    def get_as_dict(self):
        return {
            "rotation_center_mm": {
                "x": self.rotation_center_mm.x,
                "y": self.rotation_center_mm.y,
                "z": self.rotation_center_mm.z
            },
            "rotation_center_avg_residual_mm": self.rotation_center_avg_residual_mm,
            "rotation_center_max_error_mm": self.rotation_center_max_error_mm,
            "average_slope_deviation_deg": self.average_slope_deviation_deg,
            "rotation_center_error_deg": {
                "x": self.rotation_center_error_deg.x,
                "y": self.rotation_center_error_deg.y,
                "z": self.rotation_center_error_deg.z
            }
        }

@dataclass
class MeasurementSet:
    poses: list[Pose]
    filename: Optional[str] = None

    @classmethod
    def load(cls, filename: str) -> "MeasurementSet":

        with open(filename) as f:
            raw = json.load(f)

        poses = []
        
        for pose_data in raw:

            frames = {}

            for frame_id, frame in pose_data["frames"].items():

                frames[frame_id] = FrameMeasurement(
                    frame_id=frame_id,
                    correction_z_mm=0.001 * frame["correction_z_um"],

                    # position=Point.from_um(
                    # frame["transform_rot"]["translation"]
                    # )

                    # position=Point.from_um(
                    # frame["transform_to_initial"]["translation"]
                    # )

                    position=Point.from_um(
                    frame["transform_fixed"]["translation"]
                    )

                )

            poses.append(
                Pose(
                    pose_id=pose_data["pose_id"],
                    rx_cmd=pose_data["rx_cmd"],
                    ry_cmd=pose_data["ry_cmd"],
                    frames=frames
                )
            )
        
        base_name = os.path.splitext(os.path.basename(filename))[0]

        return cls(poses=poses, filename=base_name)

    def fit_planes(self):
        for pose in self.poses:
            pose.fit_plane()

    @property
    def normals(self):
        return [p.plane.normal for p in self.poses if p.plane is not None]

    @property
    def centers(self):
        return [p.plane.center for p in self.poses if p.plane is not None]

    @property
    def residuals(self):
        return [p.plane.residual_mm for p in self.poses if p.plane is not None]

    @property
    def equal_angles(self):
        angles = []
        for p in self.poses:
            if p.rx_cmd == p.ry_cmd and p.plane is not None:
                angles.append((p.plane.tilt_x_deg, p.plane.tilt_y_deg))
        return angles

    @property
    def anti_equal_angles(self):
        angles = []
        for p in self.poses:
            if p.rx_cmd == -p.ry_cmd and p.plane is not None:
                angles.append((p.plane.tilt_x_deg, p.plane.tilt_y_deg))
        return angles

    @property
    def x_angles(self):
        angles = []
        for p in self.poses:
            if p.ry_cmd == 0 and p.plane is not None:
                angles.append((p.plane.tilt_x_deg, p.plane.tilt_y_deg))
        return angles

    @property
    def y_angles(self):
        angles = []
        for p in self.poses:
            if p.rx_cmd == 0 and p.plane is not None:
                angles.append((p.plane.tilt_x_deg, p.plane.tilt_y_deg))
        return angles

    @property
    def angle_bounds(self):
        xs = []
        ys = []

        for p in self.poses:
            if p.plane is None:
                continue
            xs.append(p.plane.tilt_x_deg)
            ys.append(p.plane.tilt_y_deg)

        return min(xs), max(xs), min(ys), max(ys)

    def fit_line_equal_angles(self):
        angles = self.equal_angles
        if len(angles) < 2:
            return None

        x = np.array([a[0] for a in angles])
        y = np.array([a[1] for a in angles])

        return fit_line(x, y)


    def fit_line_anti_equal_angles(self):
        angles = self.anti_equal_angles
        if len(angles) < 2:
            return None

        x = np.array([a[0] for a in angles])
        y = np.array([a[1] for a in angles])

        return fit_line(x, y)


    def fit_line_x_angles(self):
        angles = self.x_angles
        if len(angles) < 2:
            return None

        x = np.array([a[0] for a in angles])
        y = np.array([a[1] for a in angles])

        return fit_line(x, y)

    def fit_line_y_angles(self):
        angles = self.y_angles
        if len(angles) < 2:
            return None

        x = np.array([a[0] for a in angles])
        y = np.array([a[1] for a in angles])

        return fit_line(x, y)

    def get_line_slope_deviation(self):

        def angle_of_fit(func):
            res = func()
            if res is None:
                return None
            _, direction = res
            return line_angle(direction)

        def angle_diff(a, b):
            return (b - a + 90) % 180 - 90

        a_equal = angle_of_fit(self.fit_line_equal_angles)
        a_anti = angle_of_fit(self.fit_line_anti_equal_angles)
        a_x = angle_of_fit(self.fit_line_x_angles)
        a_y = angle_of_fit(self.fit_line_y_angles)

        if None in (a_equal, a_anti, a_x, a_y):
            return None

        # expected directions (degrees)
        expected = {
            "equal": 135,
            "anti_equal": 45,
            "x": 0,
            "y": 90
        }

        deviations = [
            angle_diff(a_equal, expected["equal"]),
            angle_diff(a_anti, expected["anti_equal"]),
            angle_diff(a_x, expected["x"]),
            angle_diff(a_y, expected["y"])
        ]

        return deviations

    def get_average_slope_deviation(self):
        deviations = self.get_line_slope_deviation()

        if deviations is None:
            return None

        return np.mean(deviations)

    def calculate_line_intersections(self):
        lines = {
            "equal": self.fit_line_equal_angles(),
            "anti_equal": self.fit_line_anti_equal_angles(),
            "x_angles": self.fit_line_x_angles(),
            "y_angles": self.fit_line_y_angles()
        }

        intersections = {}

        for (name1, line1), (name2, line2) in combinations(lines.items(), 2):
            if line1 is None or line2 is None:
                continue

            (p1, d1) = line1
            (p2, d2) = line2

            # skip near-parallel lines
            cross = np.cross(np.append(d1, 0), np.append(d2, 0))
            if np.linalg.norm(cross) < 1e-8:
                continue

            pt = intersect(p1, d1, p2, d2)

            intersections[f"{name1} & {name2}"] = (pt[0], pt[1])

        return intersections

    def calculate_center_from_intersections(self) -> Optional[Point]:
        intersections = self.calculate_line_intersections()

        if not intersections:
            return None

        pts = np.array(list(intersections.values()))

        center = pts.mean(axis=0)

        return Point(center[0], center[1], 0)

    def plot_tilt_xy(self, image_path: Optional[str] = None):
        self.fit_planes()

        planes = [p.plane for p in self.poses if p.plane is not None]

        tilt_x = np.array([p.tilt_x_deg for p in planes])
        tilt_y = np.array([p.tilt_y_deg for p in planes])

        plt.figure(figsize=(8, 8))

        # scatter points
        plt.scatter(tilt_x, tilt_y, color='blue')

        for i, (tx, ty) in enumerate(zip(tilt_x, tilt_y)):
            plt.text(tx, ty, str(i), fontsize=8)

        # ----- plot lines (PCA form) -----
        for func, label in [
            (self.fit_line_equal_angles, "equal"),
            (self.fit_line_anti_equal_angles, "anti-equal"),
            (self.fit_line_x_angles, "x"),
            (self.fit_line_y_angles, "y")
        ]:
            res = func()
            if res is None:
                continue

            center, direction = res
            plot_line(center, direction)

        # ----- intersections -----
        intersections = self.calculate_line_intersections()

        for name, (x, y) in intersections.items():
            plt.scatter(x, y, color='green', s=80)
            plt.text(x, y, name, fontsize=8)

        # ----- center -----
        center = self.calculate_center_from_intersections()

        if center is not None:
            plt.scatter(center.x, center.y,
                        color='red',
                        marker='x',
                        s=200,
                        label='center')

            plt.text(center.x, center.y, "CENTER", color='red')

        x_min, x_max, y_min, y_max = self.angle_bounds

        x_pad = 0.05 * (x_max - x_min)
        y_pad = 0.05 * (y_max - y_min)

        plt.xlim(x_min - x_pad, x_max + x_pad)
        plt.ylim(y_min - y_pad, y_max + y_pad)

        plt.grid(True)
        plt.legend()

        image_name = f"{self.filename}_tilt_xy.png"

        if image_path is not None:
            filename = os.path.join(image_path, "results", image_name)
        else:
            current_path = os.getcwd()
            results_path = os.path.join(current_path, "results")
            filename = os.path.join(results_path, image_name)
        print(f"Saving rotation center error map to: {filename}")
        plt.savefig(filename, dpi=300, bbox_inches="tight")

        plt.close()

        plt.show()


    def plot_corrected_tilt_xy(self, image_path: Optional[str] = None):
        self.fit_planes()

        planes = [p.plane for p in self.poses if p.plane is not None]

        tilt_x = np.array([p.tilt_x_deg for p in planes])
        tilt_y = np.array([p.tilt_y_deg for p in planes])

        center = self.calculate_center_from_intersections()
        if center is None:
            return

        # average correction angle
        angle_correction = self.get_average_slope_deviation()

        x_corr, y_corr = apply_correction(
            tilt_x, tilt_y,
            center,
            angle_correction
        )

        plt.figure(figsize=(8, 8))

        plt.scatter(x_corr, y_corr, color="blue")

        for i, (x, y) in enumerate(zip(x_corr, y_corr)):
            plt.text(x, y, str(i), fontsize=8)

        plt.axhline(0, color="black", linewidth=1)
        plt.axvline(0, color="black", linewidth=1)

        plt.title("Corrected Tilt XY (rotated + centered)")
        plt.grid(True)
        plt.axis("equal")

        x_min, x_max = plt.xlim()
        x_vals = np.linspace(x_min, x_max, 200)

        # +45° line (y = x)
        plt.plot(x_vals, x_vals, '--', color='gray', linewidth=1, label='+45°')

        # -45° line (y = -x)
        plt.plot(x_vals, -x_vals, '--', color='gray', linewidth=1, label='-45°')

        image_name = f"{self.filename}_corrected_tilt_xy.png"

        if image_path is not None:
            filename = os.path.join(image_path, "results", image_name)
        else:
            current_path = os.getcwd()
            results_path = os.path.join(current_path, "results")
            filename = os.path.join(results_path, image_name)

        print(f"Saving rotation center error map to: {filename}")
        plt.savefig(filename, dpi=300, bbox_inches="tight")

        plt.close()

        plt.show()


    def estimate_rotation_center(self) -> RotationCenterResult:
        self.fit_planes()
        centers= np.array(self.centers)
        normals = np.array(self.normals)

        points = np.asarray(centers, dtype=np.float64)
        normals = np.asarray(normals, dtype=np.float64)

        normals = np.array([n / np.linalg.norm(n) for n in normals])

        A = normals  # (N,3)
        b = np.array([np.dot(n, p) for n, p in zip(normals, points)])

        # solve: n_i · c = d_i
        center, *_ = np.linalg.lstsq(A, b, rcond=None)

        # residual (distance error to each plane)
        errors = np.array([
        np.dot(n, center - p)
        for p,n in zip(points,normals)
        ])

        residual = np.sqrt(np.mean(errors**2))

        for pose in self.poses:
            if pose.plane is None:
                continue
            pose.plane.compute_center_error(center)

        return RotationCenterResult(center=center, residual_mm=residual)

    def plot_rotation_center_error(self, image_path: Optional[str] = None):
        self.fit_planes()

        result = self.estimate_rotation_center()
        
        planes = [p.plane for p in self.poses if p.plane is not None]

        xs = np.array([p.tilt_x_deg for p in planes])
        ys = np.array([p.tilt_y_deg for p in planes])

        errors = np.array([
            p.rotation_center_error_mm if p.rotation_center_error_mm is not None else 0.0
            for p in planes
        ])

        plt.figure(figsize=(8, 8))

        # ---- scatter colored by error ----
        sc = plt.scatter(
            xs,
            ys,
            c=errors,
            cmap="coolwarm",
            s=80,
            edgecolor="black"
        )

        # labels
        for i, (x, y, e) in enumerate(zip(xs, ys, errors)):
            plt.text(x, y, str(i), fontsize=8)


        # ---- axes ----
        plt.axhline(0, color="gray", linewidth=1)
        plt.axvline(0, color="gray", linewidth=1)

        # ---- colorbar ----
        cbar = plt.colorbar(sc)
        cbar.set_label("rotation center error (mm)")

        plt.title("Rotation Center Error Map (Tilt Space)")
        plt.xlabel("tilt_x (deg)")
        plt.ylabel("tilt_y (deg)")
        plt.axis("equal")
        plt.grid(True)

        plt.legend()

        image_name = f"{self.filename}_rotation_center_error_map.png"

        if image_path is not None:
            filename = os.path.join(image_path, "results", image_name)
        else:
            current_path = os.getcwd()
            results_path = os.path.join(current_path, "results")
            filename = os.path.join(results_path, image_name)

        print(f"Saving rotation center error map to: {filename}")
        plt.savefig(filename, dpi=300, bbox_inches="tight")

        plt.close()

        plt.show()

    def get_measurement_results(self, plot_images = False, results_path: str = None)->MeasurementSetResults:
        
        if plot_images:
            self.plot_rotation_center_error(results_path)
            self.plot_tilt_xy(results_path)
            self.plot_corrected_tilt_xy(results_path)

        results_path = results_path if results_path is not None else os.path.join(os.getcwd(), "results")

        results_file_name = f"{self.filename}_results.json"
        results_file_path = os.path.join(results_path, results_file_name)

        self.fit_planes()
        result = self.estimate_rotation_center()
        avg_slope_dev = self.get_average_slope_deviation()
        rotation_center = self.calculate_center_from_intersections()

        if result.center is not None:
            center_point = Point(result.center[0], result.center[1], result.center[2])
        else:
            center_point = Point(0, 0, 0)

        if rotation_center is not None:
            rotation_center_point = Point(rotation_center.x, rotation_center.y, rotation_center.z)
        else:
            rotation_center_point = Point(0, 0, 0)

        planes = [p.plane for p in self.poses if p.plane is not None]

        max_error = max((p.rotation_center_error_mm for p in planes),
                default=0.0)

        results = MeasurementSetResults(
            rotation_center_mm=center_point,
            rotation_center_avg_residual_mm=result.residual_mm,
            average_slope_deviation_deg=avg_slope_dev,
            rotation_center_error_deg=rotation_center_point,
            rotation_center_max_error_mm=max_error
        )

        # save results to JSON
        with open(results_file_path, 'w') as f:
            json.dump(results.get_as_dict(), f, indent=4)

        return results


        
    # def plot_normals_and_rotation_center(self,
    #                                     scale=20.0):

    #     self.fit_planes()

    #     result = self.estimate_rotation_center()
    #     rot_center = result.center

    #     planes = [p.plane for p in self.poses if p.plane is not None]

    #     fig = plt.figure(figsize=(10, 8))
    #     ax = fig.add_subplot(111, projection="3d")

    #     # ----------------------------
    #     # Plane centers
    #     # ----------------------------
    #     centers = np.array([p.center for p in planes])

    #     ax.scatter(
    #         centers[:, 0],
    #         centers[:, 1],
    #         centers[:, 2],
    #         s=40,
    #         label="Plane Centers"
    #     )

    #     # ----------------------------
    #     # Normals
    #     # ----------------------------
    #     for i, plane in enumerate(planes):

    #         c = plane.center
    #         n = plane.normal / np.linalg.norm(plane.normal)



    #         t = np.linspace(-scale, scale, 100)

    #         pts = c + t[:, None] * n

    #         ax.plot(
    #             pts[:, 0],
    #             pts[:, 1],
    #             pts[:, 2],
    #             linewidth=1,
    #             alpha=0.5
    #         )

    #     # ----------------------------
    #     # Rotation center
    #     # ----------------------------
    #     ax.scatter(
    #         rot_center[0],
    #         rot_center[1],
    #         rot_center[2],
    #         s=250,
    #         marker="*",
    #         label="Rotation Center"
    #     )

    #     ax.text(
    #         rot_center[0],
    #         rot_center[1],
    #         rot_center[2],
    #         "CENTER",
    #         fontsize=12
    #     )

    #     # ----------------------------
    #     # Labels
    #     # ----------------------------
    #     ax.set_xlabel("X [mm]")
    #     ax.set_ylabel("Y [mm]")
    #     ax.set_zlabel("Z [mm]")

    #     ax.set_title(
    #         f"Plane Normals and Estimated Rotation Center\n"
    #         f"Residual = {result.residual_mm:.4f} mm"
    #     )

    #     ax.legend()

    #     plt.tight_layout()
    #     plt.show()

    
    def plot_normals_and_rotation_center(self, normal_length=300):

        self.fit_planes()

        result = self.estimate_rotation_center()
        rot_center = result.center

        planes = [p.plane for p in self.poses if p.plane is not None]

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection="3d")

        # --------------------------------------------------
        # Plot rotation center
        # --------------------------------------------------
        ax.scatter(
            rot_center[0],
            rot_center[1],
            rot_center[2],
            color="red",
            marker="*",
            s=300,
            label="Estimated Rotation Center"
        )

        ax.text(
            rot_center[0],
            rot_center[1],
            rot_center[2],
            "CENTER",
            color="red"
        )

        # --------------------------------------------------
        # Plot plane centers and normals
        # --------------------------------------------------
        for i, plane in enumerate(planes):

            p = plane.center
            n = plane.normal / np.linalg.norm(plane.normal)

            # plane center
            ax.scatter(
                p[0],
                p[1],
                p[2],
                color="blue",
                s=40
            )

            ax.text(
                p[0],
                p[1],
                p[2],
                str(i),
                fontsize=8
            )

            # ------------------------------------------
            # Full normal line
            # ------------------------------------------
            t = np.linspace(-normal_length, normal_length, 200)

            pts = p + t[:, None] * n

            ax.plot(
                pts[:, 0],
                pts[:, 1],
                pts[:, 2],
                color="black",
                alpha=0.6
            )

            # ------------------------------------------
            # Line from plane center to fitted center
            # ------------------------------------------
            ax.plot(
                [p[0], rot_center[0]],
                [p[1], rot_center[1]],
                [p[2], rot_center[2]],
                color="green",
                linewidth=2
            )

            # ------------------------------------------
            # Diagnostics
            # ------------------------------------------
            v = rot_center - p

            dist_to_line = np.linalg.norm(
                np.cross(v, n)
            )

            angle_deg = np.rad2deg(
                np.arccos(
                    np.clip(
                        np.abs(np.dot(
                            v / np.linalg.norm(v),
                            n
                        )),
                        -1,
                        1
                    )
                )
            )

            print(
                f"Plane {i:2d}: "
                f"distance={dist_to_line:8.4f} mm, "
                f"angle={angle_deg:7.4f} deg"
            )

        # --------------------------------------------------
        # Equal scaling
        # --------------------------------------------------
        all_points = np.vstack([
            np.array([p.center for p in planes]),
            rot_center.reshape(1, 3)
        ])

        mins = all_points.min(axis=0)
        maxs = all_points.max(axis=0)

        center = (mins + maxs) / 2
        radius = np.max(maxs - mins) / 2

        ax.set_xlim(center[0] - radius, center[0] + radius)
        ax.set_ylim(center[1] - radius, center[1] + radius)
        ax.set_zlim(center[2] - radius, center[2] + radius)

        ax.set_xlabel("X [mm]")
        ax.set_ylabel("Y [mm]")
        ax.set_zlabel("Z [mm]")

        ax.set_title(
            f"Normals vs Estimated Rotation Center\n"
            f"Residual = {result.residual_mm:.4f} mm"
        )

        ax.legend()

        plt.tight_layout()
        plt.show()


    def get_corrected_tilts(self):
        """
        Returns corrected tilt measurements and corresponding commanded tilts.

        ```
        Returns
        -------
        x_corr : np.ndarray
        y_corr : np.ndarray
        rx_cmd : np.ndarray
        ry_cmd : np.ndarray
        """

        self.fit_planes()

        planes = [p.plane for p in self.poses if p.plane is not None]

        tilt_x = np.array([p.tilt_x_deg for p in planes])
        tilt_y = np.array([p.tilt_y_deg for p in planes])

        center = self.calculate_center_from_intersections()
        if center is None:
            raise RuntimeError("Could not determine correction center")

        angle_correction = self.get_average_slope_deviation()

        x_corr, y_corr = apply_correction(
            tilt_x,
            tilt_y,
            center,
            angle_correction
        )

        rx_cmd = np.array([p.rx_cmd for p in self.poses])
        ry_cmd = np.array([p.ry_cmd for p in self.poses])

        return x_corr, y_corr, rx_cmd, ry_cmd

    def get_tilt_errors(self):
        """
        Compare corrected tilts to commanded tilts.

        ```
        Returns
        -------
        dict
        """

        x_corr, y_corr, rx_cmd, ry_cmd = self.get_corrected_tilts()

        # change the sign depending on the ref cs
        err_x = rx_cmd - x_corr
        err_y = ry_cmd + y_corr

        for i in range(len(x_corr)):
            print("x")
            print(f"{rx_cmd[i]:+7.3f} - {x_corr[i]:+7.3f} = {err_x[i]:+7.3f}")
            print("y")
            print(f"{ry_cmd[i]:+7.3f} + {y_corr[i]:+7.3f} = {err_y[i]:+7.3f}")


        err_total = np.sqrt(err_x**2 + err_y**2)


        return {
            "error_x": err_x,
            "error_y": err_y,
            "error_total": err_total,

            "mean_error_x": float(np.mean(err_x)),
            "mean_error_y": float(np.mean(err_y)),

            "mean_abs_error_x": float(np.mean(np.abs(err_x))),
            "mean_abs_error_y": float(np.mean(np.abs(err_y))),

            "rmse_x": float(np.sqrt(np.mean(err_x**2))),
            "rmse_y": float(np.sqrt(np.mean(err_y**2))),

            "mean_total_error": float(np.mean(err_total)),
            "max_total_error": float(np.max(err_total))
        }
