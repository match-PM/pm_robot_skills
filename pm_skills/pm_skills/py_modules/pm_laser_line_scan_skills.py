from __future__ import annotations

import asyncio
import csv
import math
import re
import threading
import time
from datetime import datetime
from html import escape
from pathlib import Path
from typing import TYPE_CHECKING, Literal, Mapping, Sequence

from geometry_msgs.msg import Pose, TransformStamped
from rclpy.action import CancelResponse, GoalResponse
from rclpy.time import Time

import assembly_manager_interfaces.srv as ami_srv
import pm_moveit_interfaces.srv as pm_moveit_srv
import pm_skills_interfaces.action as pm_skill_action
from assembly_scene_publisher.py_modules.tf_functions import get_transform_for_frame_in_world
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_skills.py_modules.PmRobotUtils import PmRobotUtils
from pm_skills.py_modules.pm_skill_domain import PmSkillDomain

if TYPE_CHECKING:
    from rclpy.action.server import ServerGoalHandle
    from pm_skills.pm_skills import PmSkills


class LineScanCancelled(Exception):
    """Raised when cancellation is requested between scan operations."""


class PmLineScanSkills(PmSkillDomain):
    """Run laser or confocal line scans and correct assembly frames.

    The skill plans five poses along a line before execution, samples the chosen
    distance sensor at a requested spacing, and moves the correction frame to the selected
    minimum or maximum. Distances used internally are metres unless a variable
    or action field explicitly ends in ``_mm``.
    """

    MAX_SAMPLES = 200
    COLLISION_CHECK_COUNT = 5
    REFINEMENT_INTERVAL_COUNT = 30
    REFINEMENT_SAMPLE_COUNT = REFINEMENT_INTERVAL_COUNT + 1
    MAX_REFINEMENT_LENGTH_UM = 200.0
    ROBOT_APPROACH_CLEARANCE_M = 0.04
    TF_WAIT_TIMEOUT_S = 10.0
    TF_RETRY_INTERVAL_S = 0.1

    ROBOT_TRANSLATION_JOINTS = (
        PmRobotUtils.X_Axis_JOINT_NAME,
        PmRobotUtils.Y_Axis_JOINT_NAME,
        PmRobotUtils.Z_Axis_JOINT_NAME,
    )
    SMARPOD_JOINTS = (
        'SP_X_Joint',
        'SP_Y_Joint',
        'SP_Z_Joint',
        'SP_A_Joint',
        'SP_B_Joint',
        'SP_C_Joint',
    )

    def __init__(self, node: 'PmSkills') -> None:
        """Initialize the line-scan skill and its hexapod planning client.

        Args:
            node: Shared ``PmSkills`` ROS node that owns utility objects,
                callback groups, TF state, and assembly-manager clients.

        Returns:
            None.
        """
        super().__init__(node)
        self._active_lock = threading.Lock()
        self._active = False
        self._current_start_frame: str = ''
        self._current_end_frame: str = ''
        self._move_smarpod_to_pose_client = node.create_client(
            pm_moveit_srv.MoveToPose,
            '/pm_moveit_server/move_smarpod_to_pose',
        )

    def goal_callback(
        self,
        goal_request: pm_skill_action.CorrectFrameLineScan.Goal,
    ) -> GoalResponse:
        """Validate and reserve an incoming line-scan goal.

        Args:
            goal_request: Requested assembly frames, sensor offset in mm,
                measurement spacing in mm, min/max mode, and optional
                micrometre refinement window.

        Returns:
            ``GoalResponse.ACCEPT`` for a valid goal when no scan is active;
            otherwise ``GoalResponse.REJECT``.
        """
        if (
            not goal_request.start_frame
            or not goal_request.end_frame
            or not math.isfinite(goal_request.step_size_mm)
            or goal_request.step_size_mm <= 0.0
            or not math.isfinite(goal_request.sensor_offset_mm)
            or (
                goal_request.use_min_max_refinement
                and (
                    not math.isfinite(goal_request.refinement_length_um)
                    or goal_request.refinement_length_um <= 0.0
                    or goal_request.refinement_length_um > self.MAX_REFINEMENT_LENGTH_UM
                )
            )
        ):
            self._logger.error('Rejecting invalid line-scan goal.')
            return GoalResponse.REJECT

        with self._active_lock:
            if self._active:
                self._logger.warning('Rejecting line scan because another scan is active.')
                return GoalResponse.REJECT
            self._active = True

        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle: ServerGoalHandle) -> CancelResponse:
        """Accept a cancellation request for an executing line scan.

        Args:
            _goal_handle: Goal whose cancellation was requested. The value is
                intentionally unused because all active line scans are
                cancellable.

        Returns:
            ``CancelResponse.ACCEPT``. Execution observes the request between
            every trajectory movement and sensor measurement.
        """
        return CancelResponse.ACCEPT

    @staticmethod
    def _position(transform: TransformStamped) -> tuple[float, float, float]:
        """Extract Cartesian translation from a stamped transform.

        Args:
            transform: ROS transform containing a translation in metres.

        Returns:
            ``(x, y, z)`` translation in metres.
        """
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )

    @staticmethod
    def _distance(point_1: Sequence[float], point_2: Sequence[float]) -> float:
        """Calculate the Euclidean distance between two Cartesian points.

        Args:
            point_1: First Cartesian point in metres.
            point_2: Second Cartesian point in metres.

        Returns:
            Euclidean distance in metres.
        """
        return math.sqrt(sum((b - a) ** 2 for a, b in zip(point_1, point_2)))

    @staticmethod
    def _interpolate_vector(
        start: Sequence[float],
        end: Sequence[float],
        fraction: float,
    ) -> tuple[float, ...]:
        """Linearly interpolate between equal-length numeric vectors.

        Args:
            start: Vector at fraction ``0.0``.
            end: Vector at fraction ``1.0``.
            fraction: Dimensionless interpolation fraction, normally in
                ``[0.0, 1.0]``.

        Returns:
            Interpolated vector as a tuple.
        """
        return tuple(a + fraction * (b - a) for a, b in zip(start, end))

    @staticmethod
    def _interpolate_pose(start_pose: Pose, end_pose: Pose, fraction: float) -> Pose:
        """Interpolate a Cartesian pose for collision checking.

        Translation is interpolated linearly. Orientation uses normalized
        quaternion interpolation with shortest-hemisphere selection.

        Args:
            start_pose: Pose at fraction ``0.0``.
            end_pose: Pose at fraction ``1.0``.
            fraction: Dimensionless interpolation fraction.

        Returns:
            Interpolated ROS pose.

        Raises:
            PmRobotError: If interpolation produces a zero-length quaternion.
        """
        pose = Pose()
        pose.position.x = start_pose.position.x + fraction * (end_pose.position.x - start_pose.position.x)
        pose.position.y = start_pose.position.y + fraction * (end_pose.position.y - start_pose.position.y)
        pose.position.z = start_pose.position.z + fraction * (end_pose.position.z - start_pose.position.z)

        start_q = (
            start_pose.orientation.x,
            start_pose.orientation.y,
            start_pose.orientation.z,
            start_pose.orientation.w,
        )
        end_q = (
            end_pose.orientation.x,
            end_pose.orientation.y,
            end_pose.orientation.z,
            end_pose.orientation.w,
        )
        if sum(a * b for a, b in zip(start_q, end_q)) < 0.0:
            end_q = tuple(-value for value in end_q)
        quaternion = tuple(a + fraction * (b - a) for a, b in zip(start_q, end_q))
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm == 0.0:
            raise PmRobotError('Cannot interpolate an invalid zero-length quaternion.')
        quaternion = tuple(value / norm for value in quaternion)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        return pose

    @staticmethod
    def _joint_map(
        response: pm_moveit_srv.MoveToPose.Response | pm_moveit_srv.MoveToFrame.Response,
    ) -> dict[str, float]:
        """Convert a MoveIt response into a joint-name lookup table.

        Args:
            response: Successful MoveIt pose or frame planning response.

        Returns:
            Mapping from joint name to target joint value in SI units.

        Raises:
            PmRobotError: If MoveIt returns different numbers of names and
                values.
        """
        if len(response.joint_names) != len(response.joint_values):
            raise PmRobotError('MoveIt returned mismatched joint names and values.')
        return dict(zip(response.joint_names, response.joint_values))

    def _frame_is_on_smarpod(self, frame_name: str) -> bool:
        """Determine whether an assembly frame descends from the top platform.

        Args:
            frame_name: Assembly-scene reference frame to inspect.

        Returns:
            ``True`` when the frame's component ancestry reaches a configured
            Smarpod top-platform frame; otherwise ``False``.

        Raises:
            RefFrameNotFoundError: Propagated by the scene analyzer when the
                frame does not exist.
        """
        analyzer = self.pm_robot_utils.assembly_scene_analyzer
        _, frame = analyzer.get_frame_from_scene(frame_name)
        parent = frame.parent_frame
        visited = set()

        while parent and parent not in visited:
            if parent in self.SMARPOD_TOP_FRAMES:
                return True
            visited.add(parent)
            if analyzer.check_component_exists(parent):
                parent = analyzer.get_parent_of_component(parent)
            else:
                break
        return False

    def _check_cancel(self, goal_handle: ServerGoalHandle) -> None:
        """Stop execution when the ROS action goal requests cancellation.

        Args:
            goal_handle: Currently executing ROS action goal.

        Returns:
            None.

        Raises:
            LineScanCancelled: If cancellation has been requested.
        """
        if goal_handle.is_cancel_requested:
            raise LineScanCancelled('Line scan cancelled.')

    async def _wait_for_world_transforms(
        self,
        goal_handle: ServerGoalHandle,
        frame_names: Sequence[str],
    ) -> dict[str, TransformStamped]:
        """Wait until every requested frame is connected to the world TF tree.

        Args:
            goal_handle: Active action goal used for cancellation checks.
            frame_names: TF frame names that must be reachable from ``world``.

        Returns:
            Mapping from each requested name to its current world transform.

        Raises:
            LineScanCancelled: If cancellation is requested while waiting.
            PmRobotError: If any required transform remains unavailable after
                ``TF_WAIT_TIMEOUT_S``.
        """
        unique_frame_names = tuple(dict.fromkeys(frame_names))
        deadline = time.monotonic() + self.TF_WAIT_TIMEOUT_S
        waiting_logged = False

        while True:
            self._check_cancel(goal_handle)
            missing = [
                frame_name for frame_name in unique_frame_names
                if not self.tf_buffer.can_transform('world', frame_name, Time())
            ]
            if not missing:
                transforms = {
                    frame_name: get_transform_for_frame_in_world(
                        frame_name,
                        self.tf_buffer,
                        self._logger,
                    )
                    for frame_name in unique_frame_names
                }
                if waiting_logged:
                    self._logger.info('Required line-scan TF transforms are now available.')
                return transforms

            if time.monotonic() >= deadline:
                raise PmRobotError(
                    f'Timed out after {self.TF_WAIT_TIMEOUT_S:.1f} s waiting for '
                    f'frames to connect to the world TF tree: {missing}'
                )
            if not waiting_logged:
                self._logger.warning(
                    f'Waiting up to {self.TF_WAIT_TIMEOUT_S:.1f} s for line-scan '
                    f'TF transforms: {missing}'
                )
                waiting_logged = True
            await asyncio.sleep(self.TF_RETRY_INTERVAL_S)

    def _plan_frame_pose(
        self,
        frame_name: str,
        offset_m: float,
        sensor_name: Literal['laser', 'confocal'],
    ) -> Pose:
        """Calculate a collision-free sensor pose for an assembly frame.

        The request asks MoveIt to plan but not execute the motion.

        Args:
            frame_name: Target assembly-scene frame.
            offset_m: World-Z sensor offset in metres.
            sensor_name: Sensor whose MoveIt frame service should be used.

        Returns:
            Calculated sensor end-effector pose in the MoveIt planning frame.

        Raises:
            PmRobotError: If the planning service is unavailable or no
                collision-free plan is found.
        """
        client = (
            self.pm_robot_utils.client_move_robot_laser_to_frame
            if sensor_name == 'laser'
            else self.pm_robot_utils.client_move_robot_confocal_top_to_frame
        )
        if not client.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{client.srv_name}' is not available.")

        request = pm_moveit_srv.MoveToFrame.Request()
        request.target_frame = frame_name
        request.translation.z = offset_m
        request.execute_movement = False
        response = client.call(request)
        if not response.success:
            raise PmRobotError(
                f"Could not plan the {sensor_name} pose for frame "
                f"'{frame_name}': {response.message}"
            )
        return response.calculated_endeffector_pose

    def _plan_pose(
        self,
        pose: Pose,
        use_hexapod: bool,
        sensor_name: Literal['laser', 'confocal'],
    ) -> dict[str, float]:
        """Plan one collision-check pose without executing it.

        Args:
            pose: Target pose in the MoveIt planning frame.
            use_hexapod: Plan with the Smarpod group when ``True``; otherwise
                plan with the selected robot sensor group.
            sensor_name: Select the laser or top-confocal MoveIt pose client.

        Returns:
            Mapping of planned joint names to target values in SI units.

        Raises:
            PmRobotError: If the selected service is unavailable or planning
                fails, including collision-related planning failure.
        """
        if use_hexapod:
            client = self._move_smarpod_to_pose_client
        elif sensor_name == 'laser':
            client = self.pm_robot_utils.client_move_robot_laser_to_pose
        else:
            client = self.pm_robot_utils.client_move_robot_confocal_top_to_pose
        if not client.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{client.srv_name}' is not available.")

        request = pm_moveit_srv.MoveToPose.Request()
        request.move_to_pose = pose
        request.execute_movement = False
        response = client.call(request)
        if not response.success:
            mover = 'hexapod' if use_hexapod else f'{sensor_name} head'
            raise PmRobotError(f'Collision-free {mover} plan not found: {response.message}')
        return self._joint_map(response)

    def _collision_check_waypoints(
        self,
        start_pose: Pose,
        end_pose: Pose,
        use_hexapod: bool,
        sensor_name: Literal['laser', 'confocal'],
    ) -> tuple[list[float], list[dict[str, float]]]:
        """Plan five evenly spaced poses along the requested scan line.

        Args:
            start_pose: First laser or hexapod pose.
            end_pose: Final laser or hexapod pose.
            use_hexapod: Select the hexapod MoveIt group when ``True``.
            sensor_name: Select the laser or top-confocal MoveIt group when the
                robot head performs the scan.

        Returns:
            A pair containing interpolation fractions and their corresponding
            collision-checked joint maps.

        Raises:
            PmRobotError: If any one of the five poses cannot be planned.
        """
        fractions = [
            index / (self.COLLISION_CHECK_COUNT - 1)
            for index in range(self.COLLISION_CHECK_COUNT)
        ]
        joint_waypoints = []
        for fraction in fractions:
            pose = self._interpolate_pose(start_pose, end_pose, fraction)
            joint_waypoints.append(self._plan_pose(pose, use_hexapod, sensor_name))
        return fractions, joint_waypoints

    @staticmethod
    def _interpolate_joint_waypoints(
        fraction: float,
        fractions: Sequence[float],
        joint_waypoints: Sequence[Mapping[str, float]],
        joint_names: Sequence[str],
    ) -> list[float]:
        """Interpolate controller targets between checked joint waypoints.

        Args:
            fraction: Overall scan fraction in ``[0.0, 1.0]``.
            fractions: Fractions associated with collision-checked waypoints.
            joint_waypoints: Joint-value mappings at each checked fraction.
            joint_names: Ordered joints required by the trajectory controller.

        Returns:
            Ordered interpolated joint values in SI units.

        Raises:
            PmRobotError: If a required joint is absent from either bounding
                waypoint.
        """
        if fraction >= 1.0:
            segment = len(fractions) - 2
            local_fraction = 1.0
        else:
            segment = min(int(fraction * (len(fractions) - 1)), len(fractions) - 2)
            span = fractions[segment + 1] - fractions[segment]
            local_fraction = (fraction - fractions[segment]) / span

        start = joint_waypoints[segment]
        end = joint_waypoints[segment + 1]
        missing = [name for name in joint_names if name not in start or name not in end]
        if missing:
            raise PmRobotError(f'MoveIt did not return required joints: {missing}')
        return [
            start[name] + local_fraction * (end[name] - start[name])
            for name in joint_names
        ]

    def _move_scan_joints(
        self,
        values: Sequence[float],
        use_hexapod: bool,
        duration_s: float,
    ) -> None:
        """Send one absolute line-scan waypoint to a trajectory controller.

        Args:
            values: Ordered robot XYZ values or Smarpod XYZABC values. Linear
                values are metres and angular values are radians.
            use_hexapod: Use the Smarpod controller when ``True``; otherwise use
                the robot XYZ-axis controller.
            duration_s: Requested controller movement duration in seconds.

        Returns:
            None.

        Raises:
            PmRobotError: If the controller does not reach the target.
        """
        if use_hexapod:
            success = self.pm_robot_utils.send_smarpod_trajectory_goal_absolut(
                x_joint=values[0],
                y_joint=values[1],
                z_joint=values[2],
                rx_joint_deg=math.degrees(values[3]),
                ry_joint_deg=math.degrees(values[4]),
                rz_joint_deg=math.degrees(values[5]),
                time=duration_s,
            )
        else:
            success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(
                x_joint=values[0],
                y_joint=values[1],
                z_joint=values[2],
                time=duration_s,
            )
        if not success:
            raise PmRobotError('Trajectory controller did not reach the line-scan waypoint.')

    def _move_sensor_to_frame(
        self,
        sensor_name: Literal['laser', 'confocal'],
        frame_name: str,
        offset_m: float,
    ) -> None:
        """Move the selected sensor to the first scan frame.

        Args:
            sensor_name: Select the laser or top-confocal sensor.
            frame_name: Assembly-scene frame at which scanning begins.
            offset_m: World-Z sensor offset in metres.

        Returns:
            None.

        Raises:
            PmRobotError: If the sensor motion fails.
        """
        if sensor_name == 'laser':
            success = self.pm_robot_utils.move_laser_to_frame(
                frame_name,
                z_offset=offset_m,
            )
            message = ''
        else:
            success, message = self.pm_robot_utils.move_confocal_top_to_frame(
                frame_name,
                z_offset=offset_m,
            )

        if not success:
            detail = f': {message}' if message else ''
            raise PmRobotError(
                f"Could not move the {sensor_name} to start frame '{frame_name}'{detail}"
            )

    # def _move_robot_up(self, distance_m: float) -> None:
    #     """Retract the robot vertically from its current Cartesian position.

    #     Args:
    #         distance_m: Positive Z-axis retraction distance in metres.

    #     Returns:
    #         None.

    #     Raises:
    #         PmRobotError: If current XYZ joint positions are unavailable or the
    #             trajectory controller cannot complete the retraction.
    #     """
    #     joint_values = [
    #         self.pm_robot_utils.get_current_joint_state(joint_name)
    #         for joint_name in self.ROBOT_TRANSLATION_JOINTS
    #     ]
    #     if any(value is None or not math.isfinite(value) for value in joint_values):
    #         raise PmRobotError('Cannot retract robot: current XYZ joint state is unavailable.')

    #     joint_values[2] += distance_m
    #     self._move_scan_joints(
    #         values=joint_values,
    #         use_hexapod=False,
    #         duration_s=max(0.5, distance_m * 1e3 / 20.0),
    #     )

    def _measurement_is_valid(
        self,
        sensor_name: Literal['laser', 'confocal'],
    ) -> bool:
        """Check whether the selected sensor currently reports a usable value.

        Args:
            sensor_name: Select the laser or top-confocal validity check.

        Returns:
            ``True`` when the current measurement is within the sensor's valid
            range; otherwise ``False``.
        """
        if sensor_name == 'laser':
            return self.pm_robot_utils._check_for_valid_laser_measurement()
        return self.pm_robot_utils.check_confocal_top_measurement_in_range()

    def _get_measurement_mm(
        self,
        sensor_name: Literal['laser', 'confocal'],
    ) -> float:
        """Read one measurement from the selected distance sensor.

        Args:
            sensor_name: Select the laser or top-confocal measurement service.

        Returns:
            Signed sensor measurement in millimetres.

        Raises:
            PmRobotError: Propagated when the measurement service is unavailable.
        """
        if sensor_name == 'laser':
            return self.pm_robot_utils.get_laser_measurement(unit='mm')
        return self.pm_robot_utils.get_confocal_top_measurement(unit='mm')

    def _read_measurement_or_nan(
        self,
        sensor_name: Literal['laser', 'confocal'],
    ) -> tuple[float, str]:
        """Read a sensor value and represent an invalid reading as ``NaN``.

        Args:
            sensor_name: Sensor used for the current line scan.

        Returns:
            A pair containing the measurement in mm and an empty reason when
            valid, or ``NaN`` and a human-readable skip reason when invalid.
        """
        if not self._measurement_is_valid(sensor_name):
            return math.nan, 'no valid measurement'
        measurement_mm = self._get_measurement_mm(sensor_name)
        if not math.isfinite(measurement_mm):
            return math.nan, 'measurement is not finite'
        return measurement_mm, ''

    @staticmethod
    def _create_profile_output_base(
        sensor_name: Literal['laser', 'confocal'],
        start_frame: str,
        end_frame: str,
    ) -> Path:
        """Create a unique common output path for a scan's SVG and CSV files.

        Args:
            sensor_name: Sensor used to acquire the line scan.
            start_frame: Assembly frame at the beginning of the scan.
            end_frame: Assembly frame at the end of the scan.

        Returns:
            Absolute path without a filename extension.

        Raises:
            OSError: If the line-scan output directory cannot be created.
        """
        safe_start = re.sub(r'[^A-Za-z0-9_.-]+', '_', start_frame)
        safe_end = re.sub(r'[^A-Za-z0-9_.-]+', '_', end_frame)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        output_dir = Path.home() / '.ros' / 'pm_skills' / 'line_scan_profiles'
        output_dir.mkdir(parents=True, exist_ok=True)
        return (output_dir / (
            f'{timestamp}_{sensor_name}_{safe_start}_to_{safe_end}'
        )).resolve()

    @staticmethod
    def _save_measurements_csv(
        output_base_path: Path,
        path_positions_mm: Sequence[float],
        measurements_mm: Sequence[float],
        refined_samples: Sequence[bool],
    ) -> str:
        """Save every coarse and refined measurement to a CSV file.

        Args:
            output_base_path: Common output path without an extension.
            path_positions_mm: Sorted positions along the scan path in mm.
            measurements_mm: Corresponding measurements in mm, including NaN.
            refined_samples: Flags identifying refinement-pass measurements.

        Returns:
            Absolute path of the generated CSV file.

        Raises:
            ValueError: If the three sample arrays have different lengths.
            OSError: If the CSV file cannot be created.
        """
        if not (
            len(path_positions_mm)
            == len(measurements_mm)
            == len(refined_samples)
        ):
            raise ValueError('Cannot save mismatched line-scan sample arrays.')

        output_path = Path(f'{output_base_path}.csv')
        with output_path.open('w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                'sample_index',
                'path_position_mm',
                'measurement_mm',
                'is_valid',
                'scan_pass',
            ])
            for index, (position_mm, measurement_mm, is_refined) in enumerate(
                zip(path_positions_mm, measurements_mm, refined_samples),
                start=1,
            ):
                is_valid = math.isfinite(measurement_mm)
                writer.writerow([
                    index,
                    f'{position_mm:.9f}',
                    f'{measurement_mm:.9f}' if is_valid else 'NaN',
                    str(is_valid).lower(),
                    'refinement' if is_refined else 'coarse',
                ])
        return str(output_path.resolve())

    @staticmethod
    def _create_profile_plot(
        output_base_path: Path,
        path_positions_mm: Sequence[float],
        measurements_mm: Sequence[float],
        refined_samples: Sequence[bool],
        selected_index: int,
        sensor_name: Literal['laser', 'confocal'],
        start_frame: str,
        end_frame: str,
    ) -> str:
        """Plot raw samples and connect consecutive finite measurements.

        Args:
            output_base_path: Common output path without an extension.
            path_positions_mm: Measurement positions along the scan in mm.
            measurements_mm: Sensor values corresponding to each position in mm.
            refined_samples: Flags identifying samples from the refinement pass.
            selected_index: Full-path index chosen for correction.
            sensor_name: Sensor used to acquire the profile.
            start_frame: Assembly frame at the beginning of the scan.
            end_frame: Assembly frame at the end of the scan.

        Returns:
            Absolute filesystem path of the generated SVG file.

        Raises:
            ValueError: If the profile arrays are empty, have different lengths,
                contain no finite values, or the selected sample is invalid.
            OSError: If the output directory or plot file cannot be created.
        """
        if (
            not path_positions_mm
            or len(path_positions_mm) != len(measurements_mm)
            or len(path_positions_mm) != len(refined_samples)
        ):
            raise ValueError('Cannot plot an empty or mismatched line-scan profile.')
        valid_indices = [
            index for index, value in enumerate(measurements_mm)
            if math.isfinite(value)
        ]
        if not valid_indices:
            raise ValueError('Cannot plot a line scan without valid measurements.')
        if (
            selected_index < 0
            or selected_index >= len(measurements_mm)
            or selected_index not in valid_indices
        ):
            raise ValueError('Selected profile index is out of range or invalid.')

        minimum_index = min(valid_indices, key=measurements_mm.__getitem__)
        maximum_index = max(valid_indices, key=measurements_mm.__getitem__)
        invalid_indices = [
            index for index in range(len(measurements_mm))
            if index not in valid_indices
        ]
        output_path = Path(f'{output_base_path}.svg')

        valid_measurements = [measurements_mm[index] for index in valid_indices]
        width, height = 1250, 560
        left, right, top, bottom = 90, 350, 70, 90
        plot_width = width - left - right
        plot_height = height - top - bottom
        legend_x = left + plot_width + 30
        x_min, x_max = min(path_positions_mm), max(path_positions_mm)
        y_min, y_max = min(valid_measurements), max(valid_measurements)
        x_span = x_max - x_min or 1.0
        y_padding = max((y_max - y_min) * 0.1, 1e-6)
        y_low, y_high = y_min - y_padding, y_max + y_padding
        y_span = y_high - y_low

        def plot_x(value: float) -> float:
            """Map a path position in mm to an SVG X coordinate."""
            return left + (value - x_min) / x_span * plot_width

        def plot_y(value: float) -> float:
            """Map a measurement in mm to an SVG Y coordinate."""
            return top + (y_high - value) / y_span * plot_height

        def cross(center_x: float, center_y: float, color: str, size: float) -> str:
            """Create one SVG cross marker without connecting adjacent samples."""
            return (
                f'<path d="M {center_x - size:.2f} {center_y - size:.2f} '
                f'L {center_x + size:.2f} {center_y + size:.2f} '
                f'M {center_x - size:.2f} {center_y + size:.2f} '
                f'L {center_x + size:.2f} {center_y - size:.2f}" '
                f'stroke="{color}" stroke-width="1.8" fill="none"/>'
            )

        measurement_segments = []
        current_segment = []
        for path_position_mm, measurement_mm in zip(
            path_positions_mm,
            measurements_mm,
        ):
            if math.isfinite(measurement_mm):
                command = 'M' if not current_segment else 'L'
                current_segment.append(
                    f'{command} {plot_x(path_position_mm):.2f} '
                    f'{plot_y(measurement_mm):.2f}'
                )
            elif current_segment:
                measurement_segments.append(' '.join(current_segment))
                current_segment = []
        if current_segment:
            measurement_segments.append(' '.join(current_segment))
        measurement_lines = '\n  '.join(
            f'<path d="{segment}" stroke="#777777" stroke-width="1.2" '
            'fill="none"/>'
            for segment in measurement_segments
        )

        coarse_markers = '\n  '.join(
            cross(
                plot_x(path_positions_mm[index]),
                plot_y(measurements_mm[index]),
                '#333333',
                3.5,
            )
            for index in valid_indices
            if not refined_samples[index]
        )
        refinement_markers = '\n  '.join(
            cross(
                plot_x(path_positions_mm[index]),
                plot_y(measurements_mm[index]),
                '#9467bd',
                4.5,
            )
            for index in valid_indices
            if refined_samples[index]
        )
        invalid_y = top + plot_height - 10
        invalid_markers = '\n  '.join(
            cross(plot_x(path_positions_mm[index]), invalid_y, '#ff7f0e', 5.0)
            for index in invalid_indices
        )
        min_x, min_y = (
            plot_x(path_positions_mm[minimum_index]),
            plot_y(measurements_mm[minimum_index]),
        )
        max_x, max_y = (
            plot_x(path_positions_mm[maximum_index]),
            plot_y(measurements_mm[maximum_index]),
        )
        selected_x, selected_y = (
            plot_x(path_positions_mm[selected_index]),
            plot_y(measurements_mm[selected_index]),
        )
        title = escape(
            f'{sensor_name.capitalize()} line scan: {start_frame} to {end_frame}'
        )
        grid_divisions = 5
        grid_lines = '\n  '.join(
            [
                f'<line x1="{left + plot_width * index / grid_divisions:.2f}" '
                f'y1="{top}" '
                f'x2="{left + plot_width * index / grid_divisions:.2f}" '
                f'y2="{top + plot_height}" stroke="#ddd" stroke-width="1"/>'
                for index in range(1, grid_divisions)
            ]
            + [
                f'<line x1="{left}" '
                f'y1="{top + plot_height * index / grid_divisions:.2f}" '
                f'x2="{left + plot_width}" '
                f'y2="{top + plot_height * index / grid_divisions:.2f}" '
                f'stroke="#ddd" stroke-width="1"/>'
                for index in range(1, grid_divisions)
            ]
        )

        svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="white"/>
  <text x="{width / 2}" y="34" text-anchor="middle" font-family="sans-serif" font-size="20">{title}</text>
  {grid_lines}
  <line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_height}" stroke="#222"/>
  <line x1="{left}" y1="{top + plot_height}" x2="{left + plot_width}" y2="{top + plot_height}" stroke="#222"/>
  <text x="{left + plot_width / 2}" y="{height - 25}" text-anchor="middle" font-family="sans-serif" font-size="15">Path position [mm]</text>
  <text x="22" y="{height / 2}" text-anchor="middle" font-family="sans-serif" font-size="15" transform="rotate(-90 22 {height / 2})">Measurement [mm]</text>
  <text x="{left}" y="{top + plot_height + 24}" text-anchor="middle" font-family="sans-serif" font-size="12">{x_min:.4f}</text>
  <text x="{left + plot_width}" y="{top + plot_height + 24}" text-anchor="middle" font-family="sans-serif" font-size="12">{x_max:.4f}</text>
  <text x="{left - 10}" y="{top + 5}" text-anchor="end" font-family="sans-serif" font-size="12">{y_high:.6f}</text>
  <text x="{left - 10}" y="{top + plot_height + 5}" text-anchor="end" font-family="sans-serif" font-size="12">{y_low:.6f}</text>
  {measurement_lines}
  {coarse_markers}
  {refinement_markers}
  {invalid_markers}
  <circle cx="{min_x:.2f}" cy="{min_y:.2f}" r="8" fill="none" stroke="#1f77b4" stroke-width="2"/>
  <circle cx="{max_x:.2f}" cy="{max_y:.2f}" r="8" fill="none" stroke="#d62728" stroke-width="2"/>
  <circle cx="{selected_x:.2f}" cy="{selected_y:.2f}" r="12" fill="none" stroke="#2ca02c" stroke-width="2.5"/>
  <rect x="{legend_x}" y="{top}" width="305" height="154" fill="white" stroke="#bbb"/>
  <line x1="{legend_x + 8}" y1="{top + 19}" x2="{legend_x + 26}" y2="{top + 19}" stroke="#777777" stroke-width="1.2"/>
  {cross(legend_x + 17, top + 19, '#333333', 3.5)}
  <text x="{legend_x + 33}" y="{top + 24}" font-family="sans-serif" font-size="13">Measurement profile</text>
  {cross(legend_x + 17, top + 43, '#9467bd', 4.5)}
  <text x="{legend_x + 33}" y="{top + 48}" font-family="sans-serif" font-size="13">Refined measurement</text>
  <circle cx="{legend_x + 17}" cy="{top + 67}" r="7" fill="none" stroke="#1f77b4" stroke-width="2"/>
  <text x="{legend_x + 33}" y="{top + 72}" font-family="sans-serif" font-size="13">Minimum: {measurements_mm[minimum_index]:.6f} mm</text>
  <circle cx="{legend_x + 17}" cy="{top + 91}" r="7" fill="none" stroke="#d62728" stroke-width="2"/>
  <text x="{legend_x + 33}" y="{top + 96}" font-family="sans-serif" font-size="13">Maximum: {measurements_mm[maximum_index]:.6f} mm</text>
  <circle cx="{legend_x + 17}" cy="{top + 115}" r="9" fill="none" stroke="#2ca02c" stroke-width="2.5"/>
  <text x="{legend_x + 33}" y="{top + 120}" font-family="sans-serif" font-size="13">Selected point</text>
  {cross(legend_x + 17, top + 139, '#ff7f0e', 5.0)}
  <text x="{legend_x + 33}" y="{top + 144}" font-family="sans-serif" font-size="13">Invalid measurement</text>
</svg>
'''
        output_path.write_text(svg, encoding='utf-8')

        return str(output_path.resolve())

    def _correct_frame(
        self,
        frame_name: str,
        fraction: float,
        measurement_m: float,
    ) -> tuple[Pose, tuple[float, float, float]]:
        """Move an assembly frame to the selected measured profile point.

        The selected line position supplies world X/Y and nominal Z. The sensor
        measurement is added to nominal Z, while the frame's orientation is
        preserved.

        Args:
            frame_name: Assembly-scene frame to modify.
            fraction: Selected position along the start-to-end scan line.
            measurement_m: Selected sensor height correction in metres.

        Returns:
            Corrected absolute pose and ``(dx, dy, dz)`` correction in metres.

        Raises:
            PmRobotError: If the assembly-manager service is unavailable or
                rejects the frame modification.
            TfFrameLookupError: If a required frame is unavailable in TF.
        """
        start_world = get_transform_for_frame_in_world(
            self._current_start_frame,
            self.tf_buffer,
            self._logger,
        )
        end_world = get_transform_for_frame_in_world(
            self._current_end_frame,
            self.tf_buffer,
            self._logger,
        )
        selected_point = self._interpolate_vector(
            self._position(start_world),
            self._position(end_world),
            fraction,
        )
        current = get_transform_for_frame_in_world(frame_name, self.tf_buffer, self._logger)

        request = ami_srv.ModifyPoseAbsolut.Request()
        request.frame_name = frame_name
        request.pose.position.x = selected_point[0]
        request.pose.position.y = selected_point[1]
        request.pose.position.z = selected_point[2] + measurement_m
        request.pose.orientation = current.transform.rotation
        request.set_laser_measured = True

        if not self.adapt_frame_client.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self.adapt_frame_client.srv_name}' is not available.")
        response = self.adapt_frame_client.call(request)
        if not response.success:
            raise PmRobotError(f"Could not correct frame '{frame_name}'.")

        correction = (
            request.pose.position.x - current.transform.translation.x,
            request.pose.position.y - current.transform.translation.y,
            request.pose.position.z - current.transform.translation.z,
        )
        return request.pose, correction

    async def correct_frame_laser_line_scan(
        self,
        goal_handle: ServerGoalHandle,
    ) -> pm_skill_action.CorrectFrameLineScan.Result:
        """Run the shared line-scan action using the laser sensor.

        Args:
            goal_handle: Accepted laser line-scan action goal.

        Returns:
            Result produced by the shared line-scan implementation.
        """
        return await self._correct_frame_line_scan(goal_handle, 'laser')

    async def correct_frame_confocal_line_scan(
        self,
        goal_handle: ServerGoalHandle,
    ) -> pm_skill_action.CorrectFrameLineScan.Result:
        """Run the shared line-scan action using the top confocal sensor.

        Args:
            goal_handle: Accepted confocal line-scan action goal.

        Returns:
            Result produced by the shared line-scan implementation.
        """
        return await self._correct_frame_line_scan(goal_handle, 'confocal')

    async def _correct_frame_line_scan(
        self,
        goal_handle: ServerGoalHandle,
        sensor_name: Literal['laser', 'confocal'],
    ) -> pm_skill_action.CorrectFrameLineScan.Result:
        """Run a cancellable scan with the endpoint-selected sensor.

        Args:
            goal_handle: Accepted action goal containing two scan frames, an
                optional correction frame, sensor offset, step size, and
                min/max selection mode.
            sensor_name: Sensor fixed by the action server handling the goal.

        Returns:
            Line-scan result containing the height profile, selected extremum,
            corrected pose, plot path, and motion-platform information.

        Notes:
            The action aborts before motion when more than ``MAX_SAMPLES`` would
            be required. Cancellation is honored between each blocking motion,
            validation, measurement, and frame-correction operation. Missing,
            out-of-range, and non-finite samples are retained as ``NaN`` in the
            full-path profile; only finite samples participate in min/max
            selection and frame correction.
        """
        action_type = pm_skill_action.CorrectFrameLineScan
        result = action_type.Result()
        goal = goal_handle.request
        path_positions_mm = []
        measurements_mm = []
        refined_samples = []
        skipped_sample_count = 0
        robot_needs_retraction = False
        goal_finished = False

        try:
            analyzer = self.pm_robot_utils.assembly_scene_analyzer
            analyzer.wait_for_initial_scene_update()
            frames = [goal.start_frame, goal.end_frame]
            if goal.frame_to_correct:
                frames.append(goal.frame_to_correct)
            missing = [name for name in frames if not analyzer.is_frame_from_scene(name)]
            if missing:
                raise PmRobotError(f'Frames are not part of the assembly scene: {missing}')

            world_transforms = await self._wait_for_world_transforms(
                goal_handle,
                frames,
            )
            start_world = world_transforms[goal.start_frame]
            end_world = world_transforms[goal.end_frame]
            start_point = self._position(start_world)
            end_point = self._position(end_world)
            distance_m = self._distance(start_point, end_point)
            if distance_m <= 1e-9:
                raise PmRobotError('Start and end frames define a zero-length scan.')

            interval_count = max(1, math.ceil(distance_m / (goal.step_size_mm * 1e-3)))
            sample_count = interval_count + 1
            refinement_sample_count = (
                self.REFINEMENT_SAMPLE_COUNT
                if goal.use_min_max_refinement else 0
            )
            total_requested_sample_count = sample_count + refinement_sample_count
            if total_requested_sample_count > self.MAX_SAMPLES:
                raise PmRobotError(
                    f'The requested line requires {total_requested_sample_count} '
                    f'measurements ({sample_count} coarse and '
                    f'{refinement_sample_count} refined); '
                    f'the maximum is {self.MAX_SAMPLES}. Increase step_size_mm.'
                )
            
            start_on_smarpod = self.pm_robot_utils.assembly_scene_analyzer.frame_is_on_smarpod(goal.start_frame)
            end_on_smarpod = self.pm_robot_utils.assembly_scene_analyzer.frame_is_on_smarpod(goal.end_frame)

            if start_on_smarpod != end_on_smarpod:
                raise PmRobotError(
                    'Start and end frames must both be on the hexapod platform or both be stationary.'
                )
            use_hexapod = start_on_smarpod and end_on_smarpod
            result.used_hexapod = use_hexapod
            self._current_start_frame = goal.start_frame
            self._current_end_frame = goal.end_frame
            offset_m = goal.sensor_offset_mm * 1e-3

            self._check_cancel(goal_handle)
            self._move_sensor_to_frame(
                sensor_name,
                goal.start_frame,
                offset_m + self.ROBOT_APPROACH_CLEARANCE_M,
            )
            self._check_cancel(goal_handle)
            self._move_sensor_to_frame(sensor_name, goal.start_frame, offset_m)
            robot_needs_retraction = True

            # Calculate the five collision-check waypoints only after the
            # sensor has reached its real starting measurement pose. This keeps
            # the MoveIt planning scene consistent for robot and Smarpod scans.
            self._check_cancel(goal_handle)
            if use_hexapod:
                top_world = (
                    await self._wait_for_world_transforms(
                        goal_handle,
                        ['Smarpod_Top_Plate'],
                    )
                )['Smarpod_Top_Plate']
                scan_delta = tuple(end - start for start, end in zip(start_point, end_point))
                start_pose = Pose()
                start_pose.position.x = top_world.transform.translation.x
                start_pose.position.y = top_world.transform.translation.y
                start_pose.position.z = top_world.transform.translation.z
                start_pose.orientation = top_world.transform.rotation
                end_pose = Pose()
                end_pose.position.x = start_pose.position.x - scan_delta[0]
                end_pose.position.y = start_pose.position.y - scan_delta[1]
                end_pose.position.z = start_pose.position.z - scan_delta[2]
                end_pose.orientation = start_pose.orientation
                joint_names = self.SMARPOD_JOINTS
            else:
                start_pose = self._plan_frame_pose(
                    goal.start_frame,
                    offset_m,
                    sensor_name,
                )
                end_pose = self._plan_frame_pose(
                    goal.end_frame,
                    offset_m,
                    sensor_name,
                )
                joint_names = self.ROBOT_TRANSLATION_JOINTS

            fractions, joint_waypoints = self._collision_check_waypoints(
                start_pose,
                end_pose,
                use_hexapod,
                sensor_name,
            )

            self._logger.info(
                f'Starting {sensor_name} line scan with {sample_count} coarse increments '
                f'over {distance_m * 1e3:.3f} mm '
                f'(maximum step size {goal.step_size_mm:.3f} mm).'
            )
            for index in range(sample_count):
                self._check_cancel(goal_handle)
                fraction = index / interval_count
                joint_values = self._interpolate_joint_waypoints(
                    fraction,
                    fractions,
                    joint_waypoints,
                    joint_names,
                )
                if index > 0 or use_hexapod:
                    step_distance_mm = distance_m * 1e3 / interval_count
                    duration_s = max(0.1, step_distance_mm / 2.0)
                    self._move_scan_joints(joint_values, use_hexapod, duration_s)

                self._check_cancel(goal_handle)
                path_position_mm = fraction * distance_m * 1e3
                measurement_mm = math.nan
                path_positions_mm.append(path_position_mm)
                measurements_mm.append(measurement_mm)
                refined_samples.append(False)
                measurement_mm, skip_reason = self._read_measurement_or_nan(sensor_name)

                if skip_reason:
                    skipped_sample_count += 1
                    feedback = action_type.Feedback()
                    feedback.current_step = index + 1
                    feedback.total_steps = total_requested_sample_count
                    feedback.path_position_mm = path_position_mm
                    feedback.measurement_mm = math.nan
                    feedback.using_hexapod = use_hexapod
                    feedback.sensor_name = sensor_name
                    feedback.message = (
                        f'Increment {index + 1}/{sample_count}: '
                        f'position={path_position_mm:.3f} mm, '
                        f'measurement=NaN ({skip_reason}).'
                    )
                    goal_handle.publish_feedback(feedback)
                    self._logger.warning(feedback.message)
                    continue

                measurements_mm[-1] = measurement_mm

                feedback = action_type.Feedback()
                feedback.current_step = index + 1
                feedback.total_steps = total_requested_sample_count
                feedback.path_position_mm = path_position_mm
                feedback.measurement_mm = measurement_mm
                feedback.using_hexapod = use_hexapod
                feedback.sensor_name = sensor_name
                feedback.message = (
                    f'Increment {index + 1}/{sample_count}: '
                    f'position={path_position_mm:.3f} mm, '
                    f'measurement={measurement_mm:.6f} mm.'
                )
                goal_handle.publish_feedback(feedback)
                self._logger.info(feedback.message)

            self._check_cancel(goal_handle)
            coarse_valid_indices = [
                index for index, value in enumerate(measurements_mm)
                if math.isfinite(value)
            ]
            if goal.use_min_max_refinement and coarse_valid_indices:
                coarse_minimum_index = min(
                    coarse_valid_indices,
                    key=measurements_mm.__getitem__,
                )
                coarse_maximum_index = max(
                    coarse_valid_indices,
                    key=measurements_mm.__getitem__,
                )
                coarse_selected_index = (
                    coarse_maximum_index
                    if goal.use_max_value else coarse_minimum_index
                )
                center_distance_m = path_positions_mm[coarse_selected_index] * 1e-3
                half_window_m = goal.refinement_length_um * 0.5e-6
                refinement_start_m = max(0.0, center_distance_m - half_window_m)
                refinement_end_m = min(distance_m, center_distance_m + half_window_m)
                refinement_span_m = refinement_end_m - refinement_start_m
                refinement_fractions = [
                    (
                        refinement_start_m
                        + refinement_span_m
                        * refinement_index
                        / self.REFINEMENT_INTERVAL_COUNT
                    ) / distance_m
                    for refinement_index in range(self.REFINEMENT_SAMPLE_COUNT)
                ]
                actual_step_um = (
                    refinement_span_m * 1e6 / self.REFINEMENT_INTERVAL_COUNT
                )
                self._logger.info(
                    f'Refining the coarse '
                    f"{'maximum' if goal.use_max_value else 'minimum'} around "
                    f'{center_distance_m * 1e3:.6f} mm over '
                    f'{refinement_span_m * 1e6:.3f} um with '
                    f'{self.REFINEMENT_SAMPLE_COUNT} measurements '
                    f'({actual_step_um:.3f} um increments).'
                )

                previous_fraction = 1.0
                for refinement_index, fraction in enumerate(refinement_fractions):
                    self._check_cancel(goal_handle)
                    joint_values = self._interpolate_joint_waypoints(
                        fraction,
                        fractions,
                        joint_waypoints,
                        joint_names,
                    )
                    movement_distance_mm = (
                        abs(fraction - previous_fraction) * distance_m * 1e3
                    )
                    self._move_scan_joints(
                        joint_values,
                        use_hexapod,
                        max(0.1, movement_distance_mm / 2.0),
                    )
                    previous_fraction = fraction

                    self._check_cancel(goal_handle)
                    path_position_mm = fraction * distance_m * 1e3
                    measurement_mm, skip_reason = self._read_measurement_or_nan(
                        sensor_name
                    )
                    path_positions_mm.append(path_position_mm)
                    measurements_mm.append(measurement_mm)
                    refined_samples.append(True)
                    if skip_reason:
                        skipped_sample_count += 1

                    feedback = action_type.Feedback()
                    feedback.current_step = sample_count + refinement_index + 1
                    feedback.total_steps = total_requested_sample_count
                    feedback.path_position_mm = path_position_mm
                    feedback.measurement_mm = measurement_mm
                    feedback.using_hexapod = use_hexapod
                    feedback.sensor_name = sensor_name
                    measurement_text = (
                        'NaN' if skip_reason else f'{measurement_mm:.3f} mm'
                    )
                    reason_text = f' ({skip_reason})' if skip_reason else ''
                    feedback.message = (
                        f'Refinement {refinement_index + 1}/'
                        f'{self.REFINEMENT_SAMPLE_COUNT}: '
                        f'position={path_position_mm:.6f} mm, '
                        f'measurement={measurement_text}{reason_text}.'
                    )
                    goal_handle.publish_feedback(feedback)
                    if skip_reason:
                        self._logger.warning(feedback.message)
                    else:
                        self._logger.info(feedback.message)

            combined_samples = sorted(
                zip(path_positions_mm, measurements_mm, refined_samples),
                key=lambda sample: sample[0],
            )
            path_positions_mm = [sample[0] for sample in combined_samples]
            measurements_mm = [sample[1] for sample in combined_samples]
            refined_samples = [sample[2] for sample in combined_samples]
            output_base_path = self._create_profile_output_base(
                sensor_name,
                goal.start_frame,
                goal.end_frame,
            )
            result.csv_file_path = self._save_measurements_csv(
                output_base_path,
                path_positions_mm,
                measurements_mm,
                refined_samples,
            )
            self._logger.info(
                f'Line-scan measurements saved to {result.csv_file_path}'
            )
            valid_indices = [
                index for index, value in enumerate(measurements_mm)
                if math.isfinite(value)
            ]
            if not valid_indices:
                raise PmRobotError(
                    f'The {sensor_name} line scan completed, but none of its '
                    f'{sample_count} samples contained a valid measurement.'
                )

            valid_sample_count = len(valid_indices)
            minimum_index = min(valid_indices, key=measurements_mm.__getitem__)
            maximum_index = max(valid_indices, key=measurements_mm.__getitem__)
            selected_index = maximum_index if goal.use_max_value else minimum_index
            selected_fraction = (
                path_positions_mm[selected_index] / (distance_m * 1e3)
            )
            selected_measurement_mm = measurements_mm[selected_index]
            total_sample_count = len(measurements_mm)
            self._logger.info(
                f'Line scan complete: {valid_sample_count}/{total_sample_count} valid; '
                f'minimum={measurements_mm[minimum_index]:.3f} mm at '
                f'{path_positions_mm[minimum_index]:.6f} mm, '
                f'maximum={measurements_mm[maximum_index]:.3f} mm at '
                f'{path_positions_mm[maximum_index]:.6f} mm.'
            )
            plot_file_path = self._create_profile_plot(
                output_base_path=output_base_path,
                path_positions_mm=path_positions_mm,
                measurements_mm=measurements_mm,
                refined_samples=refined_samples,
                selected_index=selected_index,
                sensor_name=sensor_name,
                start_frame=goal.start_frame,
                end_frame=goal.end_frame,
            )
            self._check_cancel(goal_handle)
            if goal.frame_to_correct:
                corrected_pose, correction = self._correct_frame(
                    goal.frame_to_correct,
                    selected_fraction,
                    selected_measurement_mm * 1e-3,
                )
                result.corrected_pose = corrected_pose
                (
                    result.correction_values.x,
                    result.correction_values.y,
                    result.correction_values.z,
                ) = correction
                operation_message = f"Corrected '{goal.frame_to_correct}'"
            else:
                operation_message = 'Created line-scan profile without frame correction'

            result.success = True
            result.message = (
                f'{operation_message}; selected the '
                f"{'maximum' if goal.use_max_value else 'minimum'} at profile "
                f'sample {selected_index + 1}/{total_sample_count} '
                f'({valid_sample_count} valid samples). '
                f'Skipped {skipped_sample_count}/{total_sample_count} measurements.'
            )
            result.selected_index = selected_index
            result.selected_path_position_mm = path_positions_mm[selected_index]
            result.selected_measurement_mm = selected_measurement_mm
            result.plot_file_path = plot_file_path
            goal_handle.succeed()
            goal_finished = True

        except LineScanCancelled as error:
            result.success = False
            result.message = str(error)
            goal_handle.canceled()
            goal_finished = True
        except Exception as error:
            result.success = False
            result.message = str(error)
            self._logger.error(result.message)
            goal_handle.abort()
            goal_finished = True
        finally:
            if robot_needs_retraction:
                move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0,0,-0.04,time=0.5) # move up after dispensing to be safe
                if not move_success:
                    result.message = result.message + "Failed to move up after dispensing! Please check the robot state!"
                    self.logger.error(result.message)
                    result.success = False

            result.path_positions_mm = path_positions_mm
            result.measurements_mm = measurements_mm
            with self._active_lock:
                self._active = False
            if not goal_finished:
                goal_handle.abort()

        return result
