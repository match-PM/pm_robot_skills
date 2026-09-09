"""Rectangular-spiral active-alignment action."""

from __future__ import annotations

import csv
from bisect import bisect_left
from datetime import datetime, timezone
import math
from pathlib import Path
import threading
import time
from typing import Sequence, TYPE_CHECKING
import warnings

from assembly_scene_publisher.py_modules.tf_functions import get_transform_for_frame_in_world
from geometry_msgs.msg import Pose, Quaternion, Vector3
import pm_moveit_interfaces.srv as pm_moveit_srv
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_skills.py_modules.pm_skill_domain import PmSkillDomain
from pm_skills.py_modules.PmRobotUtils import PmRobotUtils
import pm_skills_interfaces.action as pm_skill_action
from pm_skills_interfaces.msg import BoolStamped, Float64Stamped
from rclpy.action import CancelResponse, GoalResponse
from sensor_msgs.msg import JointState

if TYPE_CHECKING:
    from rclpy.action.server import ServerGoalHandle
    from pm_skills.pm_skills import PmSkills


class AlignmentSearchCancelled(Exception):
    """Raised when an active search is cancelled."""


class PmAlignmentSearchSkills(PmSkillDomain):
    """Find the maximum signal along a frame-oriented rectangular spiral."""

    ROBOT_JOINTS = (
        PmRobotUtils.X_Axis_JOINT_NAME,
        PmRobotUtils.Y_Axis_JOINT_NAME,
        PmRobotUtils.Z_Axis_JOINT_NAME,
    )
    SMARPOD_JOINTS = (
        'SP_X_Joint', 'SP_Y_Joint', 'SP_Z_Joint',
        'SP_A_Joint', 'SP_B_Joint', 'SP_C_Joint',
    )
    MAX_TURNS = 100
    SIGNAL_TIMEOUT_S = 10.0

    def __init__(self, node: 'PmSkills') -> None:
        """Initialize planning clients and signal-sampling state."""
        super().__init__(node)
        self._move_smarpod_to_pose_client = node.create_client(
            pm_moveit_srv.MoveToPose,
            '/pm_moveit_server/move_smarpod_to_pose',
        )
        self._move_tool_to_pose_client = node.create_client(
            pm_moveit_srv.MoveToPose,
            '/pm_moveit_server/move_tool_to_pose',
        )
        self._active_lock = threading.Lock()
        self._active = False
        self._signal_lock = threading.Lock()
        self._signal_event = threading.Event()
        self._position_event = threading.Event()
        self._latest_signal: float | bool | None = None
        self._latest_acquisition_ros_time_ns: int | None = None
        self._latest_ros_time_ns: int | None = None
        self._latest_wall_time_ns: int | None = None
        self._signal_is_bool: bool | None = None
        self._recording = False
        self._sampling_joint_names: Sequence[str] = ()
        self._center_joints: Sequence[float] = ()
        self._frame_orientation = Quaternion(w=1.0)
        self._signal_samples: list[
            tuple[float | bool, int, int, int]
        ] = []
        self._position_samples: list[
            tuple[int, tuple[float, float, float]]
        ] = []

    @staticmethod
    def _active_axes(goal: pm_skill_action.RectSpiralSearch.Goal) -> list[int]:
        extents = (goal.x_um, goal.y_um, goal.z_um)
        return [index for index, extent in enumerate(extents) if extent > 0.0]

    @classmethod
    def _goal_error(cls, goal: pm_skill_action.RectSpiralSearch.Goal) -> str | None:
        extents = (goal.x_um, goal.y_um, goal.z_um)
        if any(not math.isfinite(value) or value < 0.0 for value in extents):
            return 'Search extents must be finite and non-negative.'
        if len(cls._active_axes(goal)) != 2:
            return 'Exactly two search extents must be positive and the third must be zero.'
        if goal.spiral_turns < 1 or goal.spiral_turns > cls.MAX_TURNS:
            return f'spiral_turns must be between 1 and {cls.MAX_TURNS}.'
        if not math.isfinite(goal.segment_time_s) or goal.segment_time_s <= 0.0:
            return 'segment_time_s must be finite and positive.'
        if not goal.frame_name:
            return 'frame_name must not be empty.'
        if not cls._alignment_topic(goal):
            return 'alignment_topic must not be empty.'
        return None

    @staticmethod
    def _rsap_path(goal: pm_skill_action.RectSpiralSearch.Goal) -> str:
        """Return the configured output root."""
        return goal.rsap_path

    @staticmethod
    def _alignment_topic(goal: pm_skill_action.RectSpiralSearch.Goal) -> str:
        """Return the configured timestamped alignment topic."""
        return goal.alignment_topic

    def goal_callback(self, goal_request: pm_skill_action.RectSpiralSearch.Goal) -> GoalResponse:
        """Validate and reserve an incoming search goal."""
        error = self._goal_error(goal_request)
        if error:
            self._logger.error(f'Rejecting rectangular spiral search: {error}')
            return GoalResponse.REJECT
        with self._active_lock:
            if self._active:
                self._logger.warning('Another alignment search is already active.')
                return GoalResponse.REJECT
            self._active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle: 'ServerGoalHandle') -> CancelResponse:
        """Accept cancellation; execution checks between controller goals."""
        return CancelResponse.ACCEPT

    @staticmethod
    def generate_rect_spiral(
        extents_um: Sequence[float],
        turns: int,
    ) -> list[tuple[float, float, float]]:
        """Generate exactly ``turns`` center-out rectangular coils."""
        axes = [index for index, value in enumerate(extents_um) if value > 0.0]
        if len(axes) != 2 or turns < 1:
            raise ValueError('A spiral requires two active axes and at least one turn.')
        first_axis, second_axis = axes
        first_half = extents_um[first_axis] / 2.0
        second_half = extents_um[second_axis] / 2.0
        points_2d: list[tuple[float, float]] = [(0.0, 0.0)]
        previous_second = 0.0
        for turn in range(1, turns + 1):
            fraction = turn / turns
            first = first_half * fraction
            second = second_half * fraction
            points_2d.extend([
                (first, -previous_second),
                (first, second),
                (-first, second),
                (-first, -second),
            ])
            previous_second = second
        points_2d.append((first_half, -second_half))

        points = []
        for first, second in points_2d:
            offset = [0.0, 0.0, 0.0]
            offset[first_axis] = first
            offset[second_axis] = second
            points.append(tuple(offset))
        return points

    @staticmethod
    def _rotate(vector: Sequence[float], quaternion: Quaternion) -> tuple[float, float, float]:
        """Rotate a vector by a normalized quaternion."""
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            raise PmRobotError('The selected frame has an invalid orientation.')
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        vx, vy, vz = vector
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return (
            vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx,
        )

    @classmethod
    def _inverse_rotate(
        cls,
        vector: Sequence[float],
        quaternion: Quaternion,
    ) -> tuple[float, float, float]:
        inverse = Quaternion(
            x=-quaternion.x,
            y=-quaternion.y,
            z=-quaternion.z,
            w=quaternion.w,
        )
        return cls._rotate(vector, inverse)

    def _record_position_sample(
        self,
        positions: Sequence[float],
        ros_time_ns: int,
    ) -> int | None:
        """Store one timestamped XYZ position read from joint feedback."""
        if len(positions) < 3 or any(
            position is None for position in positions[:3]
        ):
            return None
        with self._signal_lock:
            self._position_samples.append((ros_time_ns, tuple(positions[:3])))
        self._position_event.set()
        return ros_time_ns

    def _joint_state_callback(self, message: JointState) -> None:
        """Record joint feedback at its acquisition time when available."""
        positions_by_name = dict(zip(message.name, message.position))
        positions = [
            positions_by_name.get(name) for name in self._sampling_joint_names[:3]
        ]
        if not self._message_has_stamp(message):
            self._logger.error(
                'Ignoring joint-state sample without an acquisition timestamp.'
            )
            return
        self._record_position_sample(
            positions,
            self._message_stamp_ns(message),
        )

    @staticmethod
    def _message_stamp_ns(message: object) -> int:
        """Return a required, non-zero ROS header stamp."""
        header = getattr(message, 'header', None)
        stamp = getattr(header, 'stamp', None)
        if stamp is None:
            raise ValueError('Message has no acquisition timestamp.')
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= 0:
            raise ValueError('Message has no acquisition timestamp.')
        return stamp_ns

    @staticmethod
    def _message_has_stamp(message: object) -> bool:
        """Return whether a message contains a non-zero ROS header stamp."""
        header = getattr(message, 'header', None)
        stamp = getattr(header, 'stamp', None)
        return stamp is not None and (stamp.sec != 0 or stamp.nanosec != 0)

    @classmethod
    def _synchronize_samples(
        cls,
        signal_samples: Sequence[
            tuple[float | bool, int, int, int]
        ],
        position_samples: Sequence[tuple[int, Sequence[float]]],
        center_joints: Sequence[float],
        orientation: Quaternion,
    ) -> list[tuple]:
        """Interpolate only between actual feedback samples."""
        if not signal_samples or not position_samples:
            return []
        ordered_positions = sorted(
            position_samples, key=lambda sample: sample[0]
        )
        position_times = [sample[0] for sample in ordered_positions]
        synchronized = []
        for (
            value,
            acquisition_time_ns,
            receive_time_ns,
            wall_time_ns,
        ) in signal_samples:
            right_index = bisect_left(position_times, acquisition_time_ns)
            if (
                right_index < len(ordered_positions)
                and position_times[right_index] == acquisition_time_ns
            ):
                position = tuple(ordered_positions[right_index][1][:3])
            elif right_index == 0 or right_index == len(ordered_positions):
                # Never extrapolate or clamp a measurement to an assumed pose.
                continue
            else:
                left_time, left_position = ordered_positions[right_index - 1]
                right_time, right_position = ordered_positions[right_index]
                if right_time == left_time:
                    position = tuple(right_position[:3])
                else:
                    fraction = (
                        (acquisition_time_ns - left_time)
                        / (right_time - left_time)
                    )
                    position = tuple(
                        left_position[axis]
                        + fraction * (right_position[axis] - left_position[axis])
                        for axis in range(3)
                    )
            world_offset_um = tuple(
                (position[axis] - center_joints[axis]) * 1e6
                for axis in range(3)
            )
            local_offset_um = cls._inverse_rotate(world_offset_um, orientation)
            synchronized.append((
                local_offset_um,
                value,
                receive_time_ns,
                wall_time_ns,
                position,
                acquisition_time_ns,
            ))
        return synchronized

    def _wait_for_position_sample_after(
        self,
        minimum_time_ns: int,
        timeout_s: float | None = None,
    ) -> tuple[int, tuple[float, float, float]]:
        """Wait for actual joint feedback acquired after a time boundary."""
        timeout = self.SIGNAL_TIMEOUT_S if timeout_s is None else timeout_s
        deadline = time.monotonic() + timeout
        while True:
            self._position_event.clear()
            with self._signal_lock:
                if self._position_samples:
                    newest = max(
                        self._position_samples,
                        key=lambda sample: sample[0],
                    )
                    if newest[0] >= minimum_time_ns:
                        return newest
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0.0 or not self._position_event.wait(remaining_s):
                raise PmRobotError(
                    'Timed out waiting for timestamped joint feedback after '
                    'the trajectory boundary.'
                )

    def _signal_callback(
        self,
        message: BoolStamped | Float64Stamped,
    ) -> None:
        receive_time_ns = self.node.get_clock().now().nanoseconds
        wall_time_ns = time.time_ns()
        if not self._message_has_stamp(message):
            self._logger.error(
                'Ignoring alignment sample without an acquisition timestamp.'
            )
            return
        acquisition_time_ns = self._message_stamp_ns(
            message
        )
        with self._signal_lock:
            message_is_bool = isinstance(message, BoolStamped)
            if self._signal_is_bool is None:
                self._signal_is_bool = message_is_bool
                if message_is_bool:
                    detected_type = 'pm_skills_interfaces/BoolStamped'
                else:
                    detected_type = 'pm_skills_interfaces/Float64Stamped'
                self._logger.info(f'Alignment topic type detected as {detected_type}.')
            elif self._signal_is_bool != message_is_bool:
                return
            self._latest_signal = message.data
            self._latest_acquisition_ros_time_ns = acquisition_time_ns
            self._latest_ros_time_ns = receive_time_ns
            self._latest_wall_time_ns = wall_time_ns
            if self._recording:
                self._signal_samples.append((
                    message.data,
                    acquisition_time_ns,
                    receive_time_ns,
                    wall_time_ns,
                ))
            self._signal_event.set()

    def _wait_for_signal(self, timeout_s: float | None = None) -> float | bool:
        timeout = self.SIGNAL_TIMEOUT_S if timeout_s is None else timeout_s
        if timeout <= 0.0 or not self._signal_event.wait(timeout):
            raise PmRobotError('Timed out waiting for the alignment topic.')
        with self._signal_lock:
            if self._latest_signal is None:
                raise PmRobotError('The alignment topic has not supplied a value.')
            return self._latest_signal

    def _wait_for_signal_acquired_after(
        self,
        minimum_time_ns: int,
        timeout_s: float | None = None,
    ) -> float | bool:
        """Wait for a signal whose source acquisition meets the boundary."""
        timeout = self.SIGNAL_TIMEOUT_S if timeout_s is None else timeout_s
        deadline = time.monotonic() + timeout
        while True:
            self._signal_event.clear()
            self._position_event.clear()
            with self._signal_lock:
                if (
                    self._latest_signal is not None
                    and self._latest_acquisition_ros_time_ns is not None
                    and self._latest_acquisition_ros_time_ns >= minimum_time_ns
                ):
                    return self._latest_signal
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0.0 or not self._signal_event.wait(remaining_s):
                raise PmRobotError(
                    'Timed out waiting for an alignment sample acquired after '
                    'the trajectory boundary.'
                )

    def _detect_signal_type(
        self,
        topic: str,
        deadline: float,
    ) -> tuple[type[BoolStamped] | type[Float64Stamped], bool]:
        """Discover the single supported type advertised for a topic."""
        supported_types = {
            'pm_skills_interfaces/msg/BoolStamped': (BoolStamped, True),
            'pm_skills_interfaces/msg/Float64Stamped': (
                Float64Stamped,
                False,
            ),
        }
        while time.monotonic() < deadline:
            publisher_info = self.node.get_publishers_info_by_topic(topic)
            advertised_types = {
                publisher.topic_type for publisher in publisher_info
            }
            matches = advertised_types.intersection(supported_types)
            if len(matches) > 1:
                raise PmRobotError(
                    f"Alignment topic '{topic}' has multiple supported types."
                )
            if len(matches) == 1:
                return supported_types[matches.pop()]
            if advertised_types:
                raise PmRobotError(
                    f"Alignment topic '{topic}' has unsupported types: "
                    f'{sorted(advertised_types)}'
                )
            time.sleep(0.05)
        raise PmRobotError(
            f'Timed out after {self.SIGNAL_TIMEOUT_S:.1f} s waiting for '
            f"an alignment publisher on '{topic}'."
        )

    @classmethod
    def _pose_with_local_offset(
        cls,
        base_pose: Pose,
        local_offset_um: Sequence[float],
    ) -> Pose:
        world_offset_um = cls._rotate(local_offset_um, base_pose.orientation)
        pose = Pose()
        pose.position.x = base_pose.position.x + world_offset_um[0] * 1e-6
        pose.position.y = base_pose.position.y + world_offset_um[1] * 1e-6
        pose.position.z = base_pose.position.z + world_offset_um[2] * 1e-6
        pose.orientation = base_pose.orientation
        return pose

    @staticmethod
    def _joint_map(response: pm_moveit_srv.MoveToPose.Response) -> dict[str, float]:
        if len(response.joint_names) != len(response.joint_values):
            raise PmRobotError('MoveIt returned mismatched joint names and values.')
        return dict(zip(response.joint_names, response.joint_values))

    def _plan_pose(
        self,
        pose: Pose,
        use_hexapod: bool,
        endeffector_frame: str,
        local_offset_um: Sequence[float],
        check_index: int | None = None,
        check_count: int | None = None,
    ) -> dict[str, float]:
        """Check a target pose for the selected moving assembly frame."""
        client = (
            self._move_smarpod_to_pose_client
            if use_hexapod else self._move_tool_to_pose_client
        )
        if not client.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{client.srv_name}' is not available.")
        request = pm_moveit_srv.MoveToPose.Request()
        request.move_to_pose = pose
        request.endeffector_frame_override = endeffector_frame
        request.execute_movement = False
        mover = 'SmarPod' if use_hexapod else 'robot tool'
        self._logger.info(
            f'Checking {mover} pose for frame {endeffector_frame!r}: '
            f'local_offset_um=({local_offset_um[0]:.3f}, '
            f'{local_offset_um[1]:.3f}, {local_offset_um[2]:.3f}), '
            f'world_position_m=({pose.position.x:.9f}, '
            f'{pose.position.y:.9f}, {pose.position.z:.9f}), '
            f'world_orientation_xyzw=({pose.orientation.x:.6f}, '
            f'{pose.orientation.y:.6f}, {pose.orientation.z:.6f}, '
            f'{pose.orientation.w:.6f}).'
        )
        response = client.call(request)
        if not response.success:
            is_initial_pose = all(
                math.isclose(value, 0.0, abs_tol=1e-9)
                for value in local_offset_um
            )
            check_name = (
                'initial search position'
                if is_initial_pose
                else 'search boundary corner'
            )
            check_progress = ''
            if check_index is not None and check_count is not None:
                check_progress = f' {check_index}/{check_count}'
            moveit_message = response.message.strip() or 'no details supplied'
            normalized_message = moveit_message.lower()
            if 'collisions between:' in normalized_message:
                diagnosis = (
                    'MoveIt reported collision contacts; inspect the listed '
                    'link pairs and the Allowed Collision Matrix.'
                )
            elif 'ik solution not found' in normalized_message:
                diagnosis = (
                    'MoveIt could not find inverse kinematics for this pose. '
                    'Verify the frame transform and SmarPod workspace/joint limits.'
                )
            elif 'planing failed' in normalized_message or 'planning failed' in normalized_message:
                diagnosis = (
                    'Inverse kinematics succeeded, but MoveIt could not plan from '
                    'the current state to that joint target. No collision contacts '
                    'were reported. Verify that the current joint state, TF, and '
                    'planning scene agree; inspect joint limits and the state in '
                    'RViz; then retry. A planner timeout can also cause this.'
                )
            else:
                diagnosis = (
                    'MoveIt rejected the pose. Inspect the pm_moveit server log and '
                    'the planning scene in RViz for the underlying reason.'
                )
            raise PmRobotError(
                f'Preflight pose check{check_progress} failed at the {check_name} '
                f'for {mover} frame {endeffector_frame!r}. '
                f'Local offset=({local_offset_um[0]:.3f}, '
                f'{local_offset_um[1]:.3f}, {local_offset_um[2]:.3f}) um; '
                f'world target position=({pose.position.x:.9f}, '
                f'{pose.position.y:.9f}, {pose.position.z:.9f}) m, '
                f'orientation=({pose.orientation.x:.6f}, '
                f'{pose.orientation.y:.6f}, {pose.orientation.z:.6f}, '
                f'{pose.orientation.w:.6f}) xyzw. {diagnosis} '
                f'MoveIt response: {moveit_message!r}. The alignment subscription '
                f'was not started and no search motion was executed.'
            )
        return self._joint_map(response)

    def _move(
        self,
        values: Sequence[float],
        use_hexapod: bool,
        duration_s: float,
    ) -> None:
        if use_hexapod:
            success = self.pm_robot_utils.send_smarpod_trajectory_goal_absolut(
                values[0], values[1], values[2],
                math.degrees(values[3]), math.degrees(values[4]),
                math.degrees(values[5]), duration_s,
            )
        else:
            success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(
                values[0], values[1], values[2], duration_s,
            )
        if not success:
            raise PmRobotError('Trajectory controller did not reach the spiral waypoint.')

    def _move_trajectory(
        self,
        targets: Sequence[Sequence[float]],
        durations_s: Sequence[float],
        use_hexapod: bool,
        goal_handle: 'ServerGoalHandle',
    ) -> None:
        """Send the complete spiral as one cancellable controller goal."""
        cancel_requested = lambda: goal_handle.is_cancel_requested
        if use_hexapod:
            success = (
                self.pm_robot_utils.send_smarpod_trajectory_goal_absolut_multi(
                    targets,
                    durations_s,
                    cancel_requested,
                )
            )
        else:
            success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut_multi(
                targets,
                durations_s,
                cancel_requested,
            )
        if goal_handle.is_cancel_requested:
            raise AlignmentSearchCancelled()
        if not success:
            raise PmRobotError(
                'Trajectory controller did not complete the continuous spiral.'
            )

    def _target_joints(
        self,
        center_joints: Sequence[float],
        local_offset_um: Sequence[float],
        orientation: Quaternion,
    ) -> list[float]:
        target = list(center_joints)
        world_offset_um = self._rotate(local_offset_um, orientation)
        for axis in range(3):
            target[axis] += world_offset_um[axis] * 1e-6
        return target

    @staticmethod
    def _segment_length(
        start: Sequence[float],
        end: Sequence[float],
    ) -> float:
        """Return the Euclidean length between two local offsets."""
        return math.sqrt(sum((b - a) ** 2 for a, b in zip(start, end)))

    @staticmethod
    def _best_result(
        samples: Sequence[tuple[Sequence[float], float | bool]],
        bool_signal: bool,
    ) -> tuple[tuple[float, float, float], float | bool]:
        """Select a result, falling back to the initial pose if no contrast exists."""
        if bool_signal:
            bool_values = [bool(value) for _, value in samples]
            if all(value == bool_values[0] for value in bool_values):
                return (0.0, 0.0, 0.0), bool_values[0]
            true_offsets = [offset for offset, value in samples if bool(value)]
            count = len(true_offsets)
            centroid = tuple(
                sum(offset[axis] for offset in true_offsets) / count
                for axis in range(3)
            )
            return centroid, True
        float_values = [float(value) for _, value in samples]
        if all(value == float_values[0] for value in float_values):
            return (0.0, 0.0, 0.0), float_values[0]
        best_offset, best_value = max(samples, key=lambda sample: sample[1])
        return tuple(best_offset), best_value

    @staticmethod
    def _float_plot_order(values: Sequence[float]) -> list[int]:
        """Return indices from smallest to largest finite plotted value."""
        return sorted(
            range(len(values)),
            key=lambda index: (
                values[index] if math.isfinite(values[index]) else -math.inf
            ),
        )

    @staticmethod
    def _output_image_path(
        path_text: str,
        timestamp: str | None = None,
    ) -> str:
        """Create a folder for one plot/CSV measurement pair."""
        requested_path = Path(path_text).expanduser()
        image_suffixes = {
            '.eps', '.jpeg', '.jpg', '.pdf', '.pgf', '.png', '.ps', '.raw',
            '.rgba', '.svg', '.svgz', '.tif', '.tiff', '.webp',
        }
        if requested_path.suffix.lower() in image_suffixes:
            output_root = requested_path.parent
            image_stem = requested_path.stem
            image_suffix = requested_path.suffix
        else:
            output_root = requested_path
            image_stem = 'rect_spiral_search'
            image_suffix = '.png'
        run_timestamp = timestamp or datetime.now().strftime(
            '%Y%m%d_%H%M%S_%f'
        )
        output_directory = output_root / 'rect_spiral_search' / run_timestamp
        output_directory.mkdir(parents=True, exist_ok=False)
        image_name = f'{image_stem}{image_suffix}'
        return str((output_directory / image_name).resolve())

    @staticmethod
    def _save_plot(
        path_text: str,
        axes: Sequence[int],
        path_offsets: Sequence[Sequence[float]],
        samples: Sequence[tuple],
        bool_signal: bool,
        selected_offset: Sequence[float],
    ) -> str:
        """Save the signal map and visibly mark the selected result."""
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore', message='Unable to import Axes3D.*')
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

        path = Path(path_text).expanduser()
        supported_suffixes = {
            '.eps', '.jpeg', '.jpg', '.pdf', '.pgf', '.png', '.ps', '.raw',
            '.rgba', '.svg', '.svgz', '.tif', '.tiff', '.webp',
        }
        if not path.suffix:
            path = path.with_suffix('.png')
        elif path.suffix.lower() not in supported_suffixes:
            path = path.with_name(f'{path.stem}_rect_spiral_search.png')
        path.parent.mkdir(parents=True, exist_ok=True)
        axis_names = ('X', 'Y', 'Z')
        first_axis, second_axis = axes
        path_first = [point[first_axis] for point in path_offsets]
        path_second = [point[second_axis] for point in path_offsets]
        sample_first = [sample[0][first_axis] for sample in samples]
        sample_second = [sample[0][second_axis] for sample in samples]
        values = [sample[1] for sample in samples]

        figure, plot = plt.subplots(figsize=(8, 7))
        plot.plot(path_first, path_second, color='0.75', linewidth=1.0, label='spiral')
        if bool_signal:
            for state, color, marker in ((False, 'tab:red', 'x'), (True, 'tab:green', 'o')):
                indices = [index for index, value in enumerate(values) if bool(value) is state]
                if indices:
                    plot.scatter(
                        [sample_first[index] for index in indices],
                        [sample_second[index] for index in indices],
                        c=color, marker=marker, s=24, label=str(state),
                    )
        else:
            colors = [float(value) for value in values]
            # Draw low values first so higher-value samples remain visible when
            # several measurements share nearly the same position.
            plot_order = PmAlignmentSearchSkills._float_plot_order(colors)
            collection = plot.scatter(
                [sample_first[index] for index in plot_order],
                [sample_second[index] for index in plot_order],
                c=[colors[index] for index in plot_order],
                cmap='viridis',
                s=18,
            )
            figure.colorbar(collection, ax=plot, label='Alignment value')
        plot.scatter(
            [selected_offset[first_axis]],
            [selected_offset[second_axis]],
            s=160,
            facecolors='none',
            edgecolors='magenta',
            linewidths=2.0,
            marker='o',
            label='selected result',
            zorder=6,
        )
        plot.legend(
            loc='upper center',
            bbox_to_anchor=(0.5, -0.12),
            ncol=3,
        )
        plot.set_xlabel(f'Frame {axis_names[first_axis]} offset [µm]')
        plot.set_ylabel(f'Frame {axis_names[second_axis]} offset [µm]')
        plot.set_title('Rectangular spiral alignment map')
        plot.set_aspect('equal', adjustable='box')
        plot.grid(True, alpha=0.25)
        figure.tight_layout()
        figure.savefig(path, dpi=300, bbox_inches='tight')
        plt.close(figure)
        return str(path.resolve())

    @staticmethod
    def _save_csv(
        image_path: str,
        samples: Sequence[tuple],
    ) -> str:
        """Save timestamped measurements and positions beside the plot."""
        path = Path(image_path).with_suffix('.csv')
        with path.open('w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                'sample_index',
                'ros_time_ns',
                'ros_time_s',
                'acquisition_ros_time_ns',
                'acquisition_ros_time_s',
                'acquisition_to_receive_ms',
                'wall_time_unix_ns',
                'wall_time_utc',
                'value',
                'local_offset_x_um',
                'local_offset_y_um',
                'local_offset_z_um',
                'controller_position_x_m',
                'controller_position_y_m',
                'controller_position_z_m',
            ])
            for index, sample in enumerate(samples):
                (
                    offset,
                    value,
                    receive_time_ns,
                    wall_time_ns,
                    position,
                    acquisition_time_ns,
                ) = sample
                wall_time_utc = datetime.fromtimestamp(
                    wall_time_ns / 1e9,
                    tz=timezone.utc,
                ).isoformat(timespec='microseconds')
                writer.writerow([
                    index,
                    receive_time_ns,
                    f'{receive_time_ns / 1e9:.9f}',
                    acquisition_time_ns,
                    f'{acquisition_time_ns / 1e9:.9f}',
                    (receive_time_ns - acquisition_time_ns) / 1e6,
                    wall_time_ns,
                    wall_time_utc,
                    value,
                    offset[0],
                    offset[1],
                    offset[2],
                    position[0],
                    position[1],
                    position[2],
                ])
        return str(path.resolve())

    @staticmethod
    def _check_cancel(goal_handle: 'ServerGoalHandle') -> None:
        if goal_handle.is_cancel_requested:
            raise AlignmentSearchCancelled()

    def rect_spiral_search(
        self,
        goal_handle: 'ServerGoalHandle',
    ) -> pm_skill_action.RectSpiralSearch.Result:
        """Execute, continuously sample, plot, and return to the maximum."""
        result = pm_skill_action.RectSpiralSearch.Result()
        subscriptions = []
        path_offsets: list[tuple[float, float, float]] = []
        try:
            goal = goal_handle.request
            error = self._goal_error(goal)
            if error:
                raise PmRobotError(error)
            analyzer = self.pm_robot_utils.assembly_scene_analyzer
            analyzer.wait_for_initial_scene_update()
            if not analyzer.is_frame_from_scene(goal.frame_name):
                raise PmRobotError(f"Frame '{goal.frame_name}' is not in the assembly scene.")
            on_hexapod = analyzer.frame_is_on_smarpod(goal.frame_name)
            on_gripper = analyzer.frames_is_on_gripper(goal.frame_name)
            if on_hexapod == on_gripper:
                raise PmRobotError('The frame must be on exactly one mover.')
            result.used_hexapod = on_hexapod

            extents = (goal.x_um, goal.y_um, goal.z_um)
            path_offsets = self.generate_rect_spiral(extents, goal.spiral_turns)
            segment_lengths = [
                self._segment_length(start, end)
                for start, end in zip(path_offsets, path_offsets[1:])
            ]
            longest_segment = max(segment_lengths)
            segment_durations = [
                goal.segment_time_s * length / longest_segment
                for length in segment_lengths
            ]
            commanded_speed_um_s = longest_segment / goal.segment_time_s
            transform = get_transform_for_frame_in_world(
                goal.frame_name, self.tf_buffer, self._logger
            )
            base_pose = Pose()
            base_pose.position.x = transform.transform.translation.x
            base_pose.position.y = transform.transform.translation.y
            base_pose.position.z = transform.transform.translation.z
            base_pose.orientation = transform.transform.rotation

            active_axes = self._active_axes(goal)
            corner_offsets = [
                offset for offset in path_offsets
                if all(
                    math.isclose(abs(offset[axis]), extents[axis] / 2.0)
                    for axis in active_axes
                )
            ]
            checked_offsets = list(dict.fromkeys([path_offsets[0]] + corner_offsets))
            if len(checked_offsets) != 5:
                raise PmRobotError('Could not construct all four collision-check corners.')
            checked = {}
            for check_index, offset in enumerate(checked_offsets, start=1):
                checked[offset] = self._plan_pose(
                    self._pose_with_local_offset(base_pose, offset),
                    on_hexapod,
                    goal.frame_name,
                    offset,
                    check_index,
                    len(checked_offsets),
                )
            joint_names = self.SMARPOD_JOINTS if on_hexapod else self.ROBOT_JOINTS
            center_map = checked[path_offsets[0]]
            missing = [name for name in joint_names if name not in center_map]
            if missing:
                raise PmRobotError(f'MoveIt did not return required joints: {missing}')
            center_joints = [center_map[name] for name in joint_names]
            self._sampling_joint_names = joint_names
            self._center_joints = center_joints
            self._frame_orientation = base_pose.orientation

            topic = self._alignment_topic(goal)
            signal_deadline = time.monotonic() + self.SIGNAL_TIMEOUT_S
            message_type, signal_is_bool = self._detect_signal_type(
                topic, signal_deadline
            )
            self._signal_event.clear()
            with self._signal_lock:
                self._latest_signal = None
                self._latest_acquisition_ros_time_ns = None
                self._latest_ros_time_ns = None
                self._latest_wall_time_ns = None
                self._signal_is_bool = signal_is_bool
                self._signal_samples = []
                self._position_samples = []
                self._recording = False
            subscriptions = [
                self.node.create_subscription(
                    JointState, '/joint_states', self._joint_state_callback, 10
                ),
                self.node.create_subscription(
                    message_type, topic, self._signal_callback, 10
                ),
            ]
            # Start consuming the signal only after every collision-check pose
            # has been calculated, but still prove the source is alive before
            # moving.
            self._wait_for_signal(signal_deadline - time.monotonic())

            self._move(center_joints, on_hexapod, goal.segment_time_s)
            center_boundary_ns = self.node.get_clock().now().nanoseconds
            center_feedback_ns, measured_center_position = (
                self._wait_for_position_sample_after(
                    center_boundary_ns
                )
            )
            measurement_center_joints = list(center_joints)
            measurement_center_joints[:3] = measured_center_position
            self._center_joints = measurement_center_joints
            self._wait_for_signal_acquired_after(center_feedback_ns)
            with self._signal_lock:
                if (
                    self._latest_signal is None
                    or self._latest_acquisition_ros_time_ns is None
                    or self._latest_ros_time_ns is None
                    or self._latest_wall_time_ns is None
                ):
                    raise PmRobotError(
                        'The alignment sample has no receive timestamp.'
                    )
                self._signal_samples.append((
                    self._latest_signal,
                    self._latest_acquisition_ros_time_ns,
                    self._latest_ros_time_ns,
                    self._latest_wall_time_ns,
                ))
                self._recording = True

            targets = [
                self._target_joints(
                    measurement_center_joints,
                    offset,
                    base_pose.orientation,
                )
                for offset in path_offsets[1:]
            ]
            self._check_cancel(goal_handle)
            self._signal_event.clear()
            self._move_trajectory(
                targets,
                segment_durations,
                on_hexapod,
                goal_handle,
            )
            trajectory_boundary_ns = self.node.get_clock().now().nanoseconds
            endpoint_feedback_ns, _ = self._wait_for_position_sample_after(
                trajectory_boundary_ns
            )
            latest_value = self._wait_for_signal_acquired_after(
                endpoint_feedback_ns
            )
            with self._signal_lock:
                self._recording = False
                signal_samples = list(self._signal_samples)
                signal_is_bool = bool(self._signal_is_bool)
                sample_count = len(self._signal_samples)
            if not signal_samples:
                raise PmRobotError(
                    'No timestamped alignment samples were recorded.'
                )
            final_signal_time_ns = max(sample[1] for sample in signal_samples)
            # Ensure the final recorded signal also has real feedback on its
            # right side, so synchronization never needs extrapolation.
            self._wait_for_position_sample_after(final_signal_time_ns)
            with self._signal_lock:
                position_samples = list(self._position_samples)
            feedback = pm_skill_action.RectSpiralSearch.Feedback()
            feedback.current_point = len(path_offsets)
            feedback.total_points = len(path_offsets)
            final_offset = path_offsets[-1]
            feedback.current_offset_um = Vector3(
                x=final_offset[0], y=final_offset[1], z=final_offset[2]
            )
            feedback.current_value = float(latest_value)
            feedback.current_bool_value = bool(latest_value)
            feedback.sample_count = sample_count
            goal_handle.publish_feedback(feedback)

            samples = self._synchronize_samples(
                signal_samples,
                position_samples,
                measurement_center_joints,
                base_pose.orientation,
            )
            discarded_sample_count = len(signal_samples) - len(samples)
            if discarded_sample_count:
                self._logger.warning(
                    f'Discarded {discarded_sample_count} alignment samples '
                    'that were not bracketed by actual joint feedback.'
                )
            if not samples:
                raise PmRobotError('No positioned alignment samples were received.')
            valid_samples = [
                sample for sample in samples
                if isinstance(sample[1], bool) or math.isfinite(float(sample[1]))
            ]
            if not valid_samples:
                raise PmRobotError('No finite alignment samples were received.')
            best_offset, best_value = self._best_result(
                [(sample[0], sample[1]) for sample in valid_samples],
                signal_is_bool,
            )
            return_duration_s = (
                self._segment_length(path_offsets[-1], best_offset)
                / commanded_speed_um_s
            )
            self._move(
                self._target_joints(
                    measurement_center_joints,
                    best_offset,
                    base_pose.orientation,
                ),
                on_hexapod,
                max(return_duration_s, 0.001),
            )
            requested_plot_path = self._rsap_path(goal)
            saved_path = ''
            if requested_plot_path:
                output_image_path = self._output_image_path(
                    requested_plot_path
                )
                saved_path = self._save_plot(
                    output_image_path,
                    active_axes,
                    path_offsets,
                    samples,
                    signal_is_bool,
                    best_offset,
                )
                self._save_csv(saved_path, samples)
            result.rsap_path = saved_path
            result.success = True
            result.message = 'Rectangular spiral search completed.'
            result.best_value = float(best_value)
            result.best_bool_value = bool(best_value)
            result.best_offset_um = Vector3(
                x=best_offset[0], y=best_offset[1], z=best_offset[2]
            )
            result.sample_offset_x_um = [sample[0][0] for sample in samples]
            result.sample_offset_y_um = [sample[0][1] for sample in samples]
            result.sample_offset_z_um = [sample[0][2] for sample in samples]
            result.sample_values = [float(sample[1]) for sample in samples]
            result.visited_points = len(path_offsets)
            goal_handle.succeed()
        except AlignmentSearchCancelled:
            result.message = 'Rectangular spiral search cancelled.'
            goal_handle.canceled()
        except Exception as error:  # Return execution errors through the action.
            result.message = str(error)
            self._logger.error(f'Rectangular spiral search failed: {error}')
            goal_handle.abort()
        finally:
            with self._signal_lock:
                self._recording = False
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)
            with self._active_lock:
                self._active = False
        return result
