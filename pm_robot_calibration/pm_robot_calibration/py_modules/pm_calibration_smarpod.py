from pathlib import Path
import json
import math
import random
from rclpy.node import Node
import pm_skills_interfaces.srv as skills_srv
import pm_skills_interfaces.action as skills_action
from geometry_msgs.msg import Transform
import assembly_manager_interfaces.srv as ami_srv
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_robot_primitive_skills.py_modules.PmRobotMeasurementError import PmRobotMeasurementError
from pm_robot_calibration.py_modules.hexapod_calibration.sphere_calibration import SphereCalibration
from pm_robot_calibration.py_modules.hexapod_calibration.calibration_analysis import CalibrationAnalysis
from pm_robot_calibration.py_modules.hexapod_calibration.calibration_comparison_report import (
    generate_hexapod_calibration_comparison_report,
)
from pm_robot_calibration.py_modules.hexapod_calibration.geometry_utils import sphere_z
from pm_robot_calibration.py_modules.hexapod_calibration.hexapod_calibration_runtime import (
    get_calibrate_smarpod_measurement_file_path,
    get_hexapod_repeatability_measurement_file_path,
    get_hexapod_repeatability_plot_base_path,
    get_hexapod_repeatability_results_dir,
    get_hexapod_repeatability_results_json_path,
    get_hexapod_calibration_positions_mm,
    get_hexapod_calibration_test_poses,
    get_hexapod_orientation_commands,
    get_smarpod_ball_repeatability_measurement_file_path,
    get_smarpod_ball_repeatability_plot_base_path,
    get_smarpod_ball_repeatability_results_dir,
    get_smarpod_ball_repeatability_results_json_path,
    get_smarpod_results_base_path,
    get_smarpod_results_dir,
    get_smarpod_results_json_path,
    get_test_calibrate_smarpod_file_path,
    get_test_calibrate_smarpod_plot_path,
    write_json_file,
)
from pm_robot_calibration.py_modules.hexapod_calibration.pivot_calibration_plot import (
    plot_hexapod_repeatability_results,
    plot_smarpod_ball_repeatability_results,
    plot_smarpod_calibration_test_measurements,
)
from assembly_scene_publisher.py_modules.geometry_functions import (
    multiply_ros_transforms,
    inverse_ros_transform,
)
import time
import datetime
from pm_robot_calibration.py_modules.pm_calibration_utils import (
    PmRobotCalibrationUtils,
    CancelCalibrationException,
)

class HexapodConstants:
    FIXED_CS_SMARPOD_FRAME = "Smarpod_Station_Base_Calibration"
    CALIBRATED_CS_SMARPOD_FRAME = "Smarpod_Station_Base"
    BALL_ENDEFFECTOR = "Smarpod_Part_Spawn"
    BALL_CALIBRATION_ENDEFFECTOR = "CAL_Sphere_Center"
    BALL_DIAMETER = 6.35 #mm
    #BALL_DIAMETER = 2*3.125 #mm - fitted from measurements
    #BALL_DIAMETER = 2*3.15 #mm - fitted from measurements
    CALIBRATION_FILE_JOINT_NAME = "Smarpod_Station"
    CALIBRATION_FILE_JOINT_NAME_SPHERE = "CAL_Smarpod_J__t__P"
    SMARPOD_CS_PIVOT_BASE_NAME = "Smarpod_Pivot_Base_Origin"
    SMARPOD_CS_PLATFORM_PIVOT = "Smarpod_Top_Plate"
    MOVE_TIME = 6.0
    SETTLE_TIME = 8.0
    BALL_REPEATABILITY_REMEASUREMENTS = 10
    BALL_REPEATABILITY_RANDOM_XY_RANGE_M = 0.02
    BALL_REPEATABILITY_MOVE_UP_M = 0.02
    BALL_REPEATABILITY_FRAME_SPAWN_OFFSET_RANGE_M = 100e-6
    HEXAPOD_REPEATABILITY_RANDOM_POSES = 10
    HEXAPOD_REPEATABILITY_XY_RANGE_MM = 3.0
    HEXAPOD_REPEATABILITY_Z_MIN_MM = -4.0
    HEXAPOD_REPEATABILITY_Z_MAX_MM = -0.5
    HEXAPOD_REPEATABILITY_RX_RY_RANGE_DEG = 4.5
    HEXAPOD_REPEATABILITY_RZ_RANGE_DEG = 2.0

class PmRobotCalibrationSmarpod:
    def __init__(self, node: Node, utils: PmRobotCalibrationUtils):
        self.node = node
        self._logger = node.get_logger()
        self.pm_calibration_utils = utils

    def _get_sensor_transform(self, use_confocal_top):
        frame = (
            self.pm_calibration_utils.pm_robot_utils.TCP_CONFOCAL_TOP
            if use_confocal_top
            else self.pm_calibration_utils.pm_robot_utils.TCP_LASER
        )

        return self.pm_calibration_utils.pm_robot_utils.get_transform_for_frame(
            frame_name=frame,
            parent_frame="SmarPod_Origin",
        )
    
    def _get_measurement(self, use_confocal_top):
        if use_confocal_top:
            if not self.pm_calibration_utils.pm_robot_utils.check_confocal_top_measurement_in_range():
                raise PmRobotMeasurementError("Confocal top measurement is out of range.")
            return self.pm_calibration_utils.pm_robot_utils.get_confocal_top_measurement(unit="mm")

        if not self.pm_calibration_utils.pm_robot_utils._check_for_valid_laser_measurement():
            raise PmRobotMeasurementError("Laser measurement is out of range.")

        return self.pm_calibration_utils.pm_robot_utils.get_laser_measurement(unit="mm")

    def _prepare_smarpod_calibration_run(self, run_name: str):
        if self.pm_calibration_utils.pm_robot_utils.get_mode() == self.pm_calibration_utils.pm_robot_utils.REAL_MODE:
            self.pm_calibration_utils.pm_robot_utils.pm_robot_config.set_to_real_HW()
            self._logger.info(f"Using real hardware bringup configuration for {run_name}.")
        else:
            self.pm_calibration_utils.pm_robot_utils.pm_robot_config.set_to_simulation_HW()
            self._logger.info(f"Using simulation hardware bringup configuration for {run_name}.")

        if not self.pm_calibration_utils.pm_robot_utils.pm_robot_config.smarpod_station.get_activate_status():
            raise PmRobotError("Smarpod station is not activated in the configuration!")

        self._logger.info(f"Chuck: {self.pm_calibration_utils.pm_robot_utils.pm_robot_config.smarpod_station.get_current_chuck()}")

        if (self.pm_calibration_utils.pm_robot_utils.pm_robot_config.smarpod_station.get_current_chuck_center() != "empty" and
            self.pm_calibration_utils.pm_robot_utils.pm_robot_config.smarpod_station.get_current_chuck() != "empty"):
            raise PmRobotError(
                "Smarpod station has already a chuck and a chuck center assigned. "
                f"Please remove them before {run_name}!"
            )

        self.spawn_smarpod_calibration_sphere_frame()

    def _move_smarpod_absolute_pose(
        self,
        pose_id: str,
        x_cmd_m: float,
        y_cmd_m: float,
        rx_cmd: float,
        ry_cmd: float,
        rz_cmd: float,
        move_time: float,
        settle_time: float,
        ):
        move_success = self.pm_calibration_utils.pm_robot_utils.send_smarpod_trajectory_goal_absolut(
            x_joint=x_cmd_m,
            y_joint=y_cmd_m,
            z_joint=0.0,
            rx_joint_deg=rx_cmd,
            ry_joint_deg=ry_cmd,
            rz_joint_deg=rz_cmd,
            time=move_time,
        )
        
        if not move_success:
            self._logger.error(f"Did not reach joint limits!!!")

        time.sleep(settle_time)

    def _move_sensor_to_sphere_top(self, use_confocal_top: bool) -> bool:
        z_offset = HexapodConstants.BALL_DIAMETER / 2 * 1e-3

        if use_confocal_top:
            move_success, message = self.pm_calibration_utils.pm_robot_utils.move_confocal_top_to_frame(
                HexapodConstants.BALL_CALIBRATION_ENDEFFECTOR,
                z_offset=z_offset,
            )
            if not move_success:
                self._logger.error(message)
            return move_success

        return self.pm_calibration_utils.pm_robot_utils.move_laser_to_frame(
            HexapodConstants.BALL_CALIBRATION_ENDEFFECTOR,
            z_offset=z_offset,
        )

    async def test_hexapod_calibration(self, goal_handle):
        goal = goal_handle.request
        use_confocal_top = goal.use_confocal_over_laser
        poses = get_hexapod_calibration_test_poses()

        result = skills_action.TestHexapodCalibration.Result()
        result.success = False
        result.message = ""
        result.log_file_path = ""

        measurement_data = []
        calibration_run_timestamp = datetime.datetime.now().isoformat()
        calibration_goal_inputs = {
            "use_confocal_over_laser": bool(use_confocal_top),
        }
        move_up = False
        goal_finished = False

        try:
            self._logger.warning("Starting calibration test 'test_hexapod_calibration'...")
            self._prepare_smarpod_calibration_run("testing the hexapod calibration")

            total_iterations = len(poses)

            for current_iteration, pose in enumerate(poses, start=1):
                if goal_handle.is_cancel_requested:
                    raise CancelCalibrationException("Hexapod calibration test cancelled.")

                x_cmd_um = pose["x_cmd_um"]
                y_cmd_um = pose["y_cmd_um"]
                x_cmd_m = x_cmd_um * 1e-6
                y_cmd_m = y_cmd_um * 1e-6
                rx_cmd = pose["rx_cmd"]
                ry_cmd = pose["ry_cmd"]
                rz_cmd = pose["rz_cmd"]
                pose_id = pose["pose_id"]

                self._logger.warning(
                    f"Starting hexapod calibration test iteration {current_iteration}/{total_iterations}: {pose_id}"
                )

                self._move_smarpod_absolute_pose(
                    pose_id=pose_id,
                    x_cmd_m=x_cmd_m,
                    y_cmd_m=y_cmd_m,
                    rx_cmd=rx_cmd,
                    ry_cmd=ry_cmd,
                    rz_cmd=rz_cmd,
                    move_time=HexapodConstants.MOVE_TIME,
                    settle_time=HexapodConstants.SETTLE_TIME,
                )

                move_success = self._move_sensor_to_sphere_top(use_confocal_top)
                if not move_success:
                    raise PmRobotError(f"Could not move sensor to the top of the calibration sphere for pose {pose_id}.")

                move_up = True

                measurement_um = self._get_measurement(use_confocal_top) * 1e3
                sensor_transform = self._get_sensor_transform(use_confocal_top)

                self._logger.warning(
                    f"Hexapod calibration test measurement at pose={pose_id}: {measurement_um:.3f} um"
                )

                current_measurement = {
                    "pose_id": pose_id,
                    "x_cmd_um": x_cmd_um,
                    "y_cmd_um": y_cmd_um,
                    "rx_cmd": rx_cmd,
                    "ry_cmd": ry_cmd,
                    "rz_cmd": rz_cmd,
                    "measurement_um": measurement_um,
                    "transform_endeffector": self.pm_calibration_utils._transform_to_dict(sensor_transform),
                    "current_iteration": current_iteration,
                }
                measurement_data.append(current_measurement)

                feedback = skills_action.TestHexapodCalibration.Feedback()
                feedback.active = True
                feedback.current_iteration = current_iteration
                feedback.total_iterations = total_iterations
                feedback.pose_id = pose_id
                feedback.measurement_um = measurement_um
                feedback.message = f"Measurement at {pose_id}: {measurement_um:.3f} um"
                goal_handle.publish_feedback(feedback)

                if not use_confocal_top:
                    self.pm_calibration_utils.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.005, 1.0)

            result.success = True
            result.message = "Hexapod calibration test succeeded."
            goal_handle.succeed()
            goal_finished = True

        except PmRobotMeasurementError as e:
            message = f"Error measuring the calibration sphere: {e}"
            self._logger.error(message)
            result.success = False
            result.message = message
            goal_handle.abort()
            goal_finished = True

        except PmRobotError as e:
            message = f"Error occurred while testing hexapod calibration: {e}"
            self._logger.error(message)
            result.success = False
            result.message = message
            goal_handle.abort()
            goal_finished = True

        except CancelCalibrationException as e:
            message = f"Hexapod calibration test cancelled: {e}"
            self._logger.warning(message)
            result.success = False
            result.message = message
            goal_handle.canceled()
            goal_finished = True

        except Exception as e:
            message = f"Hexapod calibration test failed unexpectedly: {e}"
            self._logger.error(message)
            result.success = False
            result.message = message
            goal_handle.abort()
            goal_finished = True

        finally:
            calibration_log_dir = self.pm_calibration_utils.get_calibration_log_dir_for_current_mode()
            file_path = get_test_calibrate_smarpod_file_path(calibration_log_dir)
            plot_file_path = get_test_calibrate_smarpod_plot_path(file_path)
            Path(file_path).parent.mkdir(parents=True, exist_ok=True)

            try:
                calibration_output = {
                    "timestamp": calibration_run_timestamp,
                    "calibration_fixed_reference_frame": HexapodConstants.FIXED_CS_SMARPOD_FRAME,
                    "calibration_reference_frame": HexapodConstants.CALIBRATED_CS_SMARPOD_FRAME,
                    "goal_handle": calibration_goal_inputs,
                    "plot_file_path": plot_file_path,
                    "measurement_data": measurement_data,
                }
                if write_json_file(
                    file_path=file_path,
                    data=calibration_output,
                    data_available=bool(measurement_data),
                ):
                    result.log_file_path = file_path
                    self._logger.info(f"Hexapod calibration test data saved to {file_path}")
                    try:
                        plot_smarpod_calibration_test_measurements(
                            measurement_data=measurement_data,
                            output_path=plot_file_path,
                        )
                        self._logger.info(f"Hexapod calibration test plot saved to {plot_file_path}")
                    except Exception as e:
                        self._logger.error(f"Could not save hexapod calibration test plot: {e}")
                else:
                    self._logger.info("Hexapod calibration test data empty. No file saved.")

            except Exception as e:
                self._logger.error(f"Could not save hexapod calibration test data: {e}")

            if move_up:
                self.pm_calibration_utils.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)

            self.pm_calibration_utils.pm_robot_utils.send_smarpod_trajectory_goal_absolut(x_joint=0.0, 
                                                                                          y_joint=0.0, 
                                                                                          z_joint=0.0, 
                                                                                          rx_joint_deg=0.0,
                                                                                          ry_joint_deg=0.0,
                                                                                          rz_joint_deg=0.0,
                                                                                          time=2.0)

            if not goal_finished:
                goal_handle.abort()

        return result


    def spawn_ball_frames(self, 
                          reference_frame:str, 
                          reference_parent_frame: str,
                          ball_diameter_mm = 6.35,
                          frame_spawn_offset_m: dict | None = None)->list:
        
        # spawn the calibration frames
        request = ami_srv.CreateRefFrame.Request()
        name_stem = "CAL_Smarpod_Ball_"
        grid_distance = 1.0 # mm
        z_curve = 0.0 # mm
        frame_name_list = []
        ref_pose:Transform = self.pm_calibration_utils.pm_robot_utils.get_transform_for_frame(frame_name=reference_frame,
                                                               parent_frame=reference_parent_frame)
        frame_spawn_offset_m = frame_spawn_offset_m or {}
        x_spawn_offset_m = float(frame_spawn_offset_m.get("x_m", 0.0))
        y_spawn_offset_m = float(frame_spawn_offset_m.get("y_m", 0.0))
        z_spawn_offset_m = float(frame_spawn_offset_m.get("z_m", 0.0))

        ref_pose.translation.x

        sequence = [
                (0, 0),
                ( 1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (-1, -1),
                (1, -1),
                (-1, 1),
            ]
        
        if not self.pm_calibration_utils.client_create_ref_frame.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Client {self.pm_calibration_utils.client_create_ref_frame.srv_name} not available...")
        
        for index, entry in enumerate(sequence):
            _name = f"{name_stem}{(index+1)}"
            request.ref_frame.frame_name = _name
            frame_name_list.append(_name)
            request.ref_frame.parent_frame = reference_parent_frame
            request.ref_frame.pose.position.x = ref_pose.translation.x + (entry[0]*grid_distance*1e-3) + x_spawn_offset_m
            request.ref_frame.pose.position.y = ref_pose.translation.y + (entry[1]*grid_distance*1e-3) + y_spawn_offset_m
            # case
            mult = 0
            if (abs(entry[0]) >=1) and  (abs(entry[1]) >=1):
                mult = 1
            if ((abs(entry[0]) >=1) and  (abs(entry[1]) ==0) or (abs(entry[0]) ==0) and  (abs(entry[1]) >=1)):
                mult = 1
            if (abs(entry[0]) ==0) and  (abs(entry[1]) == 0):
                mult = 0 
            request.ref_frame.pose.position.z = ref_pose.translation.z + ball_diameter_mm/2*1e-3 - mult * z_curve*1e-3 + z_spawn_offset_m
            z_offset = sphere_z(x=entry[0]*grid_distance,
                                y=entry[1]*grid_distance,
                                #diameter=ball_diameter_mm)
                                diameter=2*3.125)
            
            self._logger.warning(f"z_offset {z_offset}")
            #request.ref_frame.pose.position.z = ref_pose.translation.z + ball_diameter_mm/2*1e-3 - z_offset * 1e-3 - 320*1e-6#+ z_spawn_offset_m 

            spawn_response:ami_srv.CreateRefFrame.Response = self.pm_calibration_utils.client_create_ref_frame.call(request)  

            if not spawn_response.success:
                raise PmRobotError("Failed to spawn calibration frames")
            
        return frame_name_list

    def measure_frame_list (self, 
                            frame_names_list: list[str], 
                            use_confocal_top:bool,
                            fixed_frame_name : str,
                            goal_handle):

        if use_confocal_top:
            correction_method = self.pm_calibration_utils.correct_frame_confocal_top
        else:
            correction_method = self.pm_calibration_utils.correct_frame_laser
        
        for frame_name in frame_names_list:
            if goal_handle.is_cancel_requested:
                self._logger.warning("Calibration cancelled.")
                raise PmRobotError("Calibration cancelled.")

            correction_method(frame_id=frame_name, use_iterative_sensing=True)
        
        time.sleep(1)
        
        results = []
        for frame_name in frame_names_list:
            ref_pose:Transform = self.pm_calibration_utils.pm_robot_utils.get_transform_for_frame(frame_name=frame_name,
                                                        parent_frame=fixed_frame_name)
            transform_dict = self.pm_calibration_utils._transform_to_dict(ref_pose)
            result_dict = {f"{frame_name}": transform_dict}
            results.append(result_dict) 
        
        if not use_confocal_top:
            # move up to ensure hexapod can move feely, but only if laser is used as we have less clearance
            self.pm_calibration_utils.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.005, 1.0)

        return results
    
    def get_smarpod_measurement_dir_for_current_mode(self) -> str:
        return self.pm_calibration_utils.get_calibration_log_dir_for_current_mode()

    def _move_robot_head_random_for_ball_repeatability(self) -> dict:
        x_offset = random.uniform(
            -HexapodConstants.BALL_REPEATABILITY_RANDOM_XY_RANGE_M,
            HexapodConstants.BALL_REPEATABILITY_RANDOM_XY_RANGE_M,
        )
        y_offset = random.uniform(
            -HexapodConstants.BALL_REPEATABILITY_RANDOM_XY_RANGE_M,
            HexapodConstants.BALL_REPEATABILITY_RANDOM_XY_RANGE_M,
        )
        z_offset = -HexapodConstants.BALL_REPEATABILITY_MOVE_UP_M
        move_success = self.pm_calibration_utils.pm_robot_utils.send_xyz_trajectory_goal_relative(
            x_offset,
            y_offset,
            z_offset,
            0.5,
        )
        if not move_success:
            raise PmRobotError("Could not move robot head to random repeatability offset.")
        return {
            "x_m": x_offset,
            "y_m": y_offset,
            "z_m": z_offset,
        }

    def _get_random_ball_frame_spawn_offset(self) -> dict:
        offset_range = HexapodConstants.BALL_REPEATABILITY_FRAME_SPAWN_OFFSET_RANGE_M
        # return {
        #     "x_m": random.uniform(-offset_range, offset_range),
        #     "y_m": random.uniform(-offset_range, offset_range),
        #     "z_m": random.uniform(-offset_range, offset_range),
        # }
        return {
            "x_m": 0.0,
            "y_m": 0.0,
            "z_m": random.uniform(-offset_range, offset_range),
        }

    def _get_random_hexapod_repeatability_pose(self) -> dict:
        x_cmd_mm = random.uniform(
            -HexapodConstants.HEXAPOD_REPEATABILITY_XY_RANGE_MM,
            HexapodConstants.HEXAPOD_REPEATABILITY_XY_RANGE_MM,
        )
        y_cmd_mm = random.uniform(
            -HexapodConstants.HEXAPOD_REPEATABILITY_XY_RANGE_MM,
            HexapodConstants.HEXAPOD_REPEATABILITY_XY_RANGE_MM,
        )
        z_cmd_mm = random.uniform(
            HexapodConstants.HEXAPOD_REPEATABILITY_Z_MIN_MM,
            HexapodConstants.HEXAPOD_REPEATABILITY_Z_MAX_MM,
        )
        return {
            "x_cmd_mm": x_cmd_mm,
            "y_cmd_mm": y_cmd_mm,
            "z_cmd_mm": z_cmd_mm,
            "x_cmd_m": x_cmd_mm * 1e-3,
            "y_cmd_m": y_cmd_mm * 1e-3,
            "z_cmd_m": z_cmd_mm * 1e-3,
            "rx_cmd_deg": random.uniform(
                -HexapodConstants.HEXAPOD_REPEATABILITY_RX_RY_RANGE_DEG,
                HexapodConstants.HEXAPOD_REPEATABILITY_RX_RY_RANGE_DEG,
            ),
            "ry_cmd_deg": random.uniform(
                -HexapodConstants.HEXAPOD_REPEATABILITY_RX_RY_RANGE_DEG,
                HexapodConstants.HEXAPOD_REPEATABILITY_RX_RY_RANGE_DEG,
            ),
            "rz_cmd_deg": random.uniform(
                -HexapodConstants.HEXAPOD_REPEATABILITY_RZ_RANGE_DEG,
                HexapodConstants.HEXAPOD_REPEATABILITY_RZ_RANGE_DEG,
            ),
        }

    def _move_smarpod_to_repeatability_pose(self, pose: dict, move_time: float, settle_time: float) -> None:
        move_success = self.pm_calibration_utils.pm_robot_utils.send_smarpod_trajectory_goal_absolut(
            x_joint=float(pose["x_cmd_m"]),
            y_joint=float(pose["y_cmd_m"]),
            z_joint=float(pose["z_cmd_m"]),
            rx_joint_deg=float(pose["rx_cmd_deg"]),
            ry_joint_deg=float(pose["ry_cmd_deg"]),
            rz_joint_deg=float(pose["rz_cmd_deg"]),
            time=move_time,
        )
        if not move_success:
           self._logger.warning("Joints not reached in time!")

        time.sleep(settle_time)

    def _move_smarpod_to_zero_pose(self, move_time: float, settle_time: float) -> None:
        move_success = self.pm_calibration_utils.pm_robot_utils.send_smarpod_trajectory_goal_absolut(
            x_joint=0.0,
            y_joint=0.0,
            z_joint=0.0,
            rx_joint_deg=0.0,
            ry_joint_deg=0.0,
            rz_joint_deg=0.0,
            time=move_time,
        )
        if not move_success:
           self._logger.warning("Move to Zero: Joints not reached in time!")

        time.sleep(settle_time)

    def _build_hexapod_repeatability_results(self, measurement_file_path: str) -> dict:
        with open(measurement_file_path, "r") as f:
            payload = json.load(f)

        measurements = payload.get("measurement_data", [])
        if not measurements:
            raise PmRobotError("No hexapod repeatability measurements available for assessment.")

        values = [float(entry["measurement_um"]) for entry in measurements]
        deltas = [float(entry.get("delta_from_baseline_um", 0.0)) for entry in measurements]
        mean_value = sum(values) / len(values)
        value_variance = sum((value - mean_value) ** 2 for value in values) / len(values)
        mean_delta = sum(deltas) / len(deltas)
        delta_variance = sum((delta - mean_delta) ** 2 for delta in deltas) / len(deltas)

        return {
            "metadata": {
                "filename": Path(measurement_file_path).stem,
                "measurement_file_path": measurement_file_path,
                "timestamp": payload.get("timestamp"),
                "cal_id": (payload.get("goal_handle") or {}).get("cal_id"),
                "comments": (payload.get("goal_handle") or {}).get("comments"),
                "goal_handle": payload.get("goal_handle") or {},
                "calibration_fixed_reference_frame": payload.get("calibration_fixed_reference_frame"),
                "calibration_reference_frame": payload.get("calibration_reference_frame"),
                "random_pose_count": HexapodConstants.HEXAPOD_REPEATABILITY_RANDOM_POSES,
                "x_y_range_mm": HexapodConstants.HEXAPOD_REPEATABILITY_XY_RANGE_MM,
                "z_min_mm": HexapodConstants.HEXAPOD_REPEATABILITY_Z_MIN_MM,
                "z_max_mm": HexapodConstants.HEXAPOD_REPEATABILITY_Z_MAX_MM,
                "x_y_range_m": HexapodConstants.HEXAPOD_REPEATABILITY_XY_RANGE_MM * 1e-3,
                "z_min_m": HexapodConstants.HEXAPOD_REPEATABILITY_Z_MIN_MM * 1e-3,
                "z_max_m": HexapodConstants.HEXAPOD_REPEATABILITY_Z_MAX_MM * 1e-3,
                "rx_ry_range_deg": HexapodConstants.HEXAPOD_REPEATABILITY_RX_RY_RANGE_DEG,
                "rz_range_deg": HexapodConstants.HEXAPOD_REPEATABILITY_RZ_RANGE_DEG,
                "measurement_sequence": "random_pose_then_zero_then_measure",
            },
            "summary": {
                "measurement_um": {
                    "mean": mean_value,
                    "std": math.sqrt(value_variance),
                    "min": min(values),
                    "max": max(values),
                    "range": max(values) - min(values),
                },
                "delta_from_baseline_um": {
                    "mean": mean_delta,
                    "std": math.sqrt(delta_variance),
                    "min": min(deltas),
                    "max": max(deltas),
                    "range": max(deltas) - min(deltas),
                    "max_abs": max(abs(delta) for delta in deltas),
                },
            },
            "measurements": measurements,
        }

    async def test_hexapod_repeatability(self, goal_handle: skills_action.HexapodRepeatability.Goal):
        goal = goal_handle.request
        result = skills_action.HexapodRepeatability.Result()
        result.success = False
        result.message = ""

        measurement_data = []
        calibration_run_timestamp = datetime.datetime.now().isoformat()
        calibration_goal_inputs = {
            "cal_id": str(goal.cal_id),
            "comments": str(goal.comments),
            "sensor": "confocal_top",
        }
        measurement_file_saved = False
        goal_finished = False

        try:
            self._logger.warning("Starting hexapod repeatability test...")
            self._prepare_smarpod_calibration_run("testing hexapod repeatability")
            move_success = self.pm_calibration_utils.pm_robot_utils.send_smarpod_trajectory_goal_absolut(
                x_joint=0.0,
                y_joint=0.0,
                z_joint=0.0,
                rx_joint_deg=0.0,
                ry_joint_deg=0.0,
                rz_joint_deg=0.0,
                time=2.0,
            )
            if not move_success:
                raise PmRobotError("Could not move smarpod to all-zero pose.")

            if not self._move_sensor_to_sphere_top(use_confocal_top=True):
                raise PmRobotError("Could not move confocal top sensor to the sphere top.")

            baseline_measurement_um = self._get_measurement(use_confocal_top=True) * 1e3

            self._logger.warning(f"Base measurments is: {baseline_measurement_um} um.")

            sensor_transform = self._get_sensor_transform(use_confocal_top=True)
            measurement_data.append({
                "pose_id": "baseline_zero_pose",
                "current_iteration": 0,
                "x_cmd_m": 0.0,
                "y_cmd_m": 0.0,
                "z_cmd_m": 0.0,
                "rx_cmd_deg": 0.0,
                "ry_cmd_deg": 0.0,
                "rz_cmd_deg": 0.0,
                "measurement_um": baseline_measurement_um,
                "delta_from_baseline_um": 0.0,
                "transform_sensor": self.pm_calibration_utils._transform_to_dict(sensor_transform),
            })

            #total_iterations = HexapodConstants.HEXAPOD_REPEATABILITY_RANDOM_POSES
            total_iterations = 20

            for current_iteration in range(1, total_iterations + 1):
                if goal_handle.is_cancel_requested:
                    raise CancelCalibrationException("Hexapod repeatability test cancelled.")

                pose = self._get_random_hexapod_repeatability_pose()
                pose_id = f"hexapod_repeatability_pose_{current_iteration:02d}"
                self._logger.warning(
                    f"Starting hexapod repeatability measurement "
                    f"{current_iteration}/{total_iterations}: {pose_id}"
                )
                self._logger.warning(f"Pose is: {pose}")

                self._move_smarpod_to_repeatability_pose(
                    pose=pose,
                    move_time=12,
                    settle_time=HexapodConstants.SETTLE_TIME,
                )

                self._move_smarpod_to_zero_pose(
                    move_time=12,
                    settle_time=HexapodConstants.SETTLE_TIME,
                )

                measurement_um = self._get_measurement(use_confocal_top=True) * 1e3

                self._logger.warning(f"Top measurments is: {measurement_um} um.")

                sensor_transform = self._get_sensor_transform(use_confocal_top=True)
                measurement_data.append({
                    "pose_id": pose_id,
                    "current_iteration": current_iteration,
                    **pose,
                    "measurement_pose": "zero_after_random_pose",
                    "measurement_um": measurement_um,
                    "delta_from_baseline_um": measurement_um - baseline_measurement_um,
                    "transform_sensor": self.pm_calibration_utils._transform_to_dict(sensor_transform),
                })

                feedback = skills_action.HexapodRepeatability.Feedback()
                feedback.active = True
                feedback.current_iteration = current_iteration
                feedback.total_iterations = total_iterations
                feedback.pose_id = pose_id
                feedback.measurement_um = measurement_um
                feedback.message = f"Measurement at {pose_id}: {measurement_um:.3f} um"
                goal_handle.publish_feedback(feedback)

            result.success = True
            result.message = "Hexapod repeatability test succeeded."
            goal_handle.succeed()
            goal_finished = True

        except PmRobotMeasurementError as e:
            message = f"Error measuring hexapod repeatability: {e}"
            self._logger.error(message)
            result.success = False
            result.message = message
            goal_handle.abort()
            goal_finished = True

        except PmRobotError as e:
            message = f"Error occurred while testing hexapod repeatability: {e}"
            self._logger.error(message)
            result.success = False
            result.message = message
            goal_handle.abort()
            goal_finished = True

        except CancelCalibrationException as e:
            message = f"Hexapod repeatability test cancelled: {e}"
            self._logger.warning(message)
            result.success = False
            result.message = message
            goal_handle.canceled()
            goal_finished = True

        except Exception as e:
            message = f"Hexapod repeatability test failed unexpectedly: {e}"
            self._logger.error(message)
            result.success = False
            result.message = message
            goal_handle.abort()
            goal_finished = True

        finally:
            measurement_dir = self.get_smarpod_measurement_dir_for_current_mode()
            file_path = get_hexapod_repeatability_measurement_file_path(measurement_dir)

            if result.success:
                Path(file_path).parent.mkdir(parents=True, exist_ok=True)
                try:
                    measurement_output = {
                        "timestamp": calibration_run_timestamp,
                        "calibration_fixed_reference_frame": HexapodConstants.FIXED_CS_SMARPOD_FRAME,
                        "calibration_reference_frame": HexapodConstants.CALIBRATED_CS_SMARPOD_FRAME,
                        "goal_handle": calibration_goal_inputs,
                        "measurement_data": measurement_data,
                        "repeatability_test": {
                            "baseline_measurement_count": 1,
                            "random_pose_count": HexapodConstants.HEXAPOD_REPEATABILITY_RANDOM_POSES,
                            "x_y_range_mm": HexapodConstants.HEXAPOD_REPEATABILITY_XY_RANGE_MM,
                            "z_min_mm": HexapodConstants.HEXAPOD_REPEATABILITY_Z_MIN_MM,
                            "z_max_mm": HexapodConstants.HEXAPOD_REPEATABILITY_Z_MAX_MM,
                            "x_y_range_m": HexapodConstants.HEXAPOD_REPEATABILITY_XY_RANGE_MM * 1e-3,
                            "z_min_m": HexapodConstants.HEXAPOD_REPEATABILITY_Z_MIN_MM * 1e-3,
                            "z_max_m": HexapodConstants.HEXAPOD_REPEATABILITY_Z_MAX_MM * 1e-3,
                            "rx_ry_range_deg": HexapodConstants.HEXAPOD_REPEATABILITY_RX_RY_RANGE_DEG,
                            "rz_range_deg": HexapodConstants.HEXAPOD_REPEATABILITY_RZ_RANGE_DEG,
                            "z_direction": "negative_only",
                            "measurement_sequence": "random_pose_then_zero_then_measure",
                        },
                        "test_complete": True,
                    }
                    if write_json_file(
                        file_path=file_path,
                        data=measurement_output,
                        data_available=bool(measurement_data),
                    ):
                        measurement_file_saved = True
                        self._logger.info(f"Hexapod repeatability measurements saved to {file_path}")
                    else:
                        self._logger.info("Hexapod repeatability measurements empty. No file saved.")
                except Exception as e:
                    self._logger.error(f"Could not save hexapod repeatability measurements: {e}")

            if result.success and measurement_file_saved:
                try:
                    repeatability_results = self._build_hexapod_repeatability_results(file_path)
                    results_dir = get_hexapod_repeatability_results_dir(file_path)
                    Path(results_dir).mkdir(parents=True, exist_ok=True)
                    results_file_path = get_hexapod_repeatability_results_json_path(results_dir, file_path)
                    plot_base_path = get_hexapod_repeatability_plot_base_path(results_dir, file_path)
                    plot_paths = plot_hexapod_repeatability_results(
                        repeatability_results=repeatability_results,
                        output_base_path=plot_base_path,
                    )
                    repeatability_results["metadata"]["plot_file_paths"] = plot_paths
                    write_json_file(
                        file_path=results_file_path,
                        data=repeatability_results,
                        data_available=True,
                    )
                    result.message = (
                        "Hexapod repeatability test succeeded. "
                        f"Measurement file: {file_path}. Results file: {results_file_path}."
                    )
                    self._logger.info(result.message)
                except Exception as e:
                    message = f"Hexapod repeatability measurements saved, but analysis failed: {e}"
                    self._logger.error(message)
                    result.success = False
                    result.message = message
            elif result.success:
                result.success = False
                result.message = "Hexapod repeatability test succeeded, but no measurement file was saved."

            self.pm_calibration_utils.pm_robot_utils.send_smarpod_trajectory_goal_absolut(
                x_joint=0.0,
                y_joint=0.0,
                z_joint=0.0,
                rx_joint_deg=0.0,
                ry_joint_deg=0.0,
                rz_joint_deg=0.0,
                time=2.0,
            )

            if not goal_finished:
                goal_handle.abort()

        return result

    def _build_smarpod_ball_repeatability_results(
        self,
        measurement_file_path: str,
        fit_sphere_radius: bool = False,
    ) -> dict:
        sc = SphereCalibration.load_file(measurement_file_path)
        initial_radius_mm = HexapodConstants.BALL_DIAMETER / 2.0
        sphere_fits = sc.fit_sphere_per_set(
            radius_mm=initial_radius_mm,
            fit_radius=fit_sphere_radius,
        )
        measurement_entries = {
            str(entry.get("pose_id")): entry
            for entry in (sc.source_payload or {}).get("calibration_data", [])
            if isinstance(entry, dict)
        }

        runs = []
        centers = []
        radii = []
        ordered_sets = sorted(sc.sphere_sets, key=lambda sphere_set: sphere_set.current_iteration)
        for sphere_set in ordered_sets:
            fit_result = sphere_fits.get(sphere_set.sphere_set_id)
            if fit_result is None:
                continue
            center = fit_result.center
            center_xyz = [float(center.x), float(center.y), float(center.z)]
            centers.append(center_xyz)
            radii.append(float(fit_result.radius_mm))
            measurement_entry = measurement_entries.get(sphere_set.sphere_set_id, {})
            ball_measurements = {
                measurement.sphere_id: {
                    "x": float(measurement.measurement_position.x),
                    "y": float(measurement.measurement_position.y),
                    "z": float(measurement.measurement_position.z),
                }
                for measurement in sphere_set.sphere_measurements.get_all_sphere_measurements()
            }
            runs.append({
                "iteration": int(sphere_set.current_iteration),
                "pose_id": sphere_set.sphere_set_id,
                "movement_before_measurement_m": measurement_entry.get("movement_before_measurement_m"),
                "frame_spawn_offset_m": measurement_entry.get("frame_spawn_offset_m"),
                "ball_measurements_mm": ball_measurements,
                "center_mm": {
                    "x": center_xyz[0],
                    "y": center_xyz[1],
                    "z": center_xyz[2],
                },
                "radius_mm": float(fit_result.radius_mm),
                "rms_fit_error_um": float(fit_result.rms_error_mm * 1000.0),
                "max_abs_fit_error_um": float(fit_result.max_abs_error_mm * 1000.0),
                "point_count": int(fit_result.point_count),
                "residuals_by_sphere_id_um": {
                    sphere_id: float(value) * 1000.0
                    for sphere_id, value in fit_result.residuals_by_sphere_id.items()
                },
            })

        if not centers:
            raise PmRobotError("No repeatability sphere fits were created.")

        center_mean = [
            sum(center[axis] for center in centers) / len(centers)
            for axis in range(3)
        ]
        center_std_um = []
        center_range_um = []
        center_delta_norm_um = []
        for axis in range(3):
            values = [center[axis] for center in centers]
            mean_value = center_mean[axis]
            variance = sum((value - mean_value) ** 2 for value in values) / len(values)
            center_std_um.append(math.sqrt(variance) * 1000.0)
            center_range_um.append((max(values) - min(values)) * 1000.0)

        for center in centers:
            center_delta_norm_um.append(
                math.sqrt(sum((center[axis] - center_mean[axis]) ** 2 for axis in range(3))) * 1000.0
            )

        rms_values = [run["rms_fit_error_um"] for run in runs]
        max_values = [run["max_abs_fit_error_um"] for run in runs]
        radius_mean = sum(radii) / len(radii)
        radius_variance = sum((radius - radius_mean) ** 2 for radius in radii) / len(radii)
        radius_delta_um = [
            (radius - initial_radius_mm) * 1000.0
            for radius in radii
        ]
        radius_delta_mean = sum(radius_delta_um) / len(radius_delta_um)
        radius_delta_variance = (
            sum((delta - radius_delta_mean) ** 2 for delta in radius_delta_um)
            / len(radius_delta_um)
        )

        return {
            "metadata": {
                "filename": sc.filename,
                "measurement_file_path": measurement_file_path,
                "timestamp": sc.timestamp,
                "cal_id": (sc.goal_handle or {}).get("cal_id"),
                "comments": (sc.goal_handle or {}).get("comments"),
                "goal_handle": sc.goal_handle or {},
                "calibration_reference_frame": sc.calibration_reference_frame,
                "calibration_fixed_reference_frame": sc.calibration_fixed_reference_frame,
                "calibration_sphere_diameter_mm": HexapodConstants.BALL_DIAMETER,
                "initial_sphere_radius_mm": initial_radius_mm,
                "fit_sphere_radius": bool(fit_sphere_radius),
                "measurement_count": len(runs),
                "remeasurement_count": HexapodConstants.BALL_REPEATABILITY_REMEASUREMENTS,
                "frame_spawn_offset_range_m": HexapodConstants.BALL_REPEATABILITY_FRAME_SPAWN_OFFSET_RANGE_M,
                "frame_spawn_offset_range_um": HexapodConstants.BALL_REPEATABILITY_FRAME_SPAWN_OFFSET_RANGE_M * 1e6,
            },
            "summary": {
                "center_mean_mm": {
                    "x": center_mean[0],
                    "y": center_mean[1],
                    "z": center_mean[2],
                },
                "center_std_um": {
                    "x": center_std_um[0],
                    "y": center_std_um[1],
                    "z": center_std_um[2],
                },
                "center_range_um": {
                    "x": center_range_um[0],
                    "y": center_range_um[1],
                    "z": center_range_um[2],
                },
                "center_delta_norm_um": {
                    "mean": sum(center_delta_norm_um) / len(center_delta_norm_um),
                    "max": max(center_delta_norm_um),
                },
                "sphere_fit_rms_error_um": {
                    "mean": sum(rms_values) / len(rms_values),
                    "max": max(rms_values),
                },
                "sphere_fit_max_abs_error_um": {
                    "mean": sum(max_values) / len(max_values),
                    "max": max(max_values),
                },
                "fitted_radius_mm": {
                    "mean": radius_mean,
                    "std": math.sqrt(radius_variance),
                    "min": min(radii),
                    "max": max(radii),
                },
                "fitted_radius_delta_um": {
                    "mean": radius_delta_mean,
                    "std": math.sqrt(radius_delta_variance),
                    "min": min(radius_delta_um),
                    "max": max(radius_delta_um),
                },
            },
            "runs": runs,
        }

    async def test_smarpod_ball_repeatability(self, goal_handle: skills_action.SmarpodBallRepeatability.Goal):
        goal = goal_handle.request
        use_confocal_top = goal.use_confocal_over_laser
        calibration_run_timestamp = datetime.datetime.now().isoformat()
        calibration_goal_inputs = {
            "use_confocal_over_laser": bool(use_confocal_top),
            "cal_id": str(goal.cal_id),
            "comments": str(goal.comments),
            "fit_sphere_radius": bool(goal.fit_sphere_radius),
        }

        result = skills_action.SmarpodBallRepeatability.Result()
        result.success = False
        result.message = ""

        calibration_data = []
        measurement_file_saved = False
        goal_finished = False
        results_file_path = ""
        plot_paths = []

        try:
            self._logger.warning("Starting smarpod ball repeatability test...")
            self._prepare_smarpod_calibration_run("testing smarpod ball repeatability")
            move_success = self.pm_calibration_utils.pm_robot_utils.send_smarpod_trajectory_goal_absolut(
                x_joint=0.0,
                y_joint=0.0,
                z_joint=0.0,
                rx_joint_deg=0.0,
                ry_joint_deg=0.0,
                rz_joint_deg=0.0,
                time=2.0,
            )
            if not move_success:
                raise PmRobotError("Could not move smarpod to all-zero pose.")

            total_iterations = HexapodConstants.BALL_REPEATABILITY_REMEASUREMENTS + 1
            
            for current_iteration in range(1, total_iterations + 1):
                
                frame_spawn_offset_m = self._get_random_ball_frame_spawn_offset()
                frame_names = self.spawn_ball_frames(
                    reference_frame=HexapodConstants.BALL_CALIBRATION_ENDEFFECTOR,
                    reference_parent_frame=HexapodConstants.FIXED_CS_SMARPOD_FRAME,
                    ball_diameter_mm=HexapodConstants.BALL_DIAMETER,
                    frame_spawn_offset_m=frame_spawn_offset_m,
                )

                if goal_handle.is_cancel_requested:
                    raise CancelCalibrationException("Smarpod ball repeatability test cancelled.")

                movement_before_measurement = {
                    "x_m": 0.0,
                    "y_m": 0.0,
                    "z_m": 0.0,
                }
                if current_iteration > 1:
                    movement_before_measurement = self._move_robot_head_random_for_ball_repeatability()

                pose_id = f"repeatability_zero_pose_{current_iteration:02d}"
                self._logger.warning(
                    f"Starting smarpod ball repeatability measurement "
                    f"{current_iteration}/{total_iterations}: {pose_id}"
                )

                results_list = self.measure_frame_list(
                    frame_names_list=frame_names,
                    use_confocal_top=use_confocal_top,
                    fixed_frame_name=HexapodConstants.FIXED_CS_SMARPOD_FRAME,
                    goal_handle=goal_handle,
                )
                calibration_data.append({
                    "pose_id": pose_id,
                    "rx_cmd": 0.0,
                    "ry_cmd": 0.0,
                    "rz_cmd": 0.0,
                    "x_cmd": 0.0,
                    "y_cmd": 0.0,
                    "movement_before_measurement_m": movement_before_measurement,
                    "frame_spawn_offset_m": frame_spawn_offset_m,
                    "results_list": results_list,
                    "current_iteration": current_iteration,
                })

                feedback = skills_action.SmarpodBallRepeatability.Feedback()
                feedback.active = True
                goal_handle.publish_feedback(feedback)

            result.success = True
            result.message = "Smarpod ball repeatability test succeeded."
            goal_handle.succeed()
            goal_finished = True

        except PmRobotError as e:
            message = f"Error occurred while testing smarpod ball repeatability: {e}"
            self._logger.error(message)
            result.success = False
            result.message = message
            goal_handle.abort()
            goal_finished = True

        except CancelCalibrationException as e:
            message = f"Smarpod ball repeatability test cancelled: {e}"
            self._logger.warning(message)
            result.success = False
            result.message = message
            goal_handle.canceled()
            goal_finished = True

        except Exception as e:
            message = f"Smarpod ball repeatability test failed unexpectedly: {e}"
            self._logger.error(message)
            result.success = False
            result.message = message
            goal_handle.abort()
            goal_finished = True

        finally:
            measurement_dir = self.get_smarpod_measurement_dir_for_current_mode()
            file_path = get_smarpod_ball_repeatability_measurement_file_path(measurement_dir)
            Path(file_path).parent.mkdir(parents=True, exist_ok=True)

            try:
                calibration_output = {
                    "timestamp": calibration_run_timestamp,
                    "calibration_fixed_reference_frame": HexapodConstants.FIXED_CS_SMARPOD_FRAME,
                    "calibration_reference_frame": HexapodConstants.CALIBRATED_CS_SMARPOD_FRAME,
                    "goal_handle": calibration_goal_inputs,
                    "calibration_data": calibration_data,
                    "calibration_complete": result.success,
                    "repeatability_test": {
                        "baseline_measurement_count": 1,
                        "remeasurement_count": HexapodConstants.BALL_REPEATABILITY_REMEASUREMENTS,
                        "random_xy_range_m": HexapodConstants.BALL_REPEATABILITY_RANDOM_XY_RANGE_M,
                        "move_up_m": HexapodConstants.BALL_REPEATABILITY_MOVE_UP_M,
                        "frame_spawn_offset_range_m": HexapodConstants.BALL_REPEATABILITY_FRAME_SPAWN_OFFSET_RANGE_M,
                        "frame_spawn_offset_range_um": HexapodConstants.BALL_REPEATABILITY_FRAME_SPAWN_OFFSET_RANGE_M * 1e6,
                        "fit_sphere_radius": bool(goal.fit_sphere_radius),
                        "initial_sphere_radius_mm": HexapodConstants.BALL_DIAMETER / 2.0,
                    },
                }
                if write_json_file(
                    file_path=file_path,
                    data=calibration_output,
                    data_available=bool(calibration_data),
                ):
                    measurement_file_saved = True
                    self._logger.info(f"Smarpod ball repeatability measurements saved to {file_path}")
                else:
                    self._logger.info("Smarpod ball repeatability measurements empty. No file saved.")
            except Exception as e:
                self._logger.error(f"Could not save smarpod ball repeatability measurements: {e}")

            if result.success and measurement_file_saved:
                try:
                    repeatability_results = self._build_smarpod_ball_repeatability_results(
                        file_path,
                        fit_sphere_radius=bool(goal.fit_sphere_radius),
                    )
                    results_dir = get_smarpod_ball_repeatability_results_dir(file_path)
                    Path(results_dir).mkdir(parents=True, exist_ok=True)
                    results_file_path = get_smarpod_ball_repeatability_results_json_path(results_dir, file_path)
                    plot_base_path = get_smarpod_ball_repeatability_plot_base_path(results_dir, file_path)
                    plot_paths = plot_smarpod_ball_repeatability_results(
                        repeatability_results=repeatability_results,
                        output_base_path=plot_base_path,
                    )
                    repeatability_results["metadata"]["plot_file_paths"] = plot_paths
                    write_json_file(
                        file_path=results_file_path,
                        data=repeatability_results,
                        data_available=True,
                    )
                    result.message = (
                        "Smarpod ball repeatability test succeeded. "
                        f"Measurement file: {file_path}. Results file: {results_file_path}."
                    )
                    self._logger.info(result.message)
                except Exception as e:
                    message = f"Smarpod ball repeatability measurements saved, but analysis failed: {e}"
                    self._logger.error(message)
                    result.success = False
                    result.message = message
            elif result.success:
                result.success = False
                result.message = "Repeatability test succeeded, but no measurement file was saved."

            self.pm_calibration_utils.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)
            self.pm_calibration_utils.pm_robot_utils.send_smarpod_trajectory_goal_absolut(
                x_joint=0.0,
                y_joint=0.0,
                z_joint=0.0,
                rx_joint_deg=0.0,
                ry_joint_deg=0.0,
                rz_joint_deg=0.0,
                time=2.0,
            )

            if not goal_finished:
                goal_handle.abort()

        return result
    
    async def calibrate_smarpod_V3(self, goal_handle: skills_action.SmarpodCalibration.Goal):
        
        goal = goal_handle.request
        use_confocal_top = goal.use_confocal_over_laser

        calibration_complete = False

        calibration_data = []
        calibration_run_timestamp = datetime.datetime.now().isoformat()
        calibration_goal_inputs = {
            "use_confocal_over_laser": bool(use_confocal_top),
            "cal_id": str(goal.cal_id),
            "comments": str(goal.comments),
        }
        
        result = skills_action.SmarpodCalibration.Result()
        result.success = False

        move_up = False
        current_cal_transfrom_dict = {}
        measurement_file_saved = False
        goal_finished = False

        try:
            self._logger.warning(f"Starting calibration 'calibrate_smarpod'...")
            self._prepare_smarpod_calibration_run("calibrating the smarpod station")

            current_cal_transfrom:Transform = self.pm_calibration_utils.get_current_joint_calibration_transform(
                HexapodConstants.CALIBRATION_FILE_JOINT_NAME
            )
            current_cal_transfrom_dict = self.pm_calibration_utils._transform_to_dict(current_cal_transfrom)

            self.pm_calibration_utils.pm_robot_utils.send_smarpod_trajectory_goal_absolut(x_joint=0.0, y_joint=0.0, z_joint=0.0, time=1.0)

            positions = get_hexapod_calibration_positions_mm()
            orientation_commands = get_hexapod_orientation_commands()
            total_iterations = len(positions) * len(orientation_commands)
            current_iteration = 0

            if not self._move_sensor_to_sphere_top(use_confocal_top):
                raise PmRobotError(f"Could not move to desired position!")
            
            try:
                measurement_mm = self._get_measurement(use_confocal_top)

                self._logger.warning(f"Initial measurement SUCCESSED! Current value {measurement_mm} mm.")

            except PmRobotMeasurementError as e:
                raise PmRobotError(f"Initial Measurement on calibration ball failed. Make sure the hexapod is already routhgly calibrated so that the distance sensor hits the top of the ball and shows a measurement value of 0.0!")

            for x_pos, y_pos in positions:
                for command in orientation_commands:
                    if goal_handle.is_cancel_requested:
                        self._logger.warning("Calibration cancelled.")
                        raise CancelCalibrationException("Smarpod calibration cancelled.")
                    
                    current_iteration += 1
                    rx_cmd = command["rx_cmd"]
                    ry_cmd = command["ry_cmd"]
                    rz_cmd = command["rz_cmd"]
                    pose_id = f"rx{rx_cmd}_ry{ry_cmd}_rz{rz_cmd}_x{x_pos}_y{y_pos}"

                    self._logger.warning(f"Starting iteration {current_iteration}/{total_iterations}")   

                    self._move_smarpod_absolute_pose(
                        pose_id=pose_id,
                        x_cmd_m=x_pos*1e-3,
                        y_cmd_m=y_pos*1e-3,
                        rx_cmd=rx_cmd,
                        ry_cmd=ry_cmd,
                        rz_cmd=rz_cmd,
                        move_time=HexapodConstants.MOVE_TIME,
                        settle_time=HexapodConstants.SETTLE_TIME
                    )

                    name_list = self.spawn_ball_frames(reference_frame=HexapodConstants.BALL_CALIBRATION_ENDEFFECTOR,
                                    reference_parent_frame=HexapodConstants.FIXED_CS_SMARPOD_FRAME,
                                    ball_diameter_mm=HexapodConstants.BALL_DIAMETER)
                    
                    results_list = self.measure_frame_list(frame_names_list=name_list,
                                            use_confocal_top=use_confocal_top,
                                            fixed_frame_name=HexapodConstants.FIXED_CS_SMARPOD_FRAME,
                                            goal_handle = goal_handle)
                    
                    move_up = True
                    calibration_data.append({
                        "pose_id": pose_id,
                        "rx_cmd": rx_cmd,
                        "ry_cmd": ry_cmd,
                        "rz_cmd": rz_cmd,
                        "x_cmd": x_pos,
                        "y_cmd": y_pos,
                        "results_list": results_list,
                        "current_iteration": current_iteration,
                    })
            
            result.success = True
            result.message = "Calibration succeded!"
            goal_handle.succeed()
            goal_finished = True
            calibration_complete = True
        
        except PmRobotMeasurementError as e:
            message = f"Error measuring the calibration ball: {e}"
            self._logger.error(message)
            result.success = False
            result.message = message
            goal_handle.abort()
            goal_finished = True

        except PmRobotError as e:
            message = f"Error occurred while calibrating smarpod: {e}"
            self._logger.error(message)
            result.success = False
            result.message = message
            goal_handle.abort()
            goal_finished = True

        except CancelCalibrationException as e:
            message = f"Calibration cancelled: {e}"
            self._logger.warning(message)
            result.success = False
            result.message = message
            goal_handle.canceled()
            goal_finished = True

        finally:
            # save the calibration data
            measurement_dir = self.get_smarpod_measurement_dir_for_current_mode()
            Path(measurement_dir).mkdir(parents=True, exist_ok=True)

            file_path = get_calibrate_smarpod_measurement_file_path(measurement_dir)
            
            try:
                calibration_output = {
                    "timestamp": calibration_run_timestamp,
                    "calibration_fixed_reference_frame": HexapodConstants.FIXED_CS_SMARPOD_FRAME,
                    "calibration_reference_frame": HexapodConstants.CALIBRATED_CS_SMARPOD_FRAME,
                    "current_calibration_transformation": current_cal_transfrom_dict, 
                    "goal_handle": calibration_goal_inputs,
                    "calibration_data": calibration_data,
                    "calibration_complete": calibration_complete
                }
                if write_json_file(
                    file_path=file_path,
                    data=calibration_output,
                    data_available=bool(calibration_data),
                ):
                    measurement_file_saved = True
                    self._logger.info(f"Calibration data saved to {file_path}")
                else:
                    self._logger.info(f"Calibration data empty. No file saved!")

            except Exception as e:
                self._logger.error(f"Could not save calibration data: {e}")

            if result.success and measurement_file_saved:
                assess_request = skills_srv.AssessHexapodCalibration.Request()
                assess_response = skills_srv.AssessHexapodCalibration.Response()
                assess_request.results_file_path = file_path
                assess_request.write_to_joint_config = True
                assess_request.fit_sphere_diameter = False
                assess_response = self.assess_hexapod_calibration(
                    assess_request,
                    assess_response,
                )
                if not assess_response.success:
                    result.success = False
                    result.message = assess_response.message

                self.pm_calibration_utils._calibration_history_log(history_entry="calibrate_smarpod")

            elif result.success:
                result.success = False
                result.message = "Calibration succeeded, but no measurement file was saved for assessment."

            if move_up:
                self.pm_calibration_utils.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)

            # Move smarpod back to zero position
            self.pm_calibration_utils.pm_robot_utils.send_smarpod_trajectory_goal_absolut(x_joint=0.0, 
                                                                                          y_joint=0.0, 
                                                                                          z_joint=0.0, 
                                                                                          rx_joint_deg=0.0,
                                                                                          ry_joint_deg=0.0,
                                                                                          rz_joint_deg=0.0,
                                                                                          time=2.0)

            if not goal_finished:
                goal_handle.abort()
        
        return result
        
    def spawn_smarpod_calibration_sphere_frame(self):
        current_transform_sphere = self.pm_calibration_utils.get_current_joint_calibration_transform(HexapodConstants.CALIBRATION_FILE_JOINT_NAME_SPHERE)
            
        spawn_request = ami_srv.CreateRefFrame.Request()
        spawn_request.ref_frame.frame_name = "CAL_Sphere_Center"
        spawn_request.ref_frame.parent_frame = HexapodConstants.SMARPOD_CS_PLATFORM_PIVOT

        if (current_transform_sphere.translation.x != 0 and
            current_transform_sphere.translation.y != 0 and
            current_transform_sphere.translation.z != 0):

            spawn_request.ref_frame.pose.position.x = current_transform_sphere.translation.x
            spawn_request.ref_frame.pose.position.y = current_transform_sphere.translation.y
            spawn_request.ref_frame.pose.position.z = current_transform_sphere.translation.z

        else:
            default_ball_transform:Transform = self.pm_calibration_utils.pm_robot_utils.get_transform_for_frame(frame_name=HexapodConstants.BALL_ENDEFFECTOR,
                                            parent_frame=HexapodConstants.SMARPOD_CS_PLATFORM_PIVOT)
            
            spawn_request.ref_frame.pose.position.x = default_ball_transform.translation.x
            spawn_request.ref_frame.pose.position.y = default_ball_transform.translation.y
            spawn_request.ref_frame.pose.position.z = default_ball_transform.translation.z

        _res = self.pm_calibration_utils.pm_robot_utils.create_ref_frame(spawn_request)

    def assess_hexapod_calibration(self, request:skills_srv.AssessHexapodCalibration.Request, response:skills_srv.AssessHexapodCalibration.Response):
        
        try:
            
            sc = SphereCalibration.load_file(request.results_file_path)

            self._logger.warn(f"Start calculating the Smarpot Pivot Point!")

            analysis:CalibrationAnalysis = sc.run_calibration(
                diameter_mm=HexapodConstants.BALL_DIAMETER,
                fit_sphere_radius=bool(request.fit_sphere_diameter),
            )
            
            # Keep assessment artifacts with the explicitly supplied measurement
            # file. The configured calibration log directory may belong to a
            # different machine/user and must not override the request path.
            results_dir = get_smarpod_results_dir(request.results_file_path)
            Path(results_dir).mkdir(parents=True, exist_ok=True)
            
            results_json_path = get_smarpod_results_json_path(
                results_dir,
                request.results_file_path,
            )
            analysis.save_results(file_path=results_json_path)
            
            analysis.plot_results(file_path=get_smarpod_results_base_path(results_dir, request.results_file_path))

            translation_ball = analysis.get_J_t_P_translation(unit='m')
            translation_ball_mm = analysis.get_J_t_P_translation(unit='mm')
            euler_angles = analysis.get_B_T_P_euler("deg")
            pivot_translation = analysis.get_B_T_P_translation('mm')

            response.p_t_s.x = float(translation_ball_mm[0])
            response.p_t_s.y = float(translation_ball_mm[1])
            response.p_t_s.z = float(translation_ball_mm[2])
            response.b_t_p.x = float(pivot_translation[0])
            response.b_t_p.y = float(pivot_translation[1])
            response.b_t_p.z = float(pivot_translation[2])
            response.b_t_p_euler_angles = [float(angle) for angle in euler_angles]

            if not request.write_to_joint_config:
                try:
                    comparison_summary = generate_hexapod_calibration_comparison_report(
                        measurement_file_path=request.results_file_path,
                        result_file_path=results_json_path,
                    )
                    self._logger.info(
                        "Hexapod calibration comparison report generated: "
                        f"{comparison_summary['report_dir']}"
                    )
                except Exception as e:
                    self._logger.error(
                        "Hexapod calibration assessment succeeded, but comparison "
                        f"report generation failed: {e}"
                    )

                response.success = True
                response.message = (
                    "Hexapod calibration assessed successfully. Results and "
                    "plots were saved; the joint configuration was not changed."
                )
                self._logger.info(response.message)
                return response

            B__T__J = analysis.get_B_T_P_ros_transform()
            
            current_transform_sphere = self.pm_calibration_utils.get_current_joint_calibration_transform(HexapodConstants.CALIBRATION_FILE_JOINT_NAME_SPHERE)
            
            current_transform_sphere.translation.x = translation_ball[0]
            current_transform_sphere.translation.y = translation_ball[1]
            current_transform_sphere.translation.z = translation_ball[2]

            C__T__J:Transform = self.pm_calibration_utils.pm_robot_utils.get_transform_for_frame(frame_name=HexapodConstants.SMARPOD_CS_PIVOT_BASE_NAME,
                                                                        parent_frame=HexapodConstants.CALIBRATED_CS_SMARPOD_FRAME)

            sphere_cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                joint_name=HexapodConstants.CALIBRATION_FILE_JOINT_NAME_SPHERE,
                rel_transformation=current_transform_sphere,
                overwrite=True,
            )

            sphere_save_success = self.pm_calibration_utils.save_joint_config(joint_name=HexapodConstants.CALIBRATION_FILE_JOINT_NAME_SPHERE,
                                   rel_transformation=current_transform_sphere,
                                   overwrite=True)
            
            J__T__C = inverse_ros_transform(C__T__J, output_type=Transform)
            B__T__C = multiply_ros_transforms(B__T__J, J__T__C, output_type=Transform)

            smarpod_cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                joint_name=HexapodConstants.CALIBRATION_FILE_JOINT_NAME,
                rel_transformation=B__T__C,
                overwrite=True,
            )
            
            goal_metadata = sc.goal_handle or {}
            calibration_metadata = {
                "cal_id": goal_metadata.get("cal_id",""),
                "comments": goal_metadata.get("comments",""),
                "measurement_file_path": request.results_file_path,
                "fit_sphere_diameter": bool(request.fit_sphere_diameter),
                "calibration_sphere_diameter_mm": float(analysis.diameter_mm),
                "initial_sphere_diameter_mm": float(analysis.initial_diameter_mm),
            }
            sphere_cal_dict["metadata"] = calibration_metadata
            smarpod_cal_dict["metadata"] = calibration_metadata
            
            smarpod_save_success = self.pm_calibration_utils.save_joint_config(joint_name=HexapodConstants.CALIBRATION_FILE_JOINT_NAME,
                        rel_transformation=B__T__C,
                        overwrite=True)

            if not (sphere_save_success and smarpod_save_success):
                raise PmRobotError("Saving of Smarpod calibration configuration failed!")

            self.pm_calibration_utils.log_calibration(file_name=HexapodConstants.CALIBRATION_FILE_JOINT_NAME_SPHERE,
                                 calibration_dict=sphere_cal_dict)
            
            self.pm_calibration_utils.log_calibration(file_name=HexapodConstants.CALIBRATION_FILE_JOINT_NAME,
                                 calibration_dict=smarpod_cal_dict)
            
            try:
                comparison_summary = generate_hexapod_calibration_comparison_report(
                    measurement_file_path=request.results_file_path,
                    result_file_path=results_json_path,
                )
                self._logger.info(
                    "Hexapod calibration comparison report generated: "
                    f"{comparison_summary['report_dir']}"
                )
            except Exception as e:
                self._logger.error(
                    "Hexapod calibration assessment succeeded, but comparison "
                    f"report generation failed: {e}"
                )


            self._logger.info(f"Calibration values written successfully to the joint calibration.")
            
            self.spawn_smarpod_calibration_sphere_frame()
            
            response.success = True
            response.message = (
                "Hexapod calibration assessed successfully and written to "
                "the joint configuration."
            )
            
        except FileNotFoundError as e:
            message = (
                "Assessing the hexapod calibration file failed: "
                f"results file does not exist: {request.results_file_path}"
            )
            self._logger.error(message)
            self._logger.debug(str(e))
            response.message = message
            response.success = False

        except PmRobotError as e:
            message = f"Assessing the hexapod calibration file failed: {e}"
            self._logger.error(message)
            response.message = message
            response.success = False

        except Exception as e:
            message = f"Assessing the hexapod calibration file failed unexpectedly: {e}"
            self._logger.error(message)
            response.message = message
            response.success = False

        finally:
            pass

        return response


    
def main(args=None):
    pass


if __name__ == '__main__':
    main()
