from pathlib import Path
from rclpy.node import Node
import pm_skills_interfaces.srv as skills_srv
import pm_skills_interfaces.action as skills_action
from geometry_msgs.msg import Transform
import assembly_manager_interfaces.srv as ami_srv
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_robot_primitive_skills.py_modules.PmRobotMeasurementError import PmRobotMeasurementError
from pm_robot_calibration.py_modules.hexapod_calibration.sphere_calibration import SphereCalibration
from pm_robot_calibration.py_modules.hexapod_calibration.calibration_analysis import CalibrationAnalysis
from pm_robot_calibration.py_modules.hexapod_calibration.geometry_utils import sphere_z
from pm_robot_calibration.py_modules.hexapod_calibration.hexapod_calibration_runtime import (
    get_calibrate_smarpod_measurement_file_path,
    get_hexapod_calibration_positions_mm,
    get_hexapod_calibration_test_poses,
    get_hexapod_orientation_commands,
    get_smarpod_results_base_path,
    get_smarpod_results_dir,
    get_smarpod_results_json_path,
    get_test_calibrate_smarpod_file_path,
    get_test_calibrate_smarpod_plot_path,
    write_json_file,
)
from pm_robot_calibration.py_modules.hexapod_calibration.pivot_calibration_plot import (
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
    CALIBRATION_FILE_JOINT_NAME = "Smarpod_Station"
    CALIBRATION_FILE_JOINT_NAME_SPHERE = "CAL_Smarpod_J__t__P"
    SMARPOD_CS_PIVOT_BASE_NAME = "Smarpod_Pivot_Base_Origin"
    SMARPOD_CS_PLATFORM_PIVOT = "Smarpod_Top_Plate"

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
            raise PmRobotError(f"Could not move hexapod to pose {pose_id}.")

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
                    move_time=6.0,
                    settle_time=3.0,
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

            self.pm_calibration_utils.pm_robot_utils.send_smarpod_trajectory_goal_absolut(x_joint=0.0, y_joint=0.0, z_joint=0.0, time=2.0)

            if not goal_finished:
                goal_handle.abort()

        return result


    def spawn_ball_frames(self, 
                          reference_frame:str, 
                          reference_parent_frame: str,
                          ball_diameter_mm = 6.35)->list:
        
        # spawn the calibration frames
        request = ami_srv.CreateRefFrame.Request()
        name_stem = "CAL_Smarpod_Ball_"
        grid_distance = 1.0 # mm
        z_curve = 0.0 # mm
        frame_name_list = []
        ref_pose:Transform = self.pm_calibration_utils.pm_robot_utils.get_transform_for_frame(frame_name=reference_frame,
                                                               parent_frame=reference_parent_frame)

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
            request.ref_frame.pose.position.x = ref_pose.translation.x + (entry[0]*grid_distance*1e-3)
            request.ref_frame.pose.position.y = ref_pose.translation.y + (entry[1]*grid_distance*1e-3)
            # case
            mult = 0
            if (abs(entry[0]) >=1) and  (abs(entry[1]) >=1):
                mult = 1
            if ((abs(entry[0]) >=1) and  (abs(entry[1]) ==0) or (abs(entry[0]) ==0) and  (abs(entry[1]) >=1)):
                mult = 1
            if (abs(entry[0]) ==0) and  (abs(entry[1]) == 0):
                mult = 0 
            request.ref_frame.pose.position.z = ref_pose.translation.z + ball_diameter_mm/2*1e-3 - mult * z_curve*1e-3
            z_offset = sphere_z(x=entry[0]*grid_distance,
                                y=entry[1]*grid_distance,
                                diameter=ball_diameter_mm)
            
            self._logger.warning(f"z_offset {z_offset}")
            #request.ref_frame.pose.position.z = ref_pose.translation.z + ball_diameter_mm/2*1e-3 - z_offset * 1e-3

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
    
    async def calibrate_smarpod_V3(self, goal_handle: skills_action.SmarpodCalibration.Goal):
        
        goal = goal_handle.request
        use_confocal_top = goal.use_confocal_over_laser

        calibration_data = []
        calibration_run_timestamp = datetime.datetime.now().isoformat()
        calibration_goal_inputs = {
            "use_confocal_over_laser": bool(use_confocal_top),
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
                        move_time=5.0,
                        settle_time=3.0,
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
            self.pm_calibration_utils.pm_robot_utils.send_smarpod_trajectory_goal_absolut(x_joint=0.0, y_joint=0.0, z_joint=0.0, time=2.0)

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

            analysis:CalibrationAnalysis = sc.run_calibration()
            
            results_dir = get_smarpod_results_dir(
                self.pm_calibration_utils.get_calibration_log_dir_for_current_mode(),
                request.results_file_path,
            )
            Path(results_dir).mkdir(parents=True, exist_ok=True)
            
            analysis.save_results(file_path=get_smarpod_results_json_path(results_dir, request.results_file_path))
            
            analysis.plot_results(file_path=get_smarpod_results_base_path(results_dir, request.results_file_path))
            
            euler_angles = analysis.get_B_T_P_euler()

            translation_pivot = analysis.get_B_T_P_translation()

            translation_ball = analysis.get_J_t_P_translation(unit='m')

            B__T__J = analysis.get_B_T_P_ros_transform()

            #self._logger.warn(f"euler: {str(euler_angles)}")
            #self._logger.warn(f"translation pivot: {str(translation_pivot)}")
            #self._logger.warn(f"translation ball: {str(translation_ball)}")
            
            current_transform_sphere = self.pm_calibration_utils.get_current_joint_calibration_transform(HexapodConstants.CALIBRATION_FILE_JOINT_NAME_SPHERE)
            
            current_transform_sphere.translation.x = translation_ball[0]
            current_transform_sphere.translation.y = translation_ball[1]
            current_transform_sphere.translation.z = translation_ball[2]

            #self._logger.warn(f"current transform: {str(current_transform_sphere)}")
            
            default_ball_transform:Transform = self.pm_calibration_utils.pm_robot_utils.get_transform_for_frame(frame_name=HexapodConstants.BALL_ENDEFFECTOR,
                                                            parent_frame=HexapodConstants.SMARPOD_CS_PIVOT_BASE_NAME)
            
            self._logger.warn(f"current transform: {str(default_ball_transform)}")


            trans_fixed_calibrated:Transform = self.pm_calibration_utils.pm_robot_utils.get_transform_for_frame(frame_name=HexapodConstants.FIXED_CS_SMARPOD_FRAME,
                                                                        parent_frame=HexapodConstants.CALIBRATED_CS_SMARPOD_FRAME)
            
            C__T__J:Transform = self.pm_calibration_utils.pm_robot_utils.get_transform_for_frame(frame_name=HexapodConstants.SMARPOD_CS_PIVOT_BASE_NAME,
                                                                        parent_frame=HexapodConstants.CALIBRATED_CS_SMARPOD_FRAME)
            
            #self._logger.warn(f"Current Calibration Value: {str(trans_fixed_calibrated)}")
            #self._logger.warn(f"Calibration Pivot: {str(C__T__J)}")

            sphere_cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                calibration_dict=self.pm_calibration_utils._transform_to_dict(current_transform_sphere),
                joint_name=HexapodConstants.CALIBRATION_FILE_JOINT_NAME_SPHERE,
                rel_transformation=current_transform_sphere,
                overwrite=True,
            )

            self.pm_calibration_utils.save_joint_config(joint_name=HexapodConstants.CALIBRATION_FILE_JOINT_NAME_SPHERE,
                                   rel_transformation=current_transform_sphere,
                                   overwrite=True)
            
            J__T__C = inverse_ros_transform(C__T__J, output_type=Transform)
            B__T__C = multiply_ros_transforms(B__T__J, J__T__C, output_type=Transform)

            smarpod_cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                calibration_dict=self.pm_calibration_utils._transform_to_dict(B__T__C),
                joint_name=HexapodConstants.CALIBRATION_FILE_JOINT_NAME,
                rel_transformation=B__T__C,
                overwrite=True,
            )
            
            self.pm_calibration_utils.save_joint_config(joint_name=HexapodConstants.CALIBRATION_FILE_JOINT_NAME,
                        rel_transformation=B__T__C,
                        overwrite=True)

            self.pm_calibration_utils.log_calibration(file_name=HexapodConstants.CALIBRATION_FILE_JOINT_NAME_SPHERE,
                                 calibration_dict=sphere_cal_dict)
            
            self.pm_calibration_utils.log_calibration(file_name=HexapodConstants.CALIBRATION_FILE_JOINT_NAME,
                                 calibration_dict=smarpod_cal_dict)
            


            self._logger.info(f"Calibration values written successfully to the joint calibration.")
            
            self.spawn_smarpod_calibration_sphere_frame()
            
            response.success = True
            
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
