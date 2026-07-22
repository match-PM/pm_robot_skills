from rclpy.node import Node
from pm_moveit_interfaces.srv import MoveToFrame
from pm_vision_interfaces.srv import ExecuteVision
import pm_vision_interfaces.msg as vision_msg
from geometry_msgs.msg import Transform
from pm_msgs.srv import EmptyWithSuccess
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
import time
from pm_robot_calibration.py_modules.pm_calibration_utils import PmRobotCalibrationUtils

class PmRobotDistanceSensorCalibration:
    def __init__(self, node: Node, utils: PmRobotCalibrationUtils):
        self.node = node
        self._logger = node.get_logger()
        self.pm_calibration_utils = utils
        self.pm_robot_utils = utils.pm_robot_utils

    def calibrate_laser_xy_on_camera_bottom(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        """
        The functionality of the laser calibration has been verified by manually moving the laser to the camera tcp. By experts!!!
        """
        try:
            self._logger.warning(f"Starting calibration 'calibrate_laser_xy_on_camera_bottom'...")

            CAMERA_TARGET_HEIGHT = 1.6 #    mm - this is not needed as the fiducials are on top of the platelet
            
            self.pm_calibration_utils.set_calibration_platelet_forward()

            move_success = self.pm_robot_utils.move_laser_to_frame('Calibration_Platelet_Calibration_Frame',
                                                                z_offset=0.05)

            if not move_success:
                raise PmRobotError("Failed to move laser to frame")
            
            move_success = self.pm_robot_utils.move_laser_to_frame('Calibration_Platelet_Calibration_Frame',
                                                                z_offset=0.00)

            if not move_success:
                raise PmRobotError("Failed to move laser to frame")           

            if not self.pm_robot_utils._check_for_valid_laser_measurement():
                raise PmRobotError("Could not get a valid laser measurement!")
            
            initial_z_height = self.pm_robot_utils.get_laser_measurement(unit='m')
            
            self._logger.error(f'Heigt {initial_z_height}')

            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 1.0)

            time.sleep(1)

            # move to a position where the laser is visible
            z_joint_target = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)
            x_joint_target = -0.4447
            y_joint_target = -0.0317

            move_success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(   
                                                                    x_joint= x_joint_target,
                                                                    y_joint = y_joint_target,
                                                                    z_joint = z_joint_target,
                                                                    time=0.5)

            if not move_success:
                response.success = False
                return response
            
            request_execute_vision_bottom = ExecuteVision.Request()
            request_execute_vision_bottom.camera_config_filename = self.pm_robot_utils.get_cam_file_name_bottom()
            request_execute_vision_bottom.image_display_time = -1
            request_execute_vision_bottom.process_filename = "PM_Robot_Calibration/Calibration_Laser_xy_on_Camera_Bottom.json"
            request_execute_vision_bottom.process_uid = f"Laser_xy_on_Cam_Bottom"

            if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
                raise PmRobotError('Vision Manager not available...')

            response_execute_vision_bottom:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_bottom)
            
            if not response_execute_vision_bottom.success:
                raise PmRobotError("Failed to execute vision for bottom camera")
            
            if len(response_execute_vision_bottom.vision_response.results.circles) != 1:
                raise PmRobotError("Vision did not find a single circle!")
            
            circle:vision_msg.VisionCircle = response_execute_vision_bottom.vision_response.results.circles[0]

            x_offset = circle.center_point.axis_value_1
            y_offset = circle.center_point.axis_value_2

            rel_trans = Transform()

            time.sleep(2.0)

            transfrom_camera_TCP = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_LASER,
                                                                parent_frame=self.pm_robot_utils.TCP_CAMERA_BOTTOM)


            #self._logger.error(f"Transform x: {transfrom.translation.x}")
            #self._logger.error(f"Transform y: {transfrom.translation.y}")

            rel_trans.translation.x = transfrom_camera_TCP.translation.x - x_offset * 1e-6
            rel_trans.translation.y = transfrom_camera_TCP.translation.y - y_offset * 1e-6

            self._logger.warning(f"Correction value x: {rel_trans.translation.x*1e6} um")
            self._logger.warning(f"Correction value y: {rel_trans.translation.y*1e6} um")

            rel_trans.translation.x = -1*rel_trans.translation.x
            rel_trans.translation.y = -1*rel_trans.translation.y

            rel_trans_camera_b_tcp = Transform()

            # this seems to be the right solution, but it does not work. it seems that the camera focus point is closer to the top surface of the camera calibration platelet
            #rel_trans_camera_b_tcp.translation.z = transfrom_camera_TCP.translation.z - CAMERA_TARGET_HEIGHT*1e-3
            empirical_value = 0.0005       # verified by experts
            rel_trans_camera_b_tcp.translation.z = transfrom_camera_TCP.translation.z + empirical_value

            
            laser_cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                calibration_dict=self.pm_calibration_utils._transform_to_dict(rel_trans),
                joint_name='Laser_Toolhead_TCP_Joint',
                rel_transformation=rel_trans,
                overwrite=False,
            )
            camera_bottom_cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                calibration_dict=self.pm_calibration_utils._transform_to_dict(rel_trans_camera_b_tcp),
                joint_name='Camera_Station_TCP_Joint',
                rel_transformation=rel_trans_camera_b_tcp,
                overwrite=False,
            )

            save_success = self.pm_calibration_utils.save_joint_config ( joint_name='Laser_Toolhead_TCP_Joint',
                                                    rel_transformation=rel_trans,
                                                    overwrite=False)
            
            save_success = self.pm_calibration_utils.save_joint_config ( joint_name='Camera_Station_TCP_Joint',
                                                    rel_transformation=rel_trans_camera_b_tcp,
                                                    overwrite=False)
            
            # log results
            self.pm_calibration_utils.log_calibration(file_name="calibrate_laser_xy_on_camera_bottom",
                                calibration_dict=laser_cal_dict)
            
            self.pm_calibration_utils.log_calibration(file_name="calibrate_camera_bottom_tcp_z_on_laser_z",
                                calibration_dict=camera_bottom_cal_dict)

            if not save_success:
                raise PmRobotError("Saving of configuration failed!")

            # move out of danger zone
            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.02, 1.0)
            
            response.success = True
        
        except PmRobotError as e:
            self._logger.error(f"Error occurred during 'calibrate_confocal_bottom_xy_on_cam_top': {e}")
            response.success = False

        finally:
            self._logger.info(f"Camera calibration process completed with success: {response.success}")
            try:
                self.pm_calibration_utils.set_calibration_platelet_backward()

            except PmRobotError as e2:
                self._logger.error(f"Error occurred while moving calibration target backward: {e2}")
                response.success = False
        return response

    def calibrate_confocal_top_xy_on_camera_bottom(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):

        try:
            self._logger.warning(f"Starting calibration 'calibrate_confocal_top_xy_on_camera_bottom'...")

            move_success, move_msg = self.pm_robot_utils.move_confocal_top_to_frame(self.pm_robot_utils.TCP_CAMERA_BOTTOM,
                                                                          z_offset=0.05)

            if not move_success:
                raise PmRobotError(f"Failed to move confocal top to frame: {move_msg}")
            
            self.pm_calibration_utils.set_calibration_platelet_forward()
            
            move_success, move_msg = self.pm_robot_utils.move_confocal_top_to_frame(self.pm_robot_utils.TCP_CAMERA_BOTTOM,
                                                                          z_offset=0.002)

            if not move_success:
                raise PmRobotError(f"Failed to move confocal top to frame: {move_msg}")
            
            if not self.pm_robot_utils.check_confocal_top_measurement_in_range():
                            
                step_inc = 1.0 # in mm
                self._logger.warning(f"Laser measurement not valid! Trying to iteratively find a valid value!")                

                x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
                                                measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
                                                length = (0.0, 0.0, 4.0),
                                                step_inc = step_inc,
                                                total_time = 2.0)
                
                if x is None:
                    raise PmRobotError(f"Laser measurement not valid! OUT OF RANGE")
            
            initial_z_height = self.pm_robot_utils.get_confocal_top_measurement(unit='m')
            
            self._logger.error(f"Measured confocal heigt '{initial_z_height*1e-6}' um")

            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 1.0)

            time.sleep(1)

            initial_z_height = self.pm_robot_utils.get_confocal_top_measurement(unit='m')
            
            request_execute_vision_bottom = ExecuteVision.Request()
            request_execute_vision_bottom.camera_config_filename = self.pm_robot_utils.get_cam_file_name_bottom()
            request_execute_vision_bottom.image_display_time = -1
            request_execute_vision_bottom.process_filename = "PM_Robot_Calibration/Calibration_Confocal_Top_xy_on_Camera_Bottom.json"
            request_execute_vision_bottom.process_uid = f"Confocal_xy_on_Cam_Bottom"

            if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
                raise PmRobotError('Vision Manager not available...')

            # first try
            response_execute_vision_bottom:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_bottom)
            
            if not response_execute_vision_bottom.success:
                raise PmRobotError("Failed to execute vision for bottom camera")
            
            if len(response_execute_vision_bottom.vision_response.results.circles) != 1:
                raise PmRobotError("Vision did not find a single circle!")

            circle:vision_msg.VisionCircle = response_execute_vision_bottom.vision_response.results.circles[0]

            x_offset = circle.center_point.axis_value_1
            y_offset = circle.center_point.axis_value_2

            time.sleep(1.0)

            self.pm_robot_utils.send_xyz_trajectory_goal_relative(x_joint_rel=-x_offset*1e-6,
                                                                  y_joint_rel=-y_offset*1e-6,
                                                                  z_joint_rel=0.0, 
                                                                  time=0.5)

            # second try
            response_execute_vision_bottom:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_bottom)
            
            if not response_execute_vision_bottom.success:
                raise PmRobotError("Failed to execute vision for bottom camera")
            
            if len(response_execute_vision_bottom.vision_response.results.circles) != 1:
                raise PmRobotError("Vision did not find a single circle!")

            circle:vision_msg.VisionCircle = response_execute_vision_bottom.vision_response.results.circles[0]

            x_offset = circle.center_point.axis_value_1
            y_offset = circle.center_point.axis_value_2

            time.sleep(1.0)

            rel_trans = Transform()
    
            transform = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_CONFOCAL_TOP,
                                                                parent_frame=self.pm_robot_utils.TCP_CAMERA_BOTTOM)
            
            rel_trans.translation.x = transform.translation.x - x_offset * 1e-6
            rel_trans.translation.y = transform.translation.y - y_offset * 1e-6

            self._logger.warning(f"Correction value x: {rel_trans.translation.x*1e6} um")
            self._logger.warning(f"Correction value y: {rel_trans.translation.y*1e6} um")

            rel_trans.translation.x = -1*rel_trans.translation.x
            rel_trans.translation.y = -1*rel_trans.translation.y
            
            cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                calibration_dict=self.pm_calibration_utils._transform_to_dict(rel_trans),
                joint_name='Confocal_Sensor_Top_TCP_Joint',
                rel_transformation=rel_trans,
                overwrite=False,
            )

            save_success = self.pm_calibration_utils.save_joint_config ( joint_name='Confocal_Sensor_Top_TCP_Joint',
                                                    rel_transformation=rel_trans,
                                                    overwrite=False)

            self.pm_calibration_utils.log_calibration(file_name="calibrate_confocal_top_xy_on_camera_bottom",
                                 calibration_dict=cal_dict)

            if not save_success:
                raise PmRobotError("Saving of configuration failed!")    

            response.success = True
    
        except PmRobotError as e:
            self._logger.error(f"Error occurred during 'calibrate_confocal_top_xy_on_camera_bottom': {e}")
            response.success = False

        finally:
            self._logger.info(f"Camera calibration process completed with success: {response.success}")
            try:
                self.pm_calibration_utils.set_calibration_platelet_backward()
                # move out of danger zone
                self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.02, 1.0)      

            except PmRobotError as e2:
                self._logger.error(f"Error occurred while moving calibration target backward: {e2}")
                response.success = False
        return response

    def calibrate_confocal_bottom_xy_on_cam_top(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        try:
            self._logger.warning(f"Starting calibration 'calibrate_confocal_bottom_xy_on_cam_top'...")
            
            self.pm_calibration_utils.set_calibration_platelet_forward()
            
            request_move_to_frame = MoveToFrame.Request()
            request_move_to_frame.target_frame = 'Laser_Height_Calibration_Frame'
            request_move_to_frame.execute_movement = True
            request_move_to_frame.translation.z = -0.00387

            if not self.pm_robot_utils.client_move_robot_cam1_to_frame.wait_for_service(timeout_sec=1.0):
                raise PmRobotError('Camera move service not available...')
            
            response_move_to_frame:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_cam1_to_frame.call(request_move_to_frame)

            if not response_move_to_frame.success:
                raise PmRobotError("Failed to move laser to frame")

            time.sleep(1)
            z_joint_target = 0.0010185
            x_joint_target = -0.2091909
            y_joint_target = -0.05865595
            
            move_success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(   
                                                                    x_joint= x_joint_target,
                                                                    y_joint = y_joint_target,
                                                                    z_joint = z_joint_target,
                                                                    time=0.1)

            if not move_success:
                raise PmRobotError("TBD")
            
            x_offset, y_offset = self.pm_calibration_utils._get_circle_from_vision(process_file_name="Calibration_Confocal_Bottom_xy_on_Camera_Top.json",
                                                            camera_file_name=self.pm_robot_utils.get_cam_file_name_top(),
                                                            process_name="Confocal_xy_on_Cam_Top")
            
            if x_offset is None:
                raise PmRobotError("TBD")
            
            #self._logger.error(f"Camera measurement {x_offset} um, {y_offset} um")

            time.sleep(2.0)

            rel_trans = Transform()

            transfrom = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_CONFOCAL_BOTTOM,
                                                                parent_frame=self.pm_robot_utils.TCP_CAMERA_TOP)
            
            #self._logger.error(f"Transform x: {transfrom.translation.x}")
            #self._logger.error(f"Transform y: {transfrom.translation.y}")

            rel_trans.translation.x = transfrom.translation.x - x_offset * 1e-6
            rel_trans.translation.y = transfrom.translation.y - y_offset * 1e-6

            self._logger.warning(f"Result x: {rel_trans.translation.x*1e6} um")
            self._logger.warning(f"Result y: {rel_trans.translation.y*1e6} um")

            rel_trans.translation.x = -1*rel_trans.translation.x
            rel_trans.translation.y = -1*rel_trans.translation.y
            
            cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                calibration_dict=self.pm_calibration_utils._transform_to_dict(rel_trans),
                joint_name='Confocal_Sensor_Bottom_TCP_Joint',
                rel_transformation=rel_trans,
                overwrite=False,
            )

            save_success = self.pm_calibration_utils.save_joint_config ( joint_name='Confocal_Sensor_Bottom_TCP_Joint',
                                                    rel_transformation=rel_trans,
                                                    overwrite=False)

            if not save_success:
                raise PmRobotError("Saving of configuration failed!")

            self.pm_calibration_utils.log_calibration(file_name='calibrate_confocal_bottom_xy_on_cam_top',
                                calibration_dict=cal_dict)

            # move out of danger zone
            #self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)
            #backward_response:EmptyWithSuccess.Response = self.pm_robot_utils.client_move_calibration_target_backward.call(backward_request)

            response.success = True
            
        except PmRobotError as e:
            self._logger.error(f"Error occurred during 'calibrate_confocal_bottom_xy_on_cam_top': {e}")
            response.success = False

        finally:
            self._logger.info(f"Camera calibration process completed with success: {response.success}")
            try:
                self.pm_calibration_utils.set_calibration_platelet_backward()

            except PmRobotError as e2:
                self._logger.error(f"Error occurred while moving calibration target backward: {e2}")
                response.success = False
        return response

    def calibrate_confocal_bottom_z_on_laser(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        try:

            self._logger.warning(f"Starting calibration 'calibrate_confocal_bottom_z_on_laser'...")
            
            self.pm_calibration_utils.set_calibration_platelet_forward()
                    
            move_success = self.pm_robot_utils.move_laser_to_frame(frame_name='TCP_Confocal_Sensor_Bottom',
                                                                z_offset=0.004,
                                                                y_offset=-0.003)

            if not move_success:
                raise PmRobotError("Failed to move laser to frame")

            if not self.pm_robot_utils.check_confocal_bottom_measurement_in_range():
                raise PmRobotError("Could not get a valid confocal bottom measurement!")
            
            if not self.pm_robot_utils._check_for_valid_laser_measurement():

                move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0, 0, -3.0*1e-3,time=0.5)
                                                
                if not move_success:
                    raise PmRobotError("TBD")
                
                step_inc = 0.4 # in mm
                self._logger.warning(f"Laser measurement not valid! Trying to iteratively find a valid value!")                

                x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
                                                measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
                                                length = (0.0, 0.0, 8.0),
                                                step_inc = step_inc,
                                                total_time = 8.0)
                
                if x is None:
                    raise PmRobotError(f"Laser measurement not valid! OUT OF RANGE")
                    
            initial_z_height = self.pm_robot_utils.get_laser_measurement(unit='m')
            
            self._logger.warning(f"Measured laser heigt '{initial_z_height*1e6}' um")

            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 1.0)

            confocal_bottom_measurement = self.pm_robot_utils.get_confocal_bottom_measurement(unit='m')

            self._logger.warning(f"Measured confocal bottom heigt '{confocal_bottom_measurement*1e6}' um")

            time.sleep(1.0)

            # beginning the calculations

            rel_trans = Transform()
            
            transfrom = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_LASER,
                                                                parent_frame=self.pm_robot_utils.TCP_CONFOCAL_BOTTOM)
            
            self._logger.warning(f"Transform from Confocal bottom to laser - z: {transfrom.translation.z*1e6} um.")

            rel_trans.translation.z = transfrom.translation.z

            distance_between_laser_confocal_bottom = self.pm_calibration_utils.LASER_CALIBRATION_TARGET_THICKNESS*1e-3 - confocal_bottom_measurement

            #rel_trans.translation.z = -1*(distance_between_laser_confocal_bottom - rel_trans.translation.z )

            rel_trans.translation.z = rel_trans.translation.z - distance_between_laser_confocal_bottom

            self._logger.warning(f"Calibration result z: {rel_trans.translation.z*1e6} um.")

            calibration_dict = {}
            calibration_dict["transform"] = self.pm_calibration_utils._transform_to_dict(rel_trans)
            calibration_dict["confocal_bottom_measurement"] = round(confocal_bottom_measurement*1e6,1)
            calibration_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                calibration_dict=calibration_dict,
                joint_name='Confocal_Sensor_Bottom_TCP_Joint',
                rel_transformation=rel_trans,
                overwrite=False,
            )

            save_success = self.pm_calibration_utils.save_joint_config ( joint_name ='Confocal_Sensor_Bottom_TCP_Joint',
                                                    rel_transformation=rel_trans,
                                                    overwrite=False)

            if not save_success:
                raise PmRobotError(f"Saving of configuration failed!")

            # move out of danger zone
            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)

            self.pm_calibration_utils.log_calibration(file_name='calibrate_confocal_bottom_z_on_laser',
                                calibration_dict=calibration_dict)

            response.success = True
                
        except PmRobotError as e:
            self._logger.error(f"Error occurred during 'calibrate_confocal_bottom_xy_on_cam_top': {e}")
            response.success = False

        finally:
            self._logger.info(f"Camera calibration process completed with success: {response.success}")
            try:
                self.pm_calibration_utils.set_calibration_platelet_backward()

            except PmRobotError as e2:
                self._logger.error(f"Error occurred while moving calibration target backward: {e2}")
                response.success = False
        return response

    def calibrate_confocal_top_z_on_laser(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        try:
            self._logger.warning(f"Starting calibration 'calibrate_confocal_top_z_on_laser'...")

            #frame_name = 'Calibration_Platelet_Calibration_Frame'
            frame_name = 'Calibration_Qube'

            x_offset = -0.003
            y_offset = -0.003

            move_success = self.pm_robot_utils.move_laser_to_frame(frame_name=frame_name,
                                                                    z_offset=0.05,
                                                                    x_offset=x_offset,
                                                                    y_offset=y_offset)

            if not move_success:
                raise PmRobotError("Failed to move laser to frame")

            #self.pm_calibration_utils.set_calibration_platelet_forward()

            move_success = self.pm_robot_utils.move_laser_to_frame(frame_name=frame_name,
                                                                    z_offset=0.002,
                                                                    x_offset=x_offset,
                                                                    y_offset=y_offset)

            if not move_success:
                raise PmRobotError("Failed to move laser to frame")
            
            step_inc = 0.4 # in mm
            self._logger.warning(f"Laser measurement not valid! Trying to iteratively find a valid value!")                

            x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
                                            measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
                                            length = (0.0, 0.0, 3.0),
                                            step_inc = step_inc,
                                            total_time = 4.0)
            
            if x is None:
                raise PmRobotError(f"Laser measurement not valid! OUT OF RANGE")
                    
            initial_z_height = self.pm_robot_utils.get_laser_measurement(unit='m')

            #self._logger.error(f"Measured laser height measurement'{initial_z_height*1e6}' um")

            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 0.5)

            time.sleep(2.0)

            laser_transform = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_LASER,
                                                                parent_frame='world')

            laser_transform_z = laser_transform.translation.z

            self._logger.error(f"Laser offset value' {laser_transform_z*1e6}' um")

            if not self.pm_robot_utils.client_move_robot_confocal_top_to_frame.wait_for_service(timeout_sec=1.0):
                raise PmRobotError('Camera move service not available...')
            
            move_success, move_msg = self.pm_robot_utils.move_confocal_top_to_frame(frame_name=frame_name,
                                                                          x_offset=x_offset,
                                                                          y_offset=y_offset)

            if not move_success:
                raise PmRobotError(f"Failed to move laser to frame: {move_msg}")
           
            if not self.pm_robot_utils.check_confocal_top_measurement_in_range():
                
                # move 3 mm up
                move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0, 0, -3.0*1e-3,time=0.3)
                                                
                if not move_success:
                    raise PmRobotError("TBD")
                
                step_inc = 1.0 # in mm
                self._logger.warning(f"Laser measurement not valid! Trying to iteratively find a valid value!")                

                x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
                                                measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
                                                length = (0.0, 0.0, 4.0),
                                                step_inc = step_inc,
                                                total_time = 2.0)
                
                if x is None:
                    raise PmRobotError(f"Laser measurement not valid! OUT OF RANGE")

            confocal_top_measurement = self.pm_robot_utils.get_confocal_top_measurement('m')

            #self._logger.warning(f"Measured confocal heigt measurement '{confocal_top_measurement*1e6}' um")

            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -confocal_top_measurement, 0.5)

            time.sleep(2.0)

            confocal_transform = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_CONFOCAL_TOP,
                                                                            parent_frame='world')

            confocal_transform_z = confocal_transform.translation.z

            self._logger.error(f"Confocal offset value' {confocal_transform_z*1e6}' um")

            rel_trans = Transform()
            
            transform_z = (confocal_transform_z - laser_transform_z)

            rel_trans.translation.z = transform_z

            self._logger.warning(f"Calibration value z: {rel_trans.translation.z*1e6} um.")

            cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                calibration_dict=self.pm_calibration_utils._transform_to_dict(rel_trans),
                joint_name='Confocal_Sensor_Top_TCP_Joint',
                rel_transformation=rel_trans,
                overwrite=False,
            )

            save_success = self.pm_calibration_utils.save_joint_config ( joint_name='Confocal_Sensor_Top_TCP_Joint',
                                                    rel_transformation=rel_trans,
                                                    overwrite=False)

            self.pm_calibration_utils.log_calibration(file_name='calibrate_confocal_top_z_on_laser', 
                                 calibration_dict=cal_dict)

            if not save_success:
                raise PmRobotError("Saving of configuration failed!") 

            response.success = True
        
        except PmRobotError as e:
            self._logger.error(f"Error occurred during 'calibrate_confocal_top_z_on_laser': {e}")
            response.success = False

        finally:
            self._logger.info(f"Camera calibration process completed with success: {response.success}")
            try:
                self.pm_calibration_utils.set_calibration_platelet_backward()
                # Move out of the danger zone
                self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)

            except PmRobotError as e2:
                self._logger.error(f"Error occurred while moving calibration target backward: {e2}")
                response.success = False
        return response

    def calibrate_calibration_cube_z_on_confocal_top(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        try:
            self._logger.warning(f"Starting calibration 'calibrate_calibration_cube_z_on_confocal_top'...")

            move_confocal_success, move_msg = self.pm_robot_utils.move_confocal_top_to_frame(frame_name='Calibration_Cube_Bottom',
                                                                                   z_offset=0.05)
            
            if not move_confocal_success:
                raise PmRobotError(f"Moving confocal top to frame failed: {move_msg}")

            move_confocal_success, move_msg = self.pm_robot_utils.move_confocal_top_to_frame(frame_name='Calibration_Cube_Bottom',
                                                                                   z_offset=0.002)

            if not move_confocal_success:
                raise PmRobotError(f"Moving confocal top to frame failed: {move_msg}")

            step_inc = 0.5
            x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
                    measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
                    length = (0.0, 0.0, 4.0),
                    step_inc = step_inc,
                    total_time = 2.0)
                
            if x is None:
                raise PmRobotError(f"Laser measurement not valid! OUT OF RANGE")

            confocal_measurement = self.pm_robot_utils.get_confocal_top_measurement('m')

            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -confocal_measurement, 1.0)

            confocal_transform = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_CONFOCAL_TOP,
                                                                parent_frame='Calibration_Cube_Bottom')
            
            result_transform =Transform()
            result_transform.translation.z = confocal_transform.translation.z

            cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                calibration_dict=self.pm_calibration_utils._transform_to_dict(result_transform),
                joint_name='Calibration_Cube_Bottom_Joint',
                rel_transformation=result_transform,
            )

            save_success = self.pm_calibration_utils.save_joint_config ( joint_name='Calibration_Cube_Bottom_Joint',
                                        rel_transformation=result_transform)

            if not save_success:
                raise PmRobotError(f"Failed to save joint configuration for 'Calibration_Cube_Bottom_Joint'")

            self.pm_calibration_utils.log_calibration(file_name='calibrate_calibration_cube_z_on_confocal_top',
                                calibration_dict=cal_dict)
            
            response.success = True

        except PmRobotError as e:
            self._logger.error(f"Error occurred while calibrating: {e}")
            response.success = False
        
        finally:
            # move up
            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)
            pass

        return response

def main(args=None):
    pass


if __name__ == '__main__':
    main()
