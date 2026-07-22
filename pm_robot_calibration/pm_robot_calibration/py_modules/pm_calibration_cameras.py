from rclpy.node import Node
from pm_moveit_interfaces.srv import MoveToFrame
from pm_vision_interfaces.srv import ExecuteVision, CalibrateAngle, CalibratePixelPerUm
import pm_vision_interfaces.msg as vision_msg
from geometry_msgs.msg import Transform
from pm_msgs.srv import EmptyWithSuccess
import numpy as np
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_robot_primitive_skills.py_modules.PmRobotMeasurementError import PmRobotMeasurementError
import time
from pm_robot_calibration.py_modules.pm_calibration_utils import PmRobotCalibrationUtils

CAMERA_CALIBRATION_JOINT_VALUES_X = -0.266460 # in m
CAMERA_CALIBRATION_JOINT_VALUES_Y = -0.045949 # in m
CAMERA_CALIBRATION_JOINT_VALUES_Z = 0.002535 # in m

class PmRobotCalibrationCameras:
    def __init__(self, node: Node, utils: PmRobotCalibrationUtils):
        self.node = node
        self._logger = node.get_logger()
        self.pm_calibration_utils = utils
        self.pm_robot_utils = utils.pm_robot_utils
        self.client_calibrate_camera_pixel = utils.client_calibrate_camera_pixel
        self.client_calibrate_camera_angle = utils.client_calibrate_camera_angle


    ###########################################
    ### Calibrate cameras #####################
    ###########################################
    
    def calibrate_cameras_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        self._logger.warning("Starting Camera calibration!")
        
        try:

            self.pm_calibration_utils.set_calibration_platelet_forward()

            request_move_to_frame = MoveToFrame.Request()
            request_move_to_frame.target_frame = 'Camera_Station_TCP'
            request_move_to_frame.execute_movement = True
            request_move_to_frame.translation.z = 0.003
            request_move_to_frame.translation.x = 0.00

            if not self.pm_robot_utils.client_move_robot_cam1_to_frame.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f'Client "{self.pm_robot_utils.client_move_robot_cam1_to_frame.srv_name}" not available...')
            
            response_move_to_frame:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_cam1_to_frame.call(request_move_to_frame)
            
            if not response_move_to_frame.success:
                raise PmRobotError("Failed to move to camera station for calibration")

            move_success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(    x_joint = CAMERA_CALIBRATION_JOINT_VALUES_X,
                                                                                    y_joint = CAMERA_CALIBRATION_JOINT_VALUES_Y,
                                                                                    z_joint = CAMERA_CALIBRATION_JOINT_VALUES_Z,
                                                                                    time=1)
            
            if not move_success:
                response.success = False
                return response

            # this loop is run twice.
            # the first loop calibrates the pixel size and the angle
            # the second loop the displacement of the cooridnate systems
            for iter in range(2):
                # measure frame with bottom cam
                request_execute_vision_bottom = ExecuteVision.Request()
                request_execute_vision_bottom.camera_config_filename = self.pm_robot_utils.get_cam_file_name_bottom()
                request_execute_vision_bottom.image_display_time = -1
                request_execute_vision_bottom.process_filename = "PM_Robot_Calibration/Camera_Calibration_Bottom_Process.json"
                request_execute_vision_bottom.process_uid = f"Cam_Cal_B_{iter}"

                if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
                    raise PmRobotError(f'Client "{self.pm_robot_utils.client_execute_vision.srv_name}" not available...')
                
                response_execute_vision_bottom:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_bottom)
                
                if not response_execute_vision_bottom.success:
                    raise PmRobotError("Failed to execute vision for bottom camera")
                
                # measure frame with top cam
                request_execute_vision_top = ExecuteVision.Request()
                request_execute_vision_top.camera_config_filename = self.pm_robot_utils.get_cam_file_name_top()
                request_execute_vision_top.image_display_time = -1
                request_execute_vision_top.process_filename = "PM_Robot_Calibration/Camera_Calibration_Top_Process.json"
                request_execute_vision_top.process_uid = f"Cam_Cal_T_{iter}"

                if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
                    raise PmRobotError(f'Client "{self.pm_robot_utils.client_execute_vision.srv_name}" not available...')

                response_execute_vision_top:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_top)

                if not response_execute_vision_top.success:
                    raise PmRobotError("Failed to execute vision for top camera")
                
                # Only for the first iteration
                if iter == 0:
                    process_success = self._process_calibrate_cameras(vision_result_top=response_execute_vision_top.vision_response.results,
                                            vision_result_bottom=response_execute_vision_bottom.vision_response.results)
                    
                    if not process_success:
                        raise PmRobotError("Failed to calibrate cameras")

            calibrate_cs_success = self._process_calibrate_coordinate_systems(vision_result_top=response_execute_vision_top.vision_response.results,
                                                                            vision_result_bottom=response_execute_vision_bottom.vision_response.results)
            
            if not calibrate_cs_success:
                raise PmRobotError("Failed to calibrate coordinate systems")

            response.success = process_success
            self.pm_calibration_utils.set_calibration_platelet_backward()
            
        except (PmRobotError, PmRobotMeasurementError) as e:
            self._logger.error(f"Error occurred during camera calibration: {e}")
            response.success = False

        finally:
            self._logger.info(f"Camera calibration process completed with success: {response.success}")
            try:
                #self.pm_calibration_utils.set_calibration_platelet_backward()
                pass

            except PmRobotError as e2:
                self._logger.error(f"Error occurred while moving calibration target backward: {e2}")
                response.success = False
        return response

    def _process_calibrate_coordinate_systems(self, vision_result_top: vision_msg.VisionResults,
                                vision_result_bottom: vision_msg.VisionResults)->bool:
        
        circle_0_top:vision_msg.VisionCircle = vision_result_top.circles[0]
        circle_1_top:vision_msg.VisionCircle = vision_result_top.circles[1]
        circle_2_top:vision_msg.VisionCircle = vision_result_top.circles[2]
        circle_3_top:vision_msg.VisionCircle = vision_result_top.circles[3]

        circle_0_bottom:vision_msg.VisionCircle = vision_result_bottom.circles[0]
        circle_1_bottom:vision_msg.VisionCircle = vision_result_bottom.circles[1]
        circle_2_bottom:vision_msg.VisionCircle = vision_result_bottom.circles[2]
        circle_3_bottom:vision_msg.VisionCircle = vision_result_bottom.circles[3]

        d_x_0 = circle_0_top.center_point.axis_value_1 - circle_0_bottom.center_point.axis_value_1
        d_y_0 = circle_0_top.center_point.axis_value_2 - circle_0_bottom.center_point.axis_value_2

        d_x_1 = circle_1_top.center_point.axis_value_1 - circle_1_bottom.center_point.axis_value_1
        d_y_1 = circle_1_top.center_point.axis_value_2 - circle_1_bottom.center_point.axis_value_2

        d_x_2 = circle_2_top.center_point.axis_value_1 - circle_2_bottom.center_point.axis_value_1
        d_y_2 = circle_2_top.center_point.axis_value_2 - circle_2_bottom.center_point.axis_value_2

        d_x_3 = circle_3_top.center_point.axis_value_1 - circle_3_bottom.center_point.axis_value_1
        d_y_3 = circle_3_top.center_point.axis_value_2 - circle_3_bottom.center_point.axis_value_2

        d_x = (d_x_0 + d_x_1 + d_x_2 + d_x_3)/4
        d_y = (d_y_0 + d_y_1 + d_y_2 + d_y_3)/4


        self._logger.error(f"Results x: {d_x_0}, {d_x_1}, {d_x_2}, {d_x_3}")
        self._logger.error(f"Results y: {d_y_0}, {d_y_1}, {d_y_2}, {d_y_3}")

        self._logger.error(f"Result x: {d_x}")
        self._logger.error(f"Result y: {d_y}")

        rel_trans = Transform()

        rel_trans.translation.x = d_x * 1e-6
        rel_trans.translation.y = d_y * 1e-6

        time.sleep(1.0)

        trans = self.pm_robot_utils.get_transform_for_frame(frame_name='Cam1_Toolhead_TCP',
                                                            parent_frame='Camera_Station_TCP')

        #rel_trans.translation.x = (rel_trans.translation.x + trans.translation.x)
        #rel_trans.translation.y = (rel_trans.translation.y + trans.translation.y)
        rel_trans.translation.x = -1*trans.translation.x - rel_trans.translation.x
        rel_trans.translation.y = -1*trans.translation.y - rel_trans.translation.y


        self._logger.error(f"Result trans x: {rel_trans.translation.x}")
        self._logger.error(f"Result trans y: {rel_trans.translation.y}")


        cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
            calibration_dict=self.pm_calibration_utils._transform_to_dict(rel_trans),
            joint_name='Cam1_Toolhead_TCP_Joint',
            rel_transformation=rel_trans,
            overwrite=False,
        )

        save_success = self.pm_calibration_utils.save_joint_config ( joint_name='Cam1_Toolhead_TCP_Joint',
                                                rel_transformation=rel_trans,
                                                overwrite=False)

        self.pm_calibration_utils.log_calibration(file_name='Cam1_Toolhead_TCP_Joint', calibration_dict=cal_dict)


        save_success = True

        return save_success
        

    def _process_calibrate_cameras(self, vision_result_top: vision_msg.VisionResults,
                                   vision_result_bottom: vision_msg.VisionResults)->bool:

        ######## At this point, we can start with the calculations for the calibraiton
        # Top image
        self._logger.error(f"Processing Top Vision Circles")
        top_angle, top_dist = self._process_four_circles(vision_result_top.circles)

        self._logger.error(f"Processing Bottom Vision Circles")
        bottom_angle, bottom_dist = self._process_four_circles(vision_result_bottom.circles)

        if (not self.client_calibrate_camera_angle.wait_for_service(1)):
            self._logger.error(f"Service not available {self.client_calibrate_camera_angle.srv_name}")
            return False
        
        if not self.client_calibrate_camera_pixel.wait_for_service(1):
            self._logger.error(f"Service not available {self.client_calibrate_camera_pixel.srv_name}")
            return False
        
        FIDUCIAL_DISTANCE = 1000
        # Set the results for the bottom cam
        angle = 0
        pixel_multiplicator = FIDUCIAL_DISTANCE/bottom_dist

        request_pixel = CalibratePixelPerUm.Request()
        request_pixel.multiplicator = float(pixel_multiplicator)
        request_pixel.camera_config_file_name = self.pm_robot_utils.get_cam_file_name_bottom()

        request_angle = CalibrateAngle.Request()
        request_angle.angle_diff = float(angle)
        request_angle.camera_config_file_name = self.pm_robot_utils.get_cam_file_name_bottom()

        response_pixel: CalibratePixelPerUm.Response = self.client_calibrate_camera_pixel.call(request_pixel)
        response_angle: CalibrateAngle.Response = self.client_calibrate_camera_angle.call(request_angle)

        result_dict = {}
        result_dict["bottom_camera"] = {
            "pixel": request_pixel.multiplicator,
            "angle": request_angle.angle_diff}

        if (not response_angle.success and response_pixel.success):
            return False

        # Set the results for the top cam
        pixel_multiplicator = FIDUCIAL_DISTANCE/top_dist

        request_pixel = CalibratePixelPerUm.Request()
        request_pixel.multiplicator = float(pixel_multiplicator)
        request_pixel.camera_config_file_name = self.pm_robot_utils.get_cam_file_name_top()

        request_angle = CalibrateAngle.Request()
        request_angle.angle_diff = float(top_angle)
        #request_angle.angle_diff = 0.0

        request_angle.camera_config_file_name = self.pm_robot_utils.get_cam_file_name_top()

        response_pixel: CalibratePixelPerUm.Response = self.client_calibrate_camera_pixel.call(request_pixel)
        response_angle: CalibrateAngle.Response = self.client_calibrate_camera_angle.call(request_angle)

        result_dict["top_camera"] = {
            "pixel": request_pixel.multiplicator,
            "angle": request_angle.angle_diff
        }

        self.pm_calibration_utils.log_calibration(file_name='Camera_Calibration_Angle', calibration_dict=result_dict)

        return (response_pixel.success and response_angle.success)

    def _process_four_circles(self, circle_list: list[vision_msg.VisionCircle])-> tuple[float, float]:
        circle_ind_0 = 0
        circle_ind_1 = 1
        circle_ind_2 = 2
        circle_ind_3 = 3

        if len(circle_list) != 4:
            raise PmRobotMeasurementError(f"Camera calibration expects to find 4 circles. However {len(circle_list)} have been found!")

        circle_0 = circle_list[0]
        circle_1 = circle_list[1]
        circle_2 = circle_list[2]
        circle_3 = circle_list[3]

        average_radius = (circle_0.radius + circle_1.radius + circle_2.radius + circle_3.radius)/len(circle_list) 

        max_radius = max([circle_0.radius, circle_1.radius, circle_2.radius, circle_3.radius])
        min_radius = min([circle_0.radius, circle_1.radius, circle_2.radius, circle_3.radius])

        def distance_between_centers(circle_a:vision_msg.VisionCircle, circle_b:vision_msg.VisionCircle):
            dx = circle_a.center_point.axis_value_1 - circle_b.center_point.axis_value_1
            dy = circle_a.center_point.axis_value_2 - circle_b.center_point.axis_value_2
            return np.sqrt(dx**2 + dy**2)

        length_01 = distance_between_centers(circle_0, circle_1)
        length_13 = distance_between_centers(circle_1, circle_3)
        length_32 = distance_between_centers(circle_3, circle_2)
        length_20 = distance_between_centers(circle_2, circle_0)

        def angle_between(a:vision_msg.VisionCircle, b:vision_msg.VisionCircle):
            dx = b.center_point.axis_value_1 - a.center_point.axis_value_1
            dy = b.center_point.axis_value_2 - a.center_point.axis_value_2
            return np.degrees(np.arctan2(dy, dx))

        angle_01 = angle_between(circle_0, circle_1)-180
        angle_13 = angle_between(circle_1, circle_3)-90
        angle_32 = angle_between(circle_3, circle_2)-0
        angle_20 = angle_between(circle_2, circle_0)+90

        if abs(angle_01)>90:
            angle_01 = -(angle_01+360) 

        self._logger.warning(f"Average radius: {average_radius} um")
        self._logger.warning(f"max radius: {max_radius} um")
        self._logger.warning(f"min radius: {min_radius} um")
        self._logger.warning(f"Radius: {average_radius} um")
        self._logger.warning(f"Radius: [{circle_0.radius}, {circle_1.radius}, {circle_2.radius}, {circle_3.radius}] um")
        self._logger.warning(f"Distances: [{length_01}, {length_13}, {length_32}, {length_20}] um")
        self._logger.warning(f"Angles: [{angle_01}, {angle_13}, {angle_32}, {angle_20}] um")

        average_angle = (angle_01 + angle_13 + angle_20 + angle_32)/4

        average_length = (length_01 + length_13 + length_32 + length_20)/4

        self._logger.warning(f"Avg angle: {average_angle} um")
        self._logger.warning(f"Avg length: {average_length} um")

        return (average_angle, average_length)

    def calibrate_calibration_cube_to_cam_top(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):

        self._logger.warning(f"Starting calibration 'calibrate_calibration_cube_to_cam_top'...")
        move_up = False
        try:
            # Spawn the frames

            self.pm_calibration_utils.spawn_calibration_frames('CF_Calibration_Qube_Cam_Top.json')

            unique_identifier = self.pm_calibration_utils.get_unique_identifier('CF_Calibration_Qube_Cam_Top.json')

            vision_request = ExecuteVision.Request()
            vision_request.process_filename = f"Assembly_Manager/Frames/{unique_identifier}Vision_Dynamic.json"
            vision_request.process_uid = "Cal_Calibrate_Cube"
            vision_request.image_display_time = -1
            vision_request.camera_config_filename = self.pm_robot_utils.get_cam_file_name_top()


            if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
                raise PmRobotError("Service 'ExecuteVision' not available...")

            frame_name = f'{unique_identifier}Vision_Dynamic'
            move_success_offset, move_offset_msg = self.pm_robot_utils.move_camera_top_to_frame(frame_name=frame_name,
                                                                                            z_offset=0.05)

            if not move_success_offset:
                raise PmRobotError(f"Could not move the camera to frame {frame_name}: {move_offset_msg}")
            
            move_success, move_msg = self.pm_robot_utils.move_camera_top_to_frame(frame_name=frame_name)

            if not move_success:
                raise PmRobotError(f"Could not move the camera to frame {frame_name}: {move_msg}")

            # Measure the frame
            result:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(vision_request)

            if not result.success:
                raise PmRobotError("Vision process failed...")
                
            #detected_point:vision_msg.VisionPoint = result.vision_response.results.points[0]
            if len(result.vision_response.results.circles) != 1:
                raise PmRobotMeasurementError(f"Did not find single circle while measuring the calibration cube. Found {len(result.vision_response.results.circles)}!")
            
            circle:vision_msg.VisionCircle = result.vision_response.results.circles[0]
            detected_point:vision_msg.VisionPoint = circle.center_point

            
            rel_transform = Transform()
            rel_transform.translation.x = detected_point.axis_value_1 * 1e-6
            rel_transform.translation.y = detected_point.axis_value_2 * 1e-6

            cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                calibration_dict=self.pm_calibration_utils._transform_to_dict(rel_transform),
                joint_name='Calibration_Qube_Joint',
                rel_transformation=rel_transform,
            )

            success = self.pm_calibration_utils.save_joint_config('Calibration_Qube_Joint', 
                                             rel_transform)
            
            self.pm_calibration_utils.log_calibration(file_name='calibrate_calibration_cube_to_cam_top', 
                                 calibration_dict=cal_dict)
            
            if not success:
                raise PmRobotError("Failed to save joint config...")
            
            response.success = True
            move_up = True
        
        except (PmRobotError, PmRobotMeasurementError) as e:
            response.success = False
            self._logger.error(f"Calibration failed: {e}")

        finally:
            # move out of danger zone
            if move_up:
                self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 0.5)

        return response

def main(args=None):
    pass


if __name__ == '__main__':
    main()
