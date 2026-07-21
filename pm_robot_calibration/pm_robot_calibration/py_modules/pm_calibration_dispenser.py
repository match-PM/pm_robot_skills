from rclpy.node import Node
from geometry_msgs.msg import Transform
import assembly_manager_interfaces.srv as ami_srv
from pm_msgs.srv import EmptyWithSuccess
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
import time
from pm_robot_calibration.py_modules.pm_calibration_utils import PmRobotCalibrationUtils

class PmRobotCalibrationDispenser:
    def __init__(self, node: Node, utils: PmRobotCalibrationUtils):
        self.node = node
        self._logger = node.get_logger()
        self.pm_calibration_utils = utils
        self.pm_robot_utils = utils.pm_robot_utils



    def calibrate_1K_dispenser_xy_on_cam_bottom(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        #frame = '1K_Dispenser_TCP'
        try:
            self.pm_robot_utils.move_1k_dispenser_to_frame(frame_name=self.pm_robot_utils.TCP_CAMERA_BOTTOM,
                                                        z_offset=self.pm_calibration_utils.DISPENSER_TRAVEL_DISTANCE + 0.01)

            self.pm_robot_utils.open_protection()

            time.sleep(1.0)

            self.pm_robot_utils.extend_dispenser()

            time.sleep(1.0)

            time_to_wait = 20
            self._logger.warning(f"You have {time_to_wait} seconds to clean the dispenser needle.")

            for t in range(time_to_wait):
                self._logger.info(f"You have {time_to_wait - t} seconds to clean the dispenser needle.")
                time.sleep(1.0)

            self.pm_robot_utils.move_1k_dispenser_to_frame(frame_name=self.pm_robot_utils.TCP_CAMERA_BOTTOM)

            frame_name = self.spawn_1k_dispenser_cal_frame()
            
            time.sleep(1.0)
            
            success_correct_frame = self.pm_calibration_utils.correct_frame_vison(frame_name).success

            if not success_correct_frame:
                seconds = 100
                self._logger.error(f"Failed to correct 1K dispenser vision frame. You have {seconds} seconds to fix this issue.")

                for t in range(seconds):
                    self._logger.info(f"You have {seconds - t} seconds to fix the issue.")
                    time.sleep(1.0)

                success_correct_frame_2 = self.pm_calibration_utils.correct_frame_vison(frame_name).success

                if not success_correct_frame_2:
                    raise PmRobotError("Failed to correct 1K dispenser vision frame")
            
            # relative_transform:Transform = get_rel_transform_for_frames(scene=self.pm_robot_utils.object_scene,
            #                                 from_frame=self.pm_robot_utils.TCP_1K_DISPENSER,
            #                                 to_frame=frame_name,
            #                                 tf_buffer=self.pm_robot_utils.tf_buffer,
            #                                 logger=self._logger)
            
            relative_transform:Transform = self.pm_robot_utils.get_transform_for_frame(frame_name=frame_name,
                                                                                        parent_frame=self.pm_robot_utils.TCP_1K_DISPENSER)
            

            cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                calibration_dict=self.pm_calibration_utils._transform_to_dict(relative_transform),
                joint_name='1K_Dispenser_TCP_Joint',
                rel_transformation=relative_transform,
            )

            self.pm_calibration_utils.save_joint_config('1K_Dispenser_TCP_Joint', relative_transform)

            self.pm_calibration_utils.log_calibration(file_name='calibrate_1K_dispenser_xy_on_cam_bottom', calibration_dict=cal_dict)
                
            response.success = True
        
        except PmRobotError as e:
            self._logger.error(f"Error occurred while calibrating 1K dispenser: {e}")
            response.success = False

        finally:
            #Always try to reset the dispenser
            try:
                self.pm_robot_utils.retract_dispenser()
                time.sleep(1.0)
                self.pm_robot_utils.close_protection()
                time.sleep(1.0)
                pass
            except PmRobotError as e2:
                self._logger.error(f"Error occurred while resetting 1K dispenser: {e2}")
                response.success = False
            pass

        return response

    def spawn_1k_dispenser_cal_frame(self)->str:

        spawn_request = ami_srv.CreateRefFrame.Request()

        # get current dispenser tip
        self.pm_robot_utils.update_pm_robot_config()
        dispenser_tip = self.pm_robot_utils.pm_robot_config.dispenser_1k.get_current_dispenser_tip()

        frame_name = f"CAL_{dispenser_tip}_Vision_Frame"

        spawn_request.ref_frame.frame_name = frame_name
        spawn_request.ref_frame.parent_frame = self.pm_robot_utils.TCP_1K_DISPENSER


        if not self.pm_robot_utils.client_create_ref_frame.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Client '{self.pm_robot_utils.client_create_ref_frame.srv_name}' not available")

        response:ami_srv.CreateRefFrame.Response = self.pm_robot_utils.client_create_ref_frame.call(spawn_request)

        if not response.success:
            raise PmRobotError(f"Failed to create reference frame: {spawn_request.ref_frame.frame_name}")

        return spawn_request.ref_frame.frame_name

    def calibrate_1K_dispenser_z_on_calibration_cube(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        #frame = '1K_Dispenser_TCP'

        try:
            frame_name = 'Calibration_Cube_Bottom'
            self.pm_robot_utils.move_1k_dispenser_to_frame(frame_name=frame_name,
                                                        z_offset=self.pm_calibration_utils.DISPENSER_TRAVEL_DISTANCE + 0.01)
            
            self.pm_robot_utils.open_protection()

            time.sleep(1.0)

            self.pm_robot_utils.extend_dispenser()

            time.sleep(1.0)

            time_to_wait = 2
            self._logger.warning(f"You have {time_to_wait} seconds to clean the dispenser needle.")

            for t in range(time_to_wait):
                self._logger.info(f"You have {time_to_wait - t} seconds to clean the dispenser needle.")
                time.sleep(1.0)

            self.pm_robot_utils.move_1k_dispenser_to_frame(frame_name=frame_name,
                                                           z_offset =0.003)

            step_inc = 0.5  

            self._logger.info("Starting coarse approach...")
            x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=None,
                                measurement_valid_function = self.pm_robot_utils.check_reference_cube_pressed,
                                length = (0.0, 0.0, 5.0),
                                step_inc = step_inc,
                                total_time = 8.0)
            
            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.0005, 1.0)

            self._logger.info("Starting fine approach...")
            step_inc = 0.05
            x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=None,
                                measurement_valid_function = self.pm_robot_utils.check_reference_cube_pressed,
                                length = (0.0, 0.0, 1.0),
                                step_inc = step_inc,
                                total_time = 8.0)
            
            
            relative_transform:Transform = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_1K_DISPENSER,
                                                                                        parent_frame=frame_name)

            rel_transform = Transform()
            rel_transform.translation.z = -1*relative_transform.translation.z

            cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                calibration_dict=self.pm_calibration_utils._transform_to_dict(rel_transform),
                joint_name='1K_Dispenser_TCP_Joint',
                rel_transformation=rel_transform,
            )

            self.pm_calibration_utils.save_joint_config('1K_Dispenser_TCP_Joint', rel_transform)

            self.pm_calibration_utils.log_calibration(file_name='calibrate_1K_dispenser_z_on_calibration_cube', calibration_dict=cal_dict)
            
            response.success = True
        
        except PmRobotError as e:
            self._logger.error(f"Error occurred while calibrating 1K dispenser: {e}")
            response.success = False

        finally:
            #Always try to reset the dispenser
            try:
                self.pm_robot_utils.retract_dispenser()
                time.sleep(1.0)
                self.pm_robot_utils.close_protection()
                time.sleep(1.0)
                # move out of danger zone
                self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)

            except PmRobotError as e2:
                self._logger.error(f"Error occurred while resetting 1K dispenser: {e2}")
                response.success = False
            pass

        return response
    
def main(args=None):
    pass


if __name__ == '__main__':
    main()
