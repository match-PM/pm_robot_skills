from rclpy.node import Node
from geometry_msgs.msg import Pose, Transform
from scipy.spatial.transform import Rotation as R
import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
from pm_msgs.srv import EmptyWithSuccess
import numpy as np
from circle_fit import circle_fit
import matplotlib.pyplot as plt
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
import time
import os
import datetime
import copy
from assembly_scene_publisher.py_modules.scene_functions import get_rel_transform_for_frames
from pm_robot_calibration.py_modules.pm_calibration_utils import PmRobotCalibrationUtils

class PmRobotCalibrationGripper:
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

    def calibrate_gripper_xyt_on_camera_bottom(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        try:
            spawn_success, frames, unique_identifier = self.pm_calibration_utils.spawn_frames_for_current_gripper()
            
            if not spawn_success:
                raise PmRobotError("Spawning of gripper frames failed!")
            
            move_success = self.pm_robot_utils.send_t_trajectory_goal_absolut(0.0, 2.0)

            if not move_success:
                raise PmRobotError("Failed to move gripper to rotation 0.0")
            
            # maybe make this a service input?? so that the user can dynamically decide on the number of rotations?

            #rotations = [0.0, 20, 30, 40, 50, 60, 80]
            #rotations = [90, 65, 55, 45, 35, 25, 0]
            # rotations = [90, 60, 30, 0]
            rotations = [0]

            #rotations = [60, 40, 20, 0]

            # move gripper close to camera to calibration start position
            move_to_start_success, move_msg = self.pm_robot_utils.move_camera_top_to_frame(frame_name=self.pm_robot_utils.TCP_CAMERA_BOTTOM,
                                                                                endeffector_override=self.pm_robot_utils.TCP_TOOL,
                                                                                z_offset=-0.05)
            
            if not move_to_start_success:
                raise PmRobotError(f"Failed to move to start position: {move_msg}")

            # convert to rad
            rotations = [r * np.pi / 180.0 for r in rotations]
            
            if len(rotations) == 0:
                raise PmRobotError("No angles for the gripper calibration specified")
            
            distance_list: list[float] = []  
            frame_poses_list:list[Pose] = []      

            for index, rotation in enumerate(rotations):
                move_success = self.pm_robot_utils.send_t_trajectory_goal_absolut(rotation, 2.0)
                self._logger.error("Gripper rotation: " + str(rotation))
                
                if not move_success:
                    raise PmRobotError("Failed to move gripper to rotation: " + str(rotation))
         
                for frame in frames:
                    
                    if 'Vision' in frame or 'vision' in frame:
                        correct_frame_success = self.pm_calibration_utils.correct_frame_vison(frame).success
                        #correct_frame_success = self.pm_calibration_utils.measure_frame(frame)
                        
                        if not correct_frame_success:
                            raise PmRobotError("Failed to correct frame: " + frame)
                
                ref_frame = self.pm_robot_utils.assembly_scene_analyzer.get_ref_frame_by_name(f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP')
                
                if ref_frame is None:
                    raise PmRobotError("Failed to get reference frame...")
                
                if index !=0:
                    pose_1 = frame_poses_list[index-1]
                    pose_2 = ref_frame.pose
                    distance = np.sqrt((pose_1.position.x - pose_2.position.x)**2 + (pose_1.position.y - pose_2.position.y)**2)
                    distance_list.append(distance)

                frame_poses_list.append(copy.deepcopy(ref_frame.pose))
            
            relative_transform:Transform = get_rel_transform_for_frames(scene=self.pm_robot_utils.assembly_scene_analyzer._get_scene(),
                                        from_frame=f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP',
                                        to_frame=f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP_initial',
                                        tf_buffer=self.pm_robot_utils.tf_buffer,
                                        logger=self._logger)
            
            # Check if calibration was done without rotation (single rotation of 0)
            if len(rotations) > 1 and len(distance_list) > 0:
                min_distance = min(distance_list)
                max_distance = max(distance_list)
                # fit a circle to the points to find the center
                        
                #2.3656241026821336e-07
                self._logger.info("Min distance: " + str(min_distance * 1e6) + " um")
                self._logger.info(f" length of frame_poses_list: {len(frame_poses_list)}")
                
                # if the rotational axis is not ideal all the frames in the 'frame_poses_list' form a circle. We saved all the distances between the points in the circle
                # if we are already calibrated, the points should be very close together and not form a circle.
                # we check the distances between the points, if the max_distance exeedes a threshold we calibrate the axis offset

                #if distance > 20 * 1e-6:
                # If the rotation axis need to be corrected
                if max_distance > 20 * 1e-6:
                    # plot a circle through all the poses
                    x,y,r,s = self.find_circle_coordinates(copy.copy(frame_poses_list))
                    self._logger.info("Circle center: " + str(x) + ", " + str(y) + ", radius (um): " + str(r* 1e6) + ", s: " + str(s))
                    
                    rel_t_joint = Transform()
                    rel_t_joint.translation.x = (relative_transform.translation.x - x)
                    rel_t_joint.translation.y = (relative_transform.translation.y - y)
                    
                    relative_transform.translation.x = x
                    relative_transform.translation.y = y
                    
                    self._logger.warning(f"T-Axis has offset: {x* 1e6}, {y* 1e6} um")
                    
                    self._logger.error(f"Translation of the rotation point")
                    self._logger.error(f"x offset: {rel_t_joint.translation.x * 1e6} um")
                    self._logger.error(f"y offset: {rel_t_joint.translation.y * 1e6} um")
                    
                    t_axis_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                        calibration_dict=self.pm_calibration_utils._transform_to_dict(rel_t_joint),
                        joint_name='T_Axis_Joint',
                        rel_transformation=rel_t_joint,
                    )

                    self.pm_calibration_utils.save_joint_config('T_Axis_Joint', rel_t_joint)

                    self.pm_calibration_utils.log_calibration(file_name='calibrate_gripper_T_axis', calibration_dict=t_axis_dict)

                    self.plot_gripper_calibration_poses(copy.copy(frame_poses_list),
                                                        radius=r,
                                                        circle_x=x,
                                                        circle_y=y)
                    
                # assuming the gripper is at the center of the circle
                else:
                    self._logger.warning("T-Axis has no offset...")
            else:
                # Single rotation calibration (no multi-rotation axis correction needed)
                self._logger.info("Calibrating gripper without rotation - single pose calibration")

            # log relative pose
            #self._logger.error("Relative pose: " + str(relative_transform))
            self._logger.error("Translation of the gripper tip")
            self._logger.error(f"x offset: {relative_transform.translation.x * 1e6} um")
            self._logger.error(f"y offset: {relative_transform.translation.y * 1e6} um")
            
            cal_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
                calibration_dict=self.pm_calibration_utils._transform_to_dict(relative_transform),
                joint_name='PM_Robot_Tool_TCP_Joint',
                rel_transformation=relative_transform,
            )
            self.pm_calibration_utils.save_joint_config('PM_Robot_Tool_TCP_Joint', relative_transform)
            self.pm_calibration_utils.log_calibration(file_name='PM_Robot_Tool_TCP_Joint', calibration_dict=cal_dict)

            response.success = True

        except PmRobotError as e:
            self._logger.error(f"Calibrate Gripper failed: {e}")
            response.success = False

        finally:
            # move out of danger zone
            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)


        return response

    def calibrate_gripper_plane(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()
        
        spawn_success, frames, unique_identifier = self.pm_calibration_utils.spawn_frames_for_current_gripper()
        
        result_dict = {}
        if not spawn_success:
            self._logger.error("Failed to spawn gripper frames...")
            response.success = False
            return response
        
        move_success = self.pm_robot_utils.send_t_trajectory_goal_absolut(0.0, 0.5)

        if not move_success:
            self._logger.error("Failed to move gripper to rotation 0.0")
            response.success = False
            return response

        result_list = []
        # we now measure the rotational deviations of the gripper
        for frame in frames:                      
            if 'Laser' in frame or 'laser' in frame:
                correct_frame_success = self.pm_calibration_utils.correct_frame_confocal_bottom(frame)
                
                if not correct_frame_success:
                    self._logger.error("Failed to correct frame: " + frame)
                    response.success = False
                    return response
                
                ref_frame: ami_msg.RefFrame = self.pm_robot_utils.assembly_scene_analyzer.get_ref_frame_by_name(frame)

                x_value = ref_frame.pose.position.x
                y_value = ref_frame.pose.position.y
                z_value = ref_frame.pose.position.z

                z_joint = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)
                self._logger.error(f"z joint: {z_joint*1e3} mm")
                result_list.append({#"x_value": int(x_value*1e6), 
                                    #"y_value": int(y_value*1e6), 
                                    "frame_name": frame,
                                    "z_value": round(z_value*1e6,1), 
                                    "z_joint": round(z_joint*1e6,1)})

        default_transformation = Transform()

        
        relative_transform_angle:Transform =    get_rel_transform_for_frames(scene=self.pm_robot_utils.assembly_scene_analyzer._get_scene(),
                                        from_frame=f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP_initial',
                                        to_frame=f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP',
                                        tf_buffer=self.pm_robot_utils.tf_buffer,
                                        logger=self._logger)
        
        roll, pitch, yaw = R.from_quat([relative_transform_angle.rotation.x, 
                                        relative_transform_angle.rotation.y,
                                        relative_transform_angle.rotation.z,
                                        relative_transform_angle.rotation.w,]).as_euler('xyz', degrees=True)
        
        self._logger.warning(f"Results - Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

        result_dict['results'] = result_list
        result_dict["rx"] = round(roll, 5)
        result_dict["ry"] = round(pitch, 5)
        result_dict["rz"] = round(yaw, 5)
        result_dict["z"] = round(relative_transform_angle.translation.z*1e6, 1)

        default_transformation.rotation = relative_transform_angle.rotation
        default_transformation.translation.z = relative_transform_angle.translation.z
        
        result_dict = self.pm_calibration_utils.add_joint_value_update_to_calibration_dict(
            calibration_dict=result_dict,
            joint_name='PM_Robot_Tool_TCP_Joint',
            rel_transformation=default_transformation,
            overwrite=False,
        )

        self.pm_calibration_utils.save_joint_config('PM_Robot_Tool_TCP_Joint', 
                               default_transformation, 
                               overwrite=False)

        # move out of danger zone
        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.04, 0.5)

        self.pm_calibration_utils.log_calibration(file_name='calibrate_gripper_plane',
                             calibration_dict=result_dict)
        
        response.success = True
        return response

    def find_circle_coordinates(self, poses:list[Pose]):
        points = []
        for pose in poses:
            points.append([pose.position.x*1e6, pose.position.y*1e6])    
        # Fit a circle to the points
        x, y, r, s = circle_fit.hyperLSQ(points)
        x = x * 1e-6
        y = y * 1e-6
        r = r * 1e-6
        return x,y,r,s

    def plot_gripper_calibration_poses(self, frame_poses_list: list[Pose], 
                                    radius: float, 
                                    circle_x: float, 
                                    circle_y: float):
        
        fig, ax = plt.subplots()
        ax.set_title('Gripper Calibration Poses')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')

        for pose in frame_poses_list:
            x = pose.position.x * 1e6
            y = pose.position.y * 1e6
            ax.scatter(x, y, c='r', marker='o')

        # Create and add the circle to the axes
        circle = plt.Circle((circle_x * 1e6, circle_y * 1e6), radius * 1e6, color='b', fill=False)
        ax.add_patch(circle)  # ← This is what was missing

        ax.grid(True)
        ax.set_aspect('equal')  # Ensures circle is not distorted
        #plt.show()
        path = os.path.join(self.pm_calibration_utils.get_calibration_log_dir_for_current_mode(), 'gripper_plots', '')

        # check path
        if not os.path.exists(path):
            os.makedirs(path)

        file_dir = os.path.join(path, f'gripper_calibration_poses_{datetime.datetime.now()}.png')

        #save the figure
        fig.savefig(file_dir)   
        self._logger.info(f"Calibration image has been saved to '{file_dir}'.")

def main(args=None):
    pass


if __name__ == '__main__':
    main()
