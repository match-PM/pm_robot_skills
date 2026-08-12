import re
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, Pose
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

import assembly_manager_interfaces.msg as ami_msg
import pm_moveit_interfaces.srv as pm_moveit_srv
import pm_msgs.msg as pm_msg
import pm_msgs.srv as pm_msg_srv
import pm_skills_interfaces.srv as pm_skill_srv
import std_msgs.msg as std_msg
from ament_index_python.packages import get_package_share_directory
from assembly_scene_publisher.py_modules.scene_errors import RefFrameNotFoundError
from pm_robot_modules.submodules.pm_dispense_path_generator import DispenseSequenceGenerator
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_skills.py_modules.pm_skill_domain import PmSkillDomain


class PmDispensingSkills(PmSkillDomain):
    def dispense_2k_unity_done_callback(self, msg: std_msg.Bool):
        if msg.data:
            self.dispense_2k_unity_done_event.set()

    def dispense_2k_at_path(self, request: pm_msg_srv.DispenseAtPath.Request, response:pm_msg_srv.DispenseAtPath.Response):

        try:
            if not request.sequence_file_path:
                raise PmRobotError("sequence_file_path is empty!")
            disp_gen = DispenseSequenceGenerator()
            disp_gen.load_from_file(request.sequence_file_path)

            self.logger.info(f"Loaded dispense sequence from file '{request.sequence_file_path}'!")
            
            move_request = pm_moveit_srv.MoveToFrame.Request()
            move_request.execute_movement = True
            move_request.target_frame = request.target_frame_disp

            self.pm_robot_utils.prepare_dispenser(move_request) 


            if not (self.pm_robot_utils.client_move_robot_1k_dispenser_to_frame.wait_for_service(timeout_sec=1.0)):
                raise PmRobotError(f"Service '{self.pm_robot_utils.client_move_robot_1k_dispenser_to_frame.srv_name}' not available!")
            
            response_move: pm_moveit_srv.MoveToFrame.Response = self.pm_robot_utils.client_move_robot_1k_dispenser_to_frame.call(move_request)

            if not response_move.success:
                raise PmRobotError(f"Failed to move dispenser to frame '{request.target_frame_disp}': {response_move.message}")

            start_joints = Point() 
            start_joints.x = response_move.joint_values[0]
            start_joints.y = response_move.joint_values[1]
            start_joints.z = response_move.joint_values[2]

            transform = self.tf_buffer.lookup_transform("world", request.target_frame_disp, rclpy.time.Time())
            
            start_pose = Pose()
            start_pose.position.x = transform.transform.translation.x
            start_pose.position.y = transform.transform.translation.y
            start_pose.position.z = transform.transform.translation.z
            start_pose.orientation = transform.transform.rotation

            g_code = disp_gen.generate_g_code(start_pose=start_pose,
                                            start_joint_values=start_joints)
            
            disp_gen.save_g_code_to_file(start_pose=start_pose,
                                            start_joint_values=start_joints,
                                            file_path=get_package_share_directory("pm_skills") + "/example_g_code")

            if self.pm_robot_utils.get_mode() == self.pm_robot_utils.REAL_MODE:

                self.logger.warn("Switching off controller!")

                self.pm_robot_utils.set_controller_activation("pm_robot_xyz_axis_controller", activate = False) 

                time.sleep(10.0) # wait for controller switch

                self.pm_robot_utils.set_controller_activation("pm_robot_xyz_axis_controller", True) 

                self.logger.warn("Switching on controller!")

            elif self.pm_robot_utils.get_mode() == self.pm_robot_utils.UNITY_MODE:
                self.logger.warn("Unity mode detected!")

                # self.pm_robot_utils.extend_2k_dispenser()

                self.logger.warn("Switching off controller!")

                self.pm_robot_utils.set_controller_activation("pm_robot_xyz_axis_controller", activate = False) 

                time.sleep(3.0) # wait for controller switch

                # Call the Unity-specific dispensing service. The request carries the
                # start frame so Unity can attach the adhesive bead to that frame's
                # parent component. The service only acks that the motion was
                # accepted/started; completion is signalled later on
                # '/unity_skills/dispense_2k_done'. Clear the event before calling so we
                # don't consume a stale signal from a previous run.
                self.dispense_2k_unity_done_event.clear()
                unity_request = pm_skill_srv.DispensePathUnity.Request()
                unity_request.start_frame = request.target_frame_disp
                unity_response = self.dispense_2k_unity_client.call(unity_request)
                self.logger.info(f"Unity dispensing service response: success={unity_response.success}, message='{unity_response.message}'")

                if not unity_response.success:
                    raise PmRobotError(f"Unity dispensing service failed: {unity_response.message}")

                # Block until Unity reports the dispense motion has finished.
                self.logger.info("Waiting for Unity to finish the dispense motion...")
                if not self.dispense_2k_unity_done_event.wait(timeout=600.0):
                    raise PmRobotError("Timed out waiting for Unity dispense completion signal!")
                self.logger.info("Unity dispense motion finished.")

                self.pm_robot_utils.set_controller_activation("pm_robot_xyz_axis_controller", True)
                time.sleep(5.0) # wait for controller switch
            else:
                # only for gazebo
                self._test_gcode(g_code, start_frame=request.target_frame_disp)

            self.logger.info(f"Generated G-code:\n{g_code}")
            response.success = True     

        except (PmRobotError,
            LookupException,
            ConnectivityException,
            ExtrapolationException,
            FileNotFoundError,
            Exception) as e:
            response.success = False
            response.message = f"Error during dispensing at path: {e}"
            self.logger.error(response.message)

        finally:
            
            # self.pm_robot_utils.retract_2k_dispenser()

            self.pm_robot_utils.retract_dispenser()
            time.sleep(0.5)
            self.pm_robot_utils.close_protection()

            success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0,0,-0.05,time=0.5) # move up after dispensing to be safe
            if not success:
                response.message = response.message + "Failed to move up after dispensing! Please check the robot state!"
                self.logger.error(response.message)
                response.success = False

        return response

    def _test_gcode(self, g_code: str, start_frame:str):

        def extract_xyzf_from_gcode_meters(gcode: str) -> List[Tuple[float, float, float, float]]:
            """
            Extract X, Y, Z coordinates + speed (F) from G-code and convert to meters.

            Special handling:
                - G30: dip → go to position, then return to previous position

            Returns:
                List of tuples (x, y, z, speed)
                - position in meters
                - speed unchanged
            """

            # Capture G-code + X Y Z + optional F
            pattern = r"(G\d+).*?X([-+]?\d*\.?\d+)\s+Y([-+]?\d*\.?\d+)\s+Z([-+]?\d*\.?\d+)(?:\s+F([-+]?\d*\.?\d+))?"
            
            matches = re.findall(pattern, gcode)

            result = []
            last_point: Optional[Tuple[float, float, float]] = None
            last_speed: float = 0.0

            for cmd, x, y, z, f in matches:
                point = (
                    float(x) * 0.001,
                    float(y) * 0.001,
                    float(z) * 0.001
                )

                speed = float(f) if f else last_speed

                if cmd == "G30":
                    # Move to dip position
                    result.append((point[0], point[1], point[2], speed))

                    # Return to previous position
                    if last_point is not None:
                        result.append((last_point[0], last_point[1], last_point[2], last_speed))

                else:
                    result.append((point[0], point[1], point[2], speed))
                    last_point = point
                    last_speed = speed

            return result
    
        move_off_success = self.pm_robot_utils.move_camera_top_to_frame(frame_name=start_frame,
                                                                        z_offset=0.01)
        
        if not move_off_success:
            raise PmRobotError(f"Failed to move off surface for g-code testing on frame '{start_frame}'")
        
        move_succes = self.pm_robot_utils.move_camera_top_to_frame(frame_name=start_frame)

        if not move_succes:
            raise PmRobotError(f"Failed to move to start frame '{start_frame}' for g-code testing")

        coords = extract_xyzf_from_gcode_meters(g_code)

        for x, y, z, f in coords:
            if f <= 0:
                self.logger.warn(f"Non-positive speed {f} in G-code, using default speed 0.01 m/s")
                f = 10

            # convert mm/s to time
            speed_mm_s = 10 
            time = speed_mm_s/f
            self.logger.info(f"Moving to X:{x}, Y:{y}, Z:{z} from frame '{start_frame}'")
            move_success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(x, y, z, time=time)
            if not move_success:
                raise PmRobotError(f"Failed to move to X:{x}, Y:{y}, Z:{z} relative to frame '{start_frame}' during g-code testing")

    def dispense_at_frames_callback(self, request: pm_msg_srv.DispenseAtPoints.Request, response:pm_msg_srv.DispenseAtPoints.Response):
        return self._dispense_at_frames(request, response, use_2k_dispenser=False)

    def dispense_2k_at_frames_callback(self, request: pm_msg_srv.DispenseAtPoints.Request, response:pm_msg_srv.DispenseAtPoints.Response):
        return self._dispense_at_frames(request, response, use_2k_dispenser=True)

    def _dispense_at_frames(self, request: pm_msg_srv.DispenseAtPoints.Request,
                            response:pm_msg_srv.DispenseAtPoints.Response,
                            use_2k_dispenser: bool = False):
        """
        Dispenses at all requested frames with either the 1K or the 2K dispenser.

        The sequence is identical for both dispensers (prepare once, dispense at every
        point, retract afterwards). The 2K dispenser has no protection lid, so the
        protection is only handled for the 1K dispenser.
        """
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

        try:
            if len(request.dispense_points) == 0:
                raise PmRobotError("Dispense points list is empty!")

            dispenser_prepared = False

            for dispense_point in request.dispense_points:

                prop = ami_msg.RefFrameProperties()
                prop.glue_pt_frame_properties.dispense_offset_mm = dispense_point.dispense_z_offset_mm
                prop.glue_pt_frame_properties.is_glue_point = True
                prop.glue_pt_frame_properties.time_ms = dispense_point.time_ms
                prop.glue_pt_frame_properties.has_been_placed = True

                dispense_point: pm_msg.DispensePoint
                move_to_frame_request = pm_moveit_srv.MoveToFrame.Request()

                move_to_frame_request.target_frame = dispense_point.frame_name
                move_to_frame_request.execute_movement = True


                if not dispenser_prepared:
                    if use_2k_dispenser:
                        self.pm_robot_utils.prepare_2k_dispenser(move_to_frame_request)
                    else:
                        self.pm_robot_utils.prepare_dispenser(move_to_frame_request)
                    dispenser_prepared = True

                if use_2k_dispenser:
                    self.pm_robot_utils.dispense_2k_at_frame(move_to_frame_request,
                                                    dispense_point.frame_name,
                                                    time_ms=dispense_point.time_ms,
                                                    dispense_z_offset_mm=dispense_point.dispense_z_offset_mm)
                else:
                    self.pm_robot_utils.dispense_at_frame(move_to_frame_request,
                                                    dispense_point.frame_name,
                                                    time=dispense_point.time_ms,
                                                    dispense_z_offset_mm=dispense_point.dispense_z_offset_mm)

                self.pm_robot_utils.set_frame_properties(dispense_point.frame_name, prop)

            if use_2k_dispenser:
                self.pm_robot_utils.retract_2k_dispenser()
            else:
                self.pm_robot_utils.retract_dispenser()
                time.sleep(0.5)
                self.pm_robot_utils.close_protection()

            response.success = True

        except PmRobotError as e:
            self.get_logger().error(f"Error during dispensing at points: {e.message}")
            response.success = False
            response.message = e.message
        return response



    def dispense_at_frames_adv_callback(self, request: pm_skill_srv.DispenseAtPointsAdv.Request, response:pm_skill_srv.DispenseAtPointsAdv.Response):
        return self._dispense_at_frames_adv(request, response, use_2k_dispenser=False)

    def dispense_2k_at_frames_adv_callback(self, request: pm_skill_srv.DispenseAtPointsAdv.Request, response:pm_skill_srv.DispenseAtPointsAdv.Response):
        return self._dispense_at_frames_adv(request, response, use_2k_dispenser=True)

    def _dispense_at_frames_adv(self, request: pm_skill_srv.DispenseAtPointsAdv.Request,
                                response:pm_skill_srv.DispenseAtPointsAdv.Response,
                                use_2k_dispenser: bool = False):
        """
        Dispenses at the given glue point frames, reading dispense time and offset from the
        frame properties of the assembly scene. Uses the 2K dispenser if requested.
        """
        dispense_request = pm_msg_srv.DispenseAtPoints.Request()
        dispense_response = pm_msg_srv.DispenseAtPoints.Response()

        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()
        try:
            # populate the request
            for dispense_point in request.dispense_points:
                dispense_point_msg = pm_msg.DispensePoint()
                dispense_point_msg.frame_name = dispense_point

                properties = self.pm_robot_utils.assembly_scene_analyzer.get_frame_properties(dispense_point).glue_pt_frame_properties

                if not properties.is_glue_point:
                    raise PmRobotError(f"Frame '{dispense_point}' is not a glue point frame according to the assembly scene analyzer!")

                dispense_point_msg.time_ms = properties.time_ms
                dispense_point_msg.dispense_z_offset_mm = properties.dispense_offset_mm

                if dispense_point_msg.time_ms <= 0:
                    raise PmRobotError(f"Frame '{dispense_point}' has invalid dispense time '{dispense_point_msg.time_ms}' ms. Time must be positive and non-zero!")
            
                if dispense_point_msg.dispense_z_offset_mm <= 0:
                    raise PmRobotError(f"Frame '{dispense_point}' has invalid dispense offset '{dispense_point_msg.dispense_z_offset_mm}' mm. Offset must be positive and non-zero!")

                # THIS IS OPTIONAL DECIDE FOR BEHAVIOUR
                # if properties.has_been_placed:
                #     self._logger.warn(f"Frame '{dispense_point}' has already been marked as placed. Skipping dispensing at this frame to avoid double dispensing!")
                #     continue

                dispense_request.dispense_points.append(dispense_point_msg)

            response_disp = self._dispense_at_frames(dispense_request, dispense_response,
                                                     use_2k_dispenser=use_2k_dispenser)

            response.success = response_disp.success
            response.message = response_disp.message

        except (PmRobotError, RefFrameNotFoundError) as e:
            self.get_logger().error(f"Error during advanced dispensing at points: {e.message}")
            response.success = False
            response.message = str(e)
        return response

