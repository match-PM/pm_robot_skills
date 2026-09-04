import sys
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import pm_skills_interfaces.srv as skills_srv
from pm_moveit_interfaces.srv import MoveToPose,  MoveToFrame
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from pm_vision_interfaces.srv import ExecuteVision
import pm_vision_interfaces.msg as vision_msg
from geometry_msgs.msg import Vector3, TransformStamped, Pose, PoseStamped, Quaternion
import pm_skills_interfaces.srv as pm_skill_srv

from pm_vision_manager.va_py_modules.vision_assistant_class import VisionProcessClass
from assembly_scene_publisher.py_modules.scene_errors import *
from assembly_manager_interfaces.msg import RefFrameProperties
import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
from assembly_scene_publisher.py_modules.tf_functions import get_transform_for_frame_in_world
from scipy.spatial.transform import Rotation as R
import time

from pm_skills.py_modules.PmRobotUtils import PmRobotUtils
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError

from assembly_scene_publisher.py_modules.scene_errors import (RefAxisNotFoundError, 
                                                              RefFrameNotFoundError, 
                                                              RefPlaneNotFoundError, 
                                                              ComponentNotFoundError)

class VisionSkillsNode(Node):
    SERVICE_CALL_TIMEOUT_SEC = 60.0

    def __init__(self):
        super().__init__('pm_skills')
        self._logger = self.get_logger()

        self.callback_group = MutuallyExclusiveCallbackGroup()

        self._logger.info(f"Node '{self.get_name()}' started...")

        self.srv_measure = self.create_service(skills_srv.MeasureFrameVision, self.get_name()+'/vision_measure_frame', self.measure_frame, callback_group=self.callback_group)
        self.srv_correct = self.create_service(skills_srv.CorrectFrameVision, self.get_name()+'/vision_correct_frame', self.correct_frame, callback_group=self.callback_group)

        self.srv_check_frame_measureble_cam_top = self.create_service(skills_srv.CheckFrameMeasurable, self.get_name()+'/check_frame_measureble_cam_top', self.check_frame_mes_cam_top, callback_group=self.callback_group)   
        self.srv_check_frame_measureble_cam_bottom = self.create_service(skills_srv.CheckFrameMeasurable, self.get_name()+'/check_frame_measureble_cam_bottom', self.check_frame_mes_cam_bottom, callback_group=self.callback_group)   
    
        self.tf_buffer = Buffer()
        
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pm_robot_utils = PmRobotUtils(self)
        # overwriting the callback function!!
        self.pm_robot_utils.object_scene_callback = self.object_scene_callback
        self.pm_robot_utils.start_object_scene_subscribtion()   
        self._last_measure_camera_tcp_frame = None

    def _call_service_with_timeout(self,
                                   client,
                                   request,
                                   service_name: str,
                                   timeout_sec: float = SERVICE_CALL_TIMEOUT_SEC):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec

        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        
        if not future.done():
            client.remove_pending_request(future)
            raise PmRobotError(
                f"Service '{service_name}' did not return within {timeout_sec:.1f} seconds."
            )

        if future.exception() is not None:
            raise PmRobotError(f"Service '{service_name}' failed: {future.exception()}")

        result = future.result()
        if result is None:
            raise PmRobotError(f"Service '{service_name}' returned no response.")

        return result


    def measure_frame(self, 
                      request:skills_srv.MeasureFrameVision.Request, 
                      response:skills_srv.MeasureFrameVision.Response):
        
        # get compenent id if possible
        comp_id = "None"
        component_name = "Not a component frame"
        is_from_component = False
        try:
            component_name = self.pm_robot_utils.assembly_scene_analyzer.get_component_for_frame_name(request.frame_name)
            component = self.pm_robot_utils.assembly_scene_analyzer.get_component_by_name(component_name)
            comp_id = component.uuid
            is_from_component = True

        except (ComponentNotFoundError) as e:
            component_name = "None"
            comp_id = "None"

        except (RefFrameNotFoundError) as e:
            message = str(e)
            raise PmRobotError(message)

        response.component_name = component_name
        response.component_uuid = comp_id

        vision_request = ExecuteVision.Request()
        vision_request.process_filename = request.vision_process_file_name

        if is_from_component:
            vision_request.process_uid = f"Component ID: {comp_id}, Frame: {request.frame_name}"
        else:
            vision_request.process_uid = f"Frame: {request.frame_name}"

        try:
            vision_request.image_display_time = 15

            if request.frame_name == '':
                raise PmRobotError("Frame name is empty. Please provide a valid frame name to measure!")
            self._last_measure_camera_tcp_frame = None
            
            if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
                raise PmRobotError("Service 'ExecuteVision' not available...")
            
            self._logger.info("Service 'ExecuteVision' is available...")

            # try to move the frame to the bottom camera
            move_request = MoveToFrame.Request()
            move_request.execute_movement = True
            move_request.endeffector_frame_override = request.frame_name
            move_request.target_frame = 'Camera_Station_TCP'

            #self._logger.error(f"Moving tool to frame: {request.frame_name}... executing movement: {move_request.execute_movement} edgeffector frame override: {move_request.endeffector_frame_override} target frame: {move_request.target_frame}")
            
            # select the bottom camera
            if self.pm_robot_utils.get_mode() == self.pm_robot_utils.REAL_MODE:
                vision_request.camera_config_filename = 'pm_robot_basler_bottom_cam_2.yaml'

            elif self.pm_robot_utils.get_mode() == self.pm_robot_utils.UNITY_MODE:
                self._logger.info("Unity is running. Using simulated camera...")
                vision_request.camera_config_filename = 'pm_robot_bottom_cam_2_Unity.yaml'
            else:
                raise PmRobotError("Gazebo mode is not supported for this operation!")

            #if not self.pm_robot_utils.client_move_robot_tool_to_frame.wait_for_service(timeout_sec=1.0):
            if not self.pm_robot_utils.client_move_robot_cam1_to_frame.wait_for_service(timeout_sec=1.0):
                raise PmRobotError("Service 'MoveCamToFrame' not available...")

            #result_tool_to_bottom_cam:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_tool_to_frame.call(move_request)
            result_tool_to_bottom_cam:MoveToFrame.Response = self._call_service_with_timeout(
                self.pm_robot_utils.client_move_robot_cam1_to_frame,
                move_request,
                'MoveCamToFrame'
            )

            #self._logger.warn(f"MOOOOVOEE SUCCCEESS {result_tool_to_bottom_cam.success}")
            
            # If the tool cannot be moved to the bottom camera, try to move it to the top camera
            if not result_tool_to_bottom_cam.success:
                self._logger.warn("Can not move frame to the bottom cam. Trying to reach frame with the top camera...")

                # selct the top camera
                if self.pm_robot_utils.get_mode() == self.pm_robot_utils.REAL_MODE:
                    vision_request.camera_config_filename = 'pm_robot_basler_top_cam_1.yaml'

                elif self.pm_robot_utils.get_mode() == self.pm_robot_utils.UNITY_MODE:
                    self._logger.info("Unity is running. Using simulated camera...")
                    vision_request.camera_config_filename = 'pm_robot_top_cam_1_Unity.yaml'

                else:
                    raise PmRobotError("Gazebo mode is not supported for this operation!")

                move_request = MoveToFrame.Request()
                move_request.execute_movement = True
                move_request.target_frame = request.frame_name

                if not self.pm_robot_utils.client_move_robot_cam1_to_frame.wait_for_service(timeout_sec=1.0):
                    raise PmRobotError("Service 'MoveCamToFrame' not available...")
                
                result_move_tool:MoveToFrame.Response = self._call_service_with_timeout(
                    self.pm_robot_utils.client_move_robot_cam1_to_frame,
                    move_request,
                    'MoveCamToFrame'
                )

                if not result_move_tool.success:
                    raise PmRobotError("Failed to move tool to frame with both cameras. Cannot execute vision measurement!")
                self._last_measure_camera_tcp_frame = self.pm_robot_utils.TCP_CAMERA_TOP
            else:
                self._last_measure_camera_tcp_frame = self.pm_robot_utils.TCP_CAMERA_BOTTOM

            # Measure the frame
            if (self.pm_robot_utils.get_mode() == self.pm_robot_utils.REAL_MODE or 
                self.pm_robot_utils.get_mode() == self.pm_robot_utils.UNITY_MODE):

                if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
                    raise PmRobotError("Service 'ExecuteVision' not available.")

                # vision_available = self.pm_robot_utils.check_available_client_execute_vision()

                # if not vision_available:
                #     self._logger.error("Vision client not available...")
                #     response.success= False
                #     return response
                #self._logger.warn(f"Executing vision process for frame: {request.frame_name}...")
                #self._logger.warn(f"Using process file: {vision_request.process_filename}...")
                #self._logger.warn(f"Using camera config file: {vision_request.camera_config_filename}...")
                
                result:ExecuteVision.Response = self._call_service_with_timeout(
                    self.pm_robot_utils.client_execute_vision,
                    vision_request,
                    'ExecuteVision'
                )

                response.vision_response = result.vision_response

                if not result.success:
                    raise PmRobotError("Vision process failed!")
                
                detected_circles = result.vision_response.results.circles
                detected_points = result.vision_response.results.points

                if len(detected_points) == 0 and len(detected_circles) == 0:
                    raise PmRobotError("No points and circles detected. Vision process failed to detect the required features!")
                
                if len(detected_points) > 1 or len(detected_circles) > 1:
                    raise PmRobotError("Multiple points or circles detected. Please make sure that the vision process detects only one feature!")


                multiplier = 0.000001 # Convert to m  
                result_angle = 0.0
                
                if len(detected_circles) == 0:
                    self._logger.warn("No circles detected...")
                
                else:
                    result_vector = Vector3()
                    detected_cricle:vision_msg.VisionCircle = detected_circles[0]
                    
                    if detected_cricle.center_point.axis_suffix_1 == 'x' or detected_cricle.center_point.axis_suffix_1 == 'X':
                        result_vector.x = detected_cricle.center_point.axis_value_1*multiplier
                        
                    if detected_cricle.center_point.axis_suffix_1 == 'y' or detected_cricle.center_point.axis_suffix_1 == 'Y':
                        result_vector.y = detected_cricle.center_point.axis_value_1*multiplier

                    if detected_cricle.center_point.axis_suffix_1 == 'z' or detected_cricle.center_point.axis_suffix_1 == 'Z':
                        result_vector.z = detected_cricle.center_point.axis_value_1*multiplier

                    if detected_cricle.center_point.axis_suffix_2 == 'x' or detected_cricle.center_point.axis_suffix_2 == 'X':
                        result_vector.x = detected_cricle.center_point.axis_value_2*multiplier

                    if detected_cricle.center_point.axis_suffix_2 == 'y' or detected_cricle.center_point.axis_suffix_2 == 'Y':
                        result_vector.y = detected_cricle.center_point.axis_value_2*multiplier

                    if detected_cricle.center_point.axis_suffix_2 == 'z' or detected_cricle.center_point.axis_suffix_2 == 'Z':
                        result_vector.z = detected_cricle.center_point.axis_value_2*multiplier

                    result_angle = detected_cricle.center_point.angle


                if len(detected_points) == 0:
                    self._logger.warn("No points detected...")

                else:
                    self._logger.warn(f"Detected points: {detected_points}")
                    detected_point:vision_msg.VisionPoint = detected_points[0]
                    result_vector = Vector3()
                    
                    if detected_point.axis_suffix_1 == 'x' or detected_point.axis_suffix_1 == 'X':
                        result_vector.x = detected_point.axis_value_1*multiplier
                    
                    if detected_point.axis_suffix_1 == 'y' or detected_point.axis_suffix_1 == 'Y':
                        result_vector.y = detected_point.axis_value_1*multiplier

                    if detected_point.axis_suffix_1 == 'z' or detected_point.axis_suffix_1 == 'Z':
                        result_vector.z = detected_point.axis_value_1*multiplier

                    if detected_point.axis_suffix_2 == 'x' or detected_point.axis_suffix_2 == 'X':
                        result_vector.x = detected_point.axis_value_2*multiplier

                    if detected_point.axis_suffix_2 == 'y' or detected_point.axis_suffix_2 == 'Y':
                        result_vector.y = detected_point.axis_value_2*multiplier

                    if detected_point.axis_suffix_2 == 'z' or detected_point.axis_suffix_2 == 'Z':
                        result_vector.z = detected_point.axis_value_2*multiplier

                    result_angle = detected_point.angle
                                
                result_vector.x = result_vector.x
                result_vector.y = result_vector.y
                result_vector.z = result_vector.z
                response.result_vector = result_vector     
                response.result_angle = result_angle
                response.success = result.success
                
                #self._logger.info(f"Vision process executed...{result}")
            else:
                # Gazebo is running
                response.success = True

        except PmRobotError as e:
            self._logger.error(str(e))
            response.success = False
            response.message = str(e)
       
        finally:
            return response
    
    def correct_frame(self, 
                      request:skills_srv.CorrectFrameVision.Request, 
                      response:skills_srv.CorrectFrameVision.Response):
        try:
            self.pm_robot_utils.wait_for_initial_scene_update()
            
            if not self.pm_robot_utils.assembly_scene_analyzer.is_frame_from_scene(request.frame_name):
                raise RefFrameNotFoundError(f"Frame '{request.frame_name}' not found in the current assembly scene.")

            try:
                _obj_name = self.pm_robot_utils.assembly_scene_analyzer.get_component_for_frame_name(request.frame_name)
            
            except ComponentNotFoundError as e:
                _obj_name = None

            # self._logger.warn(f"Correcting frame: {request.frame_name}...")
            # self._logger.warn(f"Object name: {_obj_name}")
            # self._logger.warn(f"Frame name: {_frame_name}")

            measure_frame_request = skills_srv.MeasureFrameVision.Request()
            measure_frame_request.frame_name =request.frame_name

            if self.pm_robot_utils.get_mode() == self.pm_robot_utils.UNITY_MODE:
                extention = '_sim'
            else:
                extention = ''

            if _obj_name is None:
                measure_frame_request.vision_process_file_name = f"Assembly_Manager/Frames/{request.frame_name}{extention}.json"
            else:
                measure_frame_request.vision_process_file_name = f"Assembly_Manager/{_obj_name}/{request.frame_name}{extention}.json"

            self._logger.warn(f"Requesting measure frame for frame: {request.frame_name}...using process file: {measure_frame_request.vision_process_file_name}")
            
            if request.remeasure_after_correction == True:
                iterations = 2

            else:
                iterations = 1

            for i in range(iterations):
                if i > 0:
                    self._logger.info(
                        f"Remeasuring frame '{request.frame_name}' after correction "
                        f"({i + 1}/{iterations})."
                    )

                response_em = skills_srv.MeasureFrameVision.Response()
                result:skills_srv.MeasureFrameVision.Response = self.measure_frame(
                    measure_frame_request,
                    response_em
                )

                response.correction_values = result.result_vector
                response.correction_angle = result.result_angle
                response.vision_response = result.vision_response
                response.component_name = result.component_name
                response.component_uuid = result.component_uuid
                if not result.success:
                    raise PmRobotError("Measurement for correction failed.")
                
                world_pose:TransformStamped = get_transform_for_frame_in_world(request.frame_name, self.tf_buffer, self._logger)

                world_pose.transform.translation.x += result.result_vector.x
                world_pose.transform.translation.y += result.result_vector.y
                world_pose.transform.translation.z += result.result_vector.z

                # Apply the measured rotation. The rotation axis is the one NOT present
                # in the translation result_vector (i.e. the third axis). The translation
                # may be zero in one or both populated axes; the rotation must still be
                # applied around the un-translated axis.
                correction_angle_deg = float(result.result_angle)
                rot_axis = self._get_rotation_axis(result.vision_response)
                modify_orientation = False
                
                if rot_axis is not None and abs(correction_angle_deg) > 1e-9:
                    camera_tcp_frame = self._last_measure_camera_tcp_frame
                    if camera_tcp_frame is None:
                        raise PmRobotError("Could not determine camera TCP used for vision measurement.")
                    camera_pose: TransformStamped = get_transform_for_frame_in_world(
                        camera_tcp_frame,
                        self.tf_buffer,
                        self._logger
                    )
                    self._logger.info(
                        f"Applying rotation of {correction_angle_deg} deg about {rot_axis}-axis "
                        f"to frame '{request.frame_name}' from camera TCP '{camera_tcp_frame}'."
                    )
                    world_pose.transform.rotation = self._apply_world_rotation(
                        camera_pose.transform.rotation, rot_axis, correction_angle_deg
                    )
                    modify_orientation = True
                else:
                    self._logger.debug(
                        f"No rotation applied (angle={correction_angle_deg}, rot_axis={rot_axis})."
                    )

                adapt_frame_request = ami_srv.ModifyPoseAbsolut.Request()
                adapt_frame_request.frame_name = request.frame_name
                adapt_frame_request.pose.position.x = world_pose.transform.translation.x
                adapt_frame_request.pose.position.y = world_pose.transform.translation.y
                adapt_frame_request.pose.position.z = world_pose.transform.translation.z
                adapt_frame_request.pose.orientation = world_pose.transform.rotation
                adapt_frame_request.set_vision_measured = True
                adapt_frame_request.modify_orientation = modify_orientation

                if not self.pm_robot_utils.client_adapt_frame_absolut.wait_for_service(timeout_sec=1.0):
                    raise PmRobotError("Service 'ModifyPoseAbsolut' not available.")
                
                result_adapt:ami_srv.ModifyPoseAbsolut.Response = self._call_service_with_timeout(
                    self.pm_robot_utils.client_adapt_frame_absolut,
                    adapt_frame_request,
                    'ModifyPoseAbsolut'
                )
                response.success = result_adapt.success

                if not result_adapt.success:
                    raise PmRobotError("Adapting frame pose failed.")
                
                threshold = 2*1e-6
                angle_threshold_deg = 1e-3
                if (abs(result.result_vector.x) < threshold
                    and abs(result.result_vector.y) < threshold
                    and abs(result.result_vector.z) < threshold
                    and abs(result.result_angle) < angle_threshold_deg
                    and request.remeasure_after_correction == True
                    and i != 1):
                    self._logger.info(
                        f"Correction has been smaller than {threshold*1e6} um and "
                        f"{angle_threshold_deg} deg. Remesuring will not be triggered!"
                    )
                    return response
            
        except (RefFrameNotFoundError, PmRobotError) as e:
            self._logger.error(str(e))
            response.success = False
            response.message = str(e)
            return response

        return response

    @staticmethod
    def _get_rotation_axis(vision_response) -> str | None:
        """Determine the rotation axis from a vision response.

        The vision process produces translation deltas along two of the three
        world axes (the axes that were selected via ``axis_suffix_1`` /
        ``axis_suffix_2`` on the detected point/circle). The third axis is
        the rotation axis. This works even if the translation is zero on
        one or both of the populated axes (i.e. a pure rotation correction).

        Returns:
            ``'x'``, ``'y'`` or ``'z'`` for the rotation axis, or ``None`` if
            it could not be determined (no detected point/circle, or the
            configured axes do not uniquely identify a single third axis).
        """
        detected_point = None
        if vision_response is None:
            return None
        results = getattr(vision_response, 'results', None)
        if results is None:
            return None
        if getattr(results, 'points', None):
            detected_point = results.points[0]
        elif getattr(results, 'circles', None):
            detected_point = results.circles[0].center_point
        if detected_point is None:
            return None

        suffix_1 = (detected_point.axis_suffix_1 or '').lower()
        suffix_2 = (detected_point.axis_suffix_2 or '').lower()
        return {
            frozenset(('x', 'y')): 'z',
            frozenset(('x', 'z')): 'y',
            frozenset(('y', 'z')): 'x',
        }.get(frozenset((suffix_1, suffix_2)))

    @staticmethod
    def _apply_world_rotation(orientation: Quaternion, rot_axis: str, angle_deg: float) -> Quaternion:
        """Apply ``angle_deg`` only to the requested Euler rotation component.

        For example, measurements with ``x`` and ``y`` suffixes correct only
        the ``z`` rotation component.
        """
        axis_index = {'x': 0, 'y': 1, 'z': 2}.get(rot_axis)
        if axis_index is None:
            raise ValueError(f"Unknown rotation axis '{rot_axis}'.")

        current_euler = R.from_quat([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]).as_euler('xyz', degrees=True)
        current_euler[axis_index] += angle_deg

        q_new = R.from_euler('xyz', current_euler, degrees=True).as_quat()
        result = Quaternion()
        result.x = float(q_new[0])
        result.y = float(q_new[1])
        result.z = float(q_new[2])
        result.w = float(q_new[3])
        return result

    def object_scene_callback(self, msg: ami_msg.ObjectScene):
        """Handles updates to the object scene and generates process files accordingly."""
        
        # If this is the first received scene, initialize and process it
        if self.pm_robot_utils.object_scene_un.scene is None:
            self._logger.info("First object scene received. Processing all objects and frames...")
        else:
            # If the scene has not changed, do nothing
            if self.pm_robot_utils.object_scene_un.scene == msg:
                self._logger.debug("No changes detected in the object scene.")
                return
            self._logger.info("Processing updated object scene...")

        self.pm_robot_utils.object_scene_un.scene = msg
        
        try:
            # Process objects in the scene (both first-time and updated scenes)
            for obj in msg.objects_in_scene:
                obj: ami_msg.Object
                for frame in obj.ref_frames:
                    frame: ami_msg.RefFrame
                    if not frame.properties.vision_frame_properties.is_vision_frame:
                        continue
                    VisionProcessClass.create_process_file(
                                f"Assembly_Manager/{obj.obj_name}", frame.frame_name, logger=self._logger
                                )

                    
                    VisionProcessClass.create_process_file(
                        f"Assembly_Manager/{obj.obj_name}", frame.frame_name+'_sim', logger=self._logger
                    )                

            # Process reference frames in the scene
            for frame in msg.ref_frames_in_scene:
                if not frame.properties.vision_frame_properties.is_vision_frame:
                    continue
                VisionProcessClass.create_process_file(
                    "Assembly_Manager/Frames", frame.frame_name, logger=self._logger
                )
                VisionProcessClass.create_process_file(
                    "Assembly_Manager/Frames", frame.frame_name+'_sim', logger=self._logger
                )
        except Exception as e:
            self._logger.error(f"Error auto creating vision process files: {str(e)}")


    def check_frame_mes_cam_top(self, request:pm_skill_srv.CheckFrameMeasurable.Request, response:pm_skill_srv.CheckFrameMeasurable.Response):
        CAMERA_TOP_OFFSET = 90*1e-3 # in m, distance from confocal top to laser point

        res = self.pm_robot_utils.check_frame_mes(request = request,
                                                        offset_m=CAMERA_TOP_OFFSET,
                                                         move_client = self.pm_robot_utils.client_move_robot_cam1_to_frame)
        response = res

        return response

    def check_frame_mes_cam_bottom(self, request:pm_skill_srv.CheckFrameMeasurable.Request, response:pm_skill_srv.CheckFrameMeasurable.Response):
        CAMERA_BOTTOM_OFFSET =40*1e-3 # in m, distance from confocal bottom to laser point

        res = self.pm_robot_utils.check_frame_mes_bot(request = request,
                                                        target_tcp_frame = self.pm_robot_utils.TCP_CAMERA_BOTTOM,
                                                        offset_m=CAMERA_BOTTOM_OFFSET)
        response = res

        return response

def main(args=None):
    rclpy.init(args=args)

    node = VisionSkillsNode()

    executor = MultiThreadedExecutor(num_threads=6) 

    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    


if __name__ == '__main__':
    main()
