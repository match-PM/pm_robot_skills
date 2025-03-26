import sys
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from pm_skills_interfaces.srv import MeasureFrame, CorrectFrame
from pm_moveit_interfaces.srv import MoveToPose,  MoveToFrame
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from pm_vision_interfaces.srv import ExecuteVision
import pm_vision_interfaces.msg as vision_msg
from geometry_msgs.msg import Vector3, TransformStamped, Pose, PoseStamped, Quaternion

from pm_vision_manager.va_py_modules.vision_assistant_class import VisionProcessClass

import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg

from assembly_scene_publisher.py_modules.scene_functions import is_frame_from_scene
from assembly_scene_publisher.py_modules.tf_functions import get_transform_for_frame_in_world
import time

class VisionSkillsNode(Node):

    def __init__(self):
        super().__init__('pm_skills')
        self._logger = self.get_logger()

        self.callback_group = MutuallyExclusiveCallbackGroup()

        self._logger.info(f"Node '{self.get_name()}' started...")

        self.srv = self.create_service(MeasureFrame, self.get_name()+'/vision_measure_frame', self.measure_frame, callback_group=self.callback_group)
        self.srv = self.create_service(CorrectFrame, self.get_name()+'/vision_correct_frame', self.correct_frame, callback_group=self.callback_group)

        #self.srv = self.create_service(MeasureFrame, self.get_name()+'/vision_measure_frame', self.measure_frame, callback_group=self.callback_group)
        #self.srv = self.create_service(CorrectFrame, self.get_name()+'/vision_correct_frame', self.correct_frame, callback_group=self.callback_group)

        self.tf_buffer = Buffer()
        
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.client_execute_vision = self.create_client(ExecuteVision, '/pm_vision_manager/ExecuteVision')
        self.move_robot_cam_skill = self.create_client(MoveToFrame, '/pm_moveit_server/move_cam1_to_frame')
        self.move_robot_tool_skill = self.create_client(MoveToFrame, '/pm_moveit_server/move_tool_to_frame')
        self.adapt_frame_skill = self.create_client(ami_srv.ModifyPoseAbsolut, '/assembly_manager/modify_frame_absolut')

        self.objcet_scene_subscriber = self.create_subscription(ami_msg.ObjectScene, '/assembly_manager/scene', self.object_scene_callback, 10)
        self.object_scene:ami_msg.ObjectScene = None

        self._logger.info(f"Gazebo running: {self.is_gazebo_running()}")
        self._logger.info(f"Unity running: {self.is_unity_running()}")

    def measure_frame(self, request:MeasureFrame.Request, response:MeasureFrame.Response):

        vision_request = ExecuteVision.Request()
        vision_request.process_filename = request.vision_process_file_name
        vision_request.process_uid = request.frame_name
        vision_request.image_display_time = 5

        if request.frame_name == '':
            self._logger.error("Frame name is empty...")
            response.success= False
            return response
        
        while not self.client_execute_vision.wait_for_service(timeout_sec=1.0):
            self._logger.error("Service 'ExecuteVision' not available...")
            response.success= False
            return response
        
        self._logger.info("Service 'ExecuteVision' is available...")

        # try to move the frame to the bottom camera
        move_request = MoveToFrame.Request()
        move_request.execute_movement = True
        move_request.endeffector_frame_override = request.frame_name
        move_request.target_frame = 'Camera_Station_TCP'

        self._logger.error(f"Moving tool to frame: {request.frame_name}... executing movement: {move_request.execute_movement} edgeffector frame override: {move_request.endeffector_frame_override} target frame: {move_request.target_frame}")
        
        # select the bottom camera
        if not self.is_unity_running():
            vision_request.camera_config_filename = 'pm_robot_basler_bottom_cam_2.yaml'
        else:
            self._logger.info("Unity is running. Using simulated camera...")
            vision_request.camera_config_filename = 'pm_robot_bottom_cam_2_Unity.yaml'

        while not self.move_robot_cam_skill.wait_for_service(timeout_sec=1.0):
            self._logger.error("Service 'MoveCamToFrame' not available...")
            response.success= False
            return response
        
        result_tool_to_bottom_cam:MoveToFrame.Response = self.move_robot_cam_skill.call(move_request)
        
        self._logger.warn(f"MOOOOVOEE SUCCCEESS {result_tool_to_bottom_cam.success}")

        if not result_tool_to_bottom_cam.success:
            self._logger.warn("Can not move frame to the bottom cam. Trying to reach frame with the top camera...")

            # selct the top camera
            if not self.is_unity_running():
                vision_request.camera_config_filename = 'pm_robot_basler_top_cam_1.yaml'
            else:
                self._logger.info("Unity is running. Using simulated camera...")
                vision_request.camera_config_filename = 'pm_robot_top_cam_1_Unity.yaml'

            move_request = MoveToFrame.Request()
            move_request.execute_movement = True
            move_request.target_frame = request.frame_name

            while not self.move_robot_cam_skill.wait_for_service(timeout_sec=1.0):
                self._logger.error("Service 'MoveCamToFrame' not available...")
                response.success= False
                return response
            
            result_move_tool:MoveToFrame.Response = self.move_robot_cam_skill.call(move_request)

            # Cannot move to the frame
            if not result_move_tool.success:
                self._logger.error("Error moving tool to frame...")
                response.success= False
                return response

        if not self.is_gazebo_running():
            result:ExecuteVision.Response = self.client_execute_vision.call(vision_request)

            detected_circles = result.vision_response.results.circles

            self._logger.warn(f"Result: {str(result.vision_response.results)}")

            if len(detected_circles) == 0:
                self._logger.warn("No circles detected...")
            
            else:
                result_vector = Vector3()
                detected_point:vision_msg.VisionCircle = detected_circles[0]
                if detected_point.axis_suffix_1 == 'x' or detected_point.axis_suffix_1 == 'X':
                    result_vector.x = detected_point.axis_value_1
                    
                if detected_point.axis_suffix_1 == 'y' or detected_point.axis_suffix_1 == 'Y':
                    result_vector.y = detected_point.axis_value_1

                if detected_point.axis_suffix_1 == 'z' or detected_point.axis_suffix_1 == 'Z':
                    result_vector.z = detected_point.axis_value_1

                if detected_point.axis_suffix_2 == 'x' or detected_point.axis_suffix_2 == 'X':
                    result_vector.x = detected_point.axis_value_2

                if detected_point.axis_suffix_2 == 'y' or detected_point.axis_suffix_2 == 'Y':
                    result_vector.y = detected_point.axis_value_2

                if detected_point.axis_suffix_2 == 'z' or detected_point.axis_suffix_2 == 'Z':
                    result_vector.z = detected_point.axis_value_2

            detected_points = result.vision_response.results.points

            if len(detected_points) == 0:
                self._logger.warn("No points detected...")
                response.success= False
                return response
            else:
                self._logger.warn(f"Detected points: {detected_points}")
                detected_point:vision_msg.VisionPoint = detected_points[0]
                result_vector = Vector3()
                multiplier = 0.000001 # Convert to m  
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
            
            result_vector.x = -result_vector.x
            result_vector.y = -result_vector.y
            result_vector.z = -result_vector.z
            response.result_vector = result_vector     
            response.success = result.success

            self._logger.info(f"Vision process executed...{result}")
        else:
            # Gazebo is running
            response.success = True
       
        return response
    
    def correct_frame(self, request:CorrectFrame.Request, response:CorrectFrame.Response):
        self.wait_for_initial_scene_update()

        _obj_name, _frame_name =  is_frame_from_scene(self.object_scene, request.frame_name)

        self._logger.warn(f"Correcting frame: {request.frame_name}...")
        self._logger.warn(f"Object name: {_obj_name}")
        self._logger.warn(f"Frame name: {_frame_name}")

        if _frame_name is not None:
            frame_from_scene = True
        else:
            frame_from_scene = False

        measure_frame_request = MeasureFrame.Request()
        measure_frame_request.frame_name =request.frame_name

        if self.is_unity_running():
            extention = '_sim'
        else:
            extention = ''

        if frame_from_scene:
            if _obj_name is None:
                measure_frame_request.vision_process_file_name = f"Assembly_Manager/Frames/{_frame_name}"
            else:
                measure_frame_request.vision_process_file_name = f"Assembly_Manager/{_obj_name}/{_frame_name}{extention}.json"
        else:
            measure_frame_request.vision_process_file_name = f"PM_Robot_Calibration/TO_BE_DEFINED"

        response_em = MeasureFrame.Response()

        self._logger.warn(f"Requesting measure frame for frame: {request.frame_name}...using process file: {measure_frame_request.vision_process_file_name}")
        result:MeasureFrame.Response = self.measure_frame(measure_frame_request, response_em)

        if not result.success:
            response.success = False
            return response
        
        world_pose:TransformStamped = get_transform_for_frame_in_world(request.frame_name, self.tf_buffer, self._logger)

        world_pose.transform.translation.x += result.result_vector.x
        world_pose.transform.translation.y += result.result_vector.y
        world_pose.transform.translation.z += result.result_vector.z

        adapt_frame_request = ami_srv.ModifyPoseAbsolut.Request()
        adapt_frame_request.frame_name = request.frame_name
        adapt_frame_request.pose.position.x = world_pose.transform.translation.x
        adapt_frame_request.pose.position.y = world_pose.transform.translation.y
        adapt_frame_request.pose.position.z = world_pose.transform.translation.z
        adapt_frame_request.pose.orientation = world_pose.transform.rotation

        if frame_from_scene:
            while not self.adapt_frame_skill.wait_for_service(timeout_sec=1.0):
                self._logger.error("Service 'ModifyPoseAbsolut' not available. Assembly manager started?...")
                response.success= False
                return response
            
            result_adapt:ami_srv.ModifyPoseAbsolut.Response = self.adapt_frame_skill.call(adapt_frame_request)
        else:
            result_adapt = ami_srv.ModifyPoseAbsolut.Response()

        response.success = result_adapt.success
        response.correction_values = result.result_vector
        
        return response
    
    def object_scene_callback(self, msg: ami_msg.ObjectScene):
        """Handles updates to the object scene and generates process files accordingly."""
        
        # If this is the first received scene, initialize and process it
        if self.object_scene is None:
            self._logger.info("First object scene received. Processing all objects and frames...")
        else:
            # If the scene has not changed, do nothing
            if self.object_scene == msg:
                self._logger.debug("No changes detected in the object scene.")
                return
            self._logger.info("Processing updated object scene...")

        # Process objects in the scene (both first-time and updated scenes)
        for obj in msg.objects_in_scene:
            obj: ami_msg.Object
            for frame in obj.ref_frames:
                frame: ami_msg.RefFrame
                if not ('Vision' in frame.frame_name or 'vision' in frame.frame_name):
                    continue
                VisionProcessClass.create_process_file(
                    f"Assembly_Manager/{obj.obj_name}", frame.frame_name, logger=self._logger
                )
                VisionProcessClass.create_process_file(
                    f"Assembly_Manager/{obj.obj_name}", frame.frame_name+'_sim', logger=self._logger
                )                

        # Process reference frames in the scene
        for frame in msg.ref_frames_in_scene:
            if not ('Vision' in frame.frame_name or 'vision' in frame.frame_name):
                continue
            VisionProcessClass.create_process_file(
                "Assembly_Manager/Frames", frame.frame_name, logger=self._logger
            )
            VisionProcessClass.create_process_file(
                "Assembly_Manager/Frames", frame.frame_name+'_sim', logger=self._logger
            )

        # Update the stored object scene
        self.object_scene = msg
                           
    def is_gazebo_running(self)->bool:
        """Check if the Gazebo node is active."""
        node_names = self.get_node_names()
        if 'gazebo' in node_names:
            return True
        return False
    
    def is_unity_running(self)->bool:
        """Check if the Unity node is active."""
        node_names = self.get_node_names()
        if 'ROS2UnityCam1Publisher' in node_names:
            return True
        return False
    
    def wait_for_initial_scene_update(self):
        while self.object_scene is None:
            self._logger.warn("Waiting for object scene to be updated...")
            time.sleep(0.1)

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