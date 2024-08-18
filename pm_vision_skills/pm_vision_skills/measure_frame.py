import sys
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from pm_robot_skills_interfaces.srv import MeasureFrame, CorrectFrame
from pm_moveit_interfaces.srv import MoveToPose,  MoveToFrame
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from pm_vision_interfaces.srv import ExecuteVision
import pm_vision_interfaces.msg as vision_msg
from geometry_msgs.msg import Vector3, TransformStamped, Pose, PoseStamped, Quaternion

from pm_vision_manager.va_py_modules.vision_assistant_class import VisionProcessClass

import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg

from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene

from assembly_scene_publisher.py_modules.tf_functions import get_transform_for_frame_in_world

class MeasureFrameNode(Node):

    def __init__(self):
        super().__init__('pm_vision_skills')
        self._logger = self.get_logger()

        self.callback_group = MutuallyExclusiveCallbackGroup()

        self._logger.info("Initializing MeasureFrameNode...")

        self.srv = self.create_service(MeasureFrame, self.get_name()+'/measure_frame', self.measure_frame, callback_group=self.callback_group)
        self.srv = self.create_service(CorrectFrame, self.get_name()+'/correct_frame', self.correct_frame, callback_group=self.callback_group)
        
        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.client_execute_vision = self.create_client(ExecuteVision, '/pm_vision_manager/ExecuteVision')
        self.move_robot_cam_skill = self.create_client(MoveToFrame, '/pm_moveit_server/move_cam1_to_frame')
        self.move_robot_tool_skill = self.create_client(MoveToFrame, '/pm_moveit_server/move_tool_to_frame')
        self.adapt_frame_skill = self.create_client(ami_srv.ModifyPoseAbsolut, '/assembly_manager/modify_frame_absolut')

        self.objcet_scene_subscriber = self.create_subscription(ami_msg.ObjectScene, '/assembly_manager/scene', self.object_scene_callback, 10)
        self.object_scene:ami_msg.ObjectScene = None

    def measure_frame(self, request:MeasureFrame.Request, response:MeasureFrame.Response):

        vision_request = ExecuteVision.Request()
        vision_request.process_filename = request.vision_process_file_name
        vision_request.process_uid = request.frame_name
        vision_request.image_display_time = 1


        if request.frame_name == '':
            self._logger.error("Frame name is empty...")
            response.success= False
            return response
        
        while not self.client_execute_vision.wait_for_service(timeout_sec=1.0):
            self._logger.error("Service 'ExecuteVision' not available...")
            response.success= False
            return response
        
        self._logger.info("Service 'ExecuteVision' is available...")

        move_request = MoveToFrame.Request()
        move_request.execute_movement = True
        move_request.target_frame = request.frame_name

        vision_request.camera_config_filename = 'webcam.yaml'

        #vision_request.camera_config_filename = 'pm_robot_basler_top_cam_1.yaml'

        result_move_cam:MoveToFrame.Response = self.move_robot_cam_skill.call(move_request)

        while not self.move_robot_cam_skill.wait_for_service(timeout_sec=1.0):
            self._logger.error("Service 'MoveCamToFrame' not available...")
            response.success= False
            return response
        
        if not result_move_cam.success:
            self._logger.error("Error moving camera to frame...")
            move_request.endeffector_frame_override = request.frame_name
            move_request.target_frame = 'Camera_Station_TCP'

            #vision_request.camera_config_filename = 'pm_robot_basler_bottom_cam_2.yaml'

            while not self.move_robot_cam_skill.wait_for_service(timeout_sec=1.0):
                self._logger.error("Service 'MoveToolToFrame' not available...")
                response.success= False
                return response
            
            result_move_tool:MoveToFrame.Response = self.move_robot_cam_skill.call(move_request)

            # Cannot move to the frame
            if not result_move_tool.success:
                self._logger.error("Error moving tool to frame...")
                response.success= False
                return response

        
        result:ExecuteVision.Response = self.client_execute_vision.call(vision_request)

        detected_circles = result.vision_response.results.circles

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
            detected_point:vision_msg.VisionPoint = detected_points[0]
            result_vector = Vector3()

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

        response.result_vector = result_vector    
        response.success= result.success

        self._logger.info(f"Vision process executed...{result}")
       
        return response
    
    def correct_frame(self, request:CorrectFrame.Request, response:CorrectFrame.Response):
        
        _obj_name, _frame_name =  AssemblyManagerScene.is_frame_from_scene(self.object_scene, request.frame_name)

        if not _obj_name is not None and not _frame_name is not None:
            frame_from_scene = True
        else:
            frame_from_scene = False

        measure_frame_request = MeasureFrame.Request()
        measure_frame_request.frame_name =request.frame_name

        if frame_from_scene:
            if _obj_name is None:
                measure_frame_request.vision_process_file_name = f"Assembly_Manager/Frames/{_frame_name}"
            else:
                measure_frame_request.vision_process_file_name = f"Assembly_Manager/{_obj_name}/{_frame_name}"
        else:
            measure_frame_request.vision_process_file_name = f"PM_Robot_Calibration/TO_BE_DEFINED"

        response_em = MeasureFrame.Response()

        result:MeasureFrame.Response = self.measure_frame(measure_frame_request, response_em)

        #self._logger.warn(f"Measure frame result: !!!!!!!!!!!!!!!!!!!")

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
            result_adapt:ami_srv.ModifyPoseAbsolut.Response = self.adapt_frame_skill.call(adapt_frame_request)
        else:
            result_adapt = ami_srv.ModifyPoseAbsolut.Response()

        response.success = result_adapt.success

        return response
    
    def object_scene_callback(self, msg:ami_msg.ObjectScene):

        if not self.object_scene == msg:
            
            if self.object_scene is None:
                self.object_scene = msg
                return
            
            for obj in self.object_scene.objects_in_scene:
                obj:ami_msg.Object
                for frame in obj.ref_frames:
                    frame:ami_msg.RefFrame
                    VisionProcessClass.create_process_file(f"Assembly_Manager/{obj.obj_name}", frame.frame_name, logger=self._logger)
                    self._logger.debug(f"Creating process file for frame: {frame.frame_name} in object: {obj.obj_name}")
            for frame in self.object_scene.ref_frames_in_scene:
                frame:ami_msg.RefFrame
                VisionProcessClass.create_process_file("Assembly_Manager/Frames", frame.frame_name, logger=self._logger)
                self._logger.debug(f"Creating process file for frame: {frame.frame_name}")

            self.object_scene = msg
                    
                    
                    

        

def main(args=None):
    rclpy.init(args=args)

    node = MeasureFrameNode()

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