import rclpy
import copy
import time
import random

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

import pm_skills_interfaces.srv as pm_skill_srv
from pm_moveit_interfaces.srv import MoveToPose,  MoveToFrame, MoveRelative
from example_interfaces.srv import SetBool
import std_msgs.msg as std_msg
import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
import pm_moveit_interfaces.srv as pm_moveit_srv

from pm_msgs.srv import EmptyWithSuccess
from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene

from assembly_scene_publisher.py_modules.tf_functions import get_transform_for_frame_in_world



class PmSkills(Node):
    
    PM_ROBOT_GRIPPER_FRAME = 'PM_Robot_Tool_TCP'
    PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR = 'Gonio_Left_Part'
    PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR = 'Gonio_Right_Part'
    GRIPPING_FRAME_IDENTIFICATORS = ['Grip', 'grip']
    GRIP_RELATIVE_LIFT_DISTANCE = 0.02
    RELEASE_LIFT_DISTNACE = 0.01
    ASSEMBLY_FRAME_INDICATOR = 'assembly_frame_Description'
    TARGET_FRAME_INDICATOR = 'target_frame_Description'
    GRIPPING_OFFSET = 0.0001
    
    def __init__(self) -> None:
        super().__init__('pm_skills')

        self.callback_group_me = MutuallyExclusiveCallbackGroup()
        self.callback_group_re = ReentrantCallbackGroup()

        self.grip_component_srv = self.create_service(pm_skill_srv.GripComponent, "pm_skills/grip_component", self.grip_component_callback,callback_group=self.callback_group_me)
        self.place_component_srv = self.create_service(pm_skill_srv.PlaceComponent, "pm_skills/place_component", self.place_component_callback,callback_group=self.callback_group_me)
        self.assemble_srv  = self.create_service(EmptyWithSuccess, "pm_skills/assemble", self.assemble_callback,callback_group=self.callback_group_me)
        #self.vision_service = self.create_service(ExecuteVision, "pm_skills/execute_vision", self.vision_callback)
        
        self.vacuum_gripper_service = self.create_service(pm_skill_srv.VacuumGripper, "pm_skills/vacuum_gripper", self.grip_vaccum_gripper_callback, callback_group=self.callback_group_me)
        self.move_uv_in_curing_position_service = self.create_service(SetBool, "pm_skills/move_uv_in_curing_position", self.move_uv_in_curing_position_service_callback,callback_group=self.callback_group_me)
    
        self.dispenser_service = self.create_service(pm_skill_srv.DispenseAdhesive, "pm_skills/dispense_adhesive", self.dispenser_callback)
        self.confocal_laser_service = self.create_service(pm_skill_srv.ConfocalLaser, "pm_skills/confocal_laser", self.confocal_laser_callback)
        self.vision_service = self.create_service(pm_skill_srv.ExecuteVision, "pm_skills/execute_vision", self.vision_callback)

        self.attach_component = self.create_client(ami_srv.ChangeParentFrame, '/assembly_manager/change_obj_parent_frame')
        self.set_UV_Front_Joint = self.create_client(SetBool, "pm_pneumatic_dummy/set_UV_LED_Front_Joint")
        self.set_UV_Back_Joint = self.create_client(SetBool, "pm_pneumatic_dummy/set_UV_LED_Back_Joint")

        self.move_robot_tool_client = self.create_client(MoveToFrame, '/pm_moveit_server/move_tool_to_frame')
        self.move_robot_tool_relative = self.create_client(MoveRelative, '/pm_moveit_server/move_tool_relative')
        self.attach_component = self.create_client(ami_srv.ChangeParentFrame, '/assembly_manager/change_obj_parent_frame')
        self.align_gonio_right_client = self.create_client(pm_moveit_srv.AlignGonio, '/pm_moveit_server/align_gonio_right')
        self.align_gonio_left_client = self.create_client(pm_moveit_srv.AlignGonio, '/pm_moveit_server/align_gonio_left')

        self.objcet_scene_subscriber = self.create_subscription(ami_msg.ObjectScene, '/assembly_manager/scene', self.object_scene_callback, 10, callback_group=self.callback_group_re)
        self.simtime_subscriber = self.create_subscription(std_msg.Bool, '/sim_time', self.simtime_callback, 10, callback_group=self.callback_group_re)

        self.object_scene:ami_msg.ObjectScene = None

        self.logger = self.get_logger()
    
    def grip_component_callback(self, request:pm_skill_srv.GripComponent.Request, response:pm_skill_srv.GripComponent.Response):
        """TO DO: Add docstring"""
       
        try:
            if not self.check_object_exists(request.component_name):
                response.success = False
                response.message = f"Object '{request.component_name}' does not exist!"
                self.logger.error(response.message)
                return response
            
            if not self.is_gripper_empthy():
                response.success = False
                response.message = "Gripper is not empty! Can not grip new component!"
                self.logger.error(response.message)
                return response
            
        except ValueError as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)
            return response
        
        gripping_frame = self.get_gripping_frame(request.component_name)

        if gripping_frame is None:
            response.success = False
            response.message = f"No gripping frame found for object '{request.component_name}'"
            self.logger.error(response.message)
            return response
        
        self.logger.info(f"Gripping component '{request.component_name}' at frame '{gripping_frame}'")
        
        if self.is_object_on_gonio_left(request.component_name):
            algin_success = self.align_gonio_left(gripping_frame)

        elif self.is_object_on_gonio_right(request.component_name):
            algin_success = self.align_gonio_right(gripping_frame)
        
        else:
            response.success = False
            response.message = "Object not on gonio!"
            self.logger.error("Object not on gonio!")
            return response
        
        if not algin_success:
            response.success = False
            response.message = f"Failed to align gonio for component '{request.component_name}'"
            self.logger.error(response.message)
            return response
        
        #Offset
        move_tool_to_part_offset_success = self.move_gripper_to_frame(gripping_frame, z_offset=self.RELEASE_LIFT_DISTNACE)

        if not move_tool_to_part_offset_success:
            response.success = False
            response.message = f"Failed to move gripper to part '{request.component_name}'"
            self.logger.error(response.message)
            return response
        
        move_tool_to_part_success = self.move_gripper_to_frame(gripping_frame,z_offset=self.GRIPPING_OFFSET)

        if not move_tool_to_part_success:
            response.success = False
            response.message = f"Failed to move gripper to part '{request.component_name}'"
            self.logger.error(response.message)
            return response
        
        attach_component_success = self.attach_component_to_gripper(request.component_name)

        if not attach_component_success:
            move_relatively_success = self.lift_gripper_relative(self.GRIP_RELATIVE_LIFT_DISTANCE)
            response.success = False
            response.message = f"Failed to attach component '{request.component_name}' to gripper"
            self.logger.error(response.message)
            return response
        
        move_relatively_success = self.lift_gripper_relative(self.GRIP_RELATIVE_LIFT_DISTANCE)

        if not move_relatively_success:
            response.success = False
            response.message = f"Failed to lift gripper after attaching component '{request.component_name}'"
            self.logger.error(response.message)
            return response
        
        self.logger.info(f"Move tool relative success: {move_relatively_success}")

        response.success = True

        response.message = f"Component '{request.component_name}' gripped successfully!"
        self.logger.info(response.message)

        return response

    def place_component_callback(self, request:pm_skill_srv.PlaceComponent.Request, response:pm_skill_srv.PlaceComponent.Response):
        try:
            if self.is_gripper_empthy():
                response.success = False
                response.message = "Gripper is empty! Can not place component!"
                self.logger.error(response.message)
                return response
        except ValueError as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)
            return response

        
        gripped_component = self.get_gripped_component()
        gripped_component_frames = self.get_assembly_target_frame_gripped_component()

        self.logger.warn(f"Component frames '{str(gripped_component_frames)}'!")

        all_involved_frames_tuples = self.get_assembly_and_target_frames()

        target_frame = None

        for component_frame in gripped_component_frames:
            if self.ASSEMBLY_FRAME_INDICATOR in component_frame:
                component_assembly_frame = component_frame
                #strip indicator from frame name
                assembly_description = component_frame.replace(self.ASSEMBLY_FRAME_INDICATOR, '')
                self.logger.info(f"Placing component '{gripped_component}'")

                for frame_tuple in all_involved_frames_tuples:
                    if assembly_description in frame_tuple[1] and frame_tuple[0] != gripped_component:
                        target_frame = frame_tuple[1]
                        break
                
                self.logger.info(f"Placing component '{gripped_component}' at frame '{target_frame}'")


        if target_frame is None:
            response.success = False
            response.message = "No assembly or target frame found for gripped component!"
            self.logger.error(response.message)
            return response

        if target_frame is None:
            response.success = False
            response.message = f"No target frame found for component '{gripped_component}'"
            self.logger.error(response.message)
            return response 
        
        target_component = self.get_component_for_frame_name(target_frame)

        if target_component is None:
            response.success = False
            response.message = f"No component found for target frame '{target_frame}'"
            self.logger.error(response.message)
            return response
        
        # RECALCULATE THE ASSEMBLY TRANSFORMATION
        # INSERT HERE

        # Align gonio to the left or right depending on the target frame
        if self.is_object_on_gonio_left(target_component):
            algin_success = self.align_gonio_left(endeffector_override=target_frame,
                                                    alignment_frame=component_assembly_frame)

        elif self.is_object_on_gonio_right(target_component):
            algin_success = self.align_gonio_right(endeffector_override=target_frame,
                                                    alignment_frame=component_assembly_frame)

        else:
            response.success = False
            response.message = "Target object not on gonio!"
            self.logger.error("Target object not on gonio!")
            return response
        
        if not algin_success:
            response.success = False
            response.message = f"Failed to align gonio for target component '{target_component}'"
            self.logger.error(response.message)
            return response
        
        self.logger.warn("Endeffector override: " + component_assembly_frame)
        # Move component to the target frame with a z offset
        move_component_to_part_offset_success = self.move_gripper_to_frame(target_frame, endeffector_override=component_assembly_frame, z_offset=self.RELEASE_LIFT_DISTNACE)

        if not move_component_to_part_offset_success:
            response.success = False
            response.message = f"Failed to move gripper to target part '{target_component}'"
            self.logger.error(response.message)
            return response

        # Move component to the target frame
        move_component_to_part_success = self.move_gripper_to_frame(target_frame, endeffector_override=component_assembly_frame)

        if not move_component_to_part_success:
            response.success = False
            response.message = f"Failed to move gripper to target part '{target_component}'"
            self.logger.error(response.message)
            return response
        
        # release component
        attach_component_success = self.attach_component_to_component(gripped_component, target_component)

        if not attach_component_success:
            move_relatively_success = self.lift_gripper_relative(self.RELEASE_LIFT_DISTNACE)
            response.success = False
            response.message = f"Failed to attach component '{gripped_component}' to component '{target_component}'"
            self.logger.error(response.message)
            return response

        move_relatively_success = self.lift_gripper_relative(self.GRIP_RELATIVE_LIFT_DISTANCE)

        if not move_relatively_success:
            response.success = False
            response.message = f"Failed to lift gripper after placing component '{gripped_component}'"
            self.logger.error(response.message)
            return response
        
        response.success = True
        response.message = f"Component '{gripped_component}' placed successfully!"
        return response
        
    def assemble_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        global_stationary_component = self.get_global_statonary_component()
        assemble_list = self.get_components_to_assemble()
        first_component = self.find_matches_for_component(global_stationary_component, only_unassembled=False)

        self.assembly_loop(first_component)
        
        response.success = True

        return response

    def move_uv_in_curing_position_service_callback(self, request:SetBool.Request, response:SetBool.Response):
        """Moves both UV LEDs in curing position"""
        
        response.success = self.move_uv_in_curing_position(request)
        response.message = "UV LEDs moved in curing position!" if response.success else "Failed to move UV LEDs in curing position!"

        return response

    def grip_vaccum_gripper_callback(self, request:pm_skill_srv.GripComponent.Request, response:pm_skill_srv.GripComponent.Response):
        """TO DO: Add docstring"""

        try:
            if not self.check_object_exists(request.component_name):
                response.success = False
                response.message = f"Object '{request.component_name}' does not exist!"
                self.logger.error(response.message)
                return response
            
            if not self.is_gripper_empthy():
                response.success = False
                response.message = "Gripper is not empty! Can not grip new component!"
                self.logger.error(response.message)
                return response
            
        except ValueError as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)
            return response
        
        response.success = self.attach_component_to_gripper(request.component_name)
        response.message = f"Component '{request.component_name}' gripped!" if response.success else f"Failed to grip component '{request.component_name}'!"
        
        return response
    
    def dispenser_callback(self, request, response):
        """TO DO: Add docstring"""

        self.logger.info(f"Starting {request.dispenser}")

        # Simulating dispension time by adding delay
        wait_duration = request.dispense_time
        if wait_duration > 0.0:
            time.sleep(wait_duration)
            response.success = True
        else:
            self.logger.warn("Invaild dispense Duration!")
            response.success = False
            
        self.logger.info(f"Done {request.dispenser}")
        return response

    def confocal_laser_callback(self, request, response):
        """TO DO: Add docstring"""

        self.logger.info(f"Starting laser: {request.laser}")

        # Generating random number for meassurement    
        result = ((random.randrange(0, 6)) / 10.0) - 0.3

        response.result = str(f"{result} mm.")
        response.success = True

        return response
    
    def vision_callback(self, request, response):
        self.logger.info(f'Loading {request.process_filename} as process file.')
        self.logger.info(f'Starting process {request.process_uid}')
        self.logger.info(f"Loading {request.camera_config_filename} as camera configuration file.")

        if request.image_display_time > 0.0:
            self.logger.info('Image aquisiton is running')
            time.sleep(request.image_display_time)
            response.success = True
        else: 
            self.logger.warn('Invalid image display time!')
            response.success = False

        if request.run_cross_validation:
            self.logger.info("Running cross validtation.")

        response.results_dict = "Result dict"
        self.logger.info(f'Saving results ...')
        response.results_path = "Results path."
        response.points = [random.random()*100 for _ in range(4)]
        response.process_uid = str(random.randint(1000,50000))

        return response
            
    def move_uv_in_curing_position(self, request:SetBool.Request)->bool:
        
        if not self.set_UV_Front_Joint.wait_for_service(timeout_sec=2.0) and not self.set_UV_Back_Joint.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Services not available!")
            return False

        req_front: SetBool.Request = copy.deepcopy(request)
        req_back: SetBool.Request = copy.deepcopy(request)

        # inverse movement direction for front module
        req_front.data = not req_front.data

        response_front:SetBool.Response = self.set_UV_Front_Joint.call(req_front)
        response_back:SetBool.Response = self.set_UV_Back_Joint.call(req_back)
        
        self.get_logger().info(f"Service call result: success={response_front.success}, msg='{response_front.message}'")
        self.get_logger().info(f"Service call result: success={response_back.success}, msg='{response_back.message}'")

        if response_front.success and response_back.success:
            return True
        else:
            return False

    def assembly_loop(self, list_of_components:list[str]):
        
        for component in list_of_components:
            self.logger.error(f"ComponentTTTTT: {component}")
            if not self.is_component_assembled(component):

                response = self.grip_component_callback(pm_skill_srv.GripComponent.Request(component_name=component), pm_skill_srv.GripComponent.Response())
                if not response.success:
                    self.logger.error(f"Failed to grip component '{component}'")
                    return False
                
                response = self.place_component_callback(pm_skill_srv.PlaceComponent.Request(), pm_skill_srv.PlaceComponent.Response())
                if not response.success:
                    self.logger.error(f"Failed to place component '{component}'")
                    return False
                
            list_of_component= self.find_matches_for_component(component,only_unassembled=False)
            self.logger.warn(f"List of components: {list_of_component}")
            self.assembly_loop(list_of_component)

        return True

    def simtime_callback(self, msg:std_msg.Bool):
        if msg.data:
            self.logger.info("Simulation time is active!")
        else:
            self.logger.info("Simulation time is inactive!")

    def lift_gripper_relative(self, distance:float)-> bool:
        call_async = False

        if not self.move_robot_tool_relative.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_tool_relative' not available")
            return False
        
        req = pm_moveit_srv.MoveRelative.Request()
        req.translation.z = distance
        req.execute_movement = True

        if call_async:
            future = self.move_robot_tool_relative.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:pm_moveit_srv.MoveRelative.Response = self.move_robot_tool_relative.call(req)
            return response.success
        
    def move_gripper_to_frame(self, frame_name:str, endeffector_override = None, z_offset=None)-> bool:
        call_async = False

        if not self.move_robot_tool_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_tool_to_frame' not available")
            return False
        
        req = pm_moveit_srv.MoveToFrame.Request()
        req.target_frame = frame_name
        req.execute_movement = True
        req.translation.z = 0.0000001

        if endeffector_override is not None:
            req.endeffector_frame_override = endeffector_override

        if z_offset is not None:
            req.translation.z = z_offset

        if call_async:
            future = self.move_robot_tool_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:pm_moveit_srv.MoveToFrame.Response = self.move_robot_tool_client.call(req)
            return response.success

    def move_gripper_to_relative(self, frame_name:str)-> bool:
        call_async = False

        if not self.move_robot_tool_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_tool_to_frame' not available")
            return False
        
        req = pm_moveit_srv.MoveToFrame.Request()
        req.target_frame = frame_name
        req.execute_movement = True
        req.translation.z = 0.0000001

        if call_async:
            future = self.move_robot_tool_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:pm_moveit_srv.MoveToFrame.Response = self.move_robot_tool_client.call(req)
            return response.success
        
    def attach_component_to_gripper(self, component_name:str)-> bool:
        call_async = False

        if not self.attach_component.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/assembly_manager/change_obj_parent_frame' not available")
            return False
        
        req = ami_srv.ChangeParentFrame.Request()
        req.obj_name = component_name
        req.new_parent_frame = self.PM_ROBOT_GRIPPER_FRAME

        if call_async:
            future = self.attach_component.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:ami_srv.ChangeParentFrame.Response = self.attach_component.call(req)
            return response.success
        
    def attach_component_to_component(self, component_name:str, parent_component_name:str)-> bool:
        call_async = False

        if not self.attach_component.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/assembly_manager/change_obj_parent_frame' not available")
            return False
        
        req = ami_srv.ChangeParentFrame.Request()
        req.obj_name = component_name
        req.new_parent_frame = parent_component_name

        if call_async:
            future = self.attach_component.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:ami_srv.ChangeParentFrame.Response = self.attach_component.call(req)
            return response.success

    def align_gonio_right(self, endeffector_override:str, alignment_frame:str = PM_ROBOT_GRIPPER_FRAME)-> bool:
        call_async = False

        if not self.align_gonio_right_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/align_gonio_right' not available")
            return False
        
        req = pm_moveit_srv.AlignGonio.Request()
        req.target_frame = alignment_frame
        req.execute_movement = True
        req.endeffector_frame_override = endeffector_override

        if call_async:
            future = self.align_gonio_right_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:pm_moveit_srv.AlignGonio.Response = self.align_gonio_right_client.call(req)
            return response.success
        
    def align_gonio_left(self, endeffector_override:str, alignment_frame:str = PM_ROBOT_GRIPPER_FRAME)-> bool:
        call_async = False

        if not self.align_gonio_left_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/align_gonio_left' not available")
            return False
        
        req = pm_moveit_srv.AlignGonio.Request()
        req.target_frame = alignment_frame
        req.execute_movement = True
        req.endeffector_frame_override = endeffector_override

        if call_async:
            future = self.align_gonio_left_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:pm_moveit_srv.AlignGonio.Response = self.align_gonio_left_client.call(req)
            return response.success
        
    def object_scene_callback(self, msg:ami_msg.ObjectScene)-> str:
        self.object_scene = msg
                    
    def get_gripping_frame(self, object_name:str)-> str:
        grip_frames = []
        for obj in self.object_scene.objects_in_scene:
            obj:ami_msg.Object
            if obj.obj_name == object_name:
                for frame in obj.ref_frames:
                    frame:ami_msg.RefFrame
                    for identificador in self.GRIPPING_FRAME_IDENTIFICATORS:
                        # check if string is part of string
                        if identificador in frame.frame_name:
                            grip_frames.append(frame.frame_name)

        if len(grip_frames) == 0:
            self.logger.error(f"No gripping frame found for object '{object_name}'")
            return None
        
        if len(grip_frames) > 1:
            self.logger.error(f"Multiple potential gripping frames found for object '{object_name}'. Identificators: {self.GRIPPING_FRAME_IDENTIFICATORS}")
            return None
        else:
            return grip_frames[0]
        
    def get_parent_of_object(self, object_name:str):
        self.wait_for_initial_scene_update()
        for obj in self.object_scene.objects_in_scene:
            obj:ami_msg.Object
            if obj.obj_name == object_name:
                return obj.parent_frame
        return None

    def is_object_on_gonio_left(self, object_name:str)-> bool:
        parent_frame = self.get_parent_of_object(object_name)

        if self.PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR in parent_frame:
            return True
        
        parent_object_1 = self.get_parent_of_object(parent_frame)

        if parent_object_1 is None:
            return False
        
        if self.PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR in parent_object_1:
            return True
        
        parent_object_2 = self.get_parent_of_object(parent_object_1)

        if parent_object_2 is None:
            return False
        
        if self.PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR in parent_object_2:
            return True
        
        return False

    def is_object_on_gonio_right(self, object_name:str)-> bool:
        parent_frame = self.get_parent_of_object(object_name)
        if self.PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR in parent_frame:
            return True
        
        parent_object_1 = self.get_parent_of_object(parent_frame)

        if parent_object_1 is None:
            return False
        
        if self.PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR in parent_object_1:
            return True
        
        parent_object_2 = self.get_parent_of_object(parent_object_1)

        if parent_object_2 is None:
            return False
        
        if self.PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR in parent_object_2:
            return True
        
        return False

    def is_gripper_empthy(self)-> bool:
        self.wait_for_initial_scene_update()

        if self.object_scene is None:
            raise ValueError("Object scene not available!")
        

        for obj in self.object_scene.objects_in_scene:
            obj:ami_msg.Object
            if obj.parent_frame == self.PM_ROBOT_GRIPPER_FRAME:
                return False
        return True

    def check_object_exists(self, object_name:str)-> bool:
        self.wait_for_initial_scene_update()
        if self.object_scene is None:
            return False
        
        for obj in self.object_scene.objects_in_scene:
            obj:ami_msg.Object
            if obj.obj_name == object_name:
                return True
        return False

    def get_gripped_component(self)-> str:
        self.wait_for_initial_scene_update()
        for obj in self.object_scene.objects_in_scene:
            obj:ami_msg.Object
            if obj.parent_frame == self.PM_ROBOT_GRIPPER_FRAME:
                return obj.obj_name
        return None

    def get_assembly_and_target_frames(self)-> list[tuple[str,str]]:
        frames = []
        self.wait_for_initial_scene_update()
        for obj in self.object_scene.objects_in_scene:
            obj:ami_msg.Object
            for frame in obj.ref_frames:
                frame:ami_msg.RefFrame
                if self.ASSEMBLY_FRAME_INDICATOR in frame.frame_name:
                    frames.append((obj.obj_name,frame.frame_name))
                if self.TARGET_FRAME_INDICATOR in frame.frame_name:
                    frames.append((obj.obj_name,frame.frame_name))
        return frames

    def get_assembly_target_frame_gripped_component(self)->list[str]:
        """
        Get the assembly and target frames for the gripped component. 

        Returns:
            list[str]: List of assembly and target frames for the gripped component.
        """
        gripped_component = self.get_gripped_component()
        
        frames = []

        if gripped_component is None:
            return None
        
        frames_tuple = self.get_assembly_and_target_frames()
        for frame_tuple in frames_tuple:
            if frame_tuple[0] == gripped_component:
                frames.append(frame_tuple[1])
        
        return frames

    def get_component_for_frame_name(self, frame_name:str)-> str:
        self.wait_for_initial_scene_update()
        for obj in self.object_scene.objects_in_scene:
            obj:ami_msg.Object
            for frame in obj.ref_frames:
                frame:ami_msg.RefFrame
                if frame.frame_name == frame_name:
                    return obj.obj_name
        return None



    def get_list_of_components(self)-> list[str]:
        self.wait_for_initial_scene_update()
        components = []
        for obj in self.object_scene.objects_in_scene:
            obj:ami_msg.Object
            components.append(obj.obj_name)
        return components

    def get_global_statonary_component(self)-> str:
        """
        Returns the name of the component that is not moved among all of the components.
        """

        self.wait_for_initial_scene_update()

        components = self.get_list_of_components()
        
        for component in components:
            if self.is_component_stationary(component):
                return component

    def is_component_stationary(self, component_name:str)-> bool:

        self.wait_for_initial_scene_update()

        is_stationary = True
        for instruction in self.object_scene.assembly_instructions:
            instruction:ami_msg.AssemblyInstruction
            if component_name == instruction.component_1 and instruction.component_1_is_moving_part:
                is_stationary = False

            if component_name == instruction.component_2 and not instruction.component_1_is_moving_part:
                is_stationary = False

        return is_stationary

    def is_component_assembled(self, component_name:str)-> bool:
        self.wait_for_initial_scene_update()

        is_assembled = False

        for instruction in self.object_scene.assembly_instructions:
            instruction:ami_msg.AssemblyInstruction
            if component_name == instruction.component_1 and instruction.component_2 == self.get_parent_of_object(component_name):
                return True

            if component_name == instruction.component_2 and instruction.component_1 == self.get_parent_of_object(component_name):
                return True
        
        return False

    def get_components_to_assemble(self)-> list[str]:
        components_to_assembly = []

        self.wait_for_initial_scene_update()

        for instruction in self.object_scene.assembly_instructions:
            instruction:ami_msg.AssemblyInstruction
            if instruction.component_1_is_moving_part:
                moving_object = instruction.component_1
                stationary_object = instruction.component_2
            else:
                moving_object = instruction.component_2
                stationary_object = instruction.component_1
            
            if stationary_object != self.get_parent_of_object(moving_object):
                components_to_assembly.append(moving_object)
        
        return components_to_assembly

    def find_matches_for_component(self, component_name:str, only_unassembled = True)-> list[str]:
        """
        The component to find the matches for should be the stationary component.
        """
        self.wait_for_initial_scene_update()

        matches = []
        for instruction in self.object_scene.assembly_instructions:
            instruction:ami_msg.AssemblyInstruction
            self.logger.warn(f"Instruction: {instruction.component_1} -> {instruction.component_2}")

            test= self.is_component_assembled(instruction.component_2)
            self.logger.warn(f"Is component {instruction.component_2} assembled: {test}")

            if component_name == instruction.component_1:
                if ((not only_unassembled or self.is_component_assembled(instruction.component_2)) and 
                    not instruction.component_1_is_moving_part):
                    #not instruction.component_2 == self.get_global_statonary_component()):

                    matches.append(instruction.component_2)

            if component_name == instruction.component_2:
                if ((not only_unassembled or self.is_component_assembled(instruction.component_1)) and 
                    instruction.component_1_is_moving_part):
                    #not instruction.component_1 == self.get_global_statonary_component()):

                    matches.append(instruction.component_1)
        return matches

    def wait_for_initial_scene_update(self):
        while self.object_scene is None:
            self.logger.warn("Waiting for object scene to be updated...")
            time.sleep(0.1)
            

def main(args=None):
    rclpy.init(args=args)

    node = PmSkills()

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
