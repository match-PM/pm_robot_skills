import random
import time

import rclpy
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R

import assembly_manager_interfaces.msg as ami_msg
import assembly_manager_interfaces.srv as ami_srv
import pm_moveit_interfaces.srv as pm_moveit_srv
import pm_msgs.srv as pm_msg_srv
import pm_skills_interfaces.srv as pm_skill_srv
import std_msgs.msg as std_msg
from assembly_scene_publisher.py_modules.geometry_functions import quaternion_multiply
from assembly_scene_publisher.py_modules.scene_errors import (
    AssemblyFrameNotFoundError,
    AssemblyInstructionNotFoundError,
    ComponentNotFoundError,
    GrippingFrameNotFoundError,
    RefFrameNotFoundError,
    TargetFrameNotFoundError,
)
from pm_msgs.srv import EmptyWithSuccess
from pm_robot_modules.submodules.pm_robot_config import ParallelGripperConfig, VacuumGripperConfig
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_skills.py_modules.pm_skill_domain import PmSkillDomain


class PmGripperSkills(PmSkillDomain):
    def force_grip_component_callback(self, request:pm_skill_srv.GripComponent.Request, response:pm_skill_srv.GripComponent.Response):
        """TO DO: Add docstring"""

        self.pm_robot_utils.wait_for_initial_scene_update()

        try:
            should_move_up_at_error = False
            self.logger.info(f"Force gripping component '{request.component_name}'")

            if not self.pm_robot_utils.assembly_scene_analyzer.check_object_exists(request.component_name):
                raise PmRobotError(f"Object '{request.component_name}' does not exist!")

            self.logger.info(f"Force gripping component '{request.component_name}'")

            if not self.pm_robot_utils.assembly_scene_analyzer.is_gripper_empty():
                raise PmRobotError("Gripper is not empty! Can not grip new component!")

            if self.pm_robot_utils.assembly_scene_analyzer.check_component_assembled(request.component_name):
                raise PmRobotError(f"Component '{request.component_name}' is already assembled! Can not grip it!")
            
            gripping_frame = self.pm_robot_utils.assembly_scene_analyzer.get_gripping_frame_of_component(request.component_name)
            
            self.logger.info(f"Gripping component '{request.component_name}' at frame '{gripping_frame}'")
            
            algin_success = True
            align_msg = ""

            if self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_left(request.component_name) and request.align_orientation:
                algin_success, align_msg = self.node.gonio_skills.align_gonio_left(gripping_frame)

            elif self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_right(request.component_name) and request.align_orientation:
                algin_success, align_msg = self.node.gonio_skills.align_gonio_right(gripping_frame)
            
            else:
                pass

            if not algin_success:
                raise PmRobotError(f"Failed to align gonio for component '{request.component_name}': {align_msg}")

            #Offset
            move_tool_to_part_offset_success, move_offset_msg = self.move_gripper_to_frame(gripping_frame, z_offset=self.GRIP_APPROACH_OFFSET)

            if not move_tool_to_part_offset_success:
                raise PmRobotError(f"Failed to move gripper to part '{request.component_name}' at offset distance: {move_offset_msg}")
            

            self.logger.info("Moving to grip sensing start position!")

            move_tool_to_part_success, move_part_msg = self.move_gripper_to_frame(gripping_frame, z_offset=self.GRIP_SENSING_START_OFFSET)

            if not move_tool_to_part_success:
                raise PmRobotError(f"Failed to move gripper to part '{request.component_name}': {move_part_msg}")

            should_move_up_at_error = True

            self.pm_robot_utils.set_gripper_component_collision(component_name=request.component_name, 
                                                                state=False)
            
            # This is a position that is above the gripping point for a rough approach
            planning_req_rough = pm_moveit_srv.MoveToFrame.Request()
            planning_req_rough.target_frame = gripping_frame
            planning_req_rough.execute_movement = False
            planning_req_rough.translation.z = self.GRIP_SENSING_END_ROUGH

            planning_res_rough:pm_moveit_srv.MoveToFrame.Response = self.client_move_robot_tool_to_frame.call(planning_req_rough)

            if planning_res_rough.success is False:
                raise PmRobotError("Planning of END_ROUGH_OFFSET position failed!")
            
            # calculating the lower position
            planning_req = pm_moveit_srv.MoveToFrame.Request()
            planning_req.target_frame = gripping_frame
            planning_req.execute_movement = False
            planning_req.translation.z = self.GRIP_SENSING_END_OFFSET

            planning_res:pm_moveit_srv.MoveToFrame.Response = self.client_move_robot_tool_to_frame.call(planning_req)
            
            if planning_res.success is False:
                raise PmRobotError("Planning of END_OFFSET position failed!")
            
            if not self.pm_robot_utils.is_unity_running():
            # Iterative approach  
                force_sensing_rough_request = pm_msg_srv.GripperForceMove.Request()
                force_sensing_rough_response = pm_msg_srv.GripperForceMove.Response()

                force_sensing_rough_request.step_size = float(100) # in um
                force_sensing_rough_request.target_joints_xyz = [planning_res_rough.joint_values[0], planning_res_rough.joint_values[1], planning_res_rough.joint_values[2]]
                force_sensing_rough_request.max_f_xyz = [1.0, 1.0, 1.0] # in N

                self.logger.info(f"Starting rough approach with force sensing (step size: {force_sensing_rough_request.step_size} um. Approaching gripping point at {self.GRIP_SENSING_END_ROUGH*1e6} um above it)")
                
                force_sensing_rough_response = self.node.force_skills.force_sensing_move_callback(force_sensing_rough_request, force_sensing_rough_response)
                
                if not force_sensing_rough_response.success:
                    raise PmRobotError("Rough approach force sensing failed!")
                
                if force_sensing_rough_response.completed:
                    raise PmRobotError("Rough approach already exceeded the force limits! Make sure the gripping frame has been measured correctly!")
                else:
                    self.logger.info("Rough approach completed without exceeding force limits, proceeding to fine approach.")

                self.logger.warn(f"joints {str(planning_res.joint_names)}")
                self.logger.warn(f"joints {str(planning_res.joint_values)}")


            # Iterative approach  
            force_sensing_request = pm_msg_srv.GripperForceMove.Request()
            force_sensing_response = pm_msg_srv.GripperForceMove.Response()

            force_sensing_request.step_size = float(10) # in um
            force_sensing_request.target_joints_xyz = [planning_res.joint_values[0], planning_res.joint_values[1], planning_res.joint_values[2]]
            force_sensing_request.max_f_xyz = [1.0, 1.0, 1.0] # in N

            self.logger.info(f"Starting fine approach with force sensing (step size: {force_sensing_request.step_size} um. ")

            force_sensing_response = self.node.force_skills.force_sensing_move_callback(force_sensing_request, force_sensing_response)
            
            if not force_sensing_response.success:
                raise PmRobotError("Force sensing failed!")
            
            if not force_sensing_response.completed:
                raise PmRobotError("Fine approach did not detect contact! Make sure the gripping frame has been measured correctly and that the force thresholds are set appropriately!")
            
            # enable vacuum
            enable_success = self.pm_robot_utils.set_tool_vaccum(True)

            if not enable_success:
                raise PmRobotError(f"Failed to enable vacuum for component '{request.component_name}'!")

            disable_success = True

            # turn off the vacuum
            if self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_left(request.component_name, max_depth=1):
                self.logger.info(f"Disabling vacuum on gonio left for component '{request.component_name}'")
                disable_success = self.pm_robot_utils.set_gonio_left_vacuum(False)

            elif self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_right(request.component_name, max_depth=1):
                self.logger.info(f"Disabling vacuum on gonio right for component '{request.component_name}'")
                disable_success = self.pm_robot_utils.set_gonio_right_vacuum(False)
            else:
                self.logger.info(f"Component '{request.component_name}' is not on a gonio!")

            if not disable_success:
                raise PmRobotError(f"Failed to disable vacuum for component '{request.component_name}'!")

            # attach the component to the gripper
            attach_component_success = self.attach_component_to_gripper(request.component_name)
            
            if not attach_component_success:
                raise PmRobotError(f"Failed to attach component '{request.component_name}' to gripper")
            
            properties = ami_msg.ComponentProperties()
            properties.is_gripped = True

            set_properties_response: ami_srv.SetComponentProperties.Response = self.pm_robot_utils.set_component_properties(request.component_name, properties)

            if not set_properties_response.success:
                raise PmRobotError(f"Failed to set component properties for component '{request.component_name}' after gripping!")  
            
            response.success = True

            response.message = f"Component '{request.component_name}' gripped successfully!"
            self.logger.info(response.message)
            time.sleep(0.5)  # wait for properties to update in the assembly manager

        except (PmRobotError, ComponentNotFoundError, GrippingFrameNotFoundError) as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)

        finally:
            if should_move_up_at_error:
                move_relatively_success, lift_msg = self.lift_gripper_relative(self.GRIP_RELATIVE_LIFT_DISTANCE)
                if not move_relatively_success:
                    response.success = False
                    response.message = f"Failed to lift gripper after attaching component '{request.component_name}': {lift_msg}"
                    self.logger.error(response.message)

        return response
    
    def grip_component_callback(self, request:pm_skill_srv.GripComponent.Request, response:pm_skill_srv.GripComponent.Response):
        """TO DO: Add docstring"""

        self.pm_robot_utils.wait_for_initial_scene_update()

        try:
            should_move_up_at_error = False
            self.logger.info(f"Force gripping component '{request.component_name}'")

            if not self.pm_robot_utils.assembly_scene_analyzer.check_object_exists(request.component_name):
                raise PmRobotError(f"Object '{request.component_name}' does not exist!")

            self.logger.info(f"Force gripping component '{request.component_name}'")

            if not self.pm_robot_utils.assembly_scene_analyzer.is_gripper_empty():
                raise PmRobotError("Gripper is not empty! Can not grip new component!")

            if self.pm_robot_utils.assembly_scene_analyzer.check_component_assembled(request.component_name):
                raise PmRobotError(f"Component '{request.component_name}' is already assembled! Can not grip it!")
            
            gripping_frame = self.pm_robot_utils.assembly_scene_analyzer.get_gripping_frame_of_component(request.component_name)
            
            self.logger.info(f"Gripping component '{request.component_name}' at frame '{gripping_frame}'")
            
            algin_success = True
            align_msg = ""

            if self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_left(request.component_name) and request.align_orientation:
                algin_success, align_msg = self.node.gonio_skills.align_gonio_left(gripping_frame)

            elif self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_right(request.component_name) and request.align_orientation:
                algin_success, align_msg = self.node.gonio_skills.align_gonio_right(gripping_frame)
            
            else:
                pass

            if not algin_success:
                raise PmRobotError(f"Failed to align gonio for component '{request.component_name}': {align_msg}")

            #Offset
            move_tool_to_part_offset_success, move_offset_msg = self.move_gripper_to_frame(gripping_frame, z_offset=self.GRIP_APPROACH_OFFSET)

            if not move_tool_to_part_offset_success:
                raise PmRobotError(f"Failed to move gripper to part '{request.component_name}' at offset distance: {move_offset_msg}")
            

            self.logger.info("Moving to grip sensing start position!")

            move_tool_to_part_success, move_part_msg = self.move_gripper_to_frame(gripping_frame, z_offset=self.GRIP_SENSING_START_OFFSET)

            if not move_tool_to_part_success:
                raise PmRobotError(f"Failed to move gripper to part '{request.component_name}': {move_part_msg}")

            should_move_up_at_error = True

            self.pm_robot_utils.set_gripper_component_collision(component_name=request.component_name, 
                                                                state=False)
            

            # open gripper if it is a parallel gripper, before moving down onto the part
            if self.pm_robot_utils.pm_robot_config.tool.get_active_tool_type() == ParallelGripperConfig.TOOL_GRIPPER_1_JAW_IDENT:
                self.logger.info(f"Opening parallel 1 jaw gripper for component '{request.component_name}'")

                # TODO: Implement the actual gripper opening logic here (no 1-jaw service available yet)

            if self.pm_robot_utils.pm_robot_config.tool.get_active_tool_type() == ParallelGripperConfig.TOOL_GRIPPER_2_JAW_IDENT:
                self.logger.info(f"Opening parallel 2 jaw gripper for component '{request.component_name}'")
                self.pm_robot_utils.open_gripper_2_jaws()


            move_tool_to_part_success, move_part_msg = self.move_gripper_to_frame(gripping_frame, z_offset=0.0)

            if not move_tool_to_part_success:
                raise PmRobotError(f"Failed to move gripper to part '{request.component_name}': {move_part_msg}")

            if self.pm_robot_utils.pm_robot_config.tool.get_active_tool_type() == VacuumGripperConfig.TOOL_VACUUM_IDENT:
                # enable vacuum
                enable_success = self.pm_robot_utils.set_tool_vaccum(True)
                self.logger.info(f"Enabling vacuum for component '{request.component_name}'")

                if not enable_success:
                    raise PmRobotError(f"Failed to enable vacuum for component '{request.component_name}'!")

            if self.pm_robot_utils.pm_robot_config.tool.get_active_tool_type() == ParallelGripperConfig.TOOL_GRIPPER_1_JAW_IDENT:
                self.logger.info(f"Closing parallel 1 jaw gripper for component '{request.component_name}'")

                # TODO: Implement the actual gripper closing logic here (no 1-jaw service available yet)

            if self.pm_robot_utils.pm_robot_config.tool.get_active_tool_type() == ParallelGripperConfig.TOOL_GRIPPER_2_JAW_IDENT:
                self.logger.info(f"Closing parallel 2 jaw gripper for component '{request.component_name}'")
                self.pm_robot_utils.close_gripper_2_jaws()

            disable_success = True

            # turn off the vacuum
            if self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_left(request.component_name, max_depth=1):
                self.logger.info(f"Disabling vacuum on gonio left for component '{request.component_name}'")
                disable_success = self.pm_robot_utils.set_gonio_left_vacuum(False)

            elif self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_right(request.component_name, max_depth=1):
                self.logger.info(f"Disabling vacuum on gonio right for component '{request.component_name}'")
                disable_success = self.pm_robot_utils.set_gonio_right_vacuum(False)
            else:
                self.logger.info(f"Component '{request.component_name}' is not on a gonio!")

            if not disable_success:
                raise PmRobotError(f"Failed to disable vacuum for component '{request.component_name}'!")

            # attach the component to the gripper
            attach_component_success = self.attach_component_to_gripper(request.component_name)
            
            if not attach_component_success:
                raise PmRobotError(f"Failed to attach component '{request.component_name}' to gripper")
            
            properties = ami_msg.ComponentProperties()
            properties.is_gripped = True

            set_properties_response: ami_srv.SetComponentProperties.Response = self.pm_robot_utils.set_component_properties(request.component_name, properties)

            if not set_properties_response.success:
                raise PmRobotError(f"Failed to set component properties for component '{request.component_name}' after gripping!")  
            
            response.success = True

            response.message = f"Component '{request.component_name}' gripped successfully!"
            self.logger.info(response.message)
            time.sleep(0.5)  # wait for properties to update in the assembly manager

        except (PmRobotError, ComponentNotFoundError, GrippingFrameNotFoundError) as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)

        finally:
            if should_move_up_at_error:
                move_relatively_success, lift_msg = self.lift_gripper_relative(self.GRIP_RELATIVE_LIFT_DISTANCE)
                if not move_relatively_success:
                    response.success = False
                    response.message = f"Failed to lift gripper after attaching component '{request.component_name}': {lift_msg}"
                    self.logger.error(response.message)

        return response

    def place_component_callback(self, request:pm_skill_srv.PlaceComponent.Request, response:pm_skill_srv.PlaceComponent.Response):
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

        try:
            should_move_up_at_error = False

            if self.pm_robot_utils.assembly_scene_analyzer.is_gripper_empty():
                raise PmRobotError("Gripper is empty! Can not place component!")
            
            gripped_component = self.pm_robot_utils.assembly_scene_analyzer.get_gripped_component()
            
            assembly_frame = self.pm_robot_utils.assembly_scene_analyzer.get_assembly_frame_for_component(gripped_component)

            target_frame = self.pm_robot_utils.assembly_scene_analyzer.get_target_frame_for_component(gripped_component)

            self.logger.warn(f"Gripped Component: '{str(gripped_component)}'!")
            self.logger.warn(f"Assembly Frame: '{str(assembly_frame)}'!")
            self.logger.warn(f"Target Frame: '{str(target_frame)}'!")

            target_component = self.pm_robot_utils.assembly_scene_analyzer.get_component_for_frame_name(target_frame)
            
            instruction = self.pm_robot_utils.assembly_scene_analyzer.get_assembly_instruction(assembly_component=gripped_component,
                                                                                                  target_component=target_component)

            # recalculate the instruction before creating the place frame
            self.pm_robot_utils.recalculate_assembly_instruction(instruction_id=instruction.id)

            placing_frame = self._create_place_offset_frame(target_frame, request)

            placing_frame = target_frame

            # Align gonio to the left or right depending on the target frame

            algin_success = True
            align_msg = ""

            if self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_left(target_component) and request.align_orientation:
                algin_success, align_msg = self.node.gonio_skills.align_gonio_left(endeffector_override=placing_frame,
                                                        alignment_frame=assembly_frame)

            elif self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_right(target_component) and request.align_orientation:
                algin_success, align_msg = self.node.gonio_skills.align_gonio_right(endeffector_override=placing_frame,
                                                        alignment_frame=assembly_frame)

            else:
                pass

            if not algin_success:
                raise PmRobotError(f"Failed to align gonio for target component '{target_component}': {align_msg}")
            
            self.logger.warn("Endeffector override: " + assembly_frame)
            # Move component to the target frame with a z offset
            move_component_to_part_offset_success, move_offset_msg = self.move_gripper_to_frame(placing_frame, 
                                                                               endeffector_override=assembly_frame, 
                                                                               z_offset=self.RELEASE_LIFT_DISTANCE)

            if not move_component_to_part_offset_success:
                raise PmRobotError(f"Failed to move gripper to target part '{target_component}': {move_offset_msg}")

            should_move_up_at_error = True

            # Move component to the target frame
            move_component_to_part_success, move_part_msg = self.move_gripper_to_frame(placing_frame, 
                                                                        endeffector_override=assembly_frame,
                                                                        z_offset=0.0)
            
            if not move_component_to_part_success:
                raise PmRobotError(f"Failed to move gripper to target part '{target_component}': {move_part_msg}")
            
                      
            properties = ami_msg.ComponentProperties()
            properties.is_gripped = True
            properties.is_placed = True

            set_properties_response: ami_srv.SetComponentProperties.Response = self.pm_robot_utils.set_component_properties(gripped_component, properties)

            if not set_properties_response.success:
                raise PmRobotError(f"Failed to set component properties for component '{gripped_component}' after placing!")  
            
            response.success = True
            response.message = f"Component '{gripped_component}' placed successfully!"

        except (PmRobotError, 
                ComponentNotFoundError, 
                RefFrameNotFoundError, 
                AssemblyFrameNotFoundError,
                AssemblyInstructionNotFoundError,
                TargetFrameNotFoundError) as e:
            if should_move_up_at_error:
                move_relatively_success, lift_msg = self.lift_gripper_relative(self.RELEASE_LIFT_DISTANCE)
                if not move_relatively_success:
                    response.success = False
                    response.message = f"Failed to lift gripper after placing component '{gripped_component}': {lift_msg}"
                    self.logger.error(response.message)
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)
            
        finally:
            pass

        return response

    def _create_place_offset_frame(self, 
                                  target_frame, 
                                  request:pm_skill_srv.PlaceComponent.Request)->str:
        """
        Create a new frame for placing the component with an offset
        Args:
            target_frame (str): The name of the target frame to place the component
            request (pm_skill_srv.PlaceComponent.Request): The request containing the offset values
        Returns:
            str: The name of the created offset frame
        Raises:
            PmRobotError: If the frame creation fails
            RefFrameNotFoundError: If the target frame does not exist
            ComponentNotFoundError: If the target component does not exist
        """

        target_frame_obj = self.pm_robot_utils.assembly_scene_analyzer.get_ref_frame_by_name(target_frame)
        target_component = self.pm_robot_utils.assembly_scene_analyzer.get_component_for_frame_name(target_frame)

        #generate random number
        random_number = random.randint(1000, 9999)
        spawn_request = ami_srv.CreateRefFrame.Request()
        spawn_request.ref_frame.frame_name = f"{target_component}_PLACEMENT_offset_{random_number}"
        spawn_request.ref_frame.parent_frame = target_component
        spawn_request.ref_frame.pose.position.x = target_frame_obj.pose.position.x + request.x_offset_um * 1e-6
        spawn_request.ref_frame.pose.position.y = target_frame_obj.pose.position.y + request.y_offset_um * 1e-6
        spawn_request.ref_frame.pose.position.z = target_frame_obj.pose.position.z + request.z_offset_um * 1e-6

        # convert euler angles to quaternion
        if request.rx_offset_deg != 0.0 or request.ry_offset_deg != 0.0 or request.rz_offset_deg != 0.0:
            q = R.from_euler('xyz', [request.rx_offset_deg, request.ry_offset_deg, request.rz_offset_deg], degrees=True).as_quat()
            quat = Quaternion()
            quat.x = q[0]
            quat.y = q[1]
            quat.z = q[2]
            quat.w = q[3]
            # multiply quaternions
            # current orientation
            result_quat = quaternion_multiply(target_frame_obj.pose.orientation, quat)
            spawn_request.ref_frame.pose.orientation = result_quat
        else:
            spawn_request.ref_frame.pose.orientation = target_frame_obj.pose.orientation
        
        self.pm_robot_utils.create_ref_frame(spawn_request)

        return spawn_request.ref_frame.frame_name

    def release_component_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

        try:
            if self.pm_robot_utils.assembly_scene_analyzer.is_gripper_empty():
                raise PmRobotError("Gripper is empty! Can not release component!")

            gripped_component = self.pm_robot_utils.assembly_scene_analyzer.get_gripped_component()
            
            if gripped_component is None:
                raise PmRobotError("No gripped component found!")
            
            gripped_component = self.pm_robot_utils.assembly_scene_analyzer.get_gripped_component()
            
            assembly_frame = self.pm_robot_utils.assembly_scene_analyzer.get_assembly_frame_for_component(gripped_component)

            target_frame = self.pm_robot_utils.assembly_scene_analyzer.get_target_frame_for_component(gripped_component)

            target_component = self.pm_robot_utils.assembly_scene_analyzer.get_component_for_frame_name(target_frame)

            self.logger.warn(f"Gripped Component: '{str(gripped_component)}'!")
            self.logger.warn(f"Assembly Frame: '{str(assembly_frame)}'!")
            self.logger.warn(f"Target Frame: '{str(target_frame)}'!")

            # release component
            attach_component_success = self.attach_component_to_component(gripped_component, target_component)

            if not attach_component_success:
                raise PmRobotError(f"Failed to attach component '{gripped_component}' to component '{target_component}'")
            
            # release the component depending on the active tool type
            if self.pm_robot_utils.pm_robot_config.tool.get_active_tool_type() == VacuumGripperConfig.TOOL_VACUUM_IDENT:
                vaccum_off_success = self.pm_robot_utils.set_tool_vaccum(False)

                if not vaccum_off_success:
                    raise PmRobotError(f"Failed to deactivate vacuum for component '{gripped_component}'")

            elif self.pm_robot_utils.pm_robot_config.tool.get_active_tool_type() == ParallelGripperConfig.TOOL_GRIPPER_1_JAW_IDENT:
                self.logger.info(f"Opening parallel 1 jaw gripper to release component '{gripped_component}'")

                # TODO: Implement the actual gripper opening logic here (no 1-jaw service available yet)

            elif self.pm_robot_utils.pm_robot_config.tool.get_active_tool_type() == ParallelGripperConfig.TOOL_GRIPPER_2_JAW_IDENT:
                self.logger.info(f"Opening parallel 2 jaw gripper to release component '{gripped_component}'")
                self.pm_robot_utils.open_gripper_2_jaws()

            move_relatively_success, lift_msg = self.lift_gripper_relative(self.RELEASE_LIFT_DISTANCE)
            if not move_relatively_success:
                raise PmRobotError(f"Failed to lift gripper after releasing component '{gripped_component}': {lift_msg}")

            self.pm_robot_utils.set_gripper_component_collision(component_name=gripped_component, 
                                                    state=True)
            
            properties = ami_msg.ComponentProperties()
            properties.is_gripped = False
            properties.is_placed = True

            set_properties_response: ami_srv.SetComponentProperties.Response = self.pm_robot_utils.set_component_properties(gripped_component, properties)

            if not set_properties_response.success:
                raise PmRobotError(f"Failed to set component properties for component '{gripped_component}' after releasing!")  
            
            response.success = True
            response.message = f"Component '{gripped_component}' released successfully. New parent: '{target_component}'!"
            
        except (PmRobotError, 
                ComponentNotFoundError, 
                RefFrameNotFoundError, 
                AssemblyFrameNotFoundError,
                TargetFrameNotFoundError) as e:

                response.message = str(e)
                self.logger.error(response.message)
                response.success = False

        return response


    # def assemble_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
    #     self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

    #     global_stationary_component = self.pm_robot_utils.assembly_scene_analyzer.get_global_stationary_component()
    #     assemble_list = self.pm_robot_utils.assembly_scene_analyzer.get_components_to_assemble()
    #     first_component = self.pm_robot_utils.assembly_scene_analyzer.find_matches_for_component(global_stationary_component, only_unassembled=False)

    #     self.assembly_loop(first_component)
        
    #     response.success = True

    #     return response

    def vaccum_gripper_on_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        """Mimics the vacuum gripper funtionality by activating the vacuum and changing the parent frame"""

        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

        sim_time = self.pm_robot_utils.is_gazebo_running() or self.pm_robot_utils.is_unity_running()
        self.logger.info(f"Gazebo Simulation: {sim_time}")

        assembly_and_target_frames = self.pm_robot_utils.assembly_scene_analyzer.get_all_assembly_and_target_frames()      

        try:
            if not self.pm_robot_utils.assembly_scene_analyzer.is_gripper_empty():
                response.success = False
                response.message = "Gripper is not empty! Can not grip new component!"
                self.logger.error(response.message)
                return response
            
            # get gripping component
            for component_frame in assembly_and_target_frames:
                if self.ASSEMBLY_FRAME_INDICATOR in component_frame[1]:
                    target_component = component_frame[0]
                    self.logger.info(f"moving component: '{target_component}'")
                    break
            
            # activate the vacuum at head_nozzle
            if not sim_time:
                response_vacuum:EmptyWithSuccess.Response = self.pm_robot_utils.client_turn_on_vacuum_tool_head.call(EmptyWithSuccess.Request())
                if not response_vacuum.success:
                    response.success = False
                    response.message = "Failed to activate vacuum!"
                    self.logger.error(response.message)
                    return response

            response_attachment = self.attach_component_to_gripper(target_component)
            if not response_attachment:
                response.success = False
                response.message = "Failed change parent frame!"
                self.logger.error(response.message)
                return response

            response.success = True
            response.message = "Component gripped and attached to the gripper!" 
                
        except ValueError as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)
            return response
                
        return response

    def vaccum_gripper_off_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        """Mimics the vacuum gripper funtionality by deactivating the vacuum and changing the parent frame"""

        sim_time = self.pm_robot_utils.is_gazebo_running() or self.pm_robot_utils.is_unity_running()
        self.logger.info(f"Gazebo Simulation: {sim_time}")

        assembly_and_target_frames = self.pm_robot_utils.assembly_scene_analyzer.get_all_assembly_and_target_frames()

        try:
            if not sim_time:
                response_vacuum:EmptyWithSuccess.Response = self.pm_robot_utils.client_turn_off_vacuum_tool_head.call(EmptyWithSuccess.Request())
                if not response_vacuum.success:
                    response.success = False
                    response.message = "Failed to deactivate vacuum!"
                    self.logger.error(response.message)
                    return response
            
            # get target component
            target_component = None
            for component_frame in assembly_and_target_frames:
                if self.TARGET_FRAME_INDICATOR in component_frame[1]:
                    target_component = component_frame[0]
                    self.logger.info(f"target component: '{target_component}'")
                    break

            if target_component is None:
                raise ValueError(f"No target component found in assembly_and_target_frames (indicator: '{self.TARGET_FRAME_INDICATOR}')")

            response_attachment = self.attach_component_to_component(self.pm_robot_utils.assembly_scene_analyzer.get_gripped_component(), target_component)
            if not response_attachment:
                response.success = False
                response.message = "Failed change parent frame!"
                self.logger.error(response.message)
                return response
            
            response.success = True
            response.message = "Component released and attached to the target component!"   
                
        except ValueError as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)
            return response
                
        return response

            
    # def assembly_loop(self, list_of_components:list[str]):
    #     self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()
    #     for component in list_of_components:
    #         self.logger.error(f"ComponentTTTTT: {component}")
    #         if not self.pm_robot_utils.assembly_scene_analyzer.check_component_assembled(component):

    #             response = self.grip_component_callback(pm_skill_srv.GripComponent.Request(component_name=component), pm_skill_srv.GripComponent.Response())
    #             if not response.success:
    #                 self.logger.error(f"Failed to grip component '{component}'")
    #                 return False
                
    #             response = self.place_component_callback(pm_skill_srv.PlaceComponent.Request(), pm_skill_srv.PlaceComponent.Response())
    #             if not response.success:
    #                 self.logger.error(f"Failed to place component '{component}'")
    #                 return False
                
    #         list_of_component= self.pm_robot_utils.assembly_scene_analyzer.find_matches_for_component(component,only_unassembled=False)
    #         self.logger.warn(f"List of components: {list_of_component}")
    #         self.assembly_loop(list_of_component)

    #     return True
    
    def simtime_callback(self, msg:std_msg.Bool):
        if msg.data:
            self.logger.info("Simulation time is active!")
            self.sim_time = True
        else:
            self.logger.info("Simulation time is inactive!")
            self.sim_time = False

    def lift_gripper_relative(self, distance:float)-> tuple[bool, str]:
        call_async = False

        if not self.pm_robot_utils.client_move_robot_tool_relative.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_tool_relative' not available")
            return False, "Service '/pm_moveit_server/move_tool_relative' not available"
        
        req = pm_moveit_srv.MoveRelative.Request()
        req.translation.z = distance
        req.execute_movement = True

        if call_async:
            future = self.pm_robot_utils.client_move_robot_tool_relative.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False, f"Service call failed: {future.exception()}"
            return future.result().success, future.result().message
        else:
            response:pm_moveit_srv.MoveRelative.Response = self.pm_robot_utils.client_move_robot_tool_relative.call(req)
            return response.success, response.message
        
    def move_gripper_to_frame(self, frame_name:str, endeffector_override = None, x_offset=None, y_offset=None, z_offset=None)-> tuple[bool, str]:
        call_async = False

        if not self.client_move_robot_tool_to_frame.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_tool_to_frame' not available")
            return False, "Service '/pm_moveit_server/move_tool_to_frame' not available"
        
        req = pm_moveit_srv.MoveToFrame.Request()
        req.target_frame = frame_name
        req.execute_movement = True
        req.translation.x = 0.0000001
        req.translation.y = 0.0000001
        req.translation.z = 0.0000001

        if endeffector_override is not None:
            req.endeffector_frame_override = endeffector_override

        if x_offset is not None:
            req.translation.x = x_offset

        if y_offset is not None:
            req.translation.y = y_offset

        if z_offset is not None:
            req.translation.z = z_offset

        if call_async:
            future = self.client_move_robot_tool_to_frame.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False, f"Service call failed: {future.exception()}"
            return future.result().success, future.result().message
        else:
            response:pm_moveit_srv.MoveToFrame.Response = self.client_move_robot_tool_to_frame.call(req)
            return response.success, response.message

    def move_laser_to_frame(self, frame_name:str, z_offset: float=None)-> tuple[bool, str]:
        """
        z_offset in m
        """
        call_async = False

        if not self.pm_robot_utils.client_move_robot_laser_to_frame.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_laser_to_frame' not available")
            return False, "Service '/pm_moveit_server/move_laser_to_frame' not available"
        
        req = pm_moveit_srv.MoveToFrame.Request()
        req.target_frame = frame_name
        req.execute_movement = True

        if z_offset is not None:
            req.translation.z = z_offset

        if call_async:
            future = self.pm_robot_utils.client_move_robot_laser_to_frame.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False, f"Service call failed: {future.exception()}"
            return future.result().success, future.result().message
        else:
            response:pm_moveit_srv.MoveToFrame.Response = self.pm_robot_utils.client_move_robot_laser_to_frame.call(req)
            return response.success, response.message

    def move_confocal_top_to_frame(self, frame_name:str, z_offset: float=None)-> tuple[bool, str]:
        """
        z_offset in m
        """
        call_async = False

        if not self.pm_robot_utils.client_move_robot_confocal_top_to_frame.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_confocal_top_to_frame' not available")
            return False, "Service '/pm_moveit_server/move_confocal_top_to_frame' not available"

        req = pm_moveit_srv.MoveToFrame.Request()
        req.target_frame = frame_name
        req.execute_movement = True

        if z_offset is not None:
            req.translation.z = z_offset

        if call_async:
            future = self.pm_robot_utils.client_move_robot_confocal_top_to_frame.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False, f"Service call failed: {future.exception()}"
            return future.result().success, future.result().message
        else:
            response:pm_moveit_srv.MoveToFrame.Response = self.pm_robot_utils.client_move_robot_confocal_top_to_frame.call(req)
            return response.success, response.message

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
