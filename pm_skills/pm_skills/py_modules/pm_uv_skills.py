from example_interfaces.srv import SetBool

import assembly_manager_interfaces.msg as ami_msg
import assembly_manager_interfaces.srv as ami_srv
import pm_msgs.srv as pm_msg_srv
from assembly_scene_publisher.py_modules.scene_errors import RefFrameNotFoundError
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_skills.py_modules.pm_skill_domain import PmSkillDomain


class PmUvSkills(PmSkillDomain):
    def move_uv_in_curing_position_service_callback(self, request:pm_msg_srv.EmptyWithSuccess.Request, response:pm_msg_srv.EmptyWithSuccess.Response):
        """Moves both UV LEDs in curing position"""
        try:
            srv_request = SetBool.Request()
            srv_request.data = True
            response.success = self.pm_robot_utils.move_uv_in_curing_position(srv_request)
             
        except PmRobotError as e:
            self.get_logger().error(f"Error moving UV LEDs in curing position: {e.message}")
            response.success = False
            response.message = e.message
        return response
    
    def move_uv_out_of_curing_position_service_callback(self, request:pm_msg_srv.EmptyWithSuccess.Request, response:pm_msg_srv.EmptyWithSuccess.Response):
        """Moves both UV LEDs out of curing position"""
        try:
            srv_request = SetBool.Request()
            srv_request.data = False
            response.success = self.pm_robot_utils.move_uv_in_curing_position(srv_request)

        except PmRobotError as e:
            self.get_logger().error(f"Error moving UV LEDs out of curing position: {e.message}")
            response.success = False
            response.message = e.message
        return response

    def uv_cure_adv_callback(self, request: pm_msg_srv.UVCuringSkill.Request, response:pm_msg_srv.UVCuringSkill.Response):
        """
        Advanced UV curing skill that not only performs the UV curing action but also updates the assembly scene analyzer's frame properties for glue points. 
        This ensures that after curing, the system is aware of which glue points have been cured, allowing for better decision-making in subsequent steps of the assembly process.
        """
        
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()
        try:

            if not self.pm_robot_utils.client_uv_cure.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.pm_robot_utils.client_uv_cure.srv_name}' not available!")

            res: pm_msg_srv.UVCuringSkill.Response = self. pm_robot_utils.client_uv_cure.call(request)
            
            if not res.success:
                response.message = res.message
                raise PmRobotError(f"UV curing service call failed: {res.message}")
            
            all_frame_names = self.pm_robot_utils.assembly_scene_analyzer.get_all_component_frames()

            for frame_name in all_frame_names:
                frame = self.pm_robot_utils.assembly_scene_analyzer.get_ref_frame_by_name(frame_name)
                g_properties = frame.properties.glue_pt_frame_properties
                    
                if g_properties.is_glue_point and g_properties.has_been_placed:
                    g_properties.has_been_cured = True
                    self.pm_robot_utils.set_frame_properties(frame.frame_name, frame.properties)

            placed_components = self.pm_robot_utils.assembly_scene_analyzer.get_placed_components()
            
            if not placed_components:
                raise PmRobotError("No placed components found!")
            
            properties = ami_msg.ComponentProperties()
            properties.is_assembled = True

            # do not change the is_gripped property during curing
            if self.pm_robot_utils.assembly_scene_analyzer.is_gripper_empty():
                properties.is_gripped = False
            else:
                properties.is_gripped = True

            for placed_component in placed_components:
                set_properties_response: ami_srv.SetComponentProperties.Response = self.pm_robot_utils.set_component_properties(placed_component, properties)
            
            if not set_properties_response.success:
                raise PmRobotError(f"Failed to set component properties for component '{placed_component}' after curing!") 
            
            response.success = True
            response.message = "UV curing completed successfully and glue point frame properties updated."

        except (PmRobotError, RefFrameNotFoundError) as e:
            self.get_logger().error(f"Error during advanced UV curing: {e.message}")
            response.success = False
            response.message = str(e)
        return response

