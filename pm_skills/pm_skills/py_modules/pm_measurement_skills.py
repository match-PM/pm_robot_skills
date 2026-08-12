import time

from geometry_msgs.msg import TransformStamped

import assembly_manager_interfaces.srv as ami_srv
import pm_skills_interfaces.srv as pm_skill_srv
from assembly_scene_publisher.py_modules.scene_errors import ComponentNotFoundError, RefFrameNotFoundError
from assembly_scene_publisher.py_modules.tf_functions import get_transform_for_frame_in_world
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_skills.py_modules.PmRobotUtils import PmRobotUtils
from pm_skills.py_modules.pm_skill_domain import PmSkillDomain


class PmMeasurementSkills(PmSkillDomain):
    def measure_with_laser_callback(self, 
                                    request:pm_skill_srv.CorrectFrameLaser.Request, 
                                    response:pm_skill_srv.CorrectFrameLaser.Response):
        try:
            move_laser_to_frame_success, move_msg = self.node.gripper_skills.move_laser_to_frame(request.frame_name)
            
            # get compenent id if possible
            comp_id = "None"
            component_name = "Not a component frame"
            try:
                component_name = self.pm_robot_utils.assembly_scene_analyzer.get_component_for_frame_name(request.frame_name)
                component = self.pm_robot_utils.assembly_scene_analyzer.get_component_by_name(component_name)
                comp_id = component.uuid

            except ComponentNotFoundError:
                component_name = "None"
                comp_id = "None"

            except (RefFrameNotFoundError) as e:
                message = str(e)
                raise PmRobotError(message)

            response.component_name = component_name
            response.component_uuid = comp_id

            offset = 0.0
            
            if not move_laser_to_frame_success:
                raise PmRobotError(f"Failed to move laser to frame '{request.frame_name}': {move_msg}")


            if not self.pm_robot_utils._check_for_valid_laser_measurement():

                if request.use_iterative_sensing:

                    # time.sleep(1)

                    initial_z = self.pm_robot_utils.get_current_joint_state(PmRobotUtils.Z_Axis_JOINT_NAME)

                    # time.sleep(1)

                    # move up
                    self._logger.warn("MOVING UP")
                    move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0, 0, -3.0*1e-3,time=1)
                                                    
                    if not move_success:
                        raise PmRobotError(f"Failed to move up for iterative laser sensing on frame '{request.frame_name}'")
                    
                    step_inc = 0.4 # in mm
                    self._logger.warn("Laser measurement not valid! Trying to iteratively find a valid value!")                

                    x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
                                                    measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
                                                    length = (0.0, 0.0, 4.0),
                                                    step_inc = step_inc,
                                                    total_time = 8.0)
                    
                    if x is None:
                        raise PmRobotError(f"Iterative sensing failed to find valid laser measurement for frame '{request.frame_name}'")
                    
                    offset = initial_z - final_z
                    self._logger.info(f"Found valid value at: {offset} m")

                else:
                    raise PmRobotError(f"Laser measurement not valid for frame '{request.frame_name}'! OUT OF RANGE")


                self._logger.info("Valid value found!")      
            
            laser_measurement = self.pm_robot_utils.get_laser_measurement(unit="m") + float(offset)
            
            self._logger.info(f"Laser measurement: {laser_measurement} m ")

            response.correction_values.z = laser_measurement
            response.success = True
            response.message = f"Measurement: {laser_measurement}"

        except PmRobotError as e:
            response.success = False
            response.message = str(e)
            self._logger.error(response.message)

        return response

    def correct_frame_with_laser(self, request:pm_skill_srv.CorrectFrameLaser.Request, response:pm_skill_srv.CorrectFrameLaser.Response):
        try:
            self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

            frame_from_scene = self.pm_robot_utils.assembly_scene_analyzer.is_frame_from_scene(request.frame_name)

            if not frame_from_scene:
                raise RefFrameNotFoundError(f"Frame '{request.frame_name}' is not from assembly scene!")
            
            measure_frame_request = pm_skill_srv.CorrectFrameLaser.Request()

            measure_frame_request.frame_name = request.frame_name
            measure_frame_request.remeasure_after_correction = request.remeasure_after_correction
            measure_frame_request.use_iterative_sensing = request.use_iterative_sensing
            
            iterations = 1

            if request.remeasure_after_correction:
                iterations = 2
            
            for i in range(iterations):

                if i == 1:
                    self._logger.info("Remeasuring after correction...")
                    
                measure_frame_response = pm_skill_srv.CorrectFrameLaser.Response()

                response_mes:pm_skill_srv.CorrectFrameLaser.Response = self.measure_with_laser_callback(measure_frame_request, measure_frame_response)
                
                response.component_name = response_mes.component_name
                response.component_uuid = response_mes.component_uuid

                if not response_mes.success:
                    raise PmRobotError(f"Measuring frame '{request.frame_name}' with laser failed! Reason: {response_mes.message}")
                
                world_pose:TransformStamped = get_transform_for_frame_in_world(request.frame_name, self.tf_buffer, self._logger)

                world_pose.transform.translation.z += response_mes.correction_values.z
                response.correction_values.z = response_mes.correction_values.z
                
                adapt_frame_request = ami_srv.ModifyPoseAbsolut.Request()
                adapt_frame_request.frame_name = request.frame_name
                adapt_frame_request.pose.position.x = world_pose.transform.translation.x
                adapt_frame_request.pose.position.y = world_pose.transform.translation.y
                adapt_frame_request.pose.position.z = world_pose.transform.translation.z
                adapt_frame_request.pose.orientation = world_pose.transform.rotation
                adapt_frame_request.set_laser_measured = True
                
                if not self.adapt_frame_client.wait_for_service(timeout_sec=1.0):
                    self._logger.error(f"Service '{self.adapt_frame_client.srv_name}' not available. Assembly manager started?...")
                    response.success= False
                    response.message = f"Service '{self.adapt_frame_client.srv_name}' not available. Assembly manager started?..."
                    return response
                
                result_adapt:ami_srv.ModifyPoseAbsolut.Response = self.adapt_frame_client.call(adapt_frame_request)

                response.success = result_adapt.success

        except (PmRobotError,RefFrameNotFoundError) as e:
            response.success = False
            response.message = str(e)
            self._logger.error(response.message)

        return response
    
    def measure_frame_with_confocal_bottom(self, request:pm_skill_srv.CorrectFrameLaser.Request, response:pm_skill_srv.CorrectFrameLaser.Response):
        
        tcp_name = self.pm_robot_utils.TCP_CONFOCAL_BOTTOM  # we need to move the frame attached to the robot to the tcp. We use the move camera method for that
        try:
            # get compenent id if possible
            comp_id = "None"
            component_name = "Not a component frame"
            try:
                component_name = self.pm_robot_utils.assembly_scene_analyzer.get_component_for_frame_name(request.frame_name)
                component = self.pm_robot_utils.assembly_scene_analyzer.get_component_by_name(component_name)
                comp_id = component.uuid

            except ComponentNotFoundError:
                component_name = "None"
                comp_id = "None"

            except (RefFrameNotFoundError) as e:
                message = str(e)
                raise PmRobotError(message)

            response.component_name = component_name
            response.component_uuid = comp_id

            move_frame_to_confocal_bottom_success, move_msg = self.pm_robot_utils.move_camera_top_to_frame(   frame_name = tcp_name,
                                                                                                    endeffector_override=request.frame_name)
            
            if not move_frame_to_confocal_bottom_success:
                raise PmRobotError(f"Failed to move frame '{request.frame_name}' to confocal bottom: {move_msg}")

            time.sleep(1)

            if not self.pm_robot_utils.check_confocal_bottom_measurement_in_range():
                raise PmRobotError(f"Confocal bottom measurement not valid for frame '{request.frame_name}'! OUT OF RANGE")
            
            confocal_measurement = self.pm_robot_utils.get_confocal_bottom_measurement(unit="m")
            
            self._logger.info(f"Measurement: {confocal_measurement*1e6} um ")

            response.correction_values.z = confocal_measurement
            response.success = True
            response.message = f"Measurement: {confocal_measurement*1e6} um"

        except PmRobotError as e:
            response.success = False
            response.message = str(e)
            self._logger.error(response.message)

        return response

    def correct_frame_with_confocal_bottom(self, request:pm_skill_srv.CorrectFrameLaser.Request, response:pm_skill_srv.CorrectFrameLaser.Response):
        
        try:
            self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

            if not(self.pm_robot_utils.assembly_scene_analyzer.is_frame_from_scene(request.frame_name)):
                raise RefFrameNotFoundError(f"Frame '{request.frame_name}' is not from assembly scene!")

            measure_frame_request = pm_skill_srv.CorrectFrameLaser.Request()
            measure_frame_response = pm_skill_srv.CorrectFrameLaser.Response()
            
            measure_frame_request.frame_name = request.frame_name
            measure_frame_request.remeasure_after_correction = request.remeasure_after_correction
            measure_frame_request.use_iterative_sensing = request.use_iterative_sensing
            
            response_mes:pm_skill_srv.CorrectFrameLaser.Response = self.measure_frame_with_confocal_bottom(measure_frame_request, measure_frame_response)
            
            response.component_name = response_mes.component_name
            response.component_uuid = response_mes.component_uuid

            #self._logger.warn(f"REs {str(response_mes)}")

            if not response_mes.success:
                raise PmRobotError(f"Measuring frame '{request.frame_name}' with confocal bottom failed!")
                    
            world_pose:TransformStamped = get_transform_for_frame_in_world(request.frame_name, self.tf_buffer, self._logger)

            world_pose.transform.translation.z += response_mes.correction_values.z

            adapt_frame_request = ami_srv.ModifyPoseAbsolut.Request()
            adapt_frame_request.frame_name = request.frame_name
            adapt_frame_request.pose.position.x = world_pose.transform.translation.x
            adapt_frame_request.pose.position.y = world_pose.transform.translation.y
            adapt_frame_request.pose.position.z = world_pose.transform.translation.z
            adapt_frame_request.pose.orientation = world_pose.transform.rotation
            adapt_frame_request.set_laser_measured = True

            if not self.adapt_frame_client.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.adapt_frame_client.srv_name}' not available. Assembly manager started?...")

            result_adapt:ami_srv.ModifyPoseAbsolut.Response = self.adapt_frame_client.call(adapt_frame_request)

            # result_adapt = ami_srv.ModifyPoseAbsolut.Response()

            response.success = result_adapt.success
            response.correction_values.z = response_mes.correction_values.z

        except (PmRobotError,RefFrameNotFoundError) as e:
            response.success = False
            response.message = str(e)
            self._logger.error(response.message)

        return response

    def measure_frame_with_confocal_top(self, request:pm_skill_srv.CorrectFrameLaser.Request, response:pm_skill_srv.CorrectFrameLaser.Response):

        move_confocal_top_to_frame_success, move_msg = self.pm_robot_utils.move_confocal_top_to_frame(request.frame_name)

        time.sleep(1)

        try:
            comp_id = "None"
            component_name = "Not a component frame"
            try:
                component_name = self.pm_robot_utils.assembly_scene_analyzer.get_component_for_frame_name(request.frame_name)
                component = self.pm_robot_utils.assembly_scene_analyzer.get_component_by_name(component_name)
                comp_id = component.uuid

            except ComponentNotFoundError:
                component_name = "None"
                comp_id = "None"

            except (RefFrameNotFoundError) as e:
                message = str(e)
                raise PmRobotError(message)

            response.component_name = component_name
            response.component_uuid = comp_id

            offset = 0.0
            
            if not move_confocal_top_to_frame_success:
                raise PmRobotError(f"Failed to move confocal top to frame '{request.frame_name}': {move_msg}")

            if not self.pm_robot_utils.check_confocal_top_measurement_in_range():

                if request.use_iterative_sensing:

                    # time.sleep(1)

                    initial_z = self.pm_robot_utils.get_current_joint_state(PmRobotUtils.Z_Axis_JOINT_NAME)

                    # time.sleep(1)

                    # move up
                    self._logger.warn("MOVING UP")
                    move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0, 0, -3.0*1e-3,time=1)
                                                    
                    if not move_success:
                        raise PmRobotError(f"Failed to move up for iterative confocal top sensing on frame '{request.frame_name}'")
                    
                    step_inc = 0.4 # in mm
                    self._logger.warn("Confocal top measurement not valid! Trying to iteratively find a valid value!")                

                    x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
                                                    measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
                                                    length = (0.0, 0.0, 4.0),
                                                    step_inc = step_inc,
                                                    total_time = 8.0)
                    
                    if x is None:
                        raise PmRobotError(f"Iterative sensing failed to find valid confocal top measurement for frame '{request.frame_name}'")
                    
                    offset = initial_z - final_z
                    self._logger.info(f"Found valid value at: {offset} m")

                else:
                    raise PmRobotError(f"Confocal top measurement not valid for frame '{request.frame_name}'! OUT OF RANGE")


            confocal_measurement = self.pm_robot_utils.get_confocal_top_measurement(unit="m") + float(offset)

            self._logger.warn(f"Confocal measurement: {confocal_measurement*1e6} um ")

            response.correction_values.z = confocal_measurement
            response.success = True
            response.message = f"Measurement: {confocal_measurement*1e6:.2f} um"

        except PmRobotError as e:
            response.success = False
            response.message = str(e)
            self._logger.error(response.message)

        return response
    

    def correct_frame_with_confocal_top(self, request:pm_skill_srv.CorrectFrameLaser.Request, response:pm_skill_srv.CorrectFrameLaser.Response):

        try:
            self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

            if not self.pm_robot_utils.assembly_scene_analyzer.is_frame_from_scene(request.frame_name):
                raise RefFrameNotFoundError(f"Frame '{request.frame_name}' is not from assembly scene!")

            measure_frame_request = pm_skill_srv.CorrectFrameLaser.Request()
            
            measure_frame_request.frame_name = request.frame_name
            measure_frame_request.remeasure_after_correction = request.remeasure_after_correction
            measure_frame_request.use_iterative_sensing = request.use_iterative_sensing
            

            iterations = 1

            if request.remeasure_after_correction:
                iterations = 2
            
            for i in range(iterations):
                if i == 1:
                    self._logger.warn("Re-measuring after correction...")

                measure_frame_response = pm_skill_srv.CorrectFrameLaser.Response()

                response_mes:pm_skill_srv.CorrectFrameLaser.Response = self.measure_frame_with_confocal_top(measure_frame_request, measure_frame_response)
                
                response.component_name = response_mes.component_name
                response.component_uuid = response_mes.component_uuid
                
                #self._logger.warn(f"REs {str(response_mes)}")

                if not response_mes.success:
                    raise PmRobotError(f"Measuring frame '{request.frame_name}' with confocal top failed!")
                        
                world_pose:TransformStamped = get_transform_for_frame_in_world(request.frame_name, self.tf_buffer, self._logger)

                world_pose.transform.translation.z += response_mes.correction_values.z

                response.correction_values.z = response_mes.correction_values.z
                
                adapt_frame_request = ami_srv.ModifyPoseAbsolut.Request()
                adapt_frame_request.frame_name = request.frame_name
                adapt_frame_request.pose.position.x = world_pose.transform.translation.x
                adapt_frame_request.pose.position.y = world_pose.transform.translation.y
                adapt_frame_request.pose.position.z = world_pose.transform.translation.z
                adapt_frame_request.pose.orientation = world_pose.transform.rotation
                adapt_frame_request.set_laser_measured = True
                
                if not self.adapt_frame_client.wait_for_service(timeout_sec=1.0):
                    self._logger.error(f"Service '{self.adapt_frame_client.srv_name}' not available. Assembly manager started?...")
                    response.success= False
                    response.message = f"Service '{self.adapt_frame_client.srv_name}' not available. Assembly manager started?..."
                    return response
                
                result_adapt:ami_srv.ModifyPoseAbsolut.Response = self.adapt_frame_client.call(adapt_frame_request)

                response.success = result_adapt.success


        except (PmRobotError,RefFrameNotFoundError) as e:
            response.success = False
            response.message = str(e)
            self._logger.error(response.message)    
        
        return response

    def check_frame_mes_confocal_top(self, request:pm_skill_srv.CheckFrameMeasurable.Request, response:pm_skill_srv.CheckFrameMeasurable.Response):

        CONFOCAL_LASER_OFFSET = 8*1e-3 # in m, distance from confocal top to laser point

        res = self.pm_robot_utils.check_frame_mes(request = request,
                                                         offset_m=CONFOCAL_LASER_OFFSET,
                                                         move_client= self.pm_robot_utils.client_move_robot_laser_to_frame)
        response = res
        
        return response


    def check_frame_mes_laser_top(self, request:pm_skill_srv.CheckFrameMeasurable.Request, response:pm_skill_srv.CheckFrameMeasurable.Response):
        CONFOCAL_LASER_OFFSET = 3*1e-3 # in m, distance from confocal top to laser point

        res = self.pm_robot_utils.check_frame_mes(request = request,
                                                         offset_m=CONFOCAL_LASER_OFFSET,
                                                         move_client = self.pm_robot_utils.client_move_robot_laser_to_frame)
        response = res

        return response

    def check_frame_mes_confocal_bottom(self, request:pm_skill_srv.CheckFrameMeasurable.Request, response:pm_skill_srv.CheckFrameMeasurable.Response):
        CONFOCAL_LASER_OFFSET =40*1e-3 # in m, distance from confocal top to laser point

        res = self.pm_robot_utils.check_frame_mes_bot(request = request,
                                                        target_tcp_frame = self.pm_robot_utils.TCP_CONFOCAL_BOTTOM,
                                                        offset_m=CONFOCAL_LASER_OFFSET)
        response = res

        return response
