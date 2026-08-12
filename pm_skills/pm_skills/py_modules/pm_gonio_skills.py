import time

import numpy as np
import rclpy
from geometry_msgs.msg import Transform
from scipy.spatial.transform import Rotation as R

import pm_moveit_interfaces.srv as pm_moveit_srv
import pm_skills_interfaces.srv as pm_skill_srv
from pm_moveit_interfaces.srv import MoveRelative
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_skills.py_modules.pm_skill_domain import PmSkillDomain

PM_ROBOT_GRIPPER_FRAME = 'PM_Robot_Tool_TCP'


class PmGonioSkills(PmSkillDomain):
    def iterative_align_gonio_right(self, request: pm_skill_srv.IterativeGonioAlign.Request, response:pm_skill_srv.IterativeGonioAlign.Response):
        
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

        try:

            # the request never changes
            align_request = pm_moveit_srv.AlignGonio.Request()
            align_request.execute_movement = True
            align_request.endeffector_frame_override = request.component_alignment_frame
            align_request.target_frame = request.target_alignment_frame

            log_message = {}
            iterations = request.num_iterations

            initial_approach = True
            for iter in range(iterations):
                run_number = iter + 1
                self.logger.info(f"STARTING RUN '{run_number}/{iterations}")
            
                for frame in request.frames_to_measure:
                    self.logger.info(f"Measuring frame '{frame}'")
                    measure_request = pm_skill_srv.CorrectFrameLaser.Request()
                    measure_response = pm_skill_srv.CorrectFrameLaser.Response()
                    measure_request.frame_name = frame
                    measure_request.use_iterative_sensing = True

                    if initial_approach:
                        move_success, move_msg = self.node.gripper_skills.move_laser_to_frame(frame, z_offset=0.02)
                        if not move_success:
                            self.logger.error(f"Moving laser to frame '{frame}' for initial approach failed!")
                            response.success = False
                            response.message = f"Moving laser to frame '{frame}' for initial approach failed: {move_msg}"
                            return response
                        initial_approach = False

                    if request.confocal_laser:
                        measure_response = self.node.measurement_skills.correct_frame_with_confocal_top(measure_request, measure_response)
                    else:
                        measure_response = self.node.measurement_skills.correct_frame_with_laser(measure_request, measure_response)

                    if not measure_response.success:
                        raise PmRobotError(f"Correcting frame '{frame}' failed! {measure_response.message}")

                # calculating the angle difference
                try:
                    transform: Transform = self.pm_robot_utils.get_transform_for_frame(request.component_alignment_frame, 
                                                                            request.target_alignment_frame)
                    
                    current_gonio_angles: Transform = self.pm_robot_utils.get_transform_for_frame(request.component_alignment_frame, 
                                                                                                    'world')
                except ValueError as e:
                    self.logger.error(f"Error: {e}")
                    self.logger.error(f"Frame '{frame}' does not seem to exist.")
                    raise PmRobotError(f"Frame '{frame}' does not seem to exist: {e}")

                angles = R.from_quat([transform.rotation.x,
                                    transform.rotation.y,
                                    transform.rotation.z,
                                    transform.rotation.w]
                                    ).as_euler('xyz', degrees=True)

                angles_current = R.from_quat([current_gonio_angles.rotation.x,
                                                current_gonio_angles.rotation.y,
                                                current_gonio_angles.rotation.z,
                                                current_gonio_angles.rotation.w]
                                            ).as_euler('xyz', degrees=True)

                self.logger.info(f"Initial deviation at {iter} - x: {angles[0]}, y: {angles[1]}, z: {angles[2]}")
                self.logger.info(f"Current angles of the goniometer at {iter} - x: {angles_current[0]}, y: {angles_current[1]}, z: {angles_current[2]}")
                self.logger.info("Aligning goniometer!")

                #get joints
                joint_1_pre = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_RIGHT_STAGE_1)
                joint_2_pre = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_RIGHT_STAGE_2)

                if not self.pm_robot_utils.client_align_gonio_right.wait_for_service(1):
                    raise PmRobotError(f"Client '{self.pm_robot_utils.client_align_gonio_right.srv_name} not available!")
                
                align_response:pm_moveit_srv.AlignGonio.Response = self.pm_robot_utils.client_align_gonio_right.call(align_request)

                if not align_response.success:
                    raise PmRobotError(f"Aligning goniometer failed! {align_response.message}")
                
                time.sleep(1)  # wait for the robot to settle

                joint_1_post = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_RIGHT_STAGE_1)
                joint_2_post = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_RIGHT_STAGE_2)

                difference_joint_1 = (joint_1_post - joint_1_pre)*180/np.pi
                difference_joint_2 = (joint_2_post - joint_2_pre)*180/np.pi

                log_message[iter] = f"Iteration {run_number}: Gonio joints moved by {round(difference_joint_1, 5)} and {round(difference_joint_2, 5)} (deg)"
                self.logger.warn(log_message[iter])

            self.logger.info("Success")
            move_relative_request = MoveRelative.Request()
            move_relative_request.execute_movement = True
            move_relative_request.translation.z = 0.03


            move_relative_response: MoveRelative.Response = self.pm_robot_utils.client_move_laser_relative.call(move_relative_request)

            if not move_relative_response.success:
                raise PmRobotError(f"Endmove relative failed! {move_relative_response.message}")

            response.success = True
            response.message = "Iterative gonio right alignment completed successfully! \n Frames measured: " + ", ".join(request.frames_to_measure) + "\n"+ "\n".join(log_message.values())

        except PmRobotError as e:
            self.logger.error(f"Error occurred: {e}")
            response.success = False
            response.message = str(e)

        return response

    def iterative_align_gonio_left(self, request: pm_skill_srv.IterativeGonioAlign.Request, response:pm_skill_srv.IterativeGonioAlign.Response):
        
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

        if not self.pm_robot_utils.client_align_gonio_left.wait_for_service(1):
            self.logger.error(f"Client '{self.pm_robot_utils.client_align_gonio_left.srv_name}' not available!")
            response.success = False
            response.message = f"Client '{self.pm_robot_utils.client_align_gonio_left.srv_name}' not available!"
            return response

        # the request never changes
        align_request = pm_moveit_srv.AlignGonio.Request()
        align_request.execute_movement = True
        align_request.endeffector_frame_override = request.component_alignment_frame
        align_request.target_frame = request.target_alignment_frame

        iterations = request.num_iterations

        initial_approach = True

        log_message = {}

        for iter in range(iterations):
            run_number = iter + 1
            self.logger.info(f"STARTING RUN '{run_number}/{iterations}")

            for frame in request.frames_to_measure:
                self.logger.info(f"Measuring frame '{frame}'")
                measure_request = pm_skill_srv.CorrectFrameLaser.Request()
                measure_response = pm_skill_srv.CorrectFrameLaser.Response()
                measure_request.frame_name = frame
                measure_request.use_iterative_sensing = True

                if initial_approach:
                    if request.confocal_laser:
                        move_success, move_msg = self.node.gripper_skills.move_confocal_top_to_frame(frame, z_offset=0.02)
                    else:  
                        move_success, move_msg = self.node.gripper_skills.move_laser_to_frame(frame, z_offset=0.02)

                    if not move_success:
                        response.success = False
                        response.message = f"Moving laser to frame '{frame}' for initial approach failed: {move_msg}"
                        self.logger.error(response.message)
                        return response
                    initial_approach = False
                
                if request.confocal_laser:
                    measure_response = self.node.measurement_skills.correct_frame_with_confocal_top(measure_request, measure_response)
                else:
                    measure_response = self.node.measurement_skills.correct_frame_with_laser(measure_request, measure_response)

                if not measure_response.success:
                    response.success = False
                    response.message = f"Correcting frame '{frame}' failed! {measure_response.message}"
                    self.logger.error(response.message)
                    return response
                
            # calculating the angle difference 
            try:
                transform: Transform = self.pm_robot_utils.get_transform_for_frame(request.component_alignment_frame, 
                                                                        request.target_alignment_frame)
            except ValueError as e:
                self.logger.error(f"Error: {e}")
                self.logger.error(f"Frame '{frame}' does not seem to exist.")
                response.success = False
                response.message = f"Frame '{frame}' does not seem to exist: {e}"
                return response

            
            angles = R.from_quat([transform.rotation.x,
                                  transform.rotation.y,
                                  transform.rotation.z,
                                  transform.rotation.w]
                                 ).as_euler('xyz', degrees=True)

            self.logger.info(f"Initial deviation at {iter} - x: {angles[0]}, y: {angles[1]}, z: {angles[2]}")
            self.logger.info("Aligning goniometer!")

            joint_1_pre = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_LEFT_STAGE_1)
            joint_2_pre = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_LEFT_STAGE_2)

            align_response:pm_moveit_srv.AlignGonio.Response = self.pm_robot_utils.client_align_gonio_left.call(align_request)

            if not align_response.success:
                response.success = False
                response.message = "Aligning goniometer left failed! " + align_response.message
                self.logger.error(response.message)
                return response
            
            time.sleep(1)  # wait for the robot to settle

            joint_1_post = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_LEFT_STAGE_1)
            joint_2_post = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_LEFT_STAGE_2)

            difference_joint_1 = (joint_1_post - joint_1_pre)*180/np.pi
            difference_joint_2 = (joint_2_post - joint_2_pre)*180/np.pi

            log_message[iter] = f"Iteration {run_number}: Gonio joints moved by {round(difference_joint_1,5)} and {round(difference_joint_2,5)} (deg)"
            self.logger.warn(log_message[iter])

        self.logger.info("Success")
        move_relative_request = MoveRelative.Request()
        move_relative_request.execute_movement = True
        move_relative_request.translation.z = 0.01

        move_relative_response: MoveRelative.Response = self.pm_robot_utils.client_move_laser_relative.call(move_relative_request)

        if not move_relative_response.success:
            response.success = False
            response.message = "Endmove relative failed! " + move_relative_response.message
            self.logger.error(response.message)
            return response
        
        response.success = True
        response.message = "Iterative gonio right alignment completed successfully! \n Frames measured: " + ", ".join(request.frames_to_measure) + "\n"+ "\n".join(log_message.values())

        return response

    def align_gonio_right(self, endeffector_override:str, alignment_frame:str = PM_ROBOT_GRIPPER_FRAME)-> tuple[bool, str]:
        call_async = False

        if not self.pm_robot_utils.client_align_gonio_right.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/align_gonio_right' not available")
            return False, "Service '/pm_moveit_server/align_gonio_right' not available"
        
        req = pm_moveit_srv.AlignGonio.Request()
        req.target_frame = alignment_frame
        req.execute_movement = True
        req.endeffector_frame_override = endeffector_override

        if call_async:
            future = self.pm_robot_utils.client_align_gonio_right.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False, f"Service call failed: {future.exception()}"
            return future.result().success, future.result().message
        else:
            response:pm_moveit_srv.AlignGonio.Response = self.pm_robot_utils.client_align_gonio_right.call(req)
            return response.success, response.message
        
    def align_gonio_left(self, endeffector_override:str, alignment_frame:str = PM_ROBOT_GRIPPER_FRAME)-> tuple[bool, str]:
        call_async = False

        if not self.pm_robot_utils.client_align_gonio_left.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/align_gonio_left' not available")
            return False, "Service '/pm_moveit_server/align_gonio_left' not available"
        
        req = pm_moveit_srv.AlignGonio.Request()
        req.target_frame = alignment_frame
        req.execute_movement = True
        req.endeffector_frame_override = endeffector_override

        if call_async:
            future = self.pm_robot_utils.client_align_gonio_left.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False, f"Service call failed: {future.exception()}"
            return future.result().success, future.result().message
        else:
            response:pm_moveit_srv.AlignGonio.Response = self.pm_robot_utils.client_align_gonio_left.call(req)
            return response.success, response.message
