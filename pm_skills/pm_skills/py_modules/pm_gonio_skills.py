import time

import numpy as np
import rclpy
from geometry_msgs.msg import Transform
from scipy.spatial.transform import Rotation as R

import pm_moveit_interfaces.srv as pm_moveit_srv
import pm_skills_interfaces.srv as pm_skill_srv
from assembly_scene_publisher.py_modules.scene_errors import (
    AssemblyFrameNotFoundError,
    AssemblyInstructionNotFoundError,
    ComponentNotFoundError,
    GrippingFrameNotFoundError,
    RefFrameNotFoundError,
    TargetFrameNotFoundError,
)
from pm_moveit_interfaces.srv import MoveRelative
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_skills.py_modules.pm_skill_domain import PmSkillDomain



class PmGonioSkills(PmSkillDomain):
    def align_component_to_gripper_callback(
        self,
        request: pm_skill_srv.AlignComponentToGripper.Request,
        response: pm_skill_srv.AlignComponentToGripper.Response,
    ):
        """Align a component's gripping frame using the goniometer carrying it."""
        return self._align_component_to_gripper(request, response, execute_movement=True)

    def check_gripper_is_alignable_callback(
        self,
        request: pm_skill_srv.CheckGripperIsAlignable.Request,
        response: pm_skill_srv.CheckGripperIsAlignable.Response,
    ):
        """Check the component-to-gripper alignment without executing movement."""
        return self._align_component_to_gripper(request, response, execute_movement=False)

    def _align_component_to_gripper(self, request, response, execute_movement: bool):
        response.success = False
        try:
            self.pm_robot_utils.wait_for_initial_scene_update()
            analyzer = self.pm_robot_utils.assembly_scene_analyzer
            if not analyzer.check_object_exists(request.component_name):
                raise PmRobotError(f"Object '{request.component_name}' does not exist!")

            if analyzer.is_object_on_gonio_left(request.component_name):
                align = self.align_gonio_left
                side = 'left'
            elif analyzer.is_object_on_gonio_right(request.component_name):
                align = self.align_gonio_right
                side = 'right'
            else:
                raise PmRobotError(
                    f"Component '{request.component_name}' is not on either goniometer.")

            gripping_frame = analyzer.get_gripping_frame_of_component(request.component_name)
            success, message = align(
                endeffector_override=gripping_frame,
                alignment_frame=self.PM_ROBOT_GRIPPER_FRAME,
                execute_movement=execute_movement,
            )
            if not success:
                raise PmRobotError(
                    f"Alignment of component '{request.component_name}' on the {side} "
                    f'goniometer failed: {message}')

            response.success = True
            if execute_movement:
                response.message = (
                    f"Component '{request.component_name}' aligned to gripper frame "
                    f"'{self.PM_ROBOT_GRIPPER_FRAME}' using the {side} goniometer.")
            else:
                response.message = (
                    f"Alignment of component '{request.component_name}' to gripper frame "
                    f"'{self.PM_ROBOT_GRIPPER_FRAME}' can be planned using the {side} "
                    'goniometer. No movement was executed.')
            self.logger.info(response.message)
        except (PmRobotError, ComponentNotFoundError, GrippingFrameNotFoundError,
                RefFrameNotFoundError, RuntimeError) as exc:
            response.message = str(exc)
            self.logger.error(response.message)
        return response

    def iterative_align_gonio_right(self, request: pm_skill_srv.IterativeGonioAlign.Request, response: pm_skill_srv.IterativeGonioAlign.Response):
        return self._iterative_align_gonio_callback('right', request, response)

    def iterative_align_gonio_left(self, request: pm_skill_srv.IterativeGonioAlign.Request, response: pm_skill_srv.IterativeGonioAlign.Response):
        return self._iterative_align_gonio_callback('left', request, response)

    def _iterative_align_gonio_callback(self, side: str, request, response):
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()
        try:
            (response.message, response.gonio_stage_1_moved_deg,
             response.gonio_stage_2_moved_deg) = self._run_iterative_gonio_alignment(
                side=side,
                component_alignment_frame=request.component_alignment_frame,
                target_alignment_frame=request.target_alignment_frame,
                frames_to_measure=list(request.frames_to_measure),
                num_iterations=request.num_iterations,
                confocal_laser=request.confocal_laser,
            )
            response.success = True
        except PmRobotError as e:
            self.logger.error(f"Error occurred: {e}")
            response.success = False
            response.message = str(e)
        return response

    def iterative_align_component_to_gripper_laser_callback(
        self,
        request: pm_skill_srv.IterativeAlignComponentToGripper.Request,
        response: pm_skill_srv.IterativeAlignComponentToGripper.Response,
    ):
        """Iteratively align a component's gripping frame to the gripper, measuring with the laser."""
        return self._iterative_align_component_to_gripper(request, response, confocal_laser=False)

    def iterative_align_component_to_gripper_confocal_callback(
        self,
        request: pm_skill_srv.IterativeAlignComponentToGripper.Request,
        response: pm_skill_srv.IterativeAlignComponentToGripper.Response,
    ):
        """Iteratively align a component's gripping frame to the gripper, measuring with the confocal top sensor."""
        return self._iterative_align_component_to_gripper(request, response, confocal_laser=True)

    def _iterative_align_component_to_gripper(self, request, response, confocal_laser: bool):
        """Measure the laser frames that define the gripping frame and align it with the gonio.

        The frames to measure are the gripping frame itself (if it is a laser
        frame) and its recursive active constraint dependencies with a laser
        role. All of them must move with the same goniometer as the component,
        otherwise measuring them cannot observe the alignment.
        """
        response.success = False
        response.frames_measured = []
        sensor = 'confocal' if confocal_laser else 'laser'
        try:
            self.pm_robot_utils.wait_for_initial_scene_update()
            analyzer = self.pm_robot_utils.assembly_scene_analyzer
            component = request.component_name
            if not analyzer.check_object_exists(component):
                raise PmRobotError(f"Object '{component}' does not exist!")

            side = self._gonio_side_of_object(component)
            if side is None:
                raise PmRobotError(f"Component '{component}' is not on either goniometer.")

            gripping_frame = analyzer.get_gripping_frame_of_component(component)
            laser_frames = self._laser_frames_moving_with_gonio(
                gripping_frame, analyzer.get_laser_frames_for_gripping_frame(gripping_frame),
                component, side, 'Gripping frame')

            num_iterations = self._checked_num_iterations(request.num_iterations)

            message, stage_1_moved, stage_2_moved = self._run_iterative_gonio_alignment(
                side=side,
                component_alignment_frame=gripping_frame,
                target_alignment_frame=self.PM_ROBOT_GRIPPER_FRAME,
                frames_to_measure=laser_frames,
                num_iterations=num_iterations,
                confocal_laser=confocal_laser,
            )
            response.success = True
            response.frames_measured = laser_frames
            response.gonio_stage_1_moved_deg = stage_1_moved
            response.gonio_stage_2_moved_deg = stage_2_moved
            response.message = (
                f"Component '{component}' gripping frame '{gripping_frame}' aligned to "
                f"'{self.PM_ROBOT_GRIPPER_FRAME}' ({sensor}). " + message)
            self.logger.info(response.message)
        except (PmRobotError, ComponentNotFoundError, GrippingFrameNotFoundError,
                RefFrameNotFoundError, ValueError, RuntimeError) as exc:
            response.message = str(exc)
            self.logger.error(response.message)
        return response

    def iterative_align_assembly_component_to_target_laser_callback(
        self,
        request: pm_skill_srv.IterativeAlignAssemblyComponentToTarget.Request,
        response: pm_skill_srv.IterativeAlignAssemblyComponentToTarget.Response,
    ):
        """Iteratively align the target frame to the component's assembly frame, measuring with the laser."""
        return self._iterative_align_assembly_component_to_target(
            request, response, confocal_laser=False)

    def iterative_align_assembly_component_to_target_confocal_callback(
        self,
        request: pm_skill_srv.IterativeAlignAssemblyComponentToTarget.Request,
        response: pm_skill_srv.IterativeAlignAssemblyComponentToTarget.Response,
    ):
        """Iteratively align the target frame to the component's assembly frame, measuring with the confocal top sensor."""
        return self._iterative_align_assembly_component_to_target(
            request, response, confocal_laser=True)

    def _iterative_align_assembly_component_to_target(self, request, response, confocal_laser: bool):
        """Align the target component's gonio so its target frame matches the assembly frame.

        The assembly component may be gripped or lie anywhere else, but not on
        the same goniometer as the target component. In every iteration the
        laser frames the target frame depends on are measured, then the
        assembly instruction is recalculated so the target frame reflects the
        measurement, then the goniometer is aligned.
        """
        response.success = False
        response.frames_measured = []
        sensor = 'confocal' if confocal_laser else 'laser'
        try:
            self.pm_robot_utils.wait_for_initial_scene_update()
            analyzer = self.pm_robot_utils.assembly_scene_analyzer
            component = request.component_name
            if not analyzer.check_object_exists(component):
                raise PmRobotError(f"Object '{component}' does not exist!")
            num_iterations = self._checked_num_iterations(request.num_iterations)

            assembly_frame = analyzer.get_assembly_frame_for_component(component)
            target_frame = analyzer.get_target_frame_for_assembly_frame(assembly_frame)
            target_component = analyzer.get_component_for_frame_name(target_frame)
            response.assembly_frame = assembly_frame
            response.target_frame = target_frame
            response.target_component = target_component

            if target_component == component:
                raise PmRobotError(
                    f"Target frame '{target_frame}' belongs to the assembly component "
                    f"'{component}' itself.")

            side = self._gonio_side_of_object(target_component)
            if side is None:
                raise PmRobotError(
                    f"Target component '{target_component}' is not on either goniometer.")
            if self._gonio_side_of_object(component) == side:
                raise PmRobotError(
                    f"Assembly component '{component}' and target component "
                    f"'{target_component}' are both on the {side} goniometer. "
                    'Aligning one to the other is not possible.')

            laser_frames = self._laser_frames_moving_with_gonio(
                target_frame, analyzer.get_laser_frames_for_target_frame(target_frame),
                target_component, side, 'Target frame')

            unmeasured = [frame.frame_name for frame, measured in
                          analyzer.get_measurement_frames_for_assembly_frame(assembly_frame)
                          if measured is not True]
            if unmeasured:
                self.logger.warn(
                    f"Assembly frame '{assembly_frame}' depends on unmeasured frames: "
                    f"{', '.join(unmeasured)}. Its orientation may be inaccurate.")

            instruction = analyzer.get_assembly_instruction(
                assembly_component=component, target_component=target_component)

            def recalculate_instruction():
                # TODO: remove once the scene recalculates instructions itself
                # after a relevant frame has been measured.
                self.logger.info(f"Recalculating assembly instruction '{instruction.id}'")
                self.pm_robot_utils.recalculate_assembly_instruction(instruction_id=instruction.id)

            message, stage_1_moved, stage_2_moved = self._run_iterative_gonio_alignment(
                side=side,
                component_alignment_frame=target_frame,
                target_alignment_frame=assembly_frame,
                frames_to_measure=laser_frames,
                num_iterations=num_iterations,
                confocal_laser=confocal_laser,
                after_measurement=recalculate_instruction,
            )
            response.success = True
            response.frames_measured = laser_frames
            response.gonio_stage_1_moved_deg = stage_1_moved
            response.gonio_stage_2_moved_deg = stage_2_moved
            response.message = (
                f"Target frame '{target_frame}' of '{target_component}' aligned to assembly "
                f"frame '{assembly_frame}' of '{component}' ({sensor}). " + message)
            self.logger.info(response.message)
        except (PmRobotError, ComponentNotFoundError, AssemblyFrameNotFoundError,
                TargetFrameNotFoundError, AssemblyInstructionNotFoundError,
                RefFrameNotFoundError, ValueError, RuntimeError) as exc:
            response.message = str(exc)
            self.logger.error(response.message)
        return response

    @staticmethod
    def _checked_num_iterations(num_iterations: int) -> int:
        if num_iterations < 1:
            raise PmRobotError(f'num_iterations must be at least 1, got {num_iterations}.')
        return num_iterations

    def _laser_frames_moving_with_gonio(self, frame: str, laser_frames, component: str,
                                        side: str, role: str) -> list:
        """Return the laser frame names; fail unless all of them move with the gonio."""
        analyzer = self.pm_robot_utils.assembly_scene_analyzer
        names = [laser_frame.frame_name for laser_frame, _ in laser_frames]
        if not names:
            raise PmRobotError(
                f"{role} '{frame}' of component '{component}' does not depend on any laser "
                'frame. Its orientation cannot be measured.')

        not_on_gonio = []
        for name in names:
            owner = analyzer.get_component_for_frame_name(name)
            if owner != component and self._gonio_side_of_object(owner) != side:
                not_on_gonio.append(f"{name} ({owner})")
        if not_on_gonio:
            raise PmRobotError(
                f"Laser frames of {role.lower()} '{frame}' are not on the {side} goniometer: "
                f"{', '.join(not_on_gonio)}. They do not move with the alignment.")

        if len(names) < 3:
            self.logger.warn(
                f"{role} '{frame}' depends on only {len(names)} laser frame(s). "
                'Fewer than 3 may not determine both gonio angles.')
        return names

    def _gonio_side_of_object(self, object_name: str):
        analyzer = self.pm_robot_utils.assembly_scene_analyzer
        if analyzer.is_object_on_gonio_left(object_name):
            return 'left'
        if analyzer.is_object_on_gonio_right(object_name):
            return 'right'
        return None

    def _gonio_config(self, side: str):
        """Return (align client, stage joint 1, stage joint 2, final lift in m) for a side."""
        utils = self.pm_robot_utils
        if side == 'right':
            return (utils.client_align_gonio_right, utils.GONIO_RIGHT_STAGE_1,
                    utils.GONIO_RIGHT_STAGE_2, 0.03)
        if side == 'left':
            return (utils.client_align_gonio_left, utils.GONIO_LEFT_STAGE_1,
                    utils.GONIO_LEFT_STAGE_2, 0.01)
        raise PmRobotError(f"Unknown goniometer side '{side}'.")

    def _run_iterative_gonio_alignment(
        self, side: str, component_alignment_frame: str, target_alignment_frame: str,
        frames_to_measure: list, num_iterations: int, confocal_laser: bool,
        after_measurement=None,
    ) -> tuple[str, list, list]:
        """Measure frames and align the goniometer, repeatedly. Raise PmRobotError on failure.

        after_measurement, if given, is called once per iteration after all
        frames were measured and before the goniometer is aligned.

        Return the summary message and, per iteration, how far gonio stage 1
        and stage 2 moved (deg).
        """
        client, stage_1, stage_2, end_lift = self._gonio_config(side)
        if not client.wait_for_service(1):
            raise PmRobotError(f"Client '{client.srv_name}' not available!")

        # the request never changes
        align_request = pm_moveit_srv.AlignGonio.Request()
        align_request.execute_movement = True
        align_request.endeffector_frame_override = component_alignment_frame
        align_request.target_frame = target_alignment_frame

        log_message = []
        stage_1_moved = []
        stage_2_moved = []
        initial_approach = True
        for iteration in range(num_iterations):
            run_number = iteration + 1
            self.logger.info(f"STARTING RUN '{run_number}/{num_iterations}")

            for frame in frames_to_measure:
                self.logger.info(f"Measuring frame '{frame}'")
                measure_request = pm_skill_srv.CorrectFrameLaser.Request()
                measure_response = pm_skill_srv.CorrectFrameLaser.Response()
                measure_request.frame_name = frame
                measure_request.use_iterative_sensing = True

                if initial_approach:
                    if confocal_laser:
                        move_success, move_msg = self.node.gripper_skills.move_confocal_top_to_frame(frame, z_offset=0.02)
                    else:
                        move_success, move_msg = self.node.gripper_skills.move_laser_to_frame(frame, z_offset=0.02)
                    if not move_success:
                        raise PmRobotError(
                            f"Moving laser to frame '{frame}' for initial approach failed: {move_msg}")
                    initial_approach = False

                if confocal_laser:
                    measure_response = self.node.measurement_skills.correct_frame_with_confocal_top(measure_request, measure_response)
                else:
                    measure_response = self.node.measurement_skills.correct_frame_with_laser(measure_request, measure_response)

                if not measure_response.success:
                    raise PmRobotError(f"Correcting frame '{frame}' failed! {measure_response.message}")

            if after_measurement is not None:
                after_measurement()

            # calculating the angle difference
            try:
                transform: Transform = self.pm_robot_utils.get_transform_for_frame(
                    component_alignment_frame, target_alignment_frame)
            except ValueError as e:
                raise PmRobotError(
                    f"Transform from '{component_alignment_frame}' to "
                    f"'{target_alignment_frame}' not available: {e}") from e

            angles = R.from_quat([transform.rotation.x,
                                  transform.rotation.y,
                                  transform.rotation.z,
                                  transform.rotation.w]
                                 ).as_euler('xyz', degrees=True)

            self.logger.info(f"Deviation at iteration {run_number} - x: {angles[0]}, y: {angles[1]}, z: {angles[2]}")
            self.logger.info(f"Aligning goniometer {side}!")

            joint_1_pre = self.pm_robot_utils.get_current_joint_state(stage_1)
            joint_2_pre = self.pm_robot_utils.get_current_joint_state(stage_2)

            align_response: pm_moveit_srv.AlignGonio.Response = client.call(align_request)
            if align_response is None or not align_response.success:
                detail = align_response.message if align_response is not None else 'no response'
                raise PmRobotError(f"Aligning goniometer {side} failed! {detail}")

            time.sleep(1)  # wait for the robot to settle

            joint_1_post = self.pm_robot_utils.get_current_joint_state(stage_1)
            joint_2_post = self.pm_robot_utils.get_current_joint_state(stage_2)

            difference_joint_1 = (joint_1_post - joint_1_pre) * 180 / np.pi
            difference_joint_2 = (joint_2_post - joint_2_pre) * 180 / np.pi

            stage_1_moved.append(float(difference_joint_1))
            stage_2_moved.append(float(difference_joint_2))
            log_message.append(
                f"Iteration {run_number}: Gonio joints moved by {round(difference_joint_1, 5)} "
                f"and {round(difference_joint_2, 5)} (deg)")
            self.logger.warn(log_message[-1])

        move_relative_request = MoveRelative.Request()
        move_relative_request.execute_movement = True
        move_relative_request.translation.z = end_lift
        move_relative_response: MoveRelative.Response = self.pm_robot_utils.client_move_laser_relative.call(move_relative_request)
        if not move_relative_response.success:
            raise PmRobotError(f"Endmove relative failed! {move_relative_response.message}")

        message = (f"Iterative gonio {side} alignment completed successfully! \n Frames measured: "
                   + ", ".join(frames_to_measure) + "\n" + "\n".join(log_message))
        return message, stage_1_moved, stage_2_moved

    def align_gonio_right(
        self, endeffector_override: str, alignment_frame: str = None,
        execute_movement: bool = True,
    ) -> tuple[bool, str]:
        call_async = False
        if alignment_frame is None:
            alignment_frame = self.PM_ROBOT_GRIPPER_FRAME

        if not self.pm_robot_utils.client_align_gonio_right.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/align_gonio_right' not available")
            return False, "Service '/pm_moveit_server/align_gonio_right' not available"
        
        req = pm_moveit_srv.AlignGonio.Request()
        req.target_frame = alignment_frame
        req.execute_movement = execute_movement
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
            if response is None:
                return False, "Service '/pm_moveit_server/align_gonio_right' returned no response"
            return response.success, response.message
        
    def align_gonio_left(
        self, endeffector_override: str, alignment_frame: str = None,
        execute_movement: bool = True,
    ) -> tuple[bool, str]:
        call_async = False
        if alignment_frame is None:
            alignment_frame = self.PM_ROBOT_GRIPPER_FRAME

        if not self.pm_robot_utils.client_align_gonio_left.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/align_gonio_left' not available")
            return False, "Service '/pm_moveit_server/align_gonio_left' not available"
        
        req = pm_moveit_srv.AlignGonio.Request()
        req.target_frame = alignment_frame
        req.execute_movement = execute_movement
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
            if response is None:
                return False, "Service '/pm_moveit_server/align_gonio_left' returned no response"
            return response.success, response.message
