import math
import time
from datetime import datetime

import numpy as np

import pm_msgs.srv as pm_msg_srv
import pm_skills_interfaces.srv as pm_skill_srv
from assembly_scene_publisher.py_modules.tf_functions import get_transform_for_frame_in_world
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_skills.py_modules.pm_skill_domain import PmSkillDomain


class PmForceSkills(PmSkillDomain):
    def force_sensing_move_callback(self, request:pm_msg_srv.GripperForceMove.Request, response:pm_msg_srv.GripperForceMove.Response):

        self.get_logger().info('Received ForceSensingMove request.')

        self.pm_robot_utils.set_force_sensor_bias()
        time.sleep(2.0)
        
        # Check force sensor values after bias reset
        current_force_values = self.pm_robot_utils._current_force_sensor_data.data
        self.get_logger().info(f'Force sensor values after bias reset: {current_force_values}')
        
        force_thrshold = [abs(request.max_f_xyz[0]), abs(request.max_f_xyz[1]), abs(request.max_f_xyz[2])]

        if not self.pm_robot_utils.is_unity_running() and (abs(current_force_values[0]) > force_thrshold[0] or abs(current_force_values[1]) > force_thrshold[1] or abs(current_force_values[2]) > force_thrshold[2]):
            response.error = f'Initial force values exceed threshold of {force_thrshold} N: X={current_force_values[0]}, Y={current_force_values[1]}, Z={current_force_values[2]}'
            raise PmRobotError(f"{response.error}")
        
        # Validate the request parameters. If any max force is > than 10, set threshold_exceeded to True and return failure.
        threshold_value = 10.0  # N
        max_step_size = 100  # micrometers
        max_steps = 1000  # maximum number of steps


        if not self.pm_robot_utils.is_unity_running() and (abs(request.max_f_xyz[0]) > threshold_value or abs(request.max_f_xyz[1]) > threshold_value or abs(request.max_f_xyz[2]) > threshold_value):
            self.get_logger().error('Max force exceeded the threshold of 10N.')
            response.success = False
            response.error = 'Max force exceeded the threshold of 10N.'
            return response
        
        if request.step_size > max_step_size:
            self.get_logger().error(f'Step size {request.step_size} micrometers exceeds the maximum allowed step size of {max_step_size} micrometers.')
            response.success = False
            response.error = f'Step size exceeds the maximum allowed step size of {max_step_size} micrometers.'
            return response

        step_size = request.step_size*1e-6  # Convert step size from micrometers to meters

        # start_x = request.initial_joints_xyzt[0]
        # start_y = request.initial_joints_xyzt[1]
        # start_z = request.initial_joints_xyzt[2]

        start_x = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.X_Axis_JOINT_NAME)
        start_y = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Y_Axis_JOINT_NAME)
        start_z = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)

        target_x = request.target_joints_xyz[0]
        target_y = request.target_joints_xyz[1]
        target_z = request.target_joints_xyz[2]
        current_position = [start_x, start_y, start_z]
        target_position = [target_x, target_y, target_z]

        length_x = target_x - start_x
        length_y = target_y - start_y
        length_z = target_z - start_z
        # Calculate the length of the vector from start to target position
        length = math.sqrt(length_x**2 + length_y**2 + length_z**2)

        if length == 0.0:
            self.get_logger().info('Already at target position.')
            response.success = True
            response.completed = False
            return response

        if (length/ step_size) > max_steps:
            self.get_logger().error(f'The distance to the target position is too large. The maximum number of steps is {max_steps}.')
            response.success = False
            response.error = f'The distance to the target position is too large. The maximum number of steps is {max_steps}.'
            return response

        self.get_logger().info(f'Current force sensor data: {self.pm_robot_utils._current_force_sensor_data.data}')

        step_size_x = step_size * length_x / length
        step_size_y = step_size * length_y / length
        step_size_z = step_size * length_z / length
        counter = 0
        while current_position != target_position:
            # Check if the force sensor data exceeds the thresholds
            for i, (force, max_force, axis) in enumerate(zip(self.pm_robot_utils._current_force_sensor_data.data, [abs(request.max_f_xyz[0]), abs(request.max_f_xyz[1]), abs(request.max_f_xyz[2])], ['X', 'Y', 'Z'])):
                if abs(force) > max_force:
                    self.get_logger().warn(f'Force in {axis} direction exceeded threshold: {force:.3f} > {max_force:.3f}')
                    response.success = True
                    response.completed = True
                    return response

            step_target = [
                current_position[0] + step_size_x,
                current_position[1] + step_size_y,
                current_position[2] + step_size_z,
            ]

            self.get_logger().info(f'Moving to position: {step_target}. Step {counter}. With step size: {[round(step_size_x*1e6, 5), round(step_size_y*1e6, 5), round(step_size_z*1e6, 5)]} um')

            # Move to the next step position
            success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(
                step_target[0],
                step_target[1],
                step_target[2],
                time=1.0
            )

            if not success:
                self.get_logger().error('Failed to move to the next position.')
                response.success = False
                response.error = 'Failed to move to the next position.'
                return response

            # Update the current position
            current_position = step_target

            # calculate the distance to the target position
            distance_to_target = math.sqrt(
                (target_position[0] - current_position[0])**2 +
                (target_position[1] - current_position[1])**2 +
                (target_position[2] - current_position[2])**2
            )
            if distance_to_target < step_size:
                self.get_logger().info('Reached the target position.')
                break
            counter += 1

        self.get_logger().info('Target position reached. Nothing found.')
        response.success = True
        response.completed = False
        response.error = 'Target position reached. Nothing found.'
        return response
    
    
    def force_scan_callback(self, request: pm_skill_srv.ForceScan.Request, response: pm_skill_srv.ForceScan.Response):
        """
        Request:
            - target_frame (string):    Frame des Bauteils das gemessen werden soll
            - direction (Vector3):      Richtung der Bewegung in Weltkoordinaten (wird normalisiert)
            - max_force (Vector3):      Kraftschwellwerte in N fuer X, Y, Z
            - step_size (float32):      Schrittgroesse in Mikrometern

        Response:
            - success (bool):           True wenn Kraft erkannt wurde
            - message (string):         Statusmeldung
            - detected_position (Pose): Weltkoordinaten des TCPs beim Kraftkontakt
        """




        try:

            for i in range (30):
                scan_number = i+1
                self.get_logger().info(f"Starting force scan {scan_number} of 25.")

                timestamp = datetime.now().isoformat()

                piezo_force_request = pm_msg_srv.GripperGetForces.Request()
                frame_position_gripper = get_transform_for_frame_in_world(
                    self.PM_ROBOT_GRIPPER_FRAME,
                    self.tf_buffer,
                    self.get_logger()
                )

                PiezoGripperForce: pm_msg_srv.GripperGetForces.Response = self.smart_gripper_force_client.call(piezo_force_request)

                self.logger.info(f"Current gripper forces: X={PiezoGripperForce.fx}, Y={PiezoGripperForce.fy}, Z={PiezoGripperForce.fz}")

                # raise NotImplementedError("Force scan skill is still in development. The current implementation is a placeholder and may not work as expected.")


                # Schritt 1: Richtungsvektor pruefen und normalisieren
                direction_length = math.sqrt(
                    request.direction.x**2 +
                    request.direction.y**2 +
                    request.direction.z**2
                )
                if direction_length == 0.0:
                    response.success = False
                    response.message = 'Direction vector is zero!'
                    self.get_logger().error(response.message)
                    return response

                direction_normalized = [
                    request.direction.x / direction_length,
                    request.direction.y / direction_length,
                    request.direction.z / direction_length
                ]

                # Schritt 2: Schrittgroesse und Kraftgrenzwert pruefen
                max_step_size_um = 100
                if request.step_size > max_step_size_um:
                    response.success = False
                    response.message = (
                        f'Step size {request.step_size} um exceeds '
                        f'the maximum of {max_step_size_um} um.'
                    )
                    self.get_logger().error(response.message)
                    return response

                max_force_limit = 400.0
                if not self.pm_robot_utils.is_unity_running():
                    if (abs(request.max_force.x) > max_force_limit or
                        abs(request.max_force.y) > max_force_limit or
                        abs(request.max_force.z) > max_force_limit):
                        response.success = False
                        response.message = f'Force threshold {max_force_limit}, pick lower force values'
                        self.get_logger().error(response.message)
                        return response

                # Schritt 3: Zur Anfahrposition fahren
                # (Offset von 20 mm entgegen der Scanrichtung je Achse, Vorzeichen
                # ergibt sich aus der Richtung des normierten Vektors)
                # offset = [
                #     -0.002 if axis > 0
                #     else 0.002 if axis < 0
                #     else 0.0
                #     for axis in direction_normalized
                # ]

                self.get_logger().info('target frame: ' + request.target_frame)
                frame_target = get_transform_for_frame_in_world(
                    request.target_frame,
                    self.tf_buffer,
                    self.get_logger()
                )
                frame_position_target = [
                    frame_target.transform.translation.x,
                    frame_target.transform.translation.y,
                    frame_target.transform.translation.z
                ]

                success, message = self.node.gripper_skills.move_gripper_to_frame(request.target_frame, x_offset=-0.0021, y_offset=0.0, z_offset=-0.0015) 
                if not success:
                    response.success = False
                    response.message = f'could not move to target frame: {message}'
                    self.get_logger().error(response.message)
                    return response

                # Schritt 4: Tatsaechliche Startposition ermitteln
                start_position = [
                    self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.X_Axis_JOINT_NAME),
                    self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Y_Axis_JOINT_NAME),
                    self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)
                ]
                current_position = list(start_position)


                self.get_logger().info(
                    f'Start position: '
                    f'X={round(start_position[0]*1e3, 3)} mm, '
                    f'Y={round(start_position[1]*1e3, 3)} mm, '
                    f'Z={round(start_position[2]*1e3, 3)} mm'
                )
                
                # Schritt 5: Kraftsensor nullen (Vermeidung eines Messbias)
                self.pm_robot_utils.set_force_sensor_bias()
                time.sleep(2.0)

                # Schritt 6: Anfangskraefte pruefen
                if not self.pm_robot_utils.is_unity_running():
                    force_response = self.smart_gripper_force_client.call(piezo_force_request)

                    current_force_values = [
                        force_response.fx,
                        force_response.fy,
                        force_response.fz
                    ]
                else:
                    current_force_values = self.pm_robot_utils._current_force_sensor_data.data[:3]
                self.get_logger().info(f'Force sensor initial values: {current_force_values}')

                force_threshold = [
                    abs(request.max_force.x),
                    abs(request.max_force.y),
                    abs(request.max_force.z)
                ]

                if not self.pm_robot_utils.is_unity_running():
                    if (abs(current_force_values[0]) > force_threshold[0] or
                        abs(current_force_values[1]) > force_threshold[1] or
                        abs(current_force_values[2]) > force_threshold[2]):
                        response.success = False
                        response.message = (
                            f'current force values exceed threshold {force_threshold} before starting the scan: '
                            f'X={current_force_values[0]:.3f}, '
                            f'Y={current_force_values[1]:.3f}, '
                            f'Z={current_force_values[2]:.3f}'
                        )
                        self.get_logger().error(response.message)
                        return response

                # Schritt 7: Schrittgroessen in Meter umrechnen
                step_size_m_min = request.step_size * 1e-6

                # Grobe Suchschrittgroesse: 0,5 mm pro Schritt, unabhaengig von der
                # angeforderten Ziel-Schrittgroesse. Die Verfeinerung in Richtung
                # der Ziel-Schrittgroesse erfolgt erst im Zustand REFINE.
                step_size_m_search = 0.0002

                # Schritt 8: Scan-Schleife (Zustandsmaschine)

                max_scan_distance_m = 0.025
                travelled_distance_m = 0.0

            
                last_valid_position = current_position.copy()

                contact_position = None        

                detected_position = None

                # contact_detected = False

                scan_start_time = time.time()
                search_iterations = 0
                refine_iterations = 0
                contact_force = [math.nan, math.nan, math.nan]
                frame_position_contact = [math.nan, math.nan, math.nan]

                hard_contact_threshold = [150.0, 
                                          150.0, 
                                          150.0
                ]

                step_counter = 0
                                          
                
                state = "SEARCH"

                while travelled_distance_m < max_scan_distance_m:

                    if state == "SEARCH":            
                        search_iterations += 1        
                        # Schrittweise Annaeherung in Richtung des normierten Vektors

                        current_position[0] += step_size_m_search * direction_normalized[0]
                        current_position[1] += step_size_m_search * direction_normalized[1]
                        current_position[2] += step_size_m_search * direction_normalized[2]

                        self.pm_robot_utils.send_xyz_trajectory_goal_absolut(
                            *current_position,
                            time=0.05
                        )

                        time.sleep(0.5)

                        current_position = [
                            self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.X_Axis_JOINT_NAME),
                            self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Y_Axis_JOINT_NAME),
                            self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)
                        ]

                        if not self.pm_robot_utils.is_unity_running():
                            force_response = self.smart_gripper_force_client.call(piezo_force_request)
                            force_values = [
                                force_response.fx,
                                force_response.fy,
                                force_response.fz
                                ]
                        else:
                            force_values = self.pm_robot_utils._current_force_sensor_data.data[:3]

                        step_counter += 1
                        self.csv_force_scan_step(
                            scan_number,
                            request,
                            step_counter, 
                            "SEARCH",
                            step_size_m_search * 1e6,
                            force_values,
                            current_position,
                            datetime.now().isoformat()
                        )

                        hard_contact = any(
                            abs(force_values[i]) > hard_contact_threshold[i]
                            for i in range(3)
                        )

                        possible_contact = any(
                            abs(force_values[i]) > force_threshold[i]
                            for i in range(3)
                        )

                        if hard_contact:

                            contact_position = [
                                self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.X_Axis_JOINT_NAME),
                                self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Y_Axis_JOINT_NAME),
                                self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)
                            ]

                            frame_position_contact = [
                                frame_position_gripper.transform.translation.x,
                                frame_position_gripper.transform.translation.y,
                                frame_position_gripper.transform.translation.z
                            ]
                            
                            if not self.pm_robot_utils.is_unity_running():
                                force_response = self.smart_gripper_force_client.call(piezo_force_request)

                                contact_force = [
                                    force_response.fx,
                                    force_response.fy,
                                    force_response.fz
                                ]
                            else:
                                contact_force = self.pm_robot_utils._current_force_sensor_data.data[:3]

                            step_counter += 1
                            self.csv_force_scan_step(
                                scan_number,
                                request,
                                step_counter, 
                                "CONTACT",
                                step_size_m_search * 1e6,
                                contact_force,
                                contact_position,
                                datetime.now().isoformat()
                            )

                            self.get_logger().info(f'force at current contact: {contact_force}')

                            state = "CONTACT"
                            continue

                        elif possible_contact:
                            state = "CONTACT_CHECK"
                            continue

                        else:
                            last_valid_position = current_position.copy()
                            travelled_distance_m += step_size_m_search
                            continue

                    elif state == "CONTACT_CHECK":

                        check_values = []

                        # Mehrfach messen ohne den Roboter zu bewegen
                        for _ in range(10):

                            if not self.pm_robot_utils.is_unity_running():
                                force_response = self.smart_gripper_force_client.call(
                                    piezo_force_request
                                )
                                force = [
                                    force_response.fx,
                                    force_response.fy,
                                    force_response.fz
                                ]
                            else:
                                force = self.pm_robot_utils._current_force_sensor_data.data[:3]

                            check_values.append(force)

                            step_counter += 1
                            self.csv_force_scan_step(
                                scan_number,
                                request,
                                step_counter,
                                "CONTACT_CHECK",
                                step_size_m_search * 1e6,
                                force,
                                current_position,
                                datetime.now().isoformat()
                            )

                            time.sleep(0.5)

                        mean_force = [
                            np.mean([v[0] for v in check_values]),
                            np.mean([v[1] for v in check_values]),
                            np.mean([v[2] for v in check_values]),
                        ]

                        confirmed_contact = any(
                            abs(mean_force[i]) > force_threshold[i]
                            for i in range(3)
                        )

                        if confirmed_contact:

                            contact_position = current_position.copy()
                            contact_force = mean_force.copy()

                            step_counter += 1
                            self.csv_force_scan_step(
                                scan_number,
                                request,
                                step_counter,
                                "CONTACT_CONFIRMED",
                                step_size_m_search * 1e6,
                                contact_force,
                                contact_position,
                                datetime.now().isoformat()
                            )

                            self.get_logger().info(
                                f"Contact confirmed: {contact_force}"
                            )

                            state = "CONTACT"

                        else:

                            step_counter += 1
                            self.csv_force_scan_step(
                                scan_number,
                                request,
                                step_counter,
                                "CONTACT_REJECTED",
                                step_size_m_search * 1e6,
                                mean_force,
                                current_position,
                                datetime.now().isoformat()
                            )

                            # Kontakt war nur Rauschen -> weitersuchen
                            last_valid_position = current_position.copy()
                            travelled_distance_m += step_size_m_search

                            state = "SEARCH"

                        continue

                    elif state == "CONTACT":                    
                        # Zurueck zur letzten kontaktfreien Position fahren
                        self.pm_robot_utils.send_xyz_trajectory_goal_absolut(*last_valid_position,
                            time=0.05
                        )

                        travelled_distance_m -= step_size_m_search

                        time.sleep(0.5)

                        current_position = [
                            self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.X_Axis_JOINT_NAME),
                            self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Y_Axis_JOINT_NAME),
                            self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)
                        ]

                        if not self.pm_robot_utils.is_unity_running():
                            force_response = self.smart_gripper_force_client.call(piezo_force_request)

                            current_force = [
                                force_response.fx,
                                force_response.fy,
                                force_response.fz
                            ]
                        else:
                            current_force = self.pm_robot_utils._current_force_sensor_data.data[:3]

                        step_counter += 1
                        self.csv_force_scan_step(   
                            scan_number,
                            request,
                            step_counter, 
                            "moved back",
                            step_size_m_search * 1e6,
                            current_force,
                            current_position,
                            datetime.now().isoformat()
                        )

                        if step_size_m_search <= step_size_m_min:
                            # Ziel-Schrittgroesse bereits erreicht -> Scan beenden
                            state = "STOP"
                            continue

                        else:
                            # Noch nicht fein genug -> weiter verfeinern
                            state = "REFINE"
                            continue

                    elif state == "REFINE":
                        refine_iterations += 1
                        # Suchschrittgroesse halbieren, jedoch nicht unter die
                        # angeforderte Ziel-Schrittgroesse
                        step_size_m_search = max(step_size_m_search / 2, step_size_m_min)

                        state = "SEARCH"
                        continue

                    elif state == "STOP":
                        detected_position = list(contact_position)
                        break


                # Frame-Korrektur berechnen und anwenden
                # HINWEIS: Stiftradius wird hier noch nicht beruecksichtigt, da der
                # Wert noch nicht eingemessen wurde. Muss vor den eigentlichen
                # Versuchen ergaenzt werden, sonst verbleibt ein systematischer
                # Versatz in der Korrektur.

                # scene = self.pm_robot_utils.assembly_scene_analyzer
                # target_pose = scene.get_pose_from_frame(request.target_frame)

                # if detected_position is not None:
                #     correction_transform = self.compute_frame_correction(
                #         detected_position,
                #         target_pose
                #     )

                #     self.pm_robot_utils.assembly_scene_analyzer.modify_frame_with_transform(
                #         request.target_frame,
                #         correction_transform
                #     )

                scan_time = time.time() - scan_start_time
                contact_distance = np.dot(
                    np.array(detected_position) - np.array(start_position),
                    np.array(direction_normalized)
                ) if detected_position is not None else math.nan

                # Schritt 9: Antwort setzen
                if detected_position is None:
                    response.success = False
                    response.message = (
                    f'No contact detected after {contact_distance * 1e3:.3f} mm'                
                    )
                else:
                    response.success = True
                    response.message = (
                        f'Contact detected after {contact_distance * 1e3:.3f} mm'
                    )

                    response.detected_position.position.x = detected_position[0]
                    response.detected_position.position.y = detected_position[1]
                    response.detected_position.position.z = detected_position[2]
                    response.detected_position.orientation.w = 1.0

                    self.get_logger().info(
                        f'Contact position (World): '
                        f'X={round(detected_position[0]*1e3, 3)} mm, '
                        f'Y={round(detected_position[1]*1e3, 3)} mm, '
                        f'Z={round(detected_position[2]*1e3, 3)} mm'
                    )


                detected_position_log = [detected_position[i] for i in range(3)] if detected_position is not None else [math.nan, math.nan, math.nan]
                self.get_logger().info(
                    f"detected_position: "
                    f"{detected_position_log}"
                )

                final_search_step_um = step_size_m_search * 1e6

                success_flag = detected_position is not None                
                self.csv_force_scan(
                    scan_number,
                    request,
                    final_search_step_um,
                    direction_normalized,
                    detected_position_log,
                    frame_position_contact,
                    frame_position_target,
                    start_position,
                    contact_distance,
                    contact_force,
                    search_iterations,
                    refine_iterations,
                    scan_time,
                    timestamp,
                    success_flag,
                    hard_contact_threshold
                )
                self.get_logger().info("Data saved to CSV.")

                # Schritt 10: Zurück zur Startposition (Anfahrposition)
                self.get_logger().info('Returning to start position...')

                returned = self._return_to_start(start_position)

                if not returned:
                    self.get_logger().error("Stopping repeatability test: return failed")
                    break

                self.get_logger().info(response.message)
                time.sleep(0.5)

        except Exception as e:
            response.success = False
            response.message = f'Error: {str(e)}'
            self.get_logger().error(response.message)

        return response
    



    def _return_to_start(self, start_position: list) -> bool:
        """Hilfsfunktion: Faehrt zur angegebenen Startposition zurueck."""
        return_ok = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(
            start_position[0],
            start_position[1],
            start_position[2],
            time=1.0
        )
        if not return_ok:
            self.get_logger().error('could not return to start position!')
        return return_ok

    def csv_force_scan_step(
            self,
            scan_number,
            request,
            step,
            state,
            step_size_um,
            current_force,
            current_position,
            timestamp
        ):

        import os
        import csv

        base_folder = (
            "/home/pmlab/pm_Server/01_PM_Zelle/03_PM_DataBase/pm_assembly_database/RSAP_Processes/Bente/documentation_and_plots/messungen_neu"
        )

        folder_name = (
            f"S{request.step_size}"
            f"_fx{request.max_force.x:.1f}"
            f"_fy{request.max_force.y:.1f}"
            f"_fz{request.max_force.z:.1f}" 
        )

        folder = os.path.join(base_folder, folder_name)

        data_folder = os.path.join(folder, "data")

        os.makedirs(data_folder, exist_ok=True)

        file_name = "search_refine_log.csv"
        file_path = os.path.join(data_folder, file_name)

        file_exists = os.path.isfile(file_path)

        fieldnames = [
            "ScanNumber",
            "Timestamp",
            "Step",
            "State",
            "StepSize_um",
            "Force_x",
            "Force_y",
            "Force_z",
            "Position_x",
            "Position_y",
            "Position_z",
        ]

        row = {
            "ScanNumber": scan_number,
            "Timestamp": timestamp,
            "Step": step,
            "State": state,
            "StepSize_um": step_size_um,
            "Force_x": current_force[0],
            "Force_y": current_force[1],
            "Force_z": current_force[2],
            "Position_x": current_position[0],
            "Position_y": current_position[1],
            "Position_z": current_position[2],
        }

        with open(file_path, "a", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)


    def csv_force_scan(
        self,
        scan_number,
        request,
        final_search_step_um,
        direction_normalized,
        detected_position,
        frame_position_contact,
        frame_position_target,
        start_position,
        contact_distance,
        contact_force,
        search_iterations,
        refine_iterations,
        scan_time,
        timestamp,
        success_flag,
        hard_contact_threshold
    ):

        import os
        import csv
        import math
    
        base_folder = (
            "/home/pmlab/pm_Server/01_PM_Zelle/03_PM_DataBase/pm_assembly_database/RSAP_Processes/Bente/documentation_and_plots/messungen_neu"
        )

        folder_name = (
            f"S{request.step_size}"
            f"_fx{request.max_force.x:.1f}"
            f"_fy{request.max_force.y:.1f}"
            f"_fz{request.max_force.z:.1f}"
        )

        folder = os.path.join(base_folder, folder_name)

        data_folder = os.path.join(folder, "data")

        os.makedirs(data_folder, exist_ok=True)


        file_name = "data.csv"
        file_path = os.path.join(data_folder, file_name)

        file_exists = os.path.isfile(file_path)


        fieldnames = [
            "ScanNumber",
            "Timestamp",
            "StepSize_um",
            "final_search_step_um",
            "Threshold_x",
            "Threshold_y",
            "Threshold_z",
            "Direction_x",
            "Direction_y",
            "Direction_z",
            "Start_x",
            "Start_y",
            "Start_z",
            "Contact_x",
            "Contact_y",
            "Contact_z",
            "Frame_x_contact",
            "Frame_y_contact",
            "Frame_z_contact",
            "Frame_x_target",
            "Frame_y_target",
            "Frame_z_target",
            "Contact_distance_um",
            "Force_x_contact",
            "Force_y_contact",
            "Force_z_contact",
            "Search_iterations",
            "Refine_iterations",
            "Scan_time_s",
            "Success",
            "hard_contact_Threshold_x",
            "hard_contact_Threshold_y",
            "hard_contact_Threshold_z",
        ]

        def safe_vec3(v):
            if v is None or len(v) != 3:
                return (math.nan, math.nan, math.nan)
            return v

        fx, fy, fz = safe_vec3(frame_position_contact)
        cx, cy, cz = safe_vec3(detected_position) if success_flag else (math.nan, math.nan, math.nan)

        with open(file_path, mode='a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=';')

            if not file_exists:
                writer.writeheader()

            writer.writerow({
                "ScanNumber": scan_number,
                "Timestamp": timestamp,
                "StepSize_um": request.step_size,
                "final_search_step_um": final_search_step_um,
                "Threshold_x": request.max_force.x,
                "Threshold_y": request.max_force.y,
                "Threshold_z": request.max_force.z,
                "Direction_x": direction_normalized[0],
                "Direction_y": direction_normalized[1],
                "Direction_z": direction_normalized[2],
                "Start_x": start_position[0],
                "Start_y": start_position[1],
                "Start_z": start_position[2],
                "Contact_x": cx,
                "Contact_y": cy,
                "Contact_z": cz,
                "Frame_x_contact": fx,
                "Frame_y_contact": fy,
                "Frame_z_contact": fz,
                "Frame_x_target": frame_position_target[0],
                "Frame_y_target": frame_position_target[1],
                "Frame_z_target": frame_position_target[2],
                "Contact_distance_um": contact_distance * 1e6,
                "Force_x_contact": contact_force[0],
                "Force_y_contact": contact_force[1],
                "Force_z_contact": contact_force[2],
                "Search_iterations": search_iterations,
                "Refine_iterations": refine_iterations,
                "Scan_time_s": scan_time,
                "Success": int(success_flag),
                "hard_contact_Threshold_x": hard_contact_threshold[0],
                "hard_contact_Threshold_y": hard_contact_threshold[1],
                "hard_contact_Threshold_z": hard_contact_threshold[2],
            })

    def edge_scan_callback(self, request: pm_skill_srv.EdgeScan.Request, response: pm_skill_srv.EdgeScan.Response):
        """
        Kamera-basierte Wiederholmessung einer Kante, analog zu force_scan_callback.
 
        Request:
            - target_frame (string):          Frame, zu dem die Kamera gefahren wird
            - camera_config_filename (string): z.B. self.pm_robot_utils.get_cam_file_name_bottom()
            - process_filename (string):       Name des Vision-Prozesses (Kantenerkennung)
            - num_repetitions (int32):         Anzahl der Wiederholungen (z.B. 25)
 
        Response:
            - success (bool)
            - message (string)
        """

        from pm_vision_interfaces.srv import ExecuteVision
        import random
 
        try:
            for i in range(request.num_repetitions):
                scan_number = i + 1
                self.get_logger().info(f"Starting edge scan {scan_number} of {request.num_repetitions}.")

                offset_range = 0.003 # 3 mm

                dx = random.uniform(-offset_range, offset_range)
                dy = random.uniform(-offset_range, offset_range)
 
                timestamp = datetime.now().isoformat()
 
                # --- Schritt 1: Kamera zur Zielposition fahren ---
                move_success, move_msg = self.pm_robot_utils.move_camera_top_to_frame(
                    request.target_frame,
                    x_offset=dx,
                    y_offset=dy
                )

                if not move_success:
                    response.success = False
                    response.message = f"Could not move to target frame (+offset): {move_msg}"
                    self.get_logger().error(response.message)
                    return response

                move_success, move_msg = self.pm_robot_utils.move_camera_top_to_frame(
                    request.target_frame
                )

                if not move_success:
                    response.success = False
                    response.message = f"Could not move to target frame: {move_msg}"
                    self.get_logger().error(response.message)
                    return response
                
                time.sleep(0.2)
 
                # --- Schritt 2: Vision-Messung durchfuehren ---
                vision_request = ExecuteVision.Request()
                vision_request.camera_config_filename = request.camera_config_filename
                vision_request.image_display_time = -1
                vision_request.process_filename = request.process_filename
                vision_request.process_uid = f"Edge_Scan_{scan_number}"
 
                if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
                    raise PmRobotError("Vision Manager not available...")
 
                vision_response: ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(vision_request)
 
                if not vision_response.success:
                    raise PmRobotError("Vision measurement failed!")
 
                # --- Ergebnis extrahieren ---
                # Kanten-/Eckenerkennung liefert einen einzelnen Punkt.
                if len(vision_response.vision_response.results.points) != 1:
                    raise PmRobotError("Vision did not find a single edge/corner point!")
 
                detected_point = vision_response.vision_response.results.points[0]
                x_detected = detected_point.axis_value_1
                y_detected = detected_point.axis_value_2
 
                self.get_logger().info(
                    f"Detected edge position: X={x_detected} um, Y={y_detected} um"
                )
 
                # --- Schritt 3: Ergebnis loggen ---
                self.csv_edge_scan_step(
                    scan_number=scan_number,
                    request=request,
                    x_detected=x_detected,
                    y_detected=y_detected,
                    timestamp=timestamp,
                    dx=dx,
                    dy=dy
                )
 
            response.success = True
            response.message = f"Edge scan completed with {request.num_repetitions} repetitions."
            self.get_logger().info(response.message)
 
        except PmRobotError as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
 
        return response
 
    def csv_edge_scan_step(self, scan_number, request, x_detected, y_detected, timestamp, dx, dy):
 
        import os
        import csv
 
        base_folder = (
            "/home/pmlab/pm_Server/01_PM_Zelle/03_PM_DataBase/pm_assembly_database/RSAP_Processes/Bente/documentation_and_plots/measurements"
        )
 
        folder_name = f"EdgeScan_{request.target_frame}_"
        folder = os.path.join(base_folder, folder_name)
        data_folder = os.path.join(folder, "data")
        os.makedirs(data_folder, exist_ok=True)
 
        file_name = "edge_scan_data.csv"
        file_path = os.path.join(data_folder, file_name)
        file_exists = os.path.isfile(file_path)
 
        fieldnames = [
            "ScanNumber",
            "Timestamp",
            "TargetFrame",
            "X_detected_um",
            "Y_detected_um",
            "dx_m",
            "dy_m"
        ]
 
        row = {
            "ScanNumber": scan_number,
            "Timestamp": timestamp,
            "TargetFrame": request.target_frame,
            "X_detected_um": x_detected,
            "Y_detected_um": y_detected,
            "dx_m": dx,
            "dy_m": dy
        }
 
        with open(file_path, "a", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)
