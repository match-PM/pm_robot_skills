import threading

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

import assembly_manager_interfaces.srv as ami_srv
import pm_moveit_interfaces.srv as pm_moveit_srv
import pm_msgs.srv as pm_msg_srv
import pm_skills_interfaces.srv as pm_skill_srv
import pm_skills_interfaces.action as pm_skill_action
import std_msgs.msg as std_msg
from pm_msgs.srv import EmptyWithSuccess

from pm_skills.py_modules.PmRobotUtils import PmRobotUtils
from pm_skills.py_modules.pm_alignment_search_skills import PmAlignmentSearchSkills
from pm_skills.py_modules.pm_dispensing_skills import PmDispensingSkills
from pm_skills.py_modules.pm_force_skills import PmForceSkills
from pm_skills.py_modules.pm_gonio_skills import PmGonioSkills
from pm_skills.py_modules.pm_gripper_skills import PmGripperSkills
from pm_skills.py_modules.pm_laser_line_scan_skills import PmLineScanSkills
from pm_skills.py_modules.pm_measurement_skills import PmMeasurementSkills
from pm_skills.py_modules.pm_uv_skills import PmUvSkills


class PmSkills(Node):
    PM_ROBOT_GRIPPER_FRAME = 'PM_Robot_Tool_TCP'
    PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR = 'Gonio_Left_Part'
    PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR = 'Gonio_Right_Part'
    GRIP_RELATIVE_LIFT_DISTANCE = 0.05
    RELEASE_LIFT_DISTANCE = 0.02
    GRIP_APPROACH_OFFSET = 0.05
    GRIP_SENSING_START_OFFSET = 0.000500
    GRIP_SENSING_END_OFFSET = -0.0005
    GRIP_SENSING_END_ROUGH = 0.00020
    ASSEMBLY_FRAME_INDICATOR = 'assembly_frame_Description'
    TARGET_FRAME_INDICATOR = 'target_frame_Description'
    GRIPPING_OFFSET = 0.0001
    def __init__(self) -> None:
        super().__init__('pm_skills')

        self.callback_group_me = MutuallyExclusiveCallbackGroup()
        self.callback_group_re = ReentrantCallbackGroup()
        self.pm_robot_utils = PmRobotUtils(self)
        self.pm_robot_utils.start_object_scene_subscribtion()

        # Shared clients and state must exist before constructing the skill
        # domains, which receive explicit typed references to them.
        self.attach_component = self.create_client(ami_srv.ChangeParentFrame, '/assembly_manager/change_obj_parent_frame')
        self.smart_gripper_force_client = self.create_client(pm_msg_srv.GripperGetForces, '/SmarAct_Gripper/GetForces')
        self.dispense_2k_unity_client = self.create_client(pm_skill_srv.DispensePathUnity, '/unity_skills/dispense_2k_unity')
        self.dispense_2k_unity_done_event = threading.Event()
        self.adapt_frame_client = self.create_client(ami_srv.ModifyPoseAbsolut, '/assembly_manager/modify_frame_absolut')
        self.client_move_robot_tool_to_frame = self.create_client(pm_moveit_srv.MoveToFrame, "/pm_moveit_server/move_tool_to_frame")
        self.logger = self.get_logger()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.force_skills = PmForceSkills(self)
        self.gonio_skills = PmGonioSkills(self)
        self.gripper_skills = PmGripperSkills(self)
        self.measurement_skills = PmMeasurementSkills(self)
        self.line_scan_skills = PmLineScanSkills(self)
        self.alignment_search_skills = PmAlignmentSearchSkills(self)
        self.dispensing_skills = PmDispensingSkills(self)
        self.uv_skills = PmUvSkills(self)
        self.skill_domains = (
            self.force_skills,
            self.gonio_skills,
            self.gripper_skills,
            self.measurement_skills,
            self.line_scan_skills,
            self.alignment_search_skills,
            self.dispensing_skills,
            self.uv_skills,
        )
        
        # services
        self.grip_component_srv = self.create_service(pm_skill_srv.GripComponent, "pm_skills/grip_component", self.gripper_skills.grip_component_callback,callback_group=self.callback_group_me)
        self.force_grip_component_srv = self.create_service(pm_skill_srv.GripComponent, "pm_skills/force_grip_component", self.gripper_skills.force_grip_component_callback,callback_group=self.callback_group_me)

        self.place_component_srv = self.create_service(pm_skill_srv.PlaceComponent, "pm_skills/place_component", self.gripper_skills.place_component_callback,callback_group=self.callback_group_me)
        self.release_component_srv = self.create_service(EmptyWithSuccess, "pm_skills/release_component", self.gripper_skills.release_component_callback,callback_group=self.callback_group_me)

        #self.assemble_srv  = self.create_service(EmptyWithSuccess, "pm_skills/assemble", self.assemble_callback,callback_group=self.callback_group_me)
        
        self.vacuum_gripper_on_service = self.create_service(EmptyWithSuccess, "pm_skills/vacuum_gripper/vacuum_on", self.gripper_skills.vaccum_gripper_on_callback, callback_group=self.callback_group_me)
        self.vacuum_gripper_off_service = self.create_service(EmptyWithSuccess, "pm_skills/vacuum_gripper/vacuum_off", self.gripper_skills.vaccum_gripper_off_callback, callback_group=self.callback_group_me)
            
        # dummy
        #self.dispenser_service = self.create_service(pm_skill_srv.DispenseAdhesive, "pm_skills/dispense_adhesive", self.dispenser_callback)
        #self.confocal_laser_service = self.create_service(pm_skill_srv.ConfocalLaser, "pm_skills/confocal_laser", self.confocal_laser_callback)
        #self.vision_service = self.create_service(pm_skill_srv.ExecuteVision, "pm_skills/execute_vision", self.vision_callback)

        self.move_uv_in_curing_position_service = self.create_service(pm_msg_srv.EmptyWithSuccess, "/pm_skills"+"/move_uv_in_curing_position", self.uv_skills.move_uv_in_curing_position_service_callback,callback_group=self.callback_group_me)
        self.move_uv_out_of_curing_position_service = self.create_service(pm_msg_srv.EmptyWithSuccess, "/pm_skills"+"/move_uv_out_of_curing_position", self.uv_skills.move_uv_out_of_curing_position_service_callback,callback_group=self.callback_group_me)

        self.measue_with_laser_srv = self.create_service(pm_skill_srv.CorrectFrameLaser, "pm_skills/measure_with_laser", self.measurement_skills.measure_with_laser_callback, callback_group=self.callback_group_me)
        self.correct_frame_with_laser_srv = self.create_service(pm_skill_srv.CorrectFrameLaser, "pm_skills/correct_frame_with_laser", self.measurement_skills.correct_frame_with_laser, callback_group=self.callback_group_me)

        self.correct_frame_laser_line_scan_action = ActionServer(
            self,
            pm_skill_action.CorrectFrameLineScan,
            '/pm_skills/correct_frame_laser_line_scan',
            execute_callback=self.line_scan_skills.correct_frame_laser_line_scan,
            goal_callback=self.line_scan_skills.goal_callback,
            cancel_callback=self.line_scan_skills.cancel_callback,
            callback_group=self.callback_group_re,
        )
        self.correct_frame_confocal_line_scan_action = ActionServer(
            self,
            pm_skill_action.CorrectFrameLineScan,
            '/pm_skills/correct_frame_confocal_line_scan',
            execute_callback=self.line_scan_skills.correct_frame_confocal_line_scan,
            goal_callback=self.line_scan_skills.goal_callback,
            cancel_callback=self.line_scan_skills.cancel_callback,
            callback_group=self.callback_group_re,
        )
        self.rect_spiral_search_action = ActionServer(
            self,
            pm_skill_action.RectSpiralSearch,
            '/pm_skills/rect_spiral_search',
            execute_callback=self.alignment_search_skills.rect_spiral_search,
            goal_callback=self.alignment_search_skills.goal_callback,
            cancel_callback=self.alignment_search_skills.cancel_callback,
            callback_group=self.callback_group_re,
        )
        
        self.force_sensing_move_srv = self.create_service(pm_msg_srv.GripperForceMove, self.get_name()+'/gripper_force_sensing', self.force_skills.force_sensing_move_callback, callback_group=self.callback_group_me)
        self.force_scan_srv = self.create_service(pm_skill_srv.ForceScan, self.get_name() + "/force_scan", self.force_skills.force_scan_callback, callback_group=self.callback_group_me)
        self.edge_scan_srv = self.create_service(pm_skill_srv.EdgeScan, self.get_name() + "/edge_scan", self.force_skills.edge_scan_callback, callback_group=self.callback_group_me)

        
        self.measure_frame_with_confocal_bottom_srv = self.create_service(pm_skill_srv.CorrectFrameLaser, "pm_skills/measure_frame_with_confocal_bottom", self.measurement_skills.measure_frame_with_confocal_bottom, callback_group=self.callback_group_me)
        self.correct_frame_with_confocal_bottom_srv = self.create_service(pm_skill_srv.CorrectFrameLaser, "pm_skills/correct_frame_with_confocal_bottom", self.measurement_skills.correct_frame_with_confocal_bottom, callback_group=self.callback_group_me)

        self.disp_at_path_srv = self.create_service(pm_msg_srv.DispenseAtPath, self.get_name()+'/dispense_2k_at_path', self.dispensing_skills.dispense_2k_at_path, callback_group=self.callback_group_me)

        self.measure_frame_with_confocal_top_srv = self.create_service(pm_skill_srv.CorrectFrameLaser, "pm_skills/measure_frame_with_confocal_top", self.measurement_skills.measure_frame_with_confocal_top, callback_group=self.callback_group_me)
        self.correct_frame_with_confocal_top_srv = self.create_service(pm_skill_srv.CorrectFrameLaser, "pm_skills/correct_frame_with_confocal_top", self.measurement_skills.correct_frame_with_confocal_top, callback_group=self.callback_group_me)

        self.srv_iter_align_gonio_right = self.create_service(pm_skill_srv.IterativeGonioAlign, self.get_name()+'/iterative_align_gonio_right', self.gonio_skills.iterative_align_gonio_right, callback_group=self.callback_group_me)
        self.srv_iter_align_gonio_left = self.create_service(pm_skill_srv.IterativeGonioAlign, self.get_name()+'/iterative_align_gonio_left', self.gonio_skills.iterative_align_gonio_left, callback_group=self.callback_group_me)

        self.srv_check_frame_measureble_confocal_top = self.create_service(pm_skill_srv.CheckFrameMeasurable, self.get_name()+'/check_frame_measureble_confocal_top', self.measurement_skills.check_frame_mes_confocal_top, callback_group=self.callback_group_me)
        self.srv_check_frame_measureble_laser_top = self.create_service(pm_skill_srv.CheckFrameMeasurable, self.get_name()+'/check_frame_measureble_laser_top', self.measurement_skills.check_frame_mes_laser_top, callback_group=self.callback_group_me)
        self.srv_check_frame_measureble_confocal_bottom = self.create_service(pm_skill_srv.CheckFrameMeasurable, self.get_name()+'/check_frame_measureble_confocal_bottom', self.measurement_skills.check_frame_mes_confocal_bottom, callback_group=self.callback_group_me)

        self.srv_dispense_at_points = self.create_service(pm_msg_srv.DispenseAtPoints, self.get_name()+'/dispense_at_frames', self.dispensing_skills.dispense_at_frames_callback, callback_group=self.callback_group_me)

        self.srv_dispense_at_points_adv = self.create_service(pm_skill_srv.DispenseAtPointsAdv, self.get_name()+'/dispense_at_frames_adv', self.dispensing_skills.dispense_at_frames_adv_callback, callback_group=self.callback_group_me)

        # Same skills for the 2K dispenser. It has no protection lid and is moved down/up
        # with the '2K_Dispenser_Joint' cylinder only.
        self.srv_dispense_2k_at_points = self.create_service(pm_msg_srv.DispenseAtPoints, self.get_name()+'/dispense_2k_at_frames', self.dispensing_skills.dispense_2k_at_frames_callback, callback_group=self.callback_group_me)

        self.srv_dispense_2k_at_points_adv = self.create_service(pm_skill_srv.DispenseAtPointsAdv, self.get_name()+'/dispense_2k_at_frames_adv', self.dispensing_skills.dispense_2k_at_frames_adv_callback, callback_group=self.callback_group_me)
        self.srv_uv_cure_adv = self.create_service(pm_msg_srv.UVCuringSkill, self.get_name()+'/uv_cure', self.uv_skills.uv_cure_adv_callback, callback_group=self.callback_group_me)

        # Passes the dispense start frame to Unity in the request so Unity can attach
        # the visualized adhesive bead to that frame's parent component.
        # Completion arrives on this topic. The subscription is reentrant so it
        # can fire while the dispense callback waits on the event.
        self.dispense_2k_unity_done_subscriber = self.create_subscription(std_msg.Bool, '/unity_skills/dispense_2k_done', self.dispensing_skills.dispense_2k_unity_done_callback, 10, callback_group=self.callback_group_re)

        self.simtime_subscriber = self.create_subscription(std_msg.Bool, '/sim_time', self.gripper_skills.simtime_callback, 10, callback_group=self.callback_group_re)


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
