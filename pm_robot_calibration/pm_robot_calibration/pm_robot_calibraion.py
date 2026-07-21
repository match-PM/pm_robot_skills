from rclpy.executors import MultiThreadedExecutor
import rclpy
from rclpy.node import Node
import pm_skills_interfaces.srv as skills_srv
import pm_skills_interfaces.action as skills_action
from rclpy.action import ActionServer
from pm_msgs.srv import EmptyWithSuccess
from pm_robot_calibration.py_modules.pm_calibration_cameras import PmRobotCalibrationCameras
from pm_robot_calibration.py_modules.pm_calibration_smarpod import PmRobotCalibrationSmarpod
from pm_robot_calibration.py_modules.pm_calibration_gripper import PmRobotCalibrationGripper
from pm_robot_calibration.py_modules.pm_calibration_distance_sensors import PmRobotDistanceSensorCalibration
from pm_robot_calibration.py_modules.pm_calibration_chuck import PmRobotChuckCalibration
from pm_robot_calibration.py_modules.pm_calibration_utils import PmRobotCalibrationUtils

class PmRobotCalibrationNode(Node):
    def __init__(self):
        
        super().__init__('pm_robot_calibration')
        self._logger = self.get_logger()

        self._logger.info(" Pm Robot Calibration Node started...")
        
        self.pm_calibration_utils = PmRobotCalibrationUtils(self)
        self.callback_group = self.pm_calibration_utils.callback_group

        self.camera_calibration = PmRobotCalibrationCameras(self, self.pm_calibration_utils)
        self.distance_sensor_calibration = PmRobotDistanceSensorCalibration(self, self.pm_calibration_utils)
        self.gripper_calibration = PmRobotCalibrationGripper(self, self.pm_calibration_utils)
        self.chuck_calibration = PmRobotChuckCalibration(self, self.pm_calibration_utils)
        self.smarpod_calibration = PmRobotCalibrationSmarpod(self, self.pm_calibration_utils)
        
        # services
        self.calibrate_cameras_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_cameras', self.camera_calibration.calibrate_cameras_callback, callback_group=self.callback_group)
        self.calibrate_calibration_cube_xy_on_cam_top_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_calibration_cube_xy_on_camera_top', self.camera_calibration.calibrate_calibration_cube_to_cam_top, callback_group=self.callback_group)
        self.calibrate_calibration_cube_z_laser_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_calibration_cube_z_on_confocal_top', self.distance_sensor_calibration.calibrate_calibration_cube_z_on_confocal_top, callback_group=self.callback_group)

        #self.calibrate_laser_on_calibration_cube_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_laser_on_calibration_cube', self.calibrate_laser_on_calibration_cube_callback, callback_group=self.callback_group)
        #self.calbirate_confocal_top_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_top', self.calibrate_confocal_top_callback, callback_group=self.callback_group)
        self.calibrate_laser_head_on_camera_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_laser_xy_on_camera_bottom', self.distance_sensor_calibration.calibrate_laser_xy_on_camera_bottom, callback_group=self.callback_group)
        self.calibrate_confocal_head_on_camera_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_top_xy_on_camera_bottom', self.distance_sensor_calibration.calibrate_confocal_top_xy_on_camera_bottom, callback_group=self.callback_group)
        self.calibrate_confocal_bottom_z_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_bottom_z_on_laser', self.distance_sensor_calibration.calibrate_confocal_bottom_z_on_laser, callback_group=self.callback_group)
        self.calibrate_confocal_top_z_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_top_z_on_laser', self.distance_sensor_calibration.calibrate_confocal_top_z_on_laser, callback_group=self.callback_group)
        self.calibrate_confocal_bottom_xy_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_bottom_xy_on_cam_top', self.distance_sensor_calibration.calibrate_confocal_bottom_xy_on_cam_top, callback_group=self.callback_group)

        #self.calibrate_siemens_gripper_on_cal_cube = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_siemens_gripper_on_cal_cube', self.calibrate_sim_gripper_on_cube, callback_group=self.callback_group)
        self.calbibrate_gonio_left_chuck_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gonio_left_chuck', self.chuck_calibration.calibrate_gonio_left_chuck_callback, callback_group=self.callback_group)
        self.calbibrate_gonio_right_chuck_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gonio_right_chuck', self.chuck_calibration.calibrate_gonio_right_chuck_callback, callback_group=self.callback_group)
        self.calibrate_gripper_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gripper_xyt_on_camera_bottom', self.gripper_calibration.calibrate_gripper_xyt_on_camera_bottom, callback_group=self.callback_group)
        self.calibrate_gripper_plane_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gripper_plane', self.gripper_calibration.calibrate_gripper_plane, callback_group=self.callback_group)
        
        self.calibrate_1K_dispenser_xy_on_cam_bottom_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_1K_dispenser_xy_on_cam_bottom', self.gripper_calibration.calibrate_1K_dispenser_xy_on_cam_bottom, callback_group=self.callback_group)
        self.calibrate_1K_dispenser_z_on_calibration_cube_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_1K_dispenser_z_on_calibration_cube', self.gripper_calibration.calibrate_1K_dispenser_z_on_calibration_cube, callback_group=self.callback_group)
       
        self.assess_hexapod_calibration_file_srv = self.create_service(skills_srv.AssessHexapodCalibration, '/pm_robot_calibration/assess_hexapod_calibration_file', self.smarpod_calibration.assess_hexapod_calibration, callback_group=self.callback_group)

        self.calibrate_smarpod_action_srv = ActionServer(self,
            skills_action.SmarpodCalibration,
            f'/pm_robot_calibration/calibrate_smarpod',
            execute_callback=self.smarpod_calibration.calibrate_smarpod_V3,
            goal_callback=self.pm_calibration_utils._goal_calibration_callback,
            cancel_callback=self.pm_calibration_utils._cancel_calibration_callback
        )

        self.test_hexapod_calibration_action_srv = ActionServer(self,
            skills_action.TestHexapodCalibration,
            f'/pm_robot_calibration/test_calibrate_smarpod',
            execute_callback=self.smarpod_calibration.test_hexapod_calibration,
            goal_callback=self.pm_calibration_utils._goal_calibration_callback,
            cancel_callback=self.pm_calibration_utils._cancel_calibration_callback
        )
    

    
def main(args=None):
    rclpy.init(args=args)

    node = PmRobotCalibrationNode()

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
