from rclpy.node import Node
from pm_msgs.srv import EmptyWithSuccess
from pm_robot_calibration.py_modules.pm_calibration_utils import PmRobotCalibrationUtils

class PmRobotChuckCalibration:
    def __init__(self, node: Node, utils: PmRobotCalibrationUtils):
        self.node = node
        self._logger = node.get_logger()
        self.pm_calibration_utils = utils
        self.pm_robot_utils = utils.pm_robot_utils

    def calibrate_gonio_left_chuck_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        # To be implemented...
        self._logger.error("Gonio left chuck not implemented yet...")   

        return response

    def calibrate_gonio_right_chuck_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):

        # To be implemented...
        self._logger.error("Gonio right chuck not implemented yet...")
        
        return response

def main(args=None):
    pass


if __name__ == '__main__':
    main()
