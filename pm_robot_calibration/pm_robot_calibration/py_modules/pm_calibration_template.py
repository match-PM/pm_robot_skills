from rclpy.node import Node
from pm_robot_calibration.py_modules.pm_calibration_utils import PmRobotCalibrationUtils

class PmRobotCalibrationTemplate:
    def __init__(self, node: Node, utils: PmRobotCalibrationUtils):
        self.node = node
        self._logger = node.get_logger()
        self.pm_calibration_utils = utils



    
def main(args=None):
    pass


if __name__ == '__main__':
    main()
