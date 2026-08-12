from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.impl.rcutils_logger import RcutilsLogger
    from pm_skills.pm_skills import PmSkills
    from pm_skills.py_modules.PmRobotUtils import PmRobotUtils


class PmSkillDomain:
    """Base class for a group of skills hosted by the shared ROS node."""

    def __init__(self, node: 'PmSkills') -> None:
        # Keep these dependencies explicit so static analysis and IDE completion
        # can expose the PmRobotUtils API from every skill domain.
        self.node: 'PmSkills' = node
        self.pm_robot_utils: 'PmRobotUtils' = node.pm_robot_utils
        self.logger: 'RcutilsLogger' = node.get_logger()
        self._logger: 'RcutilsLogger' = self.logger

        # Shared ROS clients/state. They are initialized by PmSkills before the
        # domain objects are constructed.
        self.tf_buffer = node.tf_buffer
        self.attach_component = node.attach_component
        self.smart_gripper_force_client = node.smart_gripper_force_client
        self.dispense_2k_unity_client = node.dispense_2k_unity_client
        self.dispense_2k_unity_done_event = node.dispense_2k_unity_done_event
        self.adapt_frame_client = node.adapt_frame_client
        self.client_move_robot_tool_to_frame = node.client_move_robot_tool_to_frame

        # Constants used by the split-out skill implementations.
        self.PM_ROBOT_GRIPPER_FRAME = node.PM_ROBOT_GRIPPER_FRAME
        self.PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR = node.PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR
        self.PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR = node.PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR
        self.GRIP_RELATIVE_LIFT_DISTANCE = node.GRIP_RELATIVE_LIFT_DISTANCE
        self.RELEASE_LIFT_DISTANCE = node.RELEASE_LIFT_DISTANCE
        self.GRIP_APPROACH_OFFSET = node.GRIP_APPROACH_OFFSET
        self.GRIP_SENSING_START_OFFSET = node.GRIP_SENSING_START_OFFSET
        self.GRIP_SENSING_END_OFFSET = node.GRIP_SENSING_END_OFFSET
        self.GRIP_SENSING_END_ROUGH = node.GRIP_SENSING_END_ROUGH
        self.ASSEMBLY_FRAME_INDICATOR = node.ASSEMBLY_FRAME_INDICATOR
        self.TARGET_FRAME_INDICATOR = node.TARGET_FRAME_INDICATOR
        self.GRIPPING_OFFSET = node.GRIPPING_OFFSET

    def get_logger(self) -> 'RcutilsLogger':
        """Return the shared node logger without relying on dynamic lookup."""
        return self.logger
