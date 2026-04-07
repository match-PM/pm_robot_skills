import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/match-pm/ros2_ws_pm/src/pm_robot_skills/install/pm_skills'
