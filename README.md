# PM Robot Skills

A comprehensive ROS 2 package suite providing robotic manipulation and automation skills for the PM Robot system. This repository contains multiple skill packages that enable griping, placing, vision-based measurement, calibration, and laser-based frame correction capabilities.

## Overview

The `pm_robot_skills` repository is composed of several integrated packages:

- **pm_skills**: Core manipulation skills including griping, placing, and laser-based frame correction
- **pm_vision_skills**: Vision-based measurement and frame correction using camera systems
- **pm_robot_calibration**: Comprehensive calibration tools for cameras, laser, grippers, and goniometers
- **pm_skills_interfaces**: ROS 2 service and message definitions for skill interfaces
- **pm_launch_skills**: Launch configuration files for easy system startup

## Prerequisites

- ROS 2 (Humble or later recommended)
- Python 3.8+
- Dependencies:
  - `pm_moveit_interfaces`
  - `assembly_manager_interfaces`
  - `pm_vision_interfaces`
  - `pm_msgs`
  - `assembly_scene_publisher`
  - `pm_vision_manager`

## Installation

1. Clone the repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/your-repo/pm_robot_skills.git
```

2. Install dependencies (this has nt been tested):
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the packages:
```bash
colcon build
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Quick Start

### Launch the PM Robot Skills System

To launch all skill nodes together:

```bash
ros2 launch pm_launch_skills pm_skills.launch.py
```

This will start the following nodes:
- `pm_vision_skills` - Vision-based measurement and correction services
- `pm_skills` - Core manipulation and laser measurement services
- `pm_robot_calibration` - Calibration services

## Available ROS 2 Services

### pm_skills Node

#### Griping and Placement

- **`/pm_skills/force_grip_component`** (`GripComponent`)
  - Force-based gripping with iterative force sensing
  - Parameters: component_name (str), align_orientation (bool)
  - Returns: success (bool), message (str)

- **`/pm_skills/place_component`** (`PlaceComponent`)
  - Places a gripped component at target position
  - Parameters: align_orientation (bool), x/y/z_offset_um (int), rx/ry/rz_offset_deg (float)
  - Returns: success (bool), message (str)

- **`/pm_skills/release_component`** (`EmptyWithSuccess`)
  - Releases the currently gripped component
  - Returns: success (bool), message (str)

#### Vacuum Gripper Control

- **`/pm_skills/vacuum_gripper/vacuum_on`** (`EmptyWithSuccess`)
  - Activates the vacuum gripper
  - Returns: success (bool), message (str)

- **`/pm_skills/vacuum_gripper/vacuum_off`** (`EmptyWithSuccess`)
  - Deactivates the vacuum gripper
  - Returns: success (bool), message (str)

#### Laser-Based Measurements and Corrections

- **`/pm_skills/measure_with_laser`** (`CorrectFrame`)
  - Measures frame position using laser sensor
  - Parameters: frame_name (str), use_iterative_sensing (bool), remeasure_after_correction (bool)
  - Returns: success (bool), message (str), correction_values (Vector3)

- **`/pm_skills/correct_frame_with_laser`** (`CorrectFrame`)
  - Corrects frame position based on laser measurement
  - Parameters: frame_name (str), use_iterative_sensing (bool), remeasure_after_correction (bool)
  - Returns: success (bool), message (str), correction_values (Vector3)

#### Confocal Microscope Measurements

- **`/pm_skills/measure_frame_with_confocal_bottom`** (`CorrectFrame`)
  - Measures frame using bottom confocal microscope
  - Parameters: frame_name (str), use_iterative_sensing (bool)
  - Returns: success (bool), message (str), correction_values (Vector3)

- **`/pm_skills/correct_frame_with_confocal_bottom`** (`CorrectFrame`)
  - Corrects frame based on bottom confocal measurement
  - Parameters: frame_name (str), use_iterative_sensing (bool)
  - Returns: success (bool), message (str), correction_values (Vector3)

- **`/pm_skills/measure_frame_with_confocal_top`** (`CorrectFrame`)
  - Measures frame using top confocal microscope
  - Parameters: frame_name (str), use_iterative_sensing (bool)
  - Returns: success (bool), message (str), correction_values (Vector3)

- **`/pm_skills/correct_frame_with_confocal_top`** (`CorrectFrame`)
  - Corrects frame based on top confocal measurement
  - Parameters: frame_name (str), use_iterative_sensing (bool)
  - Returns: success (bool), message (str), correction_values (Vector3)

#### Goniometer Alignment

- **`/pm_skills/iterative_align_gonio_right`** (`IterativeGonioAlign`)
  - Iteratively aligns right goniometer to target orientation
  - Parameters: gonio_endeffector_frame (str), target_alignment_frame (str), num_iterations (int), frames_to_measure (list[str])
  - Returns: success (bool), message (str)

- **`/pm_skills/iterative_align_gonio_left`** (`IterativeGonioAlign`)
  - Iteratively aligns left goniometer to target orientation
  - Parameters: gonio_endeffector_frame (str), target_alignment_frame (str), num_iterations (int), frames_to_measure (list[str])
  - Returns: success (bool), message (str)

#### Sensor and Frame Checking

- **`/pm_skills/gripper_force_sensing`** (`GripperForceMove`)
  - Performs force-controlled movement with force sensing
  - Parameters: step_size (float), target_joints_xyz (list[float]), max_f_xyz (list[float])
  - Returns: success (bool), completed (bool), error (str)

- **`/pm_skills/check_frame_measureble_confocal_top`** (`CheckFrameMeasurable`)
  - Checks if a frame is measurable with top confocal microscope
  - Parameters: frame_name (str)
  - Returns: success (bool), is_measurable (bool)

- **`/pm_skills/check_frame_measureble_confocal_bottom`** (`CheckFrameMeasurable`)
  - Checks if a frame is measurable with bottom confocal microscope
  - Parameters: frame_name (str)
  - Returns: success (bool), is_measurable (bool)

- **`/pm_skills/check_frame_measureble_laser_top`** (`CheckFrameMeasurable`)
  - Checks if a frame is measurable with laser
  - Parameters: frame_name (str)
  - Returns: success (bool), is_measurable (bool)

### pm_vision_skills Node

#### Vision-Based Measurements

- **`/pm_skills/vision_measure_frame`** (`MeasureFrame`)
  - Measures frame position using camera vision
  - Parameters: frame_name (str), vision_process_file_name (str)
  - Returns: success (bool), message (str), correction_values (Vector3)

- **`/pm_skills/vision_correct_frame`** (`CorrectFrame`)
  - Corrects frame based on vision measurement
  - Parameters: frame_name (str), use_iterative_sensing (bool)
  - Returns: success (bool), message (str), correction_values (Vector3)

#### Camera Frame Checking

- **`/pm_skills/check_frame_measureble_cam_top`** (`CheckFrameMeasurable`)
  - Checks if a frame is measurable with top camera
  - Parameters: frame_name (str)
  - Returns: success (bool), is_measurable (bool)

- **`/pm_skills/check_frame_measureble_cam_bottom`** (`CheckFrameMeasurable`)
  - Checks if a frame is measurable with bottom camera
  - Parameters: frame_name (str)
  - Returns: success (bool), is_measurable (bool)

### pm_robot_calibration Node

#### Calibration Services

The calibration node provides comprehensive calibration services. Recommended calibration order:

1. **`/pm_robot_calibration/calibrate_cameras`**
   - Calibrate all camera systems

2. **`/pm_robot_calibration/calibrate_calibration_cube_to_cam_top`**
   - Calibrate calibration cube position relative to top camera

3. **`/pm_robot_calibration/calibrate_laser_on_calibration_cube`**
   - Calibrate laser position using calibration cube

4. **`/pm_robot_calibration/calibrate_confocal_top`**
   - Calibrate top confocal microscope

5. **`/pm_robot_calibration/calibrate_calibration_target_to_cam_bottom`**
   - Calibrate calibration target relative to bottom camera

6. **`/pm_robot_calibration/calibrate_confocal_bottom`**
   - Calibrate bottom confocal microscope

7. **`/pm_robot_calibration/calibrate_gonio_left_chuck`**
   - Calibrate left goniometer chuck

8. **`/pm_robot_calibration/calibrate_gonio_right_chuck`**
   - Calibrate right goniometer chuck

9. **`/pm_robot_calibration/calibrate_gripper`**
   - Calibrate gripper position and orientation

10. **`/pm_robot_calibration/calibrate_1K_dispenser`**
    - Calibrate 1K dispenser position


## Package Structure

```
pm_robot_skills/
├── pm_skills/                          # Core manipulation skills
│   ├── pm_skills/
│   │   ├── pm_skills.py               # Main skills node
│   │   └── py_modules/
│   │       └── PmRobotUtils.py        # Utility functions and robot interface
│   ├── package.xml
│   └── setup.py
├── pm_vision_skills/                   # Vision-based skills
│   ├── pm_vision_skills/
│   │   └── pm_vision_skills.py        # Vision skills node
│   ├── package.xml
│   └── setup.py
├── pm_robot_calibration/               # Calibration services
│   ├── pm_robot_calibration/
│   │   └── pm_robot_calibraion.py     # Calibration node
│   ├── package.xml
│   └── setup.py
├── pm_skills_interfaces/               # Service and message definitions
│   ├── srv/
│   │   ├── GripComponent.srv
│   │   ├── PlaceComponent.srv
│   │   ├── CorrectFrame.srv
│   │   ├── IterativeGonioAlign.srv
│   │   ├── CheckFrameMeasurable.srv
│   │   └── ... (other service definitions)
│   ├── msg/
│   │   └── DispensePoint.msg
│   ├── CMakeLists.txt
│   └── package.xml
├── pm_launch_skills/                   # Launch configurations
│   ├── launch/
│   │   └── pm_skills.launch.py        # Main launch file
│   ├── package.xml
│   └── setup.py
└── README.md                           # This file
```

## Configuration

### Robot Modes

The system supports three operation modes:
- **REAL_MODE**: Real hardware robot
- **GAZEBO_MODE**: Gazebo simulation
- **UNITY_MODE**: Unity simulator

The active mode is automatically detected by `PmRobotUtils`.



## License

TODO: Add appropriate license

## Contact

For questions or issues, please contact the maintainers:
- PM Lab Team
- Email: pmlab@example.com
