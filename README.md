# phi_p3dx_mapping

This repository contains ROS2 packages for mapping and exploration with the Pioneer P3-DX robot in different environments: MobileSim (2D) and physical robot.

## Overview

The `phi_p3dx_mapping` package provides:
- **Mapping nodes**: Occupancy grid creation and processing using odometry and sensor data.
- **Exploration nodes**: Algorithms for autonomous navigation and environment discovery.
- **Multi-platform compatibility**: Works in MobileSim and physical robots.
- **Educational examples**: Commented code in Python and C++ for robotics students.

### Package Structure
- `scripts/`: Python scripts (e.g., `mapping_example.py`, `exploration_example.py`).
- `phi_p3dx_mapping/`: Python base classes.
- `src/`: C++ code (e.g., `mapping_example.cpp`, `mapping_node.cpp`, `exploration_example.cpp`, `exploration_node.cpp`).
- `include/`: C++ headers (e.g., `mapping_node.hpp`, `exploration_node.hpp`).
- `launch/`: Launch files for different environments.

## Prerequisites

- **ROS2 Humble** (or compatible).
- **MobileSim** (for 2D simulation).
- Dependencies: `rclcpp`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2`, `visualization_msgs`.

## Installation

1. **Clone the repository** into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/phir2-lab/phi_p3dx_mapping.git
   ```

2. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select phi_p3dx_mapping
   source install/setup.bash
   ```

## Usage

### MobileSim Simulation (2D)

1. Launch the simulation:
   ```bash
   ros2 launch phi_p3dx_mapping bringup_mobilesim.launch.py map_name:=obstacles
   ```

2. Run the mapping node (Python):
   ```bash
   ros2 run phi_p3dx_mapping mapping_example_py
   ```
   Or in C++:
   ```bash
   ros2 run phi_p3dx_mapping mapping_example_cpp
   ```

3. Run the exploration node (Python):
   ```bash
   ros2 run phi_p3dx_mapping exploration_example_py
   ```
   Or in C++:
   ```bash
   ros2 run phi_p3dx_mapping exploration_example_cpp
   ```

### Real Robot

1. Connect the Pioneer P3-DX robot.

2. Launch the system:
   ```bash
   ros2 launch phi_p3dx_mapping bringup_robot.launch.py
   ```

3. Run the mapping and exploration nodes.



## License

This project is distributed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.
