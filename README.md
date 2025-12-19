# Ballbot — ROS 2 Jazzy

A comprehensive mobile robot platform for ROS 2 Jazzy with differential drive, LIDAR-based SLAM, and autonomous navigation capabilities.

**Author**: Aman Mishra  
**Email**: amanrox97@gmail.com  
**License**: Apache-2.0  
**ROS Distribution**: Jazzy  
**Simulator**: Gazebo Harmonic

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Robot Description](#robot-description)
3. [Package Structure](#package-structure)
4. [Prerequisites](#prerequisites)
5. [Installation](#installation)
6. [Quick Start](#quick-start)
7. [Package Documentation](#package-documentation)
    - [ballbot_description](#ballbot_description)
    - [ballbot_gazebo](#ballbot_gazebo)
    - [ballbot_control](#ballbot_control)
    - [ballbot_teleop](#ballbot_teleop)
    - [ballbot_slam](#ballbot_slam)
    - [ballbot_nav2](#ballbot_nav2)
    - [ballbot_bringup](#ballbot_bringup)
    - [ballbot_imu (Experimental)](#ballbot_imu-experimental)
8. [Assessment World](#assessment-world)
9. [Usage Examples](#usage-examples)
10. [Troubleshooting](#troubleshooting)
11. [Contributing](#contributing)

---

## Project Overview

Ballbot is a differential-drive mobile robot designed for educational and research purposes in robotics. The project demonstrates a complete robotics software stack including:

-   **Robot Modeling**: URDF/Xacro-based robot description with accurate physics
-   **Simulation**: Full Gazebo Harmonic integration with sensors and actuators
-   **Control**: ros2_control-based differential drive and position-controlled manipulator flaps
-   **Teleoperation**: Custom keyboard teleoperation for driving and flap control
-   **SLAM**: Real-time mapping using slam_toolbox
-   **Navigation**: Autonomous navigation using Nav2 stack with AMCL localization

### Key Features

-   Differential drive locomotion with two powered wheels and a passive castor
-   Dual articulated "collector arms" with controllable flaps for object manipulation
-   360° LIDAR sensor for mapping and obstacle detection
-   IMU sensor integration (experimental)
-   Support for both empty worlds and the PDE4430 assessment environment

---

## Robot Description

### Physical Specifications

| Component        | Dimension                 | Description                       |
| ---------------- | ------------------------- | --------------------------------- |
| Chassis          | 0.30m × 0.65m × 0.30m     | Main body (blue)                  |
| Drive Wheels     | 0.05m radius, 0.04m width | Left and right powered wheels     |
| Wheel Separation | 0.54m                     | Distance between wheel centers    |
| Castor Wheel     | 0.04m radius              | Passive rear support              |
| Collector Arms   | 0.60m length              | Fixed arms extending forward      |
| Flaps            | 0.25m length              | Articulated end-effectors on arms |
| LIDAR            | 360° scan, 10m range      | Mounted on vertical pole          |

### Coordinate Frame

-   **base_footprint**: Ground projection of the robot center
-   **base_link**: Main chassis frame
-   **odom**: Odometry frame (from wheel encoders)
-   **map**: Global map frame (from SLAM/localization)

### TF Tree

```
map
└── odom
    └── base_footprint
        └── base_link
            ├── left_wheel
            ├── right_wheel
            ├── castor_wheel
            ├── left_arm
            │   └── left_flap
            ├── right_arm
            │   └── right_flap
            ├── vertical_pole
            │   └── lidar_link
            └── imu_link
```

---

## Package Structure

```
ballbot_ws/src/
├── ballbot_bringup/          # Top-level launch orchestration
├── ballbot_control/          # ros2_control configuration
├── ballbot_description/      # URDF/Xacro and meshes
├── ballbot_gazebo/           # Gazebo simulation launch
├── ballbot_imu/              # IMU integration (experimental)
├── ballbot_nav2/             # Nav2 configuration and launch
├── ballbot_slam/             # SLAM toolbox configuration
├── ballbot_teleop/           # Teleoperation nodes
└── ros2_assessment_world/    # Assessment environment
```

---

## Prerequisites

### System Requirements

-   **OS**: Ubuntu 24.04 LTS (Noble Numbat)
-   **RAM**: Minimum 8GB, recommended 16GB
-   **GPU**: Discrete GPU recommended for Gazebo rendering

### Software Dependencies

-   ROS 2 Jazzy Jalisco
-   Gazebo Harmonic
-   Python 3.12+

---

## Installation

### 1. Install ROS 2 Jazzy

```bash
# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Desktop
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-argcomplete
```

### 2. Install Additional Dependencies

```bash
# Development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# Gazebo and ROS-Gazebo integration
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control

# Control packages
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers

# Navigation and SLAM
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install -y ros-jazzy-slam-toolbox

# Visualization
sudo apt install -y ros-jazzy-rviz2 ros-jazzy-joint-state-publisher

# Initialize rosdep
sudo rosdep init || true
rosdep update
```

### 3. Clone and Build Workspace

```bash
# Create workspace
mkdir -p ~/ros2/ballbot_ws/src
cd ~/ros2/ballbot_ws/src

# Clone repository (replace with your repository URL)
git clone https://github.com/mrrox1337/ballbot.git .

# Install package dependencies
cd ~/ros2/ballbot_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### 4. Add to Shell Configuration (Optional)

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2/ballbot_ws/install/setup.bash" >> ~/.bashrc
```

---

## Quick Start

### Launch Everything with Default Settings

The simplest way to run the complete system:

```bash
# Terminal 1: Launch simulation with SLAM
ros2 launch ballbot_bringup bringup.launch.py slam:=true teleop:=true

# Drive around to create a map, then save it
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### Launch Navigation with Pre-built Map

```bash
# Terminal 1: Launch simulation with navigation
ros2 launch ballbot_bringup bringup.launch.py navigation:=true

# Terminal 2: After setting 2D Pose Estimate in RViz, run commander
ros2 run ballbot_nav2 simple_commander
```

---

## Package Documentation

### ballbot_description

**Purpose**: Defines the robot's physical structure, visual appearance, and sensor configuration.

**Key Files**:

-   `urdf/ballbot.urdf.xacro` - Main robot description file
-   `rviz/rviz.rviz` - RViz visualization configuration
-   `launch/display.launch.py` - Standalone visualization launch

**Topics Published**:

-   `/robot_description` (std_msgs/String) - URDF as string parameter

**Standalone Usage**:

```bash
# View robot in RViz (no simulation)
ros2 launch ballbot_description display.launch.py
```

---

### ballbot_gazebo

**Purpose**: Integrates the robot with Gazebo Harmonic simulation environment.

**Key Files**:

-   `launch/ballbot_gazebo.launch.py` - Simulation launch file

**Launch Arguments**:
| Argument | Default | Description |
|----------|---------|-------------|
| `world_type` | `empty` | World to load: `empty` or `assessment` |
| `use_sim_time` | `true` | Use simulation clock |

**Topics Bridged**:

-   `/clock` - Simulation time
-   `/scan` - LIDAR data (bridged separately in SLAM/Nav launches)

**Standalone Usage**:

```bash
# Launch with empty world
ros2 launch ballbot_gazebo ballbot_gazebo.launch.py world_type:=empty

# Launch with assessment world
ros2 launch ballbot_gazebo ballbot_gazebo.launch.py world_type:=assessment
```

---

### ballbot_control

**Purpose**: Configures ros2_control for differential drive and flap position control.

**Key Files**:

-   `config/ballbot_controllers.yaml` - Controller parameters
-   `launch/ballbot_control.launch.py` - Controller spawning launch

**Controllers**:
| Controller | Type | Interface |
|------------|------|-----------|
| `joint_state_broadcaster` | JointStateBroadcaster | State publishing |
| `diff_drive_base_controller` | DiffDriveController | Velocity command |
| `flap_controller` | ForwardCommandController | Position command |

**Command Topics**:

-   `/diff_drive_base_controller/cmd_vel` (geometry_msgs/TwistStamped) - Drive commands
-   `/flap_controller/commands` (std_msgs/Float64MultiArray) - Flap positions

**State Topics**:

-   `/joint_states` (sensor_msgs/JointState) - All joint states
-   `/odom` (nav_msgs/Odometry) - Wheel odometry

**Standalone Usage**:

```bash
# Launch Gazebo + Controllers
ros2 launch ballbot_control ballbot_control.launch.py world_type:=assessment
```

---

### ballbot_teleop

**Purpose**: Provides keyboard-based teleoperation for driving and flap control.

**Key Files**:

-   `ballbot_teleop/teleop_flap_node.py` - Main teleoperation node

**Controls**:

```
Moving:
   w
 a s d
   x

q/z : increase/decrease max speeds by 10%
w/x : linear movement (forward/back)
a/d : angular movement (left/right)
s   : force stop

Flap Controls:
1 : Reset Flaps (0.0, 0.0)
2 : Open Flaps  (-0.5, 0.5)
3 : Close/Grip  (-1.57, 1.57)

CTRL-C to quit
```

**Standalone Usage**:

```bash
# Run teleop node (requires controllers to be running)
ros2 run ballbot_teleop teleop_flap_node
```

---

### ballbot_slam

**Purpose**: Performs simultaneous localization and mapping using slam_toolbox.

**Key Files**:

-   `config/slam_async_config.yaml` - SLAM parameters
-   `launch/online_async_slam.launch.py` - Complete SLAM launch
-   `maps/assessment_map.yaml` - Pre-built assessment world map
-   `maps/assessment_map.pgm` - Map image file

**SLAM Configuration**:

-   **Mode**: Asynchronous online SLAM
-   **Resolution**: 0.05m per pixel
-   **Laser Range**: 0.1m to 10.0m
-   **Map Update Interval**: 2.0 seconds

**Topics**:

-   `/map` (nav_msgs/OccupancyGrid) - Generated map
-   `/scan` (sensor_msgs/LaserScan) - LIDAR input

**Standalone Usage**:

```bash
# Launch complete SLAM system
ros2 launch ballbot_slam online_async_slam.launch.py world_type:=assessment

# Save the map after exploring
ros2 run nav2_map_server map_saver_cli -f ~/ros2/ballbot_ws/src/ballbot_slam/maps/my_map
```

---

### ballbot_nav2

**Purpose**: Provides autonomous navigation using the Nav2 stack.

**Key Files**:

-   `config/ballbot_nav2_params.yaml` - Complete Nav2 parameters
-   `launch/navigation.launch.py` - Navigation stack launch
-   `ballbot_nav2/cmd_vel_relay.py` - Twist to TwistStamped converter
-   `ballbot_nav2/simple_commander.py` - Example goal sender

**Nav2 Components**:

-   **Localization**: AMCL (Adaptive Monte Carlo Localization)
-   **Planning**: NavFn global planner
-   **Control**: DWB local planner
-   **Recovery**: Spin, backup, wait behaviors

**Robot Footprint**:

```
[[1.0, 0.35], [1.0, -0.35], [-0.27, -0.35], [-0.27, 0.35]]
```

This accounts for the extended arms and flaps.

**Important**: After launching navigation, you **must** set a 2D Pose Estimate in RViz before the robot can navigate. The AMCL localization needs an initial pose to start.

**Standalone Usage**:

```bash
# Terminal 1: Launch navigation
ros2 launch ballbot_nav2 navigation.launch.py

# In RViz: Click "2D Pose Estimate" and set robot's approximate position

# Terminal 2: Send navigation goal (after setting pose estimate)
ros2 run ballbot_nav2 simple_commander
```

---

### ballbot_bringup

**Purpose**: Unified launch system for all robot configurations.

**Key Files**:

-   `launch/bringup.launch.py` - Master launch file

**Launch Arguments**:

| Argument       | Default      | Description                            |
| -------------- | ------------ | -------------------------------------- |
| `world`        | `assessment` | World to load: `assessment` or `empty` |
| `teleop`       | `false`      | Launch teleoperation in new terminal   |
| `slam`         | `false`      | Enable SLAM mapping                    |
| `navigation`   | `false`      | Enable Nav2 navigation stack           |
| `use_sim_time` | `true`       | Use simulation time                    |

**Usage Examples**:

```bash
# Basic simulation (just robot in assessment world)
ros2 launch ballbot_bringup bringup.launch.py

# Simulation with teleoperation
ros2 launch ballbot_bringup bringup.launch.py teleop:=true

# SLAM mapping session
ros2 launch ballbot_bringup bringup.launch.py slam:=true teleop:=true

# Autonomous navigation (remember to set 2D Pose Estimate!)
ros2 launch ballbot_bringup bringup.launch.py navigation:=true

# Empty world for testing
ros2 launch ballbot_bringup bringup.launch.py world:=empty teleop:=true
```

**Important Notes**:

1. `slam` and `navigation` are mutually exclusive - don't enable both
2. When using `navigation`, manually set 2D Pose Estimate in RViz before sending goals
3. The `simple_commander` node should be run manually after navigation is ready

---

### ballbot_imu (Experimental)

**Status**: ⚠️ **Experimental - Not integrated into main system**

**Purpose**: Attempted integration of IMU sensor fusion for improved localization.

This package was developed to emulate a BNO055 absolute orientation sensor and fuse IMU data with wheel odometry for more robust localization. While the sensor simulation works, the full integration with the localization stack was not completed.

**What Works**:

-   IMU sensor plugin in URDF
-   Gazebo IMU bridge
-   IMU processor node emulating BNO055 characteristics
-   Complementary filter for sensor fusion

**What's Not Integrated**:

-   robot_localization EKF fusion
-   Integration with SLAM or Nav2

**For Future Development**:
The `ballbot_imu` package can serve as a starting point for:

-   Implementing robot_localization with IMU + wheel odometry fusion
-   Adding IMU-based orientation estimation for rough terrain
-   Sensor fusion research and experimentation

---

## Assessment World

The `ros2_assessment_world` package provides the PDE4430 assessment environment.

### Environment Specifications

-   **Arena Size**: 8m × 8m enclosed area
-   **Wall Height**: 2m
-   **Pen Areas**: Two collection zones at coordinates (±0.6, 3.3)
-   **Obstacles**: 9 cylindrical obstacles of varying sizes (0.2m - 0.5m diameter)

### Spheres

Three colored spheres spawn at random locations:

-   **Red (Small)**: 0.1m radius, 0.3kg
-   **Green (Medium)**: 0.2m radius, 0.6kg
-   **Blue (Large)**: 0.3m radius, 1.0kg

### Assessment Task

1. Navigate the environment while avoiding obstacles
2. Locate and collect the three spheres
3. Transport spheres to the pen areas using the flap mechanism

### Launching Assessment World

```bash
# Via bringup (recommended)
ros2 launch ballbot_bringup bringup.launch.py world:=assessment

# Standalone (world only, no robot)
ros2 launch assessment_world assessment_complete.launch.py
```

---

## Usage Examples

### Example 1: First-Time Setup and Testing

```bash
# Terminal 1: Launch basic simulation
ros2 launch ballbot_bringup bringup.launch.py teleop:=true

# Use WASD keys to drive, 1/2/3 to control flaps
# Press Ctrl+C to exit
```

### Example 2: Creating a New Map

```bash
# Terminal 1: Launch SLAM
ros2 launch ballbot_bringup bringup.launch.py slam:=true teleop:=true

# Drive around the entire environment to build a complete map
# Watch the map build in RViz

# Terminal 2: Save the map when done
ros2 run nav2_map_server map_saver_cli -f ~/ros2/ballbot_ws/src/ballbot_slam/maps/new_map

# Update ballbot_nav2/config/ballbot_nav2_params.yaml with new map path if needed
```

### Example 3: Autonomous Navigation

```bash
# Terminal 1: Launch navigation
ros2 launch ballbot_bringup bringup.launch.py navigation:=true

# In RViz:
# 1. Click "2D Pose Estimate" button
# 2. Click and drag on the map where the robot actually is
# 3. Wait for AMCL particle cloud to converge

# Terminal 2: Send a navigation goal
ros2 run ballbot_nav2 simple_commander

# Or use RViz "2D Goal Pose" button to send goals interactively
```

### Example 4: Manual Navigation Goal via CLI

```bash
# After navigation is running and localized:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## Troubleshooting

### Common Issues

#### Gazebo doesn't start or crashes

```bash
# Check Gazebo installation
gz sim --version

# Try with OGRE rendering (more compatible)
# This is already set in the launch files for assessment world
```

#### Robot doesn't move with teleop

1. Ensure controllers are loaded:
    ```bash
    ros2 control list_controllers
    ```
2. Check that topics are connected:
    ```bash
    ros2 topic list | grep cmd_vel
    ros2 topic echo /diff_drive_base_controller/cmd_vel
    ```

#### SLAM map not building

1. Verify LIDAR data:
    ```bash
    ros2 topic echo /scan --once
    ```
2. Check TF tree:
    ```bash
    ros2 run tf2_tools view_frames
    ```

#### Navigation fails to start

1. Ensure map file exists at the configured path
2. Check lifecycle states:
    ```bash
    ros2 lifecycle list /map_server
    ros2 lifecycle list /amcl
    ```

#### Robot spins or behaves erratically during navigation

1. **Always set 2D Pose Estimate first** - AMCL needs initial localization
2. Check that the map matches the current world
3. Verify odometry is working:
    ```bash
    ros2 topic echo /odom
    ```

### Useful Debugging Commands

```bash
# List all active nodes
ros2 node list

# List all topics
ros2 topic list

# Check topic frequency
ros2 topic hz /scan

# View TF tree
ros2 run tf2_tools view_frames

# Monitor transforms
ros2 run tf2_ros tf2_echo map base_footprint

# Check controller status
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

---

## Contributing

### Development Workflow

1. Create a feature branch
2. Make changes with `--symlink-install` for faster iteration:
    ```bash
    colcon build --symlink-install --packages-select <package_name>
    ```
3. Test thoroughly
4. Update documentation
5. Submit pull request

### Code Style

-   Python: Follow PEP 8, use type hints where practical
-   Launch files: Use Python launch format
-   YAML: Use consistent 4-space indentation

### Adding New Packages

1. Create package with `ament_python` build type
2. Add appropriate dependencies to `package.xml`
3. Include launch files and configs in `setup.py` data_files
4. Update this README with package documentation

---

## License

This project is licensed under the Apache License 2.0. See individual package LICENSE files for details.

---

## Acknowledgments

-   ROS 2 Community
-   Gazebo Development Team
-   Nav2 Project Contributors
-   slam_toolbox Developers
-   Middlesex University Dubai - PDE4430 Robotics Course

---

**Last Updated**: December 2025  
**ROS 2 Version**: Jazzy Jalisco  
**Gazebo Version**: Harmonic
