# Ballbot IMU Integration - BNO055 Emulation

## Overview

This package adds a simulated **BNO055 Absolute Orientation Sensor** to your Ballbot robot. The BNO055 is a popular 9-DOF (Degrees of Freedom) IMU that provides:

-   **Absolute orientation** (quaternion and Euler angles)
-   **Angular velocity** (from gyroscope)
-   **Linear acceleration** (from accelerometer)
-   **Internal sensor fusion** (combining gyro, accel, and magnetometer)

This implementation emulates the BNO055's behavior in Gazebo simulation, providing realistic sensor characteristics including noise, bias, and covariance values based on the actual BNO055 datasheet.

## What This Package Provides

### 1. **ballbot_imu Package**

-   `imu_processor` node: Processes raw Gazebo IMU data to emulate BNO055 output
-   `imu_filter` node: Optional complementary filter for additional smoothing
-   Configuration files with BNO055-realistic parameters
-   Launch files for easy integration

### 2. **Published Topics**

-   `/imu/data` (sensor_msgs/Imu): Main IMU data with orientation, angular velocity, and linear acceleration
-   `/imu/euler` (geometry_msgs/Vector3Stamped): Euler angles in degrees (roll, pitch, yaw)
-   `/imu/quaternion` (geometry_msgs/QuaternionStamped): Orientation as quaternion
-   `/imu/data_filtered` (sensor_msgs/Imu): Filtered IMU data (if filter enabled)

### 3. **Realistic Sensor Characteristics**

Based on BNO055 specifications:

-   **Orientation accuracy**: ±1° (0.0175 rad)
-   **Gyroscope noise**: 0.014 °/s/√Hz
-   **Accelerometer noise**: 150 µg RMS
-   **Update rate**: Up to 100 Hz
-   **Gaussian noise** and **bias** modeling

## Installation

### Step 1: Copy the Package to Your Workspace

```bash
cd ~/ros2/ballbot_ws/src
cp -r /path/to/ballbot_imu .
```

### Step 2: Update URDF to Add IMU Sensor

Open `~/ros2/ballbot_ws/src/ballbot_description/urdf/ballbot.urdf.xacro` and add the IMU sensor plugin.

**Location**: Add this RIGHT BEFORE the `</robot>` closing tag (after the lidar sensor section):

```xml
<!-- === IMU SENSOR PLUGIN START === -->
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <pose>0 0 0.2 0 0 0</pose>
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>false</visualize>
    <topic>/imu_raw</topic>
    <frame_id>imu_link</frame_id>
    <gz_frame_id>imu_link</gz_frame_id>

    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </z>
      </angular_velocity>

      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
            <bias_mean>0.005</bias_mean>
            <bias_stddev>0.0002</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
            <bias_mean>0.005</bias_mean>
            <bias_stddev>0.0002</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
            <bias_mean>0.005</bias_mean>
            <bias_stddev>0.0002</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>

<!-- IMU Link -->
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
    <material name="grey"/>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
  </collision>
  <xacro:box_inertia m="0.01" x="0.02" y="0.02" z="0.01"/>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
</joint>
<!-- === IMU SENSOR PLUGIN END === -->
```

**Note**: The complete snippet is available in `urdf_imu_snippet.xml` for easy copy-paste.

### Step 3: Build the Package

```bash
cd ~/ros2/ballbot_ws
colcon build --packages-select ballbot_imu --symlink-install
source install/setup.bash
```

## Usage

### Option 1: Standalone IMU Launch

Launch just the IMU nodes:

```bash
ros2 launch ballbot_imu imu.launch.py
```

This will start:

-   Gazebo IMU bridge (gets raw data from simulation)
-   IMU processor (emulates BNO055 processing)
-   IMU filter (optional complementary filter)

**Arguments:**

-   `use_sim_time:=true` (default) - Use simulation time
-   `use_filter:=true` (default) - Enable complementary filter

### Option 2: Integrated with Existing Launches

Add the IMU launch to your existing launch files.

**Example: Add to SLAM launch** (`ballbot_slam/launch/online_async_slam.launch.py`):

```python
from launch.actions import IncludeLaunchDescription

# Add after other includes
imu_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('ballbot_imu'),
            'launch',
            'imu.launch.py'
        ])
    ),
    launch_arguments={'use_sim_time': use_sim_time}.items()
)

# Add to LaunchDescription
ld.add_action(imu_launch)
```

**Example: Add to Nav2 launch** (`ballbot_nav2/launch/navigation.launch.py`):

```python
# Add IMU launch
imu_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('ballbot_imu'),
            'launch',
            'imu.launch.py'
        ])
    ),
    launch_arguments={'use_sim_time': use_sim_time}.items()
)

return LaunchDescription([
    # ... existing launches ...
    imu_launch,
    # ... rest of launches ...
])
```

## Verifying IMU Data

### Check Topics

```bash
# List IMU-related topics
ros2 topic list | grep imu

# Should show:
# /imu_raw
# /imu/data
# /imu/euler
# /imu/quaternion
# /imu/data_filtered (if filter enabled)
```

### Echo IMU Data

```bash
# View processed IMU data
ros2 topic echo /imu/data

# View Euler angles (in degrees)
ros2 topic echo /imu/euler

# View quaternion
ros2 topic echo /imu/quaternion
```

### Visualize in RViz

1. Launch RViz:

```bash
rviz2
```

2. Add displays:

    - **Imu**: Topic = `/imu/data`
    - **TF**: Shows `imu_link` in the transform tree

3. You should see:
    - IMU orientation visualization
    - `imu_link` frame in the TF tree
    - Angular velocity and acceleration arrows (if enabled)

### Plot Data with PlotJuggler

```bash
# Install PlotJuggler if not installed
sudo apt install ros-jazzy-plotjuggler-ros

# Launch PlotJuggler
ros2 run plotjuggler plotjuggler

# In PlotJuggler:
# 1. Streaming > Start: ROS2 Topic Subscriber
# 2. Select /imu/data and /imu/euler
# 3. Drag topics to plot area
```

## How It Works

### Data Flow

```
┌─────────────────┐
│  Gazebo IMU     │  Raw IMU sensor in simulation
│  Sensor Plugin  │  (attached to base_link)
└────────┬────────┘
         │ /imu_raw (sensor_msgs/Imu)
         ▼
┌─────────────────┐
│  ros_gz_bridge  │  Bridges Gazebo to ROS 2
└────────┬────────┘
         │ /imu_raw
         ▼
┌─────────────────┐
│  IMU Processor  │  Emulates BNO055 processing:
│     Node        │  - Adds realistic noise
│                 │  - Applies covariances
│                 │  - Converts to Euler
│                 │  - Optional gravity removal
└────────┬────────┘
         │ /imu/data, /imu/euler, /imu/quaternion
         ▼
┌─────────────────┐
│   IMU Filter    │  (Optional) Complementary filter:
│     Node        │  - Combines gyro + accel
│   (Optional)    │  - Smooths orientation
└────────┬────────┘
         │ /imu/data_filtered
         ▼
┌─────────────────┐
│  Your Nodes     │  SLAM, Nav2, robot_localization, etc.
│  (SLAM, Nav2)   │  Use IMU for better odometry
└─────────────────┘
```

### BNO055 Modes Emulated

The real BNO055 has multiple operating modes. This implementation focuses on:

1. **NDOF Mode** (Nine Degrees of Freedom): Full sensor fusion with absolute orientation

    - Uses accelerometer, gyroscope, and magnetometer (simulated)
    - Provides absolute orientation relative to Earth's magnetic field
    - This is the default mode

2. **IMU Mode**: Combines accelerometer and gyroscope only
    - Enable by setting `remove_gravity: false` in config
    - Provides relative orientation

## Configuration

Edit `config/imu_config.yaml` to customize:

```yaml
imu_processor:
    ros__parameters:
        # Enable/disable gravity compensation
        remove_gravity: false # Set to true for linear acceleration mode

        # Adjust noise levels (increase for more realistic/noisy data)
        noise_stddev_orientation: 0.001 # Orientation noise
        noise_stddev_angular_vel: 0.0002 # Gyro noise
        noise_stddev_linear_accel: 0.01 # Accelerometer noise

        # Update rate (BNO055 supports up to 100Hz)
        publish_rate: 100.0

imu_filter:
    ros__parameters:
        # Complementary filter tuning
        filter_alpha:
            0.98 # 0.98 = trust gyro 98%, accel 2%
            # Increase for faster response
            # Decrease for more stability
```

## Integration with SLAM and Nav2

### Using IMU with SLAM Toolbox

SLAM Toolbox can use IMU data to improve mapping, especially for:

-   Fast rotations
-   Wheel slippage situations
-   More accurate loop closures

**Update** `ballbot_slam/config/slam_async_config.yaml`:

```yaml
slam_toolbox:
    ros__parameters:
        # ... existing config ...

        # Enable IMU usage
        use_imu: true
        imu_topic: /imu/data

        # IMU weight (0.0-1.0, higher = trust IMU more)
        # Start with 0.05 and increase if needed
        imu_weight: 0.05
```

### Using IMU with robot_localization

For even better odometry fusion, use `robot_localization` package:

```bash
sudo apt install ros-jazzy-robot-localization
```

Create a config to fuse:

-   Wheel odometry (`/odom`)
-   IMU data (`/imu/data`)
-   (Optional) GPS, if you add it later

This gives you:

-   Better orientation estimates
-   Reduced drift
-   More robust localization

**Example robot_localization config**:

```yaml
ekf_filter_node:
    ros__parameters:
        frequency: 30.0
        odom0: /odom
        odom0_config: [
                false,
                false,
                false, # x, y, z position
                false,
                false,
                false, # roll, pitch, yaw orientation
                true,
                true,
                false, # x, y, z velocity
                false,
                false,
                true, # roll, pitch, yaw velocity
                false,
                false,
                false,
            ] # x, y, z acceleration

        imu0: /imu/data
        imu0_config: [
                false,
                false,
                false, # Don't use IMU for position
                true,
                true,
                true, # Use for orientation
                false,
                false,
                false, # Don't use for velocity
                true,
                true,
                true, # Use for angular velocity
                false,
                false,
                false,
            ] # Don't use for acceleration
```

### Using IMU with Nav2

Nav2 doesn't directly use IMU, but you can improve its performance by:

1. **Using robot_localization** to fuse wheel odom + IMU → Better odom topic for Nav2
2. **Setting proper odom covariances** based on IMU accuracy

## Troubleshooting

### IMU topics not appearing

**Check:**

1. Is the URDF updated with IMU sensor?

    ```bash
    # Check if imu_link exists in TF
    ros2 run tf2_tools view_frames
    # Open frames.pdf and look for imu_link
    ```

2. Is the bridge running?

    ```bash
    ros2 node list | grep imu_bridge
    ```

3. Is Gazebo publishing IMU data?
    ```bash
    gz topic -l | grep imu
    ```

### IMU data looks wrong

**Check:**

1. Frame orientation - IMU should be mounted with:

    - X = forward
    - Y = left
    - Z = up

2. Noise levels - If data is too noisy, reduce noise in `imu_config.yaml`

3. Covariances - Ensure they match your needs:
    - Smaller covariance = trust more
    - Larger covariance = trust less

### Robot behaves strangely with IMU

1. **Disable filter first**: Set `use_filter:=false`
2. **Check IMU mounting**: Ensure `imu_joint` in URDF has correct pose
3. **Verify data**: Use `rqt_plot` to see if IMU data makes sense
4. **Adjust weights**: In SLAM/robot_localization, reduce IMU weight

## BNO055 vs Simulation Differences

### What's the Same:

✅ Data format (sensor_msgs/Imu)
✅ Update rate (up to 100Hz)
✅ Noise characteristics
✅ Covariance values
✅ Absolute orientation capability

### What's Different:

❌ **No magnetometer**: Simulation doesn't have magnetic field

-   Real BNO055 uses magnetometer for absolute heading
-   Simulation uses Gazebo's absolute orientation instead

❌ **No calibration needed**: Real BNO055 needs calibration

-   Simulation is "pre-calibrated"

❌ **Perfect initialization**: Real BNO055 takes time to converge

-   Simulation provides accurate data immediately

## Advanced Usage

### Simulating Sensor Failure

To test your robot's behavior when IMU fails:

```python
# In your node
self.declare_parameter('simulate_imu_failure', False)
self.declare_parameter('failure_start_time', 10.0)  # seconds
self.declare_parameter('failure_duration', 5.0)     # seconds
```

### Adding Temperature Compensation

Real IMUs drift with temperature. To simulate:

```python
# Add to imu_processor.py
def apply_temperature_drift(self, data, temp=25.0):
    # Simulate drift (BNO055 drift: ~0.03°/°C)
    drift_rate = 0.0005  # rad/°C
    temp_delta = temp - 25.0  # Assume 25°C calibration
    drift = drift_rate * temp_delta
    # Apply drift to yaw...
```

## Real Hardware Integration

When you move from simulation to real BNO055 hardware:

1. **Replace the launch file** to use hardware driver:

    ```bash
    sudo apt install ros-jazzy-bno055
    ```

2. **Update topics** in your config files

3. **Recalibrate**: Real BNO055 needs calibration:

    - Figure-8 motion for magnetometer
    - Place on flat surface for accelerometer
    - Keep still for gyroscope

4. **Adjust covariances** based on real sensor performance

## References

-   [BNO055 Datasheet](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
-   [ROS sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)
-   [Gazebo IMU Sensor](https://gazebosim.org/api/sensors/8/imu.html)
-   [robot_localization](http://docs.ros.org/en/jazzy/p/robot_localization/)

## Support

If you encounter issues:

1. Check Gazebo console for sensor errors
2. Verify topics with `ros2 topic list`
3. Check TF tree with `ros2 run tf2_tools view_frames`
4. Monitor data with PlotJuggler or rqt_plot

---

**Package**: ballbot_imu
**Version**: 0.0.0
**License**: Apache-2.0
**Maintainer**: Aman Mishra
