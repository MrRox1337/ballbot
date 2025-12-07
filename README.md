# Ballbot — ROS 2 Jazzy

Comprehensive guide to install, build, and run the Ballbot project on Ubuntu 24.04 with ROS 2 Jazzy.

**Contents**

-   **Description**: What this repository contains
-   **Prerequisites**: system and account requirements
-   **Install Ubuntu 24.04**: quick notes and recommended steps
-   **Install ROS 2 Jazzy**: apt repository, keys, and packages
-   **Install dependencies**: rosdep, colcon, Gazebo, and Python packages
-   **Build & install this workspace**: cloning, rosdep, and `colcon build`
-   **Per-package launch instructions**: commands for each package
-   **Full-project launch**: recommended bringup order and single-command launches
-   **Troubleshooting & FAQ**n+- **Contributing & License**

---

**Description**:

-   This repository contains a multi-package ROS 2 workspace for the Ballbot demo. Packages in this repo:
    -   `ballbot_bringup` — top-level bringup launch and configuration files
    -   `ballbot_control` — controllers and controller manager launch
    -   `ballbot_description` — URDF/Xacro, meshes and RViz config
    -   `ballbot_gazebo` — Gazebo plugins and world integrations
    -   `ballbot_teleop` — teleoperation node(s)
    -   `ros2_assessment_world` — assessment Gazebo worlds and spawn scripts

**Repository layout (top level)**

```
ballbot_bringup/
ballbot_control/
ballbot_description/
ballbot_gazebo/
ballbot_teleop/
ros2_assessment_world/
```

---

**Prerequisites**

-   A computer with Ubuntu 24.04 installed (64-bit recommended).
-   At least 8 GB RAM recommended; 16 GB for comfortable Gazebo usage.
-   A user account with `sudo` privileges.
-   Internet access to download packages and dependencies.

---

**1) Supported OS and minimal notes**
This guide assumes you already have a supported Ubuntu LTS installed (for example Ubuntu 22.04 or 24.04). Installing an OS is outside the scope of this document — follow the official Ubuntu installation guide if you need to install or upgrade the operating system.

After installing Ubuntu, make sure your system is updated:

```bash
sudo apt update
sudo apt upgrade -y
```

If you need remote access, enable and configure OpenSSH during or after installation.

---

**2) ROS 2 Jazzy — short guidance**
This repository is developed and tested against ROS 2 Jazzy. Follow the official ROS 2 Jazzy install guide for full instructions; below is a concise, minimal set of commands to get a baseline Jazzy install on Ubuntu.

Official ROS 2 docs: https://docs.ros.org

Quick Jazzy install (concise):

```bash
# prerequisites
sudo apt update && sudo apt install -y curl gnupg lsb-release

# add the ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# add the ROS 2 apt repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" |
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# install a typical desktop variant (adjust if you prefer a smaller install)
sudo apt install -y ros-jazzy-desktop python3-argcomplete

# source the distro
source /opt/ros/jazzy/setup.bash

# quick verification
ros2 pkg list | head -n 5
```

Notes:

-   Replace `ros-jazzy-desktop` with a smaller meta-package (for example `ros-jazzy-ros-base`) if you want a minimal install.
-   If a package is not available for your Ubuntu release, consult the official ROS 2 Jazzy release notes and installation docs.

---

**3) Install common development tools & dependencies**

Install `rosdep`, `colcon`, and other development packages used to build and run the workspace.

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-pip python3-vcstool
sudo apt install -y python3-rosdep

# Initialize rosdep (run once)
sudo rosdep init || true
rosdep update
```

Optional useful packages for simulation and control:

```bash
sudo apt install -y gazebo11 libgazebo11-dev # adapt version if needed
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gazebo-plugins
```

Install Python packages that some nodes may need:

```bash
python3 -m pip install --user -U setuptools colcon-common-extensions empy toml
```

Note: Replace `gazebo11` and ROS control packages with the versions compatible with your ROS distro if necessary.

---

**4) Clone this repository & workspace structure**

If you haven't already cloned this repo, do it into a workspace `ros2/ballbot_ws` with a `src` folder. Example recommended structure:

```bash
# from your home directory
mkdir -p ~/ros2/ballbot_ws/src
cd ~/ros2/ballbot_ws/src

# clone the repo (replace URL with your repo's URL)
git clone https://github.com/mrrox1337/ballbot.git

cd ~/ros2/ballbot_ws
```

If this repository already lives in `~/ros2/ballbot_ws/src/` then proceed from the workspace root: `cd ~/ros2/ballbot_ws`.

---

**5) Install package dependencies with `rosdep` and build**

From the workspace root:

```bash
cd ~/ros2/ballbot_ws

# Install system dependencies for packages in src (ignores packages that are in-source)
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source overlay (each new terminal/session)
source install/setup.bash
```

Tips:

-   If a `rosdep` rule fails, inspect the package `package.xml` for missing keys or manually install missing system packages shown by `rosdep`.
-   For iterative development, use `colcon build --symlink-install --packages-select <pkg>` to speed builds.

---

**6) Per-package launch instructions**
Open a terminal, source the ROS distro and the workspace overlay before running any launch or node commands:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2/ballbot_ws/install/setup.bash
```

Below are the per-package commands. Replace paths or filenames if your workspace is in a different location.

-   `ballbot_bringup`

    -   Launch (bringup):

    ```bash
    ros2 launch ballbot_bringup bringup.launch.py
    ```

    -   Purpose: top-level bringup, loads configs, may start controllers and other orchestration.

-   `ballbot_control`

    -   Launch controller manager and controllers:

    ```bash
    ros2 launch ballbot_control ballbot_control.launch.py
    ```

    -   Purpose: starts the controller manager and controller configurations defined in `config/ballbot_controllers.yaml`.

-   `ballbot_description`

    -   Display robot model in RViz / spawn description:

    ```bash
    ros2 launch ballbot_description display.launch.py
    ```

    -   Purpose: publish `robot_description` TF and optionally start RViz with the provided configuration.

-   `ballbot_gazebo`

    -   Start Gazebo world and spawn the robot (if included in the launch):

    ```bash
    ros2 launch ballbot_gazebo ballbot_gazebo.launch.py
    ```

    -   Purpose: integrate the robot model with Gazebo and any plugins required for simulation.

-   `ballbot_teleop`

    -   Teleoperation node (run node directly):

    ```bash
    ros2 run ballbot_teleop teleop_flap_node
    ```

    -   Or, if you have a launch available (none provided in the package), use `ros2 launch` accordingly.

-   `ros2_assessment_world`

    -   Spawn the assessment world:

    ```bash
    ros2 launch ros2_assessment_world assessment_world.launch.py
    ```

    -   Complete assessment scenario launch:

    ```bash
    ros2 launch ros2_assessment_world assessment_complete.launch.py
    ```

    -   Spawn spheres helper (if you need to spawn additional objects):

    ```bash
    ros2 launch ros2_assessment_world spawn_spheres.launch.py
    ```

---

**7) Full-project bringup (recommended sequences)**

Option A — Single bringup

-   If `ballbot_bringup` includes orchestration for Gazebo, controllers and teleop, the simplest bringup is:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2/ballbot_ws/install/setup.bash
ros2 launch ballbot_bringup bringup.launch.py
```

Option B — Manual staged bringup (recommended for debug)

1. Start Gazebo with the world (in one terminal):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2/ballbot_ws/install/setup.bash
ros2 launch ballbot_gazebo ballbot_gazebo.launch.py
```

2. Launch the robot description and RViz (new terminal):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2/ballbot_ws/install/setup.bash
ros2 launch ballbot_description display.launch.py
```

3. Start controllers (new terminal):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2/ballbot_ws/install/setup.bash
ros2 launch ballbot_control ballbot_control.launch.py
```

4. Start teleop or operator nodes (new terminal):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2/ballbot_ws/install/setup.bash
ros2 run ballbot_teleop teleop_flap_node
```

5. If running the assessment world, start it and spawn objects as needed:

```bash
ros2 launch ros2_assessment_world assessment_world.launch.py
ros2 launch ros2_assessment_world spawn_spheres.launch.py
```

This staged approach helps isolate failures (Gazebo, TF, controllers, teleop).

---

**8) Troubleshooting & FAQ**

-   Q: `ros2 launch` fails with package not found
-   A: Ensure you sourced both `/opt/ros/jazzy/setup.bash` and `~/ros2/ballbot_ws/install/setup.bash`. Check that `colcon build` succeeded. Run `ros2 pkg list | grep <package_name>` to confirm package visibility.

-   Q: `rosdep install` cannot find a system dependency
-   A: Inspect the `rosdep` output. Sometimes the `rosdep` key has no mapping for Ubuntu 24.04. You may need to manually install the missing system package or provide a custom `rosdep` mapping.

-   Q: Controller doesn't activate or robot doesn't move in Gazebo
-   A: Confirm controllers are loaded in `controller_manager`: `ros2 service call /controller_manager/list_controllers std_srvs/srv/Trigger` or inspect `ros2 topic` and `ros2 node list`. Check `robot_state_publisher` and joint names in URDF match controller configuration.

-   Q: Gazebo shows no model or spawns fail
-   A: Ensure `robot_description` parameter is being provided and `spawn_entity` is called with correct model SDF/URDF. Check the Gazebo console for plugin errors.

---

**9) Development tips**

-   Use `colcon build --symlink-install` during development to avoid re-copying files.
-   Reload your terminal environment with `source ~/ros2/ballbot_ws/install/setup.bash` after each build.
-   Use `ros2 topic echo`, `ros2 node list`, `ros2 service list`, and `ros2 run rqt_graph rqt_graph` to inspect the system.

---

**10) Contributing**

-   If you add packages or change launch names, update this `README.md` with the new commands.
-   Create PRs against the `main` branch with small, focused changes and a brief description of the behaviour change.

---

**License**

-   Copyright © Aman Mishra | 2025
-   Check each package's `LICENSE` file for licensing details. This repo contains multiple packages; each package may include its own license file.
