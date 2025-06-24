## Manipulator Hardware Interface & Controller Library

This project provides a modular hardware interface and control system for a robotic manipulator using ROS 2. It includes a hardware abstraction layer (`manipulator_hw`) and a reusable control library (`controller_lib`).

---

## Quick Start

### 1. **Prerequisites**

- ROS 2 Humble
- colcon build tool

### 2. Dependencies

Ensure these ROS 2 packages are installed before building:

- `rclcpp`
- `rclcpp_lifecycle`
- `hardware_interface`
- `pluginlib`
- `serial_driver`

Install them using:

```bash
sudo apt update
sudo apt install \
  ros-${ROS_DISTRO}-rclcpp \
  ros-${ROS_DISTRO}-rclcpp-lifecycle \
  ros-${ROS_DISTRO}-hardware-interface \
  ros-${ROS_DISTRO}-pluginlib \
  ros-${ROS_DISTRO}-serial-driver
```

### 3. Clone & **Build**

```bash
# Create the workspace
mkdir -p ros2_ws/src && cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/eviniaan/Interface.git

# Build
cd ~/ros2_ws
colcon build --symlink-install

# Source the setup script
source install/setup.bash
```

### 4. Run

```bash
ros2 launch manipulator_hw test_manip_hw.launch.py
```
