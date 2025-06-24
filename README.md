## Manipulator Hardware Interface & Controller Library

This project provides a modular hardware interface and control system for a robotic manipulator using ROS 2. Its aim is to help control a device attached to one of the tools of the Da Vinci surgical robot. The device includes four Dynamixel (DXL) motors, each responsible for actuating one joint, and is controlled via an OpenRB-150 microcontroller board.

The system includes:

- A ROS 2 Control hardware interface (`manipulator_hw`) that enables communication between ROS 2 and the attached device, allowing it to be controlled and monitored in real time.
- A control library (`controller_lib`) that helps establish a connection between the OpenRB-150 board and ROS 2. 

Before launching the ROS 2 system, the `arduino_script.ino` must be uploaded to the OpenRB-150 board to initialize the device and prepare it for communication.

## Quick Start

### 1. **Prerequisites**

- ROS 2 Humble
- colcon build tool

### 2. Dependencies

- `rclcpp`
- `rclcpp_lifecycle`
- `hardware_interface`
- `pluginlib`
- `serial_driver`

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
