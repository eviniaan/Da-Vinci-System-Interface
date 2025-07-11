## Manipulator Hardware Interface & Controller Library

This project provides a modular hardware interface and control system for a robotic manipulator using ROS 2. Its aim is to help control a device attached to one of the tools of the Da Vinci surgical robot. The device includes four Dynamixel (DXL) motors, each responsible for actuating one joint, and is controlled via an OpenRB-150 microcontroller board.

The system includes:

- A ROS 2 Control hardware interface (`manipulator_hw`) that enables communication between ROS 2 and the attached device, allowing it to be controlled and monitored in real time.
- A control library (`controller_lib`) that helps establish a connection between the OpenRB-150 board and ROS 2.
- The ``xbox_control``package that contains two scripts that assist in controlling the da Vinci instrument either independently (``xbox_control_tool.py``) or in combination with the Kuka Med7 robotic arm (``xbox_controller_combo.py``).

Before launching the ROS 2 system, the `arduino_script.ino` must be uploaded to the OpenRB-150 board to initialize the device and prepare it for communication.

## Game Controller Package

The ``xbox_control`` pakage enables controlling the device using a game controller.

#### How it works

- When running `xbox_control_tool.py` , you can control the da Vinci instrument using the left and right sticks, as well as the D-pad on the controller.
- When running ``xbox_control_combo.py`` you can control both the da Vinci instrument and the end effector of the Kuka Med7 robotic arm. Pressing the “``A``” button switches between two control modes:
  - ***Instrument control mode:*** The da Vinci instrument is controlled similarly to `xbox_control_tool.py`, using the left and right sticks and D-pad.
  - ***Arm control mode:*** The Kuka Med7 arm can be controlled with velocity commands:
    - Linear X and Y: Left stick
    - Linear Z:D-pad (up-down)
    - Angular Z: Right stick

## Quick Start

### 1. **Prerequisites**

- ROS 2 Humble
- colcon build tool

### 2. Dependencies

- ``rclcpp`` and ``rclpy``
- `hardware_interface`
- `pluginlib`
- ``asio``
- `serial_driver`
- ``sensor_msgs`` and ``std_msgs``

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

#### 4.1 Launch the hardware interface

```bash
ros2 launch manipulator_hw test_manip_hw.launch.py
```

#### 4.2 Control the da Vinci instrument with a game controller

**Terminal 1:** Start the joystick input node

```bash
ros2 run joy joy_node
```

**Terminal 2:** Launch the hardware interface

```bash
ros2 launch manipulator_hw test_manip_hw.launch.py
```

**Terminal 3:** Run the Xbox controller node

```bash
ros2 run xbox_control xbox_control_tool	# just for the instrument

# or

ros2 run xbox_control xbox_control_combo # for both the instrument and the arm
```
