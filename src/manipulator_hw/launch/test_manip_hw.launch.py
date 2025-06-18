from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command

def generate_launch_description():
    robot_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("manipulator_hw"),
            "config",
            "manipulator_hw.yaml",
        ]
    )

    # should probably create the urdf description and add the path to it
    robot_description_path = PathJoinSubstitution(
        [
            FindPackageShare("manipulator_hw"),
            "urdf",
            "robot.urdf.xacro",
        ]
    )

    robot_description = {"robot_description": Command(['xacro', " ", robot_description_path])}

    # the following manages the device's controllers
    # namespace = "da_vinci_tool"
    manipulator_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controller_yaml],
        remappings=[("~/robot_description", "robot_description")],
        output="screen",
        # namespace=namespace,
    )

    # this controller is spawned via the controller_manager and it publishes joint states from the hw interface to the /joint_states topic
    # this uses the URDF
    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster"],
    #     parameters=[robot_description],
    # )

    # might also need a robot state publisher node to publish the tfs
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
        # namespace=namespace
    )

    return LaunchDescription([
        manipulator_control,
        # joint_state_broadcaster_spawner,
        robot_state_publisher
    ])