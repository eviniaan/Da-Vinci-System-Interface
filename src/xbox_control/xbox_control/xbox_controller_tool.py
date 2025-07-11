import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class XboxJointController(Node):
    def __init__(self):
        super().__init__("xbox_joint_controller")  # node
        self.subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.publisher = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )  # or maybe /joint_trajectory_controller/joint_trajectory (not sure of the topic name)

        self.joystick_axes = [0.0] * 8
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # roll, pitch, jaw_1, jaw_2

        self.timer_period = 0.05  # sec
        self.step_size = 0.8  # deg

        self.timer = self.create_timer(self.timer_period, self.update_joint_commands)

    def joy_callback(self, msg):
        self.joystick_axes = msg.axes

    def update_joint_commands(self):
        axis_map = [6, 7, 0, 3]  # D-pad left-right, D-pad up-down, left horiz, right horiz

        for i, axis_index in enumerate(axis_map):
            axis_value = self.joystick_axes[axis_index]
            self.joint_positions[i] += self.step_size * axis_value

        # updated positions
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = XboxJointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
