import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class XboxJointController(Node):
    def __init__(self):
        super().__init__("xbox_controller")  # node
        self.subscriber = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.daVinci_publisher = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )  # or maybe /joint_trajectory_controller/joint_trajectory (not sure of the topic name)

        self.kuka_publisher = self.create_publisher(Twist, "/lbr/command/twist", 10)

        self.joystick_axes = [0.0] * 8
        self.buttons = [0] * 11
        self.previous_buttons = [0] * 11 # to store the previous state of the "A" button

        self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # roll, pitch, jaw_1, jaw_2

        self.timer_period = 0.01  # sec
        self.step_size = 0.1  # deg

        self.mode = "Da Vinci"

        self.timer = self.create_timer(self.timer_period, self.update_mode)

    def joy_callback(self, msg):
        self.joystick_axes = msg.axes
        self.previous_buttons = self.buttons
        self.buttons = msg.buttons

        if self.buttons[0] == 1 and self.previous_buttons[0] == 0:
            if self.mode == "Da Vinci":
                self.mode = "Kuka" 
            else:
                self.mode = "Da Vinci"
            self.get_logger().info(f"Switched to '{self.mode}' mode")

    def update_daVinci_commands(self):
        controller_indexes = [6, 7, 0, 3]  # D-pad left-right, D-pad up-down, left hor, right hor
        signs = [-1.0, 1.0, 1.0, 1.0]

        for i, (index, sign) in enumerate(zip(controller_indexes, signs)):
            controller_values = self.joystick_axes[index]
            self.joint_positions[i] += 100. * self.timer_period * sign * self.step_size * controller_values

        # updated positions
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.daVinci_publisher.publish(msg)

    def update_kuka_commands(self):
        lin_x = self.joystick_axes[0]   # left joystick horizontal
        lin_y = self.joystick_axes[1]   # left joystick vertical
        lin_z = self.joystick_axes[7]   # D-pad up and down
        ang_z = self.joystick_axes[3]   # right joystick horizontal
            
        msg = Twist()
        msg.linear.x = 100. * self.timer_period * 0.1 * lin_x
        msg.linear.y = 100. * self.timer_period * 0.1 * lin_y
        msg.linear.z = 100. * self.timer_period * 0.1 * lin_z
        msg.angular.x = 100. * self.timer_period * 0.0
        msg.angular.y = 100. * self.timer_period * 0.0
        msg.angular.z = 100. * self.timer_period * 0.2 * ang_z
        self.kuka_publisher.publish(msg)

    def update_mode(self):
        if self.mode == "Da Vinci":
            self.update_daVinci_commands()
        elif self.mode == "Kuka":
            self.update_kuka_commands()



def main(args=None):
    rclpy.init(args=args)
    node = XboxJointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
