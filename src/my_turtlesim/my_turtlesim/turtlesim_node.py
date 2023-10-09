import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtlesimNode(Node):
    def __init__(self):
        super().__init__('turtlesim_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.subscription_

    def pose_callback(self, msg):
        self.get_logger().info("Turtle's Pose - X: %f, Y: %f, Theta: %f", msg.x, msg.y, msg.theta)

def main(args=None):
    rclpy.init(args=args)

    turtlesim_node = TurtlesimNode()

    rclpy.spin(turtlesim_node)

    turtlesim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

