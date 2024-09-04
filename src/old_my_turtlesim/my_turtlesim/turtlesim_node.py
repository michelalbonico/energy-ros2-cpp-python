import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from threading import Timer
import time

class MyTurtlesimNode(Node):
    def __init__(self):
        super().__init__('my_turtlesim_node')
        
        # Create a subscriber for the turtle's velocity command
        self.subscriber = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.last_message_time = time.time()

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        log_message = "Received Velocity Command - Linear X: " + str(linear_x) + ", Angular Z: " + str(angular_z)
        self.get_logger().info(log_message)
        self.last_message_time = time.time()
        # Implement your turtle control logic here based on the received velocity commands

def main(args=None):
    rclpy.init(args=args)
    node = MyTurtlesimNode()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1)
        current_time = time.time()
        if current_time - node.last_message_time > 2:
            node.get_logger().info("No messages received for more than 2 seconds. Exiting.")
            break
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
