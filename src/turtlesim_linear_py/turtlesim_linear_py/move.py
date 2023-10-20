import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ScriptTerminationException(Exception):
    pass

class RobotCleaner(Node):

    def __init__(self):
        super().__init__('robot_cleaner')
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.vel_msg = Twist()

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.fwd = True

    def timer_callback(self):
        if self.i < 100:
            # Set fixed values for speed, distance, and direction
            speed = 1.0    # Adjust as needed
            distance = 2.0 # Adjust as needed

            self.move(speed, distance, self.fwd)
            self.fwd = not self.fwd
            time.sleep(0.1)
        else:
            time.sleep(0.1)
            self.timer.cancel()
            raise ScriptTerminationException("Script has completed its task.")
        
        self.i += 1

    def move(self, speed, distance, is_forward):
        if is_forward:
            self.vel_msg.linear.x = abs(speed)
        else:
            self.vel_msg.linear.x = -abs(speed)

        # Since we are moving just in the x-axis
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0

        # Set the current time for distance calculation
        t0 = self.get_clock().now()
        current_distance = 0.0

        # Loop to move the turtle in the specified distance
        while current_distance < distance:
            # Publish the velocity
            self.velocity_publisher.publish(self.vel_msg)
            # Take the actual time for velocity calculation
            t1 = self.get_clock().now()
            # Calculate distance
            current_distance = speed * (t1 - t0).nanoseconds / 1e9

        # After the loop, stop the robot
        self.vel_msg.linear.x = 0.0
        # Force the robot to stop
        self.velocity_publisher.publish(self.vel_msg)

def main(args=None):
    rclpy.init(args=args)
    robot_cleaner = RobotCleaner() 
    try:
        rclpy.spin(RobotCleaner())
    except ScriptTerminationException as e:
        robot_cleaner.get_logger().info("Script terminated.")
    finally:
        # Destroy the node and shutdown ROS
        robot_cleaner.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
