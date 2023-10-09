import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

#import pyRAPL
#pyRAPL.setup()

class MinimalSubscriber(Node):

    def __init__(self):
        print('subscriber')
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.last_message_time = time.time()

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.last_message_time = time.time()

#@pyRAPL.measureit()
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    while rclpy.ok():
        rclpy.spin_once(minimal_subscriber, timeout_sec=1)
        current_time = time.time()
        if current_time - minimal_subscriber.last_message_time > 2:
            minimal_subscriber.get_logger().info("No messages received for more than 2 seconds. Exiting.")
            break
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()