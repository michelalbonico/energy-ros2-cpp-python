import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys


class MinimalPublisher(Node):

    def __init__(self, exec_time=300, sleep_time=0.25):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = sleep_time
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.start_time = time.time()

        self.exec_time = exec_time
        self.sleep_time = sleep_time

    def timer_callback(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        if elapsed_time >= self.exec_time:
            self.get_logger().info('Tempo limite atingido, encerrando o nó [pub].')
            sys.exit(0)

        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

        time.sleep(self.sleep_time)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) >= 3:
        exec_time = float(sys.argv[1])
        sleep_time = float(sys.argv[2])
    else:
        exec_time = 300
        sleep_time = 0.25

    minimal_publisher = MinimalPublisher(exec_time=exec_time, sleep_time=sleep_time)

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
