import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys

class MinimalSubscriber(Node):

    def __init__(self, execution_time, sleep_time):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription 

        self.start_time = time.time()
        self.execution_time = execution_time
        self.sleep_time = sleep_time
        self.message_timeout = 2
        self.last_message_time = time.time()

        self.timer = self.create_timer(sleep_time, self.check_message_timeout)

    def listener_callback(self, msg):
        self.last_message_time = time.time()

        current_time = time.time()
        elapsed_time = current_time - self.start_time
        self.get_logger().info('I heard: "%s"' % msg.data)

    def check_message_timeout(self):

        current_time = time.time()

        elapsed_time = current_time - self.start_time

        if elapsed_time >= self.execution_time:
            self.get_logger().info('Tempo limite atingido, encerrando o nó [sub].')
            sys.exit(0)

        if current_time - self.last_message_time >= self.message_timeout:
            self.get_logger().warn('No messages received for %.2f seconds.' % self.message_timeout)
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Uso: minimal_subscriber.py <tempo_execucao> <tempo_sleep>")
        print("Exemplo: minimal_subscriber.py 300 0.25")
        return

    execution_time = float(sys.argv[1])
    sleep_time = float(sys.argv[2])

    minimal_subscriber = MinimalSubscriber(execution_time, sleep_time)

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
