import sys
import time
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):
    def __init__(self, a, b, timeout_seconds, sleep_seconds):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.req = AddTwoInts.Request()
        self.req.a = a
        self.req.b = b

        self.start_time = time.time()
        self.timeout_seconds = timeout_seconds
        self.timer = self.create_timer(sleep_seconds, self.timer_callback)

    def timer_callback(self):
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= self.timeout_seconds:
            self.get_logger().info('Timeout reached. Stopping client.')
            sys.exit(0)

        future = self.cli.call_async(self.req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                'Result of add_two_ints: for %d + %d = %d' % 
                (self.req.a, self.req.b, response.sum)
            )
        else:
            self.get_logger().error('Service call failed.')


def main():
    rclpy.init()

    if len(sys.argv) != 5:
        print("Usage: ros2 run your_package your_client_node <int a> <int b> <timeout_seconds> <sleep_seconds>")
        return

    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
        timeout_seconds = int(sys.argv[3])
        sleep_seconds = float(sys.argv[4])
    except ValueError:
        print("Please provide valid integers for a, b and timeout, and a float for sleep.")
        return

    minimal_client = MinimalClientAsync(a, b, timeout_seconds, sleep_seconds)
    rclpy.spin(minimal_client)
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
