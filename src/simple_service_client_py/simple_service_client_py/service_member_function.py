import sys
import time
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalService(Node):
    def __init__(self, timeout_seconds, sleep_seconds):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.timeout_seconds = timeout_seconds
        self.sleep_seconds = sleep_seconds
        
        self.timer = self.create_timer(1.0, self.timer_callback)

    def add_two_ints_callback(self, request, response):
        time.sleep(self.sleep_seconds)

        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        return response

    def timer_callback(self):
        elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        if elapsed_time >= self.timeout_seconds:
            self.get_logger().info("Timeout reached. Shutting down the server.")
            sys.exit(0)


def main():
    rclpy.init()

    if len(sys.argv) != 3:
        print("Usage: ros2 run your_package your_server_node <timeout_seconds> <sleep_seconds>")
        return

    try:
        timeout_seconds = int(sys.argv[1])
        sleep_seconds = float(sys.argv[2])
    except ValueError:
        print("Please provide valid integer for timeout and float for sleep.")
        return

    minimal_service = MinimalService(timeout_seconds, sleep_seconds)
    rclpy.spin(minimal_service)

    minimal_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
