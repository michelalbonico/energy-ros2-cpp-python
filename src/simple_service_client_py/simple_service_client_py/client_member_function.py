import sys
import time
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)

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

    minimal_client = MinimalClientAsync()
    start_time = time.time()

    while time.time() - start_time < timeout_seconds:
        future = minimal_client.send_request(a, b)
        rclpy.spin_until_future_complete(minimal_client, future)
        
        if future.result() is not None:
            response = future.result()
            minimal_client.get_logger().info(
                'Result of add_two_ints: for %d + %d = %d' %
                (a, b, response.sum)
            )
        else:
            minimal_client.get_logger().error('Service call failed.')

        time.sleep(sleep_seconds)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
