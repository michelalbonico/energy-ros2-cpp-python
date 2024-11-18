import sys
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self, timeout_seconds, sleep_seconds):
        super().__init__('fibonacci_action_client')

        self.sleep_duration = sleep_seconds
        self.timeout = timeout_seconds - 5 # Finish before the server
        self.start_time = time.time()

        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

        self._timeout_occurred = False
        self._goal_handle = None
        self._timeout_timer = None

    def send_goal_and_wait(self, order):
        self.get_logger().info(f"Sending goal with order {order}")
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Wait for the action server to be available
        self._action_client.wait_for_server()

        # Create a future to send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        # Create a timer that will trigger after the timeout period
        self._timeout_timer = self.create_timer(self.timeout, self.on_timeout)

        # Wait until the goal is accepted or timeout occurs
        rclpy.spin_until_future_complete(self, self._send_goal_future)

        self._goal_handle = self._send_goal_future.result()

        # If the goal was not accepted, return immediately
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal was not accepted.')
            if self._timeout_timer:
                self._timeout_timer.cancel()
            return False

        # If goal is accepted, we wait for the result asynchronously
        self.get_logger().info("Goal accepted, waiting for result.")
        self._get_result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._get_result_future)

        # If the result is obtained or timeout occurs, stop waiting
        result = self._get_result_future.result()
        if result:
            self.get_logger().info(f"Result: {result.result.sequence}")
        else:
            self.get_logger().info("Result not received due to timeout.")

        # Cancel the timeout timer if we received the result or feedback
        if self._timeout_timer:
            self._timeout_timer.cancel()

        return True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

        # If feedback is received, cancel the timeout timer
        if self._timeout_timer:
            self._timeout_timer.cancel()

    def on_timeout(self):
        self.get_logger().warning('Action timed out.')
        self._timeout_occurred = True

        if self._goal_handle:
            self._goal_handle.cancel_goal_async()

        # Exit the action client logic
        self.get_logger().info('Goal cancelled due to timeout.')

        # Optionally, cancel any other action processes, like result waiting
        if self._get_result_future and not self._get_result_future.done():
            self._get_result_future.cancel()

def main(args=None):
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

    action_client = FibonacciActionClient(timeout_seconds, sleep_seconds)

    while rclpy.ok():
        if not action_client.send_goal_and_wait(10):
            break  # Exit loop if timeout is reached
        time.sleep(action_client.sleep_duration)


if __name__ == '__main__':
    main()
