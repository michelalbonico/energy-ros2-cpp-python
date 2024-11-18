import sys
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self, timeout_seconds, sleep_seconds):
        super().__init__('fibonacci_action_server')

        self.sleep_duration = sleep_seconds
        self.timeout = timeout_seconds

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

        self.start_time = self.get_clock().now()
        self.create_timer(self.timeout, self.shutdown_node)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Move somewhere else
        if (self.get_clock().now() - self.start_time).nanoseconds / 1e9 >= self.timeout:
            self.get_logger().info('Timeout reached. Shutting down...')
            sys.exit(0)

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled.')
                goal_handle.canceled()
                return Fibonacci.Result()

            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1]
            )
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')

            goal_handle.publish_feedback(feedback_msg)

            time.sleep(self.sleep_duration)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

    def shutdown_node(self):
        """Encerra o nó ao atingir o timeout."""
        self.get_logger().info('Shutting down node...')
        sys.exit(0)


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

    fibonacci_action_server = FibonacciActionServer(timeout_seconds, sleep_seconds)

    rclpy.spin(fibonacci_action_server)

    fibonacci_action_server.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
