import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')

        self.declare_parameter('sleep_duration', 0.25)
        self.declare_parameter('timeout', 300.0)

        self.sleep_duration = self.get_parameter('sleep_duration').value
        self.timeout = self.get_parameter('timeout').value

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

        while rclpy.ok():
            if (self.get_clock().now() - self.start_time).nanoseconds / 1e9 >= self.timeout:
                self.get_logger().info('Timeout reached. Shutting down...')
                break

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

            feedback_msg.partial_sequence = [0, 1]

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

    def shutdown_node(self):
        """Encerra o nó ao atingir o timeout."""
        self.get_logger().info('Shutting down node...')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
