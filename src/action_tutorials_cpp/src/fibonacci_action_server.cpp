#include "rclcpp/rclcpp.hpp"
#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/action/server.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

class FibonacciActionServer : public rclcpp::Node
{
public:
  FibonacciActionServer()
  : Node("fibonacci_action_server")
  {
    // Declare parameters
    this->declare_parameter<double>("sleep_duration", 0.25);
    this->declare_parameter<double>("timeout", 300.0);

    // Get parameters
    sleep_duration_ = this->get_parameter("sleep_duration").as_double();
    timeout_ = this->get_parameter("timeout").as_double();

    // Create action server
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FibonacciActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FibonacciActionServer::handle_accepted, this, std::placeholders::_1));

    // Set the start time
    start_time_ = this->now();

    // Create a timer for shutdown after timeout
    shutdown_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(timeout_),
      std::bind(&FibonacciActionServer::shutdown_node, this));
  }

private:
  // Action server callback when goal is accepted
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order: %ld", goal->order);
    (void)uuid;  // unused variable
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Action server callback when goal is canceled
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Action server callback for executing the goal
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    std::thread([this, goal_handle]() {
      this->execute(goal_handle);
    }).detach();
  }

  // Function to execute the Fibonacci sequence
  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal...");

    auto feedback_msg = std::make_shared<Fibonacci::Feedback>();
    feedback_msg->partial_sequence = {0, 1};

    auto start_time = this->now();

    for (int i = 1; rclcpp::ok(); ++i)
    {
      // Check timeout
      if ((this->now() - start_time).seconds() >= timeout_)
      {
        RCLCPP_INFO(this->get_logger(), "Timeout reached. Shutting down...");
        break;
      }

      // Generate Fibonacci sequence
      for (int j = 1; j < goal_handle->get_goal()->order; ++j)
      {
        if (goal_handle->is_canceling())
        {
          RCLCPP_INFO(this->get_logger(), "Goal canceled.");
          goal_handle->canceled();
          return;
        }

        feedback_msg->partial_sequence.push_back(
          feedback_msg->partial_sequence[j] + feedback_msg->partial_sequence[j - 1]);

        RCLCPP_INFO(this->get_logger(), "Feedback: %d", feedback_msg->partial_sequence.back());

        goal_handle->publish_feedback(feedback_msg);

        // Sleep to simulate processing
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_duration_));
      }

      feedback_msg->partial_sequence = {0, 1};
    }

    // Send the result back
    goal_handle->succeed();
    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = feedback_msg->partial_sequence;
    goal_handle->set_result(result);
  }

  // Shutdown the node after timeout
  void shutdown_node()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down node...");
    rclcpp::shutdown();
  }

  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
  rclcpp::Time start_time_;
  double sleep_duration_;
  double timeout_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto fibonacci_action_server = std::make_shared<FibonacciActionServer>();

  rclcpp::spin(fibonacci_action_server);

  rclcpp::shutdown();
  return 0;
}
