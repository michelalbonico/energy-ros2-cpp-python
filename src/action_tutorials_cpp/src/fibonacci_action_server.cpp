#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

namespace action_tutorials_cpp
{
class FibonacciActionServer : public rclcpp::Node
{
public:

  // public:
  // MinimalPublisher(double execution_duration, double sleep_duration)
  // : Node("minimal_publisher"), count_(0), execution_duration_(execution_duration), sleep_duration_(sleep_duration)
  // {
  //   start_time_ = this->now();
    
  //   publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    
  //   timer_ = this->create_wall_timer(
  //     std::chrono::milliseconds(static_cast<int>(sleep_duration_ * 1000)),
  //     std::bind(&MinimalPublisher::timer_callback, this));
  // }

  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  ACTION_TUTORIALS_CPP_PUBLIC
  explicit FibonacciActionServer(double execution_duration, double sleep_duration)
  : Node("fibonacci_action_server", options, execution_duration_(execution_duration), sleep_duration_(sleep_duration))
  {
    using namespace std::placeholders;

    // this->declare_parameter<double>("sleep_duration", 0.25);
    // this->declare_parameter<double>("timeout", 300.0);

    // this->get_parameter("sleep_duration", sleep_duration);
    // this->get_parameter("timeout", execution_duration);

    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(timeout_ * 1000)),
      std::bind(&FibonacciActionServer::shutdown_node, this));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;

  double sleep_duration_;
  double timeout_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;

    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    rclcpp::Time start_time = this->now();
    while (rclcpp::ok()){
     for (int i = 2; (i < goal->order) && rclcpp::ok(); ++i) {
        // Verifica se o tempo limite foi atingido
        if ((this->now() - start_time).seconds() >= timeout_) {
         RCLCPP_INFO(this->get_logger(), "Timeout reached. Shutting down...");
         result->sequence = sequence;
         goal_handle->succeed(result);
         return;
       }

        if (goal_handle->is_canceling()) {
          result->sequence = sequence;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }

        sequence.push_back(sequence[i - 1] + sequence[i - 2]);
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Published feedback: %d", sequence.back());

        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_duration_));
      }
      sequence = std::vector<int>();
      sequence.push_back(0);
      sequence.push_back(1);

    }

    result->sequence = sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }

  void shutdown_node()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down node after timeout.");
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  double execution_duration = 300.0;
  double sleep_duration = 0.25;

  if (argc >= 3) {
    execution_duration = std::atof(argv[1]);
    sleep_duration = std::atof(argv[2]);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Usando valores padrão: execution_duration = 300.0, sleep_duration = 0.25");
  }

  auto node = std::make_shared<action_tutorials_cpp::FibonacciActionServer>(execution_duration, sleep_duration);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


}  

// RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)


