#include <chrono>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_tutorials_interfaces/action/fibonacci.hpp"

using namespace std::chrono_literals;
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

class FibonacciActionServer : public rclcpp::Node {
public:
    FibonacciActionServer(int timeout_seconds, double sleep_seconds)
        : Node("fibonacci_action_server"),
          timeout_seconds_(timeout_seconds),
          sleep_duration_(std::chrono::duration<double>(sleep_seconds)),
          start_time_(this->get_clock()->now()) {
                    
        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&FibonacciActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FibonacciActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&FibonacciActionServer::handle_accepted, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::seconds(timeout_seconds_),
            std::bind(&FibonacciActionServer::shutdown_node, this)
        );
    }

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    int timeout_seconds_;
    std::chrono::duration<double> sleep_duration_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle) {
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto & sequence = feedback->partial_sequence;
        sequence.push_back(0);
        sequence.push_back(1);

        for (int i = 1; i < goal->order; ++i) {
            // if (goal_handle->is_canceling()) {
            //     goal_handle->canceled(result);
            //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
            //     return;
            // }

            sequence.push_back(sequence[i] + sequence[i - 1]);
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Feedback: [%d]", sequence.back());

            std::this_thread::sleep_for(sleep_duration_);
        }

        auto result = std::make_shared<Fibonacci::Result>();

        result->sequence = sequence;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }

    void shutdown_node() {
        RCLCPP_INFO(this->get_logger(), "Timeout reached, shutting down node...");
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);

    if (argc != 3)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run your_package your_server_node <timeout_seconds> <sleep_seconds>");
      return 1;
    }

    int timeout_seconds = std::stoi(argv[1]);
    double sleep_seconds = std::stod(argv[2]);

    auto server = std::make_shared<FibonacciActionServer>(timeout_seconds, sleep_seconds);
    rclcpp::spin(server);

    rclcpp::shutdown();
    return 0;
}