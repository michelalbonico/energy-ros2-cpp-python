#include <chrono>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_tutorials_interfaces/action/fibonacci.hpp"

using namespace std::chrono_literals;
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

class FibonacciActionClient : public rclcpp::Node {
public:
    FibonacciActionClient(int timeout_seconds, double sleep_seconds)
        : Node("fibonacci_action_client"),
          timeout_seconds_(timeout_seconds),
          sleep_duration_(std::chrono::duration<double>(sleep_seconds)),
          start_time_(this->now()) {
        
        action_client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    }

    void send_goals(int order) {
        while (rclcpp::ok() && (this->now() - start_time_).seconds() < timeout_seconds_) {
            RCLCPP_INFO(this->get_logger(), "Sending goal with order %d", order);

            auto goal_msg = Fibonacci::Goal();
            goal_msg.order = order;

            if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available!");
                return;
            }

            rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
            options.feedback_callback =
                std::bind(&FibonacciActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            options.result_callback =
                std::bind(&FibonacciActionClient::result_callback, this, std::placeholders::_1);

            auto goal_handle_future = action_client_->async_send_goal(goal_msg, options);

            // Wait for the goal to be accepted synchronously
            rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future, std::chrono::seconds(5));
            auto goal_handle = goal_handle_future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server.");
                return;
            }

            // Now, we will wait until the result is processed via the result callback
            RCLCPP_INFO(this->get_logger(), "Goal accepted, waiting for result...");
            
            // The result callback will signal when the result is available
            // We block the sending of the next goal until this is processed
            goal_result_ready_ = false;
            while (!goal_result_ready_) {
                rclcpp::spin_some(shared_from_this());  // Spin to process result callback
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Small sleep to avoid high CPU usage
            }

        }

        RCLCPP_INFO(this->get_logger(), "Timeout reached, stopping client.");
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
    rclcpp::Time start_time_;
    int timeout_seconds_;
    std::chrono::duration<double> sleep_duration_;
    bool goal_result_ready_ = false;

    void feedback_callback(
        std::shared_ptr<rclcpp_action::ClientGoalHandle<Fibonacci>>,
        const std::shared_ptr<const Fibonacci::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Feedback received:");
        for (auto num : feedback->partial_sequence) {
            RCLCPP_INFO(this->get_logger(), "%d", num);
        }
    }

    void result_callback(
        const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Result received:");
                for (auto num : result.result->sequence) {
                    RCLCPP_INFO(this->get_logger(), "%d", num);
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }

        // Set the flag to signal the result is ready
        goal_result_ready_ = true;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    if (argc != 3) {
        std::cerr << "Usage: ros2 run your_package your_client_node <timeout_seconds> <sleep_seconds>" << std::endl;
        return 1;
    }

    int timeout_seconds = std::stoi(argv[1]);
    double sleep_seconds = std::stod(argv[2]);

    auto client = std::make_shared<FibonacciActionClient>(timeout_seconds, sleep_seconds);
    client->send_goals(10);

    rclcpp::shutdown();
    return 0;
}