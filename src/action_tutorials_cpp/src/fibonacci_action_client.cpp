#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <action_tutorials_interfaces/action/fibonacci.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

class FibonacciActionClient : public rclcpp::Node
{
public:
    using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

    FibonacciActionClient() : Node("fibonacci_action_client")
    {
        // Create action client
        action_client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    }

    void send_goal(int order)
    {
        // Wait for the server to be available
        if (!action_client_->wait_for_action_server(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        // Create goal message
        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = order;

        // Send goal asynchronously
        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&FibonacciActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&FibonacciActionClient::get_result_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&FibonacciActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        // std::bind(&FibonacciActionClient::feedback_callback, this, std::placeholders::_1);

        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;

    void goal_response_callback(std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Goal accepted :)");
    }

    void get_result_callback(const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult & result)
    {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(this->get_logger(), "Action failed");
            return;
        }
        std::stringstream ss;
        ss << "Result received: ";
        for (auto number : result.result->sequence) {
        ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }

    void feedback_callback(
        GoalHandleFibonacci::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback
    )
    {
        std::stringstream ss;
        ss << "Next number in sequence received: ";
        for (auto number : feedback->partial_sequence) 
        {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto action_client = std::make_shared<FibonacciActionClient>();

    // Send the goal with the order value
    action_client->send_goal(10);

    rclcpp::spin(action_client);

    return 0;
}
