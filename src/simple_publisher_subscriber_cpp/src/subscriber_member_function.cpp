#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstdlib>
#include <ctime>

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber(double execution_time, double sleep_time)
    : Node("minimal_subscriber"), execution_time_(execution_time), sleep_time_(sleep_time), message_timeout_(2.0), last_message_time_(std::chrono::steady_clock::now())
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&MinimalSubscriber::listener_callback, this, std::placeholders::_1));

        start_time_ = std::chrono::steady_clock::now();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(sleep_time_ * 1000)),
            std::bind(&MinimalSubscriber::check_message_timeout, this)
        );
    }

private:
    void listener_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        last_message_time_ = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration<double>(current_time - start_time_).count();
        RCLCPP_INFO(this->get_logger(), "I heard: \"%s\"", msg->data.c_str());
    }

    void check_message_timeout()
    {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration<double>(current_time - start_time_).count();

        if (elapsed_time >= execution_time_)
        {
            RCLCPP_INFO(this->get_logger(), "Timeout reached, shutting down subscriber.");
            rclcpp::shutdown();
            return;
        }

        auto time_since_last_message = std::chrono::duration<double>(current_time - last_message_time_).count();
        if (time_since_last_message >= message_timeout_)
        {
            RCLCPP_WARN(this->get_logger(), "No messages received for %.2f seconds.", message_timeout_);
            rclcpp::shutdown();
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    double execution_time_;
    double sleep_time_;
    double message_timeout_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_message_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: minimal_subscriber <execution_time> <sleep_time>");
        return 1;
    }

    double execution_time = std::stod(argv[1]);
    double sleep_time = std::stod(argv[2]);

    auto minimal_subscriber = std::make_shared<MinimalSubscriber>(execution_time, sleep_time);

    rclcpp::spin(minimal_subscriber);

    return 0;
}
