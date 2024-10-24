#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(double execution_duration, double sleep_duration, double message_timeout)
  : Node("minimal_subscriber"), execution_duration_(execution_duration), 
    sleep_duration_(sleep_duration), message_timeout_(message_timeout)
  {
    start_time_ = this->now();

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    last_message_time_ = start_time_;
    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(sleep_duration_ * 1000)),
      std::bind(&MinimalSubscriber::check_message_timeout, this));
  }

private:
  void topic_callback(const std_msgs::msg::String & msg)
  {
    last_message_time_ = this->now();

    auto current_time = this->now();
    auto duration = current_time - start_time_;
    double elapsed_seconds = duration.seconds();

    if (elapsed_seconds >= execution_duration_) {
      RCLCPP_INFO(this->get_logger(), "Execution time limit reached, shutting down the node.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_duration_ * 1000)));
  }

  void check_message_timeout()
  {
    auto current_time = this->now();

    if ((current_time - last_message_time_).seconds() >= message_timeout_) {
      RCLCPP_WARN(this->get_logger(), "No messages received for %.2f seconds. Shutting down the node.", message_timeout_);
      rclcpp::shutdown();
    }
  }

  rclcpp::Time start_time_;
  rclcpp::Time last_message_time_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  double execution_duration_;
  double sleep_duration_;
  double message_timeout_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  double execution_duration = 300.0;
  double sleep_duration = 0.25;
  double message_timeout = 1.0;

  if (argc >= 2) {
    execution_duration = std::atof(argv[1]);
    sleep_duration = std::atof(argv[2]);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Using default values: execution_duration = 300.0, sleep_duration = 0.25, message_timeout = 1.0");
  }

  auto node = std::make_shared<MinimalSubscriber>(execution_duration, sleep_duration, message_timeout);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
