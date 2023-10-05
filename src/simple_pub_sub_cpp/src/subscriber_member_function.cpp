#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    timer_ = this->create_wall_timer(
      std::chrono::seconds(2), std::bind(&MinimalSubscriber::check_activity, this));
    
    last_activity_time_ = std::make_shared<rclcpp::Time>(this->now());
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    *last_activity_time_ = this->now(); 
  }

  void check_activity()
  {
    auto elapsed_time = this->now() - *last_activity_time_;
    if (elapsed_time >= std::chrono::seconds(1))
    {
      RCLCPP_INFO(this->get_logger(), "Stopping node due to inactivity.");
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::Time> last_activity_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
