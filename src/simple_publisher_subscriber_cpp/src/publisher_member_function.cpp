#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(double execution_duration, double sleep_duration)
  : Node("minimal_publisher"), count_(0), execution_duration_(execution_duration), sleep_duration_(sleep_duration)
  {
    start_time_ = this->now();
    
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(sleep_duration_ * 1000)),
      std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto current_time = this->now();
    auto duration = current_time - start_time_;
    double elapsed_seconds = duration.seconds();

    if (elapsed_seconds >= execution_duration_) {
      RCLCPP_INFO(this->get_logger(), "Tempo limite atingido, encerrando o nó.");
      exit(0);
    }

    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::Time start_time_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  double execution_duration_;
  double sleep_duration_;
};

int main(int argc, char * argv[])
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

  auto node = std::make_shared<MinimalPublisher>(execution_duration, sleep_duration);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
