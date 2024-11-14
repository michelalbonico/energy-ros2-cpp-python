#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class MinimalService : public rclcpp::Node
{
public:
  MinimalService(int timeout_seconds, double sleep_seconds)
  : Node("minimal_service"),
    timeout_seconds_(timeout_seconds),
    sleep_seconds_(sleep_seconds)
  {
    srv_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints", std::bind(&MinimalService::add_two_ints_callback, this, std::placeholders::_1, std::placeholders::_2));

    start_time_ = this->now();

    timer_ = this->create_wall_timer(1s, std::bind(&MinimalService::timer_callback, this));
  }

private:
  void add_two_ints_callback(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(sleep_seconds_));

    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld b: %ld", request->a, request->b);
  }

  void timer_callback()
  {
    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(this->now() - start_time_).count();
    if (elapsed_time >= timeout_seconds_)
    {
      RCLCPP_INFO(this->get_logger(), "Timeout reached. Shutting down the server.");
      rclcpp::shutdown();
    }
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  int timeout_seconds_;
  double sleep_seconds_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run your_package your_server_node <timeout_seconds> <sleep_seconds>");
    return 1;
  }

  int timeout_seconds;
  double sleep_seconds;

  try
  {
    timeout_seconds = std::stoi(argv[1]);
    sleep_seconds = std::stod(argv[2]);
  }
  catch (const std::invalid_argument &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Please provide valid integer for timeout and float for sleep.");
    return 1;
  }

  auto minimal_service = std::make_shared<MinimalService>(timeout_seconds, sleep_seconds);
  rclcpp::spin(minimal_service);

  rclcpp::shutdown();
  return 0;
}
