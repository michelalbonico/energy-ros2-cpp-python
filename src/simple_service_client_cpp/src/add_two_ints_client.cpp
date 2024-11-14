#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class MinimalClientAsync : public rclcpp::Node
{
public:
  MinimalClientAsync(int a, int b, int timeout_seconds, double sleep_seconds)
  : Node("minimal_client_async"),
    timeout_seconds_(timeout_seconds),
    sleep_seconds_(sleep_seconds),
    req_()  // Initialize request values here
  {
    // Create the client
    cli_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    // Wait for the service to be available
    while (!cli_->wait_for_service(1s))
    {
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // Set the request values
    req_.a = a;
    req_.b = b;

    // Start the timer for making requests periodically
    start_time_ = this->now();
    timer_ = this->create_wall_timer(std::chrono::duration<double>(sleep_seconds), std::bind(&MinimalClientAsync::timer_callback, this));
  }

private:
  // Timer callback to periodically send service requests
  void timer_callback()
  {
    auto elapsed_time = (this->now() - start_time_).seconds();
    if (elapsed_time >= timeout_seconds_)
    {
      RCLCPP_INFO(this->get_logger(), "Timeout reached. Stopping client.");
      rclcpp::shutdown();
      return;
    }

    // Call the service asynchronously
    // auto future_and_request_id = cli_->async_send_request(std::make_shared<example_interfaces::srv::AddTwoInts::Request>(req_));

    // // Use add_done_callback to set the callback for the future
    // future_and_request_id.future.add_done_callback([this](rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) {
    //     this->response_callback(future);
    // });

    auto future = cli_->async_send_request(std::make_shared<example_interfaces::srv::AddTwoInts::Request>(req_));
    future.wait();  // Blocks until the future is done.

  }

  // Callback to handle the response
  void response_callback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
  {
    if (future.get() != nullptr)
    {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Result of add_two_ints: %ld + %ld = %ld", req_.a, req_.b, response->sum);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Service call failed.");
    }
  }

  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr cli_;  // Correct service type
  example_interfaces::srv::AddTwoInts::Request req_;  // Correct request type
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  int timeout_seconds_;
  double sleep_seconds_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 5)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run your_package your_client_node <int a> <int b> <timeout_seconds> <sleep_seconds>");
    return 1;
  }

  int a, b, timeout_seconds;
  double sleep_seconds;

  try
  {
    a = std::stoi(argv[1]);
    b = std::stoi(argv[2]);
    timeout_seconds = std::stoi(argv[3]);
    sleep_seconds = std::stod(argv[4]);
  }
  catch (const std::invalid_argument &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Please provide valid integers for a, b and timeout, and a float for sleep.");
    return 1;
  }

  // Create and spin the client node
  auto minimal_client = std::make_shared<MinimalClientAsync>(a, b, timeout_seconds, sleep_seconds);
  rclcpp::spin(minimal_client);

  rclcpp::shutdown();
  return 0;
}
