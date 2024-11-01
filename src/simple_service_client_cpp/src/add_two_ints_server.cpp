#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld, b: %ld", request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                   "usage: add_two_ints_server TIME_LIMIT_SECONDS SLEEP_SECONDS");
      return 1;
  }

  int time_limit = std::stoi(argv[1]);  // Tempo limite em segundos
  double sleep_seconds = std::stod(argv[2]);  // Intervalo entre requisições (em segundos)

  auto node = rclcpp::Node::make_shared("add_two_ints_server");
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  auto start_time = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    auto elapsed_time = std::chrono::steady_clock::now() - start_time;
    if (std::chrono::duration_cast<std::chrono::seconds>(elapsed_time).count() >= time_limit) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Time limit reached. Shutting down.");
      break;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(sleep_seconds));

    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
