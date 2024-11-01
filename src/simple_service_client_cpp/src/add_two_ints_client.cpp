#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 5) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
                  "usage: add_two_ints_client X Y TIME_LIMIT_SECONDS SLEEP_SECONDS");
      return 1;
  }

  auto node = rclcpp::Node::make_shared("add_two_ints_client");
  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  int time_limit = std::stoi(argv[3]);  
  double sleep_seconds = std::stod(argv[4]);  
  auto start_time = std::chrono::steady_clock::now();

  while (true) {
    auto elapsed_time = std::chrono::steady_clock::now() - start_time;
    if (std::chrono::duration_cast<std::chrono::seconds>(elapsed_time).count() >= time_limit) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Tempo limite atingido. Encerrando.");
      break;
    }

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                     "Interrompido enquanto aguardava o serviço. Saindo.");
        return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Serviço não disponível, tentando novamente...");
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) 
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Soma: %ld", result.get()->sum);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Falha ao chamar o serviço.");
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(sleep_seconds));
  }

  rclcpp::shutdown();
  return 0;
}
