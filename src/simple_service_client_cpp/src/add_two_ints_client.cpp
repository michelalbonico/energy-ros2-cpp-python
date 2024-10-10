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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y execution_time(s) sleep_interval(s)");
    return 1;
  }

  // Parâmetros
  long a = std::atoll(argv[1]);  // Primeiro inteiro a ser somado
  long b = std::atoll(argv[2]);  // Segundo inteiro a ser somado
  int execution_time = std::atoi(argv[3]);  // Tempo total de execução em segundos
  int sleep_interval = std::atoi(argv[4]);  // Intervalo de espera (sleep) em segundos

  // Verificação dos valores de tempo
  if (execution_time <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid execution time or sleep interval. Must be greater than 0.");
    return 1;
  }

  auto node = rclcpp::Node::make_shared("add_two_ints_client");
  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = a;
  request->b = b;

  // Registrar o tempo de início
  auto start_time = std::chrono::steady_clock::now();
  auto timeout_duration = std::chrono::seconds(execution_time);
  auto sleep_duration = std::chrono::seconds(sleep_interval);  // Definido em segundos

  while (true) {
    // Verifica se o tempo de execução foi atingido
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - start_time >= timeout_duration) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Execution time reached. Shutting down.");
      break;
    }

    // Espera pelo serviço
    if (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        break;
      }

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    } else {
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
      }
    }

    // Pausa de acordo com o intervalo especificado em segundos
    std::this_thread::sleep_for(sleep_duration);
  }

  rclcpp::shutdown();
  return 0;
}
