#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>
#include <chrono>
#include <thread>
#include <cstdlib>  // Para std::atoi

void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld b: %ld",
              request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  // Inicializa o ROS 2
  rclcpp::init(argc, argv);

  // Verifica se os argumentos foram passados corretamente (tempo de execução e intervalo de sleep)
  if (argc < 3) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: add_two_ints_server execution_time(s) sleep_interval(s)");
    return 1;
  }

  // Pega os parâmetros passados como argumentos para tempo de execução e de espera
  int execution_time = std::atoi(argv[1]);  // Tempo de execução em segundos
  int sleep_interval = std::atoi(argv[2]);  // Intervalo de sleep em segundos

  // Verifica se os parâmetros são válidos
  if (execution_time <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid execution time or sleep interval. Must be greater than 0.");
    return 1;
  }

  // Cria o node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  // Cria o serviço
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  // Obtém a data e hora de início
  auto start_time = std::chrono::steady_clock::now();
  auto duration_limit = std::chrono::seconds(execution_time);  // Definindo o tempo limite com base no argumento
  auto sleep_duration = std::chrono::seconds(sleep_interval);  // Definindo a pausa com base no argumento

  // Loop principal de execução
  while (rclcpp::ok()) {
    // Verifica o tempo atual
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = current_time - start_time;

    // Se o tempo decorrido for maior ou igual ao limite, finaliza o node
    if (elapsed_time >= duration_limit) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Time limit reached. Shutting down the node.");
      break;
    }

    // Faz o spin de uma única iteração
    rclcpp::spin_some(node);

    // Aguarda o intervalo especificado antes de continuar o loop
    std::this_thread::sleep_for(sleep_duration);
  }

  // Finaliza o ROS 2
  rclcpp::shutdown();
  return 0;
}
