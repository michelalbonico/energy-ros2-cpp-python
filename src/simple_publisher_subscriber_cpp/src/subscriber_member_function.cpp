#include <chrono>    // Para std::chrono::milliseconds
#include <functional>
#include <memory>
#include <thread>     // Para std::this_thread::sleep_for
#include <cstdlib>    // Para std::atof

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;  // Habilita o sufixo ms para std::chrono

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(double execution_duration, double sleep_duration)
  : Node("minimal_subscriber"), execution_duration_(execution_duration), sleep_duration_(sleep_duration)
  {
    // Salva o tempo de início
    start_time_ = this->now();

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    // Verifica o tempo atual e calcula a diferença em segundos
    auto current_time = this->now();
    auto duration = current_time - start_time_;
    double elapsed_seconds = duration.seconds();

    // Checa se o tempo de execução ultrapassou o limite especificado
    if (elapsed_seconds >= execution_duration_) {
      RCLCPP_INFO(this->get_logger(), "Tempo limite atingido, encerrando o nó.");
      rclcpp::shutdown();
      return;
    }

    // Exibe a mensagem recebida
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

    // Pausa de acordo com a duração especificada
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_duration_ * 1000)));
  }

  rclcpp::Time start_time_;  // Tempo de início do node
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  double execution_duration_;  // Duração total de execução em segundos
  double sleep_duration_;      // Intervalo de espera entre recebimentos de mensagens em segundos
};

int main(int argc, char * argv[])
{
  // Inicializa o nó e verifica os argumentos de entrada
  rclcpp::init(argc, argv);

  // Define valores padrão para tempo de execução e de sleep
  double execution_duration = 300.0;  // 5 minutos (300 segundos) por padrão
  double sleep_duration = 0.25;       // 0.25 segundos por padrão

  // Checa se foram passados argumentos para tempo de execução e de sleep
  if (argc >= 3) {
    execution_duration = std::atof(argv[1]);  // Primeiro argumento é o tempo de execução em segundos
    sleep_duration = std::atof(argv[2]);      // Segundo argumento é o tempo de sleep em segundos
  } else {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Usando valores padrão: execution_duration = 300.0, sleep_duration = 0.25");
  }

  // Cria a instância do nó com os valores especificados
  rclcpp::spin(std::make_shared<MinimalSubscriber>(execution_duration, sleep_duration));
  rclcpp::shutdown();
  return 0;
}
