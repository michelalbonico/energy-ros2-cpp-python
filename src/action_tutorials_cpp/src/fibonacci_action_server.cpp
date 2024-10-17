#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

namespace action_tutorials_cpp
{
class FibonacciActionServer : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  ACTION_TUTORIALS_CPP_PUBLIC
  explicit FibonacciActionServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
  {
    using namespace std::placeholders;

    // Obtém parâmetros passados por linha de comando
    this->declare_parameter<double>("sleep_duration", 0.25);
    this->declare_parameter<double>("timeout", 10.0);

    this->get_parameter("sleep_duration", sleep_duration_);
    this->get_parameter("timeout", timeout_);

    // Cria o servidor de ação
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));

    // Cria o timer para encerrar o nó após `timeout_` segundos
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(timeout_ * 1000)),
      std::bind(&FibonacciActionServer::shutdown_node, this));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;

  double sleep_duration_;
  double timeout_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;

    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    rclcpp::Time start_time = this->now();
    while (rclcpp::ok()){
     for (int i = 2; (i < goal->order) && rclcpp::ok(); ++i) {
        // Verifica se o tempo limite foi atingido
        if ((this->now() - start_time).seconds() >= timeout_) {
         RCLCPP_INFO(this->get_logger(), "Timeout reached. Shutting down...");
         result->sequence = sequence;
         goal_handle->succeed(result);
         return;
       }

       // Verifica se há uma solicitação de cancelamento
        if (goal_handle->is_canceling()) {
          result->sequence = sequence;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }

        // Atualiza a sequência e publica o feedback
        sequence.push_back(sequence[i - 1] + sequence[i - 2]);
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Published feedback: %d", sequence.back());

        // Aguarda o tempo de sleep configurado
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_duration_));
      }
      sequence = std::vector<int>();
      sequence.push_back(0);
      sequence.push_back(1);

    }

    // Objetivo concluído
    result->sequence = sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }

  // Função chamada pelo timer para encerrar o nó
  void shutdown_node()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down node after timeout.");
    rclcpp::shutdown();
  }
};  // class FibonacciActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)
