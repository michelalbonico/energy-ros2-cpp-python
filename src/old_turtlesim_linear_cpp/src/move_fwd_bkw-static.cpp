#include <iostream>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class RobotCleaner : public rclcpp::Node
{
public:
  RobotCleaner()
  : Node("robot_driver"),count_(0),fwd_(true)
  {
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    vel_msg_ = std::make_shared<geometry_msgs::msg::Twist>();

    timer_ = this->create_wall_timer(
      500ms, std::bind(&RobotCleaner::timer_callback, this));
  }

  void timer_callback()
  {
    // Set fixed values for speed, distance, and direction
    double speed = 1.0;    // Adjust as needed
    double distance = 2.0; // Adjust as needed

    if (count_ < 100) {
      move(speed, distance, fwd_);
      fwd_ = !fwd_;
      sleep(0.1); 
    } else {
      sleep(0.1);
      timer_->cancel();
      rclcpp::shutdown();
    }

    count_++;
  }

  void move(double speed, double distance, bool is_forward)
  {
    
    if (is_forward)
    {
      vel_msg_->linear.x = std::abs(speed);
    }
    else
    {
      vel_msg_->linear.x = -std::abs(speed);
    }

    // Since we are moving just in the x-axis
    vel_msg_->linear.y = 0.0;
    vel_msg_->linear.z = 0.0;
    vel_msg_->angular.x = 0.0;
    vel_msg_->angular.y = 0.0;
    vel_msg_->angular.z = 0.0;

    // Set the current time for distance calculation
    rclcpp::Time t0 = this->now();
    double current_distance = 0.0;

    // Loop to move the turtle in the specified distance
    while (current_distance < distance)
    {
      // Publish the velocity
      velocity_publisher_->publish(*vel_msg_);
      // Take the actual time for velocity calculation
      rclcpp::Time t1 = this->now();
      // Calculate distance
      current_distance = speed * (t1 - t0).nanoseconds() / 1e9;
    }

    // After the loop, stop the robot
    vel_msg_->linear.x = 0.0;
    // Force the robot to stop
    velocity_publisher_->publish(*vel_msg_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  std::shared_ptr<geometry_msgs::msg::Twist> vel_msg_;
  int count_;
  bool fwd_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = std::make_shared<RobotCleaner>();

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  // ros::MultiThreadedSpinner spinner(4);
  // spinner.spin(std::make_shared<RobotCleaner>());

  // Create a multithreaded spinner
  //rclcpp::executors::SingleThreadedExecutor spinner;
  //auto spinner = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  // spinner.add_node(std::make_shared<RobotCleaner>());
  // printf('I am here');
  // spinner.spin();

  // rclcpp::spin(std::make_shared<RobotCleaner>());
  return 0;
}