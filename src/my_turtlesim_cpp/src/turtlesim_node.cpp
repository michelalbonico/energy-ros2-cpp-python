#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MyTurtlesimNode : public rclcpp::Node {
public:
    MyTurtlesimNode() : Node("my_turtlesim_node") {
        // Create a publisher for the turtle's velocity command
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        // Create a timer to publish commands periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() {
                auto msg = std::make_unique<geometry_msgs::msg::Twist>();
                // Set the linear and angular velocity commands here
                msg->linear.x = 1.0;
                msg->angular.z = 0.5;
                publisher_->publish(std::move(msg));
            }
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyTurtlesimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
