#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class MyTurtlesimNode : public rclcpp::Node {
public:
    MyTurtlesimNode() : Node("my_turtlesim_node") {
        // Create a subscriber for the turtle's velocity command
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel",
            10,std::bind(&MyTurtlesimNode::topic_callback, this, _1));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&MyTurtlesimNode::check_activity, this)
        );
        
        last_activity_time_ = std::make_shared<rclcpp::Time>(this->now());
    }

private:

    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;
        RCLCPP_INFO(this->get_logger(), "Received Velocity Command - Linear X: %f, Angular Z: %f", linear_x, angular_z);
        *last_activity_time_ = this->now(); 
        // Implement your turtle control logic here based on the received velocity commands
    }

    void check_activity()
    {
        auto elapsed_time = this->now() - *last_activity_time_;
        if (elapsed_time >= std::chrono::seconds(1))
        {
        RCLCPP_INFO(this->get_logger(), "Stopping node due to inactivity.");
        rclcpp::shutdown();
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Time> last_activity_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyTurtlesimNode>();
    rclcpp::spin(node);
    //rclcpp::shutdown();
    return 0;
}
