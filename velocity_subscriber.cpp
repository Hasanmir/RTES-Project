#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

class VelocitySubscriber : public rclcpp::Node {
public:
    VelocitySubscriber() : Node("velocity_subscriber") {
        // Subscribe to cmd_vel
        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&VelocitySubscriber::velocity_callback, this, std::placeholders::_1)
        );

        // Subscribe to blue_ball_detected
        sphere_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "blue_ball_detected", 10, std::bind(&VelocitySubscriber::sphere_callback, this, std::placeholders::_1)
        );
    }

private:
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Log the angular velocity received
        RCLCPP_INFO(this->get_logger(), "Received angular velocity: '%f'", msg->angular.z);
    }

    void sphere_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Blue object detected!");
        } else {
            RCLCPP_INFO(this->get_logger(), "No blue object detected.");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sphere_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocitySubscriber>());
    rclcpp::shutdown();
    return 0;
}

