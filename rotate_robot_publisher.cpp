#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class RotateRobot : public rclcpp::Node {
public:
    RotateRobot() : Node("rotate_robot") {
        // Create publishers
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sphere_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("blue_ball_detected", 10);

        // Timer to publish velocity commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RotateRobot::update_velocity, this)
        );

        // Subscribe to camera images
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&RotateRobot::process_image, this, std::placeholders::_1)
        );

        // Initialize OpenCV window
        cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);
    }

    ~RotateRobot() {
        cv::destroyWindow("Camera Feed");
    }

private:
    void update_velocity() {
        auto msg = geometry_msgs::msg::Twist();

        if (blue_ball_detected_) {
            // Stop rotation and centralize the object
            msg.angular.z = 0.0;

            // Centralize based on detected object's position
            if (object_center_x_ < frame_center_x_ - 20) { // Object is to the left
                msg.angular.z = 0.2; // Rotate right
            } else if (object_center_x_ > frame_center_x_ + 20) { // Object is to the right
                msg.angular.z = -0.2; // Rotate left
            }

            RCLCPP_INFO(this->get_logger(), "Centralizing the blue object...");
        } else {
            msg.angular.z = 1.0; // Default rotation speed
            RCLCPP_INFO(this->get_logger(), "Rotating the robot...");
        }

        vel_pub_->publish(msg);

        // Publish detection status
        auto detection_msg = std_msgs::msg::Bool();
        detection_msg.data = blue_ball_detected_;
        sphere_detected_pub_->publish(detection_msg);
    }

    void process_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Convert to HSV
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        // Threshold to get blue colors
        cv::Mat mask;
        cv::inRange(hsv_image, cv::Scalar(100, 150, 0), cv::Scalar(140, 255, 255), mask);

        // Display the mask for debugging
        cv::imshow("Mask", mask);
        cv::waitKey(1);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        blue_ball_detected_ = false;

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 500) { // Minimum area threshold for detection
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(contour, center, radius);

                // Check roundness
                if (std::fabs(1 - ((double)radius * radius * M_PI) / area) <= 0.2) {
                    blue_ball_detected_ = true;
                    object_center_x_ = center.x; // Update object center x position
                    RCLCPP_INFO(this->get_logger(), "Blue object detected! Center: [%f, %f], Radius: %f", center.x, center.y, radius);

                    // Draw the detected circle
                    cv::circle(cv_ptr->image, center, radius, cv::Scalar(0, 255, 0), 2);
                    cv::circle(cv_ptr->image, center, 5, cv::Scalar(0, 0, 255), -1);
                    break;
                }
            }
        }

        // Display the image
        cv::imshow("Camera Feed", cv_ptr->image);
        cv::waitKey(1); // Update the window

        // Set the frame center for object centralization logic
        frame_center_x_ = cv_ptr->image.cols / 2;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sphere_detected_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    bool blue_ball_detected_ = false;
    float object_center_x_ = 0.0;
    float frame_center_x_ = 0.0; // Center of the frame
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RotateRobot>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

