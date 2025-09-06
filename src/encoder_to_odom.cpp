#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <optional>

class EncoderOdomNode : public rclcpp::Node {
public:
  EncoderOdomNode() : Node("encoder_to_odom") {
    // Robot parameters (adjust!)
    wheel_radius_ = 0.065;  // meters
    wheel_base_   = 0.13;  // meters between wheels

    // Robot state
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;

    // Subscribers & publishers
    sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/encoders/angles", 10,
      std::bind(&EncoderOdomNode::encoderCallback, this, std::placeholders::_1)
    );

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  }

private:
  double unwrapAngle(double current, double &prev, int &turns) {
    // Compute difference
    double diff = current - prev;

    // Handle wrap-around across 0/360 boundaries
    if (diff > M_PI) {
      turns -= 1;  // crossed 360 -> 0
    } else if (diff < -M_PI) {
      turns += 1;  // crossed 0 -> 360
    }

    prev = current;
    return current + turns * 2.0 * M_PI;
  }

  void encoderCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Received /encoders/angles with less than 2 values");
      return;
    }

    // Convert to radians
    double left_now  = msg->data[0] * M_PI / 180.0;
    double right_now = msg->data[1] * M_PI / 180.0;

    if (!prev_left_angle_.has_value() || !prev_right_angle_.has_value()) {
      // Initialize state
      prev_left_angle_ = left_now;
      prev_right_angle_ = right_now;
      left_turns_ = 0;
      right_turns_ = 0;
      unwrapped_left_ = left_now;
      unwrapped_right_ = right_now;
      return;
    }

    // Unwrap angles to continuous values
    unwrapped_left_  = unwrapAngle(left_now,  prev_left_angle_.value(),  left_turns_);
    unwrapped_right_ = unwrapAngle(right_now, prev_right_angle_.value(), right_turns_);

    // Δθ wheel rotation → displacement
    double d_left  = (unwrapped_left_  - last_left_)  * wheel_radius_;
    double d_right = (unwrapped_right_ - last_right_) * wheel_radius_;

    last_left_  = unwrapped_left_;
    last_right_ = unwrapped_right_;

    // Robot motion
    double d_center = (d_left + d_right) / 2.0;
    double d_theta  = (d_right - d_left) / wheel_base_;

    theta_ += d_theta;
    x_ += d_center * cos(theta_);
    y_ += d_center * sin(theta_);

    // Publish Odometry
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    pub_->publish(odom);
  }

  // Parameters
  double wheel_radius_;
  double wheel_base_;

  // State
  double x_, y_, theta_;

  // Angle tracking
  std::optional<double> prev_left_angle_, prev_right_angle_;
  double last_left_{0.0}, last_right_{0.0};
  double unwrapped_left_{0.0}, unwrapped_right_{0.0};
  int left_turns_{0}, right_turns_{0};

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderOdomNode>());
  rclcpp::shutdown();
  return 0;
}

