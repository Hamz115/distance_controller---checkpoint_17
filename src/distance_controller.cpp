#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class PIDController {
public:
  PIDController(double Kp, double Ki, double Kd, double setpoint)
      : Kp_(Kp), Ki_(Ki), Kd_(Kd), setpoint_(setpoint), integral_(0.0), previous_error_(0.0) {}

  double calculate(double current_value) {
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - previous_time_;
    double error = setpoint_ - current_value;

    integral_ += error * elapsed_time.count();
    double derivative = (error - previous_error_) / elapsed_time.count();

    double output = (Kp_ * error) + (Ki_ * integral_) + (Kd_ * derivative);

    previous_error_ = error;
    previous_time_ = current_time;

    return output;
  }

private:
  double Kp_, Ki_, Kd_, setpoint_, integral_, previous_error_;
  std::chrono::time_point<std::chrono::steady_clock> previous_time_ = std::chrono::steady_clock::now();
};

class DistanceController : public rclcpp::Node {
public:
  DistanceController()
      : Node("distance_controller"), waypoints_{1.0, 2.0, 3.0}, current_waypoint_index_(0), Kp_(1.0), Ki_(0.1), Kd_(0.05),
        max_speed_(1.0), high_speed_distance_(0.5) {
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/rosbot_xl_base_controller/odom", 10, std::bind(&DistanceController::odometry_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(100ms, std::bind(&DistanceController::control_loop, this));
    pid_ = std::make_shared<PIDController>(Kp_, Ki_, Kd_, waypoints_[current_waypoint_index_]);
  }

private:
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_distance_ = std::sqrt(std::pow(msg->pose.pose.position.x, 2) + std::pow(msg->pose.pose.position.y, 2));
  }

  void control_loop() {
    if (current_waypoint_index_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "All waypoints reached. Process finished.");
      rclcpp::shutdown();
      return;
    }

    double control_signal;
    if (current_waypoint_index_ == 0 && current_distance_ < high_speed_distance_) {
      // Move at high speed towards the first waypoint
      control_signal = max_speed_;
    } else {
      // Use PID control for precise movement
      control_signal = pid_->calculate(current_distance_);
      control_signal = std::min(control_signal, max_speed_); // Cap the control signal to the max speed
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = control_signal;
    velocity_publisher_->publish(cmd);

    if (std::abs(waypoints_[current_waypoint_index_] - current_distance_) < 0.01) {
      cmd.linear.x = 0.0;
      velocity_publisher_->publish(cmd);
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu at %.2f meters. Stopping for stabilization.", current_waypoint_index_ + 1, waypoints_[current_waypoint_index_]);

      // Pause for a period to ensure the robot comes to a complete stop
      std::this_thread::sleep_for(std::chrono::seconds(3)); // Pause for 3 seconds

      current_waypoint_index_++;
      if (current_waypoint_index_ < waypoints_.size()) {
        pid_ = std::make_shared<PIDController>(Kp_, Ki_, Kd_, waypoints_[current_waypoint_index_]);
      } else {
        RCLCPP_INFO(this->get_logger(), "All waypoints reached. Process finished.");
        rclcpp::shutdown(); // Shut down the node to exit the program
      }
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<double> waypoints_;
  size_t current_waypoint_index_;
  double current_distance_ = 0.0;
  double Kp_, Ki_, Kd_;
  double max_speed_;
  double high_speed_distance_; // Distance within which the robot moves at high speed
  std::shared_ptr<PIDController> pid_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceController>());
  rclcpp::shutdown();
  return 0;
}