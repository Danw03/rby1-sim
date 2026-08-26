#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

class PowerServoMonitor : public rclcpp::Node {
public:
  using RobotType = rb::Robot<rb::y1_model::M>;
  using RobotPtr =
      decltype(RobotType::Create(std::declval<const std::string &>()));
  PowerServoMonitor()
      : Node("power_servo_monitor", "/rby1") {
    robot_ip_ = this->declare_parameter<std::string>(
        "robot_ip", "");
    poll_period_sec_ = this->declare_parameter<double>(
        "poll_period_sec", 0.5);
    power_pattern_ = this->declare_parameter<std::string>(
        "power_pattern", ".*");
    servo_pattern_ = this->declare_parameter<std::string>(
        "servo_pattern", ".*");

    if (robot_ip_.empty()) {
      throw std::runtime_error(
          "robot_ip parameter is required. "
          "Example: -p robot_ip:=192.168.30.1:50051");
    }

    if (poll_period_sec_ <= 0.0) {
      poll_period_sec_ = 0.5;
    }

    power_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "power_state", 10);
    servo_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "servo_state", 10);

    RCLCPP_INFO(
        this->get_logger(),
        "Connecting read-only Power/Servo monitor to %s",
        robot_ip_.c_str());

    robot_ = RobotType::Create(robot_ip_);

    if (!robot_->Connect()) {
      throw std::runtime_error(
          "Failed to connect to RB-Y1 at " + robot_ip_);
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Connected. Monitoring Power='%s', Servo='%s'.",
        power_pattern_.c_str(),
        servo_pattern_.c_str());

    const auto period =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(poll_period_sec_));

    timer_ = this->create_wall_timer(
        period,
        std::bind(
            &PowerServoMonitor::poll_state,
            this));

    poll_state();
  }

private:
  void poll_state() {
    try {
      const bool power_on =
          robot_->IsPowerOn(power_pattern_);
      const bool servo_on =
          robot_->IsServoOn(servo_pattern_);

      std_msgs::msg::Bool power_msg;
      power_msg.data = power_on;
      power_pub_->publish(power_msg);

      std_msgs::msg::Bool servo_msg;
      servo_msg.data = servo_on;
      servo_pub_->publish(servo_msg);

      if (!last_power_.has_value() ||
          last_power_.value() != power_on) {
        RCLCPP_INFO(
            this->get_logger(),
            "Power state: %s",
            power_on ? "ON" : "OFF");
        last_power_ = power_on;
      }

      if (!last_servo_.has_value() ||
          last_servo_.value() != servo_on) {
        RCLCPP_INFO(
            this->get_logger(),
            "Servo state: %s",
            servo_on ? "ON" : "OFF");
        last_servo_ = servo_on;
      }

    } catch (const std::exception &e) {
      // Do not publish a guessed value after a failed SDK read.
      // ros_backend.py will make stale feedback UNKNOWN.
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          3000,
          "Power/Servo query failed: %s",
          e.what());
    } catch (...) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          3000,
          "Power/Servo query failed with unknown exception.");
    }
  }

  std::string robot_ip_;
  std::string power_pattern_;
  std::string servo_pattern_;
  double poll_period_sec_{0.5};

  RobotPtr robot_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr power_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<bool> last_power_;
  std::optional<bool> last_servo_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<PowerServoMonitor>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(
        rclcpp::get_logger("power_servo_monitor"),
        "%s",
        e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
