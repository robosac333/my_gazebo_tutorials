#ifndef WALKER_WALKER_BOT_HPP
#define WALKER_WALKER_BOT_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class WalkerState;

class Walker : public rclcpp::Node {
 public:
  Walker();
  void change_state(WalkerState* new_state);
  void publish_velocity(double linear, double angular);
  bool is_path_clear(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;
  void toggle_rotation_direction();
  double get_rotation_direction() const { return rotation_direction_; }
  rclcpp::TimerBase::SharedPtr create_timer(
      const std::chrono::duration<double>& period,
      std::function<void()> callback);

 private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  WalkerState* current_state_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  double rotation_direction_;
  const double SAFE_DISTANCE = 0.8;
};

class WalkerState {
 public:
  virtual ~WalkerState() = default;
  virtual void handle(Walker* walker,
                     const sensor_msgs::msg::LaserScan::SharedPtr scan) = 0;
};

class ForwardState : public WalkerState {
 public:
  void handle(Walker* walker,
             const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
};

class RotationState : public WalkerState {
 public:
  RotationState() : initial_rotation_(true), rotation_timer_(nullptr) {}
  void handle(Walker* walker,
             const sensor_msgs::msg::LaserScan::SharedPtr scan) override;

 private:
  bool initial_rotation_;  // Flag to track initial 5-second rotation
  rclcpp::TimerBase::SharedPtr rotation_timer_;
};

#endif  // WALKER_WALKER_BOT_HPP