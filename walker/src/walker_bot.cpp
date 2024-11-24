#include "walker/walker_bot.hpp"

Walker::Walker() : Node("walker"), rotation_direction_(1.0) {
  vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  auto qos_profile = rclcpp::QoS(10)
                         .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                         .history(rclcpp::HistoryPolicy::KeepLast);

  scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos_profile,
      std::bind(&Walker::scan_callback, this, std::placeholders::_1));

  current_state_ = new ForwardState();
}

void Walker::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  current_state_->handle(this, scan);
}

void Walker::change_state(WalkerState* new_state) {
  delete current_state_;
  current_state_ = new_state;
}

void Walker::publish_velocity(double linear, double angular) {
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear;
  msg.angular.z = angular;
  vel_publisher_->publish(msg);
}

bool Walker::is_path_clear(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
  const int left_start = 0;        // Starting from front center
  const int left_end = 17;         // 45 degrees to the left
  const int right_start = 343;     // 45 degrees to the right
  const int right_end = 359;       // Back to front center

  // Check left side of front arc (0 to 45 degrees)
  for (int i = left_start; i <= left_end; ++i) {
    if (scan->ranges[i] < SAFE_DISTANCE) {
      RCLCPP_INFO(rclcpp::get_logger("Walker"), 
                  "Obstacle detected in left arc at angle %d", i);
      return false;
    }
  }

  // Check right side of front arc (315 to 359 degrees)
  for (int i = right_start; i <= right_end; ++i) {
    if (scan->ranges[i] < SAFE_DISTANCE) {
      RCLCPP_INFO(rclcpp::get_logger("Walker"), 
                  "Obstacle detected in right arc at angle %d", i);
      return false;
    }
  }
  return true;
}

void Walker::toggle_rotation_direction() { 
  rotation_direction_ *= -1.0; 
}

rclcpp::TimerBase::SharedPtr Walker::create_timer(
    const std::chrono::duration<double>& period,
    std::function<void()> callback) {
  return this->create_wall_timer(period, callback);
}

void RotationState::handle(Walker* walker,
                         const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // If this is the initial rotation period
  if (initial_rotation_) {
    // Start rotating
    walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());
    
    // If timer hasn't been created yet, create it
    if (!rotation_timer_) {
      rotation_timer_ = walker->create_timer(
          std::chrono::seconds(10),
          [this]() {
            // After 5 seconds, mark initial rotation as complete
            initial_rotation_ = false;
            rotation_timer_ = nullptr;
          });
      
      RCLCPP_INFO(walker->get_logger(), 
                  "Starting initial 10-second rotation period");
    }
  }
  // After initial rotation period, check path and continue rotating if needed
  else {
    if (walker->is_path_clear(scan)) {
      RCLCPP_INFO(walker->get_logger(), "Path is clear, moving forward");
      walker->change_state(new ForwardState());
    } else {
      // Continue rotating in the same direction
      walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());
      RCLCPP_INFO(walker->get_logger(), "Path blocked, continuing rotation");
    }
  }
}

void ForwardState::handle(Walker* walker,
                        const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (walker->is_path_clear(scan)) {
    walker->publish_velocity(0.5, 0.0);
  } else {
    // Toggle rotation direction before changing to rotation state
    walker->toggle_rotation_direction();
    RCLCPP_INFO(walker->get_logger(), 
                "Obstacle detected, changing rotation direction and starting rotation");
    walker->change_state(new RotationState());
  }
}