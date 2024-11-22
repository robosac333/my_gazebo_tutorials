/*
 * Copyright 2024 Sachin Jadhav.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "walker/walker_bot.hpp"

/**
 * @brief Constructor for the Walker class.
 * Initializes the ROS 2 node, sets up publishers and subscribers,
 * and sets the initial state to ForwardState.
 */
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

/**
 * @brief Callback function for LaserScan messages.
 * Delegates processing of the scan to the current state.
 *
 * @param scan Shared pointer to the LaserScan message.
 */
void Walker::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  current_state_->handle(this, scan);
}

/**
 * @brief Changes the state of the Walker.
 * Deletes the current state and transitions to the new state.
 *
 * @param new_state Pointer to the new state.
 */
void Walker::change_state(WalkerState* new_state) {
  delete current_state_;
  current_state_ = new_state;
}

/**
 * @brief Publishes velocity commands to the robot.
 *
 * @param linear Linear velocity (m/s).
 * @param angular Angular velocity (rad/s).
 */
void Walker::publish_velocity(double linear, double angular) {
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear;
  msg.angular.z = angular;
  vel_publisher_->publish(msg);
}

/**
 * @brief Checks if the path in front of the robot is clear.
 *
 * @param scan Shared pointer to the LaserScan message.
 * @return true if the path is clear, false otherwise.
 */
bool Walker::is_path_clear(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
  const int front_start = 0;
  const int front_end = 15;
  const int back_start = 345;
  const int back_end = 359;

  for (int i = front_start; i <= front_end; ++i) {
    if (scan->ranges[i] < SAFE_DISTANCE) return false;
  }

  for (int i = back_start; i <= back_end; ++i) {
    if (scan->ranges[i] < SAFE_DISTANCE) return false;
  }

  return true;
}

/**
 * @brief Toggles the rotation direction between clockwise and
 * counter-clockwise.
 */
void Walker::toggle_rotation_direction() { rotation_direction_ *= -1.0; }

/**
 * @brief Handles behavior in the RotationState.
 * If the path is clear, transitions to ForwardState. Otherwise, continues
 * rotating.
 *
 * @param walker Pointer to the Walker instance.
 * @param scan Shared pointer to the LaserScan message.
 */
void RotationState::handle(Walker* walker,
                           const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (walker->is_path_clear(scan)) {
    walker->change_state(new ForwardState());
  } else {
    walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());
  }
}

/**
 * @brief Handles behavior in the ForwardState.
 * If the path is clear, moves forward. Otherwise, toggles rotation direction
 * and transitions to RotationState.
 *
 * @param walker Pointer to the Walker instance.
 * @param scan Shared pointer to the LaserScan message.
 */
void ForwardState::handle(Walker* walker,
                          const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (walker->is_path_clear(scan)) {
    walker->publish_velocity(0.2, 0.0);
  } else {
    walker->toggle_rotation_direction();
    walker->change_state(new RotationState());
  }
}
