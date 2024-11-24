/**
 * @file walker_bot.cpp
 * @brief Implementation of the Walker robot class and its states.
 * @copyright 2024 Sachin Jadhav
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
 * @brief Constructs a new Walker object.
 *
 * Initializes the ROS2 node with publishers and subscribers.
 * Sets up velocity publisher for robot control and laser scan subscriber
 * for obstacle detection. Initializes the robot in ForwardState.
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
 * @brief Callback function for processing laser scan messages.
 * @param scan Shared pointer to the received laser scan message.
 *
 * Delegates the handling of laser scan data to the current state object.
 */
void Walker::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  current_state_->handle(this, scan);
}

/**
 * @brief Changes the current state of the Walker.
 * @param new_state Pointer to the new state object.
 *
 * Deletes the current state object and transitions to the new state.
 */
void Walker::change_state(WalkerState* new_state) {
  delete current_state_;
  current_state_ = new_state;
}

/**
 * @brief Publishes velocity commands to the robot.
 * @param linear Linear velocity in meters per second.
 * @param angular Angular velocity in radians per second.
 *
 * Creates and publishes a Twist message with the specified velocities.
 */
void Walker::publish_velocity(double linear, double angular) {
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear;
  msg.angular.z = angular;
  vel_publisher_->publish(msg);
}

/**
 * @brief Checks if the path ahead is clear of obstacles.
 * @param scan Shared pointer to the laser scan data.
 * @return true if path is clear, false if obstacle detected.
 *
 * Examines laser scan data in a 90-degree arc in front of the robot
 * (45 degrees on each side) for obstacles within SAFE_DISTANCE.
 */
bool Walker::is_path_clear(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
  const int left_start = 0;     ///< Front center
  const int left_end = 17;      ///< 45 degrees to the left
  const int right_start = 343;  ///< 45 degrees to the right
  const int right_end = 359;    ///< Back to front center

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

/**
 * @brief Toggles the rotation direction of the robot.
 *
 * Multiplies the rotation_direction_ by -1.0 to switch between
 * clockwise and counter-clockwise rotation.
 */
void Walker::toggle_rotation_direction() { rotation_direction_ *= -1.0; }

/**
 * @brief Creates a timer with specified period and callback.
 * @param period Duration between timer callbacks.
 * @param callback Function to be called when timer expires.
 * @return Shared pointer to the created timer.
 */
rclcpp::TimerBase::SharedPtr Walker::create_timer(
    const std::chrono::duration<double>& period,
    std::function<void()> callback) {
  return this->create_wall_timer(period, callback);
}

/**
 * @brief Handles robot behavior in rotation state.
 * @param walker Pointer to the Walker object.
 * @param scan Shared pointer to laser scan data.
 *
 * Manages the robot's rotation behavior, including initial 10-second rotation
 * period and subsequent path checking. Transitions to ForwardState when path
 * becomes clear.
 */
void RotationState::handle(Walker* walker,
                           const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (initial_rotation_) {
    walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());

    if (!rotation_timer_) {
      rotation_timer_ =
          walker->create_timer(std::chrono::seconds(10), [this]() {
            initial_rotation_ = false;
            rotation_timer_ = nullptr;
          });

      RCLCPP_INFO(walker->get_logger(),
                  "Starting initial 10-second rotation period");
    }
  } else {
    if (walker->is_path_clear(scan)) {
      RCLCPP_INFO(walker->get_logger(), "Path is clear, moving forward");
      walker->change_state(new ForwardState());
    } else {
      walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());
      RCLCPP_INFO(walker->get_logger(), "Path blocked, continuing rotation");
    }
  }
}

/**
 * @brief Handles robot behavior in forward state.
 * @param walker Pointer to the Walker object.
 * @param scan Shared pointer to laser scan data.
 *
 * Manages the robot's forward movement behavior. Moves forward when path
 * is clear and transitions to RotationState when obstacle is detected,
 * toggling rotation direction before transition.
 */
void ForwardState::handle(Walker* walker,
                          const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (walker->is_path_clear(scan)) {
    walker->publish_velocity(0.5, 0.0);
  } else {
    walker->toggle_rotation_direction();
    RCLCPP_INFO(
        walker->get_logger(),
        "Obstacle detected, changing rotation direction and starting rotation");
    walker->change_state(new RotationState());
  }
}
