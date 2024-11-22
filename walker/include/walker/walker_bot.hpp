/**
 * @file walker_bot.hpp
 * @brief Header file defining the Walker bot class and its states for ROS2.
 */

#ifndef WALKER_WALKER_BOT_HPP
#define WALKER_WALKER_BOT_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/**
 * @brief Forward declaration of the WalkerState class.
 */
class WalkerState;

/**
 * @class Walker
 * @brief Implements a robot walker node with state management.
 */
class Walker : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the Walker class.
   */
  Walker();

  /**
   * @brief Changes the current state of the Walker.
   * @param new_state Pointer to the new state to transition to.
   */
  void change_state(WalkerState* new_state);

  /**
   * @brief Publishes velocity commands to the robot.
   * @param linear Linear velocity component.
   * @param angular Angular velocity component.
   */
  void publish_velocity(double linear, double angular);

  /**
   * @brief Determines if the path is clear based on laser scan data.
   * @param scan Shared pointer to the LaserScan message.
   * @return True if the path is clear, false otherwise.
   */
  bool is_path_clear(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;

  /**
   * @brief Toggles the rotation direction between clockwise and
   * counterclockwise.
   */
  void toggle_rotation_direction();

  /**
   * @brief Gets the current rotation direction.
   * @return Rotation direction (1.0 for CCW, -1.0 for CW).
   */
  double get_rotation_direction() const { return rotation_direction_; }

 private:
  /**
   * @brief Callback function for LaserScan messages.
   * @param scan Shared pointer to the received LaserScan message.
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  WalkerState* current_state_;  ///< Pointer to the current state of the walker.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      vel_publisher_;  ///< Publisher for velocity commands.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscriber_;        ///< Subscriber for LaserScan messages.
  double rotation_direction_;  ///< Current rotation direction (1.0 for CCW,
                               ///< -1.0 for CW).
  const double SAFE_DISTANCE = 1.5;  ///< Minimum safe distance from obstacles.
};

/**
 * @class WalkerState
 * @brief Abstract base class for defining Walker states.
 */
class WalkerState {
 public:
  /**
   * @brief Virtual destructor for the WalkerState class.
   */
  virtual ~WalkerState() = default;

  /**
   * @brief Handles the behavior of the Walker in the current state.
   * @param walker Pointer to the Walker object.
   * @param scan Shared pointer to the LaserScan message.
   */
  virtual void handle(Walker* walker,
                      const sensor_msgs::msg::LaserScan::SharedPtr scan) = 0;
};

/**
 * @class ForwardState
 * @brief State in which the Walker moves forward.
 */
class ForwardState : public WalkerState {
 public:
  /**
   * @brief Handles the behavior of the Walker in the forward state.
   * @param walker Pointer to the Walker object.
   * @param scan Shared pointer to the LaserScan message.
   */
  void handle(Walker* walker,
              const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
};

/**
 * @class RotationState
 * @brief State in which the Walker rotates in place.
 */
class RotationState : public WalkerState {
 public:
  /**
   * @brief Handles the behavior of the Walker in the rotation state.
   * @param walker Pointer to the Walker object.
   * @param scan Shared pointer to the LaserScan message.
   */
  void handle(Walker* walker,
              const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
};

#endif  // WALKER_WALKER_BOT_HPP
