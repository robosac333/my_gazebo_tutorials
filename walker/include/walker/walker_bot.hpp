#ifndef WALKER_WALKER_BOT_HPP
#define WALKER_WALKER_BOT_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class WalkerState;

class Walker : public rclcpp::Node {
public:
    Walker();
    void change_state(WalkerState* new_state);
    void publish_velocity(double linear, double angular);
    bool is_path_clear(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;
    void toggle_rotation_direction();
    double get_rotation_direction() const { return rotation_direction_; }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    
    WalkerState* current_state_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    double rotation_direction_;  // 1.0 for CCW, -1.0 for CW
    const double SAFE_DISTANCE = 1.5;
};

class WalkerState {
public:
    virtual ~WalkerState() = default;
    virtual void handle(Walker* walker, const sensor_msgs::msg::LaserScan::SharedPtr scan) = 0;
};

class ForwardState : public WalkerState {
public:
    void handle(Walker* walker, const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
};

class RotationState : public WalkerState {
public:
    void handle(Walker* walker, const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
};

#endif // WALKER_WALKER_BOT_HPP