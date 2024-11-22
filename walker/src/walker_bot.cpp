#include "walker/walker_bot.hpp"

Walker::Walker() : Node("walker"), rotation_direction_(1.0) {
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
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

bool Walker::is_path_clear(const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
    // Check front area for obstacles
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

void Walker::toggle_rotation_direction() {
    rotation_direction_ *= -1.0;
}

void RotationState::handle(Walker* walker, const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (walker->is_path_clear(scan)) {
        walker->change_state(new ForwardState());
    } else {
        walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());  // Rotate
    }
}

void ForwardState::handle(Walker* walker, const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (walker->is_path_clear(scan)) {
        walker->publish_velocity(0.2, 0.0);  // Move forward
    } else {
        walker->toggle_rotation_direction();  // Alternate rotation direction
        walker->change_state(new RotationState());
    }
}