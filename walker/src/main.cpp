#include "walker/walker_bot.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Walker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}