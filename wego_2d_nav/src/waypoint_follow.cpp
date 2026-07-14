#include "wego_2d_nav/waypoint_follow.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DoosanWaypointFollow>());
    rclcpp::shutdown();
    return 0;
}