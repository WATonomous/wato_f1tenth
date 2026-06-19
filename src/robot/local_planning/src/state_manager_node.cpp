#include "planning/state_manager_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<local_planning::StateManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
