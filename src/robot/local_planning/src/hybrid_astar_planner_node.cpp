#include "planning/hybrid_astar_planner_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<local_planning::HybridAStarPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
