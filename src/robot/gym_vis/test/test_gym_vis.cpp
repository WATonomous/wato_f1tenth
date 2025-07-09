#include "gtest/gtest.h"
#include "gym_vis.hpp"
#include <rclcpp/rclcpp.hpp>

TEST(GymVisTest, MakeNode) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<GymVis>();
    EXPECT_EQ(node->get_name(), std::string("gym_vis"));
    rclcpp::shutdown();
}

TEST(GymVisTest, NodeExists) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<GymVis>();
    EXPECT_TRUE(node != nullptr);
    rclcpp::shutdown();
}