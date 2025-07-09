#include "gtest/gtest.h"
#include "../src/gym_vis.cpp"
#include <rclcpp/rclcpp.hpp>

TEST(GymVisTest, MakeNode) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<GymVis>();
    EXPECT_EQ(node->get_name(), std::string("gym_vis"));
    rclcpp::shutdown();
}

TEST(GymVisTest, HasPublishers) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<GymVis>();
    auto publishers = node->get_publishers_info_by_topic("f1tenth_car");
    EXPECT_FALSE(publishers.empty());
    rclcpp::shutdown();
}