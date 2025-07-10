#include "gtest/gtest.h"
// #include <rclcpp/rclcpp.hpp>
// should probably make a header file to access because including the cpp has
// main... #include "../src/gym_vis.cpp"

// TEST(GymVisTest, MakeNode) {
//     rclcpp::init(0, nullptr);
//     auto node = std::make_shared<GymVis>();
//     EXPECT_EQ(node->get_name(), std::string("gym_vis"));
//     rclcpp::shutdown();
// }

// TEST(GymVisTest, NodeExists) {
//     rclcpp::init(0, nullptr);
//     auto node = std::make_shared<GymVis>();
//     EXPECT_TRUE(node != nullptr);
//     rclcpp::shutdown();
// }

TEST(GymVisTest, TestTheTestSuite) { EXPECT_TRUE(true); }