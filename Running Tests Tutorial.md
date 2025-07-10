### Using and Writing Tests for Micro Autonomy Pipelines

Some GitHub workflows have been set up in this repo to run tests on any PRs against main. It checks for **code style, if it compiles properly, and any unit tests** you may want to add. These are nearly identical to the ones in the monorepo, so you can also find more examples there.

After making a PR, check the `Actions` panel in the repo. You should be able to find the test results run on your branch.

### Using the Auto-Linter
To use the auto-linter on a PR against main, add the green `auto-lint` label on the sidebar. The WATonomous bot will add a commit to reformat your code.

### Adding Unit Tests

#### 1. To add a test for a ROS node, make a `test` folder in the node with a testing class.

Example structure with `gym_vis` node:

```
└── gym_vis
    ├── assets
    │   └── f1tenth_car.stl
    ├── CMakeLists.txt
    ├── include
    │   └── gym_vis.hpp
    ├── LICENSE
    ├── package.xml
    ├── src
    │   └── gym_vis.cpp
    └── test
        └── test_gym_vis.cpp
```

#### 2. Write your tests in `test_{node_name}.cpp`. Make sure to import the `#include "gtest/gtest.h"` library for testing.

It should look something like this.

```cpp
#include "gtest/gtest.h"
#include <rclcpp/rclcpp.hpp>
#include "gym_vis.hpp"

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
// you can start by adding this test to make sure it runs the test as expected
TEST(GymVisTest, TestTheTestSuite) { EXPECT_TRUE(true); }
```

#### 3. Add any dependencies you need in `CMakeLists.txt`
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_gym_vis.cpp
    # src/gym_vis.cpp // example
  )
  target_include_directories(${PROJECT_NAME}_test PUBLIC include)
  # add your dependencies. ex:
  # ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs nav_msgs visualization_msgs sensor_msgs ackermann_msgs tf2_ros tf2_geometry_msgs)
endif()
```

### 4. Similarly, add dependencies in `package.xml`

Namely, `<test_depend>ament_cmake_gtest</test_depend>`

### Last thing,
The build/testing pipeline is especially slow on GitHub so it's a good idea to run it locally first.

It's something like:
```bash
./watod build
./watod up
```
Build find, and enter your docker container (looks like: ...robot-dev...)
```bash
docker ps
docker exec -it {container-id} /bin/bash
source /opt/ros/humble/setup.bash
```
Build your node
```bash
cd {into your node}
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select {your_ros_node}
```

Run your tests
```bash
colcon test --packages-select {node_name}
colcon test-result --verbose
```
