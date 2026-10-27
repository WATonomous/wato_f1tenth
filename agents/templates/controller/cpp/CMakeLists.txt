# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.8)
project(my_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# The control law, without ROS plumbing, so unit tests can call it directly
add_library(${PROJECT_NAME}_lib src/my_controller.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC ${nav_msgs_TARGETS})

add_executable(${PROJECT_NAME}_node src/my_controller_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${ackermann_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(wato_test REQUIRED)
  add_wato_test(test_my_controller
    test/test_my_controller.cpp
    LIBRARIES
      ${PROJECT_NAME}_lib
  )
endif()

ament_package()
