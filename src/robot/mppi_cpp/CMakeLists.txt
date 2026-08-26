cmake_minimum_required(VERSION 3.18)
project(mppi_cpp LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
if(NOT CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 87)  # Jetson Orin
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(mppi_cuda STATIC src/mppi_cuda.cu)
target_include_directories(mppi_cuda PUBLIC include)
target_link_libraries(mppi_cuda PUBLIC CUDA::cudart)

add_executable(mppi_node src/mppi_node.cpp src/track.cpp src/wall_sdf.cpp)
target_include_directories(mppi_node PRIVATE include)
target_link_libraries(mppi_node mppi_cuda Eigen3::Eigen yaml-cpp)
ament_target_dependencies(mppi_node
  rclcpp ament_index_cpp ackermann_msgs geometry_msgs nav_msgs std_msgs visualization_msgs)

install(TARGETS mppi_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch data DESTINATION share/${PROJECT_NAME})

ament_package()
