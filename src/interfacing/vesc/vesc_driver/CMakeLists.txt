cmake_minimum_required(VERSION 3.5)
project(vesc_driver)

# std::optional needs C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Threads)

###########
## Build ##
###########

# node library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/vesc_driver.cpp
  src/vesc_interface.cpp
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${CMAKE_THREAD_LIBS_INIT}
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN vesc_driver::VescDriver
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_add_executable(
  vesc_device_namer
  src/vesc_device_namer.cpp
  src/vesc_device_uuid_lookup.cpp
)

#############
## Testing ##
#############

# Linting runs through pre-commit; unit tests use wato_test (add_wato_test).

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    params
)
