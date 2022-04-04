cmake_minimum_required(VERSION 3.8)
project(ball_trajectory_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(fmt REQUIRED)

# add_executable(trajectory_node src/trajectory_node.cpp)
# target_include_directories(trajectory_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(trajectory_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(trajectory_node rclcpp geometry_msgs sensor_msgs can_msgs fmt)

add_executable(basic_trajectory_node src/basic_trajectory.cpp)
target_include_directories(basic_trajectory_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(basic_trajectory_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(basic_trajectory_node fmt)
ament_target_dependencies(basic_trajectory_node rclcpp geometry_msgs sensor_msgs can_msgs fmt)

install(TARGETS basic_trajectory_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
