cmake_minimum_required(VERSION 3.8)
project(homework_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executables for each node
add_executable(division_node src/division_node.cpp)
ament_target_dependencies(division_node rclcpp std_msgs)

add_executable(division_node_B src/division_node_B.cpp)
ament_target_dependencies(division_node_B rclcpp std_msgs)

add_executable(division_node_C src/division_node_C.cpp)
ament_target_dependencies(division_node_C rclcpp std_msgs)

# Install targets
install(TARGETS
  division_node
  division_node_B
  division_node_C
  DESTINATION lib/${PROJECT_NAME}
)

# Test dependencies (if BUILD_TESTING is enabled)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
