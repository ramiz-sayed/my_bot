cmake_minimum_required(VERSION 3.5)
project(my_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler flags for GNU/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the ament_cmake package
find_package(ament_cmake REQUIRED)

# Optionally find other dependencies, uncomment and add as needed
# find_package(rclcpp REQUIRED)    # For example, if you're using rclcpp
# find_package(sensor_msgs REQUIRED)  # Example dependency for sensor_msgs

# If you're using ROS 2 testing tools (e.g., gtest), enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Uncomment the following line when a copyright and license are not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # Uncomment the following line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Declare your executable and its source files
# Add your ROS 2 nodes or executables here, e.g.:
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node rclcpp)

# Install your nodes, launch files, configuration files, etc.
install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install your executable, if you have one
# install(TARGETS my_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Add further install statements for other resources like parameters, scripts, etc.

# Declare the ament package (this makes the package a ROS 2 package)
ament_package()
