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

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament_cmake package
find_package(ament_cmake REQUIRED)

# Optionally find other dependencies
# find_package(<dependency> REQUIRED)

# Add testing support
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Uncomment the following line when a copyright and license are not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # Uncomment the following line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install directories
install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Declare the ament package
ament_package()
