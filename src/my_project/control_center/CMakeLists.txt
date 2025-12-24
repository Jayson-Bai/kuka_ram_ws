cmake_minimum_required(VERSION 3.8)
project(control_center)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_project_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ZLIB REQUIRED)

add_library(cnpy STATIC
  third_party/cnpy/cnpy.cpp
)
target_include_directories(cnpy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/cnpy>
)
target_link_libraries(cnpy ZLIB::ZLIB)

add_executable(center_node
  src/center_node.cpp
  src/npz_loader.cpp
  src/queue_manager.cpp
)

ament_target_dependencies(
  center_node
  rclcpp
  my_project_interfaces
  std_msgs
)
target_include_directories(center_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(center_node cnpy)

install(TARGETS center_node DESTINATION lib/${PROJECT_NAME})

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
