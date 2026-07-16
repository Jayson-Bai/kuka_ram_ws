cmake_minimum_required(VERSION 3.8)
project(uart_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_project_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_library(uart_extrusion_wire STATIC src/extrusion_wire.cpp)
target_include_directories(uart_extrusion_wire PUBLIC include)
target_compile_features(uart_extrusion_wire PUBLIC cxx_std_17)

add_executable(uart_node src/uart_node.cpp)
target_compile_features(uart_node PUBLIC cxx_std_17)
ament_target_dependencies(
  uart_node
  rclcpp
  std_msgs
  my_project_interfaces
)
target_link_libraries(uart_node uart_extrusion_wire Boost::system)

install(TARGETS uart_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_extrusion_wire test/test_extrusion_wire.cpp)
  target_link_libraries(test_extrusion_wire uart_extrusion_wire)
  ament_add_pytest_test(
    test_extrude_reset_handshake
    test/test_extrude_reset_handshake.py
  )
  ament_add_pytest_test(
    test_uart_extrusion_boundary_contract
    test/test_uart_extrusion_boundary_contract.py
  )
  ament_add_pytest_test(
    test_temperature_ready_threshold
    test/test_temperature_ready_threshold.py
  )

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
