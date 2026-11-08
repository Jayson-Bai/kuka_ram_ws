cmake_minimum_required(VERSION 3.8)
project(runtime_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_project_interfaces REQUIRED)
find_package(runtime_logging_core REQUIRED)

add_executable(runtime_logger_node
  src/runtime_logger_node.cpp
  src/collector.cpp
  src/integrity.cpp
  src/jsonl_writer.cpp
  src/session_manifest.cpp
)
target_compile_features(runtime_logger_node PUBLIC cxx_std_17)
target_include_directories(runtime_logger_node PRIVATE include)
ament_target_dependencies(runtime_logger_node
  rclcpp std_msgs my_project_interfaces runtime_logging_core)

add_executable(runtime_telemetry_ui_node
  src/runtime_telemetry_ui_node.cpp
)
target_compile_features(runtime_telemetry_ui_node PUBLIC cxx_std_17)
ament_target_dependencies(runtime_telemetry_ui_node
  rclcpp std_msgs my_project_interfaces)

install(TARGETS runtime_logger_node runtime_telemetry_ui_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/check_runtime_log.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_check_runtime_log test/test_check_runtime_log.py)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_integrity
    test/test_integrity.cpp
    src/integrity.cpp)
  target_include_directories(test_integrity PRIVATE include)
  target_compile_features(test_integrity PUBLIC cxx_std_17)

  ament_add_gtest(test_session_manifest
    test/test_session_manifest.cpp
    src/session_manifest.cpp)
  target_include_directories(test_session_manifest PRIVATE include)
  target_compile_features(test_session_manifest PUBLIC cxx_std_17)
endif()

ament_package()
