cmake_minimum_required(VERSION 3.8)
project(runtime_logging_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(runtime_logging_core STATIC
  src/record_codec.cpp
  src/bounded_ring.cpp
  src/whitelist.cpp
  src/ipc_client.cpp
)
target_include_directories(runtime_logging_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(runtime_logging_core PUBLIC cxx_std_17)

install(TARGETS runtime_logging_core
  EXPORT export_runtime_logging_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_runtime_logging_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bounded_ring test/test_bounded_ring.cpp)
  target_link_libraries(test_bounded_ring runtime_logging_core)
  ament_add_gtest(test_whitelist test/test_whitelist.cpp)
  target_link_libraries(test_whitelist runtime_logging_core)
  ament_add_gtest(test_record_codec test/test_record_codec.cpp)
  target_link_libraries(test_record_codec runtime_logging_core)
endif()

ament_package()
