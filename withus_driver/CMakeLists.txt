cmake_minimum_required(VERSION 3.8)
project(withus_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Threads REQUIRED)

add_library(withus_can_transport SHARED
  src/socket_can_transport.cpp
)

target_compile_features(withus_can_transport PUBLIC cxx_std_17)

target_include_directories(withus_can_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(withus_protocol SHARED
  src/withus_protocol.cpp
)

target_compile_features(withus_protocol PUBLIC cxx_std_17)

target_include_directories(withus_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_executable(can_connection_check
  src/can_connection_check.cpp
)

add_executable(withus_driver_node
  src/main.cpp
  src/withus_driver_node.cpp
)

target_link_libraries(withus_driver_node PRIVATE
  rclcpp::rclcpp
  withus_can_transport
  withus_protocol
  Threads::Threads
)

target_link_libraries(can_connection_check PRIVATE
  withus_can_transport
)

install(TARGETS 
  can_connection_check
  withus_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS withus_can_transport
  EXPORT export_withus_can_transport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS withus_protocol
  EXPORT export_withus_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(
  export_withus_can_transport
  export_withus_protocol 
  HAS_LIBRARY_TARGET
)

ament_package()
