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
find_package(withus_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

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
  src/withus_command_protocol.cpp
)

target_compile_features(withus_protocol PUBLIC cxx_std_17)

target_include_directories(withus_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_executable(can_connection_check
  src/can_connection_check.cpp
)

add_executable(can_send_check
  src/can_send_check.cpp
)

add_executable(withus_driver_node
  src/main.cpp
  src/withus_driver_node.cpp
  src/control_manager.cpp
  src/hardware_drive_controller.cpp
  src/can_interface.cpp
)

target_link_libraries(withus_driver_node PRIVATE
  rclcpp::rclcpp
  withus_can_transport
  withus_protocol
  Threads::Threads
)

ament_target_dependencies(withus_driver_node PUBLIC
  withus_msgs
  sensor_msgs
  diagnostic_msgs
  rcl_interfaces
)

target_link_libraries(can_connection_check PRIVATE
  withus_can_transport
)

target_link_libraries(can_send_check PRIVATE
  withus_can_transport
)

install(TARGETS 
  can_connection_check
  can_send_check
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

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  foreach(test_name IN ITEMS test_control_manager test_hardware_mode_request)
    add_executable(${test_name} test/${test_name}.cpp src/control_manager.cpp)
    target_compile_features(${test_name} PRIVATE cxx_std_17)
    target_include_directories(${test_name} PRIVATE include)
    ament_add_test(${test_name} COMMAND "$<TARGET_FILE:${test_name}>"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  endforeach()
  # The CAN/ROS integration test is run explicitly on an existing vcan link.
  add_executable(test_hardware_drive_controller
    test/test_hardware_drive_controller.cpp src/hardware_drive_controller.cpp)
  target_compile_features(test_hardware_drive_controller PRIVATE cxx_std_17)
  target_include_directories(test_hardware_drive_controller PRIVATE include)
  ament_add_test(test_hardware_drive_controller COMMAND "$<TARGET_FILE:test_hardware_drive_controller>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_package()
