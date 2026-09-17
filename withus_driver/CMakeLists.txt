cmake_minimum_required(VERSION 3.8)
project(withus_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_library(withus_can_transport SHARED
  src/socket_can_transport.cpp
)

target_compile_features(withus_can_transport PUBLIC cxx_std_17)

target_include_directories(withus_can_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
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

ament_export_targets(export_withus_can_transport HAS_LIBRARY_TARGET)

ament_package()
