cmake_minimum_required(VERSION 3.16)
project(st3215_hardware)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(SERIAL REQUIRED libserial)

# Include directories
include_directories(include)

# Create the hardware interface library
add_library(${PROJECT_NAME}
  src/st3215_communication.cpp
  src/st3215_servo.cpp
  src/st3215_hardware_interface.cpp
)

# Specify dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# Link serial library
target_link_libraries(${PROJECT_NAME} ${SERIAL_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${SERIAL_INCLUDE_DIRS})

# Export targets
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install plugin description file
install(FILES st3215_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# Plugin export
pluginlib_export_plugin_description_file(hardware_interface st3215_hardware.xml)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # Run linting
  ament_lint_auto_find_test_dependencies()
  
  # Try to find ament_gtest and add tests if available
  find_package(ament_gtest QUIET)
  if(ament_gtest_FOUND)
    # Add unit tests
    ament_add_gtest(test_st3215_communication test/test_st3215_communication.cpp)
    target_link_libraries(test_st3215_communication ${PROJECT_NAME})
    ament_target_dependencies(test_st3215_communication rclcpp)
    
    ament_add_gtest(test_st3215_hardware_interface test/test_st3215_hardware_interface.cpp)
    target_link_libraries(test_st3215_hardware_interface ${PROJECT_NAME})
    ament_target_dependencies(test_st3215_hardware_interface rclcpp hardware_interface)
  else()
    message(WARNING "ament_gtest not found, skipping unit tests")
  endif()
endif()

ament_package()