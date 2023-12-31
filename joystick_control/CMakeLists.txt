cmake_minimum_required(VERSION 3.8)
project(joystick_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_library(vehicle_cmd_filter ./include/joystick_control/vehicle_cmd_filter.hpp ./include/vehicle_cmd_filter.cpp)
ament_target_dependencies(vehicle_cmd_filter rclcpp autoware_auto_control_msgs)

target_include_directories(vehicle_cmd_filter PUBLIC ./include)

add_executable(joystick_interface ./src/joystick_interface.cpp)
ament_target_dependencies(joystick_interface rclcpp sensor_msgs tier4_vehicle_msgs autoware_auto_control_msgs autoware_auto_vehicle_msgs)
target_link_libraries(joystick_interface vehicle_cmd_filter)

install(TARGETS
  joystick_interface
  DESTINATION lib/${PROJECT_NAME}
)

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
