cmake_minimum_required(VERSION 3.8)
project(maxon_epos4_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(canopen REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(maxon_epos4_ros2 REQUIRED)



cogen_dcf(bus_config_cia402_epos4_pos)
cogen_dcf(bus_config_cia402_epos4_vel)

include_directories(src)

add_executable(epos4_pos src/epos4_pos.cpp)
# Link libraries
ament_target_dependencies(epos4_pos
  rclcpp
  std_msgs
  std_srvs
  canopen_interfaces
  sensor_msgs
)

add_executable(epos4_vel src/epos4_vel.cpp)
# Link libraries
ament_target_dependencies(epos4_vel
  rclcpp
  std_msgs
  std_srvs
  canopen_interfaces
  sensor_msgs
)

install(TARGETS epos4_pos epos4_vel
  DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
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
