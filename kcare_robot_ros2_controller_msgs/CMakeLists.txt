cmake_minimum_required(VERSION 3.5)
project(kcare_robot_ros2_controller_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LMCommand.msg"
  "msg/LMState.msg"
  "msg/HeadCommand.msg"
  "msg/HeadState.msg"
  "msg/HeadGoHomeRequest.msg"
  "msg/GripperCommand.msg"
  "msg/GripperState.msg"
  "msg/RemoteCommand.msg"
  "srv/GripperCommand.srv"
  "srv/ElevationCommand.srv"
  "srv/HeadPoseCommand.srv"
  "srv/MobileMoveLabel.srv"
  DEPENDENCIES std_msgs
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
