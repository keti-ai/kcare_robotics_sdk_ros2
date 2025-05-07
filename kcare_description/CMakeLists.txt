cmake_minimum_required(VERSION 3.5)
project(kcare_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rviz2 REQUIRED)

# Install launch files.
install(DIRECTORY
  launch robots rviz meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()