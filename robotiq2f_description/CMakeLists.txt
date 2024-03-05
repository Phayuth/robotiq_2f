cmake_minimum_required(VERSION 3.10.2)
project(robotiq2f_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    meshes
    urdf
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
