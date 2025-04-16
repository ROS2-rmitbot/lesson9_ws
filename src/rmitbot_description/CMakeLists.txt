cmake_minimum_required(VERSION 3.8)
project(rmitbot_description)

find_package(ament_cmake REQUIRED)

# Folder to be added
install(
  DIRECTORY meshes urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
