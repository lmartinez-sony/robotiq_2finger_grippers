cmake_minimum_required(VERSION 3.5)
project(robotiq_2f_85_gripper_visualization)
find_package(ament_cmake REQUIRED)

# Install launch files.
install(DIRECTORY
  launch meshes urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

