cmake_minimum_required(VERSION 3.8)
project(humanoid_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install RViz config files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

# Install meshes if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
