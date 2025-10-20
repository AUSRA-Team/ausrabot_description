cmake_minimum_required(VERSION 3.8)
project(ausrabot_description)

# Find necessary ROS 2 build tools
find_package(ament_cmake REQUIRED)

# Install the package.xml file
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install all the necessary directories from your package
install(
  DIRECTORY urdf launch meshes config
  DESTINATION share/${PROJECT_NAME}
)

# This command must be at the end
ament_package()