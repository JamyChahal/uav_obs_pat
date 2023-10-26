cmake_minimum_required(VERSION 3.5)
project(tello_description)

find_package(ament_cmake REQUIRED)

#=============
# Install
#=============

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/xacro")

install(
  DIRECTORY xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()