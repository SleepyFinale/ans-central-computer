################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_navigation2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
set(_tb3_nav2_share_dirs launch map rviz param)
foreach(_dir IN LISTS _tb3_nav2_share_dirs)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}")
    install(
      DIRECTORY ${_dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()
install(
  PROGRAMS scripts/normalize_laser_scan.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
