cmake_minimum_required (VERSION 2.6)
project (FK_KDL_Tutorial)

find_package (orocos_kdl REQUIRED)
include_directories (${orocos_kdl_INCLUDE_DIRS})

add_executable (SimpleRobotIK main.cpp)
target_link_libraries (SimpleRobotIK ${orocos_kdl_LIBRARIES})
