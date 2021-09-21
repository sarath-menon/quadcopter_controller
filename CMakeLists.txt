cmake_minimum_required(VERSION 3.20)
project(quad_controller VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)

###############################################################################
# Find installed packages
###############################################################################

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

include_directories(external/PX4-Matrix)


add_subdirectory(project)
add_subdirectory(controller)
add_subdirectory(external)
