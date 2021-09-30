cmake_minimum_required(VERSION 3.20)
project(quad_controller VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)

###############################################################################
# Find installed packages
###############################################################################

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(spdlog REQUIRED)

include_directories(external/PX4-Matrix)
include_directories(external/helper)

add_subdirectory(cascaded_controller_app)
add_subdirectory(lib)
add_subdirectory(external)
