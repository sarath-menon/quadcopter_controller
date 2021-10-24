

add_executable(quad_controller_2d
src/main.cpp
)

target_include_directories(quad_controller_2d PRIVATE include)

target_link_libraries(quad_controller_2d PRIVATE
fastcdr 
fastrtps
yaml-cpp
spdlog::spdlog
)

target_link_libraries(quad_controller_2d PRIVATE
controller_lib
utilities_lib
comm_lib
)



