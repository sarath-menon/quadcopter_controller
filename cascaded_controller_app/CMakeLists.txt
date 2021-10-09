

add_executable(quad_controller
src/main.cpp
)

target_include_directories(quad_controller PRIVATE include)

target_link_libraries(quad_controller PRIVATE
fastcdr 
fastrtps
yaml-cpp
spdlog::spdlog
)

target_link_libraries(quad_controller PRIVATE
controller_lib
utilities_lib
comm_lib
)



