

add_executable(pos_controller_3d
src/main.cpp
)

target_include_directories(pos_controller_3d PRIVATE include)

target_link_libraries(pos_controller_3d PRIVATE
fastcdr 
fastrtps
yaml-cpp
spdlog::spdlog
)

target_link_libraries(pos_controller_3d PRIVATE
controller_lib
utilities_lib
comm_lib
)



