

add_executable(quad_controller
src/main.cpp
)

target_include_directories(quad_controller PRIVATE include)

target_link_libraries(quad_controller PRIVATE
fastcdr 
fastrtps
yaml-cpp
)

target_link_libraries(quad_controller PRIVATE
controller_lib
msg_lib
mocap_fastdds_lib
motor_commands_msg_lib
motor_commands_lib
)



