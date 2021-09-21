

add_executable(quad_controller
main.cpp
)

target_include_directories(quad_controller PRIVATE include)

target_link_libraries(quad_controller PRIVATE
controller_lib
yaml-cpp
fastcdr 
fastrtps
msg_lib
mocap_fastdds_lib
)




