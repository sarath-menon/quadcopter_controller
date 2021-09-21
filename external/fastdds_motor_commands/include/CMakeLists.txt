
cmake_minimum_required(VERSION 2.8.12)

project("generated_code")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find requirements
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)


message(STATUS "Configuring motor_commands...")
add_library(motor_commands_lib motor_commands.cxx)
target_link_libraries(motor_commands_lib fastcdr fastrtps)

add_executable(motor_commands motor_commandsPubSubTypes.cxx motor_commandsPublisher.cxx motor_commandsSubscriber.cxx motor_commandsPubSubMain.cxx)
target_link_libraries(motor_commands fastcdr fastrtps
        motor_commands_lib )



