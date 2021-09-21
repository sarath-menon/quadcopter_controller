cmake_minimum_required(VERSION 3.16.0)
project("fastdds_motor_commands" VERSION 1.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

###############################################################################
# CMake build rules
###############################################################################
set(IS_TOP_LEVEL TRUE)
if(PROJECT_SOURCE_DIR)
    set(IS_TOP_LEVEL FALSE)
endif()


###############################################################################
# Load external packages
###############################################################################

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

###############################################################################
# Library for message types
###############################################################################

add_library(motor_commands_msg_lib
include/motor_commands.cxx
)

target_link_libraries(motor_commands_msg_lib
fastcdr 
fastrtps
)
###############################################################################
# Library for publishers, subscirbers and such
###############################################################################

add_library(motor_commands_lib
include/motor_commandsPubSubTypes.cxx
include/motor_commandsPublisher.cxx
include/motor_commandsSubscriber.cxx
)

target_link_libraries(motor_commands_lib
fastcdr 
fastrtps
)

target_include_directories(motor_commands_lib PUBLIC include)



