
add_library(basic_controllers_lib STATIC
src/pid.cpp
)

target_include_directories(basic_controllers_lib PUBLIC include)

