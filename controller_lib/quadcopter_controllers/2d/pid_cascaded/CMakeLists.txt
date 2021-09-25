
add_library(quadcopter_controllers_lib STATIC
src/pid_cascaded.cpp
src/set_values.cpp
)

target_include_directories(quadcopter_controllers_lib PUBLIC include)

target_link_libraries(quadcopter_controllers_lib PRIVATE
basic_controllers_lib
yaml-cpp
)

