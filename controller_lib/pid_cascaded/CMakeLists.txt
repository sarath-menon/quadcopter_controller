
add_library(controller_lib STATIC
src/pid_cascaded.cpp
src/set_gains.cpp
)

target_include_directories(controller_lib PUBLIC include)

target_link_libraries(controller_lib PRIVATE
yaml-cpp
)

