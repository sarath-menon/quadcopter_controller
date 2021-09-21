
add_library(controller_lib STATIC
pid_cascaded/pid_cascaded.cpp
pid_cascaded/pid_controllers.cpp
)

target_include_directories(controller_lib PUBLIC
controller_common
pid_cascaded
)

target_link_libraries(controller_lib PRIVATE
model_lib
yaml-cpp
)

