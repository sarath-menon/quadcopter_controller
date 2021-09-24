
add_library(quadcopter_mixers_lib STATIC
src/saturation_cutoff_mixer.cpp
src/set_values.cpp
)

target_include_directories(quadcopter_mixers_lib PUBLIC include)

target_link_libraries(quadcopter_mixers_lib PRIVATE
basic_controllers_lib
yaml-cpp
)

