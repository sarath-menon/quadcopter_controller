#include "saturation_cutoff_mixer.h"

void QuadcopterMixer::set_quad_properties(std::string path) {
  // Load yaml file containing quadcopter properties
  YAML::Node mixer_yaml = YAML::LoadFile(path);

  arm_length = mixer_yaml["arm_length"].as<float>();
  k_f = mixer_yaml["k_f"].as<float>();
  motor_thrust_max = mixer_yaml["motor_thrust_max"].as<float>();
  motor_thrust_min = mixer_yaml["motor_thrust_min"].as<float>();
}
