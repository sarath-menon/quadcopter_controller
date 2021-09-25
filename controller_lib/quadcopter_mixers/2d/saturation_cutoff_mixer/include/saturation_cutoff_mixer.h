#pragma once
#include "basic_controllers.h"
#include "math_helper.h"
#include <string>
#include <yaml-cpp/yaml.h>

class QuadcopterMixer {

private:
  // Quadcopter Properties
  float arm_length = 0;
  float k_f = 0;
  float motor_thrust_max = 0;
  float motor_thrust_min = 0;

public:
  // Mixer
  void motor_mixer(float motor_commands[4], const float thrust_command,
                   const float torque_command);

public:
  // To load quadcopter properties from yaml file
  void set_quad_properties(std::string path);
};