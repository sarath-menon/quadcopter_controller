#pragma once
#include <yaml-cpp/yaml.h>

///////////////////////////////////////////////////////////////////////////////////////////
// Quadcopter Properties
///////////////////////////////////////////////////////////////////////////////////////////

// YAML::Node quad_yaml = YAML::LoadFile(
//     "cascaded_controller_app/parameters/quadcopter_parameters.yaml");

// const float arm_length = quad_yaml["arm_length"].as<float>();
// const float thrust_max = quad_yaml["motor_thrust_max"].as<float>() * 4;
// const float thrust_min = quad_yaml["motor_thrust_min"].as<float>() * 4;
// const float k_f = quad_yaml["k_f"].as<float>();

///////////////////////////////////////////////////////////////////////////////////////////
// Setpoint
///////////////////////////////////////////////////////////////////////////////////////////

YAML::Node setpoint_yaml = YAML::LoadFile(
    "/Users/sarathmenon/Desktop/eth_soft/code/Controllers/"
    "quadcopter_controller/cascaded_controller_app/parameters/setpoint.yaml");

const float x_position_target = setpoint_yaml["x_position_target"].as<float>();
const float y_position_target = setpoint_yaml["y_position_target"].as<float>();
const float z_position_target = setpoint_yaml["z_position_target"].as<float>();