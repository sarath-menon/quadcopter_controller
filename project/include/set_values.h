#pragma once
#include <yaml-cpp/yaml.h>

///////////////////////////////////////////////////////////////////////////////////////////
// Simulation Properties
///////////////////////////////////////////////////////////////////////////////////////////

YAML::Node sim_yaml_file =
    YAML::LoadFile("project/parameters/simulation_parameters.yaml");

const float altitude_target = sim_yaml_file["altitude_target"].as<float>();
const float horizontal_target = sim_yaml_file["horizontal_target"].as<float>();

// Euler integration timestep
const float dt = sim_yaml_file["dt"].as<float>();
const float euler_steps = sim_yaml_file["euler_steps"].as<float>();
const int sim_time = dt * 1000;
// Fastdds publisher activate or not
const bool pose_pub_flag = sim_yaml_file["pose_pub"].as<bool>();

///////////////////////////////////////////////////////////////////////////////////////////
// Plot Properties
///////////////////////////////////////////////////////////////////////////////////////////

YAML::Node plot_yaml_file =
    YAML::LoadFile("project/parameters/plot_parameters.yaml");

const bool plot_flag = plot_yaml_file["plot_flag"].as<bool>();

const bool altitude_plot_flag = plot_yaml_file["altitude_plot"].as<bool>();
const bool translation_plot_flag =
    plot_yaml_file["translation_plot"].as<bool>();
const bool thrust_plot_flag = plot_yaml_file["thrust_plot"].as<bool>();
const bool torque_plot_flag = plot_yaml_file["torque_plot"].as<bool>();
const bool roll_angle_flag = plot_yaml_file["roll_angle_plot"].as<bool>();
