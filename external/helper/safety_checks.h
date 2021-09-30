#pragma once
#include <filesystem>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>

namespace safety_checks {

// Checks whether given yaml files exist
inline void yaml_file_vector_check(std::vector<std::string> files_to_check) {

  for (const auto &file : files_to_check) {
    try {
      if (std::filesystem::exists(file) == false) {
        throw(file);
      }
    } catch (std::string file) {
      std::cerr << "YAML file error: " << file << " does not exist" << '\n';
      std::exit(EXIT_FAILURE);
    }
  }
}

// Checks whether given yaml files exist
inline void yaml_file_check(std::string yaml_file) {

  try {
    if (std::filesystem::exists(yaml_file) == false)
      throw(yaml_file);
  } catch (std::string yaml_file) {
    std::cout << "YAML file error: " << yaml_file << " does not exist" << '\n';
    std::exit(EXIT_FAILURE);
  }
}

} // namespace safety_checks