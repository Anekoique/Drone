#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>
#include <string>

namespace drone
{

class ConfigLoader
{
public:
  static constexpr auto kPackageName = "drone";

  static YAML::Node load(const std::string & filename)
  {
    auto path = std::filesystem::path(ament_index_cpp::get_package_share_directory(kPackageName)) /
                "config" / filename;

    if (!std::filesystem::exists(path)) {
      throw std::runtime_error("Config file not found: " + path.string());
    }

    return YAML::LoadFile(path.string());
  }
};

}  // namespace drone
