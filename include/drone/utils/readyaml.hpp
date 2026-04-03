// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>
#include <string>

namespace drone
{

/// Loads YAML config files from the package's config/ directory.
class ConfigLoader
{
public:
  static constexpr auto kPackageName = "drone";

  /// Load a YAML file from the package share config directory.
  /// @param filename Name of the YAML file (e.g. "mission.yaml").
  /// @return Parsed YAML node tree.
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
