#pragma once

#include <iostream>
#include <yaml-cpp/yaml.h>

namespace pandar128 {

template <typename _T>
bool TryGetParameter(const YAML::Node &cfg, const std::string &name,
                     _T &param) {
  if (auto value = cfg[name]) {
    try {
      param = value.as<_T>();
    } catch (const std::exception &) {
      std::cerr << "Parameter '" << name << "' exists but failed to load";
      return false;
    }
  }
  return true;
}

template <typename _T>
bool MustGetParameter(const YAML::Node &cfg, const std::string &name,
                      _T &param) {
  try {
    param = cfg[name].as<_T>();
  } catch (const std::exception &) {
    std::cerr << "Parameter '" << name << "' failed to load";
  }
  return true;
}

} // namespace pandar128
