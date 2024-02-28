#include "utils/config_yaml.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

// std::string package_path = ament_index_cpp::get_package_share_directory("lci_calibration");
// YAML::Node ConfigYaml::cfg_root = YAML::LoadFile(package_path + "/lci_calibration/config/config.config.yaml");
const std::string ConfigYaml::config_file = "/root/workspace/ros2/src/lci_calibration/config/config.yaml";
YAML::Node ConfigYaml::cfg_root = YAML::LoadFile(config_file);


const std::string ConfigYaml::if_prefix = "if_";
const std::string ConfigYaml::int_prefix = "int_";
const std::string ConfigYaml::float_prefix = "float_";

ConfigYaml cfg;

