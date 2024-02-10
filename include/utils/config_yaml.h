#ifndef __CONFIG_YAML__
#define __CONFIG_YAML__

#include "yaml-cpp/yaml.h"
// #include "yaml-cpp/node/node.h"
// #include "yaml-cpp/node/parse.h"

class ConfigYaml{
public:
    const std::string config_file = "../../share/lci_calibration/config/config.yaml";
    static YAML::Node cfg_root;
    ConfigYaml(){
        
    }
private:

};

#endif
