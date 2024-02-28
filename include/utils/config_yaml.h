#ifndef __CONFIG_YAML__
#define __CONFIG_YAML__

#include "yaml-cpp/yaml.h"
#include <type_traits> 
#include <iostream>
#include <fstream>
// #include "yaml-cpp/node/node.h"
// #include "yaml-cpp/node/parse.h"
// #define CFG_LINE() std::cout << "[" << __FILE__ << "]" << "[" << __LINE__ << "]"

#define CFG_LINE() _Pragma("message(\"if_\" __FILE__ __LINE__)") 
#define CFG_IF() ConfigYaml::cfgDebugData<bool>(ConfigYaml::configGetName(__FILE__, __LINE__, "if_"))
#define CFG_INT() ConfigYaml::cfgDebugData<int>(ConfigYaml::configGetName(__FILE__, __LINE__, "int_"))
#define CFG_FLOAT() ConfigYaml::cfgDebugData<double>(ConfigYaml::configGetName(__FILE__, __LINE__, "float_"))
#define CFG_GET_INT(str) ConfigYaml::cfg_root[str].as<int>()
#define CFG_GET_FLOAT(str) ConfigYaml::cfg_root[str].as<double>()
#define CFG_GET_BOOL(str) ConfigYaml::cfg_root[str].as<bool>()

class ConfigYaml{
public:
    static const std::string config_file;
    static YAML::Node cfg_root;
    static const std::string if_prefix;
    static const std::string int_prefix;
    static const std::string float_prefix;

    ConfigYaml(){
        
    }
    
    static std::string configGetName(std::string file, int line, std::string prefix){
        std::string name;
        int idx0 = file.find_last_of('/');
        name = file.substr(idx0 + 1, file.length() - idx0);
        int idx1 = name.find_last_of(".");
        name[idx1] = '_';
        name += "_" + std::to_string(line);
        name = prefix + name;
        return name;
    }
    template<typename T>
    static T cfgDebugData(std::string str){
        std::ofstream fout(config_file, std::ios::app);
        if(!cfg_root[str]){
            if (std::is_same<T, bool>::value){
                cfg_root[str] = false;
                if(fout.is_open()){
                    fout << str << ": false" << std::endl;
                    fout.close();
                }
            }else if(std::is_same<T, int>::value){
                cfg_root[str] = 0;
                if(fout.is_open()){
                    fout << str << ": 0" << std::endl;
                    fout.close();
                }
            }else{
                cfg_root[str] = 0.0;
                if(fout.is_open()){
                    fout << str << ": 0.0" << std::endl;
                    fout.close();
                }
            }
        }
        fout.close();
        return cfg_root[str].as<T>();
    }

};

#endif
