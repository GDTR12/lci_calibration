
#define PCL_NO_PRECOMPILE
#include "sensor_data/cloud_type.h"
#include "rclcpp/rclcpp.hpp"
#include "pcl/point_cloud.h"
#include "utils/input.hpp"
#include "lci_cali/continuous_se3.hpp"
#include "lci_cali/lci_cali.hpp"
#include "utils/config_yaml.h"
#include "utils/ndt_utils.hpp"
#include "pcl/visualization/cloud_viewer.h"
#include "utils/ui_utils.hpp"

using namespace lci_cali;
int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    LCICali lci_cali;
    lci_cali.run();
    // rclcpp::spin(); 
    rclcpp::shutdown();
}
