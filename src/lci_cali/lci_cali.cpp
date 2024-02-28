#include <rclcpp/rclcpp.hpp>
#include "lci_cali/lci_cali.hpp"

namespace lci_cali{


LCICali::LCICali(/* args */):rclcpp::Node("Lci_Node")
{
    Initialization();
}

LCICali::~LCICali()
{
}

void LCICali::Initialization(){
    input.reset(slam_utils::Input<sensor_data::LidarData<PointIRT>>::createInstance());
}


}

