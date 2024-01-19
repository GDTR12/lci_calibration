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
    input = std::make_shared<Input>("Input");
    trajetory = std::make_shared<BSplineIMU>(0, input->GetEndtime(), 0.01);
    trajetory->SetIMUData(input->GetIMUData());
    trajetory->InitWithDefaultMeasurement();
}


}

