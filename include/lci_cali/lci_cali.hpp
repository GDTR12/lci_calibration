#include <rclcpp/rclcpp.hpp>
#include <lci_cali/input.hpp>
#include <lci_cali/continuous_imu.hpp>

namespace lci_cali{

class LCICali: public rclcpp::Node
{
private:
    std::shared_ptr<Input> input;
    std::shared_ptr<BSplineIMU> trajetory;
public:
    LCICali();
    ~LCICali();
    void Initialization();
};

}