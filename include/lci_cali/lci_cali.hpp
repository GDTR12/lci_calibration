#include <rclcpp/rclcpp.hpp>
#include <utils/input.hpp>
#include <lci_cali/continuous_imu.hpp>
#include "sensor_data/lidar_data.h"
namespace lci_cali{

class LCICali: public rclcpp::Node
{
private:
    std::shared_ptr<slam_utils::Input<sensor_data::LidarData<PointIRT>>> input;
public:
    LCICali();
    ~LCICali();
    void Initialization();
};

}