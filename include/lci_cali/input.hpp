#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rosbag2_cpp/converter.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_data/imu_data.hpp"
#include "sensor_data/lidar_data.h"

namespace lci_cali
{
 
class Input: public rclcpp::Node
{
private:
    // sensor_msgs::msg::PointCloud2::SharedPtr 
    std::string topic_imu, topic_image, topic_pcl;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pub_image;

    rosbag2_cpp::Reader reader_;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> slz_pcl;
    rclcpp::Serialization<sensor_msgs::msg::Imu> slz_imu;
    rclcpp::Serialization<sensor_msgs::msg::Image> slz_img;

    std::vector<IMUData, Eigen::aligned_allocator<IMUData>> imu_source;
    std::vector<sensor_data::LidarData<PointIRT>> lidar_source;

    double time_for_cali = 0;
public:
    double getEndTime(){
        return time_for_cali;
    }
    explicit Input(std::string name);
    std::vector<IMUData, Eigen::aligned_allocator<IMUData>>& getIMUData(){return imu_source;}
    std::vector<sensor_data::LidarData<PointIRT>>& getLidarData(){return lidar_source;}
    ~Input();

};
   
} // namespace lci_cali
