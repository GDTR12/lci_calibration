#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rosbag2_cpp/converter.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_data/imu_data.hpp"
#include "sensor_data/lidar_data.h"
// #include "velodyne_msgs/msg/velodyne_scan.hpp"

namespace slam_utils
{

using namespace sensor_data;

// class InputBase: public rclcpp::Node{
// public:
//     virtual void init() = 0;
//     virtual void lidar_data_preprocess() = 0;
//     virtual void lidar_data_preprocess() = 0;
// };


template<typename LidarDataT, typename PCLBefore = typename LidarDataT::PointCloudT>
class Input: public rclcpp::Node
{
public:
    using PointT = typename LidarDataT::PointT;
    using PointCloudT = typename LidarDataT::PointCloudT;
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

    std::shared_ptr<std::vector<IMUData, Eigen::aligned_allocator<IMUData>>> imu_source;
    std::shared_ptr<std::vector<LidarData<PointT>>> lidar_source;

    double read_end = INFINITY;
    double read_start = 0.0;
public:

    explicit Input(std::string name = "Input"): rclcpp::Node(name){}

    static Input* createInstance()
    {
        Input* instance = new Input();
        instance->init();
        return instance;
    }
    static Input* createInstance(double read_start, double end)
    {
        assert(end > read_start);
        Input* instance = new Input();
        instance->read_end = end;
        instance->read_start = read_start;
        instance->init();
        return instance;
    }
    
    void init()
    {
        std::string bag_file;
        this->declare_parameter("lidar_topic_name", rclcpp::PARAMETER_STRING);
        this->get_parameter("lidar_topic_name", topic_pcl);
        this->declare_parameter("camera_topic_name", rclcpp::PARAMETER_STRING);
        this->get_parameter("camera_topic_name", topic_image);
        this->declare_parameter("imu_topic_name", rclcpp::PARAMETER_STRING);
        this->get_parameter("imu_topic_name", topic_imu);
        this->declare_parameter("bag_file", rclcpp::PARAMETER_STRING);
        this->get_parameter("bag_file", bag_file);
        this->declare_parameter("input_start_time", rclcpp::PARAMETER_DOUBLE);
        this->get_parameter("input_start_time", read_start);
        this->declare_parameter("input_end_time", rclcpp::PARAMETER_DOUBLE);
        this->get_parameter("input_end_time", read_end);
        RCLCPP_DEBUG(this->get_logger(), "======================================");
        RCLCPP_DEBUG(this->get_logger(), "The config parameters of input node:");
        RCLCPP_DEBUG(this->get_logger(), "bag_file: %s", bag_file.c_str());
        RCLCPP_DEBUG(this->get_logger(), "imu_topic: %s", topic_imu.c_str());
        RCLCPP_DEBUG(this->get_logger(), "input_end_time: %f", read_end);
        RCLCPP_DEBUG(this->get_logger(), "======================================");
        reader_.open(bag_file);

        lidar_source = std::make_shared<std::vector<LidarData<PointT>>>();
        imu_source = std::make_shared<std::vector<IMUData, Eigen::aligned_allocator<IMUData>>>();

        RCLCPP_DEBUG(this->get_logger(), "Open bag file successfully");
        auto all_topics = reader_.get_all_topics_and_types();
        
        RCLCPP_DEBUG(this->get_logger(), "============================================");
        RCLCPP_DEBUG(this->get_logger(), "The topic in the bag:");
        for(auto it = all_topics.begin(); it != all_topics.end(); it++)
        {
            RCLCPP_DEBUG(this->get_logger(), "topic: %s, type: %s", (*it).name.c_str(), (*it).type.c_str());
        }
        RCLCPP_DEBUG(this->get_logger(), "============================================");
        bool first_frame = true;
        double start_time;
        while (reader_.has_next())
        {
            rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_.read_next();
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

            if(msg->topic_name == topic_imu){
                sensor_msgs::msg::Imu::SharedPtr one_imu = std::make_shared<sensor_msgs::msg::Imu>();
                slz_imu.deserialize_message(&serialized_msg, one_imu.get());
                if(first_frame == true){
                    first_frame = false;
                    start_time =  rclcpp::Time(one_imu->header.stamp).seconds();
                }else{
                    if(start_time >= rclcpp::Time(one_imu->header.stamp).seconds()){
                        break;
                    }
                }
                
                IMUData imu_data = {
                    .timestamp = (rclcpp::Time(one_imu->header.stamp).seconds()- start_time),
                    .gyro = Eigen::Vector3d(one_imu->angular_velocity.x, one_imu->angular_velocity.y, one_imu->angular_velocity.z),
                    .accel = Eigen::Vector3d(one_imu->linear_acceleration.x, one_imu->linear_acceleration.y, one_imu->linear_acceleration.z),
                    .orientation = Eigen::Quaterniond(one_imu->orientation.w, one_imu->orientation.x,
                                one_imu->orientation.y, one_imu->orientation.z)
                };
                if(imu_data.timestamp < read_start || imu_data.timestamp > read_end)continue;
                imu_source->push_back(imu_data);


            }
            if(msg->topic_name == topic_pcl){
                
                sensor_msgs::msg::PointCloud2::SharedPtr one_pcl = std::make_shared<sensor_msgs::msg::PointCloud2>();
                // velodyne_msgs::msg::VelodyneScan::SharedPtr one_pcl = std::make_shared<velodyne_msgs::msg::VelodyneScan>();
                slz_pcl.deserialize_message(&serialized_msg, one_pcl.get());
                
                LidarData<PointT> lidar_data;
                PCLBefore before;

                lidar_data.data = pcl::make_shared<pcl::PointCloud<PointT>>();
                lidar_data.timestamp = (rclcpp::Time(one_pcl->header.stamp).seconds() - start_time);
                lidar_data.raw_data = one_pcl;
                lidar_data.time_max = 0;
                // velodyne_pcl::
                pcl::fromROSMsg(*one_pcl, before);
                this->lidar_data_preprocess(before, lidar_data);
                if(lidar_data.timestamp < read_start || lidar_data.timestamp > read_end)continue;
                lidar_source->push_back(lidar_data);

            }
            if(msg->topic_name == topic_image){
                sensor_msgs::msg::Image::SharedPtr one_img = std::make_shared<sensor_msgs::msg::Image>();
                slz_img.deserialize_message(&serialized_msg, one_img.get());
            }
        }
        read_end = std::max(lidar_source->back().timestamp, imu_source->back().timestamp);
        reader_.close();
    }
    
    virtual void lidar_data_preprocess(PCLBefore& before, LidarData<PointT>& lidar_data){

        pcl::copyPointCloud(before, *lidar_data.data);
    }
    auto getIMUData(){return imu_source;}
    auto getLidarData(){return lidar_source;}
    double getEndTime(){return read_end;}
    double getStartTime(){return read_start;}

    ~Input(){}

};
   
} // namespace lci_cali
