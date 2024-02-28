#pragma once
#include "cloud_type.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace sensor_data{

template<typename PointType = pcl::PointNormal>
class LidarData
{
public:
  using PointT = PointType;
  using PointCloudT = pcl::PointCloud<PointT>;
  LidarData()
      : timestamp(0),
        time_max(0),
        data(new PointCloudT),
        raw_data(new sensor_msgs::msg::PointCloud2) {}

  LidarData(const LidarData<PointT>& fea) { *this = fea; }

  void set(LidarData<PointT>* lf1, LidarData<PointT>* lf2) {
    lf1->timestamp = lf2->timestamp;
    lf1->time_max = lf2->time_max;
    *lf1->data = *lf2->data;
    *lf1->raw_data = *lf2->raw_data;
  }

  void clear() {
    timestamp = 0;
    time_max = 0;
    data->clear();
  }
  double timestamp;
  double time_max;  // [timestamp, max_time] data
  typename PointCloudT::Ptr data;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> raw_data;
};

}