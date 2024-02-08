#include "cloud_type.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace sensor_data{

template<typename PointType = pcl::PointNormal>
struct LidarData {
  using PCL = pcl::PointCloud<PointType>;
  LidarData()
      : timestamp(0),
        time_max(0),
        data(new PCL),
        raw_data(new sensor_msgs::msg::PointCloud2) {}

  LidarData(const LidarData<PointType>& fea) { *this = fea; }

  void set(LidarData<PointType>* lf1, LidarData<PointType>* lf2) {
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
  typename PCL::Ptr data;
  typename sensor_msgs::msg::PointCloud2::Ptr raw_data;
};


}