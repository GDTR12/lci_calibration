#pragma once
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>


namespace sensor_data{
using SO3d = Sophus::SO3<double>;
using SE3d = Sophus::SE3<double>;

struct IMUData {
  double timestamp;
  Eigen::Matrix<double, 3, 1> gyro;
  Eigen::Matrix<double, 3, 1> accel;
  Eigen::Quaterniond orientation;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
struct PoseData {
  PoseData()
      : timestamp(0),
        position(Eigen::Vector3d(0, 0, 0)),
        orientation(SO3d(Eigen::Quaterniond::Identity())) {}

  double timestamp;
  Eigen::Vector3d position;
  SO3d orientation;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct OdomData {
  double timestamp;
  Eigen::Matrix4d pose;
};


}

