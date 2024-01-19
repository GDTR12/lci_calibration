#include "lci_cali/continuous_imu.hpp"

namespace lci_cali{



BSplineIMU::BSplineIMU(double start_time, double end_time, double knot_distance):
  knot_distance(knot_distance), end_time(end_time), start_time(start_time), imu(std::make_shared<IMUSensor>())
{
  RCLCPP_DEBUG(rclcpp::get_logger("SplineIMU"), "Init begin");
  traj_ = std::make_shared<kontiki::trajectories::SplitTrajectory>
            (knot_distance, knot_distance, start_time, start_time);   
  Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p0(0,0,0);
  traj_->R3Spline()->ExtendTo (end_time, p0);
  traj_->SO3Spline()->ExtendTo(end_time, q0);
  RCLCPP_DEBUG(rclcpp::get_logger("SplineIMU"), "Init successful");
}

void BSplineIMU::InitWithDefaultMeasurement()
{
  RCLCPP_DEBUG(rclcpp::get_logger("SplineIMU"), "measurement begin");
  std::shared_ptr<SO3TrajEstimator> estimator_SO3;
  estimator_SO3 = std::make_shared<SO3TrajEstimator>(traj_->SO3Spline());
  const double min_time = estimator_SO3->trajectory()->MinTime();
  const double max_time = estimator_SO3->trajectory()->MaxTime();

  RCLCPP_INFO(rclcpp::get_logger("SplineIMU"), "min_time %.2f, max_time: %.2f", min_time, max_time);
  // 添加IMU数据到B样条的IMU测量
  for (const auto &v : *imu_data) {
    // std::cout << v.timestamp << " ";
    // RCLCPP_DEBUG(rclcpp::get_logger("SplineIMU"), "time:%f, gyro:%.2f", v.timestamp, v.gyro.x());
    if ( min_time > v.timestamp || max_time <= v.timestamp) {
      continue;
    }
    auto mg = std::make_shared<GyroMeasurement>(imu, v.timestamp, v.gyro, weight);
    gyro_list_.push_back(mg);
    estimator_SO3->template AddMeasurement<GyroMeasurement>(mg);
  }
  // std::cout << std::endl;
  RCLCPP_DEBUG(rclcpp::get_logger("SplineIMU"), "add imu data successful");
  //  
  double t0 = traj_->SO3Spline()->MinTime();
  Eigen::Quaterniond q0 = Eigen::Quaterniond (Eigen::AngleAxisd(0.0001, Eigen::Vector3d(0, 0, 1).matrix()));
  auto m_q0 = std::make_shared<OrientationMeasurement>(t0, q0, gyro_weight);
  estimator_SO3->AddMeasurement<OrientationMeasurement>(m_q0);

  ceres::Solver::Summary summary = estimator_SO3->Solve(30, false);
  RCLCPP_INFO(rclcpp::get_logger("SplineIMU"),"SplineIMU solver summury: %s", summary.BriefReport().c_str());

  RCLCPP_DEBUG(rclcpp::get_logger("SplineIMU"), "Measurement successful");
}

void BSplineIMU::InitWithBasalt()
{
  
}

}
