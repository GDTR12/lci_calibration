#pragma once 

#include "lci_cali/input.hpp"
// #include "basalt/spline/se3_spline.h"
#include "ceres/ceres.h"
#include "sensor_data/imu_data.hpp"
#include "kontiki/sensors/constant_bias_imu.h"
// #include <kontiki/sensors/vlp16_lidar.h>
#include <kontiki/trajectory_estimator.h>
#include <kontiki/trajectories/split_trajectory.h>
#include <kontiki/trajectories/uniform_r3_spline_trajectory.h>
#include <kontiki/trajectories/uniform_so3_spline_trajectory.h>
#include <kontiki/measurements/gyroscope_measurement.h>
#include <kontiki/measurements/accelerometer_measurement.h>
// #include <kontiki/measurements/lidar_surfel_point.h>
#include <kontiki/measurements/orientation_measurement.h>
#include <kontiki/measurements/position_measurement.h>

namespace lci_cali{

class IMUBsplineCostFunction{
public:



private:
    std::shared_ptr<std::vector<IMUData, Eigen::aligned_allocator<IMUData>>> imu;
};


class BSplineIMU
{
    using IMUSensor = kontiki::sensors::ConstantBiasImu;
    using GyroMeasurement    = kontiki::measurements::GyroscopeMeasurement<IMUSensor>;
    using AccelMeasurement   = kontiki::measurements::AccelerometerMeasurement<IMUSensor>;

    using OrientationMeasurement  = kontiki::measurements::OrientationMeasurement;
    using PositionMeasurement     = kontiki::measurements::PositionMeasurement;
    using SO3TrajEstimator   = kontiki::TrajectoryEstimator<kontiki::trajectories::UniformSO3SplineTrajectory>;
    using R3TrajEstimator    = kontiki::TrajectoryEstimator<kontiki::trajectories::UniformR3SplineTrajectory>;
    using SplitTrajEstimator = kontiki::TrajectoryEstimator<kontiki::trajectories::SplitTrajectory>;
private:

    std::vector<Sophus::SO3<double>> knots;
    std::shared_ptr<kontiki::trajectories::SplitTrajectory> traj_;
    std::shared_ptr<kontiki::sensors::ConstantBiasImu> imu;
    // std::shared_ptr<kontiki::sensors::VLP16LiDAR> lidar_;
    std::vector<IMUData, Eigen::aligned_allocator<IMUData>>* imu_data;
    std::vector< std::shared_ptr<GyroMeasurement>>  gyro_list_;
    double last_knot_time = 0;
    double start_time, end_time, knot_distance;
    double weight, gyro_weight;
    
    
public:


    BSplineIMU(double start_time, double end_time, double knot_distance);
    ~BSplineIMU(){};
    void SetIMUData(std::vector<IMUData, Eigen::aligned_allocator<IMUData>>& data){imu_data = &data;}
    void InitWithDefaultMeasurement();
    void InitWithBasalt();
};


}



