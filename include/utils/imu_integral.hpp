#pragma once

#include "sophus/so3.hpp"
#include "Eigen/Core"
#include "sensor_data/imu_data.hpp"
#include "bits/shared_ptr.h"
#include "unordered_map"
namespace slam_utils
{

class ImuIntegration
{
    using SO3d = Sophus::SO3d;
    using SE3d = Sophus::SE3d;
    using V3d = Eigen::Vector3d;
private:
    SO3d start_o;
    V3d start_p;
    V3d g = V3d(0, 0, 10.20);
    std::shared_ptr<std::vector<sensor_data::IMUData, Eigen::aligned_allocator<sensor_data::IMUData>>> data;
    double time_start, time_end;
    std::vector<double> tra_time;
    std::unordered_map<double, SE3d> trajectory;

    void addSE3(double time, SE3d se3){
        assert(std::find(tra_time.begin(), tra_time.end(), time) == tra_time.end());
        tra_time.push_back(time);
        trajectory[time] = se3;
    }
    void integrationWithoutBias(){
        V3d vel(0,0,0);
        for(int i = 1; i < data->size(); i++){
            auto& d = data->at(i);
        // }
        // for(auto& d : *data){
            
            assert(d.timestamp >= tra_time.back());
            double last_t = tra_time.back();
            double dt = d.timestamp - last_t;
            SO3d orientation = trajectory[tra_time.back()].so3() * Sophus::SO3d::exp(dt * d.gyro);
            V3d pos = trajectory[tra_time.back()].translation() + vel * dt + 0.5 * (trajectory[tra_time.back()].so3().matrix() * (d.accel)) * dt * dt + 0.5 * g * dt * dt;
            vel = vel + trajectory[tra_time.back()].so3().matrix() * (d.accel) * dt + g * dt;
            std::cout << "mult ori" << Sophus::SO3d::exp(dt * d.gyro).matrix() << std::endl; 
            std::cout << "orientation:" << orientation.matrix()<< std::endl;
            std::cout << "velo" << vel << std::endl;
            addSE3(d.timestamp, SE3d(orientation, pos));
        }
    }   
public:
    ImuIntegration(SO3d start_so3, V3d start_pos, std::vector<sensor_data::IMUData,Eigen::aligned_allocator<sensor_data::IMUData>>* imu){
        start_o = start_so3;
        start_p = start_pos;
        data.reset(imu);
        addSE3(imu->front().timestamp ,SE3d(start_o, start_p));
        integrationWithoutBias();
    }
    const SE3d& getSE3(double time){
        return trajectory[time];
    }
    void setG(V3d gravity){g = gravity;}
 
    ~ImuIntegration(){}
};




} // namespace slam_utils


