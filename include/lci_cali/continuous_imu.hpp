#pragma once 

#include "lci_cali/input.hpp"
// #include "basalt/spline/se3_spline.h"
#include "ceres/ceres.h"
#include "sensor_data/imu_data.hpp"

#include "basalt/spline/so3_spline.h"
#include "basalt/spline/rd_spline.h"
#include "basalt/spline/ceres_spline_helper.h"
#include "basalt/spline/ceres_local_param.hpp"

namespace lci_cali{

// class IterationCallback : public ceres::IterationCallback {
// public:
//   explicit IterationCallback(std::vector<>& residuals)
//       : residuals_(residuals) {}

//   ceres::CallbackReturnType operator()(
//       const ceres::IterationSummary& summary) override {
//     residuals_.push_back(summary.cost);
//     return ceres::SOLVER_CONTINUE;
//   }

// private:
//   std::vector<double>& residuals_;
// };

template<int N>
struct GyroscopFactor {
    GyroscopFactor(lci_cali::IMUData data, double normalized_time, double interval_ns):
        normalized_time(normalized_time),gyro_data(data), interval_ns(interval_ns){}
    template <class T>
    bool operator()(T const* const* sKnots, T* r) const{
        using Tangent = typename Sophus::SO3<T>::Tangent;
        Eigen::Map<Eigen::Matrix<T,3,1>> residual(r);
        residual = Tangent(T(0),T(0),T(0));
        Tangent rot_vel;

        basalt::CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, normalized_time , double(1e9)/interval_ns, nullptr, &rot_vel, nullptr);
        residual =  gyro_data.gyro.template cast<T>() - rot_vel;
        residual = residual.cwiseAbs();

        // std::cout << "time: " << gyro_data.timestamp << std::endl;
        // std::cout <<  "residual: " << std::endl;
        // std::cout <<  residual.matrix() << std::endl;
        return true;
    }
    lci_cali::IMUData gyro_data;
    double normalized_time;
    double interval_ns;
};

template<int N>
struct AccelerationFactor {
    AccelerationFactor(lci_cali::IMUData data, double normalized_time, double interval_ns):
        normalized_time(normalized_time),gyro_data(data), interval_ns(interval_ns){}
    template <class T>
    bool operator()(T const* const* sKnots, T* r) const{
        using Tangent = typename Sophus::SO3<T>::Tangent;
        Eigen::Map<Eigen::Matrix<T,3,1>> residual(r);
        residual = Tangent(T(0),T(0),T(0));
        Tangent g = Tangent(T(0.0), T(0.0), T(-9.81));
        Sophus::SO3<T> rot;
        Tangent accel;
        basalt::CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, normalized_time , double(1e9)/interval_ns, &rot);
        basalt::CeresSplineHelper<N>::template evaluate<T, 3, 2>(sKnots + 4, normalized_time, double(1e9)/ interval_ns, &accel);
        residual = gyro_data.accel.template cast<T>() - (rot.inverse() * (accel));
        residual = residual.cwiseAbs();
        // std::cout << "time: " << gyro_data.timestamp << std::endl;
        // std::cout << "accel: " <<  accel.matrix().x() << " " << accel.matrix().y() << " " <<  accel.matrix().z()  << std::endl;
        // std::cout <<  "residual: " << residual.matrix().x() << " " << residual.matrix().y() << " " <<  residual.matrix().z()  << std::endl;
        return true;
    }
    lci_cali::IMUData gyro_data;
    double normalized_time;
    double interval_ns;
};

template <int N>
class BSplineIMU
{
private:
    double start_time, end_time, interval;

    basalt::LieLocalParameterization<Sophus::SO3d> local_parameterization;
    ceres::Problem problem;

    std::pair<std::vector<double*>, double> GyroMeasFitKnots(uint64_t gyro_ns){
        std::vector<double*> ret;
        uint64_t st_ns = (gyro_ns - so3_spline->minTimeNs());
        uint64_t s = st_ns / so3_spline->getTimeIntervalNs();
        double u = double(st_ns % so3_spline->getTimeIntervalNs()) / double(so3_spline->getTimeIntervalNs());
        for ( int i = 0; i < N; i++){
            ret.emplace_back(const_cast<double*>(so3_spline->getKnots()[s + i].data()));
        }
        return std::pair<std::vector<double*>, double>(ret, u);
    }
    std::tuple<std::vector<double*>, std::vector<double*>, double> GetMeasFitKnots(uint64_t gyro_ns){
        std::vector<double*> ret_so3, ret_rd;
        uint64_t st_ns = (gyro_ns - rd_spline->minTimeNs());
        uint64_t s = st_ns / rd_spline->getTimeIntervalNs();
        double u = double(st_ns % rd_spline->getTimeIntervalNs()) / double(rd_spline->getTimeIntervalNs());
        for ( int i = 0; i < N; i++){
            ret_so3.emplace_back(const_cast<double*>(so3_spline->getKnots()[s + i].data()));
            ret_rd.emplace_back(const_cast<double*>(rd_spline->getKnots()[s + i].data()));
        }
        return std::tuple<std::vector<double*>, std::vector<double*>, double>(ret_so3, ret_rd, u);
    }
public:
    
    std::shared_ptr<basalt::So3Spline<N>> so3_spline;
    std::shared_ptr<basalt::RdSpline<3, N>> rd_spline;
    ~BSplineIMU(){
    };
    BSplineIMU(double start_time, double end_time, double interval):
        interval(interval), end_time(end_time), start_time(start_time)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("SplineIMU"), "Init begin");
        so3_spline = std::make_shared<basalt::So3Spline<N,double>>(interval * 1e9, start_time * 1e9);
        rd_spline = std::make_shared<basalt::RdSpline<3, N>>(interval * 1e9, start_time * 1e9);
        for(int i = 0; i <  (end_time - start_time) / interval + N; i++){
            so3_spline->knotsPushBack(Sophus::SO3d(Eigen::Matrix3d::Identity()));
            rd_spline->knotsPushBack(Eigen::Matrix<double, 3, 1>(0, 0, 0));
        }
        RCLCPP_DEBUG(rclcpp::get_logger("SplineIMU"), "Init successful");
    }

    

    void AddGyroMeasurement(IMUData data){
        auto [vec, u] = GyroMeasFitKnots(data.timestamp * 1e9);
        auto factor = new GyroscopFactor<N>(data, u, so3_spline->getTimeIntervalNs());
        auto* cost_function = new ceres::DynamicAutoDiffCostFunction<lci_cali::GyroscopFactor<N>>(factor);
        for(int i = 0; i < N; i++){
            cost_function->AddParameterBlock(4);
        }
        cost_function->SetNumResiduals(3);
        problem.AddResidualBlock(cost_function, NULL, vec);
        for(int i = 0; i < N; i++){
            problem.SetParameterization(vec[i], &local_parameterization);
        } 
    }

    void AddImuMeasurement(IMUData data){

        std::vector<double*> vec_so3, vec_rd;
        double u;
        std::tie(vec_so3, vec_rd, u) = GetMeasFitKnots(data.timestamp * 1e9);
        std::vector<double*> vec_so3_rd = vec_so3;
        vec_so3_rd.insert(vec_so3_rd.end(), vec_rd.begin(), vec_rd.end());
        auto factor_gyro = new GyroscopFactor<N>(data, u, so3_spline->getTimeIntervalNs());
        auto factor_acc = new AccelerationFactor<N>(data, u, so3_spline->getTimeIntervalNs());
        auto* cost_func_gyro = new ceres::DynamicAutoDiffCostFunction<lci_cali::GyroscopFactor<N>>(factor_gyro);
        auto* cost_func_acc = new ceres::DynamicAutoDiffCostFunction<lci_cali::AccelerationFactor<N>>(factor_acc);   
        for(int i = 0; i < N; i++){
            cost_func_gyro->AddParameterBlock(4);
            cost_func_acc->AddParameterBlock(4);
        }
        for(int i = 0; i < N; i++){
            cost_func_acc->AddParameterBlock(3);
        }
        cost_func_gyro->SetNumResiduals(3);
        cost_func_acc->SetNumResiduals(3);
        problem.AddResidualBlock(cost_func_acc, NULL, vec_so3_rd);
        problem.AddResidualBlock(cost_func_gyro, NULL, vec_so3);
        // std::cout << problem.NumParameterBlocks() << std::endl;
        // std::cout << problem.NumResidualBlocks() << std::endl;
        if(problem.NumParameterBlocks() == 2){
            problem.SetParameterBlockConstant(vec_so3_rd.front());
        }
        for(int i = 0; i < N; i++){
            problem.SetParameterization(vec_so3[i], &local_parameterization);
        } 
    }

    ceres::Solver::Summary solve(ceres::Solver::Options& opt){
        ceres::Solver::Summary summary;
        ceres::Solve(opt, &problem, &summary);
        return summary;
    }

};

}



