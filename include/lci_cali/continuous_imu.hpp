#pragma once 

#include "utils/input.hpp"
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
struct IMUGyroscopFactor {
    IMUGyroscopFactor(Eigen::Vector3d data, double normalized_time, double interval_ns):
        normalized_time(normalized_time),gyro_data(data), interval_ns(interval_ns){}
    template <class T>
    bool operator()(T const* const* sKnots, T* r) const{
        using Tangent = typename Sophus::SO3<T>::Tangent;
        Eigen::Map<Eigen::Matrix<T,3,1>> residual(r);
        Eigen::Map<Eigen::Matrix<T,3,1> const> bias_g(sKnots[N]);
        residual = Tangent(T(0),T(0),T(0));
        Tangent rot_vel;

        basalt::CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, normalized_time , double(1e9)/interval_ns, nullptr, &rot_vel, nullptr);
        residual =  gyro_data.template cast<T>() - rot_vel - bias_g;

        return true;
    }
    Eigen::Vector3d gyro_data;
    double normalized_time;
    double interval_ns;
};


template<int N>
struct LidarOrientationFactor {
    LidarOrientationFactor(Sophus::SO3d::Tangent data, double normalized_time, double interval_ns):
        normalized_time(normalized_time),meas_data(data), interval_ns(interval_ns){}
    template <class T>
    bool operator()(T const* const* sKnots, T* r) const{
        using Tangent = typename Sophus::SO3<T>::Tangent;
        Eigen::Map<Eigen::Matrix<T,3,1>> residual(r);
        residual = Tangent(T(0),T(0),T(0));
        Sophus::SO3<T> transform;
        basalt::CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, normalized_time , double(1e9)/interval_ns, &transform);
        residual =  meas_data.template cast<T>() - transform.log();
        return true;
    }
    Sophus::SO3d::Tangent meas_data;
    double normalized_time;
    double interval_ns;
};

template<int N>
struct AccelerationFactor {
    AccelerationFactor(sensor_data::IMUData data, double normalized_time, double interval_ns):
        normalized_time(normalized_time),gyro_data(data), interval_ns(interval_ns){}
    template <class T>
    bool operator()(T const* const* sKnots, T* r) const{
        using Tangent = typename Sophus::SO3<T>::Tangent;
        Eigen::Map<Eigen::Matrix<T,3,1>> res_so3(r);
        Eigen::Map<Eigen::Matrix<T,3,1>> res_rd3(r + 3);
        res_so3 = Tangent(T(0),T(0),T(0));
        res_rd3 = Tangent(T(0),T(0),T(0));
        Tangent g = Tangent(T(0.0), T(0.0), T(-9.81));
        Sophus::SO3<T> rot;
        Tangent rot_vel;
        Tangent accel;
        basalt::CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, normalized_time , double(1e9)/interval_ns, &rot, &rot_vel);
        basalt::CeresSplineHelper<N>::template evaluate<T, 3, 2>(sKnots + N, normalized_time, double(1e9)/ interval_ns, &accel);
        res_rd3 = gyro_data.accel.template cast<T>() - (rot.inverse() * (accel));
        res_so3 =  gyro_data.gyro.template cast<T>() - rot_vel;
        // std::cout << "time: " << gyro_data.timestamp << std::endl;
        // std::cout << "accel: " <<  accel.matrix().x() << " " << accel.matrix().y() << " " <<  accel.matrix().z()  << std::endl;
        // std::cout <<  "residual: " << residual.matrix().x() << " " << residual.matrix().y() << " " <<  residual.matrix().z()  << std::endl;
        return true;
    }
    sensor_data::IMUData gyro_data;
    double normalized_time;
    double interval_ns;
};

template<int N>
struct GyroAccelFactor{
    GyroAccelFactor(sensor_data::IMUData data, double normalized_time, double interval_ns):
         normalized_time(normalized_time),gyro_data(data), interval_ns(interval_ns){}
    template <class T>
    bool operator()(T const* const* sKnots, T* r) const{
        // for(int i = 0; i < 7; i++){
        //     std::cout << sKnots + i << " " << sKnots[i] << " " << sKnots[i][0] << std::endl;
        // }
        // std::cout << std::endl;
        using Tangent = typename Sophus::SO3<T>::Tangent;
        Eigen::Map<Eigen::Matrix<T,3,1>> res_so3(r);
        Eigen::Map<Eigen::Matrix<T,3,1>> res_rd3(r + 3);
        res_so3 = Tangent(T(0),T(0),T(0));
        res_rd3 = Tangent(T(0),T(0),T(0));

        Eigen::Map<Tangent const> bia_g(sKnots[2 * N]);
        Eigen::Map<Tangent const> bia_a(sKnots[2 * N + 1]);
        Eigen::Map<Tangent const> g(sKnots[2 * N + 2]);

        Sophus::SO3<T> rot;
        Tangent rot_vel;
        Tangent accel;
        basalt::CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, normalized_time , double(1e9)/interval_ns, &rot, &rot_vel);
        basalt::CeresSplineHelper<N>::template evaluate<T, 3, 2>(sKnots + N, normalized_time, double(1e9)/ interval_ns, &accel);
        res_rd3 = gyro_data.accel.template cast<T>() - (rot.inverse() * (accel + g)) - bia_a;
        res_so3 =  gyro_data.gyro.template cast<T>() - rot_vel - bia_g;
        return true;
    }
    sensor_data::IMUData gyro_data;
    double normalized_time;
    double interval_ns;
};


template<int N>
struct LidarPoseFactor
{
    LidarPoseFactor(Eigen::Vector3d p, double u, double interval_ns):
        pos(p), normalized_time(u), interval(interval_ns){}

    template<typename T>
    bool operator()(T const* const* sKnots, T* r) const{
        using Tangent = typename Sophus::SO3<T>::Tangent;
        Eigen::Map<Eigen::Matrix<T,3,1>> res_rd3(r);
        Eigen::Matrix<T,3,1> estimate_pos;
        basalt::CeresSplineHelper<N>::template evaluate<T, 3, 0>(sKnots, normalized_time, double(1e9)/ interval, &estimate_pos);
        res_rd3 = pos.template cast<T>() - estimate_pos;
        return true;
    }

    Eigen::Vector3d pos;
    double normalized_time;
    double interval;
};


template <int N>
class BSplineIMU
{
    using PointXYZ = pcl::PointXYZ;
    using PCL_XYZ = pcl::PointCloud<PointXYZ>;

    using V3d = Eigen::Vector3d;

private:
    double start_time, end_time, interval;

    basalt::LieLocalParameterization<Sophus::SO3d> local_parameterization;
    ceres::Problem problem;

    // std::pair<std::vector<double*>, double> GyroMeasFitKnots(uint64_t gyro_ns){
    //     std::vector<double*> ret;
    //     uint64_t st_ns = (gyro_ns - so3_spline->minTimeNs());
    //     uint64_t s = st_ns / so3_spline->getTimeIntervalNs();
    //     double u = double(st_ns % so3_spline->getTimeIntervalNs()) / double(so3_spline->getTimeIntervalNs());
    //     for ( int i = 0; i < N; i++){
    //         ret.emplace_back(const_cast<double*>(so3_spline->getKnots()[s + i].data()));
    //     }
    //     return std::pair<std::vector<double*>, double>(ret, u);
    // }
    std::pair<std::vector<double*>, double> GetMeasFitKnots(uint64_t gyro_ns){
        std::vector<double*> ret;
        uint64_t st_ns = (gyro_ns - rd_spline->minTimeNs());
        uint64_t s = st_ns / rd_spline->getTimeIntervalNs();
        double u = double(st_ns % rd_spline->getTimeIntervalNs()) / double(rd_spline->getTimeIntervalNs());
        for ( int i = 0; i < N; i++){
            ret.insert(ret.end(), &knots[7 * (s + i)], &knots[7 * (s + i) + 1]);
        }
        for ( int i = 0; i < N; i++){
            ret.insert(ret.end(), &knots[7 * (s + i) + 4], &knots[7 * (s + i) + 5]);
        }
        return std::pair<std::vector<double*>, double>(ret, u);
    }
    // std::pair<std::vector<double*>, double> GetMeasFitKnots(uint64_t gyro_ns){
    //     std::vector<double*> ret;
    //     uint64_t st_ns = (gyro_ns - rd_spline->minTimeNs());
    //     uint64_t s = st_ns / rd_spline->getTimeIntervalNs();
    //     double u = double(st_ns % rd_spline->getTimeIntervalNs()) / double(rd_spline->getTimeIntervalNs());
    //     for ( int i = 0; i < N; i++){
    //         // ret.insert(ret.end(), &knots[s + i], &knots[s + i] + 7);
    //         ret.push_back(knots[7 * (s + i)]);
    //         ret.push_back(knots[7 * (s + i) + 4]);
            
    //     }
    //     return std::pair<std::vector<double*>, double>(ret, u);
    // }
public:
    std::vector<double*> knots;
    Eigen::Vector3d bia_g, bia_a, g;
    
    std::shared_ptr<basalt::So3Spline<N>> so3_spline;
    std::shared_ptr<basalt::RdSpline<3, N>> rd_spline;
    ~BSplineIMU(){};
    BSplineIMU(double start_time, double end_time, double interval):
        interval(interval), end_time(end_time), start_time(start_time)
    {
        assert(end_time > 0);
        int num_knots = (end_time - start_time) / interval + N;
        RCLCPP_DEBUG(rclcpp::get_logger("SplineIMU"), "Init begin");
        RCLCPP_DEBUG(rclcpp::get_logger("SplineIMU"), "konts num:%d", num_knots);
        knots.resize( num_knots * 7);
        RCLCPP_DEBUG(rclcpp::get_logger("SplineIMU"), "Init begin");
        so3_spline = std::make_shared<basalt::So3Spline<N,double>>(interval * 1e9, start_time * 1e9);
        rd_spline = std::make_shared<basalt::RdSpline<3, N>>(interval * 1e9, start_time * 1e9);
        for(int i = 0; i <  ((end_time - start_time) / interval) + N; i++){
            so3_spline->knotsPushBack(Sophus::SO3d(Eigen::Matrix3d::Identity()));
            rd_spline->knotsPushBack(Eigen::Matrix<double, 3, 1>(0, 0, 0));
            knots[i*7] = const_cast<double*>(so3_spline->getKnots()[i].data());
            knots[i*7 + 1] = const_cast<double*>(so3_spline->getKnots()[i].data() + 1);
            knots[i*7 + 2] = const_cast<double*>(so3_spline->getKnots()[i].data() + 2);
            knots[i*7 + 3] = const_cast<double*>(so3_spline->getKnots()[i].data() + 3);
            knots[i*7 + 4] = const_cast<double*>(rd_spline->getKnots()[i].data());
            knots[i*7 + 5] = const_cast<double*>(rd_spline->getKnots()[i].data() + 1);
            knots[i*7 + 6] = const_cast<double*>(rd_spline->getKnots()[i].data() + 2);
            for (int j = 0; j < 7; j++){
                std::cout << knots[i * 7 + j] << " ";
            }
            std::cout << std::endl;
            g << 0,0,-10;

        }
        RCLCPP_DEBUG(rclcpp::get_logger("SplineIMU"), "Init successful");
    }


    void AddAccelMeasurement(sensor_data::IMUData data){
        // std::vector<double*> vec_so3, vec_rd;
        // double u;
        // std::tie(vec_so3, vec_rd, u) = GetMeasFitKnots(data.timestamp * 1e9);
        // auto factor = new AccelerationFactor<N>(data, u, rd_spline->getTimeIntervalNs());
        // auto cost_func = new ceres::DynamicAutoDiffCostFunction<lci_cali::AccelerationFactor<N>>(factor);

    }

    void AddImuMeasurement(sensor_data::IMUData data){

        if(data.timestamp < start_time || data.timestamp > end_time) return;
        auto[vec_so3_rd, u] = GetMeasFitKnots(data.timestamp * 1e9);
        // std::cout << vec_so3_rd.size() << std::endl;
        for(int i = 0; i < vec_so3_rd.size();i++){
            // std::cout << vec_so3_rd[i] << " " << vec_so3_rd[i+1] << std::endl;
        
        }
        auto factor = new GyroAccelFactor<N>(data, u, so3_spline->getTimeIntervalNs());
        auto* cost_func = new ceres::DynamicAutoDiffCostFunction<lci_cali::GyroAccelFactor<N>>(factor);
        for(int i = 0; i < N; i++){
            cost_func->AddParameterBlock(4);
        }
        for(int i = 0; i < N; i++){
            cost_func->AddParameterBlock(3);
        }
        cost_func->AddParameterBlock(3);
        cost_func->AddParameterBlock(3);
        cost_func->AddParameterBlock(3);
        vec_so3_rd.push_back(bia_g.data());
        vec_so3_rd.push_back(bia_a.data());
        vec_so3_rd.push_back(g.data());
        cost_func->SetNumResiduals(6);
        problem.AddResidualBlock(cost_func, NULL, vec_so3_rd);
        if(problem.NumParameterBlocks() == 1){
            problem.SetParameterBlockConstant(vec_so3_rd[0]);
            problem.SetParameterBlockConstant(vec_so3_rd[N]);
        }
        
        for(int i = 0; i < N; i++){
            problem.SetParameterization(vec_so3_rd[i], &local_parameterization);
            // problem.SetParameterization(vec_so3_rd[i + N], &);
        } 
    }

    // PCL_XYZ global_map;
    // void AddLidarSurfelMeasurement(sensor_data::LidarData& lidar_data)
    // {
    //     typename PCL_XYZ::Ptr pc_now = pcl::make_shared(PCL_XYZ);
    //     pcl::fromROSMsg(*lidar_data.rawdata, *pc_now);
        
        
    // }

    void addLidarPositionMeasurement(double time_stamp, V3d translate)
    {
        assert(time_stamp >= start_time || time_stamp <= end_time);
        auto[vec_so3_rd, u] = GetMeasFitKnots(time_stamp * 1e9);
        std::vector<double*> vec_rd;
        vec_rd.insert(vec_rd.end(), vec_so3_rd.begin() + N, vec_so3_rd.end());
        
        auto factor = new LidarPoseFactor<N>(translate, u, so3_spline->getTimeIntervalNs());
        auto* cost_func = new ceres::DynamicAutoDiffCostFunction<lci_cali::LidarPoseFactor<N>>(factor);

        for(int i = 0; i < N; i++){
            cost_func->AddParameterBlock(3);
        }
        cost_func->SetNumResiduals(3);
        problem.AddResidualBlock(cost_func, NULL, vec_rd);
        if(problem.NumParameterBlocks() == 1){
            problem.SetParameterBlockConstant(vec_rd[0]);
        }

    }

    void addIMUGyroMeasurement(double time_stamp, V3d gyro){
        assert(time_stamp >= start_time || time_stamp <= end_time);
        auto[vec_so3_rd, u] = GetMeasFitKnots(time_stamp * 1e9);
        vec_so3_rd.erase(vec_so3_rd.begin() + N, vec_so3_rd.end());
        auto factor = new IMUGyroscopFactor<N>(gyro, u, so3_spline->getTimeIntervalNs());
        auto* cost_function = new ceres::DynamicAutoDiffCostFunction<lci_cali::IMUGyroscopFactor<N>>(factor);
        for(int i = 0; i < N; i++){
            cost_function->AddParameterBlock(4);
        }
        cost_function->AddParameterBlock(3);
        vec_so3_rd.push_back(bia_g.data());
        cost_function->SetNumResiduals(3);
        problem.AddResidualBlock(cost_function, NULL, vec_so3_rd);
        for(int i = 0; i < N; i++){
            problem.SetParameterization(vec_so3_rd[i], &local_parameterization);
        }  
    }

    void addLidarOrientationMeasurement(double time_stamp, Sophus::SO3d::Tangent orient){
        assert(time_stamp >= start_time || time_stamp <= end_time);
        auto[vec_so3_rd, u] = GetMeasFitKnots(time_stamp * 1e9);
        vec_so3_rd.erase(vec_so3_rd.begin() + N, vec_so3_rd.end());
        auto factor = new LidarOrientationFactor<N>(orient, u, so3_spline->getTimeIntervalNs());
        auto* cost_function = new ceres::DynamicAutoDiffCostFunction<lci_cali::LidarOrientationFactor<N>>(factor);
        for(int i = 0; i < N; i++){
            cost_function->AddParameterBlock(4);
        }
        cost_function->SetNumResiduals(3);
        problem.AddResidualBlock(cost_function, NULL, vec_so3_rd);
        for(int i = 0; i < N; i++){
            problem.SetParameterization(vec_so3_rd[i], &local_parameterization);
        }  
    }


    void initExtrinsicParam(){

    }

    ceres::Solver::Summary solve(ceres::Solver::Options& opt){
        ceres::Solver::Summary summary;
        ceres::Solve(opt, &problem, &summary);
        return summary;
    }

};

}



