#pragma once 
#include "Eigen/Core"
#include "lci_cali/continuous_se3.hpp"

template<int N>
struct ImuSplineImuGyroMeasFactor {
    ImuSplineImuGyroMeasFactor(Eigen::Vector3d data, double normalized_time, double interval_ns, double weight):
        normalized_time(normalized_time),gyro_data(data), interval_ns(interval_ns), weight(weight){}
    template <class T>
    bool operator()(T const* const* sKnots, T* r) const{
        using Tangent = typename Sophus::SO3<T>::Tangent;
        Eigen::Map<Eigen::Matrix<T,3,1>> residual(r);
        Eigen::Map<Eigen::Matrix<T,3,1> const> bias_g(sKnots[N]);
        residual = Tangent(T(0),T(0),T(0));
        Tangent rot_vel;
        basalt::CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, normalized_time , double(1e9)/interval_ns, nullptr, &rot_vel, nullptr);
        residual =  gyro_data.template cast<T>() - rot_vel - bias_g;
        residual = T(weight) * residual;

        return true;
    }
    Eigen::Vector3d gyro_data;
    double normalized_time;
    double interval_ns, weight;
};




template<int N>
struct ImuSplineImuAccelMeasFactor {
    ImuSplineImuAccelMeasFactor(Eigen::Vector3d data, double normalized_time, double interval_ns, double weight):
        normalized_time(normalized_time),accel_data(data), interval_ns(interval_ns), weight(weight){}
    template <class T>
    bool operator()(T const* const* sKnots, T* r) const{
        using Tangent = typename Sophus::SO3<T>::Tangent;
        Eigen::Map<Eigen::Matrix<T,3,1>> residual(r);
        residual = Tangent(T(0),T(0),T(0));

        Eigen::Map<Eigen::Matrix<T, 3, 1> const> bias(sKnots[2 * N]);
        Eigen::Map<Eigen::Matrix<T, 2, 1> const> g(sKnots[2 * N + 1]);

        Tangent accel;
        Sophus::SO3<T> rot;
        basalt::CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, normalized_time , double(1e9)/interval_ns, &rot, nullptr, nullptr);
        basalt::CeresSplineHelper<N>::template evaluate<T, 3, 2>(sKnots + N, normalized_time, double(1e9)/ interval_ns, &accel);

        auto refined_g = refined_gravity(g);

        residual = rot.inverse() * (accel + refined_g) - bias;
        residual = T(weight) * residual;
        
        return true;
    }
    Eigen::Vector3d accel_data;
    double normalized_time;
    double interval_ns, weight;
};

template<int N>
struct ImuSplineLidarPlanarFactor {
    ImuSplineLidarPlanarFactor(Eigen::Vector3d data, Eigen::Vector4d surfel, double normalized_time, double interval_ns, double weight):
        normalized_time(normalized_time),pose(data),plannar(surfel),interval_ns(interval_ns), weight_(weight){}
    template <class T>
    bool operator()(T const* const* sKnots, T* r) const{
        using Tangent = typename Sophus::SO3<T>::Tangent;
        // Eigen::Map<Eigen::Matrix<T,1,1>> residual(r);
        // residual = Tangent(T(0));
        Eigen::Map<Eigen::Quaternion<T> const> q_ItoL(sKnots[2 * N]);
        Eigen::Map<Eigen::Matrix<T, 2, 1> const> t_ItoL(sKnots[2 * N + 1]);
        Tangent t_I0toIj;
        Sophus::SO3<T> R_I0toIj;
        basalt::CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, normalized_time , double(1e9)/interval_ns, &R_I0toIj, nullptr, nullptr);
        basalt::CeresSplineHelper<N>::template evaluate<T, 3, 0>(sKnots + N, normalized_time, double(1e9)/ interval_ns, &t_I0toIj);

        auto t_L0toLj = q_ItoL.matrix().transpose() * (R_I0toIj * t_ItoL + t_I0toIj - t_ItoL);
        auto p_L0 = q_ItoL.matrix().transpose() * R_I0toIj * q_ItoL.matrix() * pose + t_L0toLj;
        *r = plannar.block<3,1>(0,0).template cast<T>().dot(p_L0) + (plannar.template cast<T>[3]);
        *r = T(weight_) * (*r);
        return true;
    }
    Eigen::Vector4d plannar;
    double normalized_time;
    double interval_ns, weight_;
    Eigen::Vector3d pose;
};



namespace lci_cali{


template <int N = 4>
class IMUSpline : public BSplineSE3<N>
{

public:
    IMUSpline(double start_time, double end_time, double interval):
        BSplineSE3<N>(start_time, end_time, interval){}

    void addImuGyroMeasurement(double time, Eigen::Vector3d& gyro, Eigen::Vector3d& bias_g, double weight){
        if(time < this->start_time || time > this->end_time)return;
        auto[vec_so3_rd, u] = this->GetMeasFitKnots(time * 1e9);
        vec_so3_rd.erase(vec_so3_rd.begin() + N, vec_so3_rd.end());
        vec_so3_rd.push_back(bias_g.data());
        auto factor = new ImuSplineImuGyroMeasFactor<N>(gyro, u, this->interval * 1e9, weight);
        auto* cost_function = new ceres::DynamicAutoDiffCostFunction<ImuSplineImuGyroMeasFactor<N>>(factor);
        for(int i = 0; i < N; i++){
            cost_function->AddParameterBlock(4);
        }
        cost_function->AddParameterBlock(3); // bias
        cost_function->SetNumResiduals(3);
        if(CFG_GET_BOOL("imu_spline_gyro_use_loss")){
            this->problem->AddResidualBlock(cost_function, new ceres::CauchyLoss(CFG_GET_FLOAT("imu_spline_gyro_loss_value")), vec_so3_rd);
        }else{
            this->problem->AddResidualBlock(cost_function, nullptr, vec_so3_rd);
        }
        for(int i = 0; i < N; i++){
            this->problem->SetParameterization(vec_so3_rd[i], this->local_parameterization);
        } 
        count_imu_gyro_data++;
    }

    void addImuAccelMeasurement(double time, Eigen::Vector3d& accel, Eigen::Vector3d& bias_g, Eigen::Vector3d& g,double weight){
        if(time < this->start_time || time > this->end_time)return;
        auto[vec_so3_rd, u] = this->GetMeasFitKnots(time * 1e9);
        vec_so3_rd.push_back(bias_g.data()); // bias
        vec_so3_rd.push_back(g.data()); // g
        auto factor = new ImuSplineImuAccelMeasFactor<N>(accel, u, this->interval * 1e9, weight);
        auto* cost_function = new ceres::DynamicAutoDiffCostFunction<ImuSplineImuAccelMeasFactor<N>>(factor);
        for(int i = 0; i < N; i++){
            cost_function->AddParameterBlock(4);
        }
        cost_function->AddParameterBlock(3); // bias
        cost_function->AddParameterBlock(3); // g
        cost_function->SetNumResiduals(3);
        if(CFG_GET_BOOL("imu_spline_accel_use_loss")){
            this->problem->AddResidualBlock(cost_function, new ceres::CauchyLoss(CFG_GET_FLOAT("imu_spline_gyro_loss_value")), vec_so3_rd);
        }else{
            this->problem->AddResidualBlock(cost_function, nullptr, vec_so3_rd);
        }
        for(int i = 0; i < N; i++){
            this->problem->SetParameterization(vec_so3_rd[i], this->local_parameterization);
        } 
        count_imu_accel_data++;
    }
    template<typename PointT>
    void addLidarPlannarMeasurement(double time_stamp, PointT& p, slam_utils::PlaneMap<PointT>& map, Eigen::Quaterniond& q_ItoL, Eigen::Vector3d& t_ItoL,double weight){
        if(time_stamp < this->start_time_stamp || time_stamp > this->end_time_stamp)return;
        auto[vec_so3_rd, u] = this->GetMeasFitKnots(time_stamp * 1e9);
        vec_so3_rd.push_back(q_ItoL.coeffs().data()); // q_ItoL
        vec_so3_rd.push_back(t_ItoL.data()); // t_ItoL

        Eigen::Vector3d vec_p(p.x, p.y, p.z);
        Eigen::Vector4f plane_coffe;
        int index;
        float dis_threshold = CFG_GET_FLOAT("refine_plane_factor_dis");
        if(!map.serachNearestPlane(p, dis_threshold, plane_coffe, index)){
            return;
        }
        auto factor = new ImuSplineLidarPlanarFactor<N>(vec_p, plane_coffe.cast<double>(), u, this->interval * 1e9, weight);
        auto* cost_function = new ceres::DynamicAutoDiffCostFunction<ImuSplineLidarPlanarFactor<N>>(factor);
        for(int i = 0; i < N; i++){
            cost_function->AddParameterBlock(4);
        }
        cost_function->AddParameterBlock(4); // q_ItoL
        cost_function->AddParameterBlock(3); // t_ItoL
        cost_function->SetNumResiduals(3);
        if(CFG_GET_BOOL("imu_spline_plannar_use_loss")){
            this->problem->AddResidualBlock(cost_function, new ceres::CauchyLoss(CFG_GET_FLOAT("imu_spline_gyro_loss_value")), vec_so3_rd);
        }else{
            this->problem->AddResidualBlock(cost_function, nullptr, vec_so3_rd);
        }
        for(int i = 0; i < N; i++){
            this->problem->SetParameterization(vec_so3_rd[i], this->local_parameterization);
        }
        this->problem->SetParameterization(q_ItoL.coeffs().data(), this->local_parameterization);
        count_imu_gyro_data++;
    }
    void printInfo()
    {
        std::cout << "count_lidar_data: " << count_lidar_data << std::endl;
        std::cout << "count_imu_accel_data: " << count_imu_accel_data << std::endl;
        std::cout << "count_imu_gyro_data: " << count_imu_gyro_data << std::endl;
    }

    ~IMUSpline(){}
private:
    int count_lidar_data = 0, count_imu_gyro_data = 0, count_imu_accel_data = 0;

};

}

