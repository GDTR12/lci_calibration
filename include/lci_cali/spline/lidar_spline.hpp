#pragma once
#include "Eigen/Core"
#include "lci_cali/continuous_se3.hpp"
#include "utils/plane_map.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
namespace lci_cali{

#define GRAVITY_NORM  -9.797

template <typename T>
Eigen::Matrix<T, 3, 1> refined_gravity(
    Eigen::Map<const Eigen::Matrix<T, 2, 1>>& g_param) {
  T cr = ceres::cos(g_param[0]);
  T sr = ceres::sin(g_param[0]);
  T cp = ceres::cos(g_param[1]);
  T sp = ceres::sin(g_param[1]);
  return Eigen::Matrix<T, 3, 1>(-sp * cr * T(GRAVITY_NORM),
                                sr * T(GRAVITY_NORM),
                                -cr * cp * T(GRAVITY_NORM));
}



template<int N>
struct ImuAccelMeasFactor {
    ImuAccelMeasFactor(Eigen::Vector3d data, double normalized_time, double interval_ns, double weight):
        normalized_time(normalized_time),accel_data(data), interval_ns(interval_ns), weight(weight){}
    template <class T>
    bool operator()(T const* const* sKnots, T* r) const{
        using Tangent = typename Sophus::SO3<T>::Tangent;
        Eigen::Map<Eigen::Matrix<T,3,1>> residual(r);
        residual = Tangent(T(0),T(0),T(0));

        Eigen::Map<Eigen::Matrix<T, 2, 1> const> g(sKnots[2 * N]);
        Eigen::Map<Eigen::Matrix<T, 3, 1> const> bias(sKnots[2 * N + 1]);
        Eigen::Map<Eigen::Quaternion<T> const> q_ItoL(sKnots[2 * N  + 2]);
        Eigen::Map<Eigen::Matrix<T, 3, 1> const> t_ItoL(sKnots[2 * N + 3]);

        Tangent rot_vel, rot_accel, accel;
        Sophus::SO3<T> rot;
        basalt::CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, normalized_time , double(1e9)/interval_ns, &rot, &rot_vel, &rot_accel);
        basalt::CeresSplineHelper<N>::template evaluate<T, 3, 2>(sKnots + N, normalized_time, double(1e9)/ interval_ns, &accel);

        auto refined_g = refined_gravity(g);

        residual =  accel_data.template cast<T>() - bias - q_ItoL.matrix() * (
            rot.matrix().transpose() * (accel + refined_g) 
            - Sophus::SO3<T>::hat(rot_accel).matrix() * q_ItoL.matrix().transpose() * t_ItoL 
            - Sophus::SO3<T>::hat(rot_vel) * Sophus::SO3<T>::hat(rot_vel) * q_ItoL.matrix().transpose() * t_ItoL
        );
        // residual =  accel_data.template cast<T>() - bias - q_ItoL.matrix() * (
        //     rot.matrix().transpose() * (accel + refined_g) 
        //     - Sophus::SO3<T>::hat(rot_accel).matrix() * t_ItoL 
        //     - Sophus::SO3<T>::hat(rot_vel) * Sophus::SO3<T>::hat(rot_vel) * t_ItoL
        // );
        residual = T(weight) * residual;
        
        return true;
    }
    Eigen::Vector3d accel_data;
    double normalized_time;
    double interval_ns, weight;
};

template<int N>
struct LidarPlanarFactor {
    LidarPlanarFactor(Eigen::Vector3d data, Eigen::Vector4d surfel, double normalized_time, double interval_ns, double weight):
        normalized_time(normalized_time),pose(data),plannar(surfel),interval_ns(interval_ns), weight_(weight){}
    template <class T>
    bool operator()(T const* const* sKnots, T* r) const{
        using Tangent = typename Sophus::SO3<T>::Tangent;
        // Eigen::Map<Eigen::Matrix<T,1,1>> residual(r);
        // residual = Tangent(T(0));
        Tangent trans;
        Sophus::SO3<T> rot;
        basalt::CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, normalized_time , double(1e9)/interval_ns, &rot, nullptr, nullptr);
        basalt::CeresSplineHelper<N>::template evaluate<T, 3, 0>(sKnots + N, normalized_time, double(1e9)/ interval_ns, &trans);
        Tangent p_l0 = rot.matrix() * (pose.template cast<T>()) + trans;
        *r = plannar.block<3,1>(0,0).template cast<T>().dot(p_l0) + (plannar.template cast<T>()[3]);
        *r = T(weight_) * (*r);
        return true;
    }
    Eigen::Vector4d plannar;
    double normalized_time;
    double interval_ns, weight_;
    Eigen::Vector3d pose;
};
extern int iaad;
template<int N>
struct ImuGyroMeasFactor {
    ImuGyroMeasFactor(Eigen::Vector3d data, double normalized_time, double interval_ns, double weight):
        normalized_time(normalized_time),gyro_data(data), interval_ns(interval_ns), weight(weight){}
    template <class T>
    bool operator()(T const* const* sKnots, T* r) const{
        using Tangent = typename Sophus::SO3<T>::Tangent;
        Eigen::Map<Eigen::Matrix<T,3,1>> residual(r);
        Eigen::Map<Eigen::Quaternion<T> const> qua(sKnots[N]);
        // std::cout << qua.matrix() <<  std::endl;
        // exit(0);
        Eigen::Map<Eigen::Matrix<T,3,1> const> bias_g(sKnots[N+1]);
        residual = Tangent(T(0),T(0),T(0));
        Tangent rot_vel;
        Sophus::SO3<T> rot;
        basalt::CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, normalized_time , double(1e9)/interval_ns, &rot, &rot_vel, nullptr);
        residual =  gyro_data.template cast<T>() - qua.matrix() * rot_vel - bias_g;
        residual = T(weight) * residual;
        // std::cout << "rot_vel: " << rot_vel.x() << " " << rot_vel.y() << " " << rot_vel.z() << " " << std::endl;
        // std::cout << "residual: " << residual.x() << " " << residual.y() << " " << residual.z() << " " << std::endl;
        // if()
        // iaad++;

        // if(residual.x() > T(1000)) std::cout << nor
        return true;
    }
    Eigen::Vector3d gyro_data;
    double normalized_time;
    double interval_ns, weight;
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

template<int N>
struct LidarOrientationFactor {
    LidarOrientationFactor(Sophus::SO3d data, double normalized_time, double interval_ns):
        normalized_time(normalized_time),meas_data(data), interval_ns(interval_ns){}
    template <class T>
    bool operator()(T const* const* sKnots, T* r) const{
        using Tangent = typename Sophus::SO3<T>::Tangent;
        Eigen::Map<Eigen::Matrix<T,3,1>> residual(r);
        residual = Tangent(T(0),T(0),T(0));
        Sophus::SO3<T> transform;
        basalt::CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, normalized_time , double(1e9)/interval_ns, &transform);
        // residual =  meas_data.template cast<T>() - transform.log();
        residual = (transform * meas_data.inverse().template cast<T>()).log();
        return true;
    }
    Sophus::SO3d meas_data;
    double normalized_time;
    double interval_ns;
};

template <int N>
class LidarSpline : public BSplineSE3<N>
{
public:
    LidarSpline(double start_time, double end_time, double interval):
        BSplineSE3<N>(start_time, end_time, interval)
    {
    }

    void addIMUGyroMeasurement(double time_stamp, Eigen::Vector3d gyro, Eigen::Vector3d& bias, Eigen::Quaterniond& q_ItoL){
        assert(time_stamp >= this->start_time && time_stamp <= this->end_time);
        auto[vec_so3_rd, u] = this->GetMeasFitKnots(time_stamp * 1e9);
        vec_so3_rd.erase(vec_so3_rd.begin() + N, vec_so3_rd.end());
        vec_so3_rd.push_back(q_ItoL.coeffs().data()); // x, y, z, w
        vec_so3_rd.push_back(bias.data());
        auto factor = new ImuGyroMeasFactor<N>(gyro, u, this->interval * 1e9,  CFG_GET_FLOAT("refine_imu_gyro_weight"));
        auto* cost_function = new ceres::DynamicAutoDiffCostFunction<lci_cali::ImuGyroMeasFactor<N>>(factor);
        for(int i = 0; i < N; i++){
            cost_function->AddParameterBlock(4);
        }
        cost_function->AddParameterBlock(4); // q_ItoL
        cost_function->AddParameterBlock(3); // bias
        cost_function->SetNumResiduals(3);
        if(CFG_GET_BOOL("lidar_spline_gyro_use_loss")){
            this->problem->AddResidualBlock(cost_function, new ceres::CauchyLoss(CFG_GET_FLOAT("lidar_spline_gyro_loss_value")), vec_so3_rd);
        }else{
            this->problem->AddResidualBlock(cost_function, nullptr, vec_so3_rd);
        }
        for(int i = 0; i < N; i++){
            this->problem->SetParameterization(vec_so3_rd[i], this->local_parameterization);
        } 
        this->problem->SetParameterization(q_ItoL.coeffs().data(), this->local_parameterization);
        count_imu_gyro_data++;
        // Sophus::SO3d so3d = this->so3_spline->evaluate(time_stamp * 1e9);
        // std::cout << "At t: " << time_stamp << " " <<  so3d.unit_quaternion().coeffs().transpose() << std::endl;

    }

    void addIMUAccelMeasurement(double time_stamp, Eigen::Vector3d accel, Eigen::Vector3d& g, Eigen::Vector3d& bias,Eigen::Quaterniond& q_ItoL, Eigen::Vector3d& t_ItoL){
        assert(time_stamp >= this->start_time && time_stamp <= this->end_time);
        auto[vec_so3_rd, u] = this->GetMeasFitKnots(time_stamp * 1e9);
        vec_so3_rd.push_back(g.data());
        vec_so3_rd.push_back(bias.data());
        // Eigen::Map<Eigen::Vector4d> qua(); 
        vec_so3_rd.push_back(q_ItoL.coeffs().data()); // x, y, z, w
        vec_so3_rd.push_back(t_ItoL.data());

        auto factor = new ImuAccelMeasFactor<N>(accel, u, this->interval * 1e9,  CFG_GET_FLOAT("refine_imu_accel_weight"));
        auto* cost_function = new ceres::DynamicAutoDiffCostFunction<lci_cali::ImuAccelMeasFactor<N>>(factor);

        for(int i = 0; i < N; i++){
            cost_function->AddParameterBlock(4); // so3_spline
        }
        for(int i = 0; i < N; i++){
            cost_function->AddParameterBlock(3);  // rd_spline
        }
        cost_function->AddParameterBlock(3); // g
        cost_function->AddParameterBlock(3); // bias
        cost_function->AddParameterBlock(4); // q_ItoL
        cost_function->AddParameterBlock(3); // t_ItoL

        cost_function->SetNumResiduals(3);
        if(CFG_GET_BOOL("lidar_spline_accel_use_loss")){
            this->problem->AddResidualBlock(cost_function, new ceres::CauchyLoss(CFG_GET_FLOAT("lidar_spline_accel_loss_value")), vec_so3_rd);
        }else{
            this->problem->AddResidualBlock(cost_function, nullptr, vec_so3_rd);
        }
        for(int i = 0; i < N; i++){
            this->problem->SetParameterization(vec_so3_rd[i], this->local_parameterization);
            // this->problem->SetParameterization();
        } 
        this->problem->SetParameterization(q_ItoL.coeffs().data(), this->local_parameterization);
        count_imu_accel_data++;
    }

    void addLidarPositionMeasurement(double time_stamp, Eigen::Vector3d translate)
    {
        assert(time_stamp >= this->start_time && time_stamp <= this->end_time);
        auto[vec_so3_rd, u] = this->GetMeasFitKnots(time_stamp * 1e9);
        std::vector<double*> vec_rd;
        vec_rd.insert(vec_rd.end(), vec_so3_rd.begin() + N, vec_so3_rd.end());
        
        auto factor = new LidarPoseFactor<N>(translate, u, this->rd_spline->getTimeIntervalNs());
        auto* cost_func = new ceres::DynamicAutoDiffCostFunction<lci_cali::LidarPoseFactor<N>>(factor);

        for(int i = 0; i < N; i++){
            cost_func->AddParameterBlock(3);
        }
        cost_func->SetNumResiduals(3);
        this->problem->AddResidualBlock(cost_func, NULL, vec_rd);
        if(this->problem->NumParameterBlocks() == 1){
            this->problem->SetParameterBlockConstant(vec_rd[0]);
        }
    }

    void addLidarOrientationMeasurement(double time_stamp, Sophus::SO3d orient){
        assert(time_stamp >= this->start_time && time_stamp <= this->end_time);
        auto[vec_so3_rd, u] = this->GetMeasFitKnots(time_stamp * 1e9);
        vec_so3_rd.erase(vec_so3_rd.begin() + N, vec_so3_rd.end());
        auto factor = new LidarOrientationFactor<N>(orient, u, this->so3_spline->getTimeIntervalNs());
        auto* cost_function = new ceres::DynamicAutoDiffCostFunction<lci_cali::LidarOrientationFactor<N>>(factor);
        for(int i = 0; i < N; i++){
            cost_function->AddParameterBlock(4);
        }
        cost_function->SetNumResiduals(3);
        this->problem->AddResidualBlock(cost_function, NULL, vec_so3_rd);
        for(int i = 0; i < N; i++){
            this->problem->SetParameterization(vec_so3_rd[i], this->local_parameterization);
        }  
    }


    template<typename  PointT = pcl::PointXYZ>
    void addLidarPlannarMeasurement(double time_stamp, PointT& p, slam_utils::PlaneMap<PointT>& map){
        if(!pcl_isfinite(p.x) || !pcl_isfinite(p.y) || !pcl_isfinite(p.z))return;
        if(time_stamp > this->end_time || time_stamp < this->start_time)return;
        auto[vec_so3_rd, u] = this->GetMeasFitKnots(time_stamp * 1e9);
        const int k = 1;
        float dis_threshold = CFG_GET_FLOAT("refine_plane_factor_dis");
        // std::cout << time_stamp << " " << this->start_time << " " 
        //     << this->end_time << " " << this->so3_spline->minTimeNs() / 1e9 
        //     << " " << this->so3_spline->maxTimeNs()/1e9 << std::endl;
        Sophus::SO3d rot = this->so3_spline->evaluate(time_stamp * 1e9);
        Eigen::Vector3d trans = this->rd_spline->template evaluate<0>(time_stamp * 1e9);
        PointT point;
        Eigen::Map<Eigen::Vector3f> p_dst(&point.x);
        Eigen::Map<Eigen::Vector3f> p_src(&p.x);
        p_dst = rot.matrix().cast<float>() * p_src + trans.cast<float>();
        Eigen::Vector4f plane_coffe;
        int index;
        if(!map.serachNearestPlane(point, map.resolution * 0.8, plane_coffe, index)){
            return;
        }
        if(plane_coffe.block<3,1>(0,0).dot(p_dst) + plane_coffe(3) > dis_threshold){
            return;
        }
        if(CFG_GET_BOOL("lidar_spline_show_detail")){
            std::vector<std::array<int, 3>> color;
            std::vector<float> size;
            std::vector<typename pcl::PointCloud<PointT>::Ptr> cloud;
            
            for (size_t i = 0; i < map.pcls.size(); i++)
            {
                if(index != i){
                    color.push_back({100, 100, 100});
                    size.push_back(1.0);
                }else{
                    color.push_back({0, 255, 0});
                    size.push_back(1.0);
                }
                cloud.push_back(map.pcls.at(i));
            }
            auto center = pcl::make_shared<pcl::PointCloud<PointT>>(map.center_points);
            auto dst_point = pcl::make_shared<pcl::PointCloud<PointT>>();
            dst_point->push_back(PointT(point));
            dst_point->push_back(PointT(p));
            color.push_back({255,0,0});
            size.push_back(5.0);
            color.push_back({0,0,255});
            size.push_back(5.0);
            cloud.push_back(center);
            cloud.push_back(dst_point);

            slam_utils::showPcl<PointT>(cloud, color, size, "detail");
        }


        auto factor = new LidarPlanarFactor<N>(p_src.cast<double>(), plane_coffe.cast<double>(), u, this->interval * 1e9, CFG_GET_FLOAT("refine_imu_lidar_weight"));
        auto* cost_function = new ceres::DynamicAutoDiffCostFunction<LidarPlanarFactor<N>>(factor);
        for(int i = 0; i < N; i++){
            cost_function->AddParameterBlock(4);
        }
        for(int i = 0; i < N; i++){
            cost_function->AddParameterBlock(3);
        }
        cost_function->SetNumResiduals(1);
        if(CFG_GET_BOOL("lidar_spline_plannar_use_loss")){
            this->problem->AddResidualBlock(cost_function,  new ceres::CauchyLoss(CFG_GET_FLOAT("lidar_spline_plannar_loss_value")), vec_so3_rd);
        }else{
            this->problem->AddResidualBlock(cost_function, nullptr, vec_so3_rd);
        }
        for(int i = 0; i < N; i++){
            this->problem->SetParameterization(vec_so3_rd[i], this->local_parameterization);
        }
        count_lidar_data++;
        lidar_cost += std::abs(plane_coffe.block<3,1>(0,0).dot(p_dst) + plane_coffe(3));
    }

    
    void setParamBound(double* param, int index, double lower, double upper)
    {
        // // this->problem->();
        this->problem->SetParameterLowerBound(param, index, lower);
        this->problem->SetParameterUpperBound(param, index, upper);
    }


    // void setParaml


    void printInfo()
    {
        std::cout << "count_lidar_data: " << count_lidar_data << " cost: " << lidar_cost << std::endl;
        std::cout << "count_imu_accel_data: " << count_imu_accel_data << std::endl;
        std::cout << "count_imu_gyro_data: " << count_imu_gyro_data << std::endl;
    }

    ~LidarSpline() override
    {}

private:
    int count_lidar_data = 0, count_imu_gyro_data = 0, count_imu_accel_data = 0;
    double lidar_cost = 0;
protected:

};




}