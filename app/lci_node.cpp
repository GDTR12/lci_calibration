#include "iostream"
#include "rclcpp/rclcpp.hpp"
#include "lci_cali/input.hpp"
#include "lci_cali/lci_cali.hpp"
#include "basalt/spline/so3_spline.h"
#include "basalt/spline/rd_spline.h"
#include "basalt/spline/ceres_spline_helper.h"
#include "basalt/spline/ceres_local_param.hpp"
#include "sophus/so3.hpp"
#include "ceres/ceres.h"
#include "ceres/problem.h"


#include <ceres/ceres.h>
#include <iomanip>
#include <iostream>
#include <sophus/se3.hpp>

#include <basalt/spline/se3_spline.h>
#include <basalt/spline/so3_spline.h>

#include <ceres_lie_spline.h>


struct CostFunctor
{
    template <typename T>
    bool operator()(const T* const x, T* residual) const
    {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

static int i = 0;
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

// y = a * X^2 + b * X + c;
struct TestFactor
{
    template<typename T>
    bool operator()(T const* const* param, T* residual) const{
        residual[0] = T(meas_y) - (param[0][0] * T(std::pow(meas_x, 2)) + param[0][1] * T(meas_x) + param[0][2]); 
        return true;
    }
    double meas_x, meas_y;
};

void Test()
{
    const int meas_num = 5000;
    double y[meas_num];
    double x[meas_num] = {0};
    int a = 5; 
    int b = 3;
    int c = 6;
    for(int i = 0; i < meas_num; i++){
        x[i] = i;
        double noise = 0.1 * std::rand() / RAND_MAX;
        y[i] = a * std::pow(x[i], 2) + b * x[i] + c + noise;
    }
    double param_block[3] = {0};
    ceres::Problem problem;
    for(int i = 0; i < meas_num; i++){
        // auto cost_function = new ceres::AutoDiffCostFunction<TestFactor, 1, 3>(new TestFactor{x[i], y[i]});

        auto cost_function = new ceres::DynamicAutoDiffCostFunction<TestFactor>(new TestFactor{x[i], y[i]});
        cost_function->AddParameterBlock(3);
        cost_function->SetNumResiduals(1);
        problem.AddResidualBlock(cost_function, nullptr, param_block);
    }
    ceres::Solver::Options options;
    ceres::Solver::Summary summary; //优化信息
    ceres::Solve(options, &problem, &summary);  //开始执行求解
    std::cout << summary.FullReport() << std::endl;
    std::cout << "param:" << param_block[0] << " " << param_block[1] << " " << param_block[2] << std::endl;
}

template<int N>
std::pair<std::vector<double*>, double> GyroMeasFitKnots(uint64_t gyro_ns, So3Spline<N>& so3){
    std::vector<double*> ret;
    uint64_t st_ns = (gyro_ns - so3.minTimeNs());
    uint64_t s = st_ns / so3.getTimeIntervalNs();
    double u = double(st_ns % so3.getTimeIntervalNs()) / double(so3.getTimeIntervalNs());
    for ( int i = 0; i < N; i++){
        ret.emplace_back(const_cast<double*>(so3.getKnots()[s + i].data()));
    }
    return std::pair<std::vector<double*>, double>(ret, u);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);

    double time_end = 2;
    const int degree = 4;
    double time_interval = 0.001;
    ceres::Problem problem;
    std::vector<double*> knots;
    auto local_parameterization = new basalt::LieLocalParameterization<Sophus::SO3d>();
    basalt::So3Spline<degree, double> spline(time_interval * 1e9);
    for( int i = 0; i < time_end / time_interval + degree; i++){
        spline.knotsPushBack(Sophus::SO3d(Eigen::Matrix3d::Identity()));
    }

    for(double i = 0.02; i < time_end; i +=  0.01){
        lci_cali::IMUData data;
        data.timestamp = i;
        data.gyro << 0, 0.025 * i, 0.15 * i ;
        auto [vec, u] = GyroMeasFitKnots(data.timestamp * 1e9, spline);
        auto factor = new GyroscopFactor<degree>(data, u, spline.getTimeIntervalNs());
        auto* cost_function = new ceres::DynamicAutoDiffCostFunction<GyroscopFactor<degree>>(factor);
        for(int i = 0; i < degree; i++){
            cost_function->AddParameterBlock(4);
        }
        cost_function->SetNumResiduals(3);
        problem.AddResidualBlock(cost_function, NULL, vec);
        for(int i = 0; i < degree; i++){
            problem.SetParameterization(vec[i], local_parameterization);
        }
    }

    ceres::Solver::Options options;
   options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
   options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    ceres::Solver::Summary summary; //优化信息
    ceres::Solve(options, &problem, &summary);  //开始执行求解
    std::cout <<  summary.FullReport() << std::endl;

    for(double t = 0; t < time_end; t += 0.01){
        // std::cout << "t: " << t << std::endl;
        // Sophus::SO3d::Tangent rot_vel;
        // rot_vel = spline.velocityBody(t * 1e9);
        // auto [vec, u] = GyroMeasFitKnots(t * 1e9, spline);
        // double normalized_time = u;
        // Sophus::SO3d rot;
        // basalt::CeresSplineHelper<degree>::template evaluate_lie( reinterpret_cast<const double* const*>(vec.data()), u, 1/time_interval, &rot, &rot_vel);
        // std::cout << rot.matrix() << std::endl;
        // std::cout << rot_vel.matrix() << std::endl;

        Sophus::SO3d::Tangent rot_vel = spline.velocityBody(t * 1e9);
        Sophus::SO3d rot = spline.evaluate(t * 1e9);

        std::cout << rot.matrix() << std::endl;
        std::cout << rot_vel.matrix() << std::endl;
    }

}
