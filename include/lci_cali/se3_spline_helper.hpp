#pragma once
#include "utils/input.hpp"
#include "basalt/spline/se3_spline.h"
#include "ceres/ceres.h"
#include "sensor_data/imu_data.hpp"
#include "utils/config_yaml.h"
#include "basalt/spline/so3_spline.h"
#include "basalt/spline/rd_spline.h"
#include "basalt/spline/ceres_spline_helper.h"
#include "basalt/spline/ceres_local_param.hpp"
#include "basalt/calibration/calibration.hpp"

namespace lci_cali{

template <int _N, bool OLD_TIME_DERIV>
struct CalibGyroCostFunctorSE3 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CalibGyroCostFunctorSE3(const Eigen::Vector3d& measurement, double u,
                          double inv_dt, double inv_std = 1)
      : measurement(measurement), u(u), inv_dt(inv_dt), inv_std(inv_std) {}

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Vec6 = Eigen::Matrix<T, 6, 1>;

    Eigen::Map<Vec3> residuals(sResiduals);

    Vec6 rot_vel;

    if constexpr (OLD_TIME_DERIV) {
      CeresSplineHelperOld<_N>::template evaluate_lie_vel_old<T, Sophus::SE3>(
          sKnots, u, inv_dt, nullptr, &rot_vel);
    } else {
      CeresSplineHelper<_N>::template evaluate_lie<T, Sophus::SE3>(
          sKnots, u, inv_dt, nullptr, &rot_vel, nullptr);
    }

    Eigen::Map<Vec3 const> const bias(sKnots[_N]);

    residuals =
        inv_std * (rot_vel.template tail<3>() - measurement.cast<T>() + bias);

    return true;
  }

  Eigen::Vector3d measurement;
  double u, inv_dt, inv_std;
};

template <int _N, bool OLD_TIME_DERIV>
struct CalibAccelerationCostFunctorSE3 {
  static constexpr int N = _N;  // Order of the spline.

  using VecN = Eigen::Matrix<double, _N, 1>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CalibAccelerationCostFunctorSE3(const Eigen::Vector3d& measurement, double u,
                                  double inv_dt, double inv_std)
      : measurement(measurement), u(u), inv_dt(inv_dt), inv_std(inv_std) {}

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Vector6 = Eigen::Matrix<T, 6, 1>;

    using Matrix4 = Eigen::Matrix<T, 4, 4>;

    Eigen::Map<Vector3> residuals(sResiduals);

    Sophus::SE3<T> T_w_i;
    Vector6 vel, accel;

    if constexpr (OLD_TIME_DERIV) {
      CeresSplineHelperOld<N>::template evaluate_lie_accel_old<T, Sophus::SE3>(
          sKnots, u, inv_dt, &T_w_i, &vel, &accel);
    } else {
      CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SE3>(
          sKnots, u, inv_dt, &T_w_i, &vel, &accel);
    }

    Matrix4 vel_hat = Sophus::SE3<T>::hat(vel);
    Matrix4 accel_hat = Sophus::SE3<T>::hat(accel);

    Matrix4 ddpose = T_w_i.matrix() * (vel_hat * vel_hat + accel_hat);

    Vector3 accel_w = ddpose.col(3).template head<3>();

    // Gravity
    Eigen::Map<Vector3 const> const g(sKnots[N]);
    Eigen::Map<Vector3 const> const bias(sKnots[N + 1]);

    residuals = inv_std * (T_w_i.so3().inverse() * (accel_w + g) -
                           measurement.cast<T>() + bias);

    return true;
  }

  Eigen::Vector3d measurement;
  double u, inv_dt, inv_std;
};

template<int _N>
class SplineSE3{
public:
    static constexpr int N = _N;
    SplineSE3(int64_t time_interval_ns, int64_t start_time_ns = 0)
        :dt_ns(time_interval_ns), start_t_ns(start_time_ns)
    {
        // spline = std::make_shared<basalt::Se3Spline<N>>();
        accel_bias.setZero();
        gyro_bias.setZero();
    }
    void setAprilgridCorners3d(const Eigen::aligned_vector<Eigen::Vector4d>& v) {
        aprilgrid_corner_pos_3d = v;
    }
    void init(const Sophus::SE3d& init, int num_knots) {
        knots = Eigen::aligned_vector<Sophus::SE3d>(num_knots, init);
        ceres::Manifold* manifold = new Sophus::Manifold<Sophus::SE3>();
        for (int i = 0; i < num_knots; i++) {
            problem.AddParameterBlock(knots[i].data(), Sophus::SE3d::num_parameters,
                                    manifold);
        }

        // for (size_t i = 0; i < calib.T_i_c.size(); i++) {
        //     problem.AddParameterBlock(calib.T_i_c[i].data(),
        //                             Sophus::SE3d::num_parameters,
        //                             manifold);
        // }
  }

  void addGyroMeasurement(const Eigen::Vector3d& meas, int64_t time_ns) {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    using FunctorT = CalibGyroCostFunctorSE3<_N, 0>;

    FunctorT* functor = new FunctorT(
        meas, u, inv_dt, 1.0 / calib.dicrete_time_gyro_noise_std()[0]);

    ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(7);
    }
    cost_function->AddParameterBlock(3);
    cost_function->SetNumResiduals(3);

    std::vector<double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(knots[s + i].data());
    }
    vec.emplace_back(gyro_bias.data());

    problem.AddResidualBlock(cost_function, NULL, vec);
  }

  void addAccelMeasurement(const Eigen::Vector3d& meas, int64_t time_ns) {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    using FunctorT = CalibAccelerationCostFunctorSE3<N, 0>;

    FunctorT* functor = new FunctorT(
        meas, u, inv_dt, 1.0 / calib.dicrete_time_accel_noise_std()[0]);

    ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(7);
    }
    cost_function->AddParameterBlock(3);
    cost_function->AddParameterBlock(3);
    cost_function->SetNumResiduals(3);

    std::vector<double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(knots[s + i].data());
    }
    vec.emplace_back(g.data());
    vec.emplace_back(accel_bias.data());

    problem.AddResidualBlock(cost_function, NULL, vec);
  }
  ceres::Solver::Summary optimize() {
    ceres::Solver::Options options;
    options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
    options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 200;
    options.num_threads = 1;

    // Solve
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    return summary;
  }


  Eigen::Vector3d getGyro(int64_t time_ns) const {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    Sophus::Vector6d gyro;

    std::vector<const double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(knots[s + i].data());
    }

    CeresSplineHelper<N>::template evaluate_lie<double, Sophus::SE3>(
        &vec[0], u, inv_dt, nullptr, &gyro);

    return gyro.tail<3>();
  }

  Eigen::Vector3d getAccel(int64_t time_ns) const {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    Sophus::SE3d pose;
    Sophus::Vector6d se3_vel, se3_accel;

    {
      std::vector<const double*> vec;
      for (int i = 0; i < N; i++) {
        vec.emplace_back(knots[s + i].data());
      }

      CeresSplineHelper<N>::template evaluate_lie<double, Sophus::SE3>(
          &vec[0], u, inv_dt, &pose, &se3_vel, &se3_accel);
    }

    Eigen::Matrix4d vel_hat = Sophus::SE3d::hat(se3_vel);
    Eigen::Matrix4d accel_hat = Sophus::SE3d::hat(se3_accel);

    Eigen::Matrix4d ddpose = pose.matrix() * (vel_hat * vel_hat + accel_hat);

    Eigen::Vector3d accel_w = ddpose.col(3).template head<3>();

    Eigen::Vector3d accel_meas = pose.so3().inverse() * (accel_w + g);

    return accel_meas;
  }


    void setCalib(const basalt::Calibration<double>& c) { calib = c; }
    void setG(const Eigen::Vector3d& g_a) { g = g_a; }

    const Eigen::Vector3d& getG() { return g; }
    const basalt::Calibration<double>& getCalib() { return calib; }

    Eigen::Vector3d getGyroBias() { return gyro_bias; }
    Eigen::Vector3d getAccelBias() { return accel_bias; }
private:
    ceres::Problem problem;
    // std::shared_ptr<basalt::Se3Spline<N>> spline;
    Eigen::aligned_vector<Eigen::Vector4d> aprilgrid_corner_pos_3d;
    Eigen::aligned_vector<Sophus::SE3d> knots;
    Eigen::Vector3d g, accel_bias, gyro_bias;
    basalt::Calibration<double> calib;
    int64_t dt_ns, start_t_ns;
    double inv_dt;
};

basalt::Calibration<double> calib;


void run_calibration() {
    using SplineT = SplineSE3<5>;
    std::cout << "=============================================" << std::endl;
    std::cout << "Running calibration with SE3 method"
            << std::endl;
    constexpr int N = SplineT::N;
    const int64_t dt_ns = 1e7;

    int64_t start_t_ns = 0;
    // TODO:
    int64_t end_t_ns = 7 * 1e9;

    auto input =  slam_utils::Input<sensor_data::LidarData<PointIRT>>::createInstance();
    ConfigYaml cfg;
    YAML::Node cfg_root = ConfigYaml::cfg_root;
    auto imu_data = input->getIMUData();

    SplineT calib_spline(dt_ns, start_t_ns);
    //TODO:
    // calib_spline.setAprilgridCorners3d(4);
    calib_spline.setCalib(calib);
    // TODO: 
    // basalt::TimeCamId tcid_init(vio_dataset->get_image_timestamps().front(), 0);
    // 使用单位阵初始化Spline
    calib_spline.init(Sophus::SE3d(Eigen::Matrix4d::Identity()), (end_t_ns - start_t_ns) / dt_ns + N);
    bool g_initialized = false;
    Eigen::Vector3d g_a_init = Eigen::Vector3d(0,0,9.81);

    // for (size_t j = 0; j < vio_dataset->get_image_timestamps().size(); ++j) {
    //     int64_t timestamp_ns = vio_dataset->get_image_timestamps()[j];

    //     basalt::TimeCamId tcid(timestamp_ns, 0);
    //     const auto cp_it = calib_init_poses.find(tcid);

    //     if (cp_it != calib_init_poses.end()) {
    //     Sophus::SE3d T_a_i = cp_it->second.T_a_c * calib.T_i_c[0].inverse();

    //     if (!g_initialized) {
    //         for (size_t i = 0;
    //             i < vio_dataset->get_accel_data().size() && !g_initialized; i++) {
    //         const basalt::AccelData& ad = vio_dataset->get_accel_data()[i];
    //         if (std::abs(ad.timestamp_ns - timestamp_ns) < 3000000) {
    //             g_a_init = T_a_i.so3() * ad.data;
    //             g_initialized = true;
    //             std::cout << "g_a initialized with " << g_a_init.transpose()
    //                     << std::endl;
    //         }
    //         }
    //     }
    //     }
    // }

    calib_spline.setG(g_a_init);
    int num_gyro = 0;
    int num_accel = 0;
    int num_corner = 0;
    int num_frames = 0;

    for (const auto& v : *imu_data) {
        if (v.timestamp * 1e9 >= start_t_ns && v.timestamp * 1e9 < end_t_ns) {
        calib_spline.addGyroMeasurement(v.gyro, v.timestamp * 1e9);
        num_gyro++;
        }
    }
    for (const auto& v : *imu_data) {
        if (v.timestamp * 1e9 >= start_t_ns && v.timestamp * 1e9 < end_t_ns) {
        calib_spline.addAccelMeasurement(v.accel, v.timestamp * 1e9);
        num_accel++;
        }
    }
    ceres::Solver::Summary summary = calib_spline.optimize();
    
    std::cout << "num_gyro " << num_gyro << " num_accel " << num_accel
                << " num_corner " << num_corner << " num_frames " << num_frames
                << " duration " << (end_t_ns - start_t_ns) * 1e-9 << std::endl;

    std::cout << "g: " << calib_spline.getG().transpose() << std::endl;
    std::cout << "accel_bias: " << calib_spline.getAccelBias().transpose()
                << std::endl;
    std::cout << "gyro_bias: " << calib_spline.getGyroBias().transpose()
                << std::endl;
    // for (size_t i = 0; i < calib_spline.getCalib().T_i_c.size(); i++) {
    //     std::cout << "T_i_c" << i << ":\n"
    //             << calib_spline.getCalib().T_i_c[i].matrix() << std::endl;
    // }

    for (int64_t t_ns = start_t_ns; t_ns < end_t_ns; t_ns += 1e6) {
        Eigen::Vector3d gyro, accel;

        gyro = calib_spline.getGyro(t_ns);
        accel = calib_spline.getAccel(t_ns);

        std::cout << t_ns << "," << gyro[0] << "," << gyro[1] << "," << gyro[2] << ","
        << accel[0] << "," << accel[1] << "," << accel[2] << std::endl;
    }
    std::cout << summary.FullReport() << std::endl;
    // std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>> trajectry;
    // for (double t = 0; t < end_t_ns * 1e9; t += 0.5){
    //     Sophus::SO3d rot = spline.so3_spline->evaluate(t * 1e9);
    //     Eigen::Vector3d pos = spline.rd_spline->evaluate<0>(t * 1e9);
    //     Eigen::Isometry3d tra(rot.unit_quaternion());
    //     tra.pretranslate(pos);
    //     trajectry.push_back(tra);
    // }
}

}
