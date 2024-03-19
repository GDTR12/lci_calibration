#pragma once




#include <rclcpp/rclcpp.hpp>
#include <utils/input.hpp>
#include <lci_cali/continuous_se3.hpp>
#include <lci_cali/spline/lidar_spline.hpp>
#include <lci_cali/spline/imu_spline.hpp>
#include "sensor_data/lidar_data.h"
#include "pcl/visualization/cloud_viewer.h"
#include "utils/ndt_utils.hpp"
#include "utils/ui_utils.hpp"
#include "utils/math_utils.hpp"
#include "sensor_data/vlp16_data.hpp"
#include "utils/plane_map.hpp"



namespace lci_cali{

using namespace sensor_data;
using namespace slam_utils;



class LCICali
{
private:
    using PointT = PointIRT;
    using PointMap = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointCloudMap = pcl::PointCloud<PointMap>;

    using NdtOdometry = std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>;
    std::shared_ptr<std::vector<LidarData<PointT>>> lidar_data;
    std::vector<LidarData<PointMap>> undistorted_lidar;
    std::shared_ptr<std::vector<IMUData, Eigen::aligned_allocator<IMUData>>> imu_data;
    pcl::PointCloud<pcl::PointXYZ> map; 
    std::shared_ptr<VLP16Input::Input> input;
    // std::shared_ptr<slam_utils::Input<sensor_data::LidarData<PointIRT>, PointCloudIRT>> input;
    double time_end = INFINITY;
    double start_time = 0;
    static const int N = 4;
    const double interval_s = 0.01;

    // 待优化的变量:
    // Eigen::Vector3d bias_a = Eigen::Vector3d(9.63036e-05,-0.000233096,0.00166143);
    // Eigen::Vector3d bias_g = Eigen::Vector3d(0.000519598,0.000937668,0.00124568);
    // Eigen::Vector3d t_ItoL = Eigen::Vector3d(0.31208,0.12331,0.0595447);
    // Eigen::Vector3d g = Eigen::Vector3d(-0.444461, -4.09903, 8.68358);
    Eigen::Vector3d bias_a = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d bias_g = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d t_ItoL = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d g = Eigen::Vector3d(0,0,0);
    Eigen::Quaterniond q_ItoL = Eigen::Quaterniond::Identity();
    std::shared_ptr<IMUSpline<N>> spline_imu;
    std::shared_ptr<LidarSpline<N>> spline_lidar;
public:

    ~LCICali(){}
    LCICali(){
        // if(CFG_GET_INT("init_which_data") == 1){
            input.reset(VLP16Input::createInstance());
        // }else if(CFG_GET_INT("init_which_data") == 2){
            // input.reset(slam_utils::Input<sensor_data::LidarData<PointIRT>, PointCloudIRT>::createInstance());
        // }
        lidar_data = input->getLidarData();        
        imu_data = input->getIMUData();
        time_end = input->getEndTime();
        start_time = input->getStartTime();
        spline_imu = std::make_shared<IMUSpline<N>>(start_time, time_end, interval_s);
        spline_lidar = std::make_shared<LidarSpline<N>>(start_time, time_end, interval_s);
    }

    void run(){
        initExtrinsicParam();
        // q_ItoL = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
        auto map  = pcl::make_shared<PointCloudMap>();
        auto filtered_map = pcl::make_shared<PointCloudMap>();
        int epoch = 0;
        std::cout << "bias_a: " << bias_a.transpose() << " bias_g: " << bias_g.transpose() << " t: " << t_ItoL.transpose() << std::endl;
        for(int i = 0; i < CFG_GET_INT("refinement_epoch"); i++){
            slam_utils::PlaneMap<PointMap> surfelmaps;
            std::cout << "=======================================" << std::endl;
            std::cout << "epoch: " << epoch << std::endl;
            if(epoch == 0){
                filtered_map = initRebuildMap();
            }else{
                map = pcl::make_shared<PointCloudMap>();
                filtered_map->clear();
                lidarDataUndistort(spline_lidar);
                for (size_t j = 0; j < undistorted_lidar.size(); j++)
                {
                    auto frame = pcl::make_shared<PointCloudMap>();
                    Sophus::SO3d so3;
                    Eigen::Vector3d rd3;
                    if(!spline_lidar->getSO3(undistorted_lidar.at(j).timestamp, so3) || !spline_lidar->getRd3(undistorted_lidar.at(j).timestamp, rd3))
                        continue;
                    Sophus::SE3d se3(so3, rd3);
                    pcl::transformPointCloud(*undistorted_lidar.at(j).data, *frame, se3.matrix().cast<float>());
                    // pcl::VoxelGrid<PointMap> filter;
                    // double map_res = CFG_GET_FLOAT("refine_map_resolution");
                    // filter.setLeafSize(map_res, map_res, map_res);
                    // filter.setInputCloud(frame);
                    // filter.filter(*map);
                    *filtered_map += *frame;
                    // *map += *frame;
                }
                // NdtOdometry ndt_odometry;
                // std::vector<double> ndt_timestamp;
                // lidarDataUndistort(spline_lidar);
                // filtered_map = ndtOdometry(undistorted_lidar, ndt_odometry, ndt_timestamp, false,false);
            }
            
            std::cout << "show epoch map:" << std::endl;
            showPcl(*filtered_map);

            ndtSurferMap(filtered_map, surfelmaps); 
            addFactor(surfelmaps, filtered_map);

            // if(epoch == 0 && CFG_GET_BOOL("refine_use_imu")){
            //     // spline_lidar->lockParam(q_ItoL.coeffs().data());
                // spline_lidar->lockParam(bias_a.data());
                // spline_lidar->lockParam(bias_g.data());
            //     spline_lidar->lockKnots();
            // }
            optimize();
            epoch++;
        }
    }


    // pcl::shared_ptr<PointCloudMap> ndtOdometryWithIMU(NdtOdometry& odometry, std::vector<double>& time_list)
    // {
    //     NdtOmpUtils ndt_odometry(CFG_GET_FLOAT("ndt_resolution"), 
    //                              CFG_GET_FLOAT("ndt_step_size"), 
    //                              CFG_GET_FLOAT("ndt_epsilon"), 
    //                              CFG_GET_INT("ndt_max_iters"), 
    //                              CFG_GET_FLOAT("ndt_filter_size"));   
    //     pcl::shared_ptr<PointCloudMap> map = pcl::make_shared<PointCloudMap>();
    //     Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    //     odometry.push_back(transform);
    //     time_list.push_back(lidar_data.front().timestamp);
        
    //     // Eigen::Quaterniond q_L1toL2 = q_ItoL.inverse() * 

    // }
    // TODO:将其重构到 ndtutil
    template<typename PointT>
    pcl::shared_ptr<PointCloudMap> ndtOdometry(
                            std::vector<LidarData<PointT>>& lidar_data,
                            NdtOdometry& odometry, 
                            std::vector<double>& time_list, 
                            bool use_raw,
                            bool use_imu_spline){
        NdtOmpUtils ndt_odometry(CFG_GET_FLOAT("ndt_resolution"), 
                                 CFG_GET_FLOAT("ndt_step_size"), 
                                 CFG_GET_FLOAT("ndt_epsilon"), 
                                 CFG_GET_INT("ndt_max_iters"), 
                                 CFG_GET_FLOAT("ndt_filter_size"));
        pcl::shared_ptr<PointCloudMap> map = pcl::make_shared<PointCloudMap>();
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        odometry.push_back(transform);
        time_list.push_back(lidar_data.front().timestamp);
        Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
        for(int i = CFG_GET_INT("init_start_iter"); i < lidar_data.size() - 1; i++){

            pcl::shared_ptr<PointCloudMap> pcl_now, pcl_next;
            pcl_now = pcl::make_shared<PointCloudMap>();
            pcl_next = pcl::make_shared<PointCloudMap>();

            pcl::VoxelGrid<PointMap> filter;
            filter.setLeafSize(0.1, 0.1, 0.1);
            if (use_raw){
                pcl::fromROSMsg(*lidar_data.at(i).raw_data, *pcl_now);
                pcl::fromROSMsg(*lidar_data.at(i + 1).raw_data, *pcl_next);
            }else{
                slam_utils::pclCopyToXYZ(*lidar_data.at(i).data, *pcl_now);
                slam_utils::pclCopyToXYZ(*lidar_data.at(i+1).data, *pcl_next);
            }
            double cost_ms= 0;

            if(use_imu_spline){
                Sophus::SE3d se3_0, se3_1;
                Sophus::SE3d se3_ItoL(q_ItoL, t_ItoL);
                if (spline_imu->getSE3(lidar_data.at(i).timestamp, se3_0) &&
                    spline_imu->getSE3(lidar_data.at(i+1).timestamp, se3_1)){
                    auto se3_0to1 = se3_0.inverse() * se3_1;
                    auto R_L0toL1 = se3_ItoL.inverse() * se3_0to1 * se3_ItoL;
                    result = R_L0toL1.matrix().cast<float>();
                }
            }

            auto next = pcl::make_shared<PointCloudMap>();
            if(i == CFG_GET_INT("init_start_iter")){
                filter.setInputCloud(pcl_now);
                filter.filter(*next);
                *map += *next;
                continue;
            }
            // cost_ms = ndt_odometry.align(pcl_next, pcl_now, result);
            // transform = transform * result;
            cost_ms = ndt_odometry.align(pcl_next, map, result);
            transform = result;
            odometry.push_back(transform);
            time_list.push_back(lidar_data.at(i+1).timestamp);
            
            // PointCloudMap filtered_map;

            filter.setInputCloud(pcl_next);
            filter.filter(*next);
            auto next_transformed = pcl::make_shared<PointCloudMap>();
            pcl::transformPointCloud(*next, *next_transformed, transform);

            (*map) += (*next_transformed);
            if(CFG_GET_BOOL("init_show_process")){
                showPcl(*pcl_now, *pcl_next, *next_transformed);
            }
            if(CFG_GET_BOOL("init_show_every_map"))showPcl(*map);
            if(i == CFG_GET_INT("init_end_iter"))break;
        }
        return map;
    }


    void drawSO3Trajectry(const NdtOdometry& odometry){
        Trajectory ndt_trajectory;
        for(int i = 0; i < odometry.size() - 1; i++){
            auto& transform0 = odometry[i];
            auto& transform1 = odometry[i + 1];

            Sophus::SO3f so30(transform0.block<3,3>(0,0));
            Sophus::SO3f so31(transform1.block<3,3>(0,0));

            Eigen::Isometry3d tra(Eigen::Matrix3d::Identity());
            tra.pretranslate(so30.unit_quaternion().vec().cast<double>());
            ndt_trajectory.push_back(tra);

            float distance = MathUtils::getSO3Distance(so30, so31);
            RCLCPP_DEBUG(rclcpp::get_logger("cali"), "Distance of two so3: %f", distance);
        }
        DrawTrajectory(ndt_trajectory, "ndt so3 odometry");
    }

    void drawNdtTrajectory(const NdtOdometry& odometry)
    {
        Trajectory ndt_trajectory;
        for(int i = 0; i < odometry.size() - 1; i++){
            auto& transform0 = odometry[i];
            auto& transform1 = odometry[i + 1];

            Sophus::SO3f so30(transform0.block<3,3>(0,0));
            Sophus::SO3f so31(transform1.block<3,3>(0,0));

            Eigen::Isometry3d tra(so30.matrix().cast<double>());
            tra.pretranslate(transform0.block<3,1>(0,3).cast<double>());
            ndt_trajectory.push_back(tra);

            float distance = MathUtils::getSO3Distance(so30, so31);
            RCLCPP_DEBUG(rclcpp::get_logger("cali"), "Distance of two so3: %f", distance);
        }
        DrawTrajectory(ndt_trajectory, "ndt odometry");
    }


    bool alignQuantation(NdtOdometry& odometry, std::vector<double>& time_list){
        Eigen::MatrixXd A;
        A.resize((time_list.size() - 1) * 4, 4);
        for(int i = 0; i < time_list.size() - 1; i++)
        {
            Eigen::Quaterniond qua(odometry[i].block<3,3>(0,0).cast<double>().transpose() * odometry[i + 1].block<3,3>(0,0).cast<double>());
            Sophus::SO3d so3_0, so3_1;
            // std::cout <<time_list[i] << " " <<time_list[i+1] << std::endl;
            if(spline_imu->getSO3(time_list[i], so3_0) && spline_imu->getSO3(time_list[i+1], so3_1)){
                auto so3 = Sophus::SO3d(so3_0.matrix().transpose() * so3_1.matrix());
                A.block<4, 4>(i * 4, 0) = MathUtils::quaLeftMultiMat(so3.unit_quaternion()) - MathUtils::quaRightMultiMat(qua);
            }
            if(i < 21 || i % 5 != 0) continue;
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(A.block(0,0, (i+1)*4, 4), Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Vector4d result  = svd.matrixV().col(3); // w,x,y,z
            Eigen::Vector4d cov = svd.singularValues();
            std::cout << "cov: " << cov.transpose() << std::endl;
            if(cov(2) > CFG_GET_FLOAT("init_svd_cov")){
                
                // Eigen用向量初始化四元数时,向量里的循序是(x,y,z,w)
                q_ItoL = Eigen::Quaterniond(Eigen::Vector4d(result[1], result[2], result[3], result[0]));
                std::cout << "q (w,x,y,z):" << q_ItoL.w() << " " << q_ItoL.vec().transpose() << std::endl;
                std::cout << "svd v:" << svd.matrixV() << std::endl;
                std::cout << "feature value:" << svd.singularValues().transpose() << std::endl;
                std::cout << "result " << result.transpose() << std::endl;
                return true;
            }else{
                continue;
            }
        }
        // std::cout << A << std::endl;
        return false; 
    }

    void initSplineRotate(){
        for(auto& data : *imu_data){
            spline_imu->addIMUGyroMeasurement(data.timestamp, data.gyro, false);
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.num_threads = std::thread::hardware_concurrency();
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        ceres::Solver::Summary summary = spline_imu->solve(options);
        if(CFG_GET_BOOL("init_show_spline_imu_info")){
            std::cout << summary.FullReport() << std::endl;
        }
    }

    // void splineAligeWithCeres(NdtOdometry& odometry, std::vector<double>& time_list)
    // {
    //     auto ndt_trajectory = std::make_shared<Trajectory>();
    //     auto spline_trajectory = std::make_shared<Trajectory>();
    //     for (size_t i = 0; i < odometry.size(); i++)
    //     {
    //         Sophus::SO3d so3;
    //         if(!spline_imu->getSO3(time_list[i], so3)) continue;
    //         // spline_imu->addIMUGyroMeasurement(time_list[i], )
    //     }
        
    // }


    void splineAlign(NdtOdometry& odometry){
        auto ndt_trajectory = std::make_shared<Trajectory>();
        auto spline_trajectory = std::make_shared<Trajectory>();
        for(int i = 0; i < odometry.size() - 1; i++){
            Sophus::SO3f so3(odometry[i].block<3,3>(0,0));
            auto transd = q_ItoL * so3.unit_quaternion().cast<double>();
            // auto transd = so3.unit_quaternion().cast<double>();
            Eigen::Isometry3d tra(Eigen::Quaterniond::Identity());
            tra.pretranslate(transd.vec());
            ndt_trajectory->push_back(tra);
        }
        for(double t = spline_imu->getStartTime_s(); t < spline_imu->getEndTime_s(); t += spline_imu->getInterval_s()){
            Sophus::SO3d rot;
            if(!spline_imu->getSO3(t, rot)){
                continue;
            }
            Eigen::Isometry3d tra(Eigen::Quaterniond::Identity());
            auto transd = rot.unit_quaternion() * q_ItoL;
            // auto transd = rot.unit_quaternion();
            tra.pretranslate(transd.vec());
            spline_trajectory->push_back(tra);
        }

        std::vector<TrajectoryPtr> tras;
        tras.push_back(ndt_trajectory);
        tras.push_back(spline_trajectory);
        DrawTrajectories(tras, "Aligned tra");
    }

    void initExtrinsicParam(){
        NdtOdometry ndt_odometry;
        std::vector<double> ndt_timestamp;

        auto map = ndtOdometry(*lidar_data,ndt_odometry, ndt_timestamp, false, false);
        if(CFG_GET_BOOL("init_show_map"))showPcl(*map);

        if(CFG_GET_BOOL("init_show_ndt_traj")) {
            drawSO3Trajectry(ndt_odometry);
            drawNdtTrajectory(ndt_odometry);
        }
        initSplineRotate();
        if(CFG_GET_BOOL("init_show_spline_traj")) spline_imu->drawSO3(spline_imu->getInterval_s());
        if(alignQuantation(ndt_odometry, ndt_timestamp)){
            if(CFG_GET_BOOL("init_show_aligned_tra"))splineAlign(ndt_odometry);
            if(CFG_GET_BOOL("init_print_align_info")){
                for(int i =0; i< ndt_odometry.size(); i++){
                    Sophus::SO3f so3_ndt(ndt_odometry[i].block<3,3>(0,0));
                    Sophus::SO3d so3_imu;
                    if(!spline_imu->getSO3(ndt_timestamp[i], so3_imu)){
                        continue;
                    }
                    auto qua0 = q_ItoL * so3_ndt.unit_quaternion().cast<double>();
                    auto qua1 = so3_imu.unit_quaternion() * q_ItoL;
                    auto res = MathUtils::QuaSubtraction(qua0, qua1);
                    std::cout << "residual:" << res.transpose() << std::endl;
                }
            }
        }else{
            std::cout << "align failed" << std::endl;
        }
        initLidarSpline(ndt_odometry, ndt_timestamp);
    }

    pcl::shared_ptr<PointCloudMap> initRebuildMap(){
        NdtOdometry ndt_odometry;
        std::vector<double> ndt_timestamp;
        lidarDataUndistort(spline_imu);
        // NdtOmpUtils utils(0.4, 0.1, 0.01, 50, 1);
        // pcl::shared_ptr<PointCloudMap> map = pcl::make_shared<PointCloudMap>();
        // spline_lidar->resetProblem();
        // for(int i = 0; i < lidar_data->size(); i++){
        //     auto& data = undistorted_lidar.at(i).data;
        //     auto& data_next = lidar_data->at(i).data;
        //     PlaneMap<PointMap> surfelmaps;
        //     utils.ndtSurferMap(surfelmaps, data, 0.6, 20, 10);
        //     pcl::VoxelGrid<PointT> filter;
        //     PointCloudT filtered;
        //     auto size = CFG_GET_FLOAT("refine_lidar_filter_size");
        //     filter.setLeafSize(size, size, size);
        //     filter.setInputCloud(data_next);
        //     filter.filter(filtered);
        //     for(auto& p : filtered)
        //     {
        //         Eigen::Vector3f p_vec;
        //         spline_lidar->pointTransform<float>(
        //             Eigen::Map<Eigen::Vector3f>(&p.x), p.time, 
        //             p_vec, lidar_data->at(i).timestamp);

        //         PointMap pos(p_vec.x(), p_vec.y(), p_vec.z());
        //         spline_lidar->addLidarPlannarMeasurement(
        //             p.time, pos,surfelmaps);
        //     }
        // }
        // // ceres::opt
        // ceres::Solver::Options options;
        // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        // options.num_threads = std::thread::hardware_concurrency();
        // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        // ceres::Solver::Summary summary = spline_lidar->solve(options);
        // for (size_t i = 0; i < undistorted_lidar.size(); i++)
        // {
        //     Sophus::SE3d se3;
        //     auto frame = pcl::make_shared<PointCloudMap>();
        //     if(!spline_lidar->getSE3(lidar_data->at(i).timestamp, se3))continue;
        //     pcl::transformPointCloud(*undistorted_lidar.at(i).data, *frame, se3.matrix());
        //     (*map) += (*frame);
        // }
        
        auto map = ndtOdometry(undistorted_lidar, ndt_odometry, ndt_timestamp, false, false);
        // *map += *undistorted_lidar.front().data;
        return map;
    }
    template<typename Scalar>
    void se3Insert(Sophus::SE3<Scalar>& SE3_0,
                   Sophus::SE3<Scalar>& SE3_1, 
                   int nums, std::vector<Eigen::Matrix<Scalar,7,1>>& rets)
    {
        // Eigen::Matrix<Scalar, 6, 1> se3_0 = SE3_0.log();
        // se3_0.template block<3,1>(0,0) = SE3_0.translation();
        // Eigen::Matrix<Scalar, 6, 1> se3_1 = SE3_1.log();
        // se3_1.template block<3,1>(0,0) = SE3_1.translation();
        Eigen::Matrix<Scalar, 7, 1> se3_0;
        Eigen::Matrix<Scalar, 7, 1> se3_1;
        se3_0.template block<3,1>(0,0) = SE3_0.translation();
        se3_1.template block<3,1>(0,0) = SE3_1.translation();
        se3_0.template block<4,1>(3,0) = SE3_0.unit_quaternion().coeffs().matrix();
        se3_1.template block<4,1>(3,0) = SE3_1.unit_quaternion().coeffs().matrix();
        Eigen::Matrix<Scalar, 7, 1> delta = (se3_1 - se3_0) / (nums + 1);
        rets.push_back(se3_0);
        
        for (size_t i = 1; i < nums + 1; i++)
        {
            rets.push_back(se3_0 + delta * i);
        }
        rets.push_back(se3_1);
    }

    void initLidarSpline(NdtOdometry& odometry, std::vector<double>& time_list){
        // 用雷达odometry初始化点云
        for(int i = 0; i < odometry.size() - 1; i++){
            
            auto data_0 = Sophus::SE3f(odometry.at(i));
            auto data_1 = Sophus::SE3f(odometry.at(i+1));
            std::vector<Eigen::Matrix<float, 7, 1>> se3_s;
            int num = (time_list[i+1] - time_list[i]) * CFG_GET_FLOAT("tmp_interval")/ interval_s;
            double delta_t = (time_list[i+1] - time_list[i]) / (num + 1);
            
            se3Insert(data_0, data_1, num, se3_s);
            for(int j = 0; j < se3_s.size() - 1; j++)
            {
                double t = time_list[i] + delta_t * j;
                // std::cout << "j: " <<  j << " so3:" << se3_s.at(j).tail(4).transpose() << " " << std::endl;
                // std::cout << "length: " << se3_s.at(j).tail(4).norm() << std::endl;
                // std::cout << "distance: " << (se3_s.at(j).tail(4) - se3_s.at(j+1).tail(4)).norm() << std::endl;
                Eigen::Vector4d data = se3_s.at(j).tail(4).cast<double>();
                Sophus::SO3d data_so3(Eigen::Quaterniond(data.data()));
                spline_lidar->addLidarOrientationMeasurement(t, data_so3);
                spline_lidar->addLidarPositionMeasurement(t, se3_s.at(j).head(3).cast<double>());
            }

            // std::cout << std::endl;
            // spline_lidar->addLidarOrientationMeasurement(time_list[i], so3.log().cast<double>());
            // spline_lidar->addLidarPositionMeasurement(time_list[i], vec.cast<double>());
        }
        

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        // options.num_threads = std::thread::hardware_concurrency();
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        ceres::Solver::Summary summary = spline_lidar->solve(options);
        if(CFG_GET_BOOL("init_show_spline_lidar_info")){
            std::cout << "lidar_spline initialized result: " << std::endl;
            std::cout << summary.FullReport() << std::endl;
        }

        if(CFG_GET_BOOL("init_show_spline_lidar")){
            spline_lidar->drawTrjectory(spline_lidar->getInterval_s());
            for(int i = 0; i < odometry.size() - 1; i++){
                Sophus::SO3d so3;
                Sophus::SO3f rot(odometry.at(i).block<3,3>(0,0));
                Eigen::Vector3d trans;
                Eigen::Vector3f pos(odometry.at(i).block<3,1>(0,3));
                spline_lidar->getSO3(time_list[i], so3);
                spline_lidar->getRd3(time_list[i], trans);
                std::cout << (so3.log().cast<float>() - rot.log()).transpose() << std::endl;
                std::cout << (trans.cast<float>() - pos).transpose() << std::endl;
            }
        }

    }

    void ndtSurferMap(pcl::shared_ptr<PointCloudMap> map, slam_utils::PlaneMap<PointMap>& surfelmaps)
    {
        // std::vector<std::pair<typename PointCloudMap::Ptr, Eigen::Matrix<double, 4, 1>>> surfelmaps;
        NdtOmpUtils ndt(
            CFG_GET_FLOAT("refine_cell_size"),
            CFG_GET_FLOAT("ndt_step_size"), 
            CFG_GET_FLOAT("ndt_epsilon"), 
            CFG_GET_INT("ndt_max_iters"), 
            CFG_GET_FLOAT("ndt_filter_size"));
        // ndt.ndtSurferMap(surfelmaps, map, 0.6);
        // if(CFG_GET_BOOL("refine_show_surfels")){
        //     std::vector<typename PointCloudMap::Ptr> maps;
        //     for(int i = 0; i < surfelmaps.size(); i++)
        //     {
        //         auto& [map, surfel] = surfelmaps.at(i);
        //         maps.push_back(map);
        //     }
        //     showPcl(maps);
        // }
        
        ndt.ndtSurferMap(surfelmaps, map, 0.6, CFG_GET_INT("refine_plane_cellMinPoints"), CFG_GET_INT("refine_plane_planarMinPoints"));
        if(CFG_GET_BOOL("refine_show_surfels")){
            std::vector<typename PointCloudMap::Ptr> maps;
            for(int i = 0; i < surfelmaps.pcls.size(); i++)
            {
                auto& map = surfelmaps.pcls[i];
                maps.push_back(map);
            }
            showPcl(maps);
            // surfelmaps.showPlanes();
        }
        
    }
    

    void lidarDataUndistort(std::shared_ptr<BSplineSE3<N>> spline)
    {
        undistorted_lidar.clear();
        undistorted_lidar.resize(lidar_data->size());

        for(int i = 0; i < lidar_data->size(); i++){
            const auto& scan = lidar_data->at(i);
            auto& pcl = undistorted_lidar.at(i).data;
            undistorted_lidar.at(i).timestamp = lidar_data->at(i).timestamp;

            pcl.reset(new pcl::PointCloud<PointMap>());
            pcl->width = scan.data->width;
            pcl->height = scan.data->height;
            pcl->resize(pcl->width * pcl->height);
            for(int w = 0; w < scan.data->width; w++){
                for(int h = 0; h < scan.data->height; h++)
                {
                    // const auto& p_j = scan.data->at(w, h);

                    auto ret = [&]()->PointT&{
                        if(scan.data->height > 1) {
                            return scan.data->at(w, h);
                        }else{
                            return scan.data->at(h * scan.data->width +  w);
                        }
                    };
                    const PointT& p_j = ret();
                    const Eigen::Vector3f pos(p_j.x, p_j.y, p_j.z);
                    Sophus::SO3d rot_j, rot_i;
                    Eigen::Vector3d trans_j, trans_i;
                    if (!spline->getSO3(p_j.time, rot_j) || !spline->getRd3(p_j.time, trans_j) || 
                        !spline->getSO3(scan.timestamp, rot_i) || !spline->getRd3(scan.timestamp, trans_i)){
                        continue;
                    }
                    // auto rot_itoj = Sophus::SO3d(Eigen::Matrix3d::Identity());
                    // auto trans_itoj = Eigen::Vector3d::Zero();
                    auto rot_itoj = rot_i.inverse() * rot_j;
                    auto trans_itoj = rot_i.inverse().matrix() * (trans_j - trans_i);
                    PointMap new_p;
                    pcl::copyPoint(p_j, new_p);
                    Eigen::Map<Eigen::Vector3f> position(&new_p.x);
                    position = rot_itoj.cast<float>().matrix() * pos + trans_itoj.cast<float>();
                
                    if(!pcl_isfinite(new_p.x) || !pcl_isfinite(new_p.y) || !pcl_isfinite(new_p.z))continue;
                    pcl->at(w, h) = new_p;                   
                }

            }
            if(CFG_GET_BOOL("refine_show_undistort"))showPcl(*scan.data, *pcl);
        }
    }

    void addFactor(slam_utils::PlaneMap<PointMap>& surfelmaps, typename PointCloudMap::Ptr map)
    {
        spline_lidar->resetProblem();
        if(CFG_GET_BOOL("refine_use_plannar")){
            for(int i = 0; i < lidar_data->size(); i++){
                
                std::vector<typename PointCloudMap::Ptr> color_maps;
                std::vector<float> map_size;
                color_maps.resize(surfelmaps.pcls.size());
                std::vector<std::array<int, 3>> color;
                for (size_t j = 0; j < surfelmaps.pcls.size(); j++)
                {
                    color.push_back({(int)(255.0f * rand() / RAND_MAX), 
                                    (int)(255.0f * rand() / RAND_MAX), 
                                    (int)(255.0f * rand() / RAND_MAX)});
                    color_maps.at(j).reset(new PointCloudMap());
                    color_maps.at(j)->push_back(surfelmaps.center_points.at(j));
                    map_size.push_back(2);
                }
                map_size.push_back(1);
                color.push_back({100,100,100});
                

                auto& data = lidar_data->at(i).data;
                if(CFG_GET_BOOL("refine_lidar_factor_filter")){
                    // pcl::VoxelGrid<PointT> filter;
                    // PointCloudT filtered;
                    // auto size = CFG_GET_FLOAT("refine_lidar_filter_size");
                    // filter.setLeafSize(size, size, size);
                    // filter.setInputCloud(data);
                    // filter.filter(filtered);


                    for(auto& p : *data)
                    {
                        PointMap pos(p.x, p.y, p.z);
                        // spline_lidar->addLidarPlannarMeasurement(
                        //     p.time, pos,surfelmaps);
                        Sophus::SO3d rot;
                        Eigen::Vector3d trans;
                        spline_lidar->getSO3(p.time, rot);
                        spline_lidar->getRd3(p.time, trans);
                        // std::cout << "rot: " <<  rot.matrix() << std::endl;
                        // std::cout << "time: " << p.time << std::endl;
                        // std::cout << "rd3: " <<  trans.transpose() << std::endl;
                        PointMap point;
                        Eigen::Map<Eigen::Vector3f> p_dst(&point.x);
                        Eigen::Map<Eigen::Vector3f> p_src(&p.x);
                        p_dst = rot.matrix().cast<float>() * p_src + trans.cast<float>();
                        Eigen::Vector4f plane_coffe;
                        int index;
                        if(!pcl_isfinite(point.x) || !pcl_isfinite(point.y) || !pcl_isfinite(point.z))continue;
                        if(!surfelmaps.serachNearestPlane(point, surfelmaps.resolution * 10, plane_coffe, index)){
                            continue;
                        }else if(plane_coffe.block<3,1>(0,0).dot(p_dst) + plane_coffe(3) > 10){
                            continue;
                        }
                        // std::cout << plane_coffe.transpose() << std::endl;
                        color_maps.at(index)->push_back(point);
                    }
                    color_maps.push_back(map);

                    showPcl<PointMap>(color_maps, color, map_size,"aaa");
                }else{
                    for(auto&p : *data){
                        PointMap pos(p.x, p.y, p.z);
                        spline_lidar->addLidarPlannarMeasurement(
                            p.time, pos,surfelmaps);
                    }
                }
            }
        }

        if(CFG_GET_BOOL("refine_use_imu")){
            for(int i = 0; i < imu_data->size(); i++)
            {
                auto& data = imu_data->at(i);
                spline_lidar->addIMUGyroMeasurement(data.timestamp, data.gyro, bias_g, q_ItoL);
                spline_lidar->addIMUAccelMeasurement(data.timestamp, data.accel, g, bias_a, q_ItoL, t_ItoL);
            }
        }
        spline_lidar->printInfo();

        
    }

    // void addImuSplineFactor(slam_utils::PlaneMap<PointMap>& surfelmaps)
    // {
    //     spline_imu->resetProblem();
    //     if(CFG_GET_BOOL("refine_use_plannar")){
    //         for(int i = 0; i < lidar_data->size(); i++){
    //             auto& data = lidar_data->at(i).data;
    //             if(CFG_GET_BOOL("refine_lidar_factor_filter")){
    //                 pcl::VoxelGrid<PointT> filter;
    //                 PointCloudT filtered;
    //                 auto size = CFG_GET_FLOAT("refine_lidar_filter_size");
    //                 filter.setLeafSize(size, size, size);
    //                 filter.setInputCloud(data);
    //                 filter.filter(filtered);
    //                 for(auto& p : filtered)
    //                 {
    //                     Eigen::Vector3f p_vec;
    //                     if(!spline_imu->pointTransform<float>(
    //                         Eigen::Map<Eigen::Vector3f>(&p.x), p.time, 
    //                         p_vec, spline_imu->getStartTime_s()))continue;
    //                     PointMap pos(p_vec.x(), p_vec.y(), p_vec.z());
    //                     spline_imu->addLidarPlannarMeasurement(
    //                         p.time, pos,surfelmaps);
    //                 }
    //             }else{
    //                 for(auto&p : *data){
    //                     Eigen::Vector3f p_vec;
    //                     if(!spline_imu->pointTransform<float>(
    //                         Eigen::Map<Eigen::Vector3f>(&p.x), p.time, 
    //                         p_vec, spline_imu->getStartTime_s()))continue;
    //                     PointMap pos(p_vec.x(), p_vec.y(), p_vec.z());
    //                     spline_imu->addLidarPlannarMeasurement(
    //                         p.time, pos,surfelmaps);
    //                 }
    //             }
    //         }
    //     }
    //     if(CFG_GET_BOOL("refine_use_imu")){
    //         for(int i = 0; i < imu_data->size(); i++)
    //         {
    //             auto& data = imu_data->at(i);
    //             spline_imu->addIMUGyroMeasurement(data.timestamp, data.gyro, bias_g, q_ItoL);
    //             spline_imu->addIMUAccelMeasurement(data.timestamp, data.accel, g, bias_a, q_ItoL, t_ItoL);
    //         }
    //     }
    //     spline_imu->printInfo();      
    // }

    void optimize()
    {
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.num_threads = CFG_GET_INT("refine_solve_num_threads");
        options.minimizer_progress_to_stdout = true;
        options.use_nonmonotonic_steps = true;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.max_num_iterations = CFG_GET_INT("refine_ceres_max_iters");
        ceres::Solver::Summary summary = spline_lidar->solve(options);
        if(CFG_GET_BOOL("refine_show_spline_solve_info")){
            std::cout << summary.FullReport() << std::endl;
            std::cout << "qua:" << q_ItoL.coeffs().transpose() << std::endl;
            std::cout << "trans:" << t_ItoL.transpose() << std::endl;
            std::cout << "g:" << g.transpose() << std::endl;
            std::cout << "bias_a:" << bias_a.transpose() << std::endl;
            std::cout << "bias_g:" << bias_g.transpose() << std::endl;

            spline_lidar->drawSO3(interval_s);    
            spline_lidar->drawTrjectory(interval_s);
        }
    }

};

}