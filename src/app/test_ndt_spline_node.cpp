#include "rclcpp/rclcpp.hpp"
#include "pcl/point_cloud.h"
#include "utils/input.hpp"
#include "lci_cali/continuous_se3.hpp"
#include "utils/config_yaml.h"
#include "utils/ndt_utils.hpp"
#include "pcl/visualization/cloud_viewer.h"
#include "utils/ui_utils.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    using PointT = pcl::PointXYZ;
    // std::shared_ptr<slam_utils::Input<sensor_data::LidarData<PointT>>> input(slam_utils::Input<sensor_data::LidarData<PointT>>::createInstance());
    std::shared_ptr<sensor_data::VLP16Input::Input> input(sensor_data::VLP16Input::createInstance());
    auto imu_data = *(input->getIMUData());
    auto lidar_data = *(input->getLidarData());
    double time_end = input->getEndTime();
    const int degree = 4;
    double time_interval = CFG_GET_FLOAT("interval_time");
    lci_cali::BSplineSE3<4> spline(0, time_end, time_interval);
    auto trajectry = std::make_shared<slam_utils::Trajectory>();
    auto trajectry1 = std::make_shared<slam_utils::Trajectory>();
    if(CFG_GET_BOOL("use_sim")){
        for(double i = 0.005; i < time_end; i +=  0.01){
            sensor_data::IMUData data;
            data.timestamp = i;
            data.accel << CFG_GET_FLOAT("acc0"), CFG_GET_FLOAT("acc1"), CFG_GET_FLOAT("acc2");
            data.gyro <<CFG_GET_FLOAT("gyro0"), CFG_GET_FLOAT("gyro1"), CFG_GET_FLOAT("gyro2");;
            if(CFG_GET_BOOL("use_gyro")){
                spline.addIMUGyroMeasurement(data.timestamp, data.gyro, false);
            }else{
                spline.AddImuMeasurement(data);
            }
        }
    }else{
        for(auto& data : imu_data){
            if(CFG_GET_BOOL("use_gyro")){
                std::cout << "gyro" << data.gyro.transpose() << std::endl;
                spline.addIMUGyroMeasurement(data.timestamp, data.gyro, false);
            }else{
                std::cout << data.accel.x() << " " << data.accel.y() << " " <<  data.accel.z() << " " << data.gyro.x() << " "  << data.gyro.y() << " "  << data.gyro.z() << " "  << std::endl;
                spline.AddImuMeasurement(data);
            }
            // RCLCPP_INFO(rclcpp::get_logger("main"), "imu: %6f, %6f, %6f", data.accel.x(), data.accel.y(), data.accel.z());
        }
    }

    /*
        测试单纯的位置和旋转优化
    */
    // Sophus::SO3d so3(Eigen::Quaterniond::Identity());
    // for(double i = 0.00; i < time_end; i +=  time_interval){
    //     Eigen::Vector3d pos;
    //     pos << 0, sin(i), i;
    //     so3 = so3 * Sophus::SO3d::rotX(CFG_GET_FLOAT("value_rot")); 
    //     std::cout << so3.matrix() << std::endl;
    //     spline.addLidarPositionMeasurement(i, pos);
    //     spline.addLidarOrientationMeasurement(i, so3.log());
    // }


    slam_utils::NdtOmpUtils util(12, CFG_GET_BOOL("ndt_filter"));
    pcl::shared_ptr<pcl::PointCloud<PointT>> map = pcl::make_shared<pcl::PointCloud<PointT>>();
    pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    for(int i = CFG_GET_INT("start_iterats"); i < lidar_data.size() - 1; i++){
        std::cout << "at: " << i << "\t";
        auto pcl_now = lidar_data[i].data;
        auto pcl_next = lidar_data[i + 1].data;
        double cost_ms= 0;

        cost_ms = util.align(pcl_next, pcl_now, result);
        transform = transform * result;

        auto next_transformed = pcl::make_shared<pcl::PointCloud<PointT>>();
        pcl::transformPointCloud(*pcl_next, *next_transformed, transform);
        (*map) += (*next_transformed);
        if(CFG_GET_BOOL("show_process")){
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color_red(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pcl_now, 255, 0, 0));
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color_green(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pcl_next, 0, 255, 0));
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color_blue(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(next_transformed, 0, 0, 255));
            if(CFG_GET_BOOL("process_detail"))
            {
                viewer.addPointCloud(pcl_now, *color_red, std::string("r%d", i));
                viewer.addPointCloud(pcl_next, *color_green, std::string("g%d", i));
                viewer.updatePointCloud(pcl_now);
                viewer.updatePointCloud(pcl_next);
            }

            viewer.addPointCloud(next_transformed, *color_blue,std::string("b%d", i));
            viewer.updatePointCloud(next_transformed);
         
            viewer.spin();
        }

        if(i == CFG_GET_INT("iterats"))break;
        Eigen::Vector3d trans = transform.block<3,1>(0,3).cast<double>();
        Sophus::SO3f rot(transform.block<3,3>(0,0));
        Eigen::Vector3f qua1 = rot.unit_quaternion().vec();
        
        Eigen::Isometry3d tra(Eigen::Matrix3d::Identity());
        tra.pretranslate(qua1.cast<double>());
        trajectry1->push_back(tra);
        Sophus::SO3f last_so3((transform * result.inverse()).block<3,3>(0,0));
        Eigen::Vector3f qua = last_so3.unit_quaternion().vec();
        // float dis = (rot.log() - last_so3.log()).norm();
        float dis = (qua1 - qua).norm();
        std::cout << "dis: " << dis << std::endl;

        // Eigen::Isometry3d tra(transform.block<3,3>(0,0).cast<double>());
        // tra.pretranslate(trans);
        // trajectry->push_back(tra);
        // spline.addLidarPositionMeasurement(lidar_data[i+i].timestamp, trans);
        // Eigen::Matrix3f mat_ori = transform.block<3,3>(0,0);
        // Sophus::SO3f ori(mat_ori);
        // spline.addLidarOrientationMeasurement(lidar_data[i+i].timestamp, ori.log().cast<double>());

        if(i > lidar_data[i + 1].timestamp > time_end)break;
    }
     
    pcl::visualization::PointCloudColorHandlerCustom<PointT>::Ptr color_map(new pcl::visualization::PointCloudColorHandlerCustom<PointT>(map, 100, 100, 100));
    viewer.addPointCloud(map, *color_map, std::string("map"));
    viewer.updatePointCloud(map);
    if(CFG_GET_BOOL("show_viewer"))viewer.spin();

    ceres::Solver::Options options;
    // options.gradient_tolerance = 0.000001 * Sophus::Constants<double>::epsilon();
    // options.function_tolerance = 0.000001 * Sophus::Constants<double>::epsilon();
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    for( int i = 0; i < CFG_GET_INT("iter_times"); i++){
    ceres::Solver::Summary summary = spline.solve(options); //优化信息
        std::cout <<  summary.BriefReport() << std::endl;
    }


    for(double t = 0; t < time_end; t += 0.5){
        std::cout << "t: " << t << std::endl;
        Eigen::Vector3d pos = spline.rd_spline->evaluate<0>(t * 1e9);
        // Eigen::Vector3d accel = spline.rd_spline->evaluate<2>(t * 1e9);
        Eigen::Vector3d accel = spline.rd_spline->acceleration(t * 1e9);
        std::cout << "pos: " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl; 
        std::cout << "accel: " << accel.x() << " " << accel.y() << " " << accel.z() << std::endl; 
        Sophus::SO3d::Tangent rot_vel = spline.so3_spline->velocityBody(t * 1e9);
        Sophus::SO3d rot = spline.so3_spline->evaluate(t * 1e9);
        std::cout << rot.matrix() << std::endl;
        std::cout << rot_vel.matrix() << std::endl;
    }

    for (double t = 0; t < time_end; t += time_interval){
        Sophus::SO3d rot = spline.so3_spline->evaluate(t * 1e9);
        Eigen::Vector3d qua = rot.unit_quaternion().vec();
        Eigen::Vector3d pos = spline.rd_spline->evaluate<0>(t * 1e9);
        Eigen::Isometry3d tra(Eigen::Quaterniond::Identity());
        tra.pretranslate(qua);
        trajectry->push_back(tra);
    }
    std::cout << "bias:\n" << spline.bia_a.transpose() << "\n" << spline.bia_g.transpose() << "\n" << spline.g.transpose() << std::endl;
    std::vector<slam_utils::TrajectoryPtr> trajectries;
    if(CFG_GET_BOOL("show_tra0"))trajectries.push_back(trajectry);
    if(CFG_GET_BOOL("show_tra1"))trajectries.push_back(trajectry1);
    slam_utils::DrawTrajectories(trajectries, "test");

    rclcpp::shutdown();
}

/****************************************************************************************************************
* 纯IMU积分
***************************************************************************************************************/

// #include "utils/imu_integral.hpp"

// int main(int argc, char const *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto input = slam_utils::Input<sensor_data::LidarData<PointIRT>>::createInstance();
//     std::vector<sensor_data::IMUData, Eigen::aligned_allocator<sensor_data::IMUData>>&  imu_data = *(input->getIMUData());
//     // std::vector<sensor_data::IMUData, Eigen::aligned_allocator<sensor_data::IMUData>>  imu_data;
//     // for(double i = 0; i < 50; i+= 1){
//     //     sensor_data::IMUData imu;
//     //     imu.accel << CFG_GET_FLOAT("acc0"),CFG_GET_FLOAT("acc1"), CFG_GET_FLOAT("acc2");
//     //     imu.gyro << CFG_GET_FLOAT("gyro0"),CFG_GET_FLOAT("gyro1"), CFG_GET_FLOAT("gyro2");
//     //     imu.timestamp = i;
//     //     imu_data.push_back(imu);
//     // }
//     slam_utils::ImuIntegration tra(Sophus::SO3d(Eigen::Matrix3d::Identity()),Sophus::Vector3d(0,0,0), &imu_data);

//     std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>> trajectry;
//     for(auto& i : imu_data){
//         Sophus::SE3d se3 = tra.getSE3(i.timestamp);
//         std::cout << "t: "  << i.timestamp << "\n" << se3.matrix() << std::endl;
//         std::cout << "imu " << i.gyro.transpose() << " " << i.accel.transpose() << std::endl;
//         Sophus::SO3d so3 = se3.so3();
//         Eigen::Vector3d pos = se3.translation();
//         Eigen::Isometry3d tra(so3.unit_quaternion());
//         tra.pretranslate(pos);
//         trajectry.push_back(tra);
//     }
//     slam_utils::DrawTrajectory(trajectry);
//     rclcpp::shutdown();
//     return 0;
// }



// #include "lci_cali/se3_spline_helper.hpp"

// int main(int argc, char const *argv[])
// {
//     rclcpp::init(argc, argv);

//     lci_cali::run_calibration();
//     // slam_utils::Input<sensor_data::LidarData<PointIRT>> input("input");
//     // ConfigYaml cfg;
//     // YAML::Node cfg_root = ConfigYaml::cfg_root;
//     // auto imu_data = input.getIMUData();
//     // for(auto& i : imu_data)
//     // {
//     //     std::cout << i.timestamp << std::endl;
//     // }
//     rclcpp::shutdown();
//     return 0;
// }
