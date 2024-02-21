#include "iostream"
#include "rclcpp/rclcpp.hpp"
#include "lci_cali/input.hpp"
#include "lci_cali/lci_cali.hpp"

#include "sophus/so3.hpp"
#include "ceres/ceres.h"
#include "ceres/problem.h"
#include "factor/imu_factor.hpp"

#include <ceres/ceres.h>
#include <iomanip>
#include <iostream>
#include <sophus/se3.hpp>

#include <basalt/spline/se3_spline.h>
#include <basalt/spline/so3_spline.h>

#include <ceres_lie_spline.h>
#include "utils/lidar_utils.hpp"
#include "lci_cali/input.hpp"


struct CostFunctor
{
    template <typename T>
    bool operator()(const T* const x, T* residual) const
    {
        residual[0] = T(10.0) - x[0];
        return true;
    }
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


// int main(int argc, char* argv[])
// {
//     rclcpp::init(argc,argv);
//     lci_cali::Input input("input");
//     auto imu_data = input.getIMUData();
//     double time_end = 57.3;
//     const int degree = 4;
//     double time_interval = 0.01;

//     lci_cali::BSplineIMU<4> spline(0, time_end, time_interval);

//     for(double i = 0.00; i < time_end; i +=  0.01){
//         lci_cali::IMUData data;
//         data.timestamp = i;
//         // data.gyro << 0, 0.025 * i, 0.015 * i ;
//         data.accel << 0, 0.01 * i, 0;
//         spline.AddImuMeasurement(data);     
//     }
//     // for(auto& data : imu_data){
//     //     // spline.AddGyroMeasurement(data);
//     //     spline.AddImuMeasurement(data);
//     //     // RCLCPP_INFO(rclcpp::get_logger("main"), "imu: %6f, %6f, %6f", data.accel.x(), data.accel.y(), data.accel.z());
//     // }


//     ceres::Solver::Options options;
//     // options.gradient_tolerance = 0.000001 * Sophus::Constants<double>::epsilon();
//     // options.function_tolerance = 0.000001 * Sophus::Constants<double>::epsilon();
//     options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//     options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
//     ceres::Solver::Summary summary = spline.solve(options); //优化信息
//     std::cout <<  summary.FullReport() << std::endl;
//     for(double t = 0; t < time_end; t += 0.5){
//         std::cout << "t: " << t << std::endl;
//         Eigen::Vector3d pos = spline.rd_spline->evaluate<0>(t * 1e9);
//         // Eigen::Vector3d accel = spline.rd_spline->evaluate<2>(t * 1e9);
//         Eigen::Vector3d accel = spline.rd_spline->acceleration(t * 1e9);
//         std::cout << "pos: " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl; 
//         std::cout << "accel: " << accel.x() << " " << accel.y() << " " << accel.z() << std::endl; 
//         Sophus::SO3d::Tangent rot_vel = spline.so3_spline->velocityBody(t * 1e9);
//         Sophus::SO3d rot = spline.so3_spline->evaluate(t * 1e9);
//         std::cout << rot.matrix() << std::endl;
//         std::cout << rot_vel.matrix() << std::endl;
//     }
//     rclcpp::shutdown();
// }

// int main(int argc, char* argv[])
// {
//     rclcpp::init(argc,argv);
//     // lidar_utils::LidarUtils<PointIRT> util;
//     lci_cali::Input input("input");
//     auto lidar = input.getLidarData();
//     // for(auto& data: lidar){
//     //     util.DataPush(data.raw_data.get());
//     // }
//     // util.ShowLidar("lidar");
//     pangolin::CreateWindowAndBind("Main",1920,1240);
//     glEnable(GL_DEPTH_TEST);
//     glEnable(GL_BLEND);
//     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
//     pangolin::OpenGlRenderState s_cam(
//         pangolin::ProjectionMatrix(1920,1240,420,420,320,320,0.2,100),
//         pangolin::ModelViewLookAt(2,0,2, 0,0,0, pangolin::AxisY)
//     );
    
//     pangolin::Handler3D handler(s_cam); 
//     pangolin::View& d_cam = pangolin::CreateDisplay()
//             .SetBounds(0.0, 1.0, 0.0, 1.0, -1920.0f/1240.0f)
//             .SetHandler(&handler);

//     while( !pangolin::ShouldQuit() )
//     {
//         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//         d_cam.Activate(s_cam);
//         for(auto& data: lidar){
//             glBegin( GL_POINTS );//点设置的开始
//             glColor3f(1.0,0.50,1.0);
//             for(auto& p : data.raw_data->points){
//                 glVertex3f(p.x / 2, p.y / 2 , p.z / 2);
//                 // sleep(0.5);
//             }
//             glEnd();//点设置的结束
//             pangolin::FinishFrame();
//             sleep(0.5);
//         }
//     }

//     return 0;
// }


/*
*/
// #include "pcl/filters/statistical_outlier_removal.h"
// #include "pcl/features/normal_3d.h"
// #include "pcl/features/fpfh.h"
// #include "utils/ndt_utils.hpp"

// int main(int argc, char const *argv[])
// {
//     rclcpp::init(argc,argv);
//     lci_cali::Input input("input");
//     auto lidar_data = input.getLidarData();
//     std::string out;
//     char format[] = {"{ (%.2f, %.2f, %.2f) %.2f, %d, %.3f } "};
//     char buffer[200];
//     int i = 0;
//     int channels = 0;
//     for(auto frame: lidar_data)
//     {  
//         if(i > 5) break; i++;
//         for(auto& p : frame.raw_data->points){
//             // std::sprintf(buffer, format, p.x, p.y, p.z, p.intensity, p.ring, p.time/1e9);
            
//             if(p.ring != channels){
//                 std::cout << std::endl;
//             }
//             double yaw = std::atan2(p.y,p.x);
//             double pitch = std::atan2(p.z, p.x);
//             // std::sprintf(buffer, "[%.2f, %.2f] ", yaw, pitch);
//             std::sprintf(buffer, "[%.2f, %.6f] ", yaw * 180 / M_PI, p.time);
//             // std::
//             std::cout << buffer;
//             channels = p.ring;
//         }
//         std::cout << endl;
//         // Pcl_IRT::Ptr cloud = frame.raw_data;
//         // pcl::StatisticalOutlierRemoval<PointIRT> sor;
//         // sor.setInputCloud(cloud);
//         // sor.setMeanK(50);
//         // sor.setStddevMulThresh(1.0);
//         // sor.filter(*cloud);

//         // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//         // pcl::NormalEstimation<PointIRT, pcl::Normal> ne;
//         // ne.setInputCloud(cloud);
//         // // Set other parameters...
//         // ne.compute(*normals);

//         // pcl::FPFHEstimation<PointIRT, pcl::Normal, pcl::FPFHSignature33> fpfh;
//         // pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
//         // fpfh.setInputCloud(cloud);
//         // fpfh.setInputNormals(normals);
//         // // Set other parameters...
//         // fpfh.compute(*features);
//     }

//     return 0;
// }


#include "utils/ndt_utils.hpp"
#include "sensor_data/cloud_type.h"
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include "utils/config_yaml.h"
int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer &viewer)
{
    viewer.setBackgroundColor(1.0, 0.5, 1.0); //设置背景颜色
    pcl::PointXYZ o;                          //存储球的圆心位置
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 0.25, "sphere", 0); //添加圆球几何对象
    std::cout << "i only run once" << std::endl;
}
/***********************************************************************************
   作为回调函数，在主函数中注册后每帧显示都执行一次，函数具体实现在可视化对象中添加一个刷新显示字符串
   *************************************************************************************/
void viewerPsycho(pcl::visualization::PCLVisualizer &viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0); // 添加文字

    //FIXME: possible race condition here:
    user_data++;
}

void viewData(std::vector<sensor_data::LidarData<PointIRT>>& lidar_data){

    pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
    for(int i = 1; i < lidar_data.size() - 1; i++) {
        auto pcl = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*lidar_data[i].raw_data, *pcl);
        viewer.addPointCloud(pcl, std::string("%d", i));
        viewer.updatePointCloud(pcl);
        viewer.spinOnce();
    }
}



// int main(int argc, char const *argv[])
// {
//     std::cout << __LINE__ << std::endl;
//     rclcpp::init(argc,argv);
//     // NdtOmpUtils<lslidar::PointXYZIRT, lslidar::PointXYZIRT> util;
//     ConfigYaml config;

//     ndt_utils::NdtOmpUtils<pcl::PointXYZ, pcl::PointXYZ> util(12, config.cfg_root["ndt_filter"].as<bool>());

//     std::cout << __LINE__ << std::endl;
//     std::shared_ptr<lci_cali::Input> input = std::make_shared<lci_cali::Input>("input");
//     auto lidar_data = input->getLidarData();
//     // viewData(lidar_data);
//     pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> map = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//     // pcl::fromROSMsg(*lidar_data[0].raw_data, *map);
//     pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
//     Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
//     for(int i = config.cfg_root["start_iterats"].as<int>(); i < lidar_data.size() - 1; i++) {
//         Eigen::Matrix4f result;
//         pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_next = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//         pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_now = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//         // pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> res = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//         pcl::fromROSMsg(*lidar_data[i].raw_data, *pcl_now);
//         pcl::fromROSMsg(*lidar_data[i+1].raw_data, *pcl_next);
        
//         double cost_ms;
//         if(config.cfg_root["reverse"].as<bool>()){
//              cost_ms = util.align(pcl_now, pcl_next, result);
//         }else{
//              cost_ms = util.align(pcl_next, pcl_now, result);
//         }
//         transform = transform * result;
//         pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> next_transformed = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

//         if(config.cfg_root["if_mark4"].as<bool>()){
//             pcl::transformPointCloud(*pcl_next, *next_transformed, transform);
//         }else{
//             pcl::transformPointCloud(*pcl_next, *next_transformed, result);
//         }
//         std::cout << result << std::endl;
//         std::cout << next_transformed->points.size() << std::endl;
//         (*map) += (*next_transformed);
//         std::cout << map->points.size() << std::endl;
//         // map->concatenate()
//         // auto p1 = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
//         // auto p2 = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
//         // double cost_ms = util.align(p1, p2, result);

//         // pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_transformed(next_transformed);
//         // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color_red(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pcl_now, 255, 0, 0));
//         // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color_green(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pcl_next, 0, 255, 0));
//         // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color_blue(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(ptr_transformed, 0, 0, 255));
//         // viewer.addPointCloud(pcl_now, *color_red, std::string("r%d", i));
//         // viewer.addPointCloud(pcl_next, *color_green, std::string("g%d", i));
//         // viewer.addPointCloud(ptr_transformed, *color_blue,std::string("b%d", i));
//         // viewer.updatePointCloud(pcl_now);
//         // viewer.updatePointCloud(pcl_next);
//         // viewer.updatePointCloud(ptr_transformed);

//         std::cout << cost_ms << " " << std::endl;
//         if(i == config.cfg_root["iterats"].as<int>())break;
//     }
//     if(config.cfg_root["if_mark1"].as<bool>()){
//         viewer.spin();
//     }
//     //该注册函数在渲染输出时每次都调用
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
//     if(config.cfg_root["if_mark0"].as<bool>()) {
//         cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//         pcl::fromROSMsg(*lidar_data.back().raw_data, *cloud);
//     }else{
//         cloud = map;
//     }
//     std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::Matrix<double, 4, 1>>> planes;
//     util.ndtSurferMap(planes, cloud, 0.6);
//     if(config.cfg_root["if_mark2"].as<bool>()) {
//     auto cloud_all(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 80, 80, 80));
//     viewer.addPointCloud(cloud, *cloud_all, std::string("all"));
//     viewer.updatePointCloud(cloud);
//     }

//     int i = 1;
//     for(auto& plane : planes){
//         auto pl = plane.first;
//         auto color = pcl::getRandomColor();

//         pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr cloud_color(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pl, color.r, color.g, color.b));
//         viewer.addPointCloud(pl, *cloud_color, std::string("%d", i++));
//         viewer.updatePointCloud(pl);
//     }

//     viewer.spin();

//     return 0;
// }


int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    ConfigYaml config;

    std::shared_ptr<lci_cali::Input> input = std::make_shared<lci_cali::Input>("input");
    auto lidar_data = input->getLidarData();

        

}
