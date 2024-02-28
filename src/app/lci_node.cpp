#include "iostream"
#include "rclcpp/rclcpp.hpp"
#include "utils/input.hpp"
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
#include <Eigen/Core>
#include <ceres_lie_spline.h>
#include "utils/lidar_utils.hpp"
#include "utils/input.hpp"
#include "utils/config_yaml.h"
#include "yaml-cpp/yaml.h"
#include "utils/ui_utils.hpp"
#include "sensor_data/vlp16_data.hpp"
#include "utils/ndt_utils.hpp"
#include <pcl/visualization/cloud_viewer.h>
/****************************************************************************************************************
 TODO: 纯IMU轨迹预测
***************************************************************************************************************/
// struct CostFunctor
// {
//     template <typename T>
//     bool operator()(const T* const x, T* residual) const
//     {
//         residual[0] = T(10.0) - x[0];
//         return true;
//     }
// };

// struct TestFactor
// {
//     template<typename T>
//     bool operator()(T const* const* param, T* residual) const{
//         residual[0] = T(meas_y) - (param[0][0] * T(std::pow(meas_x, 2)) + param[0][1] * T(meas_x) + param[0][2]); 
//         return true;
//     }
//     double meas_x, meas_y;
// };

// void Test()
// {
//     const int meas_num = 5000;
//     double y[meas_num];
//     double x[meas_num] = {0};
//     int a = 5; 
//     int b = 3;
//     int c = 6;
//     for(int i = 0; i < meas_num; i++){
//         x[i] = i;
//         double noise = 0.1 * std::rand() / RAND_MAX;
//         y[i] = a * std::pow(x[i], 2) + b * x[i] + c + noise;
//     }
//     double param_block[3] = {0};
//     ceres::Problem problem;
//     for(int i = 0; i < meas_num; i++){
//         // auto cost_function = new ceres::AutoDiffCostFunction<TestFactor, 1, 3>(new TestFactor{x[i], y[i]});

//         auto cost_function = new ceres::DynamicAutoDiffCostFunction<TestFactor>(new TestFactor{x[i], y[i]});
//         cost_function->AddParameterBlock(3);
//         cost_function->SetNumResiduals(1);
//         problem.AddResidualBlock(cost_function, nullptr, param_block);
//     }
//     ceres::Solver::Options options;
//     ceres::Solver::Summary summary; //优化信息
//     ceres::Solve(options, &problem, &summary);  //开始执行求解
//     std::cout << summary.FullReport() << std::endl;
//     std::cout << "param:" << param_block[0] << " " << param_block[1] << " " << param_block[2] << std::endl;
// }




// int main(int argc, char* argv[])
// {
//     rclcpp::init(argc,argv);
//     // lidar_utils::LidarUtils<PointIRT> util;
//     slam_utils::Input<sensor_data::LidarData<PointIRT>> input("input");
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
//     slam_utils::Input<sensor_data::LidarData<PointIRT>> input("input");
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
//         // PointCloudIRT::Ptr cloud = frame.raw_data;
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




/****************************************************************************************************************
// TODO: 面元地图
***************************************************************************************************************/


#include "utils/ndt_utils.hpp"
#include "sensor_data/cloud_type.h"
#include "sensor_data/vlp16_data.hpp"
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



int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    // NdtOmpUtils<lslidar::PointXYZIRT, lslidar::PointXYZIRT> util;
    ConfigYaml config;

    slam_utils::NdtOmpUtils<pcl::PointXYZ, pcl::PointXYZ> util(12, CFG_GET_BOOL("ndt_filter"));

    // std::shared_ptr<slam_utils::Input<sensor_data::LidarData<PointIRT>>> input(slam_utils::Input<sensor_data::LidarData<PointIRT>>::createInstance());
    auto input = slam_utils::Input<sensor_data::LidarData<PointI>>::createInstance();
    auto lidar_data = input->getLidarData();
    auto front_ = lidar_data.front();
    // std::cout << front_
    for(auto& p : front_.data->points){
        // RCLCPP_INFO(rclcpp::get_logger("main"), "(%f, %f, %f) (i,r,t)(%f, %d, %f)", p.x, p.y, p.z, p.intensity, p.ring, p.time);
    }
    // viewData(lidar_data);
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> map = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // pcl::fromROSMsg(*lidar_data[0].raw_data, *map);
    pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    for(int i = CFG_GET_INT("start_iterats"); i < lidar_data.size() - 1; i++) {
        Eigen::Matrix4f result;
        pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_next = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_now = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        // pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> res = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*lidar_data[i].raw_data, *pcl_now);
        pcl::fromROSMsg(*lidar_data[i+1].raw_data, *pcl_next);
        
        double cost_ms;
        if(CFG_GET_BOOL("reverse")){
             cost_ms = util.align(pcl_now, pcl_next, result);
        }else{
             cost_ms = util.align(pcl_next, pcl_now, result);
        }
        transform = transform * result;
        pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> next_transformed = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        if(CFG_GET_BOOL("if_mark4")){
            pcl::transformPointCloud(*pcl_next, *next_transformed, transform);
        }else{
            pcl::transformPointCloud(*pcl_next, *next_transformed, result);
        }
        std::cout << result << std::endl;
        std::cout << next_transformed->points.size() << std::endl;
        (*map) += (*next_transformed);
        std::cout << map->points.size() << std::endl;
        // map->concatenate()
        // auto p1 = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        // auto p2 = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        // double cost_ms = util.align(p1, p2, result);

        if(CFG_GET_BOOL("show_process")){
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_transformed(next_transformed);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color_red(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pcl_now, 255, 0, 0));
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color_green(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pcl_next, 0, 255, 0));
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color_blue(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(ptr_transformed, 0, 0, 255));
            viewer.addPointCloud(pcl_now, *color_red, std::string("r%d", i));
            viewer.addPointCloud(pcl_next, *color_green, std::string("g%d", i));
            viewer.addPointCloud(ptr_transformed, *color_blue,std::string("b%d", i));
            viewer.updatePointCloud(pcl_now);
            viewer.updatePointCloud(pcl_next);
            viewer.updatePointCloud(ptr_transformed);
            std::cout << "at: " << i << std::endl;
            viewer.spin();
        }

        std::cout << cost_ms << " " << std::endl;
        if(i == CFG_GET_INT("iterats"))break;
    }
    if(CFG_GET_BOOL("if_mark1")){
        viewer.spin();
    }
    //该注册函数在渲染输出时每次都调用
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    if(CFG_GET_BOOL("if_mark0")) {
        cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*lidar_data.back().raw_data, *cloud);
    }else{
        cloud = map;
    }
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::Matrix<double, 4, 1>>> planes;
    util.ndtSurferMap(planes, cloud, 0.6);
    if(CFG_GET_BOOL("if_mark2")) {
    auto cloud_all(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 80, 80, 80));
    viewer.addPointCloud(cloud, *cloud_all, std::string("all"));
    viewer.updatePointCloud(cloud);
    }

    int i = 1;
    for(auto& plane : planes){
        auto pl = plane.first;
        auto color = pcl::getRandomColor();

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr cloud_color(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pl, color.r, color.g, color.b));
        viewer.addPointCloud(pl, *cloud_color, std::string("%d", i++));
        viewer.updatePointCloud(pl);
    }
    std::cout << "Mark" << std::to_string(2232) << std::endl;
    viewer.spin();

    return 0;
}


// #include <pcl/point_cloud.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/kdtree/flann.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include "utils/config_yaml.h"
// #include <iostream>
// #include <ctime>
// #include <vector>

// using namespace pcl;
// using namespace std;

// int main(int argc, char**argv)
// {
//     rclcpp::init(argc,argv);
// 	srand(time(NULL));
//     pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
// 	//创建点云对象PointCloud<PointXYZ> boost共享指针并进行实例化
// 	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
//     ConfigYaml config;
//     auto cfg = config.cfg_root;
// 	//点云生成
// 	cloud->width = 1000;
// 	cloud->height = 1;    //无序点云
// 	cloud->points.resize(cloud->width*cloud->height); //总的点数

// 	for (size_t i = 0; i < cloud->points.size(); i++)
// 	{
// 		cloud->points[i].x = 1024.0f*rand() / (RAND_MAX + 1.0f);
// 		cloud->points[i].y = 1024.0f*rand() / (RAND_MAX + 1.0f);
// 		cloud->points[i].z = 1024.0f*rand() / (RAND_MAX + 1.0f);
// 	}

//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color_gray(
//         new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(
//             cloud, cfg["value_int0"].as<int>(), cfg["value_int0"].as<int>(), cfg["value_int0"].as<int>()));
// 	//创建kd树
// 	KdTreeFLANN<PointXYZ> kdtree;
// 	kdtree.setInputCloud(cloud);

// 	PointXYZ  searchPoint;     //设置查询点，并附初始值。
// 	//searchPoint.x = 1024.0f*rand() / (RAND_MAX + 1.0f);
// 	//searchPoint.y = 1024.0f*rand() / (RAND_MAX + 1.0f);
// 	//searchPoint.z = 1024.0f*rand() / (RAND_MAX + 1.0f);
// 	searchPoint.x = 954.281f;
// 	searchPoint.y = 94.5625f;
// 	searchPoint.z = 584.969f;

// 	//k近邻搜索
// 	int k = 10;
// 	vector<int> pointIdxNKNSearch(k);          //找到的k个近邻点的索引     
// 	vector<float> pointNKNSquareDistance(k);   //查询点与近邻点的平方距离
// 	cout << "K nearest neighbor search at(" << searchPoint.x << "   " << searchPoint.y << "   " << searchPoint.z << ") with K=" << k << endl;
//     PointCloud<PointXYZ>::Ptr search_(new PointCloud<PointXYZ>);   
//     PointCloud<PointXYZ>::Ptr k_near_neibor(new PointCloud<PointXYZ>);   
//     PointCloud<PointXYZ>::Ptr radius_search(new PointCloud<PointXYZ>);   
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color_red(
//         new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(
//             search_, cfg["value_int1"].as<int>(), 0,0));
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color_green(
//         new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(
//             k_near_neibor, 0, cfg["value_int2"].as<int>(), 0));
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color_blue(
//         new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(
//             radius_search, 0, 0, cfg["value_int3"].as<int>()));
//     search_->resize(1);
//     search_->points[0] = searchPoint;
//     k_near_neibor->resize(k);

// 	/*  kdtree.nearestKSearch(searchPoint,k, pointIdxNKNSearch, pointNKNSquareDistance)

// 	    brief Search for k-nearest neighbors for the given query point.  //搜素给定点的K近邻。
		 
// 		参数1： 给定的查询点。
// 		参数2： 要搜索的近邻点的数量。
// 		参数3： 输出的k个近邻点索引
// 		参数4： 输出查询点到邻近点的平方距离。

// 		返回值：返回找到的近邻点的数量
// 	*/
// 	if (kdtree.nearestKSearch(searchPoint,k, pointIdxNKNSearch, pointNKNSquareDistance) > 0)
// 	{
// 		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
// 		{
// 			cout << "   " << cloud->points[pointIdxNKNSearch[i]].x
// 			    << "   " << cloud->points[pointIdxNKNSearch[i]].y
// 				<< "   " << cloud->points[pointIdxNKNSearch[i]].z << "(squared distance:" << pointNKNSquareDistance[i] << ")" << endl;
//             k_near_neibor->points[i] = cloud->points[pointIdxNKNSearch[i]];
// 		}
// 	}




// 	//在半径r内搜索近邻
// 	vector<int> pointIdxRadiusSearch;
// 	vector<float> pointRadiusSquareDistance;
// 	float radius = cfg["value_float0"].as<double>();

// 	cout << "Neighbors within radius search at(" << searchPoint.x << "   " << searchPoint.y << "   " << searchPoint.z << ") with radius=" << radius << endl;

// 	/*  brief Search for all the nearest neighbors of the query point in a given radius.
// 	    参数1： 给定的查询点
// 		参数2： 球面的半径，包含查询点的所有近邻
// 		参数3： 输出的k个近邻点索引
// 		参数4： 输出查询点到邻近点的平方距离。
// 	*/
// 	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquareDistance) > 0)
// 	{
//         radius_search->resize(pointIdxRadiusSearch.size());
// 		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
// 		{
// 			cout << "   " << cloud->points[pointIdxRadiusSearch[i]].x
// 				<< "   " << cloud->points[pointIdxRadiusSearch[i]].y
// 				<< "   " << cloud->points[pointIdxRadiusSearch[i]].z
// 				<< "   " << "(squared distance: " << pointRadiusSquareDistance[i] << ")" << endl;
//             radius_search->points[i] = cloud->points[pointIdxRadiusSearch[i]];
// 		}
// 	}
//     viewer.addPointCloud(cloud, *color_gray, "raw");
//     viewer.updatePointCloud(cloud);
//     viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cfg["value_int5"].as<int>(), "raw");

//     viewer.addPointCloud(search_, *color_red, "center");
//     viewer.updatePointCloud(search_);
//     viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cfg["value_int6"].as<int>(), "center");
//     if(cfg["if_mark0"].as<bool>()){
//         viewer.addPointCloud(k_near_neibor, *color_green, "k_near");
//         viewer.updatePointCloud(k_near_neibor);
//         viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cfg["value_int6"].as<int>(), "k_near");
//     }
//     viewer.addPointCloud(radius_search, *color_blue, "radius");
//     viewer.updatePointCloud(radius_search);
//     viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cfg["value_int6"].as<int>(), "radius");
//     viewer.spin();
// 	return 0;
// }



// int main(int argc, char const *argv[])
// {
//     rclcpp::init(argc,argv);
//     ConfigYaml config;

//     std::shared_ptr<slam_utils::Input<sensor_data::LidarData<PointIRT>>> input = std::make_shared<slam_utils::Input<sensor_data::LidarData<PointIRT>>>("input");
//     auto lidar_data = input->getLidarData();

        

// }
