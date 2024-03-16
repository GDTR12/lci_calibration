#pragma once
#include "Eigen/Core"
#include "Eigen/Dense"
#include <vector>
#include <pangolin/pangolin.h>
#include <typeinfo>
#include "pcl/visualization/cloud_viewer.h"
#include "lidar_utils.hpp"
#include "utils/config_yaml.h"

namespace slam_utils
{




using Trajectory = std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>>;
using TrajectoryPtr = std::shared_ptr<Trajectory>;  


void randomHSVtoRGB(int* cl);
void linearHSVToRGB(int idx, int size, int* cl);
void DrawTrajectory(Trajectory& poses, std::string name);
void DrawTrajectory(TrajectoryPtr& poses, std::string name);
void DrawTrajectories(std::vector<TrajectoryPtr>& trajs, std::string name);

extern int pcl_viewer_id;
extern double pcl_color_list[4][3];
template <typename PointT, typename... Rest>
void addPcl(pcl::visualization::PCLVisualizer& pcl_viewer,const pcl::PointCloud<PointT>& pcl, const pcl::PointCloud<Rest>&... rest) {
    double* color = pcl_color_list[pcl_viewer_id];

    auto p_pcl = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    slam_utils::pclCopyToXYZ(pcl, *p_pcl);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cl(p_pcl, color[0], color[1],color[2]);
    pcl_viewer.addPointCloud(p_pcl, cl, std::to_string(pcl_viewer_id));
    pcl_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,CFG_GET_FLOAT("ui_pcl_point_size"),std::to_string(pcl_viewer_id));
    pcl_viewer.updatePointCloud(p_pcl);
    pcl_viewer_id++;
    if constexpr (sizeof...(rest) > 0) {
        addPcl(pcl_viewer, rest...);
    }
}

template <typename PointT, typename... Rest>
void showPcl(const pcl::PointCloud<PointT>& pcl,const pcl::PointCloud<Rest>&... rest)
{
    pcl::visualization::PCLVisualizer pcl_viewer("Show PointCloud");
    pcl_viewer.removeAllPointClouds();
    pcl_viewer_id = 0;
    addPcl(pcl_viewer, pcl, rest...);
    pcl_viewer.spin();
}

template <typename PointT>
void showPcl(std::vector<pcl::shared_ptr<pcl::PointCloud<PointT>>>& pcls, std::string name=std::string("Show PointCloud"))
{
    pcl::visualization::PCLVisualizer pcl_viewer(name);
    for(int i = 0; i < pcls.size(); i++)
    {
        auto p_pcl = pcls.at(i);
        int cl[3];
        randomHSVtoRGB(cl);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color(p_pcl, cl[0], cl[1],cl[2]);
        pcl_viewer.addPointCloud(p_pcl, color, std::to_string(i));
        pcl_viewer.updatePointCloud(p_pcl);
    }
    pcl_viewer.spin();
}

template <typename PointT>
void showPcl(std::vector<pcl::shared_ptr<pcl::PointCloud<PointT>>>& pcls, std::vector<std::array<int,3>>& color, std::vector<float> size, std::string name=std::string("Show PointCloud"))
{
    pcl::visualization::PCLVisualizer pcl_viewer(name);
    assert(pcls.size() == color.size() && pcls.size() == color.size() && pcls.size() == size.size());
    for(int i = 0; i < pcls.size(); i++)
    {
        auto p_pcl = pcls.at(i);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cl(p_pcl, color.at(i)[0], color.at(i)[1], color.at(i)[2]);
        pcl_viewer.addPointCloud(p_pcl, cl, std::to_string(i));
        pcl_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size.at(i),std::to_string(i));
        pcl_viewer.updatePointCloud(p_pcl);
    }
    pcl_viewer.spin();
}



} // namespace slam_utils



