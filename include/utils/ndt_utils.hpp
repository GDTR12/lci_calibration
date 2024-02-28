#pragma once


#include "sensor_data/cloud_type.h"
#include "Eigen/Core"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/ModelCoefficients.h"
#include "pclomp/ndt_omp.h"
#include <chrono>
#include <ctime>
#include <thread>
#include "eigen_utils.hpp"
#include "config_yaml.h"
namespace slam_utils{



template<typename PointSource = pcl::PointXYZ, typename PointTarget = pcl::PointXYZ>
class NdtOmpUtils
{
    typedef pclomp::NormalDistributionsTransform<PointSource, PointTarget> Ndt;
    typedef pcl::PointCloud<PointSource> SourceType;
    typedef pcl::PointCloud<PointTarget> TargetType;
private:
    int num_threads;
    bool enable_filter;
    pcl::VoxelGrid<PointSource> filter;
    ConfigYaml yaml;
public:

    typename Ndt::Ptr ndt;
    NdtOmpUtils(int num_threads = std::thread::hardware_concurrency(), bool enable_filter = true):
        num_threads(num_threads), enable_filter(enable_filter)
    {
        ndt = typename Ndt::Ptr(new Ndt());
        ndt->setResolution(CFG_GET_FLOAT("ndt_resolution"));
        ndt->setStepSize(0.1);
        ndt->setTransformationEpsilon(0.01);
        ndt->setMaximumIterations(30);
        ndt->setNumThreads(num_threads);
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        float filter_size = CFG_GET_FLOAT("ndt_filter_size");
        filter.setLeafSize(filter_size,filter_size,filter_size);
    }


    /**
     * 返回时间[ms]
     */
    double align(typename SourceType::Ptr& source, 
                 typename TargetType::Ptr& target, 
                 Eigen::Matrix4f& mat = Eigen::Matrix4f::Identity(), 
                 typename SourceType::Ptr result = pcl::make_shared<SourceType>())
    {
        
        assert(source->points.size() > 0);
        assert(target->points.size() > 0);

        struct timespec start;
        clock_gettime(CLOCK_MONOTONIC, &start);
        double start_ms = start.tv_sec * 1000 + start.tv_nsec / 1e6;

        auto begin_opt = mat;

        if(enable_filter){
            typename TargetType::Ptr filtered_source(new SourceType());
            filter.setInputCloud(source);
            filter.filter(*filtered_source);
            ndt->setInputSource(filtered_source);
            ndt->setInputTarget(target);
            ndt->align(*result, begin_opt);
            mat = ndt->getFinalTransformation();
        }else{
            ndt->setInputSource(source);
            ndt->setInputTarget(target);
            ndt->align(*result, begin_opt);
            mat = ndt->getFinalTransformation();
        }
        if(CFG_GET_BOOL("ndt_show_info")){
            std::cout << "score:" << ndt->getFitnessScore() << "  "
                << "iterator times: " << ndt->getFinalNumIteration() << std::endl;
        }
        struct timespec end;
        clock_gettime(CLOCK_MONOTONIC, &end);
        double end_ms = end.tv_sec * 1000 + end.tv_nsec / 1e6;
        return (end_ms - start_ms);
    }

    double alignFrameMap(typename SourceType::Ptr& frame, 
                 typename TargetType::Ptr& map, 
                 Eigen::Matrix4f& mat = Eigen::Matrix4f::Identity(), 
                 typename SourceType::Ptr result = pcl::make_shared<SourceType>())
    {
        assert(frame->points.size() > 0);
        assert(map->points.size() > 0);

        struct timespec start;
        clock_gettime(CLOCK_MONOTONIC, &start);
        double start_ms = start.tv_sec * 1000 + start.tv_nsec / 1e6;

        auto begin_opt = mat;
        typename TargetType::Ptr filtered_map(new SourceType());
        filter.setInputCloud(map);
        filter.filter(*filtered_map);

        if(enable_filter){
            typename TargetType::Ptr filtered_frame(new SourceType());
            filter.setInputCloud(frame);
            filter.filter(*filtered_frame);
            ndt->setInputSource(filtered_frame);
            ndt->setInputTarget(filtered_map);
            ndt->align(*result, begin_opt);
            mat = ndt->getFinalTransformation();
        }else{
            ndt->setInputSource(frame);
            ndt->setInputTarget(filtered_map);
            ndt->align(*result, begin_opt);
            mat = ndt->getFinalTransformation();
        }
        if(CFG_GET_BOOL("ndt_show_info")){
            std::cout << "score:" << ndt->getFitnessScore() << "  "
                << "iterator times: " << ndt->getFinalNumIteration() << std::endl;
        }
        struct timespec end;
        clock_gettime(CLOCK_MONOTONIC, &end);
        double end_ms = end.tv_sec * 1000 + end.tv_nsec / 1e6;
        return (end_ms - start_ms);
    }

    void ndtSurferMap(std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::Matrix<double, 4, 1>>>& surfelmaps,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                      double lambda)
    {
        ndt->setInputTarget(target);
        for(const auto& leaf_ : ndt->getTargetCells().getLeaves()){
            auto leaf = leaf_.second;
            if (leaf.nr_points < yaml.cfg_root["ndt_cellMinPoints"].as<int>()) continue;
            int plane_type = checkPlane(leaf.getEvals(), leaf.getEvecs(), lambda);
            if (plane_type < 0) continue;
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
            Eigen::Matrix<double, 4, 1> plane_coffe;
            pcl::PointIndices plane_point_indices;
            auto pl = leaf.pointList_.makeShared();
            if (!fitPlane(pl, plane_coffe, plane_point_indices))
                continue;
            pcl::copyPointCloud(*pl, plane_point_indices, *plane);
            surfelmaps.push_back({plane, plane_coffe});

        }
    }

    int checkPlane(const Eigen::Vector3d &eigen_value,
                                      const Eigen::Matrix3d &eigen_vector,
                                      const double &p_lambda) {
        Eigen::Vector3d sorted_vec;
        Eigen::Vector3i ind;
        Eigen::sort_vec(eigen_value, sorted_vec, ind);

        double p = 2 * (sorted_vec[1] - sorted_vec[2]) /
                    (sorted_vec[2] + sorted_vec[1] + sorted_vec[0]);

        if (p < p_lambda) {
            return -1;
        }

        int min_idx = ind[2];
        Eigen::Vector3d plane_normal = eigen_vector.block<3, 1>(0, min_idx);
        plane_normal = plane_normal.array().abs();

        Eigen::sort_vec(plane_normal, sorted_vec, ind);
        return ind[2];
    }

    bool fitPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                  Eigen::Matrix<double, 4, 1>& result,
                  pcl::PointIndices& indices_inline){
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ModelCoefficients plane_coffs; // 法向量的: x, y, z, distance

        // 可选参数
        seg.setOptimizeCoefficients(true);
        // 固定参数
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.05);

        seg.setInputCloud(cloud);
        seg.segment(indices_inline, plane_coffs);
        if (indices_inline.indices.size() < yaml.cfg_root["ndt_planeMinPoints"].as<int>()) {
            return false;
        }
        for (int i = 0; i < 4; i++) {
            result(i) = plane_coffs.values[i];
        }
        return true;
    }

    ~NdtOmpUtils(){}
};



}
