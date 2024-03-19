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
#include "math_utils.hpp"
#include "config_yaml.h"
#include <utils/plane_map.hpp>

namespace slam_utils{


class NdtOmpUtils
{
    using PointT = pcl::PointXYZ;
    typedef pclomp::NormalDistributionsTransform<PointT, PointT> Ndt;
    typedef pcl::PointCloud<PointT> PointCloudT;
private:
    int num_threads;
    bool enable_filter;
    pcl::VoxelGrid<PointT> filter;
    ConfigYaml yaml;
public:

    typename Ndt::Ptr ndt;
    NdtOmpUtils(int num_threads = std::thread::hardware_concurrency(), bool enable_filter = true):
        num_threads(num_threads), enable_filter(enable_filter)
    {
        ndt = typename Ndt::Ptr(new Ndt());
        ndt->setResolution(2);
        ndt->setStepSize(0.1);

        ndt->setTransformationEpsilon(0.01);
        ndt->setMaximumIterations(30);
        ndt->setNumThreads(num_threads);
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        float filter_size = 2;
        filter.setLeafSize(filter_size,filter_size,filter_size);
    }

    explicit NdtOmpUtils(
        const float resolution, 
        float step_size, 
        float epsilon, 
        int max_iters, 
        float filter_size
    ):num_threads(std::thread::hardware_concurrency()), enable_filter(true)
    {
        ndt = typename Ndt::Ptr(new Ndt());
        ndt->setResolution(resolution);
        ndt->setStepSize(step_size);
        ndt->setTransformationEpsilon(epsilon);
        ndt->setMaximumIterations(max_iters);
        ndt->setNumThreads(num_threads);
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        filter.setLeafSize(filter_size,filter_size,filter_size);
    }

    /**
     * 返回时间[ms]
     */
    template<typename PointCloudSourcePtr>
    double align(PointCloudSourcePtr& source_, 
                 PointCloudSourcePtr& target_, 
                 Eigen::Matrix4f& mat, 
                 typename PointCloudT::Ptr result = pcl::make_shared<PointCloudT>())
    {
        
        assert(source_->points.size() > 0);
        assert(target_->points.size() > 0);
        struct timespec start;
        clock_gettime(CLOCK_MONOTONIC, &start);
        double start_ms = start.tv_sec * 1000 + start.tv_nsec / 1e6;
        auto source = pcl::make_shared<PointCloudT>();
        auto target = pcl::make_shared<PointCloudT>();
        pcl::copyPointCloud(*source_, *source);
        pcl::copyPointCloud(*target_, *target);

        auto begin_opt = mat;
        // pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> myndt;
        if(enable_filter){
            auto filtered_source = pcl::make_shared<PointCloudT>();
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
        
        struct timespec end;
        clock_gettime(CLOCK_MONOTONIC, &end);
        double end_ms = end.tv_sec * 1000 + end.tv_nsec / 1e6;
        if(CFG_GET_BOOL("ndt_show_info")){
            std::cout << "score:" << ndt->getFitnessScore() << "  "
                << "iterator times: " << ndt->getFinalNumIteration() << " "
                << "cost time: " << end_ms - start_ms << std::endl;
        }
        return (end_ms - start_ms);
    }
    
    template<typename PointCloudSourcePtr>
    double alignFrameMap(PointCloudSourcePtr& frame_, 
                 PointCloudSourcePtr& map_, 
                 Eigen::Matrix4f& mat, 
                 typename PointCloudT::Ptr result = pcl::make_shared<PointCloudT>())
    {
        assert(frame_->points.size() > 0);
        assert(map_->points.size() > 0);

        struct timespec start;
        clock_gettime(CLOCK_MONOTONIC, &start);
        double start_ms = start.tv_sec * 1000 + start.tv_nsec / 1e6;
        auto map = pcl::make_shared<PointCloudT>();
        auto frame = pcl::make_shared<PointCloudT>();
        pcl::copyPointCloud(*frame_, *frame);
        pcl::copyPointCloud(*map_, *map);

        auto begin_opt = mat;
        typename PointCloudT::Ptr filtered_map(new PointCloudT());
        filter.setInputCloud(map);
        filter.filter(*filtered_map);

        if(enable_filter){
            typename PointCloudT::Ptr filtered_frame(new PointCloudT());
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






    void ndtSurferMap(std::vector<std::pair<PointCloudT::Ptr, Eigen::Matrix<double, 4, 1>>>& surfelmaps,
                      PointCloudT::Ptr target,
                      double lambda, int cell_minPoints, int plane_minPoints)
    {
        ndt->setInputTarget(target);
        for(const auto& leaf_ : ndt->getTargetCells().getLeaves()){
            auto leaf = leaf_.second;
            if (leaf.nr_points < cell_minPoints) continue;
            int plane_type = checkPlane(leaf.getEvals(), leaf.getEvecs(), lambda);
            if (plane_type < 0) continue;
            
            PointCloudT::Ptr plane(new PointCloudT);
            Eigen::Matrix<double, 4, 1> plane_coffe;
            pcl::PointIndices plane_point_indices;
            auto pl = leaf.pointList_.makeShared();

            if (!fitPlane(pl, plane_coffe, plane_point_indices, plane_minPoints))
                continue;
            pcl::copyPointCloud(*pl, plane_point_indices, *plane);
            surfelmaps.push_back({plane, plane_coffe});
        }
    }

    Eigen::Vector3f ndtGetPlaneCenter(const Eigen::Vector4d& plane_coffe,
        PointT& coord_center)
    {
        // 
        // auto idx = ndt->getTargetCells().getGridCoordinates(p.x(), p.y(), p.z());
        // float res = ndt->getResolution();
        // auto coord_center = PointT(res*(idx.x()+0.5), res*(idx.y()+0.5), res*(idx.z()+0.5));
        Eigen::Map<Eigen::Vector3f> center_box(&coord_center.x);
        Eigen::Map<Eigen::Vector3d const> normd(plane_coffe.data());
        Eigen::Vector3f norm = normd.cast<float>();
        // 计算投影点坐标
        // std::cout << center_box.transpose() << std::endl;
        // Eigen::Vector3f center_plane = center_box + (plane_coffe(3) - center_box.dot(Eigen::Vector3f(norm.array().pow(2)))) * norm;
        Eigen::Vector3f center_plane = center_box - norm.dot(center_box + plane_coffe(3) * norm) * norm;
        return center_plane;
    }

    void ndtSurferMap(slam_utils::PlaneMap<PointT>& surfel,
                      typename PointCloudT::Ptr target,
                      double lambda, int cell_minPoints, int plane_minPoints)
    {
        ndt->setInputTarget(target);
        auto pcl_centers = ndt->getTargetCells().getCentroids();
        auto& leafs = ndt->getTargetCells().getLeaves();
        PointCloudT pcl, pcl2,center;

        for(const auto& leaf_ : ndt->getTargetCells().getLeaves()){
            auto leaf = leaf_.second;
            if (leaf.nr_points < cell_minPoints) continue;
            int plane_type = checkPlane(leaf.getEvals(), leaf.getEvecs(), lambda);
            if (plane_type < 0) continue;
            
            PointCloudT::Ptr plane(new PointCloudT);
            Eigen::Matrix<double, 4, 1> plane_coffe;
            pcl::PointIndices plane_point_indices;
            auto pl = leaf.pointList_.makeShared();
            if (!fitPlane(pl, plane_coffe, plane_point_indices, plane_minPoints))continue;

            auto p = leaf.centroid;
            auto idx = ndt->getTargetCells().getGridCoordinates(p.x(), p.y(), p.z());
            float res = ndt->getResolution();
            auto coord_center = PointT(res*(idx.x()+0.5), res*(idx.y()+0.5), res*(idx.z()+0.5));
            auto center_plane = ndtGetPlaneCenter(plane_coffe, coord_center); 
            PointT center_pos(center_plane.x(), center_plane.y(), center_plane.z());
            pcl.push_back(coord_center);
            pcl.push_back(PointT(center_plane.x(), center_plane.y(), center_plane.z()));
            pcl::copyPointCloud(*pl, plane_point_indices, *plane);

            surfel.addPlane(plane, plane_coffe.cast<float>(), center_pos, coord_center);
        }
        surfel.setResolution(ndt->getResolution());
        surfel.complete();
    }


    int checkPlane(const Eigen::Vector3d &eigen_value,
                                      const Eigen::Matrix3d &eigen_vector,
                                      const double &p_lambda) {
        Eigen::Vector3d sorted_vec;
        Eigen::Vector3i ind;
        MathUtils::sort_vec(eigen_value, sorted_vec, ind);

        double p = 2 * (sorted_vec[1] - sorted_vec[2]) /
                    (sorted_vec[2] + sorted_vec[1] + sorted_vec[0]);

        if (p < p_lambda) {
            return -1;
        }

        int min_idx = ind[2];
        Eigen::Vector3d plane_normal = eigen_vector.block<3, 1>(0, min_idx);
        plane_normal = plane_normal.array().abs();

        MathUtils::sort_vec(plane_normal, sorted_vec, ind);
        return ind[2];
    }

    bool fitPlane(const PointCloudT::Ptr& cloud, 
                  Eigen::Matrix<double, 4, 1>& result,
                  pcl::PointIndices& indices_inline, int min_points_in_plane){
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ModelCoefficients plane_coffs; // 法向量的: x, y, z, distance

        // 可选参数
        seg.setOptimizeCoefficients(true);
        // 固定参数
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // seg.setDistanceThreshold(CFG_GET_FLOAT("ndt_plane_threshold"));
        seg.setDistanceThreshold(CFG_GET_FLOAT("ndt_plane_threshold"));
// 
        seg.setInputCloud(cloud);
        seg.segment(indices_inline, plane_coffs);
        if (indices_inline.indices.size() < min_points_in_plane) {
            return false;
        }
        for (int i = 0; i < 4; i++) {
            result(i) = plane_coffs.values[i];
        }
        // result.block<3,1>(0,0) = (-result.block<3,1>(0,0)).eval(); // 方向从远点
        return true;
    }


    

    ~NdtOmpUtils(){}
};




}
