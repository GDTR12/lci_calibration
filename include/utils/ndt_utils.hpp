#pragma once
#include "sensor_data/cloud_type.h"
#include "Eigen/Core"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pclomp/ndt_omp.h"
#include <chrono>
#include <ctime>
#include <thread>

namespace ndt_utils{



template<typename PointSource = lslidar::PointXYZIRT, typename PointTarget = lslidar::PointXYZIRT>
class NdtOmpUtils
{
    typedef pclomp::NormalDistributionsTransform<PointSource, PointTarget> Ndt;
    typedef pcl::PointCloud<PointSource> SourceType;
    typedef pcl::PointCloud<PointTarget> TargetType;
private:
    typename Ndt::Ptr ndt;
    int num_threads;
    bool enable_filter;
    pcl::VoxelGrid<PointSource> filter;
public:
    NdtOmpUtils(int num_threads = std::thread::hardware_concurrency(), bool enable_filter = true):
        num_threads(num_threads), enable_filter(enable_filter)
    {
        ndt = typename Ndt::Ptr(new Ndt());
        ndt->setResolution(1.1);
        ndt->setStepSize(0.1);
        ndt->setTransformationEpsilon(0.01);
        ndt->setMaximumIterations(30);
        ndt->setNumThreads(num_threads);
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        filter.setLeafSize(2,2,2);
    }

    void beginOdometry(){
        
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

        auto begin_opt = Eigen::Matrix4f::Identity();
        // if(result == nullptr){
        //     result = pcl::make_shared<TargetType>();
        // }
        if(enable_filter){
            typename TargetType::Ptr filtered_target(new TargetType());
            typename TargetType::Ptr filtered_source(new SourceType());

            filter.setInputCloud(target);
            filter.filter(*filtered_target);
            filter.setInputCloud(source);
            filter.filter(*filtered_source);
            ndt->setInputSource(filtered_source);
            ndt->setInputTarget(filtered_target);
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
        return (end_ms - start_ms);
    }

    std::vector<Eigen::Matrix4f> getNdtOdometry(){

    }
    template<typename PointType>
    void ndtSurferMap(Eigen::aligned_vector<pcl::PointCloud<PointType>>& surfelmaps)
    {
        // nd
    }

    

    ~NdtOmpUtils(){}
};



}
