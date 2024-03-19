#pragma once

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <unordered_map>
#include <pcl/kdtree/flann.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include "utils/math_utils.hpp"

namespace slam_utils{


template<typename PointT>
struct PlaneMap{
    using PointCloudT = pcl::PointCloud<PointT>;
    PlaneMap(){}
    PlaneMap(float res): resolution(res){}

    std::unordered_map<std::string, int> center_idx;
    std::vector<pcl::shared_ptr<PointCloudT>> pcls; // 分割后的点云
    MathUtils::aligned_vector<Eigen::Matrix<float,4,3>> plane_coffes; // col(0): center of plane,  col(1): normal vector, distance  col(2):center of box
    pcl::KdTreeFLANN<PointT> kd_tree; // 用来搜索近邻
    PointCloudT center_points;
    // typename PointCloudT::Ptr continuoisEdge;
    typename PointCloudT::Ptr continuousEdge;
    float resolution;
    std::string pcl2String(PointT& p){return std::string(reinterpret_cast<char*>(&p.x), 12);} 

    void addPlane(pcl::shared_ptr<PointCloudT> pcl, 
                  Eigen::Vector4f normal_vec, 
                  PointT& center_p,
                  PointT& center_box
                  ){
        center_idx[pcl2String(center_p)] = pcls.size();
        pcls.push_back(pcl);
        Eigen::Matrix<float,4,3> mat;
        mat.block<3,1>(0,0) = Eigen::Vector3f(center_p.x, center_p.y, center_p.z);
        mat.block<4,1>(0,1) = normal_vec;
        mat.block<3,1>(0,2) = Eigen::Vector3f(center_box.x, center_box.y, center_box.z);
        plane_coffes.emplace_back(std::move(mat)); 
        center_points.push_back(center_p);
    }
    void complete(){kd_tree.setInputCloud(pcl::make_shared<PointCloudT>(center_points));}
    // return <idx, distance>
    bool serachNearestPlane(PointT& p, float dis_threshold, Eigen::Vector4f& plane, int& index){
        std::vector<int> idx(1);
        std::vector<float> dis(1);
        kd_tree.nearestKSearch(p, 1, idx, dis);
        if(dis.front() > dis_threshold) return false;
        PointT& point = center_points.points[idx[0]];
        Eigen::Map<Eigen::Matrix<float, 3,1>> p2vec(&p.x);
        index = center_idx[pcl2String(point)];
        plane = plane_coffes.at(index).block<4,1>(0,1);

        // if(CFG_GET_BOOL("plane_map_show_box"))showBox(index, p);
        return true;
    }

    void setResolution(float res){resolution = res;}
    // pcl::KdTreeFLANN<PointT>& getKdTree(){return kd_tree;}
    void showPlanes(){
        std::vector<typename PointCloudT::Ptr> pcl;
        for(int i = 0; i < plane_coffes.size(); i++)
        {
            Eigen::Vector3f n = plane_coffes.at(i).block<3,1>(0,1);
            auto d = plane_coffes.at(i)(3,1);
            auto pl = pcl::make_shared<PointCloudT>();
            Eigen::Vector3f min,max;
            Eigen::Vector3f box_half_size = Eigen::Vector3f::Ones() * resolution / 2.0f;
            min = plane_coffes.at(i).block<3,1>(0,2) - box_half_size;
            max = plane_coffes.at(i).block<3,1>(0,2) + box_half_size;
            for(int j = 0; pl->size() < 300; j++)
            {
                
                Eigen::Vector3f center = plane_coffes.at(i).block<3,1>(0,0);
                float x1 = center.x() + MathUtils::rand_float(-resolution/2, resolution/2);
                float x2 = center.y() + MathUtils::rand_float(-resolution/2, resolution/2);
                float x3 = (-d - n.x() * x1 - n.y() * x2) / n.z();
                if( x1 > max[0] || x1 < min[0] || 
                    x2 > max[1] || x2 < min[1] ||
                    x3 > max[2] || x3 < min[2]) continue;
                std::cout << pl->size() << " ";
                PointT p(x1, x2, x3);
                pl->push_back(p);
            }
            std::cout << endl;
            pcl.push_back(pl);
        }
        showPcl(pcl);
    }

    void showBox(int idx, PointT& p)
    {
        PointCloudT cloud;
        auto mat = plane_coffes.at(idx);
        PointT plane_center(mat(0,0), mat(1,0), mat(2,0));
        PointT box_center(mat(0,2),mat(1,2),mat(2,2));
        cloud.push_back(box_center);
        for(int i = 0; i < 8; i++)
        {
            PointT conner(box_center.x - 0.5f * resolution * std::pow(-1,(int)i/4),
                     box_center.y - 0.5f * resolution * std::pow(-1,(int)i/2),
                     box_center.z - 0.5f * resolution * std::pow(-1,i));
            cloud.push_back(conner);
        }
        cloud.push_back(p);
        showPcl(*pcls.at(idx), cloud);
    }

    void getDepthContinuousEdge(typename PointCloudT::Ptr pcd, float thread)
    {
        continuousEdge.reset(pcl::make_shared<PointCloudT>());
        for (size_t i = 0; i < center_points.size(); i++)
        {
            std::vector<int> indices;
            std::vector<float> distance;
            kd_tree.radiusSearch(center_points[i], resolution * 1.5, indices, distance);
            
        }
        
    }
};


}