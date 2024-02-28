#pragma once
#include "Eigen/Core"
#include "Eigen/Dense"
#include <vector>
#include <pangolin/pangolin.h>

void DrawTrajectory(std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>> poses);



