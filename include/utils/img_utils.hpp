#pragma once 

#include <opencv2/opencv.hpp>

namespace slam_utils
{
void imgEdgeDetector(const int &canny_threshold, const int &edge_threshold,
    const cv::Mat &src_img, cv::Mat &edge_img);
} // namespace slam_utils


