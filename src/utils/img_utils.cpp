#include "utils/img_utils.hpp"



namespace slam_utils
{

void imgEdgeDetector(
    const int &canny_threshold, const int &edge_threshold,
    const cv::Mat &src_img, cv::Mat &canny_result) {
    int gaussian_size = 5;


    cv::GaussianBlur(src_img, src_img, cv::Size(gaussian_size, gaussian_size), 0,
                    0);
    int width = src_img.cols;
    int height = src_img.rows;
    canny_result = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Canny(src_img, canny_result, canny_threshold, canny_threshold * 3, 3,
                true);
    // std::vector<std::vector<cv::Point>> contours;
    // std::vector<cv::Vec4i> hierarchy;
    // cv::findContours(canny_result, contours, hierarchy, cv::RETR_EXTERNAL,
    //                 cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
    // edge_img = cv::Mat::zeros(height, width, CV_8UC1);
} 
} // namespace slam_utils
