#include "utils/img_utils.hpp"
#include "utils/ORBextractor.h"

const std::string path_of_img = "/root/workspace/ros2/src/lci_calibration/tmp/bird.png";

int main(int argc, char const *argv[])
{
    cv::Mat img = cv::imread(path_of_img, cv::IMREAD_COLOR);
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
    cv::Mat edge_img;
    slam_utils::imgEdgeDetector(20, 200, gray_img, edge_img);
    std::cout << " aa" << std::endl;

    cv::imshow("Edge", edge_img);
    cv::waitKey(0);
    return 0;
}




