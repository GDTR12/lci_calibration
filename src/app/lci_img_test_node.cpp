#include "utils/img_utils.hpp"
// #include "utils/ORB-SLAM-Extract/ORBextractor.h"

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
    // std::string strSettingPath("/root/workspace/ros2/src/lci_calibration/config/TUM1.yaml");
    // cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    // int nFeatures = fSettings["ORBextractor.nFeatures"];
    // float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    // int nLevels = fSettings["ORBextractor.nLevels"];
    // int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    // int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    // ORB_SLAM2::ORBextractor orbExtract(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    
    return 0;
}




