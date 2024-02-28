#include "utils/ui_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/config_yaml.h"

void DrawTrajectory(std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>> poses){
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer",1024,768); // 创建窗口
    
    glEnable(GL_DEPTH_TEST); // 开启深度测试
    glEnable(GL_BLEND); // 开启混合渲染
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // 设置混合函数
    double min_dis[3] = {INFINITY, INFINITY,INFINITY};
    double max_dis[3] = {-INFINITY,-INFINITY,-INFINITY};
    Eigen::Vector3d scale;
    for(const auto& pos : poses){
        min_dis[0] > pos.translation().x() ? min_dis[0] = pos.translation().x() : min_dis[0] = min_dis[0]; 
        min_dis[1] > pos.translation().y() ? min_dis[1] = pos.translation().y() : min_dis[1] = min_dis[1]; 
        min_dis[2] > pos.translation().z() ? min_dis[2] = pos.translation().z() : min_dis[2] = min_dis[2]; 

        max_dis[0] < pos.translation().x() ? max_dis[0] = pos.translation().x() : max_dis[0] = max_dis[0]; 
        max_dis[1] < pos.translation().y() ? max_dis[1] = pos.translation().y() : max_dis[1] = max_dis[1]; 
        max_dis[2] < pos.translation().z() ? max_dis[2] = pos.translation().z() : max_dis[2] = max_dis[2]; 
    }
    scale[0] = max_dis[0] - min_dis[0];
    scale[1] = max_dis[1] - min_dis[1];
    scale[2] = max_dis[2] - min_dis[2];
    if(CFG_GET_BOOL("ui_normailized")){
        for(auto& pos : poses){
            pos.translation().array().colwise() /= 0.01 * (scale.array().col(0));
        }
    }
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000), //投影矩阵
        // 屏幕的宽度、高度、相机的水平视角、垂直视角、相机在z轴上的位置、相机到屏幕的距离的最小值和最大值。
        // pangolin::ModelViewLookAt(1, 1, 1, 0, 0, 0, 0.0, -1.0, 0.0) // 视图矩阵
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0) // 视图矩阵
        // 相机的位置、相机观察的目标点、相机的朝向向量
    );
 
    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
        // 表示窗口在x轴和y轴上的起点和终点位置，以及窗口的宽高比，宽高比为负数，则实际上是768：1024
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // 清空颜色缓冲区和深度缓冲区
        d_cam.Activate(s_cam); // 激活显示窗口和渲染状态对象
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // 设置清屏颜色
        glLineWidth(2); // 设置线宽
 
        for (size_t i = 0; i < poses.size(); i++) {
            // 画每个位姿的三个坐标轴
            double len_axis = CFG_GET_FLOAT("ui_length_orientation");
            Eigen::Vector3d Ow = poses[i].translation(); // 获取相机位姿矩阵中的平移部分，即相机的位置。
            Eigen::Vector3d Xw = poses[i] * (len_axis * Eigen::Vector3d(1, 0, 0)); // 获取x轴方向的单位向量,乘以0.1是为了调整坐标轴线段的长度
            Eigen::Vector3d Yw = poses[i] * (len_axis * Eigen::Vector3d(0, 1, 0)); // 获取y轴方向的单位向量
            Eigen::Vector3d Zw = poses[i] * (len_axis * Eigen::Vector3d(0, 0, 1)); // 获取z轴方向的单位向量
            
            glBegin(GL_LINES); // 开始绘制线段
            glColor3f(1.0, 0.0, 0.0); // 设置线段颜色 rgb
            // 绘制线段的两个端点
            glVertex3d(Ow[0], Ow[1], Ow[2]); // 原点的坐标
            glVertex3d(Xw[0], Xw[1], Xw[2]); // x轴方向的坐标    ----> 绘制x轴线段 为红色
 
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);// 原点的坐标
            glVertex3d(Yw[0], Yw[1], Yw[2]);// y轴方向的坐标    ----> 绘制y轴线段 为绿色
 
            // glColor3f(0.0, 0.0, 1.0);
            // glVertex3d(Ow[0], Ow[1], Ow[2]);// 原点的坐标
            // glVertex3d(Zw[0], Zw[1], Zw[2]);// z轴方向的坐标    ----> 绘制z轴线段 为蓝色
            // glEnd(); // 结束绘制
        }
        
        // 画出连线
        for (size_t i = 0; i < poses.size(); i++) {
            glColor3f(0.0, 0.0, 0.0); // 黑色
            glBegin(GL_LINES); // 开始绘制线段
            auto p1 = poses[i], p2 = poses[i + 1]; // 获取相邻相机位姿
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]); // 绘制线段的两个端点(相邻相机位姿的位置)
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        // std::cout << "running" << std::endl;
        pangolin::FinishFrame(); // 结束当前帧的绘制
    }
}