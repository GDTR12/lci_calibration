#pragma once

#include "pcl/point_cloud.h"
#include "ikd-tree/ikd_Tree.h"
#include "pangolin/pangolin.h"
#include "pcl/common/geometry.h"
#include "pclomp/ndt_omp.h"


namespace lidar_utils
{


typedef struct{
    int channels = 16;
    int scan_rate = 10;
}RotateLidarParams;

typedef struct{
    RotateLidarParams base_param;
    double time_offset = 0.0;   
    double yaw_first = 0;
    double yaw_end = 0;
}RotateLidarInfo;

template <typename T>
class LidarUtils
{
    using LidarType = pcl::PointCloud<T>;

private:

public:
    class LidarWindow
    {
    private:
        std::thread window_thread;
        std::string window_title;
        int w, h;
        pangolin::Params params;
        std::vector<LidarType*>* lidar_data;
        int current_frame = 0;
        int current_point = 0;
    public:
        void WindowThread()
        {
            pangolin::CreateWindowAndBind(window_title, w, h, params);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(w, h,420,420,320,320,0.2,100),
                pangolin::ModelViewLookAt(2,0,2, 0,0,0, pangolin::AxisY)
            );
            pangolin::Handler3D handler(s_cam); 
            pangolin::View& d_cam = pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
                    .SetHandler(&handler);      
            pangolin::Var<bool> left_button("ui.A_Button", false, false);//设置一个按钮，默认值为false，最后一个false表示按钮形式
            pangolin::Var<bool> right_button("ui.A_Button", false, false);//设置一个按钮，默认值为false，最后一个false表示按钮形式
            pangolin::Var<int> frame_slider("ui.A_Int", 3, 0, 5);//设置一个double的、可拖动变换值的玩意(不知道咋形容)！

            pangolin::Var<bool> next_button("ui.A_Button", false, false);//设置一个按钮，默认值为false，最后一个false表示按钮形式
            pangolin::Var<bool> pre_button("ui.A_Button", false, false);//设置一个按钮，默认值为false，最后一个false表示按钮形式
            pangolin::Var<int> point_slider("ui.A_Int", 3, 0, 5);//设置一个double的、可拖动变换值的玩意(不知道咋形容)！

            while( !pangolin::ShouldQuit() )
            {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                d_cam.Activate(s_cam);
                if(pangolin::Pushed(left_button)){
                    std::cout << "left button pushed" << std::endl;
                }
                if(pangolin::Pushed(right_button)){
                    std::cout << "right button pushed" << std::endl;
                }
                if(pangolin::Pushed(next_button)){
                    std::cout << "next button pushed" << std::endl;
                }
                if(pangolin::Pushed(pre_button)){
                    std::cout << "previous button pushed" << std::endl;
                }
                if(frame_slider.GuiChanged()){
                    std::cout << "frame slider drawed: " << frame_slider.Get() << std::endl;
                }
                if(point_slider.GuiChanged()){
                    std::cout << "point slider drawed: " << point_slider.Get() << std::endl;
                }
                
                pangolin::FinishFrame();
            }
            return;
        }
        LidarWindow(std::string window_title, int w = 640, int h = 480, const pangolin::Params& params = pangolin::Params()){
            window_thread = std::thread(&LidarWindow::WindowThread,this);
            window_thread.join();
            return;
        }
        void Show(std::vector<LidarType*>* lidar)
        {
            lidar_data = lidar;
        }
        ~LidarWindow(){
            window_thread.detach();
        }
    };

    LidarUtils(){

    }

    ~LidarUtils(){}
    std::vector<LidarType*> lidar_data; 
    
    void DataPush(LidarType* data)
    {
        assert(data != nullptr);
        lidar_data.push_back(data);
    }

    template<typename PointType = PointIRT>
    float PointDistance(pcl::PointCloud<PointType> p1, pcl::PointCloud<PointType> p2)
    {
        return pcl::geometry::distance(p1, p2);
    }



    template<typename PointType = PointIRT>
    void GetFeature(pcl::PointCloud<PointType> pl, RotateLidarParams param) 
    {
        // 获取一帧点云信息
        RotateLidarInfo info = GetLidarInfo(pl,param);
    }

    

    template<typename PointType = PointIRT>
    RotateLidarInfo GetLidarInfo(pcl::PointCloud<PointType>& pl, RotateLidarParams params)
    {
        RotateLidarInfo info = {
            .base_param = params,
        };
        int pl_size = pl.points.size();
        auto& points = pl.points;
        assert(pl_size != 0);

        double offset = points[pl_size].time;
        double yaw_first = atan2(points[0].y, points[0].x) * 180 / M_PI;
        double yaw_end = yaw_first;

        // 如果，雷达数据是densenity, 找到最大的俯仰角
        if(pl.is_dense == true){
            double max_yaw = 0;
            for (size_t i = 0; i < pl_size - 1; i++)
            {
                if(points[i + 1].ring != points[i].ring){
                    double yaw = atan2(points[i].y, points[i].x) * 180 / M_PI;
                    if(max_yaw < yaw){
                        max_yaw = yaw;
                    }
                }
            }
        }else{
            yaw_end = atan2(points[pl_size - 1].y, points[pl_size].x) * 180 / M_PI; 
        }
        info.time_offset = offset;
        info.yaw_first = yaw_first;
        info.yaw_end = yaw_end;
        
        return info;
    }

    template<typename T_Out>
    pcl::PointCloud<T_Out> LidarConvert(pcl::PointCloud<T> pcl){

    }
    
    void ShowLidar(std::string window_title, int w = 640, int h = 480, const pangolin::Params& params = pangolin::Params())
    {
        auto window = std::make_shared<LidarWindow>(window_title, w, h, params);
        window->Show(&lidar_data);
    }


// template<typename PointType = pcl::PointXYZ>
// class ShowOnePcl{
//     typedef pcl::PointCloud<PointType> PCL;
// private:
//     std::vector<typename PCL::Ptr> pcl_data; 

// public:    
// };
    


};







   
} // namespace lci_utils

