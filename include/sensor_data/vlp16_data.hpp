#pragma once
#include "utils/input.hpp"
#include "sensor_data/lidar_data.h"
#include "angles/angles/angles.h"

namespace sensor_data
{


class VLP16Input : public slam_utils::Input<LidarData<PointIRT>, PointCloudI>
{
public:
    using Input = slam_utils::Input<LidarData<PointIRT>, PointCloudI>;
private:
    /** configuration parameters */
    typedef struct {
        double max_range;  ///< maximum range to publish
        double min_range;
        int min_angle;  ///< minimum angle to publish
        int max_angle;  ///< maximum angle to publish
    } Config;
    Config m_config;   
    double mVLP16TimeBlock[1824][16];
    static const int RAW_SCAN_SIZE = 3;
    static const int SCANS_PER_BLOCK = 32;
    static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);
    constexpr static const float ROTATION_RESOLUTION = 0.01f;
    static const uint16_t ROTATION_MAX_UNITS = 36000u;
    constexpr static const float DISTANCE_RESOLUTION = 0.002f;

    /** @todo make this work for both big and little-endian machines */
    static const uint16_t UPPER_BANK = 0xeeff;
    static const uint16_t LOWER_BANK = 0xddff;

    static const int BLOCKS_PER_PACKET = 12;
    static const int PACKET_STATUS_SIZE = 2;

    int FIRINGS_PER_BLOCK;
    int SCANS_PER_FIRING;
    float BLOCK_TDURATION;
    float DSR_TOFFSET;
    float FIRING_TOFFSET;
    float PACKET_TIME;

    float sin_rot_table_[ROTATION_MAX_UNITS];
    float cos_rot_table_[ROTATION_MAX_UNITS];
    float cos_vert_angle_[16];
    float sin_vert_angle_[16];
    int scan_mapping_16[16];


    void setParameters() {
        m_config.max_range = 150;
        m_config.min_range = 1.0;
        m_config.min_angle = 0;
        m_config.max_angle = 36000;
        // Set up cached values for sin and cos of all the possible headings
        for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
        float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
        cos_rot_table_[rot_index] = cosf(rotation);
        sin_rot_table_[rot_index] = sinf(rotation);
        }

        FIRINGS_PER_BLOCK = 2;
        SCANS_PER_FIRING = 16;
        BLOCK_TDURATION = 110.592f;  // [µs]
        DSR_TOFFSET = 2.304f;        // [µs]
        FIRING_TOFFSET = 55.296f;    // [µs]
        PACKET_TIME = (BLOCKS_PER_PACKET * 2 * FIRING_TOFFSET);

        float vert_correction[16] = {
            -0.2617993877991494,  0.017453292519943295, -0.22689280275926285,
            0.05235987755982989,  -0.19198621771937624, 0.08726646259971647,
            -0.15707963267948966, 0.12217304763960307,  -0.12217304763960307,
            0.15707963267948966,  -0.08726646259971647, 0.19198621771937624,
            -0.05235987755982989, 0.22689280275926285,  -0.017453292519943295,
            0.2617993877991494};
        for (int i = 0; i < 16; i++) {
        cos_vert_angle_[i] = std::cos(vert_correction[i]);
        sin_vert_angle_[i] = std::sin(vert_correction[i]);
        }

        scan_mapping_16[15] = 0;
        scan_mapping_16[13] = 1;
        scan_mapping_16[11] = 2;
        scan_mapping_16[9] = 3;
        scan_mapping_16[7] = 4;
        scan_mapping_16[5] = 5;
        scan_mapping_16[3] = 6;
        scan_mapping_16[1] = 7;

        scan_mapping_16[14] = 8;
        scan_mapping_16[12] = 9;
        scan_mapping_16[10] = 10;
        scan_mapping_16[8] = 11;
        scan_mapping_16[6] = 12;
        scan_mapping_16[4] = 13;
        scan_mapping_16[2] = 14;
        scan_mapping_16[0] = 15;

        for (unsigned int w = 0; w < 1824; w++) {
        for (unsigned int h = 0; h < 16; h++) {
            mVLP16TimeBlock[w][h] =
                h * 2.304 * 1e-6 + w * 55.296 * 1e-6;  ///  16*1824
        }
        }
    }

    inline double getExactTime(int dsr, int firing) const {
        return mVLP16TimeBlock[firing][dsr];
    }
    void unpack_scan(PointCloudI& pc_in, LidarData<PointIRT>& lidar_data){
        /// point cloud
        auto& output = *lidar_data.data;
        output.points.clear();
        output.height = pc_in.height;
        output.width = pc_in.width;
        output.is_dense = false;
        output.resize(output.height * output.width);
    
        for (int w = 0; w < pc_in.width; w++) {
        for (int dsr = 0; dsr < pc_in.height; dsr++) {
            double delta_t =  getExactTime(dsr, w);

            PointIRT point;
            // already ros-coordinate: x-right y-front z-up
            point.x = pc_in.at(w, dsr).x;
            point.y = pc_in.at(w, dsr).y;
            point.z = pc_in.at(w, dsr).z;
            point.intensity = 100;
            point.ring = dsr;
            point.time = lidar_data.timestamp + delta_t;

            output.at(w, scan_mapping_16[dsr]) = point;
        }
        }
    }
    VLP16Input(){
        setParameters();
    }

public:




    void lidar_data_preprocess(PointCloudI& before, LidarData<PointIRT>& lidar_data) override
    {
        unpack_scan(before, lidar_data);
    }

    static Input* createInstance()
    {
        Input* instance = new VLP16Input();
        instance->init();
        return instance;
    }

    ~VLP16Input(){}
};   
} // namespace lci_cali
