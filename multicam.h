//
// Created by kaylor on 2022/3/4.
//

#ifndef CAPTUREPOINTCLOUD_MULTICAM_H
#define CAPTUREPOINTCLOUD_MULTICAM_H

#include <librealsense2/hpp/rs_pipeline.hpp>
#include "librealsense2/rs.h"
#include "map"
#include "vector"
#include "iostream"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>

class multicam {
public:
    multicam(std::string serial_number,
             bool rgb_enable, int rgb_width, int rgb_height, int rgb_framerate,
             bool infrared_enable, int infrared_width, int infrared_height, int infrared_framerate,
             bool depth_enable, int depth_width, int depth_height, int depth_framerate);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr multicamPointXYZRGB(void);

    pcl::PointCloud<pcl::PointXYZ>::Ptr multicamPointXYZ(void);

private:
    std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);

    rs2::pipeline mPipe;
    rs2::pointcloud mPc;

    bool mRgb_enable = false;
    bool mInfrared_enable = false;
    bool mDepth_enable = false;
    int mRgb_width;
    int mRgb_height;
    int mInfrared_width;
    int mInfrared_height;
    int mDepth_width;
    int mDepth_height;
    int mRgb_framerate;
    int mInfrared_framerate;
    int mDepth_framerate;
};


#endif //CAPTUREPOINTCLOUD_MULTICAM_H
