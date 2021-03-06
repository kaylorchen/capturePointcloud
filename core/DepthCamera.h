//
// Created by kaylor on 2022/3/4.
//

#ifndef CAPTUREPOINTCLOUD_DEPTHCAMERA_H
#define CAPTUREPOINTCLOUD_DEPTHCAMERA_H

#include <librealsense2/hpp/rs_pipeline.hpp>
#include "librealsense2/rs.h"
#include "map"
#include "vector"
#include "iostream"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include "pcl/common/transforms.h"
#include <opencv2/opencv.hpp>
#include "memory"


class DepthCamera {
public:
    DepthCamera(std::string serial_number, std::unique_ptr<Eigen::Affine3f> transform,
                pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, unsigned int offset,
                bool rgb_enable, int rgb_width, int rgb_height, int rgb_framerate,
                bool infrared_enable, int infrared_width, int infrared_height, int infrared_framerate,
                bool depth_enable, int depth_width, int depth_height, int depth_framerate);
    DepthCamera(std::string serial_number, std::unique_ptr<Eigen::Affine3f> transform,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, unsigned int offset,
                bool rgb_enable, int rgb_width, int rgb_height, int rgb_framerate,
                bool infrared_enable, int infrared_width, int infrared_height, int infrared_framerate,
                bool depth_enable, int depth_width, int depth_height, int depth_framerate);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthCameraPointXYZRGB(void);

    pcl::PointCloud<pcl::PointXYZ>::Ptr depthCameraPointXYZ(void);

    void processPointCloud(void);
    void processPointCloudwithRGB(void);

    void generatePointCloud(void);
    void generatePointCloudwithRGB(void);

    rs2::pipeline mPipe;
    rs2::pointcloud mPc;
    rs2::points mPoints;
    rs2::pipeline_profile mPipelineProfile;
    std::unique_ptr<Eigen::Affine3f> mTransform = nullptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mPointCloudXYZ = nullptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mPointCloudXYZRGB = nullptr;
    unsigned int mOffset;
    cv::Mat mX;
    cv::Mat mY;
    cv::Mat mZ;
    cv::Mat mX_pre, mY_pre, mZ_pre;
    cv::Mat mBx, mBy, mBz;


private:
    std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);

    void calculateMatrix(Eigen::Matrix3f K, Eigen::Matrix3f R, Eigen::Vector3f T);


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

    unsigned int last_time = 0;
    std::string serial_name;

    Eigen::Matrix3f A;
    Eigen::Vector3f B;

};


#endif //CAPTUREPOINTCLOUD_DEPTHCAMERA_H
