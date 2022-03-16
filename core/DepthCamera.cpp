//
// Created by kaylor on 2022/3/4.
//

#include "DepthCamera.h"
#include "time.h"

DepthCamera::DepthCamera(std::string serial_number, std::unique_ptr<Eigen::Affine3f> transform,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, unsigned int offset,
                         bool rgb_enable, int rgb_width, int rgb_height, int rgb_framerate,
                         bool infrared_enable, int infrared_width, int infrared_height, int infrared_framerate,
                         bool depth_enable, int depth_width, int depth_height, int depth_framerate) :
        serial_name(serial_number), mRgb_enable(rgb_enable), mRgb_height(rgb_height), mRgb_width(rgb_width),
        mRgb_framerate(rgb_framerate), mInfrared_enable(infrared_enable), mInfrared_height(infrared_height),
        mInfrared_width(infrared_width), mInfrared_framerate(infrared_framerate), mDepth_enable(depth_enable),
        mDepth_width(depth_width), mDepth_height(depth_height), mDepth_framerate(depth_framerate),
        mOffset(offset), mPointCloudXYZ(point_cloud) {
    std::cout << "This camera serial number is " << serial_number << std::endl;
    rs2::config cfg;
    // Add desired streams to configuration
    cfg.enable_device(serial_number);
    if (mRgb_enable) {
        cfg.enable_stream(RS2_STREAM_COLOR, mRgb_width, mRgb_height, RS2_FORMAT_RGB8, mRgb_framerate);
    }
    if (mInfrared_enable) {
        cfg.enable_stream(RS2_STREAM_INFRARED, mInfrared_width, mInfrared_height, RS2_FORMAT_Y8, mInfrared_framerate);
    }
    if (mDepth_enable) {
        cfg.enable_stream(RS2_STREAM_DEPTH, mDepth_width, mDepth_height, RS2_FORMAT_Z16, mDepth_framerate);
    }
    mPipelineProfile = mPipe.start(cfg);
    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    for (int i = 0; i < 10; ++i) {
        auto frames = mPipe.wait_for_frames();
    }
    std::cout << "Camera " << serial_number << " initialization is complete" << std::endl;
    mTransform = std::move(transform);
    if (mTransform != nullptr) {
        std::cout << mTransform->matrix() << std::endl;
        auto depth_stream = mPipelineProfile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        auto depthCameraParams = depth_stream.get_intrinsics();
        Eigen::Matrix3f K;
        K << 	depthCameraParams.fx, 		0.0, 						depthCameraParams.ppx,
                0.0, 						depthCameraParams.fy, 		depthCameraParams.ppy,
                0.0, 						0.0, 						1.0;
        calculateMatrix(K, mTransform->rotation(), mTransform->translation());
    } else {
        std::cout << "mTransform is till null\n";
    }
}

DepthCamera::DepthCamera(std::string serial_number, std::unique_ptr<Eigen::Affine3f> transform,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, unsigned int offset,
                         bool rgb_enable, int rgb_width, int rgb_height, int rgb_framerate,
                         bool infrared_enable, int infrared_width, int infrared_height, int infrared_framerate,
                         bool depth_enable, int depth_width, int depth_height, int depth_framerate) :
        serial_name(serial_number), mRgb_enable(rgb_enable), mRgb_height(rgb_height), mRgb_width(rgb_width),
        mRgb_framerate(rgb_framerate), mInfrared_enable(infrared_enable), mInfrared_height(infrared_height),
        mInfrared_width(infrared_width), mInfrared_framerate(infrared_framerate), mDepth_enable(depth_enable),
        mDepth_width(depth_width), mDepth_height(depth_height), mDepth_framerate(depth_framerate),
        mOffset(offset), mPointCloudXYZRGB(point_cloud) {
    std::cout << "This camera serial number is " << serial_number << std::endl;
    rs2::config cfg;
    // Add desired streams to configuration
    cfg.enable_device(serial_number);
    if (mRgb_enable) {
        cfg.enable_stream(RS2_STREAM_COLOR, mRgb_width, mRgb_height, RS2_FORMAT_RGB8, mRgb_framerate);
    }
    if (mInfrared_enable) {
        cfg.enable_stream(RS2_STREAM_INFRARED, mInfrared_width, mInfrared_height, RS2_FORMAT_Y8, mInfrared_framerate);
    }
    if (mDepth_enable) {
        cfg.enable_stream(RS2_STREAM_DEPTH, mDepth_width, mDepth_height, RS2_FORMAT_Z16, mDepth_framerate);
    }
    mPipelineProfile = mPipe.start(cfg);
    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    for (int i = 0; i < 10; ++i) {
        auto frames = mPipe.wait_for_frames();
    }
    std::cout << "Camera " << serial_number << " initialization is complete" << std::endl;
    mTransform = std::move(transform);
    if (mTransform != nullptr) {
        std::cout << mTransform->matrix() << std::endl;
        auto depth_stream = mPipelineProfile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        auto depthCameraParams = depth_stream.get_intrinsics();
        Eigen::Matrix3f K;
        K << 	depthCameraParams.fx, 		0.0, 						depthCameraParams.ppx,
                0.0, 						depthCameraParams.fy, 		depthCameraParams.ppy,
                0.0, 						0.0, 						1.0;
        calculateMatrix(K, mTransform->rotation(), mTransform->translation());
    } else {
        std::cout << "mTransform is till null\n";
    }
}

void DepthCamera::calculateMatrix(Eigen::Matrix3f K, Eigen::Matrix3f R, Eigen::Vector3f T) {
    A = R.inverse() * K.inverse();
    B = -R.inverse() * T;
    std::cout << "A =\n" << A << std::endl << "B =\n" << B << std::endl;
    Eigen::Vector3f tmp;
    mX_pre = cv::Mat_<float>(mDepth_height, mDepth_width);
    mY_pre = cv::Mat_<float>(mDepth_height, mDepth_width);
    mZ_pre = cv::Mat_<float>(mDepth_height, mDepth_width);

    mX = cv::Mat_<float>(mDepth_height, mDepth_width);
    mY = cv::Mat_<float>(mDepth_height, mDepth_width);
    mZ = cv::Mat_<float>(mDepth_height, mDepth_width);

    mBx = cv::Mat_<float>(mDepth_height, mDepth_width);
    mBy = cv::Mat_<float>(mDepth_height, mDepth_width);
    mBz = cv::Mat_<float>(mDepth_height, mDepth_width);

    for (int i = 0; i < mDepth_height; ++i) {
        for (int j = 0; j < mDepth_width; ++j) {
            //注意这里，先是横坐标，再是纵坐标
            tmp << j, i, 1;
            tmp = A * tmp;
            mX_pre.at<float>(i,j) = tmp(0);
            mY_pre.at<float>(i,j) = tmp(1);
            mZ_pre.at<float>(i,j) = tmp(2);
            mBx.at<float>(i,j) = B(0);
            mBy.at<float>(i,j) = B(1);
            mBz.at<float>(i,j) = B(2);
        }
    }
}

void DepthCamera::processPointCloud(void) {
    auto frames = mPipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    mPoints = mPc.calculate(depth);
    struct timespec timestamp;
    clock_gettime(CLOCK_MONOTONIC, &timestamp);
    uint64_t current = timestamp.tv_sec * 1000 + timestamp.tv_nsec / 1000000;
    //printf是线程安全的，cout不是
    printf("serial is %s, framerate is %f\n", serial_name.c_str(), 1000.0 / (current - last_time));
    last_time = current;
}

void DepthCamera::generatePointCloud(void) {
    auto frames = mPipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    cv::Mat d_img_16U = cv::Mat(cv::Size(mDepth_width, mDepth_height), CV_16U, (void *) depth.get_data(),
                                cv::Mat::AUTO_STEP);
    cv::Mat d_img;
    d_img_16U.convertTo(d_img, CV_32F);
    mX = d_img.mul(mX_pre) + mBx;
    mY = d_img.mul(mY_pre) + mBy;
    mZ = d_img.mul(mZ_pre) + mBz;
    for (int i = 0; i < mDepth_height; ++i) {
        for (int j = 0; j < mDepth_width; ++j) {
            mPointCloudXYZ->points[mOffset + i * mDepth_width + j].x = mX.at<float>(i, j)/1000;
            mPointCloudXYZ->points[mOffset + i * mDepth_width + j].y = mY.at<float>(i, j)/1000;
            mPointCloudXYZ->points[mOffset + i * mDepth_width + j].z = mZ.at<float>(i, j)/1000;
        }
    }
    struct timespec timestamp;
    clock_gettime(CLOCK_MONOTONIC, &timestamp);
    uint64_t current = timestamp.tv_sec * 1000 + timestamp.tv_nsec / 1000000;
    //printf是线程安全的，cout不是
    printf("serial is %s, framerate is %f\n", serial_name.c_str(), 1000.0 / (current - last_time));
    last_time = current;
}

void DepthCamera::generatePointCloudwithRGB(void) {

}

void DepthCamera::processPointCloudwithRGB(void) {
    auto frames = mPipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    auto color = frames.get_color_frame();
    mPc.map_to(color);
    mPoints = mPc.calculate(depth);
    struct timespec timestamp;
    clock_gettime(CLOCK_MONOTONIC, &timestamp);
    uint64_t current = timestamp.tv_sec * 1000 + timestamp.tv_nsec / 1000000;
    //printf是线程安全的，cout不是
    printf("serial is %s, framerate is %f\n", serial_name.c_str(), 1000.0 / (current - last_time));
    last_time = current;
}

