//
// Created by kaylor on 2022/3/4.
//

#include "multicam.h"

multicam::multicam(std::string serial_number, bool rgb_enable, bool infrared_enable, bool depth_enable, int width,
                   int height, int framerate) {
    std::cout << "This camera serial number is " << serial_number << std::endl;
    rs2::config cfg;
    mRgb_enable = rgb_enable;
    mInfrared_enable = infrared_enable;
    mDepth_enable = depth_enable;
    // Add desired streams to configuration
    if (mRgb_enable) {
        mRgb_framerate = framerate;
        mRgb_width = width;
        mRgb_height = height;
        cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, mRgb_framerate);
    }
    if (mInfrared_enable) {
        mInfrared_framerate = framerate;
        mInfrared_width = width;
        mInfrared_height = height;
        cfg.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8, mInfrared_framerate);
    }
    if (mDepth_enable) {
        mDepth_framerate = framerate;
        mDepth_width = width;
        mDepth_height = height;
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, mDepth_framerate);
    }
    mPipe.start(cfg);
    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    for (int i = 0; i < 100; ++i) {
        auto frames = mPipe.wait_for_frames();
    }
    std::cout << "Camera " << serial_number << " initialization is complete" << std::endl;
}

std::tuple <uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords) {
    const int w = texture.get_width(), h = texture.get_height();

    // convert normals [u v] to basic coords [x y]
    int x = std::min(std::max(int(texcoords.u * w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v * h + .5f), 0), h - 1);

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t *>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr multicamPointXYZRGB(void) {

}

pcl::PointCloud<pcl::PointXYZ>::Ptr multicamPointXYZ(void) {

}