//
// Created by kaylor on 2022/3/4.
//

#include "DepthCamera.h"

DepthCamera::DepthCamera(std::string serial_number, std::unique_ptr<Eigen::Affine3f> transform,
                         bool rgb_enable, int rgb_width, int rgb_height, int rgb_framerate,
                         bool infrared_enable, int infrared_width, int infrared_height, int infrared_framerate,
                         bool depth_enable, int depth_width, int depth_height, int depth_framerate) {
    new(this)DepthCamera(serial_number,
                         rgb_enable, rgb_width, rgb_height, rgb_framerate,
                         infrared_enable, infrared_width, infrared_height, infrared_framerate,
                         depth_enable, depth_width, depth_height, depth_framerate);
    //上面使用的new方法调用其他的构造函数，必须先执行它，不然已经赋值的参数将会被重新初始化
    mTransform = std::move(transform);
    if (mTransform != nullptr)
    {
        std::cout << mTransform->matrix() << std::endl;
    }else{
        std::cout << "mTransform is till null\n";
    }
}

DepthCamera::DepthCamera(std::string serial_number,
                         bool rgb_enable, int rgb_width, int rgb_height, int rgb_framerate,
                         bool infrared_enable, int infrared_width, int infrared_height, int infrared_framerate,
                         bool depth_enable, int depth_width, int depth_height, int depth_framerate) {
    std::cout << "This camera serial number is " << serial_number << std::endl;
    rs2::config cfg;
    mRgb_enable = rgb_enable;
    mRgb_height = rgb_height;
    mRgb_width = rgb_width;
    mRgb_framerate = rgb_framerate;
    mInfrared_enable = infrared_enable;
    mInfrared_height = infrared_height;
    mInfrared_width = infrared_width;
    mInfrared_framerate = infrared_framerate;
    mDepth_enable = depth_enable;
    mDepth_width = depth_width;
    mDepth_height = depth_height;
    mDepth_framerate = depth_framerate;


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
    mPipe.start(cfg);
    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    for (int i = 0; i < 10; ++i) {
        auto frames = mPipe.wait_for_frames();
    }
    std::cout << "Camera " << serial_number << " initialization is complete" << std::endl;
}

std::tuple<uint8_t, uint8_t, uint8_t>
DepthCamera::get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords) {
    const int w = texture.get_width(), h = texture.get_height();

    // convert normals [u v] to basic coords [x y]
    int x = std::min(std::max(int(texcoords.u * w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v * h + .5f), 0), h - 1);

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t *>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr DepthCamera::depthCameraPointXYZRGB(void) {
    auto frames = mPipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    auto colored_frame = frames.get_color_frame();

    mPc.map_to(colored_frame);
    auto points = mPc.calculate(depth);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());


    auto tex_coords = points.get_texture_coordinates();
    auto vertices = points.get_vertices();
    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); ++i) {
        cloud->points[i].x = -vertices[i].x;
        cloud->points[i].y = -vertices[i].y;
        cloud->points[i].z = vertices[i].z;

        std::tuple<uint8_t, uint8_t, uint8_t> current_color;
        current_color = get_texcolor(colored_frame, tex_coords[i]);

        // Reversed order- 2-1-0 because of BGR model used in camera
        cloud->points[i].r = std::get<2>(current_color);
        cloud->points[i].g = std::get<1>(current_color);
        cloud->points[i].b = std::get<0>(current_color);
    }
    if (mTransform != nullptr) {
        pcl::transformPointCloud(*cloud, *cloud, *mTransform);
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthCamera::depthCameraPointXYZ(void) {
    auto frames = mPipe.wait_for_frames();
    auto depth = frames.get_depth_frame();

    auto points = mPc.calculate(depth);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto vertices = points.get_vertices();
    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); ++i) {
        cloud->points[i].x = -vertices[i].x;
        cloud->points[i].y = -vertices[i].y;
        cloud->points[i].z = vertices[i].z;
    }
    if (mTransform != nullptr) {
        pcl::transformPointCloud(*cloud, *cloud, *mTransform);
    }
    return cloud;
}

