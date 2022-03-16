//
// Created by kaylor on 2022/3/8.
//

#include <iostream> //标准输入输出流
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件
#include "core/DepthCamera.h"
#include "boost/thread/thread.hpp"
#include "time.h"
#include "core/example.hpp"

void showFramerate() {
    struct timespec timestamp;
    clock_gettime(CLOCK_MONOTONIC, &timestamp);
    static uint64_t last;
    uint64_t current = timestamp.tv_sec * 1000 + timestamp.tv_nsec / 1000000;
    std::cout << "Framerate is " << 1000.0 / (current - last) << std::endl;
    last = current;
}


#define XYZRGB
// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);
const int width = 848;
const int height = 480;

int main(int argc, char **argv) {

    auto firstTransform = std::make_unique<Eigen::Affine3f>(Eigen::Affine3f::Identity());

//    firstTransform->translation() << 1.58, 0, 0;
    firstTransform->rotate(Eigen::AngleAxisf(M_PI/6, Eigen::Vector3f::UnitY()));
    std::cout << "firstTransform -> matrix():\n" << firstTransform->matrix() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->resize(width * height);
    cloud->is_dense = false;

    DepthCamera first("146222253926", std::move(firstTransform), cloud, 0,
                      true, 848, 480, 60,
                      true, 848, 480, 60,
                      true, 848, 480, 60);
//    DepthCamera second("146222253257", std::move(firstTransform), pcl::PointCloud::Ptr(),
//                       true, 848, 480, 60,
//                       true, 848, 480, 60,
//                       true, 848, 480, 60);

    while (true){
        first.generatePointCloud();
    }
    // Create a simple OpenGL window for rendering:
    window app(848, 480, "RealSense Pointcloud Example");
    // Construct an object to manage view state
    glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);
    while(app){
        first.generatePointCloud();
        draw_pointcloud(app.width(), app.height(), app_state, cloud);
        showFramerate();
    }
    return EXIT_SUCCESS;
}
