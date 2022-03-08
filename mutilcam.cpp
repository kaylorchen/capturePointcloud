//
// Created by kaylor on 2022/3/8.
//

#include <iostream> //标准输入输出流
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件
#include "DepthCamera.h"
#include "boost/thread/thread.hpp"
#include "time.h"

void showFramerate() {
    struct timespec timestamp;
    clock_gettime(CLOCK_MONOTONIC, &timestamp);
    static uint64_t last;
    uint64_t current = timestamp.tv_sec * 1000 + timestamp.tv_nsec / 1000000;
    std::cout << "Framerate is " << 1000.0 / (current - last) << std::endl;
    last = current;
}

#define XYZRGB

int main(int argc, char **argv) {
    DepthCamera first("146222253926", true, 640, 480, 30,
                      true, 640, 480, 30,
                      true, 640, 480, 30);

    DepthCamera second("146222253257", true, 640, 480, 30,
                       true, 640, 480, 30,
                       true, 640, 480, 30);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr firstCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr secondCloud;


    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    while (!viewer.wasStopped()) {
#ifdef XYZRGB
        firstCloud = first.depthCameraPointXYZRGB();
        secondCloud = second.depthCameraPointXYZRGB();
#else
        firstCloud = first.depthCameraPointXYZ();
        secondCloud = second.depthCameraPointXYZ();
#endif
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->resize(firstCloud->size() + secondCloud->size());
        cloud->is_dense = false;

        uint32_t count = 0;
        uint32_t i = 0;
        while (count < firstCloud->size()) {
            cloud->points[count] = firstCloud->points[i];
            count++;
            i++;
        }
        i = 0;
        while (count < secondCloud->size()) {
            cloud->points[count] = secondCloud->points[i];
            count++;
            i++;
        }

//        viewer.showCloud(cloud);
        showFramerate();
    }
    return EXIT_SUCCESS;
}
