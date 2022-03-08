//
// Created by kaylor on 2022/3/3.
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
    uint64_t current =  timestamp.tv_sec * 1000 + timestamp.tv_nsec / 1000000;
    std::cout << "Framerate is " << 1000.0/(current - last) << std::endl;
    last = current;
}

int main(int argc, char **argv) {
    DepthCamera main("146222253926", true, 640, 480, 30,
                     true, 640, 480, 30,
                     true, 640, 480, 30);
    auto cloud = main.multicamPointXYZRGB();

//    while (true) {
//        main.multicamPointXYZRGB();
//        showFramerate();
//    }

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
        cloud = main.multicamPointXYZRGB();
        viewer.showCloud(cloud);
        showFramerate();
    }
    return EXIT_SUCCESS;
}