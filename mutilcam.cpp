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

    auto firstTransform = std::make_unique<Eigen::Affine3f>(Eigen::Affine3f::Identity());
    firstTransform->translation() << 0, 0, -0.1;
    firstTransform->rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()));
    Eigen::Vector3f v;
    v << 1 , 2 , 3.998845;
    std::cout << v << std::endl;
    float *p;
    p = (float *) &v;

    printf("sizeof(v) = %ld, *p = %f\n", sizeof (v), *(p+2));
    return 0;
    DepthCamera first("146222253926", std::move(firstTransform),
                      true, 848, 480, 60,
                      true, 848, 480, 60,
                      true, 848, 480, 60);
    DepthCamera second("146222253257",
                       true, 848, 480, 60,
                       true, 848, 480, 60,
                       true, 848, 480, 60);
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
        while (i < firstCloud->size()) {
            cloud->points[count] = firstCloud->points[i];
            count++;
            i++;
        }
        i = 0;
        while (i < secondCloud->size()) {
            cloud->points[count] = secondCloud->points[i];
            count++;
            i++;
        }
        std::cout << cloud->size() << " " << firstCloud->size() << " " << secondCloud->size() << count<< std::endl;

        viewer.showCloud(secondCloud);
        showFramerate();
    }
    return EXIT_SUCCESS;
}
