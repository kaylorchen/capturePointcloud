//
// Created by Kaylor on 22-3-14.
//
#include "thread"
#include "iostream"
#include "core/DepthCamera.h"

const bool rgb_enable = false;
const int width = 848;
const int height = 480;
const int framerate = 30;

void process(std::string serial_number) {
    auto firstTransform = std::make_unique<Eigen::Affine3f>(Eigen::Affine3f::Identity());
//    firstTransform->translation() << 1.58, 0, 0;
//    firstTransform->rotate(Eigen::AngleAxisf(M_PI/6, Eigen::Vector3f::UnitY()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->resize(width * height);
    cloud->is_dense = false;
    DepthCamera camera(serial_number, std::move(firstTransform), cloud, 0,
                       rgb_enable, width, height, framerate,
                       true, width, height, framerate,
                       true, width, height, framerate);
    while (true) {
        if (rgb_enable) {
            camera.processPointCloudwithRGB();
        } else {
            camera.generatePointCloud();
//            camera.processPointCloud();
        }
    }
}

int main(void) {
    std::thread t1(process, "146222253257");
    std::thread t2(process, "146222253926");
    std::thread t3(process, "146222253637");
    std::thread t4(process, "146222254352");
    std::thread t5(process, "146222252441");
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();

    return 0;
}
