//
// Created by Kaylor on 22-3-14.
//
#include "thread"
#include "iostream"
#include "core/DepthCamera.h"

const bool rgb_enable = false;
const int width = 848;
const int height = 480;
const int framerate = 60;

void process(std::string serial_number) {
    DepthCamera camera(serial_number,
                       rgb_enable, width, height, framerate,
                       true, width, height, framerate,
                       true, width, height, framerate);
    while (true) {
        if (rgb_enable) {
            camera.processPointCloudwithRGB();
        } else {
            camera.processPointCloud();
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
