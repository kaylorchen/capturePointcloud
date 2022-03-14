//
// Created by Kaylor on 22-3-14.
//
#include "core/DepthCamera.h"

const bool rgb_enable = true;

int main(void) {
    std::cout << "single thread" << std::endl;
    DepthCamera first("146222253257",
                      rgb_enable, 848, 480, 60,
                      true, 848, 480, 60,
                      true, 848, 480, 60);
    DepthCamera second("146222253926",
                       rgb_enable, 848, 480, 60,
                       true, 848, 480, 60,
                       true, 848, 480, 60);
    while (true) {
        if (rgb_enable){
            first.processPointCloudwithRGB();
            second.processPointCloudwithRGB();
        } else{
            first.processPointCloud();
            second.processPointCloud();
        }

    }
    return 0;
}
