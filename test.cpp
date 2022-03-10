//
// Created by kaylor on 2022/3/10.
//

#include <iostream> //标准输入输出流
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

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);

int main(int argc, char **argv) {

    auto firstTransform = std::make_unique<Eigen::Affine3f>(Eigen::Affine3f::Identity());
    firstTransform->translation() << 0, 0, 0;
    firstTransform->rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()));
    DepthCamera first("146222253926", std::move(firstTransform),
                      true, 848, 480, 60,
                      true, 848, 480, 60,
                      true, 848, 480, 60);
    DepthCamera second("146222253257",
                       true, 848, 480, 60,
                       true, 848, 480, 60,
                       true, 848, 480, 60);
    // Create a simple OpenGL window for rendering:
    window app(848, 480, "RealSense Pointcloud Example");
    // Construct an object to manage view state
    glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    while(app){
        {
            // Wait for the next set of frames from the camera
            auto frames = first.mPipe.wait_for_frames();


            auto color = frames.get_color_frame();

            // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
            if (!color)
                color = frames.get_infrared_frame();

            // Tell pointcloud object to map to this color frame
            first.mPc.map_to(color);

            auto depth = frames.get_depth_frame();

            // Generate the pointcloud and texture mappings
            first.mPoints = first.mPc.calculate(depth);

            // Upload the color frame to OpenGL
            app_state.tex.upload(color);

            // Draw the pointcloud
            draw_pointcloud(app.width(), app.height(), app_state, first.mPoints, first.mTransform->matrix());
        }

        {
            // Wait for the next set of frames from the camera
            auto frames = second.mPipe.wait_for_frames();


            auto color = frames.get_color_frame();

            // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
            if (!color)
                color = frames.get_infrared_frame();

            // Tell pointcloud object to map to this color frame
            second.mPc.map_to(color);

            auto depth = frames.get_depth_frame();

            // Generate the pointcloud and texture mappings
            second.mPoints = second.mPc.calculate(depth);

            // Upload the color frame to OpenGL
            app_state.tex.upload(color);

            // Draw the pointcloud
            draw_pointcloud(app.width(), app.height(), app_state, second.mPoints);
        }

        showFramerate();
    }
    return EXIT_SUCCESS;
}
