cmake_minimum_required(VERSION 3.16)
project(capturePointcloud)

set(CMAKE_CXX_STANDARD 14)

add_executable(capturePointcloud capture.cpp )



list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMake")

find_package(PCL 1.6 REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
include(${CMAKE_SOURCE_DIR}/CMake/opengl_config.cmake)

find_package(Boost COMPONENTS program_options REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

#target_include_directories(pcl_testing_2 ${OPENGL_INCLUDE_DIR})
include_directories(${OpenGL_INCLUDE_DIRS})
include_directories(${glfw3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries (capturePointcloud ${PCL_LIBRARIES} librealsense2.so ${OpenCV_LIBS} ${Boost_LIBRARIES} pthread boost_thread)

add_executable(read_display read_display.cpp core/DepthCamera.cpp)
target_link_libraries(read_display ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} pthread boost_thread librealsense2.so)

add_executable(test test.cpp core/DepthCamera.cpp)
target_link_libraries (test ${PCL_LIBRARIES} ${OpenCV_LIBS} ${DEPENDENCIES} ${Boost_LIBRARIES} pthread boost_thread librealsense2.so )

add_executable(mutilcam mutilcam.cpp core/DepthCamera.cpp)
target_link_libraries(mutilcam ${PCL_LIBRARIES} ${OpenCV_LIBS} ${DEPENDENCIES} ${Boost_LIBRARIES} pthread boost_thread librealsense2.so )
