//
// Created by Kaylor on 22-3-16.
//
#include <opencv2/opencv.hpp>
#include "iostream"

int main(void) {
    uint16_t data[] = {1, 2, 1, 3};
    cv::Mat m(cv::Size(2,2), CV_16U, data, cv::Mat::AUTO_STEP);
    std::cout << m << std::endl;
    cv::Mat tmp;
    m.convertTo(tmp, CV_32F);
    std::cout << tmp << std::endl;
    cv::Mat_<float>  m2(2,2);
    m2.at<float>(0,0) = 1;
    m2.at<float>(0,1) = 3;
    m2.at<float>(1,0) = 1;
    m2.at<float>(1,1) = 1;

    cv::Mat m3;
    std::cout << m2 << std::endl;
    m3 = m2.mul(tmp);
    std::cout << m3 << std::endl;
    return 0;
}
