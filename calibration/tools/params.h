#pragma once

#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

struct CameraParameters{

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    cv::Size imageSize = cv::Size(0.0, 0.0);
};


 
class Params{
public:
    Params();
    ~Params();
    void read_data(std::string & json_dir);
    float value2float(Json::ValueIterator & it);
    int value2int(Json::ValueIterator & it);

    CameraParameters camParam;
};