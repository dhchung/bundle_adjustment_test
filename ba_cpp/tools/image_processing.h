#pragma once
#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H
#include "params.h"


class ImageProcessing{
    public:
    ImageProcessing(Params * params_);
    ~ImageProcessing();

    Params * params;
    cv::Mat UndistortImage(cv::Mat & img);

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    bool calibrate_once;

};


#endif