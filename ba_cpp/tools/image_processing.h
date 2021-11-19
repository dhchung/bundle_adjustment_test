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

    cv::Ptr<cv::Feature2D> feature;

    std::vector<cv::KeyPoint> trgKeypoints, srcKeypoints;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    void setupORB(int MatchingMethod);

    std::pair<std::vector<cv::KeyPoint>, cv::Mat> extractFeaturesAndDescriptors(cv::Mat& img);

};


#endif