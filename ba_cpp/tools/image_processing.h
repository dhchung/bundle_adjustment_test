#pragma once
#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H
#include "params.h"
#include "opencv2/xfeatures2d/nonfree.hpp"


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
    cv::Ptr<cv::DescriptorMatcher> matcher;

    void setupORB(int MatchingMethod);
    void setupSURF(int MatchingMethod);

    std::pair<std::vector<cv::KeyPoint>, cv::Mat> extractFeaturesAndDescriptors(cv::Mat& img);
    std::vector<cv::DMatch> matchFeatures(cv::Mat trgDesc, cv::Mat srcDesc);

};


#endif