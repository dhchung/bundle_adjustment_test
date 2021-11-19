#include "image_processing.h"

ImageProcessing::ImageProcessing(Params * params_):params(params_){
    cameraMatrix = params->camParam.cameraMatrix;
    distCoeffs = params->camParam.distCoeffs;
    calibrate_once = false;
}

ImageProcessing::~ImageProcessing(){}

cv::Mat ImageProcessing::UndistortImage(cv::Mat & img) {

    cv::Mat undistorted_img;
    // Need to change camera calibration matrix if there is a change in image size
    // Do it once
    if(!calibrate_once) {
        int img_width = img.size().width;
        double image_scale = static_cast<double>(params->camParam.imageSize.width) / static_cast<double>(img_width);
        cameraMatrix.at<double>(0,0) /= image_scale;
        cameraMatrix.at<double>(1,1) /= image_scale;
        cameraMatrix.at<double>(0,2) /= image_scale;
        cameraMatrix.at<double>(1,2) /= image_scale;

        std::cout<<"Camera Matrix is rescaled to :"<<std::endl;
        std::cout<<cameraMatrix<<std::endl;
        calibrate_once = true;
    }


    cv::undistort(img, undistorted_img, cameraMatrix, distCoeffs);
    return undistorted_img;
}

void ImageProcessing::setupORB(int MatchingMethod){
    feature = cv::ORB::create();
    matcher = cv::BFMatcher::create(MatchingMethod);
}

void ImageProcessing::setupSURF(int MatchingMethod){
    feature = cv::xfeatures2d::SurfFeatureDetector::create();
    // matcher = cv::BFMatcher::create(MatchingMethod);
    matcher = cv::FlannBasedMatcher::create();
}


std::pair<std::vector<cv::KeyPoint>, cv::Mat> ImageProcessing::extractFeaturesAndDescriptors(cv::Mat & img) {

    std::vector<cv::KeyPoint> KeyPoints;
    cv::Mat Descriptors;

    if(img.channels()>1) {\

        cv::Mat img_gray;
        cv::cvtColor(img, img_gray, cv::COLOR_RGB2GRAY);
        feature->detectAndCompute(img_gray, cv::Mat(), KeyPoints, Descriptors);

    } else if(img.channels() == 1) {

        feature->detectAndCompute(img, cv::Mat(), KeyPoints, Descriptors);
    }

    return std::pair<std::vector<cv::KeyPoint>, cv::Mat>(KeyPoints, Descriptors);

}

std::vector<cv::DMatch> ImageProcessing::matchFeatures(cv::Mat trgDesc, cv::Mat srcDesc) {
    std::vector<cv::DMatch> matches;
    matcher->match(trgDesc, srcDesc, matches);
    return matches;
}
