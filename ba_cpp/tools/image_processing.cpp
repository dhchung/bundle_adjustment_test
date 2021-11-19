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