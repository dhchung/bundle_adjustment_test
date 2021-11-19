#include <iostream>
#include "params.h"
#include "opencv2/opencv.hpp"
#include "image_processing.h"


int main(int argc, char**argv) {
    std::cout<<"Bundle Adjustment"<<std::endl;
    std::string json_dir = "/home/dongha/dev/bundle_adjustment_test/ba_cpp/camera_calibration_result.json";
    Params params;
    params.read_data(json_dir);

    float resize_factor = 2.0;

    ImageProcessing img_proc(&params);
    img_proc.setupORB(cv::NORM_HAMMING);
    
    cv::VideoCapture cap("../../dataset/Husky.mp4");
    if(!cap.isOpened()) {
        std::cout<<"Failed to open the video"<<std::endl;
        return -1;
    }

    cv::Mat last_frame;
    std::pair<std::vector<cv::KeyPoint>, cv::Mat> LastFeatureDescriptorPair;

    while(1) {
        cv::Mat frame;
        cap >> frame;
        if(frame.empty()) {
            break;
        }

        cv::resize(frame, frame, cv::Size(frame.size().width/resize_factor, frame.size().height/resize_factor));
        frame = img_proc.UndistortImage(frame);

        std::pair<std::vector<cv::KeyPoint>, cv::Mat> FeatureDescriptorPair = img_proc.extractFeaturesAndDescriptors(frame);

        if(last_frame.empty()) {

            last_frame = frame;
            continue;
        } 

        //Perform 2 View BA
        //Estimate Essential Matrix



        cv::imshow("Current Frame", frame);
        cv::waitKey(1);


        last_frame = frame;
        LastFeatureDescriptorPair = FeatureDescriptorPair;
    }

    return 0;
}