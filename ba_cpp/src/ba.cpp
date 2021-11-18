#include <iostream>
#include "params.h"
#include "opencv2/opencv.hpp"
#include "image_processing.h"


int main(int argc, char**argv) {
    std::cout<<"Bundle Adjustment"<<std::endl;
    std::string json_dir = "/home/dongha/dev/bundle_adjustment_test/ba_cpp/camera_calibration_result.json";
    Params params;
    params.read_data(json_dir);

    ImageProcessing img_proc(&params);

    
    cv::VideoCapture cap("../../dataset/Husky.mp4");
    if(!cap.isOpened()) {
        std::cout<<"Failed to open the video"<<std::endl;
        return -1;
    }


    while(1) {
        cv::Mat frame;
        cap >> frame;
        if(frame.empty()) {
            break;
        }

        img_proc.UndistortImage(frame);

        cv::imshow("Fucking", frame);
        cv::waitKey(1);
    }

    return 0;
}