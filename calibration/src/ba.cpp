#include <iostream>
#include "params.h"
#include "opencv2/opencv.hpp"


int main(int argc, char**argv) {
    std::cout<<"Bundle Adjustment"<<std::endl;
    std::string json_dir = "/home/dongha/dev/bundle_adjustment_test/calibration/camera_calibration_result.json";
    Params params;
    params.read_data(json_dir);
    
    cv::VideoCapture cap("../../dataset/Husky.mp4");
    if(!cap.isOpened()) {
        std::cout<<"Failed to open the video"<<std::endl;
        return -1;
    }

    double video_size = 2.1;
    params.camParam.cameraMatrix = params.camParam.cameraMatrix/2.1;
    params.camParam.cameraMatrix.at<double>(2,2) = 1.0;


    while(1) {
        cv::Mat frame;
        cap >> frame;
        if(frame.empty()) {
            break;
        }


        
        cv::imshow("Fucking", frame);
        cv::waitKey(1);
    }

    return 0;
}