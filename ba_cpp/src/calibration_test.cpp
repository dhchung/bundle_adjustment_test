#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <jsoncpp/json/json.h>
#include <fstream>
#include "params.h"


int main(int, char **)
{
    std::cout << "Calibration Test" << std::endl;
    std::string data_dir = "../calib_dataset";
    int img_num = 90;

    Params params;
    std::string json_dir = "/home/dongha/dev/bundle_adjustment_test/ba_cpp/camera_calibration_result.json";
    params.read_data(json_dir);

    cv::Mat cameraMatirx = params.camParam.cameraMatrix;
    cv::Mat distCoeff = params.camParam.distCoeffs;

    std::cout<<cameraMatirx<<std::endl;
    std::cout<<distCoeff<<std::endl;

    // cv::initUndistortRectifyMap(cameraMatirx, distCoeff, cv::Mat::eye(3, CV_64F))
    for (int i = 0; i < img_num; ++i)
    {

        std::string img_name = std::to_string(i) + ".jpg";
        if (i < 10)
        {
            img_name = "0" + img_name;
        }
        cv::Mat img = cv::imread(data_dir + "/" + img_name, cv::IMREAD_COLOR);

        cv::Mat img_undistort;
        cv::undistort(img, img_undistort, cameraMatirx, distCoeff);

        // cv::resize(img, img, cv::Size(img.size().width / 4.0, img.size().height / 4.0));

        
        cv::Mat Combined;
        cv::hconcat (img, img_undistort, Combined);

        cv::resize(Combined, Combined, cv::Size(Combined.size()/4));


        cv::imshow("undistorted img", Combined);
        cv::waitKey(0);
    }
}

