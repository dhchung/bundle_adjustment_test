#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <string>


int main(int, char**) {
    std::cout << "A Fucking Calibration Code"<<std::endl;

    int img_num = 31;

    std::string data_dir = "/mnt/sdb/Galaxy21/calibration";

    for(int i = 1; i < img_num; ++i) {

        std::string image_name = std::to_string(i)+".jpg";
        if(i < 10) {
            image_name = "0" + image_name;
        }
        
        cv::Mat image = cv::imread(data_dir + "/images/" + image_name, cv::IMREAD_COLOR);

        cv::Mat image_gray;
        cv::cvtColor(image, image_gray, cv::COLOR_RGB2GRAY);

        cv::imshow("FUCKKKKKKKKKKKKKKKKKKKKKKKKKK", image);
        cv::waitKey(1);
    }


}
