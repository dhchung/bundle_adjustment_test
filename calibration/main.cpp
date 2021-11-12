#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <string>


int CHECKERBOARD[2]{ 8, 12 };

int main(int, char**) {
    std::cout <<"Calibration Code"<<std::endl;
    std::string data_dir = "calib_dataset";
    int img_num = 24;

    std::vector<std::vector<cv::Point3d>> board3DPs;
    std::vector<cv::Point3d> board3DP(CHECKERBOARD[0]*CHECKERBOARD[1]);
    for(int i = 0; i < CHECKERBOARD[0]; ++i) {
        for(int j = 0; j < CHECKERBOARD[1]; ++j) {
            board3DP[CHECKERBOARD[1]*i+j] = cv::Point3d(i*20, j*20, 0.0);
        }
    } // marker size : 20mm

    std::vector<std::vector<cv::Point2f>> corner_pts_container;


    double resize_value = 4.0f; // If too big, it takes too much time
    int success_images = 0;

    for(int i = 0; i < img_num; ++i) {

        std::string img_name = std::to_string(i)+".jpg";
        if(i < 10) {
            img_name = "0" + img_name;
        }
        
        cv::Mat img = cv::imread(data_dir + "/img" + img_name, cv::IMREAD_COLOR);
        cv::resize(img, img, cv::Size(img.size().width/resize_value, img.size().height/resize_value));

        cv::Mat img_gray;
        cv::cvtColor(img, img_gray, cv::COLOR_RGB2GRAY);

        std::vector<cv::Point2f> corner_pts;
        bool success = cv::findChessboardCorners(img_gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        if(success) {

			cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            cv::cornerSubPix(img_gray, corner_pts, cv::Size(11, 11), cv::Size(-1, 1), criteria);

            cv::drawChessboardCorners(img, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);		// Chessboard Corner 그리기
            std::cout<<"SUCCESS"<<std::endl;
            cv::imshow("Chessboard Coner detection result", img);
            cv::waitKey(1);

            for(cv::Point2f point:corner_pts) {
                point*=resize_value; //resize point to original image size
            }

            corner_pts_container.push_back(corner_pts);
            ++success_images;

        } else {
            std::cout<<"Chessboard Coner detection failed"<<std::endl;
        }
    }

    board3DPs.resize(success_images, board3DP);

}
