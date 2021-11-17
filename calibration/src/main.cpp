#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <jsoncpp/json/json.h>
#include <fstream>


int CHECKERBOARD[2]{8, 8};

double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>> &objectPoints,
                                 const std::vector<std::vector<cv::Point2f>> &imagePoints,
                                 const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
                                 const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                 std::vector<float> &perViewErrors)
{
    std::vector<cv::Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int)objectPoints.size(); ++i)
    {
        projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix, // project
                      distCoeffs, imagePoints2);
        err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2); // difference

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err * err / n); // save for this view
        totalErr += err * err;                              // sum it up
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints); // calculate the arithmetical mean
}


int main(int, char **)
{
    std::cout << "Calibration Code" << std::endl;
    std::string data_dir = "../calib_dataset";
    int img_num = 90;

    std::vector<std::vector<cv::Point3f>> board3DPs;
    std::vector<cv::Point3f> board3DP(CHECKERBOARD[0] * CHECKERBOARD[1]);
    for (int i = 0; i < CHECKERBOARD[0]; ++i)
    {
        for (int j = 0; j < CHECKERBOARD[1]; ++j)
        {
            board3DP[CHECKERBOARD[1] * i + j] = cv::Point3f(i * 30.0, j * 30.0, 0.0);
        }
    } // marker size : 20mm

    std::vector<std::vector<cv::Point2f>> corner_pts_container;

    double resize_value = 4.0f; // If too big, it takes too much time

    int img_width = 0;
    int img_height = 0;

    for (int i = 0; i < img_num; ++i)
    {

        std::string img_name = std::to_string(i) + ".jpg";
        if (i < 10)
        {
            img_name = "0" + img_name;
        }

        cv::Mat img = cv::imread(data_dir + "/" + img_name, cv::IMREAD_COLOR);
        if (i == 0)
        {
            img_width = img.size().width;
            img_height = img.size().height;
        }

        cv::resize(img, img, cv::Size(img.size().width / resize_value, img.size().height / resize_value));

        cv::Mat img_gray;
        cv::cvtColor(img, img_gray, cv::COLOR_RGB2GRAY);

        std::vector<cv::Point2f> corner_pts;
        bool success = cv::findChessboardCorners(img_gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (success)
        {

            cv::cornerSubPix(img_gray, corner_pts, cv::Size(11, 11), cv::Size(-1, 1),
                             cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));

            cv::drawChessboardCorners(img, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success); // Chessboard Corner 그리기
            std::cout << "SUCCESS" << std::endl;
            cv::imshow("Chessboard Coner detection result", img);
            cv::waitKey(1);

            for (cv::Point2f point : corner_pts)
            {
                point *= resize_value; // resize point to original image size
            }

            corner_pts_container.push_back(corner_pts);
        }
        else
        {
            std::cout << "Chessboard Coner detection failed" << std::endl;
        }
    }
    std::cout<<"Read Complete"<<std::endl;
    board3DPs.resize(corner_pts_container.size(), board3DP);
    cv::Size img_size = cv::Size(img_width, img_height);

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<float> reprojErrs;
    double totalAvgErr = 0;

    double rms = cv::calibrateCamera(board3DPs, corner_pts_container, img_size, 
                                     cameraMatrix, distCoeffs, rvecs, tvecs,
                                     cv::CALIB_FIX_ASPECT_RATIO);
    std::cout << rms << std::endl;

    std::vector<float> perViewErrors;

    std::cout<<"Image Size: "<<std::endl;
    std::cout<<img_size<<std::endl;
    std::cout<<"Camera Matrix: "<<std::endl;
    std::cout<<cameraMatrix<<std::endl;

    Json::Value root;
    Json::Value ImageSize;
    Json::Value CameraMatrix;
    Json::Value DistortionCoeff;
    
    for(int i=0; i<3; ++i) {
        for(int j=0; j<3; ++j) {
            std::string value_num = "c"+std::to_string(i)+std::to_string(j);
            CameraMatrix[value_num] = std::to_string(cameraMatrix.at<double>(i,j));
        }
    }

    for(int i=0; i<8; ++i) {
        std::string value_num = "d"+std::to_string(i);
        DistortionCoeff[value_num] = std::to_string(distCoeffs.at<double>(i,0));
    }

    ImageSize["width"] = std::to_string(img_width);
    ImageSize["height"] = std::to_string(img_height);

    root["ImageSize"] = ImageSize;
    root["CameraMatrix"] = CameraMatrix;
    root["DistortionCoeff"] = DistortionCoeff;

    Json::StyledWriter writer;
    std::string outputConfig = writer.write(root);
    
    std::cout<<outputConfig<<std::endl;

    std::string calib_result_path = "../camera_calibration_result.json";
    std::ofstream writeFile(calib_result_path.data());
    if(writeFile.is_open()) {
        writeFile << outputConfig;
        writeFile.close();
    }
}

