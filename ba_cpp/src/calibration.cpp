#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <jsoncpp/json/json.h>
#include <fstream>

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

std::string to_string_exact(double x)
{

    if(std::abs(x)< std::numeric_limits<double>::min()) {
        return std::string("0");
    }

    std::ostringstream os;
    os << std::setprecision(std::numeric_limits<double>::max_digits10) << x;
    return os.str();
}

int main(int, char **)
{

    int checker_board_x = 13;
    int checker_board_y = 10;

    std::cout << "Calibration Code" << std::endl;
    std::string data_dir = "../calib_dataset/new";
    int img_num = 54;

    std::vector<std::vector<cv::Point3f>> board3DPs;
    // std::vector<cv::Point3f> board3DP(checker_board_x * checker_board_y);
    std::vector<cv::Point3f> board3DP;

    for (int i = 0; i < checker_board_y; ++i)
    {
        for (int j = 0; j < checker_board_x; ++j)
        {
            // board3DP[checker_board_y * i + j] = cv::Point3f(j * 30.0, i * 30.0, 0.0);
            board3DP.push_back(cv::Point3f(j * 50.0, i * 50.0, 0.0));

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
        bool success = cv::findChessboardCorners(img_gray, cv::Size(checker_board_x, checker_board_y), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (success)
        {

            cv::cornerSubPix(img_gray, corner_pts, cv::Size(11, 11), cv::Size(-1, 1),
                             cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));

            cv::drawChessboardCorners(img, cv::Size(checker_board_x, checker_board_y), corner_pts, success); // Chessboard Corner 그리기
            std::cout << "SUCCESS" << std::endl;
            cv::imshow("Chessboard Coner detection result", img);
            cv::waitKey(1);

            for(int k = 0; k < corner_pts.size(); ++k) {
                corner_pts[k].x *= resize_value;
                corner_pts[k].y *= resize_value;
            }

            corner_pts_container.push_back(corner_pts);
            board3DPs.push_back(board3DP);
        }
        else
        {
            std::cout << "Chessboard Coner detection failed" << std::endl;
        }
    }
    std::cout << "Read Complete" << std::endl;
    // board3DPs.resize(corner_pts_container.size(), board3DP);
    cv::Size img_size = cv::Size(img_width, img_height);

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<float> reprojErrs;
    double totalAvgErr = 0;

    double rms = cv::calibrateCamera(board3DPs, corner_pts_container, img_size,
                                     cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_ASPECT_RATIO);
    std::cout << rms << std::endl;

    std::vector<float> perViewErrors;

    std::cout << "Image Size: " << std::endl;
    std::cout << img_size << std::endl;
    std::cout << "Camera Matrix: " << std::endl;
    std::cout << cameraMatrix << std::endl;

    std::cout << "Distortion Coefficients: " << std::endl;
    std::cout << distCoeffs << std::endl;

    Json::Value root;
    Json::Value ImageSize;
    Json::Value CameraMatrix;
    Json::Value DistortionCoeff;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            std::string value_num = "c" + std::to_string(i) + std::to_string(j);
            CameraMatrix[value_num] = to_string_exact(cameraMatrix.at<double>(i, j));
        }
    }

    for (int i = 0; i < 8; ++i)
    {
        std::string value_num = "d" + std::to_string(i);
        DistortionCoeff[value_num] = to_string_exact(distCoeffs.at<double>(i, 0));
    }

    ImageSize["width"] = std::to_string(img_width);
    ImageSize["height"] = std::to_string(img_height);

    root["ImageSize"] = ImageSize;
    root["CameraMatrix"] = CameraMatrix;
    root["DistortionCoeff"] = DistortionCoeff;

    Json::StyledWriter writer;
    std::string outputConfig = writer.write(root);

    std::cout << outputConfig << std::endl;

    std::string calib_result_path = "../camera_calibration_result.json";
    std::ofstream writeFile(calib_result_path.data());
    if (writeFile.is_open())
    {
        writeFile << outputConfig;
        writeFile.close();
    }

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

        cv::Mat img_undistort;
        cv::undistort(img, img_undistort, cameraMatrix, distCoeffs);
        
        cv::Mat Combined;
        cv::hconcat (img, img_undistort, Combined);

        cv::resize(Combined, Combined, cv::Size(Combined.cols/4, Combined.rows/4));


        cv::imshow("undistorted img", Combined);
        cv::waitKey(1);
    }


}
