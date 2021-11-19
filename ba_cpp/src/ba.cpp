#include <iostream>
#include "params.h"
#include "opencv2/opencv.hpp"
#include "image_processing.h"

int main(int argc, char **argv)
{
    std::cout << "Bundle Adjustment" << std::endl;
    std::string json_dir = "/home/dongha/dev/bundle_adjustment_test/ba_cpp/camera_calibration_result.json";
    Params params;
    params.read_data(json_dir);

    float resize_factor = 2.0;

    ImageProcessing img_proc(&params);
    img_proc.setupORB(cv::NORM_HAMMING);
    // img_proc.setupSURF(cv::DescriptorMatcher::FLANNBASED);

    cv::VideoCapture cap("../../dataset/Husky.mp4");
    if (!cap.isOpened())
    {
        std::cout << "Failed to open the video" << std::endl;
        return -1;
    }

    cv::Mat last_frame;
    std::pair<std::vector<cv::KeyPoint>, cv::Mat> FeatDescPairLast;

    int seq = 0;
    int skipnum = 5;

    while (1)
    {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty())
        {
            break;
        }

        if(seq%skipnum!=0) {
            ++seq;
            continue;
        }
        ++seq;

        cv::resize(frame, frame, cv::Size(frame.size().width / resize_factor, frame.size().height / resize_factor));
        frame = img_proc.UndistortImage(frame);

        std::pair<std::vector<cv::KeyPoint>, cv::Mat> FeatDescPairCurr = img_proc.extractFeaturesAndDescriptors(frame);

        if (last_frame.empty())
        {

            last_frame = frame;
            FeatDescPairLast = FeatDescPairCurr;

            continue;
        }

        // Perform 2 View BA
        // Match between consecutive frames (target, source)
        std::vector<cv::DMatch> matched_features = img_proc.matchFeatures(FeatDescPairCurr.second, FeatDescPairLast.second);

        // Setup Points;
        std::vector<cv::Point2f> MatchedFeatureCurr(matched_features.size());
        std::vector<cv::Point2f> MatchedFeatureLast(matched_features.size());

        for (int i = 0; i < matched_features.size(); ++i)
        {
            MatchedFeatureCurr[i] = FeatDescPairCurr.first[matched_features[i].queryIdx].pt;
            MatchedFeatureLast[i] = FeatDescPairLast.first[matched_features[i].trainIdx].pt;
        }

        // Estimate Essential Matrix
        cv::Mat E;    // Essential Matrix: 3 by 3 essential matrix
        cv::Mat mask; // Inlier points: N by 1 matrix with 1 as inliers and 0 as outliers

        std::cout << "Find Essential Matrix" << std::endl;

        E = cv::findEssentialMat(MatchedFeatureLast, MatchedFeatureCurr, img_proc.cameraMatrix, cv::RANSAC, 0.999, 1.0, mask);
        // std::cout << E << std::endl;


        cv::Mat nonzeroEssentialIdx;
        cv::findNonZero(mask, nonzeroEssentialIdx);

        int essential_inlier_num = nonzeroEssentialIdx.rows;

        std::vector<cv::Point2f> EssentialInlierPointCurr(nonzeroEssentialIdx.rows);
        std::vector<cv::Point2f> EssentialInlierPointLast(nonzeroEssentialIdx.rows);

        for(int i = 0; i < nonzeroEssentialIdx.rows; ++i) {
            EssentialInlierPointCurr[i] = MatchedFeatureCurr[nonzeroEssentialIdx.at<int>(i,1)];
            EssentialInlierPointLast[i] = MatchedFeatureLast[nonzeroEssentialIdx.at<int>(i,1)];
        }

        cv::Mat Combined;
        cv::hconcat(frame, last_frame, Combined);
        for(int i = 0; i < nonzeroEssentialIdx.rows; ++i) {
            cv::line(Combined, 
                     EssentialInlierPointCurr[i], 
                     cv::Point2f(EssentialInlierPointLast[i].x + frame.cols, EssentialInlierPointLast[i].y), 
                     cv::Scalar(0, 255, 0), 1);
        }

        if (!E.empty())
        {
            // Estimate Rotation & Translation
            std::cout << "Recover Pose form Essential Matrix" << std::endl;
            cv::Mat R; // Rotation Matrix
            cv::Mat t; // Translation Vector
            int pose_inlier_num = cv::recoverPose(E, MatchedFeatureLast, MatchedFeatureCurr, img_proc.cameraMatrix, R, t, mask);
            // std::cout << R << std::endl;
            // std::cout << t << std::endl;
            double inlier_ratio = double(pose_inlier_num)/double(essential_inlier_num);
            std::cout<<"Inlier Ratio: "<<inlier_ratio<<std::endl;

            // Inlier Points
            cv::Mat nonzeroIdx;
            cv::findNonZero(mask, nonzeroIdx);

            std::vector<cv::Point2f> InlierFeatureCurr(pose_inlier_num);
            std::vector<cv::Point2f> InlierFeatureLast(pose_inlier_num);

            for(int i=0; i<pose_inlier_num; ++i) {
                InlierFeatureCurr[i] = MatchedFeatureCurr[nonzeroIdx.at<int>(i,1)];
                InlierFeatureLast[i] = MatchedFeatureLast[nonzeroIdx.at<int>(i,1)];
            }

            cv::Mat RtLast = cv::Mat::eye(3, 4, CV_64FC1);
            cv::Mat RtCurr = cv::Mat::eye(3, 4, CV_64FC1);
            R.copyTo(RtCurr.rowRange(0, 3).colRange(0, 3));
            t.copyTo(RtCurr.rowRange(0, 3).col(3));

            cv::Mat relative_R = R.t();
            cv::Mat relative_t = -R.t()*t;

            cv::Mat relative_Rt = cv::Mat::eye(3, 4, CV_64FC1);
            relative_R.copyTo(relative_Rt.rowRange(0, 3).colRange(0, 3));
            relative_t.copyTo(relative_Rt.rowRange(0, 3).col(3));
            std::cout<<relative_Rt<<std::endl;

            if(inlier_ratio > 0.7) {
                // Triangulate Points
                cv::Mat TriangulatedPoints;

                cv::triangulatePoints(img_proc.cameraMatrix * RtLast, img_proc.cameraMatrix * RtCurr, InlierFeatureLast, InlierFeatureCurr, TriangulatedPoints);

                // cv::Mat dst;
                // cv::drawMatches(frame, FeatDescPairCurr.first, last_frame, FeatDescPairLast.first, matched_features, dst);

                for(int i = 0; i < pose_inlier_num; ++i){
                    cv::line(Combined, 
                            InlierFeatureCurr[i], 
                            cv::Point2f(InlierFeatureLast[i].x + frame.cols, InlierFeatureLast[i].y), 
                            cv::Scalar(0, 0, 255), 1);
                }

                // cv::imshow("Current Frame", dst);
                cv::imshow("Essential Inliers & Cheirality Inliers", Combined);
                cv::waitKey(0);
            }

        }

        last_frame = frame;
        FeatDescPairLast = FeatDescPairCurr;
    }

    return 0;
}