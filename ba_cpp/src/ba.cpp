#include <iostream>
#include "params.h"
#include "opencv2/opencv.hpp"
#include "image_processing.h"
#include "opengl_rendering.h"
#include "slam.h"
#include "utils.h"
#include <fstream>


OpenglRendering ogl_rendering("PointCloud Viewer");

int main(int argc, char **argv)
{

    //Write Stuffs
    std::string pose_writer_path = "../pose.txt";
    std::ofstream write_pose(pose_writer_path.c_str());



    SLAM slam;
    // ogl_rendering.init_opengl();
    std::cout << "Bundle Adjustment" << std::endl;
    std::string json_dir = "../camera_calibration_result.json";
    Params params;
    params.read_data(json_dir);

    float resize_factor = 2.0;

    ImageProcessing img_proc(&params);
    img_proc.setupORB(cv::NORM_HAMMING);
    // img_proc.setupSURF(cv::DescriptorMatcher::FLANNBASED);

    cv::VideoCapture cap("../../dataset/CirclingAround.mp4");
    if (!cap.isOpened())
    {
        std::cout << "Failed to open the video" << std::endl;
        return -1;
    }

    cv::Mat last_frame;
    std::pair<std::vector<cv::KeyPoint>, cv::Mat> FeatDescPairLast;
    int last_frame_no = 0;

    int ptcld_no = 0;

    int seq = 0;
    int skipnum = 1;

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
            last_frame_no = seq;

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

            if(inlier_ratio > 0.6) {
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
                cv::waitKey(1);

                std::string ptcld_writer_path = "../points/point"+std::to_string(ptcld_no)+".txt";
                std::ofstream write_cloud(ptcld_writer_path.c_str());
                std::vector<Point3D> point3d(TriangulatedPoints.cols);


                for(int point_id = 0; point_id < point3d.size(); ++point_id) {
                    point3d[point_id].x = TriangulatedPoints.at<float>(0, point_id)/TriangulatedPoints.at<float>(3, point_id);
                    point3d[point_id].y = TriangulatedPoints.at<float>(1, point_id)/TriangulatedPoints.at<float>(3, point_id);
                    point3d[point_id].z = TriangulatedPoints.at<float>(2, point_id)/TriangulatedPoints.at<float>(3, point_id);

                    cv::Vec3b color = frame.at<cv::Vec3b>(cv::Point(int(InlierFeatureCurr[point_id].x), int(InlierFeatureCurr[point_id].y)));

                    std::cout<<TriangulatedPoints.col(point_id)<<std::endl;
                    std::cout<<point3d[point_id].x<<"\n"<<point3d[point_id].y<<"\n"<<point3d[point_id].z<<std::endl;


                    point3d[point_id].r = color[2];
                    point3d[point_id].g = color[1];
                    point3d[point_id].b = color[0];

                    write_cloud << std::to_string(point3d[point_id].x) << "\t"
                                << std::to_string(point3d[point_id].y) << "\t"
                                << std::to_string(point3d[point_id].z) << "\t"
                                << std::to_string(point3d[point_id].r) << "\t"
                                << std::to_string(point3d[point_id].g) << "\t"
                                << std::to_string(point3d[point_id].b) << "\n";
                }
                write_cloud.close();
                std::cout<<TriangulatedPoints.colRange(0, 10)<<std::endl;

                // ogl_rendering.draw_points(point3d);

                for(int i = 0; i <4; ++i) {
                    for(int j = 0; j < 3; ++j) {
                        if(i==3 && j==2) {
                            write_pose << std::to_string(relative_Rt.at<double>(j,i)) << "\n"; 
                        } else {
                            write_pose << std::to_string(relative_Rt.at<double>(j,i)) << "\t"; 
                        }
                    }
                }


                ++ptcld_no;

            }
            else {
                continue;
            }
        }

        last_frame = frame;
        FeatDescPairLast = FeatDescPairCurr;
        last_frame_no = seq;

    }
    ogl_rendering.terminate();
    write_pose.close();

    return 0;
}