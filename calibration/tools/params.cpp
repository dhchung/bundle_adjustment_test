#include "params.h"

Params::Params(){

}

Params::~Params(){

}


void Params::read_data(std::string & json_dir) {
    std::ifstream stream;
    stream.open(json_dir);

    Json::Value root;
    stream >> root;

    Json::Value camera_mat = root["CameraMatrix"];

    for(int i=0; i<3; ++i) {
        for(int j=0; j<3; ++j) {
            std::string cam_string = "c"+std::to_string(i)+std::to_string(j);
            camParam.cameraMatrix.at<double>(i,j) = std::stod(camera_mat[cam_string].asString());
        }
    }

    Json::Value distort_mat = root["DistortionCoeff"];
    for(int i=0; i<8; ++i) {
        std::string dist_string = "d"+std::to_string(i);
        camParam.distCoeffs.at<double>(i,0) = std::stod(distort_mat[dist_string].asString());
    }

    Json::Value image_size = root["ImageSize"];
    camParam.imageSize = cv::Size(std::stoi(image_size["width"].asString()), std::stoi(image_size["height"].asString()));

    std::cout<<camParam.cameraMatrix<<std::endl;
    std::cout<<camParam.distCoeffs<<std::endl;

}

float Params::value2float(Json::ValueIterator & it) {
    return std::stof((*it)["value"].asString());
}
int Params::value2int(Json::ValueIterator & it) {
    return std::stoi((*it)["value"].asString());
}