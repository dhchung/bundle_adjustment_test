cmake_minimum_required(VERSION 3.0.0)
project(calibration VERSION 0.1.0)

find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
jsoncpp)