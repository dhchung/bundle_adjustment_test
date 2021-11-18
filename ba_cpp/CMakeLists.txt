cmake_minimum_required(VERSION 3.0.0)
project(ba_cpp VERSION 0.1.0)

find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)
find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

find_package(Boost 1.50 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

file(GLOB CLASS_SOURCES
     "tools/*"
     )

include_directories(${PROJECT_SOURCE_DIR}/tools)


add_executable(camera_calibration
src/calibration.cpp)

add_executable(camera_calibration_test
src/calibration_test.cpp
${CLASS_SOURCES})

add_executable(bundle_adjustment
src/ba.cpp
${CLASS_SOURCES}
)

target_link_libraries(camera_calibration_test
${OpenCV_LIBS}
jsoncpp)
               

target_link_libraries(camera_calibration
${OpenCV_LIBS}
jsoncpp)

target_link_libraries(bundle_adjustment
${OpenCV_LIBS}
jsoncpp
gtsam
Eigen3::Eigen)