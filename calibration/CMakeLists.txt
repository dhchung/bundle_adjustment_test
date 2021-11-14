cmake_minimum_required(VERSION 3.0.0)
project(calibration VERSION 0.1.0)

find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)

file(GLOB CLASS_SOURCES
     "tools/*"
     )

include_directories(${PROJECT_SOURCE_DIR}/tools)

add_executable(${PROJECT_NAME} src/main.cpp)
add_executable(bundle_adjustment
               ${CLASS_SOURCES}
               src/ba.cpp)


target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
jsoncpp)

target_link_libraries(bundle_adjustment
${OpenCV_LIBS}
jsoncpp)